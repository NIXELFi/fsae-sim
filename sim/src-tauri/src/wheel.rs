//! The steering wheel, natively: DirectInput on Windows, nothing elsewhere.
//!
//! Two jobs, both on the rig thread that owns the device:
//!
//!   read    axes, buttons and hat, every tick. The rim angle read here is
//!           what the vehicle model steers by and what the force feedback
//!           damps against, at 1 kHz with no browser in the loop.
//!   write   one constant-force effect whose magnitude is updated in place.
//!
//! DirectInput objects are not thread safe and want the thread that created
//! them, so this type is `!Send` on purpose and lives entirely inside
//! `rig::run`. The Gamepad API in the webview keeps working for pads; a
//! wheel is read here because a DirectInput exclusive acquire is needed to
//! play effects, and once acquired exclusively nothing else can read it.
//!
//! The data format is c_dfDIJoystick rebuilt by hand -- the `windows` crate
//! does not export the static -- as eight axes, four hats and 32 buttons on
//! the DIJOYSTATE layout, every object optional so a base with no rudder
//! still acquires.

/// One read of the device.
#[derive(Clone, Copy, Debug, Default)]
pub struct DeviceState {
    /// Eight axes, -1..1: X Y Z Rx Ry Rz Slider0 Slider1 in DirectInput order.
    pub axes: [f32; 8],
    /// Button i is bit i.
    pub buttons: u32,
    /// Hat switch, centidegrees clockwise from up, or -1 centred.
    pub pov: i32,
}

#[cfg(windows)]
pub use win::Wheel;

#[cfg(not(windows))]
pub use stub::Wheel;

// ----------------------------------------------------------------- Windows --

#[cfg(windows)]
mod win {
    use super::DeviceState;
    use windows::core::{Interface, GUID};
    use windows::Win32::Devices::HumanInterfaceDevice::*;
    use windows::Win32::Foundation::{BOOL, HINSTANCE, HWND};
    use windows::Win32::System::LibraryLoader::GetModuleHandleW;

    const INFINITE: u32 = 0xFFFF_FFFF;
    /// dinput.h DIDFT_OPTIONAL; not exported by the windows crate.
    const DIDFT_OPTIONAL: u32 = 0x8000_0000;
    /// DirectInput's nominal magnitude scale for a constant force.
    const FF_MAX: f32 = 10_000.0;
    /// Axis range we ask the driver for, so a reading normalises trivially.
    const AXIS_RANGE: i32 = 10_000;

    /// DIJOYSTATE, laid out by hand so its offsets are ours.
    #[repr(C)]
    #[derive(Default, Clone, Copy)]
    struct JoyState {
        axes: [i32; 8],
        pov: [u32; 4],
        buttons: [u8; 32],
    }

    pub struct Wheel {
        pub name: String,
        device: IDirectInputDevice8W,
        effect: Option<IDirectInputEffect>,
        last_magnitude: i32,
        lost: u32,
    }

    fn hr(r: windows::core::Result<()>, what: &str) -> Result<(), String> {
        r.map_err(|e| format!("{what}: {e}"))
    }

    unsafe extern "system" fn on_device(inst: *mut DIDEVICEINSTANCEW, ctx: *mut core::ffi::c_void) -> BOOL {
        let list = &mut *(ctx as *mut Vec<(GUID, String)>);
        let inst = &*inst;
        let end = inst.tszProductName.iter().position(|&c| c == 0).unwrap_or(inst.tszProductName.len());
        list.push((inst.guidInstance, String::from_utf16_lossy(&inst.tszProductName[..end])));
        BOOL(DIENUM_CONTINUE as i32)
    }

    impl Wheel {
        /// Open the first attached game controller with a force-feedback
        /// actuator. `hwnd_raw` is the game window, needed for the
        /// cooperative level.
        pub fn open(hwnd_raw: isize) -> Result<Wheel, String> {
            unsafe {
                let hinst = GetModuleHandleW(None).map_err(|e| e.to_string())?;
                let mut di: Option<IDirectInput8W> = None;
                hr(
                    DirectInput8Create(HINSTANCE(hinst.0), DIRECTINPUT_VERSION, &IDirectInput8W::IID, &mut di as *mut _ as *mut _, None),
                    "DirectInput8Create",
                )?;
                let di = di.ok_or("DirectInput8Create returned nothing")?;

                let mut found: Vec<(GUID, String)> = Vec::new();
                hr(
                    di.EnumDevices(DI8DEVCLASS_GAMECTRL, Some(on_device), &mut found as *mut _ as *mut _, DIEDFL_ATTACHEDONLY | DIEDFL_FORCEFEEDBACK),
                    "EnumDevices",
                )?;
                // Prefer something that calls itself a wheel or a base over an
                // FFB joystick or a pedal set that happens to enumerate first.
                found.sort_by_key(|(_, n)| {
                    let l = n.to_lowercase();
                    if l.contains("wheel") || l.contains("base") || l.contains("moza") || l.contains("simucube") || l.contains("fanatec") { 0 } else { 1 }
                });
                let (guid, name) = found.into_iter().next().ok_or_else(|| {
                    "no force-feedback device found. Is the wheel base on?".to_string()
                })?;

                let mut device: Option<IDirectInputDevice8W> = None;
                hr(di.CreateDevice(&guid, &mut device, None), "CreateDevice")?;
                let device = device.ok_or("CreateDevice returned nothing")?;

                // c_dfDIJoystick, by hand. Offsets follow JoyState above.
                let axis_guids = [&GUID_XAxis, &GUID_YAxis, &GUID_ZAxis, &GUID_RxAxis, &GUID_RyAxis, &GUID_RzAxis, &GUID_Slider, &GUID_Slider];
                let mut objs: Vec<DIOBJECTDATAFORMAT> = Vec::with_capacity(44);
                for (i, g) in axis_guids.iter().enumerate() {
                    objs.push(DIOBJECTDATAFORMAT {
                        pguid: *g,
                        dwOfs: (i * 4) as u32,
                        dwType: DIDFT_AXIS | DIDFT_ANYINSTANCE | DIDFT_OPTIONAL,
                        dwFlags: DIDOI_ASPECTPOSITION,
                    });
                }
                for i in 0..4 {
                    objs.push(DIOBJECTDATAFORMAT {
                        pguid: &GUID_POV,
                        dwOfs: (32 + i * 4) as u32,
                        dwType: DIDFT_POV | DIDFT_ANYINSTANCE | DIDFT_OPTIONAL,
                        dwFlags: 0,
                    });
                }
                for i in 0..32 {
                    objs.push(DIOBJECTDATAFORMAT {
                        pguid: std::ptr::null(),
                        dwOfs: (48 + i) as u32,
                        dwType: DIDFT_BUTTON | DIDFT_ANYINSTANCE | DIDFT_OPTIONAL,
                        dwFlags: 0,
                    });
                }
                let mut fmt = DIDATAFORMAT {
                    dwSize: std::mem::size_of::<DIDATAFORMAT>() as u32,
                    dwObjSize: std::mem::size_of::<DIOBJECTDATAFORMAT>() as u32,
                    dwFlags: DIDF_ABSAXIS,
                    dwDataSize: std::mem::size_of::<JoyState>() as u32,
                    dwNumObjs: objs.len() as u32,
                    rgodf: objs.as_mut_ptr(),
                };
                hr(device.SetDataFormat(&mut fmt), "SetDataFormat")?;

                // Exclusive is required to play effects; foreground is the
                // conventional pairing and what the SDK samples use. The rig
                // fades torque to zero when the window loses focus anyway.
                hr(
                    device.SetCooperativeLevel(HWND(hwnd_raw as _), (DISCL_EXCLUSIVE | DISCL_FOREGROUND) as u32),
                    "SetCooperativeLevel (is another program holding the wheel?)",
                )?;

                // Axis range, all axes at once.
                let mut range = DIPROPRANGE {
                    diph: DIPROPHEADER {
                        dwSize: std::mem::size_of::<DIPROPRANGE>() as u32,
                        dwHeaderSize: std::mem::size_of::<DIPROPHEADER>() as u32,
                        dwObj: 0,
                        dwHow: DIPH_DEVICE,
                    },
                    lMin: -AXIS_RANGE,
                    lMax: AXIS_RANGE,
                };
                let _ = device.SetProperty(&DIPROP_RANGE, &mut range.diph);

                // The base's own centring spring would fight the tyre model.
                let mut auto = DIPROPDWORD {
                    diph: DIPROPHEADER {
                        dwSize: std::mem::size_of::<DIPROPDWORD>() as u32,
                        dwHeaderSize: std::mem::size_of::<DIPROPHEADER>() as u32,
                        dwObj: 0,
                        dwHow: DIPH_DEVICE,
                    },
                    dwData: DIPROPAUTOCENTER_OFF,
                };
                let _ = device.SetProperty(&DIPROP_AUTOCENTER, &mut auto.diph);

                hr(device.Acquire(), "Acquire (bring the game window to the front)")?;

                let mut w = Wheel { name, device, effect: None, last_magnitude: i32::MIN, lost: 0 };
                w.create_effect()?;
                Ok(w)
            }
        }

        fn create_effect(&mut self) -> Result<(), String> {
            unsafe {
                // One constant force on X, forever; the magnitude is what moves.
                let mut axes = [0u32]; // offset of X in JoyState == DIJOFS_X
                let mut dirs = [0i32];
                let mut cf = DICONSTANTFORCE { lMagnitude: 0 };
                let mut eff = DIEFFECT {
                    dwSize: std::mem::size_of::<DIEFFECT>() as u32,
                    dwFlags: DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS,
                    dwDuration: INFINITE,
                    dwSamplePeriod: 0,
                    dwGain: FF_MAX as u32,
                    dwTriggerButton: DIEB_NOTRIGGER,
                    dwTriggerRepeatInterval: 0,
                    cAxes: 1,
                    rgdwAxes: axes.as_mut_ptr(),
                    rglDirection: dirs.as_mut_ptr(),
                    lpEnvelope: std::ptr::null_mut(),
                    cbTypeSpecificParams: std::mem::size_of::<DICONSTANTFORCE>() as u32,
                    lpvTypeSpecificParams: &mut cf as *mut _ as *mut _,
                    dwStartDelay: 0,
                };
                let mut effect: Option<IDirectInputEffect> = None;
                hr(self.device.CreateEffect(&GUID_ConstantForce, &mut eff, &mut effect, None), "CreateEffect")?;
                let effect = effect.ok_or("CreateEffect returned nothing")?;
                hr(effect.Start(INFINITE, 0), "Effect Start")?;
                self.effect = Some(effect);
                Ok(())
            }
        }

        /// Poll and read. `None` if the device is currently lost (unplugged,
        /// window in the background); the read is retried each tick.
        pub fn read(&mut self) -> Option<DeviceState> {
            unsafe {
                let _ = self.device.Poll();
                let mut js = JoyState::default();
                let r = self.device.GetDeviceState(std::mem::size_of::<JoyState>() as u32, &mut js as *mut _ as *mut _);
                if r.is_err() {
                    self.lost += 1;
                    let _ = self.device.Acquire();
                    return None;
                }
                self.lost = 0;
                let mut s = DeviceState::default();
                for i in 0..8 {
                    s.axes[i] = (js.axes[i] as f32 / AXIS_RANGE as f32).clamp(-1.0, 1.0);
                }
                for (i, b) in js.buttons.iter().enumerate() {
                    if b & 0x80 != 0 {
                        s.buttons |= 1 << i;
                    }
                }
                s.pov = if js.pov[0] == 0xFFFF_FFFF || (js.pov[0] & 0xFFFF) == 0xFFFF { -1 } else { js.pov[0] as i32 };
                Some(s)
            }
        }

        /// Set the constant force, -1..1, positive clockwise. Skips the USB
        /// report when the quantised magnitude has not changed.
        pub fn set_torque(&mut self, sample: f32) -> Result<(), String> {
            let mag = (sample.clamp(-1.0, 1.0) * FF_MAX) as i32;
            if mag == self.last_magnitude {
                return Ok(());
            }
            let Some(effect) = self.effect.as_ref() else { return Ok(()) };
            unsafe {
                let mut cf = DICONSTANTFORCE { lMagnitude: mag };
                let mut eff = DIEFFECT {
                    dwSize: std::mem::size_of::<DIEFFECT>() as u32,
                    cbTypeSpecificParams: std::mem::size_of::<DICONSTANTFORCE>() as u32,
                    lpvTypeSpecificParams: &mut cf as *mut _ as *mut _,
                    ..Default::default()
                };
                match effect.SetParameters(&mut eff, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART) {
                    Ok(()) => {
                        self.last_magnitude = mag;
                        Ok(())
                    }
                    Err(e) => {
                        // Lost the device: reacquire and let the next tick retry.
                        let _ = self.device.Acquire();
                        Err(e.to_string())
                    }
                }
            }
        }

        pub fn close(&mut self) {
            unsafe {
                let _ = self.set_torque(0.0);
                if let Some(e) = self.effect.take() {
                    let _ = e.Stop();
                }
                let _ = self.device.Unacquire();
            }
        }
    }

    impl Drop for Wheel {
        fn drop(&mut self) {
            self.close();
        }
    }

    /// Ask the OS for 1 ms timer granularity and a real-time-ish priority for
    /// the calling thread. Without `timeBeginPeriod` a 1 ms sleep on Windows
    /// can be 15.6 ms.
    pub fn realtime_thread() {
        use windows::Win32::Media::timeBeginPeriod;
        use windows::Win32::System::Threading::{GetCurrentThread, SetThreadPriority, THREAD_PRIORITY_TIME_CRITICAL};
        unsafe {
            let _ = timeBeginPeriod(1);
            let _ = SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
        }
    }
}

// ----------------------------------------------------------- everywhere else --

#[cfg(not(windows))]
mod stub {
    use super::DeviceState;

    /// No native wheel off Windows. The rig still runs the physics; steering
    /// comes from the webview.
    pub struct Wheel {
        pub name: String,
    }

    impl Wheel {
        pub fn open(_hwnd_raw: isize) -> Result<Wheel, String> {
            Err("native wheel input and force feedback need the Windows build (DirectInput)".into())
        }
        pub fn read(&mut self) -> Option<DeviceState> {
            None
        }
        pub fn set_torque(&mut self, _sample: f32) -> Result<(), String> {
            Ok(())
        }
        pub fn close(&mut self) {}
    }
}

#[cfg(not(windows))]
pub fn realtime_thread() {}

#[cfg(windows)]
pub use win::realtime_thread;
