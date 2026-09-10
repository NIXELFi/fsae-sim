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

/// How many game controllers are read at once: the wheel base plus up to
/// three more -- a separate pedal set, a shifter, a button box.
pub const MAX_DEVICES: usize = 4;
/// Axes per device, in DirectInput order: X Y Z Rx Ry Rz Slider0 Slider1.
pub const AXES_PER_DEVICE: usize = 8;

/// One read of every device. Axis `8*d + i` is axis `i` of device `d`;
/// device 0 is always the wheel base.
#[derive(Clone, Copy, Debug)]
pub struct DeviceState {
    pub axes: [f32; MAX_DEVICES * AXES_PER_DEVICE],
    /// Per device, button i is bit i.
    pub buttons: [u32; MAX_DEVICES],
    /// The base's hat switch, centidegrees clockwise from up, or -1 centred.
    pub pov: i32,
}

impl Default for DeviceState {
    fn default() -> Self {
        Self { axes: [0.0; MAX_DEVICES * AXES_PER_DEVICE], buttons: [0; MAX_DEVICES], pov: -1 }
    }
}

/// A game controller the rig can see, for the picker.
#[derive(Clone, Debug, Default, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DeviceInfo {
    pub name: String,
    pub force_feedback: bool,
}

/// Does this product name look like a steering wheel base?
pub fn looks_like_wheel(name: &str) -> bool {
    let l = name.to_lowercase();
    ["wheel", "base", "moza", "simucube", "fanatec", "logitech g", "g29", "g920", "g923", "g27",
     "thrustmaster", "t300", "t150", "t248", "tx ", "t-gt", "simagic", "cammus", "asetek", "vrs",
     "podium", "csl", "clubsport", "driving force"]
        .iter()
        .any(|k| l.contains(k))
}

/// Does this product name look like a pedal set, shifter or button box --
/// something to read but never to drive?
pub fn looks_like_peripheral(name: &str) -> bool {
    let l = name.to_lowercase();
    ["pedal", "shifter", "handbrake", "button", "srp", "csp", "heusinkveld", "sim-lab"].iter().any(|k| l.contains(k))
}

#[cfg(windows)]
pub use win::{enumerate, Wheel};

#[cfg(not(windows))]
pub use stub::Wheel;
#[cfg(not(windows))]
pub use stub_enum::enumerate;

// ----------------------------------------------------------------- Windows --

#[cfg(windows)]
mod win {
    use super::{looks_like_peripheral, looks_like_wheel, DeviceInfo, DeviceState};
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
        /// The base: the device that steers and, if it can, is driven.
        pub name: String,
        /// Everything read, base first.
        pub names: Vec<String>,
        /// The base has a force feedback actuator DirectInput can drive.
        pub ffb: bool,
        device: IDirectInputDevice8W,
        extras: Vec<IDirectInputDevice8W>,
        effect: Option<IDirectInputEffect>,
        last_magnitude: i32,
        lost: u32,
    }

    struct Found {
        guid: GUID,
        name: String,
        ffb: bool,
    }

    fn hr(r: windows::core::Result<()>, what: &str) -> Result<(), String> {
        r.map_err(|e| format!("{what}: {e}"))
    }

    unsafe extern "system" fn on_device(inst: *mut DIDEVICEINSTANCEW, ctx: *mut core::ffi::c_void) -> BOOL {
        let list = &mut *(ctx as *mut Vec<Found>);
        let inst = &*inst;
        let end = inst.tszProductName.iter().position(|&c| c == 0).unwrap_or(inst.tszProductName.len());
        list.push(Found { guid: inst.guidInstance, name: String::from_utf16_lossy(&inst.tszProductName[..end]), ffb: false });
        BOOL(DIENUM_CONTINUE as i32)
    }

    fn create_di() -> Result<IDirectInput8W, String> {
        unsafe {
            let hinst = GetModuleHandleW(None).map_err(|e| e.to_string())?;
            let mut di: Option<IDirectInput8W> = None;
            hr(
                DirectInput8Create(HINSTANCE(hinst.0), DIRECTINPUT_VERSION, &IDirectInput8W::IID, &mut di as *mut _ as *mut _, None),
                "DirectInput8Create",
            )?;
            di.ok_or_else(|| "DirectInput8Create returned nothing".to_string())
        }
    }

    /// Every attached game controller, with whether it has a force feedback
    /// actuator. Same order every call, so an index is a stable choice.
    fn enumerate_with(di: &IDirectInput8W) -> Result<Vec<Found>, String> {
        unsafe {
            let mut all: Vec<Found> = Vec::new();
            hr(di.EnumDevices(DI8DEVCLASS_GAMECTRL, Some(on_device), &mut all as *mut _ as *mut _, DIEDFL_ATTACHEDONLY), "EnumDevices")?;
            let mut ffb: Vec<Found> = Vec::new();
            hr(di.EnumDevices(DI8DEVCLASS_GAMECTRL, Some(on_device), &mut ffb as *mut _ as *mut _, DIEDFL_ATTACHEDONLY | DIEDFL_FORCEFEEDBACK), "EnumDevices")?;
            for d in all.iter_mut() {
                d.ffb = ffb.iter().any(|f| f.guid == d.guid);
            }
            Ok(all)
        }
    }

    /// What is plugged in, for the settings panel's picker.
    pub fn enumerate() -> Vec<DeviceInfo> {
        match create_di().and_then(|di| enumerate_with(&di)) {
            Ok(v) => v.into_iter().map(|f| DeviceInfo { name: f.name, force_feedback: f.ffb }).collect(),
            Err(_) => Vec::new(),
        }
    }

    /// Which device to steer with. A name the driver picked wins; otherwise
    /// the first thing that both has an actuator and looks like a wheel,
    /// then anything with an actuator that is not obviously a pedal set,
    /// then a wheel-looking device with no actuator (a base in a console
    /// compatibility mode, or a wheel DirectInput cannot drive).
    fn choose(found: &[Found], prefer: Option<&str>) -> Option<usize> {
        if let Some(p) = prefer {
            if let Some(i) = found.iter().position(|f| f.name == p) {
                return Some(i);
            }
        }
        let score = |f: &Found| -> i32 {
            let wheel = looks_like_wheel(&f.name);
            let periph = looks_like_peripheral(&f.name);
            match (f.ffb, wheel, periph) {
                (true, true, _) => 0,
                (true, false, false) => 1,
                (false, true, _) => 2,
                (true, _, true) => 3,
                _ => 9,
            }
        };
        found.iter().enumerate().filter(|(_, f)| score(f) < 9).min_by_key(|(_, f)| score(f)).map(|(i, _)| i)
    }

    impl Wheel {
        /// Open the wheel base (and everything else plugged in, for pedals
        /// and shifters). `prefer` is a product name the driver chose.
        pub fn open(hwnd_raw: isize, prefer: Option<&str>) -> Result<Wheel, String> {
            unsafe {
                let di = create_di()?;
                let found = enumerate_with(&di)?;
                let idx = choose(&found, prefer).ok_or_else(|| {
                    if found.is_empty() {
                        "no game controller found. Is the wheel base on and in PC mode?".to_string()
                    } else {
                        format!(
                            "no steering wheel among: {}. Pick one in the controls panel.",
                            found.iter().map(|f| f.name.as_str()).collect::<Vec<_>>().join(", ")
                        )
                    }
                })?;
                let base = &found[idx];

                let device = Self::open_one(&di, base.guid, hwnd_raw, base.ffb)?;
                let mut names = vec![base.name.clone()];
                let mut extras = Vec::new();
                for (i, f) in found.iter().enumerate() {
                    if i == idx || extras.len() + 1 >= super::MAX_DEVICES {
                        continue;
                    }
                    // Pedals, shifters and button boxes: read only, shared.
                    if let Ok(d) = Self::open_one(&di, f.guid, hwnd_raw, false) {
                        extras.push(d);
                        names.push(f.name.clone());
                    }
                }

                let mut w = Wheel {
                    name: base.name.clone(),
                    names,
                    ffb: base.ffb,
                    device,
                    extras,
                    effect: None,
                    last_magnitude: i32::MIN,
                    lost: 0,
                };
                if w.ffb {
                    if let Err(e) = w.create_effect() {
                        // A device that claims an actuator but refuses the
                        // effect still steers; just say why it is silent.
                        w.ffb = false;
                        w.name = format!("{} (no force feedback: {e})", w.name);
                    }
                }
                Ok(w)
            }
        }

        /// One device with the joystick data format. `exclusive` is needed to
        /// play effects and is only asked for on the base.
        unsafe fn open_one(di: &IDirectInput8W, guid: GUID, hwnd_raw: isize, exclusive: bool) -> Result<IDirectInputDevice8W, String> {
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
            // conventional pairing. Peripherals are shared and background so
            // a pedal set keeps reading whatever else is open.
            let level = if exclusive { DISCL_EXCLUSIVE | DISCL_FOREGROUND } else { DISCL_NONEXCLUSIVE | DISCL_BACKGROUND };
            hr(
                device.SetCooperativeLevel(HWND(hwnd_raw as _), level as u32),
                "SetCooperativeLevel (is another program holding the wheel?)",
            )?;

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

            if exclusive {
                // The base's own centring spring would fight the tyre model,
                // and the device gain should be all the way up so the
                // profile's rated torque means what it says.
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
                let mut gain = DIPROPDWORD { diph: auto.diph, dwData: FF_MAX as u32 };
                let _ = device.SetProperty(&DIPROP_FFGAIN, &mut gain.diph);
            }

            hr(device.Acquire(), if exclusive { "Acquire (bring the game window to the front)" } else { "Acquire" })?;
            Ok(device)
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

        unsafe fn read_one(device: &IDirectInputDevice8W) -> Option<JoyState> {
            let _ = device.Poll();
            let mut js = JoyState::default();
            let r = device.GetDeviceState(std::mem::size_of::<JoyState>() as u32, &mut js as *mut _ as *mut _);
            if r.is_err() {
                let _ = device.Acquire();
                return None;
            }
            Some(js)
        }

        /// Poll and read every device. `None` if the base is currently lost
        /// (unplugged, window in the background); retried each tick. A lost
        /// peripheral just reads as zero.
        pub fn read(&mut self) -> Option<DeviceState> {
            unsafe {
                let Some(js) = Self::read_one(&self.device) else {
                    self.lost += 1;
                    return None;
                };
                self.lost = 0;
                let mut s = DeviceState::default();
                Self::unpack(&js, 0, &mut s);
                s.pov = if js.pov[0] == 0xFFFF_FFFF || (js.pov[0] & 0xFFFF) == 0xFFFF { -1 } else { js.pov[0] as i32 };
                for (k, d) in self.extras.iter().enumerate() {
                    if let Some(js) = Self::read_one(d) {
                        Self::unpack(&js, k + 1, &mut s);
                    }
                }
                Some(s)
            }
        }

        fn unpack(js: &JoyState, slot: usize, s: &mut DeviceState) {
            for i in 0..super::AXES_PER_DEVICE {
                s.axes[slot * super::AXES_PER_DEVICE + i] = (js.axes[i] as f32 / AXIS_RANGE as f32).clamp(-1.0, 1.0);
            }
            let mut bits = 0u32;
            for (i, b) in js.buttons.iter().enumerate() {
                if b & 0x80 != 0 {
                    bits |= 1 << i;
                }
            }
            s.buttons[slot] = bits;
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
                for d in &self.extras {
                    let _ = d.Unacquire();
                }
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
        pub names: Vec<String>,
        pub ffb: bool,
    }

    impl Wheel {
        pub fn open(_hwnd_raw: isize, _prefer: Option<&str>) -> Result<Wheel, String> {
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

#[cfg(not(windows))]
mod stub_enum {
    pub fn enumerate() -> Vec<super::DeviceInfo> {
        Vec::new()
    }
}


#[cfg(windows)]
pub use win::realtime_thread;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recognises_bases_and_peripherals() {
        assert!(looks_like_wheel("MOZA R5 Base"));
        assert!(looks_like_wheel("Logitech G923 Racing Wheel USB"));
        assert!(looks_like_wheel("Simucube 2 Sport"));
        assert!(looks_like_wheel("Thrustmaster T300RS Racing wheel"));
        assert!(!looks_like_wheel("Xbox Controller"));
        assert!(looks_like_peripheral("MOZA SR-P Pedals"));
        assert!(looks_like_peripheral("Heusinkveld Sim Pedals Sprint"));
        assert!(!looks_like_peripheral("Fanatec Podium Wheel Base DD1"));
    }
}
