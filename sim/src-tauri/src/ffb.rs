//! Force feedback bridge: DirectInput constant force, driven from the webview.
//!
//! The Gamepad API can rumble a pad and nothing else, so a direct-drive wheel
//! has to be driven from native code. This module owns one DirectInput device
//! on a dedicated thread and renders a torque stream on it at 1 kHz:
//!
//!   base torque   slewed from the last value the game sent, so a 60 Hz message
//!                 stream does not turn into 60 Hz steps at the motor
//!   texture       a sine at the amplitude and frequency the game asked for
//!   kick          a one-shot decaying transient, for a cone or a kerb
//!
//! DirectInput objects are not thread safe and want the thread that created
//! them, so the device lives entirely on its own thread and the rest of the
//! app talks to it through a channel. The webview calls `ffb_update` once per
//! frame with the mix; the thread takes the newest message and renders from
//! it until the next one arrives. If the game stops sending -- the tab froze,
//! the physics threw -- the torque decays to zero within 250 ms rather than
//! holding whatever it last was against the driver's hands.
//!
//! Only the Windows build talks to hardware. Elsewhere every command reports
//! "not supported" so the frontend can hide the option.

use serde::Serialize;
use std::sync::Mutex;

#[derive(Clone, Copy, Debug, Default)]
pub struct Frame {
    /// -1..1, fraction of the motor's rated torque, positive = clockwise.
    pub command: f32,
    /// Texture overlay: amplitude as a fraction of rated torque, and Hz.
    pub texture_amp: f32,
    pub texture_hz: f32,
    /// One-shot impact, fraction of rated torque, signed. Zero when none.
    pub kick: f32,
}

#[derive(Clone, Debug, Serialize, Default)]
pub struct Status {
    pub supported: bool,
    pub running: bool,
    pub device: String,
    pub error: String,
}

pub struct Ffb {
    inner: Mutex<Option<Box<dyn Backend>>>,
    status: Mutex<Status>,
}

impl Ffb {
    pub fn new() -> Self {
        Ffb {
            inner: Mutex::new(None),
            status: Mutex::new(Status {
                supported: platform::SUPPORTED,
                ..Default::default()
            }),
        }
    }
}

/// What a backend has to be able to do. One implementation per platform.
trait Backend: Send {
    fn send(&self, frame: Frame);
    fn stop(&self);
}

#[tauri::command]
pub fn ffb_status(state: tauri::State<'_, Ffb>) -> Status {
    state.status.lock().unwrap().clone()
}

/// Open the first force-feedback-capable DirectInput device and start the
/// render thread. Idempotent: calling it while running is a status query.
#[tauri::command]
pub fn ffb_start(window: tauri::Window, state: tauri::State<'_, Ffb>) -> Status {
    let mut inner = state.inner.lock().unwrap();
    let mut status = state.status.lock().unwrap();
    if inner.is_some() {
        return status.clone();
    }
    match platform::start(&window) {
        Ok((backend, name)) => {
            *inner = Some(backend);
            status.running = true;
            status.device = name;
            status.error.clear();
        }
        Err(e) => {
            status.running = false;
            status.error = e;
        }
    }
    status.clone()
}

#[tauri::command]
pub fn ffb_stop(state: tauri::State<'_, Ffb>) -> Status {
    let mut inner = state.inner.lock().unwrap();
    if let Some(b) = inner.take() {
        b.stop();
    }
    let mut status = state.status.lock().unwrap();
    status.running = false;
    status.clone()
}

/// The per-frame mix from the game. Cheap: one channel send.
#[tauri::command]
pub fn ffb_update(
    state: tauri::State<'_, Ffb>,
    command: f32,
    texture_amp: f32,
    texture_hz: f32,
    kick: f32,
) {
    if let Some(b) = state.inner.lock().unwrap().as_ref() {
        b.send(Frame {
            command: command.clamp(-1.0, 1.0),
            texture_amp: texture_amp.clamp(0.0, 1.0),
            texture_hz: texture_hz.clamp(0.0, 200.0),
            kick: kick.clamp(-1.0, 1.0),
        });
    }
}

/// The 1 kHz mixer. Platform-independent: it takes frames in and produces a
/// -1..1 sample stream, and the platform layer only has to write samples.
pub struct Mixer {
    target: Frame,
    base: f32,
    phase: f32,
    kick_level: f32,
    since_frame_s: f32,
}

impl Mixer {
    pub const RATE_HZ: f32 = 1000.0;
    /// How fast the base torque may move: full scale in 8 ms, which passes a
    /// 60 Hz update stream through without visible steps but still lets a
    /// snap oversteer arrive as a snap.
    const SLEW_PER_S: f32 = 125.0;
    /// If no frame arrives for this long, fade the torque out.
    const WATCHDOG_S: f32 = 0.25;

    pub fn new() -> Self {
        Mixer { target: Frame::default(), base: 0.0, phase: 0.0, kick_level: 0.0, since_frame_s: 0.0 }
    }

    pub fn push(&mut self, f: Frame) {
        // A kick is an event, not a level: it fires when it arrives and does
        // not repeat while the game keeps sending the same frame.
        if f.kick != 0.0 {
            self.kick_level = f.kick;
        }
        self.target = Frame { kick: 0.0, ..f };
        self.since_frame_s = 0.0;
    }

    /// One 1 kHz sample, -1..1.
    pub fn sample(&mut self) -> f32 {
        let dt = 1.0 / Self::RATE_HZ;
        self.since_frame_s += dt;
        if self.since_frame_s > Self::WATCHDOG_S {
            self.target = Frame::default();
        }
        let step = Self::SLEW_PER_S * dt;
        self.base += (self.target.command - self.base).clamp(-step, step);

        self.phase = (self.phase + self.target.texture_hz * dt).fract();
        let texture = self.target.texture_amp * (self.phase * std::f32::consts::TAU).sin();

        // A kick decays with a 40 ms time constant: sharp, over before the
        // next cone.
        let kick = self.kick_level;
        self.kick_level *= 1.0 - dt / 0.04;
        if self.kick_level.abs() < 1e-3 {
            self.kick_level = 0.0;
        }

        (self.base + texture + kick).clamp(-1.0, 1.0)
    }
}

// ------------------------------------------------------------------ Windows --

#[cfg(windows)]
mod platform {
    use super::{Backend, Frame, Mixer};
    use std::sync::mpsc::{channel, Sender, TryRecvError};
    use std::time::{Duration, Instant};
    use windows::core::{Interface, GUID};
    use windows::Win32::Devices::HumanInterfaceDevice::*;
    use windows::Win32::Foundation::{BOOL, HINSTANCE, HWND};
    use windows::Win32::System::LibraryLoader::GetModuleHandleW;

    /// DirectInput's "forever" duration (winbase INFINITE).
    const INFINITE: u32 = 0xFFFF_FFFF;

    pub const SUPPORTED: bool = true;

    enum Msg {
        Frame(Frame),
        Stop,
    }

    struct Di {
        tx: Sender<Msg>,
    }
    impl Backend for Di {
        fn send(&self, frame: Frame) {
            let _ = self.tx.send(Msg::Frame(frame));
        }
        fn stop(&self) {
            let _ = self.tx.send(Msg::Stop);
        }
    }

    /// DirectInput's raw magnitude scale for a constant force.
    const DI_FFNOMINALMAX_F: f32 = 10_000.0;

    pub fn start(window: &tauri::Window) -> Result<(Box<dyn Backend>, String), String> {
        // HWND is a raw pointer and not Send; carry it across as an integer.
        let hwnd_raw = window.hwnd().map_err(|e| e.to_string())?.0 as isize;
        let (tx, rx) = channel::<Msg>();
        let (ready_tx, ready_rx) = channel::<Result<String, String>>();

        std::thread::Builder::new()
            .name("ffb-directinput".into())
            .spawn(move || {
                let dev = match open_device(HWND(hwnd_raw as _)) {
                    Ok(d) => d,
                    Err(e) => {
                        let _ = ready_tx.send(Err(e));
                        return;
                    }
                };
                let _ = ready_tx.send(Ok(dev.name.clone()));
                render_loop(dev, rx);
            })
            .map_err(|e| e.to_string())?;

        match ready_rx.recv() {
            Ok(Ok(name)) => Ok((Box::new(Di { tx }), name)),
            Ok(Err(e)) => Err(e),
            Err(_) => Err("force feedback thread died before reporting".into()),
        }
    }

    struct Device {
        name: String,
        _device: IDirectInputDevice8W,
        effect: IDirectInputEffect,
    }

    fn hr(r: windows::core::Result<()>, what: &str) -> Result<(), String> {
        r.map_err(|e| format!("{what}: {e}"))
    }

    fn open_device(hwnd: HWND) -> Result<Device, String> {
        unsafe {
            let hinst = GetModuleHandleW(None).map_err(|e| e.to_string())?;
            let mut di: Option<IDirectInput8W> = None;
            hr(
                DirectInput8Create(
                    HINSTANCE(hinst.0),
                    DIRECTINPUT_VERSION,
                    &IDirectInput8W::IID,
                    &mut di as *mut _ as *mut _,
                    None,
                ),
                "DirectInput8Create",
            )?;
            let di = di.ok_or("DirectInput8Create returned nothing")?;

            // Enumerate every attached game controller with a force feedback
            // actuator and take the first. A Moza base is the only such device
            // on a sim rig; if a driver has two, this is where a picker goes.
            let mut found: Vec<(GUID, String)> = Vec::new();
            unsafe extern "system" fn on_device(inst: *mut DIDEVICEINSTANCEW, ctx: *mut core::ffi::c_void) -> BOOL {
                let list = &mut *(ctx as *mut Vec<(GUID, String)>);
                let inst = &*inst;
                let end = inst.tszProductName.iter().position(|&c| c == 0).unwrap_or(inst.tszProductName.len());
                let name = String::from_utf16_lossy(&inst.tszProductName[..end]);
                list.push((inst.guidInstance, name));
                BOOL(DIENUM_CONTINUE as i32)
            }
            hr(
                di.EnumDevices(
                    DI8DEVCLASS_GAMECTRL,
                    Some(on_device),
                    &mut found as *mut _ as *mut _,
                    DIEDFL_ATTACHEDONLY | DIEDFL_FORCEFEEDBACK,
                ),
                "EnumDevices",
            )?;
            let (guid, name) = found.into_iter().next().ok_or_else(|| {
                "no force-feedback device found. Is the wheel base on, and is Pit House not holding it?".to_string()
            })?;

            let mut device: Option<IDirectInputDevice8W> = None;
            hr(di.CreateDevice(&guid, &mut device, None), "CreateDevice")?;
            let device = device.ok_or("CreateDevice returned nothing")?;

            // The windows crate does not export the c_dfDIJoystick data format,
            // so declare the smallest one that will do: a single absolute X
            // axis at the front of a DIJOYSTATE. We never read the state --
            // steering position comes through the Gamepad API -- but a data
            // format has to be set before the device can be acquired.
            let mut fmt_obj = [DIOBJECTDATAFORMAT {
                pguid: &GUID_XAxis,
                dwOfs: 0,
                dwType: DIDFT_AXIS | DIDFT_ANYINSTANCE,
                dwFlags: DIDOI_ASPECTPOSITION,
            }];
            let mut fmt = DIDATAFORMAT {
                dwSize: std::mem::size_of::<DIDATAFORMAT>() as u32,
                dwObjSize: std::mem::size_of::<DIOBJECTDATAFORMAT>() as u32,
                dwFlags: DIDF_ABSAXIS,
                dwDataSize: std::mem::size_of::<DIJOYSTATE>() as u32,
                dwNumObjs: 1,
                rgodf: fmt_obj.as_mut_ptr(),
            };
            hr(device.SetDataFormat(&mut fmt), "SetDataFormat")?;
            // Exclusive is required to play effects. Background so the wheel
            // keeps working while the driver alt-tabs to a settings window.
            hr(
                device.SetCooperativeLevel(hwnd, (DISCL_EXCLUSIVE | DISCL_BACKGROUND) as u32),
                "SetCooperativeLevel (is another program holding the wheel?)",
            )?;

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

            hr(device.Acquire(), "Acquire")?;

            // One constant-force effect on the X axis, infinite duration; the
            // render loop updates its magnitude in place.
            // Offset of lX in DIJOYSTATE, which is what DIJOFS_X expands to.
            let mut axes = [0u32];
            let mut dirs = [0i32];
            let mut cf = DICONSTANTFORCE { lMagnitude: 0 };
            let mut eff = DIEFFECT {
                dwSize: std::mem::size_of::<DIEFFECT>() as u32,
                dwFlags: DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS,
                dwDuration: INFINITE,
                dwSamplePeriod: 0,
                dwGain: DI_FFNOMINALMAX_F as u32,
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
            hr(device.CreateEffect(&GUID_ConstantForce, &mut eff, &mut effect, None), "CreateEffect")?;
            let effect = effect.ok_or("CreateEffect returned nothing")?;
            hr(effect.Start(INFINITE, 0), "Effect Start")?;

            Ok(Device { name, _device: device, effect })
        }
    }

    fn set_magnitude(effect: &IDirectInputEffect, sample: f32) -> Result<(), String> {
        unsafe {
            let mut cf = DICONSTANTFORCE { lMagnitude: (sample * DI_FFNOMINALMAX_F) as i32 };
            let mut eff = DIEFFECT {
                dwSize: std::mem::size_of::<DIEFFECT>() as u32,
                cbTypeSpecificParams: std::mem::size_of::<DICONSTANTFORCE>() as u32,
                lpvTypeSpecificParams: &mut cf as *mut _ as *mut _,
                ..Default::default()
            };
            hr(
                effect.SetParameters(&mut eff, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART),
                "SetParameters",
            )?;
            Ok(())
        }
    }

    fn render_loop(dev: Device, rx: std::sync::mpsc::Receiver<Msg>) {
        let mut mixer = Mixer::new();
        let period = Duration::from_secs_f32(1.0 / Mixer::RATE_HZ);
        let mut next = Instant::now();
        let mut errors = 0u32;
        loop {
            // Drain to the newest frame; stale ones are worthless.
            loop {
                match rx.try_recv() {
                    Ok(Msg::Frame(f)) => mixer.push(f),
                    Ok(Msg::Stop) | Err(TryRecvError::Disconnected) => {
                        let _ = set_magnitude(&dev.effect, 0.0);
                        unsafe {
                            let _ = dev.effect.Stop();
                            let _ = dev._device.Unacquire();
                        }
                        return;
                    }
                    Err(TryRecvError::Empty) => break,
                }
            }
            let s = mixer.sample();
            if set_magnitude(&dev.effect, s).is_err() {
                // A lost device (USB unplugged, Pit House grabbed it) throws
                // every call. Try to reacquire; give up quietly if it stays gone.
                errors += 1;
                unsafe {
                    let _ = dev._device.Acquire();
                }
                if errors > 2000 {
                    return;
                }
            } else {
                errors = 0;
            }
            next += period;
            let now = Instant::now();
            if next > now {
                std::thread::sleep(next - now);
            } else {
                next = now;
            }
        }
    }
}

// ----------------------------------------------------------------- Not Windows

#[cfg(not(windows))]
mod platform {
    use super::Backend;
    pub const SUPPORTED: bool = false;

    pub fn start(_window: &tauri::Window) -> Result<(Box<dyn Backend>, String), String> {
        Err("force feedback needs the Windows build (DirectInput)".into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slews_rather_than_steps() {
        let mut m = Mixer::new();
        m.push(Frame { command: 1.0, ..Default::default() });
        let first = m.sample();
        assert!(first > 0.0 && first < 0.5, "first sample {first} should be a fraction of the step");
        for _ in 0..20 {
            m.sample();
        }
        assert!((m.sample() - 1.0).abs() < 1e-3);
    }

    #[test]
    fn watchdog_fades_out() {
        let mut m = Mixer::new();
        m.push(Frame { command: 0.8, ..Default::default() });
        for _ in 0..400 {
            m.sample();
        }
        assert!(m.sample().abs() < 1e-3, "torque must decay when frames stop");
    }

    #[test]
    fn kick_is_an_event() {
        let mut m = Mixer::new();
        m.push(Frame { kick: 0.5, ..Default::default() });
        let a = m.sample();
        assert!(a > 0.4);
        for _ in 0..300 {
            m.sample();
        }
        m.push(Frame { kick: 0.0, ..Default::default() });
        assert!(m.sample().abs() < 1e-2, "a kick must not repeat");
    }
}
