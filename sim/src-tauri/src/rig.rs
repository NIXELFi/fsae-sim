//! The rig: vehicle model, steering wheel and force feedback on one native
//! thread at 1 kHz.
//!
//! In the browser build the webview does everything. In the desktop build the
//! physics moves here, for one reason: latency. A direct-drive wheel is a
//! closed loop through the driver's hands, and every stage between the rim
//! moving and the motor answering is felt. With the model in the webview the
//! loop was: wheel -> Gamepad API poll at frame rate -> physics -> IPC ->
//! native slew -> motor, some 25-30 ms at worst. Here it is: DirectInput read
//! -> physics step -> torque write, on one thread, about a millisecond.
//!
//! Division of labour:
//!
//!   here      steering-wheel read, driver aids, gearbox requests, vehicle
//!             model, the oval barrier, the force feedback mix, the motor
//!   webview   rendering, HUD, audio, cones, timing, the ETC pedal map (the
//!             pedals arrive at frame rate either way), settings
//!
//! The webview sends `rig_frame` once per rendered frame with the pad or
//! keyboard input and gets the latest snapshot back; a wheel is read here
//! directly and the pad input is ignored while it is. Rare things -- respawn,
//! a parameter edit, the barrier -- go through `rig_command`.
//!
//! Nothing allocates on the tick path. The snapshot is a plain `Copy` struct
//! behind a mutex that is held for the length of a memcpy; the inputs the
//! same. The physics itself is a few hundred flops per substep.

use serde::{Deserialize, Serialize};
use sim_core::prelude::*;
use sim_core::solver::Solver;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::wheel::{self, DeviceInfo, DeviceState, Wheel, AXES_PER_DEVICE, MAX_DEVICES};

const RATE_HZ: f64 = 1000.0;

// ------------------------------------------------------------------ inputs --

/// What the webview sends every frame.
#[derive(Clone, Copy, Debug, Default, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RigInput {
    /// -1..1, left positive (the vehicle model's frame), from a pad or the keys.
    pub steer: f64,
    /// Throttle plate demand 0..1, already through the ETC pedal map.
    pub throttle: f64,
    pub brake: f64,
    /// Rim angle the webview read, deg right-positive, when no native wheel.
    pub rim_deg: f64,
    pub half_lock_deg: f64,
    pub traction: bool,
    pub abs: bool,
    pub auto_shift: bool,
    pub ffb_enabled: bool,
    /// Physics holds while true (menu, pause, map editor).
    pub paused: bool,
    /// Off-course and cone hits come from the course, which lives in the
    /// webview; they arrive here as feel.
    pub off_track: bool,
    pub cone_hits: u32,
    pub shift_up: bool,
    pub shift_down: bool,
}

/// The wheel profile, as far as the rig needs it.
#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct WheelConfig {
    /// Read the rim from DirectInput and steer by it.
    pub enabled: bool,
    pub steer_axis: usize,
    pub rotation_deg: f64,
    /// "match-car" or "scale-to-lock".
    pub mapping: String,
    pub soft_lock: bool,
    pub centre_trim_deg: f64,
    /// Car lock at the rim, one side (deg). Sent by the webview from the
    /// vehicle's lock and ratio.
    pub car_rim_half_deg: f64,
    /// Product name of the base the driver picked, if any. Empty = choose.
    pub device_name: String,
    pub throttle: Option<PedalCal>,
    pub brake: Option<PedalCal>,
    /// Pedal -> plate map, [[pedal %, plate %], ...]. Applied to the native
    /// throttle only; the webview applies it to a pad's.
    pub etc_points: Vec<[f64; 2]>,
}

impl Default for WheelConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            steer_axis: 0,
            rotation_deg: 900.0,
            mapping: "match-car".into(),
            soft_lock: true,
            centre_trim_deg: 0.0,
            car_rim_half_deg: 56.0,
            device_name: String::new(),
            throttle: None,
            brake: None,
            etc_points: vec![[0.0, 0.0], [100.0, 100.0]],
        }
    }
}

/// Same shape as `controlProfiles.js` `pedal()`.
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct PedalCal {
    pub axis: usize,
    pub raw_min: f64,
    pub raw_max: f64,
    pub deadzone_low: f64,
    pub deadzone_high: f64,
    pub gamma: f64,
    pub invert: bool,
}

impl Default for PedalCal {
    fn default() -> Self {
        Self { axis: 1, raw_min: 1.0, raw_max: -1.0, deadzone_low: 0.02, deadzone_high: 0.02, gamma: 1.0, invert: false }
    }
}

impl PedalCal {
    /// Port of `controlProfiles.js` `applyPedal`, step for step.
    fn apply(&self, raw: f64) -> f64 {
        let span = self.raw_max - self.raw_min;
        let mut v = if span.abs() < 1e-6 { 0.0 } else { (raw - self.raw_min) / span };
        if self.invert {
            v = 1.0 - v;
        }
        v = v.clamp(0.0, 1.0);
        let lo = self.deadzone_low;
        let hi = 1.0 - self.deadzone_high;
        v = if hi - lo < 1e-6 { 0.0 } else { (v - lo) / (hi - lo) };
        v = v.clamp(0.0, 1.0);
        if self.gamma == 1.0 {
            v
        } else {
            v.powf(self.gamma)
        }
    }
}

/// The profile's `forceFeedback` block.
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct FfbConfig {
    pub enabled: bool,
    pub gain: f64,
    pub align_torque_gain: f64,
    pub road_texture_gain: f64,
    pub damping: f64,
    pub friction: f64,
    pub soft_lock_gain: f64,
    pub min_force: f64,
    pub max_force_nm: f64,
    pub invert: bool,
}

impl Default for FfbConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            gain: 0.55,
            align_torque_gain: 1.0,
            road_texture_gain: 0.35,
            damping: 0.15,
            friction: 0.04,
            soft_lock_gain: 1.0,
            min_force: 0.0,
            max_force_nm: 5.5,
            invert: false,
        }
    }
}

/// Everything the webview's live parameter editing can change. Flat, all
/// optional, camelCase to match `params.js`; only the fields present are
/// applied.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct ParamSet {
    pub mass_kg: Option<f64>,
    pub weight_dist_front: Option<f64>,
    pub cg_height_m: Option<f64>,
    pub wheelbase_m: Option<f64>,
    pub track_front_m: Option<f64>,
    pub track_rear_m: Option<f64>,
    pub tire_radius_m: Option<f64>,
    pub izz_kg_m2: Option<f64>,
    pub unsprung_front_kg: Option<f64>,
    pub unsprung_rear_kg: Option<f64>,
    pub wheel_inertia_front_kg_m2: Option<f64>,
    pub wheel_inertia_rear_kg_m2: Option<f64>,
    pub crr: Option<f64>,
    pub cda_m2: Option<f64>,
    pub cla_m2: Option<f64>,
    pub aero_front_frac: Option<f64>,
    pub air_density_kg_m3: Option<f64>,
    pub rsd_front: Option<f64>,
    pub h_roll_arm_m: Option<f64>,
    pub rc_front_m: Option<f64>,
    pub rc_rear_m: Option<f64>,
    pub brake_torque_max_nm: Option<f64>,
    pub brake_bias_front: Option<f64>,
    pub max_steer_deg: Option<f64>,
    pub steer_lag_s: Option<f64>,
    pub steer_rate_deg_s: Option<f64>,
    pub steer_accel_deg_s2: Option<f64>,
    pub steering_ratio: Option<f64>,
    pub caster_deg: Option<f64>,
    pub kingpin_offset_trail_m: Option<f64>,
    pub rack_efficiency: Option<f64>,
    pub torque_ratio: Option<f64>,
    pub mu_lat: Option<f64>,
    pub mu_long: Option<f64>,
    pub tire_load_sensitivity: Option<f64>,
    pub relax_length_m: Option<f64>,
    pub gear_ratios: Option<Vec<f64>>,
    pub primary_reduction: Option<f64>,
    pub final_drive: Option<f64>,
    pub drivetrain_eff: Option<f64>,
    pub rev_limit_rpm: Option<f64>,
    pub idle_rpm: Option<f64>,
    pub idle_throttle_frac: Option<f64>,
    pub shift_time_s: Option<f64>,
    pub engine_inertia_kg_m2: Option<f64>,
    pub gearbox_inertia_kg_m2: Option<f64>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum RigCommand {
    Respawn { x: f64, y: f64, psi: f64, speed: f64 },
    Params(ParamSet),
    #[serde(rename_all = "camelCase")]
    Boundary { centre: Vec<[f64; 2]>, offset_m: f64 },
    ClearBoundary,
    Ffb(FfbConfig),
    Wheel(WheelConfig),
    /// Re-open the wheel, steering by the named base (or the best guess).
    #[serde(rename_all = "camelCase")]
    SelectDevice { name: String },
}

// ---------------------------------------------------------------- outputs --

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct StateOut {
    pub x: f64,
    pub y: f64,
    pub psi: f64,
    pub u: f64,
    pub v: f64,
    pub r: f64,
    pub w_f: f64,
    pub w_r: f64,
    pub delta: f64,
}

/// The JS `telemetry` object, name for name.
#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct TelemetryOut {
    pub speed: f64,
    pub ax_g: f64,
    pub ay_g: f64,
    pub body_slip_deg: f64,
    pub yaw_rate_deg_s: f64,
    #[serde(rename = "FzF")]
    pub fz_f: f64,
    #[serde(rename = "FzR")]
    pub fz_r: f64,
    #[serde(rename = "dFzLatF")]
    pub d_fz_lat_f: f64,
    #[serde(rename = "dFzLatR")]
    pub d_fz_lat_r: f64,
    pub slip_f: f64,
    pub slip_r: f64,
    pub kappa_f: f64,
    pub kappa_r: f64,
    pub util_f: f64,
    pub util_r: f64,
    pub balance: f64,
    pub downforce_n: f64,
    pub drag_n: f64,
    pub drive_force_n: f64,
    pub steer_deg: f64,
    pub locked: bool,
    pub kingpin_torque_nm: f64,
    pub rim_torque_nm: f64,
    pub trail_fm: f64,
    pub mech_trail_m: f64,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PowertrainOut {
    pub engine_rpm: f64,
    pub gear: usize,
    pub shifting: bool,
    pub slipping: bool,
    pub can_shift: bool,
    pub shift_rpm: f64,
    pub downshift_safe: bool,
    pub indicated_torque_nm: f64,
    pub plate: f64,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct AppliedOut {
    pub steer: f64,
    pub throttle: f64,
    pub brake: f64,
    /// Where the steering came from this tick.
    pub native_steer: bool,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct FfbOut {
    /// -1..1 as sent to the motor (texture and kick included).
    pub command: f64,
    pub torque_nm: f64,
    pub align: f64,
    pub damping: f64,
    pub friction: f64,
    pub soft_lock: f64,
    pub texture_nm: f64,
    pub clipped: bool,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DeviceOut {
    pub present: bool,
    /// Axis `8*d + i`: axis i of device d; device 0 is the base.
    pub axes: [f32; MAX_DEVICES * AXES_PER_DEVICE],
    /// Per device, bit i is button i.
    pub buttons: [u32; MAX_DEVICES],
    pub pov: i32,
    pub rim_deg: f64,
    pub half_lock_deg: f64,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct StatsOut {
    pub ticks: u64,
    pub tick_us_avg: f64,
    pub tick_us_max: f64,
    pub overruns: u64,
    pub rate_hz: f64,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Snapshot {
    pub state: StateOut,
    pub tel: TelemetryOut,
    pub pt: PowertrainOut,
    pub applied: AppliedOut,
    pub ffb: FfbOut,
    pub device: DeviceOut,
    pub stats: StatsOut,
    pub boundary_hit: bool,
    pub money_shift_blocked: bool,
}

#[derive(Clone, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct RigStatus {
    pub running: bool,
    /// This build can drive a wheel at all (Windows).
    pub ffb_supported: bool,
    /// A base is open and being read.
    pub wheel_present: bool,
    /// ...and it has an actuator DirectInput can drive.
    pub ffb_active: bool,
    pub wheel_name: String,
    /// Every device being read, base first.
    pub device_names: Vec<String>,
    pub wheel_error: String,
    /// Everything plugged in, for the picker.
    pub available: Vec<DeviceInfo>,
}

// ----------------------------------------------------------------- shared --

pub struct Shared {
    input: Mutex<RigInput>,
    commands: Mutex<Vec<RigCommand>>,
    snapshot: Mutex<Snapshot>,
    status: Mutex<RigStatus>,
    running: AtomicBool,
    shift_up: AtomicU32,
    shift_down: AtomicU32,
    cone_hits: AtomicU32,
}

pub struct Rig {
    shared: Arc<Shared>,
    thread: Mutex<Option<std::thread::JoinHandle<()>>>,
}

impl Rig {
    pub fn new() -> Self {
        Rig {
            shared: Arc::new(Shared {
                input: Mutex::new(RigInput::default()),
                commands: Mutex::new(Vec::new()),
                snapshot: Mutex::new(Snapshot::default()),
                status: Mutex::new(RigStatus { ffb_supported: cfg!(windows), ..Default::default() }),
                running: AtomicBool::new(false),
                shift_up: AtomicU32::new(0),
                shift_down: AtomicU32::new(0),
                cone_hits: AtomicU32::new(0),
            }),
            thread: Mutex::new(None),
        }
    }
}

// --------------------------------------------------------------- commands --

#[tauri::command]
pub fn rig_status(state: tauri::State<'_, Rig>) -> RigStatus {
    state.shared.status.lock().unwrap().clone()
}

/// Start the rig thread. Async so opening the wheel never blocks the UI.
#[tauri::command(async)]
pub fn rig_start(window: tauri::Window, state: tauri::State<'_, Rig>) -> RigStatus {
    let mut th = state.thread.lock().unwrap();
    if th.is_some() && state.shared.running.load(Ordering::SeqCst) {
        return state.shared.status.lock().unwrap().clone();
    }
    #[cfg(windows)]
    let hwnd_raw: isize = window.hwnd().map(|h| h.0 as isize).unwrap_or(0);
    #[cfg(not(windows))]
    let hwnd_raw: isize = {
        let _ = &window;
        0
    };
    let shared = state.shared.clone();
    shared.running.store(true, Ordering::SeqCst);
    let (ready_tx, ready_rx) = std::sync::mpsc::channel::<()>();
    *th = Some(
        std::thread::Builder::new()
            .name("rig".into())
            .spawn(move || run(shared, hwnd_raw, ready_tx))
            .expect("spawn rig thread"),
    );
    let _ = ready_rx.recv();
    state.shared.status.lock().unwrap().clone()
}

#[tauri::command]
pub fn rig_stop(state: tauri::State<'_, Rig>) -> RigStatus {
    state.shared.running.store(false, Ordering::SeqCst);
    if let Some(h) = state.thread.lock().unwrap().take() {
        let _ = h.join();
    }
    let mut st = state.shared.status.lock().unwrap();
    st.running = false;
    st.clone()
}

/// The per-frame exchange: inputs in, the latest snapshot out.
#[tauri::command]
pub fn rig_frame(state: tauri::State<'_, Rig>, input: RigInput) -> Snapshot {
    let sh = &state.shared;
    if input.shift_up {
        sh.shift_up.fetch_add(1, Ordering::Relaxed);
    }
    if input.shift_down {
        sh.shift_down.fetch_add(1, Ordering::Relaxed);
    }
    if input.cone_hits > 0 {
        sh.cone_hits.fetch_add(input.cone_hits, Ordering::Relaxed);
    }
    *sh.input.lock().unwrap() = input;
    *sh.snapshot.lock().unwrap()
}

#[tauri::command]
pub fn rig_command(state: tauri::State<'_, Rig>, command: RigCommand) {
    state.shared.commands.lock().unwrap().push(command);
}

// ------------------------------------------------------------------- loop --

/// Everything the tick needs, owned by the thread.
struct Loop {
    car: Box<dyn Solver>,
    assists: Assists,
    boundary: Option<Boundary>,
    etc: EtcMap,
    wheel: Option<Wheel>,
    hwnd_raw: isize,
    shared: Arc<Shared>,
    wheel_cfg: WheelConfig,
    ffb_cfg: FfbConfig,
    ffb: FfbMixer,
    device: DeviceState,
    device_present: bool,
    // for the display-only gradients the JS model carried
    ticks: u64,
    tick_us_sum: f64,
    tick_us_max: f64,
    overruns: u64,
    boundary_hit: bool,
}

fn run(shared: Arc<Shared>, hwnd_raw: isize, ready: std::sync::mpsc::Sender<()>) {
    wheel::realtime_thread();

    let mut car = build(
        Fidelity::Bicycle,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    );
    car.reset(0.0, 0.0, 0.0, 0.0);

    // The wheel is optional: no wheel means the webview steers.
    let wheel = open_wheel(&shared, hwnd_raw, None);
    {
        let mut st = shared.status.lock().unwrap();
        st.running = true;
    }
    let _ = ready.send(());

    let mut lp = Loop {
        car,
        assists: Assists::default(),
        boundary: None,
        etc: EtcMap::linear(),
        wheel,
        hwnd_raw,
        shared: shared.clone(),
        wheel_cfg: WheelConfig::default(),
        ffb_cfg: FfbConfig::default(),
        ffb: FfbMixer::default(),
        device: DeviceState::default(),
        device_present: false,
        ticks: 0,
        tick_us_sum: 0.0,
        tick_us_max: 0.0,
        overruns: 0,
        boundary_hit: false,
    };

    let period = Duration::from_secs_f64(1.0 / RATE_HZ);
    let mut next = Instant::now() + period;
    let mut last = Instant::now();
    let mut cmds: Vec<RigCommand> = Vec::new();

    while shared.running.load(Ordering::Relaxed) {
        let t0 = Instant::now();
        let dt = (t0 - last).as_secs_f64().clamp(1e-4, 0.01);
        last = t0;

        // Rare commands, drained without holding the lock across the tick.
        {
            let mut q = shared.commands.lock().unwrap();
            if !q.is_empty() {
                cmds.append(&mut q);
            }
        }
        for c in cmds.drain(..) {
            lp.apply_command(c);
        }

        let input = *shared.input.lock().unwrap();
        let shift_up = shared.shift_up.swap(0, Ordering::Relaxed);
        let shift_down = shared.shift_down.swap(0, Ordering::Relaxed);
        let cone_hits = shared.cone_hits.swap(0, Ordering::Relaxed);

        let snap = lp.tick(dt, &input, shift_up, shift_down, cone_hits);
        *shared.snapshot.lock().unwrap() = snap;

        // Pace. Sleep to just short of the deadline, spin the rest: Windows
        // sleeps are good to ~0.5 ms with timeBeginPeriod(1), not to 50 us.
        let spent = t0.elapsed().as_secs_f64() * 1e6;
        lp.ticks += 1;
        lp.tick_us_sum += spent;
        lp.tick_us_max = lp.tick_us_max.max(spent);
        let now = Instant::now();
        if next > now {
            let remaining = next - now;
            if remaining > Duration::from_micros(300) {
                std::thread::sleep(remaining - Duration::from_micros(250));
            }
            while Instant::now() < next {
                std::hint::spin_loop();
            }
            next += period;
        } else {
            lp.overruns += 1;
            next = now + period;
        }
    }

    if let Some(w) = lp.wheel.as_mut() {
        w.close();
    }
    let mut st = shared.status.lock().unwrap();
    st.running = false;
}

/// Open (or re-open) the base and report what happened in the status.
fn open_wheel(shared: &Arc<Shared>, hwnd_raw: isize, prefer: Option<&str>) -> Option<Wheel> {
    let available = wheel::enumerate();
    let result = Wheel::open(hwnd_raw, prefer);
    let mut st = shared.status.lock().unwrap();
    st.available = available;
    match result {
        Ok(w) => {
            st.wheel_present = true;
            st.ffb_active = w.ffb;
            st.wheel_name = w.name.clone();
            st.device_names = w.names.clone();
            st.wheel_error = if w.ffb { String::new() } else { "no force feedback actuator on this base (console mode, or a wheel DirectInput cannot drive); steering and pedals still work".into() };
            Some(w)
        }
        Err(e) => {
            st.wheel_present = false;
            st.ffb_active = false;
            st.wheel_name.clear();
            st.device_names.clear();
            st.wheel_error = e;
            None
        }
    }
}

impl Loop {
    fn apply_command(&mut self, c: RigCommand) {
        match c {
            RigCommand::Respawn { x, y, psi, speed } => {
                self.car.reset(x, y, psi, speed);
                self.ffb = FfbMixer::default();
            }
            RigCommand::Params(p) => self.apply_params(&p),
            RigCommand::Boundary { centre, offset_m } => {
                self.boundary = Some(Boundary::new(centre, offset_m));
            }
            RigCommand::ClearBoundary => self.boundary = None,
            RigCommand::Ffb(cfg) => self.ffb_cfg = cfg,
            RigCommand::Wheel(cfg) => {
                let pts: Vec<(f64, f64)> = cfg.etc_points.iter().map(|p| (p[0], p[1])).collect();
                if pts.len() >= 2 {
                    self.etc.set_points(&pts);
                }
                let want = cfg.device_name.clone();
                let have = self.wheel.as_ref().map(|w| w.name.clone()).unwrap_or_default();
                self.wheel_cfg = cfg;
                if !want.is_empty() && want != have {
                    self.reopen(Some(&want));
                }
            }
            RigCommand::SelectDevice { name } => {
                let prefer = if name.is_empty() { None } else { Some(name.as_str()) };
                self.reopen(prefer);
            }
        }
    }

    fn reopen(&mut self, prefer: Option<&str>) {
        if let Some(mut w) = self.wheel.take() {
            w.close();
        }
        self.device = DeviceState::default();
        self.device_present = false;
        self.wheel = open_wheel(&self.shared, self.hwnd_raw, prefer);
    }

    fn apply_params(&mut self, p: &ParamSet) {
        {
            let v = self.car.params_mut();
            macro_rules! set {
                ($($src:ident => $dst:expr),* $(,)?) => { $( if let Some(x) = p.$src { $dst = x; } )* };
            }
            set! {
                mass_kg => v.mass_kg, weight_dist_front => v.weight_dist_front, cg_height_m => v.cg_height_m,
                wheelbase_m => v.wheelbase_m, track_front_m => v.track_front_m, track_rear_m => v.track_rear_m,
                tire_radius_m => v.tyre_radius_m, izz_kg_m2 => v.izz_kg_m2,
                unsprung_front_kg => v.unsprung_front_kg, unsprung_rear_kg => v.unsprung_rear_kg,
                wheel_inertia_front_kg_m2 => v.wheel_inertia_front_kg_m2, wheel_inertia_rear_kg_m2 => v.wheel_inertia_rear_kg_m2,
                crr => v.crr, cda_m2 => v.aero.cda_m2, cla_m2 => v.aero.cla_m2, aero_front_frac => v.aero.front_frac,
                air_density_kg_m3 => v.aero.air_density, rsd_front => v.roll.rsd_front, h_roll_arm_m => v.roll.roll_arm_m,
                rc_front_m => v.roll.rc_front_m, rc_rear_m => v.roll.rc_rear_m,
                brake_torque_max_nm => v.brakes.max_torque_nm, brake_bias_front => v.brakes.bias_front,
                steer_lag_s => v.steering.lag_s, steering_ratio => v.steering.ratio,
                kingpin_offset_trail_m => v.steering.kingpin_offset_trail_m, rack_efficiency => v.steering.rack_efficiency,
            }
            if let Some(x) = p.max_steer_deg { v.steering.max_steer_rad = x.to_radians(); }
            if let Some(x) = p.steer_rate_deg_s { v.steering.rate_rad_s = x.to_radians(); }
            if let Some(x) = p.steer_accel_deg_s2 { v.steering.accel_rad_s2 = x.to_radians(); }
            if let Some(x) = p.caster_deg { v.steering.caster_rad = x.to_radians(); }
            if p.torque_ratio.is_some() { v.steering.torque_ratio = p.torque_ratio; }
        }
        let nominal = self.car.params().nominal_tyre_load();
        if let Some(t) = self.car.tyre_mut().as_any_mut().and_then(|a| a.downcast_mut::<MagicFormulaTyre>()) {
            t.nominal_load = nominal;
            if let Some(x) = p.mu_lat { t.mu_y = x; }
            if let Some(x) = p.mu_long { t.mu_x = x; }
            if let Some(x) = p.tire_load_sensitivity { t.load_sensitivity = x; }
            if let Some(x) = p.relax_length_m { t.relaxation_m = x; }
        }
        if let Some(e) = self.car.powertrain_mut().as_any_mut().and_then(|a| a.downcast_mut::<GearedEngine>()) {
            if let Some(g) = &p.gear_ratios { if !g.is_empty() { e.gear_ratios = g.clone(); } }
            if let Some(x) = p.primary_reduction { e.primary = x; }
            if let Some(x) = p.final_drive { e.final_drive = x; }
            if let Some(x) = p.drivetrain_eff { e.efficiency = x; }
            if let Some(x) = p.rev_limit_rpm { e.rev_limit_rpm = x; }
            if let Some(x) = p.idle_rpm { e.idle_rpm = x; }
            if let Some(x) = p.idle_throttle_frac { e.idle_throttle_frac = x; }
            if let Some(x) = p.shift_time_s { e.shift_time_s = x; }
            if let Some(x) = p.engine_inertia_kg_m2 { e.crank_inertia_kg_m2 = x; }
            if let Some(x) = p.gearbox_inertia_kg_m2 { e.gearbox_inertia_kg_m2 = x; }
        }
    }

    /// One millisecond of the world.
    fn tick(&mut self, dt: f64, input: &RigInput, shift_up: u32, shift_down: u32, cone_hits: u32) -> Snapshot {
        // ---- the wheel, if there is one ----
        if let Some(w) = self.wheel.as_mut() {
            match w.read() {
                Some(s) => {
                    self.device = s;
                    self.device_present = true;
                }
                None => self.device_present = false,
            }
        }
        let native = self.device_present && self.wheel_cfg.enabled;

        // ---- steering ----
        let (steer, rim_deg, half_lock) = if native {
            let wc = &self.wheel_cfg;
            let raw = self.device.axes.get(wc.steer_axis).copied().unwrap_or(0.0) as f64;
            let rim = raw * (wc.rotation_deg.max(1.0) / 2.0) - wc.centre_trim_deg;
            let half = if wc.mapping == "match-car" { wc.car_rim_half_deg.max(1e-6) } else { wc.rotation_deg.max(1.0) / 2.0 };
            let mut norm = rim / half;
            if wc.soft_lock {
                norm = norm.clamp(-1.0, 1.0);
            }
            // Device is right-positive; the model is left-positive.
            (-norm.clamp(-1.0, 1.0), rim, half)
        } else {
            (input.steer, input.rim_deg, input.half_lock_deg.max(1e-6))
        };

        // ---- pedals ----
        let (throttle_demand, brake_demand) = if native {
            let th = self.wheel_cfg.throttle.map(|c| c.apply(self.device.axes.get(c.axis).copied().unwrap_or(0.0) as f64));
            let br = self.wheel_cfg.brake.map(|c| c.apply(self.device.axes.get(c.axis).copied().unwrap_or(0.0) as f64));
            // Native throttle goes through the ETC map here; the webview
            // already mapped a pad's.
            (th.map(|p| self.etc.evaluate(p)).unwrap_or(input.throttle), br.unwrap_or(input.brake))
        } else {
            (input.throttle, input.brake)
        };

        let prev = self.car.telemetry();
        self.assists.traction = input.traction;
        self.assists.abs = input.abs;
        let throttle = self.assists.throttle(throttle_demand.clamp(0.0, 1.0), prev.kappa[2]);
        let brake = self.assists.brake(brake_demand.clamp(0.0, 1.0), prev.kappa[0], prev.kappa[2]);

        // ---- gearbox ----
        let mut money_shift_blocked = false;
        if !input.paused {
            let w_r = prev.wheel_omega_rear;
            let pt = self.car.powertrain_mut();
            if input.auto_shift {
                if pt.can_shift() {
                    let t = pt.telemetry();
                    if t.engine_rpm > pt.optimal_upshift_rpm() {
                        pt.shift_up();
                    } else if t.gear > 0 && t.engine_rpm < 5200.0 && pt.downshift_safe(w_r) {
                        pt.shift_down();
                    }
                }
            } else {
                for _ in 0..shift_up {
                    pt.shift_up();
                }
                for _ in 0..shift_down {
                    if pt.downshift_safe(w_r) {
                        pt.shift_down();
                    } else {
                        money_shift_blocked = true;
                    }
                }
            }
        }

        // ---- physics ----
        if !input.paused {
            self.car.step(dt, Controls { steer, throttle, brake });
            self.boundary_hit = match self.boundary.as_mut() {
                Some(b) => b.constrain(self.car.state_mut()),
                None => false,
            };
        }
        let s = self.car.state();
        let tel = self.car.telemetry();

        // ---- force feedback ----
        let feel = Feel {
            spin: ((tel.kappa[2] - 0.2).max(0.0) * 2.5).min(1.0),
            lock: ((-tel.kappa[0].min(tel.kappa[2]) - 0.2).max(0.0) * 2.5).min(1.0),
            off_track: input.off_track && tel.speed > 2.0,
            cone_hits,
        };
        let ffb_on = self.ffb_cfg.enabled && input.ffb_enabled && !input.paused;
        let ffb = self.ffb.mix(dt, &self.ffb_cfg, ffb_on, &tel, rim_deg, half_lock, native, &feel);
        if let Some(w) = self.wheel.as_mut() {
            if w.ffb {
                let _ = w.set_torque(if ffb_on { ffb.command as f32 } else { 0.0 });
            }
        }

        // ---- powertrain readouts the HUD and audio want ----
        let ptm = self.car.powertrain_mut();
        let ptt = ptm.telemetry();
        let pt_out = PowertrainOut {
            engine_rpm: ptt.engine_rpm,
            gear: ptt.gear,
            shifting: ptt.shifting,
            slipping: ptt.slipping,
            can_shift: ptm.can_shift(),
            shift_rpm: ptm.optimal_upshift_rpm(),
            downshift_safe: ptm.downshift_safe(tel.wheel_omega_rear),
            indicated_torque_nm: if ptt.shifting { 0.0 } else { ptm.indicated_torque_nm(ptt.engine_rpm, throttle) },
            plate: if ptt.shifting { 0.0 } else { ptm.plate_position(ptt.engine_rpm, throttle) },
        };

        let fz_f = tel.fz[FL] + tel.fz[FR];
        let fz_r = tel.fz[RL] + tel.fz[RR];
        Snapshot {
            state: StateOut {
                x: s.x, y: s.y, psi: s.psi, u: s.u, v: s.v, r: s.r,
                w_f: tel.wheel_omega_front, w_r: tel.wheel_omega_rear, delta: tel.steer_rad,
            },
            tel: TelemetryOut {
                speed: tel.speed,
                ax_g: tel.ax_g,
                ay_g: tel.ay_g,
                body_slip_deg: tel.body_slip_deg,
                yaw_rate_deg_s: tel.yaw_rate_deg_s,
                fz_f,
                fz_r,
                d_fz_lat_f: (tel.fz[FR] - tel.fz[FL]).abs() / 2.0,
                d_fz_lat_r: (tel.fz[RR] - tel.fz[RL]).abs() / 2.0,
                slip_f: tel.slip_deg[FL],
                slip_r: tel.slip_deg[RL],
                kappa_f: tel.kappa[FL],
                kappa_r: tel.kappa[RL],
                util_f: tel.utilisation[FL],
                util_r: tel.utilisation[RL],
                balance: tel.balance,
                downforce_n: tel.downforce_n,
                drag_n: tel.drag_n,
                drive_force_n: tel.drive_force_n,
                steer_deg: tel.steer_rad.to_degrees(),
                locked: tel.locked,
                kingpin_torque_nm: tel.kingpin_torque_nm,
                rim_torque_nm: tel.rim_torque_nm,
                trail_fm: tel.trail_front_m,
                mech_trail_m: tel.mech_trail_m,
            },
            pt: pt_out,
            applied: AppliedOut { steer, throttle, brake, native_steer: native },
            ffb,
            device: DeviceOut {
                present: self.device_present,
                axes: self.device.axes,
                buttons: self.device.buttons,
                pov: self.device.pov,
                rim_deg,
                half_lock_deg: half_lock,
            },
            stats: StatsOut {
                ticks: self.ticks,
                tick_us_avg: if self.ticks > 0 { self.tick_us_sum / self.ticks as f64 } else { 0.0 },
                tick_us_max: self.tick_us_max,
                overruns: self.overruns,
                rate_hz: RATE_HZ,
            },
            boundary_hit: self.boundary_hit,
            money_shift_blocked,
        }
    }
}

// -------------------------------------------------------------------- ffb --

struct Feel {
    spin: f64,
    lock: f64,
    off_track: bool,
    cone_hits: u32,
}

/// The force feedback mix, once per tick. Port of `forceFeedback.js` with the
/// review's corrections: the minimum-force floor is applied to the final
/// command, texture is given headroom so it cannot bias the base torque
/// through the clip, and the rim rate is a real 1 kHz derivative.
#[derive(Default)]
struct FfbMixer {
    rim_deg: f64,
    rim_rate_deg_s: f64,
    friction_state: f64,
    phase: f64,
    kick: f64,
}

impl FfbMixer {
    #[allow(clippy::too_many_arguments)]
    fn mix(&mut self, dt: f64, cfg: &FfbConfig, on: bool, tel: &sim_core::solver::Telemetry, rim_deg: f64, half_lock: f64, native: bool, feel: &Feel) -> FfbOut {
        let mut out = FfbOut::default();
        // Rim velocity. A native read is a clean 1 kHz signal and wants a
        // short filter; a webview rim arrives at frame rate and needs a
        // longer one or the derivative is a comb of spikes.
        let tau = if native { 0.004 } else { 0.025 };
        let raw_rate = (rim_deg - self.rim_deg) / dt.max(1e-4);
        self.rim_rate_deg_s += (raw_rate - self.rim_rate_deg_s) * (dt / tau).min(1.0);
        self.rim_deg = rim_deg;
        if !on {
            self.kick = 0.0;
            return out;
        }
        let rated = cfg.max_force_nm.max(0.1);
        let rate = self.rim_rate_deg_s.to_radians();

        // Tyres. The model is left-positive; the wheel is clockwise-positive.
        out.align = -tel.rim_torque_nm * cfg.align_torque_gain;
        // Damping: `damping` is the fraction of rated torque at 10 rad/s.
        out.damping = -cfg.damping * rated * (rate / 10.0);
        // Coulomb friction with a soft sign.
        let target = (rate / 0.3).tanh();
        self.friction_state += (target - self.friction_state) * (dt / 0.03).min(1.0);
        out.friction = -cfg.friction * rated * self.friction_state;
        // End stops past the car's lock.
        let over = rim_deg.abs() - half_lock;
        if over > 0.0 {
            out.soft_lock = -rim_deg.signum() * (over / 6.0).min(1.0) * cfg.soft_lock_gain * rated;
        }
        out.torque_nm = out.align + out.damping + out.friction + out.soft_lock;

        let mut cmd = out.torque_nm * cfg.gain / rated;
        if cfg.min_force > 0.0 && out.align.abs() > 1e-3 && cmd.abs() < cfg.min_force {
            cmd = cmd.signum() * cfg.min_force;
        }
        if cmd.abs() > 1.0 {
            out.clipped = true;
        }
        cmd = cmd.clamp(-1.0, 1.0);

        // Texture: wheelspin and lockup at wheel frequency, grass as a slow
        // rumble. Rendered here as a sine, with headroom so it rides on top of
        // the base torque instead of pushing it through the clip.
        let slip_tex = feel.spin.max(feel.lock);
        let rough = if feel.off_track { 1.0 } else { 0.0 };
        let mut texture = 0.0;
        if slip_tex > 0.02 || rough > 0.0 {
            let amp_nm = ((slip_tex * 0.35 + rough * 0.25) * cfg.road_texture_gain * rated).min(0.5 * rated);
            let hz = if rough > 0.0 && slip_tex < 0.02 { 12.0 } else { (tel.speed / (std::f64::consts::TAU * 0.2)).clamp(8.0, 45.0) };
            self.phase = (self.phase + hz * dt).fract();
            let amp = (amp_nm / rated).min(1.0 - cmd.abs());
            texture = amp * (self.phase * std::f64::consts::TAU).sin();
            out.texture_nm = amp * rated;
        }

        // A cone: a short kick against the current steer, decaying in 40 ms.
        if feel.cone_hits > 0 {
            self.kick = 0.6 * if rim_deg >= 0.0 { -1.0 } else { 1.0 };
        }
        let kick = self.kick;
        self.kick *= 1.0 - (dt / 0.04).min(1.0);
        if self.kick.abs() < 1e-3 {
            self.kick = 0.0;
        }

        let mut total = cmd + texture + kick;
        if cfg.invert {
            total = -total;
        }
        out.command = total.clamp(-1.0, 1.0);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tel_with_rim(rim: f64) -> sim_core::solver::Telemetry {
        sim_core::solver::Telemetry { rim_torque_nm: rim, speed: 15.0, ..Default::default() }
    }

    #[test]
    fn left_turn_commands_clockwise() {
        let mut m = FfbMixer::default();
        let cfg = FfbConfig::default();
        let feel = Feel { spin: 0.0, lock: 0.0, off_track: false, cone_hits: 0 };
        // A left turn: the model's rim torque is negative (tries to steer
        // back right); the wheel must be driven clockwise.
        let o = m.mix(0.001, &cfg, true, &tel_with_rim(-3.0), -20.0, 56.0, true, &feel);
        assert!(o.command > 0.0, "{o:?}");
    }

    #[test]
    fn texture_keeps_headroom() {
        let mut m = FfbMixer::default();
        let cfg = FfbConfig { gain: 1.0, ..Default::default() };
        let feel = Feel { spin: 1.0, lock: 0.0, off_track: false, cone_hits: 0 };
        for _ in 0..50 {
            let o = m.mix(0.001, &cfg, true, &tel_with_rim(-5.4), 0.0, 56.0, true, &feel);
            assert!(o.command.abs() <= 1.0);
            assert!(o.texture_nm <= 0.11, "texture must fit above a near-full base: {}", o.texture_nm);
        }
    }

    #[test]
    fn off_means_zero_and_no_stale_kick() {
        let mut m = FfbMixer::default();
        let cfg = FfbConfig::default();
        let feel = Feel { spin: 0.0, lock: 0.0, off_track: false, cone_hits: 1 };
        let _ = m.mix(0.001, &cfg, true, &tel_with_rim(0.0), 0.0, 56.0, true, &feel);
        let o = m.mix(0.001, &cfg, false, &tel_with_rim(0.0), 0.0, 56.0, true, &feel);
        assert_eq!(o.command, 0.0);
        assert_eq!(m.kick, 0.0);
    }

    /// The whole thread, for real: start it, hold the throttle, and check
    /// that the car moved, the loop kept its rate, and a tick is cheap.
    #[test]
    fn loop_runs_at_rate_and_drives_the_car() {
        let rig = Rig::new();
        let shared = rig.shared.clone();
        shared.running.store(true, Ordering::SeqCst);
        let (tx, rx) = std::sync::mpsc::channel();
        let th = {
            let shared = shared.clone();
            std::thread::spawn(move || run(shared, 0, tx))
        };
        rx.recv().unwrap();
        *shared.input.lock().unwrap() = RigInput { throttle: 1.0, auto_shift: true, ..Default::default() };
        std::thread::sleep(Duration::from_millis(400));
        let snap = *shared.snapshot.lock().unwrap();
        shared.running.store(false, Ordering::SeqCst);
        th.join().unwrap();
        assert!(snap.state.x > 0.3, "car did not move: {:?}", snap.state);
        assert!(snap.stats.ticks >= 300, "ticks {}", snap.stats.ticks);
        assert!(snap.stats.tick_us_avg < 300.0, "tick too slow: {} us", snap.stats.tick_us_avg);
        assert!(snap.stats.overruns < snap.stats.ticks / 10, "overruns {}", snap.stats.overruns);
        assert!(snap.pt.engine_rpm > 2000.0);
    }

    #[test]
    fn pedal_calibration_matches_js() {
        let c = PedalCal { axis: 1, raw_min: 1.0, raw_max: -1.0, deadzone_low: 0.0, deadzone_high: 0.0, gamma: 1.0, invert: false };
        assert!((c.apply(1.0) - 0.0).abs() < 1e-12);
        assert!((c.apply(-1.0) - 1.0).abs() < 1e-12);
        assert!((c.apply(0.0) - 0.5).abs() < 1e-12);
    }
}
