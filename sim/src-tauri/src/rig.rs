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
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};

use crate::wheel::{self, DeviceInfo, DeviceState, Wheel, AXES_PER_DEVICE, MAX_DEVICES};

/// Aligning-torque weight vs speed: 0 below 0.3 m/s, 1 from 1.8 m/s, smooth between.
///
/// The fade exists because the solver clamps forward speed at 3.0 m/s inside
/// the slip-angle calculation, so below walking pace any drift is a full-size
/// slip angle and a full-size torque that flips sign as the car wriggles. It
/// used to run to 4 m/s, which threw away every cue in the slow autocross
/// elements and left the paddock weightless. What the tyre model cannot
/// supply down there is supplied instead by the standstill scrub and the
/// caster/KPI jacking terms in the mixer, which are real and do not depend
/// on speed at all.
fn low_speed_fade(speed: f64) -> f64 {
    let x = ((speed - 0.3) / 1.5).clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// Normalised command through a gamma lift and a tanh soft knee.
///
/// A hard clamp throws away everything above 1.0, which on a small base is
/// precisely the part worth feeling: the torque peak and the fall-off past it.
/// The knee maps [knee, inf) onto [knee, 1) so that shape survives, compressed,
/// instead of flattening into a ceiling. `gamma` below 1 lifts everything under
/// full scale, which is what AC's `ff_post_process` GAMMA does and why a 5.5
/// N.m base feels weighty there and thin here.
fn compress(x: f64, gamma: f64, knee: f64) -> f64 {
    if x == 0.0 || !x.is_finite() {
        return 0.0;
    }
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let mut m = x.abs();
    if gamma > 0.0 && (gamma - 1.0).abs() > 1e-9 {
        m = m.powf(gamma);
    }
    let k = knee.clamp(0.0, 1.0);
    if m > k {
        let span = 1.0 - k;
        m = if span > 1e-9 { k + span * ((m - k) / span).tanh() } else { k };
    }
    sign * m
}

/// Set FSAE_RIG_TRACE=1 to print the rig's inputs and force feedback terms to stderr.
fn trace_on() -> bool {
    static T: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *T.get_or_init(|| std::env::var_os("FSAE_RIG_TRACE").is_some())
}

const RATE_HZ: f64 = 1000.0;
/// The longest step the vehicle model is ever asked to take. A tick that
/// arrives later than this (a stall) steps the car by this much and the rest
/// of the wall-clock time is dropped -- and counted, in `StatsOut::lost_ms`.
const MAX_DT: f64 = 0.01;
/// How long the rig drives on the last inputs before it decides the webview
/// has gone away (reload, exception, devtools pause) and holds the car with
/// the pedals up and the motor off.
const INPUT_STALE: Duration = Duration::from_millis(250);
/// How often to look for a base when none is open or the open one went quiet.
const RESCAN_EVERY: Duration = Duration::from_secs(2);
/// The window the controls panel's "us max" is the maximum over.
const TICK_MAX_WINDOW: Duration = Duration::from_secs(1);
/// A rim rate above this is a glitch (respawn, reacquire), not a driver.
const MAX_RIM_RATE_DEG_S: f64 = 5000.0;

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
    /// Launch control held: engine on the LC limiter, clutch out. Released,
    /// the clutch is dumped rather than fed in.
    pub launch: bool,
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
            // 46 deg of road wheel, which the measured rack puts at 179 deg of
            // rim (the nominal 4.411 ratio would say 203). Only reached if
            // the webview never sends a wheel config; it used to say 56, which
            // was a 14 deg x 4.0 car that has not existed for a long time.
            car_rim_half_deg: 123.5,
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
    /// Power-law lift on the normalised command: below 1.0 it raises the small
    /// on-centre torques toward the motor's usable range, the way AC's
    /// `ff_post_process` GAMMA does. 1.0 = off.
    pub gamma: f64,
    /// Where the soft knee starts, as a fraction of rated torque. Above it the
    /// command is compressed with a tanh so the torque peak and the fall-off
    /// past it both stay inside the motor instead of flattening into the clip.
    /// 1.0 = off (hard clip, the old behaviour).
    pub knee: f64,
    /// Coulomb resistance of a stationary tyre twisting against the ground,
    /// as a fraction of rated torque. Faded in as `low_speed_fade` fades out.
    pub park_friction: f64,
    /// Damping inside the end stop only, fraction of rated torque at 10 rad/s.
    /// Without it the stop is a ~6 Hz spring with nothing damping it.
    pub stop_damping: f64,
}

impl Default for FfbConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            // 5.5 N.m rated against ~15 N.m of rim torque at the peak: this
            // puts the TORQUE peak at the top of the motor, where the old 0.55
            // put 0.8 g there and clipped everything above it. See
            // `defaultGainFor` in wheelPresets.js.
            gain: 0.37,
            align_torque_gain: 1.0,
            road_texture_gain: 0.35,
            damping: 0.10,
            friction: 0.04,
            soft_lock_gain: 1.0,
            min_force: 0.0,
            max_force_nm: 5.5,
            invert: false,
            gamma: 0.75,
            knee: 0.6,
            park_friction: 0.10,
            stop_damping: 0.35,
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
    pub steer_slip_cap_deg: Option<f64>,
    pub steer_rate_speed_ref_mps: Option<f64>,
    pub steer_rate_speed_exp: Option<f64>,
    pub steering_ratio: Option<f64>,
    pub caster_deg: Option<f64>,
    pub kingpin_offset_trail_m: Option<f64>,
    pub diff_power_lock: Option<f64>,
    pub diff_coast_lock: Option<f64>,
    pub diff_preload_nm: Option<f64>,
    pub rack_efficiency: Option<f64>,
    pub torque_ratio: Option<f64>,
    pub mu_lat: Option<f64>,
    pub mu_long: Option<f64>,
    pub tire_load_sensitivity: Option<f64>,
    pub relax_length_m: Option<f64>,
    pub front_grip_factor: Option<f64>,
    pub gear_ratios: Option<Vec<f64>>,
    pub primary_reduction: Option<f64>,
    pub final_drive: Option<f64>,
    pub drivetrain_eff: Option<f64>,
    pub rev_limit_rpm: Option<f64>,
    pub idle_rpm: Option<f64>,
    pub idle_throttle_frac: Option<f64>,
    pub launch_rpm: Option<f64>,
    pub shift_time_s: Option<f64>,
    pub shift_reintro_s: Option<f64>,
    pub engine_inertia_kg_m2: Option<f64>,
    pub gearbox_inertia_kg_m2: Option<f64>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum RigCommand {
    /// `seq` is the webview's own counter, echoed back verbatim in every
    /// snapshot. Not a counter the rig keeps for itself: the webview can
    /// reload while the rig runs on, and a count of respawns the RIG has seen
    /// would then never agree with a page that started again from one.
    #[serde(rename_all = "camelCase")]
    Respawn { x: f64, y: f64, psi: f64, speed: f64, #[serde(default)] seq: u32 },
    Params(Box<ParamSet>),
    #[serde(rename_all = "camelCase")]
    Boundary { centre: Vec<[f64; 2]>, offset_m: f64 },
    ClearBoundary,
    Ffb(FfbConfig),
    Wheel(Box<WheelConfig>),
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
    /// Caster/KPI jacking: the only self-centring torque at a standstill.
    pub jacking: f64,
    pub soft_lock: f64,
    pub texture_nm: f64,
    pub clipped: bool,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DeviceOut {
    pub present: bool,
    /// The base enumerated with a force feedback actuator -- whether or not
    /// the effect started. This is what tells the webview the device it is
    /// steering by is a wheel and not a pad the driver picked from the
    /// controls panel; `present` alone does not, because the rig opens
    /// whatever was picked. See `Wheel::force_feedback`.
    pub force_feedback: bool,
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
    /// Longest tick in the last one to two seconds (`TICK_MAX_WINDOW`), so
    /// the number in the controls panel says something about NOW rather
    /// than about the worst thing that ever happened.
    pub tick_us_max: f64,
    /// Longest tick since the thread started.
    pub tick_us_max_all: f64,
    /// Late ticks, counted in periods: a tick that arrives 611 ms late is
    /// 611 ticks that did not happen, not one.
    pub overruns: u64,
    /// Wall-clock time the car did NOT simulate because a tick arrived more
    /// than `MAX_DT` late and the step was clamped. Zero on a healthy rig.
    pub lost_ms: f64,
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
    /// The respawn token of the last respawn this loop applied.
    ///
    /// The webview sets the car's pose locally the instant it asks for a
    /// respawn, then overwrites its whole state from whichever snapshot comes
    /// back next -- and for a frame or two that snapshot is still the one
    /// computed BEFORE the command was drained. Without a way to tell, the
    /// game reads the pre-respawn speed just after putting the car on the
    /// line, and the lap clock, which starts when the car moves, starts
    /// itself.
    ///
    /// The token comes FROM the webview and is echoed back unchanged, so the
    /// comparison is exact whatever either side has been through -- including
    /// a page reload against a rig that kept running.
    pub respawn_seq: u32,
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
    /// When the webview last sent a frame; the watchdog reads it.
    frame_at: Mutex<Instant>,
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
                frame_at: Mutex::new(Instant::now()),
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

    /// Stop the thread and wait for it: the wheel is released (torque zero,
    /// effect stopped, device unacquired) on the way out. Safe to call twice.
    pub fn stop(&self) -> RigStatus {
        self.shared.running.store(false, Ordering::SeqCst);
        if let Some(h) = self.thread.lock().unwrap().take() {
            let _ = h.join();
        }
        let mut st = self.shared.status.lock().unwrap();
        st.running = false;
        st.clone()
    }
}

impl Shared {
    /// Mark the inputs fresh. Every path that writes `input` must call this or
    /// the watchdog will hold the car.
    fn touch(&self) {
        *self.frame_at.lock().unwrap() = Instant::now();
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
    let alive = th.as_ref().map(|h| !h.is_finished()).unwrap_or(false);
    if alive && state.shared.running.load(Ordering::SeqCst) {
        return state.shared.status.lock().unwrap().clone();
    }
    // A thread that finished on its own (panic) leaves a stale handle.
    if let Some(h) = th.take() {
        let _ = h.join();
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
    state.stop()
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
    sh.touch();
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
    /// The base the driver asked for by name ("" = best guess). Compared
    /// against the next request, never against what actually opened: an
    /// open that fell back or failed must not be retried on every settings
    /// change.
    requested_device: String,
    next_rescan: Instant,
    /// A DirectInput scan in flight on its helper thread, if any. Polled
    /// with `try_recv` once per tick; never waited on.
    scan: Option<mpsc::Receiver<ScanResult>>,
    /// What the last scan found. The picker's list, and what a `SelectDevice`
    /// opens from without waiting for a fresh scan.
    found: Vec<wheel::Found>,
    ffb_cfg: FfbConfig,
    ffb: FfbMixer,
    device: DeviceState,
    device_present: bool,
    /// Consecutive ticks the base failed to read.
    lost_ticks: u32,
    /// The watchdog tripped last tick; used to log the transition once.
    held: bool,
    // for the display-only gradients the JS model carried
    ticks: u64,
    tick_us_sum: f64,
    /// Longest tick of the last completed `TICK_MAX_WINDOW`.
    tick_us_max: f64,
    /// Longest tick of the window in progress, and when it closes.
    win_max: f64,
    win_end: Instant,
    tick_us_max_all: f64,
    overruns: u64,
    lost_s: f64,
    boundary_hit: bool,
    /// The last respawn token the webview sent. See `Snapshot::respawn_seq`.
    respawn_seq: u32,
}

fn run(shared: Arc<Shared>, hwnd_raw: isize, ready: std::sync::mpsc::Sender<()>) {
    wheel::realtime_thread();

    let mut car = build(
        Fidelity::Bicycle,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    );
    car.reset(0.0, 0.0, 0.0, 0.0);

    let mut lp = Loop {
        car,
        assists: Assists::default(),
        boundary: None,
        etc: EtcMap::linear(),
        wheel: None,
        hwnd_raw,
        shared: shared.clone(),
        wheel_cfg: WheelConfig::default(),
        requested_device: String::new(),
        next_rescan: Instant::now() + RESCAN_EVERY,
        scan: None,
        found: Vec::new(),
        ffb_cfg: FfbConfig::default(),
        ffb: FfbMixer::default(),
        device: DeviceState::default(),
        device_present: false,
        lost_ticks: 0,
        held: false,
        ticks: 0,
        tick_us_sum: 0.0,
        tick_us_max: 0.0,
        win_max: 0.0,
        win_end: Instant::now() + TICK_MAX_WINDOW,
        tick_us_max_all: 0.0,
        overruns: 0,
        lost_s: 0.0,
        boundary_hit: false,
        respawn_seq: 0,
    };

    // The wheel is optional: no wheel means the webview steers. The first
    // scan is synchronous -- nothing is ticking yet and `rig_start` is
    // waiting for the answer -- every later one runs on a helper thread.
    lp.on_scan(wheel::scan());
    {
        let mut st = shared.status.lock().unwrap();
        st.running = true;
    }
    let _ = ready.send(());

    let period = Duration::from_secs_f64(1.0 / RATE_HZ);
    let mut next = Instant::now() + period;
    let mut last = Instant::now();
    let mut cmds: Vec<RigCommand> = Vec::new();

    while shared.running.load(Ordering::Relaxed) {
        let t0 = Instant::now();
        // The step is clamped so a stall cannot become one giant integration
        // step; what the clamp throws away is counted rather than vanishing.
        let elapsed = (t0 - last).as_secs_f64();
        let dt = elapsed.clamp(1e-4, MAX_DT);
        if elapsed > MAX_DT {
            lp.lost_s += elapsed - MAX_DT;
        }
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

        let mut input = *shared.input.lock().unwrap();
        // Watchdog. The webview is the only thing that unpauses the car; if
        // it stops talking, hold everything and take the torque off the rim.
        let stale = shared.frame_at.lock().unwrap().elapsed() > INPUT_STALE;
        if stale {
            input.paused = true;
            input.throttle = 0.0;
            input.brake = 0.0;
        }
        if stale != lp.held {
            lp.held = stale;
            eprintln!("rig: webview {}", if stale { "silent, holding the car" } else { "back" });
        }
        let shift_up = shared.shift_up.swap(0, Ordering::Relaxed);
        let shift_down = shared.shift_down.swap(0, Ordering::Relaxed);
        let cone_hits = shared.cone_hits.swap(0, Ordering::Relaxed);

        lp.poll_scan();
        lp.maybe_rescan();
        let snap = lp.tick(dt, &input, shift_up, shift_down, cone_hits);
        *shared.snapshot.lock().unwrap() = snap;

        // Pace. Sleep to just short of the deadline, spin the rest: Windows
        // sleeps are good to ~0.5 ms with timeBeginPeriod(1), not to 50 us.
        let spent = t0.elapsed().as_secs_f64() * 1e6;
        lp.ticks += 1;
        lp.tick_us_sum += spent;
        lp.win_max = lp.win_max.max(spent);
        lp.tick_us_max_all = lp.tick_us_max_all.max(spent);
        if t0 >= lp.win_end {
            lp.tick_us_max = lp.win_max;
            lp.win_max = 0.0;
            lp.win_end = t0 + TICK_MAX_WINDOW;
        }
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
            // One overrun per period missed, so a stall shows up as the
            // number of ticks it cost and not as a single late one.
            lp.overruns += 1 + ((now - next).as_secs_f64() * RATE_HZ) as u64;
            next = now + period;
        }
    }

    if let Some(w) = lp.wheel.as_mut() {
        w.close();
    }
    let mut st = shared.status.lock().unwrap();
    st.running = false;
}

/// What the scan thread sends back.
type ScanResult = Result<Vec<wheel::Found>, String>;

impl Loop {
    /// Take (or fail to take) a freshly opened base and say so in the status.
    fn set_wheel(&mut self, result: Result<Wheel, String>) {
        let mut st = self.shared.status.lock().unwrap();
        match result {
            Ok(w) => {
                st.wheel_present = true;
                st.ffb_active = w.ffb;
                st.wheel_name = w.name.clone();
                st.device_names = w.names.clone();
                st.wheel_error = if w.ffb { String::new() } else { "no force feedback actuator on this base (console mode, or a wheel DirectInput cannot drive); steering and pedals still work".into() };
                self.wheel = Some(w);
            }
            Err(e) => {
                st.wheel_present = false;
                st.ffb_active = false;
                st.wheel_name.clear();
                st.device_names.clear();
                st.wheel_error = e;
                self.wheel = None;
            }
        }
    }

    fn prefer(&self) -> Option<&str> {
        if self.requested_device.is_empty() {
            None
        } else {
            Some(self.requested_device.as_str())
        }
    }

    /// Has the open base stopped answering for long enough to give up on it?
    fn wheel_lost(&self) -> bool {
        self.wheel.is_some() && self.lost_ticks > (2.0 * RATE_HZ) as u32
    }

    /// Start a DirectInput scan on a helper thread, unless one is running.
    /// The rig thread never waits for it: `poll_scan` picks the answer up.
    fn request_scan(&mut self) {
        if self.scan.is_some() {
            return;
        }
        let (tx, rx) = mpsc::channel();
        let spawned = std::thread::Builder::new()
            .name("rig-scan".into())
            .spawn(move || {
                let _ = tx.send(wheel::scan());
            });
        if spawned.is_ok() {
            self.scan = Some(rx);
        }
    }

    /// Once per tick: a finished scan, if there is one. Non-blocking.
    fn poll_scan(&mut self) {
        let Some(rx) = self.scan.as_ref() else { return };
        match rx.try_recv() {
            Ok(res) => {
                self.scan = None;
                self.on_scan(res);
            }
            Err(mpsc::TryRecvError::Empty) => {}
            Err(mpsc::TryRecvError::Disconnected) => self.scan = None,
        }
    }

    /// A scan came back: refresh the picker's list and, with no base open
    /// (or one that has gone quiet), open from it. `Wheel::open_from` does
    /// nothing at all unless the list holds a candidate, so with a pad or
    /// nothing plugged in this costs the tick a string compare.
    fn on_scan(&mut self, res: ScanResult) {
        match res {
            Ok(found) => {
                self.found = found;
                self.shared.status.lock().unwrap().available = self.found.iter().map(wheel::Found::info).collect();
                if self.wheel.is_none() || self.wheel_lost() {
                    self.open_from_found();
                }
            }
            Err(e) => {
                self.shared.status.lock().unwrap().available.clear();
                if self.wheel.is_none() {
                    self.shared.status.lock().unwrap().wheel_error = e;
                }
            }
        }
    }

    /// Close whatever is open and open the best base in `found`.
    fn open_from_found(&mut self) {
        if let Some(mut w) = self.wheel.take() {
            w.close();
        }
        self.device = DeviceState::default();
        self.device_present = false;
        self.lost_ticks = 0;
        self.next_rescan = Instant::now() + RESCAN_EVERY;
        let result = Wheel::open_from(&self.found, self.hwnd_raw, self.prefer());
        self.set_wheel(result);
    }
    fn apply_command(&mut self, c: RigCommand) {
        match c {
            RigCommand::Respawn { x, y, psi, speed, seq } => {
                self.car.reset(x, y, psi, speed);
                // Stored, not counted: see `RigCommand::Respawn`.
                self.respawn_seq = seq;
                // Keep the rim angle and rate: zeroing them makes the next
                // tick see a 90,000 deg/s step and the damping term clips at
                // full rated torque. Only the transients belong to the run.
                self.ffb.kick = 0.0;
                self.ffb.phase = 0.0;
                self.ffb.friction_state = 0.0;
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
                self.wheel_cfg = *cfg;
                if want != self.requested_device {
                    self.requested_device = want;
                    self.reopen();
                }
            }
            RigCommand::SelectDevice { name } => {
                self.requested_device = name;
                self.reopen();
            }
        }
    }

    /// The driver picked a base. The picker showed the last scan's list, so
    /// open from that list right now -- the choice should take effect this
    /// tick, not after a scan -- and start a fresh scan behind it. If the
    /// immediate open failed (the device is gone, or held elsewhere) the
    /// scan's answer retries it.
    fn reopen(&mut self) {
        self.open_from_found();
        self.request_scan();
    }

    /// Hot-plug. With no base open, or one that has stopped answering for a
    /// couple of seconds, scan again every `RESCAN_EVERY`. The scan runs on
    /// its own thread; this only starts it.
    fn maybe_rescan(&mut self) {
        if !cfg!(windows) {
            return;
        }
        let now = Instant::now();
        if now < self.next_rescan {
            return;
        }
        self.next_rescan = now + RESCAN_EVERY;
        if self.wheel.is_none() || self.wheel_lost() {
            self.request_scan();
        }
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
                front_grip_factor => v.front_grip_factor,
                air_density_kg_m3 => v.aero.air_density, rsd_front => v.roll.rsd_front, h_roll_arm_m => v.roll.roll_arm_m,
                rc_front_m => v.roll.rc_front_m, rc_rear_m => v.roll.rc_rear_m,
                brake_torque_max_nm => v.brakes.max_torque_nm, brake_bias_front => v.brakes.bias_front,
                diff_power_lock => v.diff.power_lock, diff_coast_lock => v.diff.coast_lock,
                diff_preload_nm => v.diff.preload_nm,
                steer_lag_s => v.steering.lag_s, steering_ratio => v.steering.ratio,
                kingpin_offset_trail_m => v.steering.kingpin_offset_trail_m, rack_efficiency => v.steering.rack_efficiency,
            }
            if let Some(x) = p.max_steer_deg { v.steering.max_steer_rad = x.to_radians(); }
            if let Some(x) = p.steer_rate_deg_s { v.steering.rate_rad_s = x.to_radians(); }
            if let Some(x) = p.steer_accel_deg_s2 { v.steering.accel_rad_s2 = x.to_radians(); }
            if let Some(x) = p.steer_slip_cap_deg { v.steering.slip_cap_rad = x.to_radians(); }
            if let Some(x) = p.steer_rate_speed_ref_mps { v.steering.rate_speed_ref_mps = x; }
            if let Some(x) = p.steer_rate_speed_exp { v.steering.rate_speed_exp = x; }
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
            if let Some(x) = p.launch_rpm { e.launch_rpm = x; }
            if let Some(x) = p.shift_time_s { e.shift_time_s = x; }
            if let Some(x) = p.shift_reintro_s { e.shift_reintro_s = x; }
            if let Some(x) = p.engine_inertia_kg_m2 { e.crank_inertia_kg_m2 = x; }
            if let Some(x) = p.gearbox_inertia_kg_m2 { e.gearbox_inertia_kg_m2 = x; }
            if let Some(x) = p.wheel_inertia_rear_kg_m2 { e.wheel_side_inertia_kg_m2 = 2.0 * x; }
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
                    self.lost_ticks = 0;
                }
                None => {
                    self.device_present = false;
                    self.lost_ticks = self.lost_ticks.saturating_add(1);
                }
            }
        }
        let native = self.device_present && self.wheel_cfg.enabled;

        // ---- steering ----
        let (steer, rim_deg, half_lock) = if native {
            let wc = &self.wheel_cfg;
            let raw = self.device.axes.get(wc.steer_axis).copied().unwrap_or(0.0) as f64;
            let rim = raw * (wc.rotation_deg.max(1.0) / 2.0) - wc.centre_trim_deg;
            let max_road = self.car.params().steering.max_steer_rad.to_degrees().max(1e-6);
            // "match-car" means the rim turns the road wheels the way the real
            // rack does -- and the real rack is the MEASURED, progressive
            // table, not the nominal constant ratio. "scale-to-lock" instead
            // maps whatever rotation the base happens to be set to onto lock.
            let (road, half) = if wc.mapping == "match-car" {
                (
                    sim_core::vehicle::road_from_rim_deg(rim),
                    // Where the SOFT LOCK actually bites, not the rack's
                    // measured stop. The soft lock clamps the road wheel at
                    // the car's live `max_steer_rad`, and that is editable
                    // while driving; pinning the end stop to the constant
                    // 179 deg meant the two agreed only at the default 46 deg
                    // of lock. Anywhere else the wheel had a band of travel
                    // that steered nothing and resisted nothing.
                    sim_core::vehicle::rim_from_road_deg(max_road)
                        .abs()
                        .min(sim_core::vehicle::STEER_RIM_LOCK_DEG),
                )
            } else {
                let h = wc.rotation_deg.max(1.0) / 2.0;
                (rim / h * max_road, h)
            };
            let mut norm = road / max_road;
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
        // Launch control. Held at a standstill it sits the engine on the LC
        // limiter with the clutch out; dropped, the clutch is dumped. Ignored
        // while paused so a menu cannot leave it armed.
        self.car.powertrain_mut().set_launch(input.launch && !input.paused);
        self.assists.traction = input.traction;
        self.assists.abs = input.abs;
        // Traction control watches BOTH rear wheels: with a differential the
        // inside one is the one that lights up, and which side that is depends
        // on which way the car is turning.
        let throttle = self
            .assists
            .throttle(throttle_demand.clamp(0.0, 1.0), prev.kappa[RL].max(prev.kappa[RR]));
        // ABS watches both rears too, for the same reason traction control does.
        // The front is single-track so FL is FR, but RR is genuinely
        // independent -- its own load, its own half of the rear brake torque --
        // so a lightly loaded inner rear can lock while RL is fine, and
        // reading only RL meant ABS never saw it.
        let brake = self.assists.brake(
            brake_demand.clamp(0.0, 1.0),
            prev.kappa[FL],
            prev.kappa[RL].min(prev.kappa[RR]),
        );

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
        let mut tel = self.car.telemetry();

        // Rim torque through the LOCAL steering ratio. The solver reports it
        // through the nominal constant 4.411, but the real rack is
        // progressive: on centre the rim sees about 16% less torque per unit
        // kingpin moment, and in a hairpin about 30% more. Corrected here
        // rather than in the solver, so the vehicle model itself stays
        // identical between the desktop and browser builds.
        let rim_ratio = if native && self.wheel_cfg.mapping == "match-car" {
            sim_core::vehicle::road_per_rim(rim_deg) * self.car.params().steering.rack_efficiency
        } else {
            self.car.params().rim_torque_ratio()
        };
        tel.rim_torque_nm = tel.kingpin_torque_nm * rim_ratio;

        // ---- force feedback ----
        // Caster/KPI jacking: turning the wheel lifts that corner of the car,
        // so gravity pulls the rim back toward centre. About 1.2 N.m at the rim
        // at the car's 46 deg full lock, and 0.65 N.m at half lock -- small,
        // but at a standstill it is 100% of the return torque, because the
        // tyre's aligning torque has faded to nothing.
        //
        // (This comment said 0.6 N.m at full lock for a while. That was the
        // half-lock figure: the number was right for 23 deg and full lock is
        // 46. Recomputed from the code's own inputs -- fz_f 1270.3 N, arm
        // 5.435 mm, rim ratio road_per_rim(179) x 0.85 = 0.2404.)
        let jacking_nm = {
            let p = self.car.params();
            let fz_f = tel.fz[FL] + tel.fz[FR];
            let arm = p.steering.scrub_m * p.steering.kpi_rad.sin()
                + p.mechanical_trail() * p.steering.caster_rad.sin();
            -fz_f * arm * tel.steer_rad.sin() * rim_ratio
        };
        let feel = Feel {
            spin: ((tel.kappa[RL].max(tel.kappa[RR]) - 0.2).max(0.0) * 2.5).min(1.0),
            // Whichever wheel is most locked, front or either rear: the texture
            // is meant to tell the driver a wheel has stopped turning, and it
            // does not matter which one.
            lock: ((-tel.kappa[FL].min(tel.kappa[RL]).min(tel.kappa[RR]) - 0.2).max(0.0) * 2.5)
                .min(1.0),
            off_track: input.off_track && tel.speed > 2.0,
            cone_hits,
            jacking_nm,
        };
        let ffb_on = self.ffb_cfg.enabled && input.ffb_enabled && !input.paused;
        let ffb = self.ffb.mix(dt, &self.ffb_cfg, ffb_on, &tel, rim_deg, half_lock, native, &feel);
        if let Some(w) = self.wheel.as_mut() {
            if w.ffb {
                let _ = w.set_torque(if ffb_on { ffb.command as f32 } else { 0.0 });
            }
        }

        // DEBUG TRACE (FSAE_RIG_TRACE=1): what the rig sees at each boundary, twice a second.
        if trace_on() && self.ticks % 500 == 0 {
            eprintln!(
                "trace t={} native={} axes={:?} rim={:.1} half={:.1} steer={:+.3} thr={:.3} brk={:.3} | align={:+.2} damp={:+.2} fric={:+.2} stop={:+.2} tex={:.2} cmd={:+.3} clip={} | rimTq={:+.2} spd={:.1} kappa={:.2}/{:.2} gear={} rpm={:.0} bal={:+.2} bslip={:+.1}",
                self.ticks, native, &self.device.axes[..8], rim_deg, half_lock, steer, throttle_demand, brake_demand,
                ffb.align, ffb.damping, ffb.friction, ffb.soft_lock, ffb.texture_nm, ffb.command, ffb.clipped,
                tel.rim_torque_nm, tel.speed, tel.kappa[0], tel.kappa[2], tel.gear, tel.engine_rpm, tel.balance, tel.body_slip_deg
            );
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
                force_feedback: self.wheel.as_ref().map_or(false, |w| w.force_feedback),
                axes: self.device.axes,
                buttons: self.device.buttons,
                pov: self.device.pov,
                rim_deg,
                half_lock_deg: half_lock,
            },
            stats: StatsOut {
                ticks: self.ticks,
                tick_us_avg: if self.ticks > 0 { self.tick_us_sum / self.ticks as f64 } else { 0.0 },
                // The last completed window or the one in progress, whichever
                // is worse: a max over the last one to two seconds.
                tick_us_max: self.tick_us_max.max(self.win_max),
                tick_us_max_all: self.tick_us_max_all,
                overruns: self.overruns,
                lost_ms: self.lost_s * 1e3,
                rate_hz: RATE_HZ,
            },
            boundary_hit: self.boundary_hit,
            respawn_seq: self.respawn_seq,
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
    /// Caster/KPI jacking torque at the rim, model frame (left positive).
    /// Computed in `tick`, where the vehicle parameters are in scope.
    jacking_nm: f64,
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
        let raw_rate = ((rim_deg - self.rim_deg) / dt.max(1e-4)).clamp(-MAX_RIM_RATE_DEG_S, MAX_RIM_RATE_DEG_S);
        self.rim_rate_deg_s += (raw_rate - self.rim_rate_deg_s) * (dt / tau).min(1.0);
        self.rim_deg = rim_deg;
        if !on {
            self.kick = 0.0;
            return out;
        }
        let rated = cfg.max_force_nm.max(0.1);
        let rate = self.rim_rate_deg_s.to_radians();

        // Tyres. The model is left-positive; the wheel is clockwise-positive.
        // Faded out at walking pace: the solver clamps forward speed at
        // 3.0 m/s inside the slip-angle calculation, so below a few m/s any
        // drift or wheelspin is a full-size slip angle and a full-size torque
        // that flips sign as the car wriggles (+-5..10 N.m at 0.4-3 m/s on
        // the rig). Same curve as `forceFeedback.js` `lowSpeedFade`.
        let fade = low_speed_fade(tel.speed);
        out.align = -tel.rim_torque_nm * cfg.align_torque_gain * fade;
        // Damping: `damping` is the fraction of rated torque at 10 rad/s.
        out.damping = -cfg.damping * rated * (rate / 10.0);
        // Coulomb friction with a soft sign.
        let target = (rate / 0.3).tanh();
        self.friction_state += (target - self.friction_state) * (dt / 0.03).min(1.0);
        out.friction = -cfg.friction * rated * self.friction_state;
        // Standstill. A stationary tyre resists being twisted about the
        // kingpin by scrubbing its contact patch -- Coulomb, so it opposes
        // motion and not angle -- while caster and KPI lift the car as the
        // wheel turns, which is the only thing that returns the rim at rest.
        // Both fade in exactly as the tyre's aligning torque fades out, so
        // the paddock stops feeling weightless.
        let park = 1.0 - fade;
        if park > 1e-3 {
            out.friction -= cfg.park_friction * rated * self.friction_state * park;
            out.jacking = -feel.jacking_nm * park;
        }
        // End stops past the car's lock. Held OUT of the compressor and out of
        // the gain below: a stop that scales with a taste setting is not a
        // stop. Damped locally so it does not bounce at its own ~6 Hz.
        let over = rim_deg.abs() - half_lock;
        let mut stop = 0.0;
        if over > 0.0 {
            stop = -rim_deg.signum() * (over / 3.0).min(1.0) * cfg.soft_lock_gain
                - (cfg.stop_damping * rate / 10.0).clamp(-0.6, 0.6);
            out.soft_lock = stop * rated;
        }
        out.torque_nm = out.align + out.damping + out.friction + out.jacking + out.soft_lock;

        // To the motor. COMPRESS rather than clip. A hard clamp at the rated
        // torque erases the one cue this whole model exists to deliver -- the
        // rim going light as the front starts to slide. On a 5.5 N.m base the
        // old gain pinned the command at 1.0 from 0.8 g through the 1.5 g
        // torque peak and the fall-off beyond it, so the driver felt a wall
        // and then a slightly lighter wall. `gamma` lifts the small on-centre
        // torques; the tanh knee bends everything above `knee` into the
        // headroom that is left, so the peak and the drop stay readable.
        let base = (out.align + out.damping + out.friction + out.jacking) * cfg.gain / rated;
        out.clipped = base.abs() > 1.0;
        let mut cmd = compress(base, cfg.gamma, cfg.knee);
        // `f64::signum(0.0)` is +1, unlike Math.sign; a zero command must
        // stay zero or a gain of 0 pushes the rim to the right.
        if cfg.min_force > 0.0 && out.align.abs() > 1e-3 && cmd != 0.0 && cmd.abs() < cfg.min_force {
            cmd = cmd.signum() * cfg.min_force;
        }
        cmd = (cmd + stop).clamp(-1.0, 1.0);

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
        let feel = Feel { spin: 0.0, lock: 0.0, off_track: false, cone_hits: 0, jacking_nm: 0.0 };
        // A left turn: the model's rim torque is negative (tries to steer
        // back right); the wheel must be driven clockwise.
        let o = m.mix(0.001, &cfg, true, &tel_with_rim(-3.0), -20.0, 56.0, true, &feel);
        assert!(o.command > 0.0, "{o:?}");
    }

    #[test]
    fn texture_keeps_headroom() {
        let mut m = FfbMixer::default();
        let cfg = FfbConfig { gain: 1.0, ..Default::default() };
        let feel = Feel { spin: 1.0, lock: 0.0, off_track: false, cone_hits: 0, jacking_nm: 0.0 };
        for _ in 0..50 {
            let o = m.mix(0.001, &cfg, true, &tel_with_rim(-5.4), 0.0, 56.0, true, &feel);
            assert!(o.command.abs() <= 1.0);
            // The compressor leaves real headroom above a near-full base where
            // the old hard clip left almost none, so the texture is allowed to
            // be bigger. What must still hold is that it fits in what is left.
            assert!(o.texture_nm <= 0.6, "texture must fit in the headroom: {}", o.texture_nm);
        }
    }

    #[test]
    fn off_means_zero_and_no_stale_kick() {
        let mut m = FfbMixer::default();
        let cfg = FfbConfig::default();
        let feel = Feel { spin: 0.0, lock: 0.0, off_track: false, cone_hits: 1, jacking_nm: 0.0 };
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
        // Keep the watchdog fed the way the webview would.
        let feeder = {
            let shared = shared.clone();
            std::thread::spawn(move || {
                for _ in 0..90 {
                    // Traction control on: this checks that the LOOP runs and
                    // drives the car, not that a driver can dump full throttle
                    // from rest. On the measured torque curve, with two rear
                    // wheels and a differential between them, flooring it from
                    // a standstill simply spins them -- as it does in the car.
                    *shared.input.lock().unwrap() = RigInput { throttle: 1.0, auto_shift: true, traction: true, ..Default::default() };
                    shared.touch();
                    std::thread::sleep(Duration::from_millis(10));
                }
            })
        };
        feeder.join().unwrap();
        let snap = *shared.snapshot.lock().unwrap();
        shared.running.store(false, Ordering::SeqCst);
        th.join().unwrap();
        assert!(snap.state.x > 0.3, "car did not move: {:?}", snap.state);
        assert!(snap.stats.ticks >= 300, "ticks {}", snap.stats.ticks);
        assert!(snap.stats.tick_us_avg < 300.0, "tick too slow: {} us", snap.stats.tick_us_avg);
        assert!(snap.stats.overruns < snap.stats.ticks / 10, "overruns {}", snap.stats.overruns);
        assert!(snap.pt.engine_rpm > 2000.0);
    }

    /// The webview stops sending frames: the car must hold, not drive off on
    /// the last throttle it was given.
    #[test]
    fn watchdog_holds_the_car_when_the_webview_goes_silent() {
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
        shared.touch();
        // One frame's worth, then silence.
        std::thread::sleep(Duration::from_millis(600));
        let snap = *shared.snapshot.lock().unwrap();
        shared.running.store(false, Ordering::SeqCst);
        th.join().unwrap();
        // 250 ms of full throttle from rest in first is well under a metre;
        // 600 ms would be several.
        assert!(snap.state.x < 0.6, "car kept driving after the webview went silent: x = {}", snap.state.x);
        assert_eq!(snap.applied.throttle, 0.0);
        assert_eq!(snap.ffb.command, 0.0);
    }

    /// Every snapshot carries back the token of the respawn that produced it.
    ///
    /// The webview drops snapshots whose token is not its own, because
    /// otherwise a frame computed before the respawn command was drained puts
    /// the car's old speed back and the lap clock starts itself on the line.
    /// Echoed rather than counted, so a page that reloaded and started its
    /// tokens again still gets an exact answer. If this stops matching, that
    /// gate stops working and the bug comes back silently -- the car drives
    /// fine, the clock just lies.
    #[test]
    fn every_respawn_is_counted_into_the_snapshot() {
        let rig = Rig::new();
        let shared = rig.shared.clone();
        shared.running.store(true, Ordering::SeqCst);
        let (tx, rx) = std::sync::mpsc::channel();
        let th = {
            let shared = shared.clone();
            std::thread::spawn(move || run(shared, 0, tx))
        };
        rx.recv().unwrap();
        shared.touch();
        std::thread::sleep(Duration::from_millis(60));
        assert_eq!(shared.snapshot.lock().unwrap().respawn_seq, 0, "nothing has respawned yet");

        // Arbitrary tokens, including one that goes BACKWARDS -- a reloaded
        // page starts counting again and the rig must simply agree with it.
        for seq in [7u32, 8, 1] {
            shared
                .commands
                .lock()
                .unwrap()
                .push(RigCommand::Respawn { x: 0.0, y: 0.0, psi: 0.0, speed: 0.0, seq });
            shared.touch();
            std::thread::sleep(Duration::from_millis(60));
            assert_eq!(
                shared.snapshot.lock().unwrap().respawn_seq,
                seq,
                "respawn token {seq} was not echoed",
            );
        }
        shared.running.store(false, Ordering::SeqCst);
        th.join().unwrap();
    }

    /// Respawn with the rim off-centre must not thump the driver.
    #[test]
    fn respawn_keeps_the_rim_state() {
        let mut m = FfbMixer::default();
        let cfg = FfbConfig { gain: 1.0, damping: 1.0, ..Default::default() };
        let feel = Feel { spin: 0.0, lock: 0.0, off_track: false, cone_hits: 0, jacking_nm: 0.0 };
        for _ in 0..20 {
            let _ = m.mix(0.001, &cfg, true, &tel_with_rim(0.0), 90.0, 56.0, true, &feel);
        }
        // What apply_command(Respawn) does to the mixer.
        m.kick = 0.0;
        m.phase = 0.0;
        m.friction_state = 0.0;
        let o = m.mix(0.001, &cfg, true, &tel_with_rim(0.0), 90.0, 56.0, true, &feel);
        assert!(o.damping.abs() < 0.5, "damping spike after respawn: {o:?}");
        // And even a genuine 90 deg step is clamped to something a rim can do.
        let o = m.mix(0.001, &cfg, true, &tel_with_rim(0.0), 0.0, 56.0, true, &feel);
        assert!(o.command.abs() <= 1.0 && o.damping.is_finite(), "{o:?}");
    }

    #[test]
    fn zero_command_stays_zero_with_a_min_force() {
        let mut m = FfbMixer::default();
        let cfg = FfbConfig { gain: 0.0, min_force: 0.05, ..Default::default() };
        let feel = Feel { spin: 0.0, lock: 0.0, off_track: false, cone_hits: 0, jacking_nm: 0.0 };
        let o = m.mix(0.001, &cfg, true, &tel_with_rim(-3.0), 0.0, 56.0, true, &feel);
        assert_eq!(o.command, 0.0, "{o:?}");
    }

    #[test]
    fn pedal_calibration_matches_js() {
        let c = PedalCal { axis: 1, raw_min: 1.0, raw_max: -1.0, deadzone_low: 0.0, deadzone_high: 0.0, gamma: 1.0, invert: false };
        assert!((c.apply(1.0) - 0.0).abs() < 1e-12);
        assert!((c.apply(-1.0) - 1.0).abs() < 1e-12);
        assert!((c.apply(0.0) - 0.5).abs() < 1e-12);
    }
}
