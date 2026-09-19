// Run recorder: every run the driver makes, logged in full.
//
// Two things come out of a recorded run, and they are written from ONE
// sampling pass so they can never disagree:
//
//   telemetry.csv -- a Helios-canonical channel log. The first column is
//     `time_s`; every other header is either a canonical Helios channel id
//     (engine.rpm, imu.lat_g, gps.lat, ...) so the Logs module resolves it
//     against docs/channels.yaml with the same names, units and colours it
//     gives the real car's logger, or a `sim.*` id for the things only a
//     simulator knows (tyre slip angles, utilisation, the force-feedback
//     command, where the car was on the course).
//
//   run.json -- the manifest: who drove, what they drove, on what, with which
//     parameters and assists, plus the finished lap and sector times, the
//     discrete events, and the summary statistics. Enough on its own to fill
//     a leaderboard row without touching the telemetry.
//
// The pose channels (sim.pos_x / sim.pos_y / sim.yaw_deg and the wheel
// angles) are in the telemetry deliberately: the replay is driven from the
// same file the analysis reads, so there is no second recording path that can
// drift from the first.
//
// Sampling is at up to 100 Hz on the SIMULATION clock, not the frame clock: a
// run does not get shorter because the renderer stuttered, and the timestamps
// are the times the samples were actually taken. It is a CEILING, not a
// guarantee -- at most one row is written per simulation step, so a machine
// rendering at 60 fps logs at 60 Hz. `stats` carries the rate achieved.

import { CONE_PENALTY_S } from "./timing.js";
import { SDM26, rimFromRoadDeg } from "../vehicle/params.js";

export const SAMPLE_HZ = 100;
const SAMPLE_DT = 1 / SAMPLE_HZ;

/**
 * Corner frequency of the attitude-rate filter, in HERTZ.
 *
 * Read that unit carefully, because the obvious coefficient gets it wrong.
 * `k = dt * f` is a single pole at `f / (2*pi)`, so asking for 12 there
 * actually filtered at 1.9 Hz and reported genuine slalom-band roll rate at a
 * third of its size -- on a channel carrying a Helios-canonical id that an
 * engineer will plot against the car's real gyro. `ATTITUDE_ALPHA` below does
 * the conversion properly.
 *
 * 8 Hz is chosen to sit well above what a car on a six-point belt actually
 * does (roll and pitch motion lives at 1-3 Hz) and well below the numerical
 * noise that differentiating at 100 Hz produces. At 100 Hz it passes 1 Hz at
 * -0.1 dB and 3 Hz at -0.6 dB, so the trace is the motion, not the filter.
 */
const ATTITUDE_FILTER_HZ = 8;

/** The single-pole coefficient for a step of `dt` seconds. The exponential
 *  form is exact at any step, so a long frame cannot overshoot: it approaches
 *  1 rather than exceeding it. */
function attitudeAlpha(dt) {
  return 1 - Math.exp(-2 * Math.PI * ATTITUDE_FILTER_HZ * dt);
}

// Refuse to grow without bound: a stuck session should not eat the machine.
// 100 Hz for 90 minutes is 540k samples, far past any endurance run.
const MAX_SAMPLES = 540000;

/**
 * Column set. Order here is the order in the CSV.
 *
 *   id   the CSV header -- a Helios canonical channel id where one exists.
 *   dp   decimals written. Keeping this tight is most of the file size.
 *   get  reads one sample from the context object built by the game.
 */
const COLUMNS = [
  // ---- what the real car's logger would also have ------------------------
  { id: "gps.lat", dp: 7, get: (c) => c.lat },
  { id: "gps.lon", dp: 7, get: (c) => c.lon },
  { id: "gps.speed", dp: 3, get: (c) => c.speed },
  { id: "drivetrain.vehicle_speed", dp: 2, get: (c) => c.speed * 3.6 },
  { id: "engine.rpm", dp: 0, get: (c) => c.rpm },
  { id: "engine.gear", dp: 0, get: (c) => c.gear },
  { id: "engine.aps", dp: 2, get: (c) => c.pedal * 100 },
  { id: "engine.tps", dp: 2, get: (c) => c.plate * 100 },
  { id: "brake.driver_load", dp: 2, get: (c) => c.brake * 100 },
  { id: "brake.front_pressure", dp: 0, get: (c) => c.brake * c.brakeBiasFront * BRAKE_FULL_KPA },
  { id: "brake.rear_pressure", dp: 0, get: (c) => c.brake * (1 - c.brakeBiasFront) * BRAKE_FULL_KPA },
  // Derived from what the FRONT WHEELS are doing, through the car's measured
  // rack -- not read off the wheel's own sensor. Two reasons, both of which
  // were live bugs in a run you can still go and look at:
  //
  // SIGN. A wheel reports right-positive, and every other channel in this
  // file is left-positive -- the road-wheel angle, the steer input, the yaw
  // rate and the lateral g all agreed with each other and this one did not.
  // Overlaid in Logs it pointed the wrong way; measured against the car's own
  // front wheels it correlated -0.998, the same signal negated.
  //
  // COVERAGE. `input.rim.deg` is only ever set on a WHEEL profile, so on a
  // keyboard or a pad this channel was flat zero through a run where the car
  // was plainly being steered.
  //
  // Derived HERE rather than assembled by the caller so the two cannot drift:
  // the road-wheel angle is already in the context and this is a function of
  // it, which is exactly the kind of thing that goes wrong when it is
  // computed somewhere else and passed in.
  { id: "chassis.steering_angle", dp: 2,
    get: (c) => rimFromRoadDeg(SDM26.steering, c.roadWheelDeg ?? 0) },
  { id: "imu.lat_g", dp: 4, get: (c) => c.t.ayG },
  { id: "imu.long_g", dp: 4, get: (c) => c.t.axG },
  { id: "imu.yaw_rate", dp: 3, get: (c) => c.t.yawRateDegS },
  { id: "imu.roll_rate", dp: 3, get: (c) => c.rollRateDegS },
  { id: "imu.pitch_rate", dp: 3, get: (c) => c.pitchRateDegS },
  { id: "drivetrain.wheel_speed_fl", dp: 1, get: (c) => radsToRpm(c.car.wF) },
  { id: "drivetrain.wheel_speed_fr", dp: 1, get: (c) => radsToRpm(c.car.wF) },
  { id: "drivetrain.wheel_speed_rl", dp: 1, get: (c) => radsToRpm(c.wRL) },
  { id: "drivetrain.wheel_speed_rr", dp: 1, get: (c) => radsToRpm(c.wRR) },
  { id: "transmission.gear_ratio", dp: 4, get: (c) => c.gearRatio },

  // ---- the road wheel and the driver's hands -----------------------------
  { id: "sim.steer_input", dp: 4, get: (c) => c.steerInput },
  { id: "sim.road_wheel_deg", dp: 3, get: (c) => c.roadWheelDeg },
  { id: "sim.rim_torque_nm", dp: 3, get: (c) => c.t.rimTorqueNm },
  { id: "sim.kingpin_torque_nm", dp: 3, get: (c) => c.t.kingpinTorqueNm },
  { id: "sim.ffb_command", dp: 4, get: (c) => c.ffbCommand },
  { id: "sim.ffb_clipped", dp: 0, get: (c) => (c.ffbClipped ? 1 : 0) },

  // ---- chassis state -----------------------------------------------------
  { id: "sim.pos_x", dp: 3, get: (c) => c.car.X },
  { id: "sim.pos_y", dp: 3, get: (c) => c.car.Y },
  { id: "sim.yaw_deg", dp: 3, get: (c) => wrapDeg((c.car.psi * 180) / Math.PI) },
  { id: "sim.vel_long", dp: 3, get: (c) => c.car.u },
  { id: "sim.vel_lat", dp: 3, get: (c) => c.car.v },
  { id: "sim.body_slip_deg", dp: 3, get: (c) => c.t.bodySlipDeg },
  { id: "sim.roll_deg", dp: 3, get: (c) => c.t.rollDeg },
  { id: "sim.pitch_deg", dp: 3, get: (c) => c.t.pitchDeg },
  { id: "sim.wheel_angle_f", dp: 3, get: (c) => c.spinFront },
  { id: "sim.wheel_angle_r", dp: 3, get: (c) => c.spinRear },

  // ---- tyres --------------------------------------------------------------
  { id: "sim.slip_front_deg", dp: 3, get: (c) => c.t.slipF },
  { id: "sim.slip_rear_deg", dp: 3, get: (c) => c.t.slipR },
  { id: "sim.kappa_front", dp: 4, get: (c) => c.t.kappaF },
  { id: "sim.kappa_rear", dp: 4, get: (c) => c.t.kappaR },
  { id: "sim.kappa_rl", dp: 4, get: (c) => c.t.kappaRL ?? c.t.kappaR },
  { id: "sim.kappa_rr", dp: 4, get: (c) => c.t.kappaRR ?? c.t.kappaR },
  { id: "sim.util_front", dp: 4, get: (c) => c.t.utilF },
  { id: "sim.util_rear", dp: 4, get: (c) => c.t.utilR },
  { id: "sim.util_rl", dp: 4, get: (c) => c.t.utilRL ?? c.t.utilR },
  { id: "sim.util_rr", dp: 4, get: (c) => c.t.utilRR ?? c.t.utilR },
  { id: "sim.balance", dp: 4, get: (c) => c.t.balance },
  { id: "sim.trail_front_m", dp: 5, get: (c) => c.t.trailFm },

  // ---- loads and forces ---------------------------------------------------
  { id: "sim.fz_front_n", dp: 1, get: (c) => c.t.FzF },
  { id: "sim.fz_rear_n", dp: 1, get: (c) => c.t.FzR },
  { id: "sim.dfz_lat_front_n", dp: 1, get: (c) => c.t.dFzLatF },
  { id: "sim.dfz_lat_rear_n", dp: 1, get: (c) => c.t.dFzLatR },
  { id: "sim.downforce_n", dp: 1, get: (c) => c.t.downforceN },
  { id: "sim.drag_n", dp: 1, get: (c) => c.t.dragN },
  { id: "sim.drive_force_n", dp: 1, get: (c) => c.t.driveForceN },
  { id: "sim.diff_locked_nm", dp: 2, get: (c) => c.t.locked ?? 0 },

  // ---- where the car is on the course --------------------------------------
  { id: "sim.track_s_m", dp: 2, get: (c) => c.s },
  { id: "sim.lateral_error_m", dp: 3, get: (c) => c.lateral },
  { id: "sim.heading_error_deg", dp: 3, get: (c) => c.headingErrorDeg },
  { id: "sim.curvature", dp: 6, get: (c) => c.curvature },
  { id: "sim.on_track", dp: 0, get: (c) => (c.onTrack ? 1 : 0) },

  // ---- timing, carried as channels so every plot can be cut by lap ---------
  //
  // `system.beacon` is the one channel here written for another program to
  // read. Helios's lap detection prefers a beacon over inferring a start/finish
  // line from the GPS trace, and a beacon is unambiguous: the simulator knows
  // exactly where the line is and exactly when the car crossed it. Writing one
  // means Helios's lap table is the same lap table the driver saw on the HUD,
  // rather than a second opinion that disagrees by a tenth.
  //
  // A transponder is detected on its RISING edge, so the channel must start
  // LOW and the first pulse must be the car crossing the line -- not the
  // recorder opening. Pulsing on row 0 made the first sample already high,
  // which is no edge at all: Helios then treated everything up to the first
  // lap completion as an untrusted out lap, and an autocross run -- one lap,
  // one crossing -- produced no lap table whatsoever.
  { id: "system.beacon", dp: 0, get: (c) => c.beacon },
  { id: "sim.lap", dp: 0, get: (c) => c.lap },
  { id: "sim.lap_time_s", dp: 3, get: (c) => c.lapTime },
  { id: "sim.sector", dp: 0, get: (c) => c.sector },
  { id: "sim.cones_lap", dp: 0, get: (c) => c.cones },
  { id: "sim.off_course_lap", dp: 0, get: (c) => c.offCourse },
  { id: "sim.penalty_s", dp: 2, get: (c) => c.penaltyS },

  // ---- the live delta, exactly as the driver saw it ------------------------
  //
  // Recomputing this after the fact is not the same thing. The delta on the
  // HUD was measured against whatever reference was loaded AT THE TIME -- the
  // driver's own best so far that session, or a lap handed in by the launcher
  // -- and a replay opened tomorrow has no way to know which. The number the
  // driver was reacting to is a fact about the run, so it is logged like any
  // other channel, and `manifest.reference` says what it was against.
  //
  // `sim.delta_valid` is not decoration: a delta of exactly 0.000 means "dead
  // level with the reference", and without a companion flag it is
  // indistinguishable from "there was no reference". Staging, an out lap and
  // the first lap of a session are all genuinely blank.
  { id: "sim.delta_s", dp: 3, get: (c) => c.deltaS },
  { id: "sim.delta_valid", dp: 0, get: (c) => (c.deltaValid ? 1 : 0) },

  // ---- assists, so a time is always readable in context --------------------
  { id: "sim.traction_control", dp: 0, get: (c) => (c.assists.traction ? 1 : 0) },
  { id: "sim.abs", dp: 0, get: (c) => (c.assists.abs ? 1 : 0) },
  { id: "sim.auto_shift", dp: 0, get: (c) => (c.assists.autoShift ? 1 : 0) },
  { id: "sim.launch_held", dp: 0, get: (c) => (c.launch ? 1 : 0) },
  { id: "sim.clutch_slip_rpm", dp: 0, get: (c) => c.clutchSlipRpm },
  { id: "sim.shifting", dp: 0, get: (c) => (c.shifting ? 1 : 0) },
];

/** A brake line at full pedal, for the two pressure channels. The model has
 *  no hydraulics -- it has a peak brake torque -- so this is a scale that
 *  makes the logged trace read in the units a brake engineer expects, not a
 *  measurement. Documented in the manifest as `derived`. */
const BRAKE_FULL_KPA = 6000;

/** Canonical CSV header order, exported so a reader can be built from it. */
export const CHANNEL_IDS = COLUMNS.map((c) => c.id);

/** Channels whose values are derived rather than modelled, called out in the
 *  manifest so nobody mistakes a plausible trace for a measured one. */
export const DERIVED_CHANNELS = {
  "gps.lat": "projected from course XY onto the venue datum",
  "gps.lon": "projected from course XY onto the venue datum",
  "brake.front_pressure": `pedal x bias x ${BRAKE_FULL_KPA} kPa; the model has no hydraulics`,
  "brake.rear_pressure": `pedal x bias x ${BRAKE_FULL_KPA} kPa; the model has no hydraulics`,
  "drivetrain.wheel_speed_fl": "single-track front: FL and FR are the same wheel",
  "drivetrain.wheel_speed_fr": "single-track front: FL and FR are the same wheel",
  "imu.roll_rate":
    "differentiated from a roll ANGLE that is itself lateral g times a fixed " +
    "gradient -- there is no suspension dynamics behind it, so treat the " +
    "magnitude as indicative and do not compare it to the car's gyro",
  "imu.pitch_rate":
    "differentiated from a pitch ANGLE that is itself longitudinal g times a " +
    "fixed gradient; same caveat as the roll rate",
};

function radsToRpm(w) { return (w * 60) / (Math.PI * 2); }

function wrapDeg(d) {
  let x = d % 360;
  if (x > 180) x -= 360;
  if (x <= -180) x += 360;
  return x;
}

/**
 * Local-tangent-plane projection of the course's metres into real degrees.
 *
 * The courses are traced geometry in a local XY frame, not GPS traces, so
 * there is no true fix to log. But `gps.lat` / `gps.lon` are what Helios's lap
 * detection defaults to and what the GPS-track widget draws, and a lap is the
 * same shape wherever you put it. So each course carries a datum -- the real
 * place it is driven -- and the log is referenced to it. The result is a
 * correct-scale, correct-shape trace at a truthful location, and `run.json`
 * records the datum so nobody mistakes it for a receiver fix.
 */
const EARTH_R = 6378137;

export function makeGeoProjection(datum) {
  const lat0 = (datum.lat * Math.PI) / 180;
  const mPerDegLat = (Math.PI / 180) * EARTH_R;
  const mPerDegLon = mPerDegLat * Math.cos(lat0);
  const rot = ((datum.bearingDeg ?? 0) * Math.PI) / 180;
  const cr = Math.cos(rot), sr = Math.sin(rot);
  return (x, y) => {
    // Course +X is east and +Y is north once the datum's bearing is applied.
    const e = x * cr - y * sr;
    const n = x * sr + y * cr;
    return { lat: datum.lat + n / mPerDegLat, lon: datum.lon + e / mPerDegLon };
  };
}

/** Where each course actually is. The autocross and endurance pads are the
 *  2026 FSAE Michigan layouts at Michigan International Speedway; `mis` is the
 *  speedway itself. */
export const TRACK_DATUMS = {
  autocross: { lat: 42.07152, lon: -84.24089, bearingDeg: 0, name: "FSAE Michigan autocross pad" },
  endurance: { lat: 42.06985, lon: -84.24515, bearingDeg: 0, name: "FSAE Michigan endurance pad" },
  mis: { lat: 42.06556, lon: -84.24139, bearingDeg: 0, name: "Michigan International Speedway" },
};

export function datumFor(trackId) {
  return TRACK_DATUMS[trackId] ?? TRACK_DATUMS.autocross;
}

/**
 * One recorded run.
 *
 * Lifecycle: `new Recorder(meta)` when a drive starts, `tick(dt, ctx)` every
 * simulation frame, `event(...)` whenever something discrete happens,
 * `recordLap()` as each lap is scored, and `finish()` to freeze it.
 * `toCsv()` and `toManifest()` produce the two artifacts.
 */
export class Recorder {
  constructor(meta) {
    this.meta = meta;
    this.project = makeGeoProjection(meta.datum ?? datumFor(meta.track));
    this.columns = COLUMNS.map(() => []);
    this.times = [];
    this.events = [];
    this.laps = [];
    this.startedAtMs = Date.now();
    this.simTime = 0;   // total simulated seconds in this run
    // How far into the CURRENT frame the rest of the game already is.
    //
    // `Game.update()` runs the physics and the timing before it ticks the
    // sampler, so for most of a frame `simTime` is still the END of the
    // previous one. Anything stamped with it -- a lap's start, a cone, a shift
    // -- therefore landed one frame early: measured at exactly 1/120 s on a
    // 120 Hz run, which is 8 ms of lie on every event and, worse, made
    // `laps[].startedAtS` disagree with the telemetry's own
    // `time_s - sim.lap_time_s` by the same amount. A reference lap loaded
    // from the archive was then a frame fast everywhere.
    this.frameDt = 0;
    /** `{ lapS, label, source }` for whatever the live delta was chasing, set
     *  by `setReference()` when the game loads or adopts one. */
    this.reference = null;
    this.acc = 0;       // fixed-rate sampler accumulator
    this.samples = 0;
    this.truncated = false;
    this.finished = false;
    this.finishedReason = null;
    // The two attitude rates are differentiated here rather than in the
    // physics: the model produces angles, a logger reports rates.
    this._prevRoll = 0;
    this._prevPitch = 0;
    this._rollRate = 0;
    this._pitchRate = 0;
    // Rows left to hold the lap beacon high. One row would do -- the detector
    // takes the rising edge -- but a few make the pulse visible when the trace
    // is plotted, and the edge is the same either way.
    this._beacon = 0;
    // Running extremes and integrals, so the manifest carries the statistics
    // without a second pass over tens of thousands of rows.
    this._peak = {
      speed: 0, rpm: 0, latG: 0, longG: 0, brakeG: 0,
      rimTorque: 0, slipF: 0, slipR: 0, bodySlip: 0,
    };
    this._sum = { speed: 0, throttle: 0, brake: 0, n: 0 };
    this._distanceM = 0;
    this._fullThrottleS = 0;
    this._brakingS = 0;
    this._offTrackS = 0;
    this._ffbClippedS = 0;
    /**
     * Seconds of steering credited to each thing that can steer, from
     * `Input.steerSource` -- observed, not declared. See `detectInputClass`.
     *
     * Time weighted rather than counted in frames so a run logged on a 60 Hz
     * laptop and one on a 240 Hz rig describe the same driving.
     */
    this._steerS = { wheel: 0, pad: 0, key: 0, mouse: 0 };
  }

  /**
   * The car crossed the start/finish line.
   *
   * Called when the lap clock starts and again as each lap closes, so the
   * beacon has a genuine low-to-high edge at every crossing and none anywhere
   * else. The recorder opening is not a crossing: a driver can sit staged on
   * the line for as long as they like, and a pulse there would both fail to
   * register (no preceding low sample) and, if it did, pad lap one by however
   * long they waited.
   */
  markLine() {
    this._beacon = 3;
  }

  /**
   * Note what the live delta is being measured against.
   *
   * Called whenever the delta timer adopts a reference -- on load from the
   * archive, and again when a quicker lap in this session replaces it. The
   * LAST one set is what the manifest carries, which is the one the driver
   * finished the run chasing.
   */
  setReference(ref) {
    this.reference = ref
      ? { lapS: ref.lapS ?? null, label: ref.label ?? null, source: ref.source ?? null }
      : null;
  }

  /** The run clock as the rest of this frame sees it. See `frameDt`. */
  get now() { return this.simTime + this.frameDt; }

  /** A discrete thing that happened, stamped with the run clock. */
  event(kind, detail = {}) {
    if (this.finished) return;
    if (this.events.length > 20000) return;
    this.events.push({ t: round(this.now, 3), kind, ...detail });
  }

  /**
   * Advance the run clock by one simulation step and, whenever the fixed
   * sample interval elapses, write a row.
   *
   * @param dt    simulated seconds this step (paused frames never get here)
   * @param ctx   everything the columns read; built by `Game.recorderContext`
   * @param last  this is the run's final step: a row is written for it
   *              whatever the sampler's accumulator says
   */
  tick(dt, ctx, { last = false } = {}) {
    if (this.finished || !(dt > 0)) return;
    // From here on `simTime` IS the frame's time, so the offset goes away.
    this.frameDt = 0;
    // The first row is the state the run starts from. Without it the log
    // begins a frame or two in, which is invisible in a plot and wrong in a
    // launch.
    if (this.samples === 0) this.writeRow(ctx, 0);
    this.simTime += dt;
    this.accumulate(dt, ctx);
    this.acc += dt;
    // The last step of a run is the one the finish happened on, and what it
    // carries has nowhere else to go: `recordLap` has just raised the beacon
    // for the finish line, and the only rows that could show it are the ones
    // after this. Leaving that to the accumulator made the finish pulse a
    // coin toss -- 9 of 13 autocross logs had it -- and then, once the run
    // was banked before this tick ran, none did. A beacon with one edge is a
    // lap Helios cannot find the end of.
    if (!last && this.acc < SAMPLE_DT) return;
    if (last) this.acc = 0;
    else this.acc -= SAMPLE_DT;
    // A long frame does NOT get the rows it owes back-filled. There is only
    // one car state for that whole span, and writing it several times over
    // would put samples in the log at instants the simulator never evaluated.
    // A gap is the truth, and both the replay and Helios interpolate by time
    // rather than by row, so a gap costs nothing but honesty.
    if (this.acc > SAMPLE_DT) this.acc = 0;
    // Stamped with the simulation clock, not with `row * interval`: the
    // sampler fires on the first step after the interval elapses, so the row
    // can be up to one frame past its nominal instant. Labelling it with the
    // nominal time would put every sample a few milliseconds early -- 14 cm of
    // position error at 20 m/s, which is exactly the kind of quiet lie that
    // makes telemetry untrustworthy.
    this.writeRow(ctx, this.simTime);
  }

  /** Integrals and extremes, updated every step rather than every sample so a
   *  100 Hz log still reports a true distance and a true peak. */
  accumulate(dt, ctx) {
    const t = ctx.t;
    const sp = ctx.speed;
    this._distanceM += sp * dt;
    if (ctx.plate > 0.97) this._fullThrottleS += dt;
    if (ctx.brake > 0.05) this._brakingS += dt;
    if (!ctx.onTrack && sp > 2) this._offTrackS += dt;
    if (ctx.ffbClipped) this._ffbClippedS += dt;
    if (this._steerS[ctx.steerSource] != null) this._steerS[ctx.steerSource] += dt;
    const p = this._peak;
    if (sp > p.speed) p.speed = sp;
    if (ctx.rpm > p.rpm) p.rpm = ctx.rpm;
    if (Math.abs(t.ayG) > p.latG) p.latG = Math.abs(t.ayG);
    if (t.axG > p.longG) p.longG = t.axG;
    if (-t.axG > p.brakeG) p.brakeG = -t.axG;
    if (Math.abs(t.rimTorqueNm) > p.rimTorque) p.rimTorque = Math.abs(t.rimTorqueNm);
    if (Math.abs(t.slipF) > p.slipF) p.slipF = Math.abs(t.slipF);
    if (Math.abs(t.slipR) > p.slipR) p.slipR = Math.abs(t.slipR);
    if (Math.abs(t.bodySlipDeg) > p.bodySlip) p.bodySlip = Math.abs(t.bodySlipDeg);
    const s = this._sum;
    s.speed += sp * dt;
    s.throttle += ctx.plate * dt;
    s.brake += ctx.brake * dt;
    s.n += dt;
  }

  writeRow(ctx, t) {
    if (this.samples >= MAX_SAMPLES) { this.truncated = true; return; }
    // Differentiated against the real gap to the previous row, which is a
    // sample interval most of the time and is not after a long frame.
    const span = this.samples > 0 ? Math.max(1e-4, t - this.times[this.samples - 1]) : SAMPLE_DT;
    // Roll and pitch here are algebraic scalings of instantaneous
    // acceleration, with no suspension dynamics and no filtering of their own.
    // Differentiating that raw at 100 Hz multiplies every gear shift, brake
    // application and cone strike by a hundred -- it produced 164 deg/s of
    // "pitch rate" on a car whose pitch angle spans two degrees. A single-pole
    // filter above the band a body actually moves in keeps the trace readable
    // without shrinking the motion; `DERIVED_CHANNELS` says what it is either
    // way.
    const roll = ctx.t.rollDeg ?? 0;
    const pitch = ctx.t.pitchDeg ?? 0;
    if (this.samples === 0) {
      // Seed from the first row rather than from zero. A run that opens with
      // the car already leaned on -- a replay resumed mid-corner, a recorder
      // opened after the flag -- would otherwise report the whole standing
      // angle as if it had happened in one sample interval.
      this._prevRoll = roll;
      this._prevPitch = pitch;
    }
    const k = attitudeAlpha(span);
    this._rollRate += ((roll - this._prevRoll) / span - this._rollRate) * k;
    this._pitchRate += ((pitch - this._prevPitch) / span - this._pitchRate) * k;
    ctx.rollRateDegS = this._rollRate;
    ctx.pitchRateDegS = this._pitchRate;
    this._prevRoll = roll;
    this._prevPitch = pitch;

    const geo = this.project(ctx.car.X, ctx.car.Y);
    ctx.lat = geo.lat;
    ctx.lon = geo.lon;

    // The first sample of a run is ALWAYS low, whatever is pending. A rising
    // edge needs something to rise from, and a driver who floors it on the
    // first frame would otherwise get a beacon that is high from row 0 -- no
    // edge, and Helios sees no laps. A pulse raised on that frame simply
    // starts on the next row, ten milliseconds later.
    ctx.beacon = this.samples > 0 && this._beacon > 0 ? 1 : 0;
    if (ctx.beacon) this._beacon--;

    this.times.push(t);
    for (let i = 0; i < COLUMNS.length; i++) {
      const v = COLUMNS[i].get(ctx);
      this.columns[i].push(Number.isFinite(v) ? v : 0);
    }
    this.samples++;
  }

  /** Record a completed lap exactly as the Timing module scored it. */
  recordLap(entry, sectorSplits) {
    this.markLine();
    this.laps.push({
      lap: entry.lap,
      raw: round(entry.raw, 3),
      cones: entry.cones,
      off: entry.off,
      penaltyS: round(entry.cones * CONE_PENALTY_S, 3),
      total: round(entry.total, 3),
      // Left the course, so it is not a time -- see `timing.js`, which is
      // stricter than the rulebook here and says why. The lap is recorded in
      // full; everything that ranks reads `valid` and nothing else has to
      // know the reason.
      valid: entry.valid !== false,
      // `null` for a sector that was never timed -- the car's course distance
      // jumped over the boundary. `round` turns a null into 0, and a 0.000 s
      // sector is a far worse answer than an admitted gap.
      sectors: (sectorSplits ?? []).map((s) => (s == null ? null : round(s, 3))),
      // `now`, not `simTime`: the lap ended during THIS frame, and the rows
      // are stamped with the clock after the frame is applied.
      startedAtS: round(this.now - entry.raw, 3),
    });
    this.event("lap", { lap: entry.lap, total: round(entry.total, 3) });
  }

  finish(reason = "finished") {
    if (this.finished) return;
    this.finishedReason = reason;
    this.event("end", { reason });
    this.finished = true;
  }

  get durationS() { return this.samples > 0 ? this.times[this.samples - 1] : 0; }

  /**
   * Is there enough here to be worth keeping?
   *
   * A run has to have a TIME on it. A lap is what makes a run a run -- it is
   * the thing the board ranks, the thing a replay is opened to look at, and
   * the thing the driver is here for. Everything else is an aborted attempt:
   * spun on the first corner, pressed restart, went for a look round the
   * paddock. Those used to be filed exactly like a real run, and at roughly
   * 280 bytes a row a few minutes of not-a-run is a megabyte that somebody has
   * to scroll past and eventually delete.
   *
   * The length floors stay as a second gate, because a "lap" on a course that
   * starts and finishes in the same place can be scored a few metres in.
   *
   * Consequence worth knowing: a FREE-ROAM venue (MIS) never completes a lap,
   * so driving round it no longer leaves a run behind. That is the intended
   * reading of "if there is no time, do not store it" -- free roam is not a
   * timed run -- but it is a behaviour change, not a bug.
   */
  get worthSaving() {
    return this.laps.length > 0 && this.samples >= SAMPLE_HZ * 3 && this._distanceM > 15;
  }

  /** Why `worthSaving` said no, for a message the driver can act on. */
  get notSavedReason() {
    if (this.worthSaving) return null;
    if (this.laps.length === 0) return "no lap completed";
    if (this.samples < SAMPLE_HZ * 3) return "too short";
    return "went nowhere";
  }

  /** The whole log, as a Helios-readable CSV string. */
  toCsv() {
    const header = "time_s," + CHANNEL_IDS.join(",");
    // Column-major to row-major once. Across ~70 columns and tens of
    // thousands of rows this join is the whole cost of saving a run, so it is
    // a single array join rather than repeated string concatenation.
    const rows = new Array(this.samples);
    const dps = COLUMNS.map((c) => c.dp);
    const cols = this.columns;
    const nCols = cols.length;
    for (let r = 0; r < this.samples; r++) {
      const cells = new Array(nCols + 1);
      // Microseconds. The sampler is on the sim clock, so a row lands a
      // fraction of a frame past its nominal instant and two decimals would
      // quantise that away -- which is the error this is here to avoid.
      cells[0] = this.times[r].toFixed(6);
      for (let c = 0; c < nCols; c++) cells[c + 1] = fmtNum(cols[c][r], dps[c]);
      rows[r] = cells.join(",");
    }
    return header + "\n" + rows.join("\n") + "\n";
  }

  /** Summary statistics, from the running accumulators. */
  stats() {
    const n = this._sum.n || 1;
    // Two different questions, and they routinely have two different answers:
    // the best lap is the best SCORED lap, because that is what an event
    // scores, while the quickest raw lap might be one the driver mowed down
    // five cones on. `bestLapRawS` is the raw time OF the best lap -- pairing
    // the best scored time with the quickest raw time from somewhere else in
    // the run reads as one lap and is not one, and the difference between them
    // reads as a penalty that was never applied.
    // Only valid laps may be a best. A run of nothing but off-course laps has
    // no best lap at all, which is exactly right: it did not set a time, and
    // downstream that makes it unrankable without anything else having to
    // know about off courses.
    const scoring = this.laps.filter((l) => l.valid !== false);
    const bestLap = scoring.reduce((b, l) => (b == null || l.total < b.total ? l : b), null);
    const fastestRaw = scoring.reduce((b, l) => (b == null || l.raw < b.raw ? l : b), null);
    // Theoretical best: the quickest each sector was ever driven, added up.
    // It is the number that tells a driver what the car already did, in
    // pieces, on a lap nobody has yet put together.
    const nSectors = this.laps.reduce((m, l) => Math.max(m, l.sectors.length), 0);
    const bestSectors = [];
    for (let i = 0; i < nSectors; i++) {
      let best = null;
      for (const l of scoring) {
        const v = l.sectors[i];
        // `null` is a sector that was never timed, which is why this reads
        // `!= null` and not a truthiness test: a genuine 0 would be absurd,
        // but so would silently treating an untimed sector as the quickest.
        if (v != null && (best == null || v < best)) best = v;
      }
      bestSectors.push(best);
    }
    const complete = bestSectors.length > 0 && bestSectors.every((s) => s != null);
    return {
      durationS: round(this.durationS, 3),
      distanceM: round(this._distanceM, 1),
      laps: this.laps.length,
      bestLapS: bestLap ? bestLap.total : null,
      bestLapRawS: bestLap ? bestLap.raw : null,
      bestLapNumber: bestLap ? bestLap.lap : null,
      bestLapCones: bestLap ? bestLap.cones : null,
      /** The outright quickest lap ignoring penalties, whichever lap that was.
       *  Worth knowing -- it is the pace the car has in it -- but it is not
       *  the time that scores. */
      fastestRawLapS: fastestRaw ? fastestRaw.raw : null,
      fastestRawLapNumber: fastestRaw ? fastestRaw.lap : null,
      bestSectors: bestSectors.map((s) => (s == null ? null : round(s, 3))),
      theoreticalBestS: complete ? round(bestSectors.reduce((a, b) => a + b, 0), 3) : null,
      totalCones: this.laps.reduce((a, l) => a + l.cones, 0),
      totalOffCourse: this.laps.reduce((a, l) => a + l.off, 0),
      /** Laps thrown out for leaving the course. `laps` counts them; nothing
       *  that ranks does. */
      invalidLaps: this.laps.length - scoring.length,
      peakSpeedMps: round(this._peak.speed, 3),
      peakSpeedKph: round(this._peak.speed * 3.6, 2),
      peakRpm: Math.round(this._peak.rpm),
      peakLatG: round(this._peak.latG, 3),
      peakAccelG: round(this._peak.longG, 3),
      peakBrakeG: round(this._peak.brakeG, 3),
      peakRimTorqueNm: round(this._peak.rimTorque, 3),
      peakSlipFrontDeg: round(this._peak.slipF, 2),
      peakSlipRearDeg: round(this._peak.slipR, 2),
      peakBodySlipDeg: round(this._peak.bodySlip, 2),
      avgSpeedMps: round(this._sum.speed / n, 3),
      avgThrottle: round(this._sum.throttle / n, 4),
      avgBrake: round(this._sum.brake / n, 4),
      fullThrottleFrac: round(this._fullThrottleS / n, 4),
      brakingFrac: round(this._brakingS / n, 4),
      offTrackS: round(this._offTrackS, 2),
      ffbClippedFrac: round(this._ffbClippedS / n, 4),
    };
  }

  toManifest() {
    return {
      // 3: a lap that left the course has NO TIME. `laps[].valid` says so,
      // and `bestLapS`, `bestLapRawS`, `bestSectors` and `theoreticalBestS`
      // are computed from the valid laps only.
      //
      // Version 2 scored an off course as +10 s and let the lap stand, so its
      // best lap and its sector bests can both be times that left the course.
      // Nothing about them looks wrong -- they are ordinary times with a
      // plausible number added -- which is why the version has to move: a
      // reader cannot tell by inspection and must not have to guess.
      //
      // 2: `laps[].sectors` are DURATIONS and the final sector is present, and
      // `stats.bestLapRawS` is the raw time of the best SCORED lap.
      //
      // In version 1 the sectors were cumulative splits with the last one
      // missing, so `theoreticalBestS` was a sum of running totals -- on a real
      // run it came out 52% SLOWER than a lap actually driven -- and
      // `bestLapRawS` was the quickest raw lap, which is often a different lap
      // from the best scored one. Both are numbers that look perfectly
      // reasonable and are wrong, so a reader has to be able to tell the two
      // formats apart rather than trusting the fields. Helios hides the
      // affected columns on a version 1 run instead of drawing a lie.
      formatVersion: 3,
      producer: "fsae-sim",
      ...this.meta,
      startedAt: new Date(this.startedAtMs).toISOString(),
      endedAt: new Date().toISOString(),
      finishedReason: this.finishedReason ?? "abandoned",
      // What `sim.delta_s` was measured against, so a replay can say so rather
      // than leaving the trace unattributed.
      reference: this.reference,
      sampleRateHz: SAMPLE_HZ,
      // What was actually achieved. The sampler writes at most one row per
      // simulation step, so on a machine rendering below 100 fps the log is at
      // the frame rate, not at SAMPLE_HZ -- the physics is still frame-rate
      // independent (it substeps at 500 Hz), but the LOG is not, and a reader
      // comparing two runs should be able to see that.
      sampleRateActualHz: this.samples > 1 && this.durationS > 0
        ? round((this.samples - 1) / this.durationS, 2)
        : null,
      samples: this.samples,
      truncated: this.truncated,
      channels: CHANNEL_IDS,
      derivedChannels: DERIVED_CHANNELS,
      /**
       * What was actually steering, in seconds, beside `profile`, which is
       * what the driver chose from a menu. The two disagreeing is not an
       * error: it is a wheel driven through the controller profile, and the
       * boards need to know it was a wheel.
       */
      inputSources: Object.fromEntries(
        Object.entries(this._steerS).map(([k, v]) => [k, round(v, 2)]),
      ),
      /** `inputSources` reduced to a leaderboard class, or null when nobody
       *  steered long enough to say. See `detectInputClass`. */
      detectedInput: detectInputClass(this._steerS),
      laps: this.laps,
      events: this.events,
      stats: this.stats(),
    };
  }
}

/**
 * Reduce observed steering time to the class a leaderboard ranks within.
 *
 * Returns "wheel", "controller", "keyboard" or null. The mouse folds into the
 * keyboard because it is the keyboard profile's own steering mode, not a
 * fourth kind of rig: a board that split them would have one entry on it.
 *
 * Whichever source steered for longest wins. A driver who genuinely swaps
 * devices mid-session lands on the board for the one they drove most of the
 * run with, which is the honest answer to a question that has no clean one.
 *
 * Null below `MIN_STEER_S`, because a run where nobody meaningfully steered
 * -- the car was staged, or rolled a few metres -- has no evidence in it, and
 * guessing from a tenth of a second of noise is worse than saying so. Helios
 * falls back to the declared profile there, which is all it ever had.
 */
export function detectInputClass(steerS) {
  const s = steerS ?? {};
  const byClass = {
    wheel: s.wheel ?? 0,
    controller: s.pad ?? 0,
    keyboard: (s.key ?? 0) + (s.mouse ?? 0),
  };
  const total = byClass.wheel + byClass.controller + byClass.keyboard;
  if (!(total >= MIN_STEER_S)) return null;
  let best = null;
  for (const [k, v] of Object.entries(byClass)) {
    if (best == null || v > byClass[best]) best = k;
  }
  return byClass[best] > 0 ? best : null;
}

/** Steering time a run needs before its inputs are evidence of anything. */
export const MIN_STEER_S = 3;

function fmtNum(v, dp) {
  if (!Number.isFinite(v)) return "0";
  if (dp === 0) return String(Math.round(v));
  const s = v.toFixed(dp);
  // Trim the trailing zeros a fixed-point format adds. Across 70 columns and
  // tens of thousands of rows that is a third of the file.
  return s.includes(".") ? s.replace(/0+$/, "").replace(/\.$/, "") : s;
}

function round(v, dp) {
  if (!Number.isFinite(v)) return 0;
  const f = 10 ** dp;
  return Math.round(v * f) / f;
}
