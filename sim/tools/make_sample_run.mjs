// Drive the real vehicle model round a real course and file the result as a
// recorded run.
//
// This is not a test fixture: it is a robot driver good enough to produce a
// run that looks like a run -- real lap and sector times, real cone strikes,
// real load transfer -- so the replay, the launch screen's run list and the
// Helios side can all be worked on and demonstrated without anyone having to
// sit down at the rig first.
//
//   node sim/tools/make_sample_run.mjs [--track autocross] [--laps 2]
//                                      [--driver NAME] [--out DIR]
//
// With no `--out` it writes into the same place the game does, so the run
// shows up in the sim's Runs tab and in Helios straight away.

import fs from "node:fs";
import path from "node:path";
import os from "node:os";
import { fileURLToPath } from "node:url";

import { SDM26 } from "../src/vehicle/params.js";
import { Powertrain } from "../src/vehicle/powertrain.js";
import { BicycleModel } from "../src/vehicle/bicycle.js";
import { Track } from "../src/track/track.js";
import { Timing } from "../src/game/timing.js";
import { Recorder, datumFor } from "../src/game/recorder.js";
import { DeltaTimer } from "../src/game/delta.js";
import { newRunId } from "../src/game/runStore.js";
import { PARAM_DEFAULTS, readParam } from "../src/vehicle/paramMeta.js";
import { ADJUSTABLE_PATHS } from "../src/vehicle/setupAdjust.js";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const DATA = path.join(HERE, "..", "data");

// ------------------------------------------------------------------ args --

const argv = process.argv.slice(2);
const arg = (name, dflt) => {
  const i = argv.indexOf(`--${name}`);
  return i >= 0 && argv[i + 1] ? argv[i + 1] : dflt;
};
const TRACK = arg("track", "autocross");
const LAPS = Number(arg("laps", 2));
const DRIVER = arg("driver", "Sample driver");
const OUT = arg("out", null);
const SEED = Number(arg("seed", 7));

// Same resolution the sim's Rust shell uses, so a run written here lands where
// the game would have written it.
function runsDir() {
  if (process.env.FSAE_SIM_RUNS_DIR) return process.env.FSAE_SIM_RUNS_DIR;
  if (process.platform === "win32" && process.env.LOCALAPPDATA) {
    return path.join(process.env.LOCALAPPDATA, "Helios", "sim-runs");
  }
  if (process.platform === "darwin") {
    return path.join(os.homedir(), "Library", "Application Support", "Helios", "sim-runs");
  }
  return path.join(process.env.XDG_DATA_HOME || path.join(os.homedir(), ".local", "share"),
                   "Helios", "sim-runs");
}

// ---------------------------------------------------------------- driver --

/**
 * A lap-time-plausible robot driver.
 *
 * It is a pure-pursuit steering controller on the centreline plus a speed
 * target from the local curvature, which is the classic minimal driver model
 * and produces a line a human would recognise. It is deliberately imperfect --
 * the lookahead is short enough to cut some corners and clip the occasional
 * cone, which is what makes the resulting run useful to look at.
 */
class RobotDriver {
  constructor(track, rng, { aLat = 1.15, aBrake = 1.0, wander = 0.2, kLat = 2.5, laBase = 3.0, laGain = 0.4 } = {}) {
    this.track = track;
    this.rng = rng;
    this.aLat = aLat;
    this.aBrake = aBrake;
    this.wander = wander;
    this.kLat = kLat;
    this.laBase = laBase;
    this.laGain = laGain;
    // Sloppiness: a constant lateral bias that drifts slowly, so the laps are
    // not identical and the lap-delta has something to show.
    this.bias = 0;
    this.biasTarget = 0;
    this.biasClock = 0;
    this.lastRoadDeg = 0;
  }

  /** Lookahead grows with speed: close in a hairpin, far down a straight. */
  lookahead(speed) { return this.laBase + speed * this.laGain; }

  /**
   * The speed to be at now.
   *
   * Not "how tight is the tightest thing within 40 m" -- that slows the car
   * the instant a hairpin comes into view and is why the first version of
   * this robot drove the course at walking pace. Instead, for every point
   * ahead, work out how fast the car may be there and how fast it may
   * therefore be HERE given how hard it can brake, and take the lowest.
   * That is the standard backwards pass of a lap simulator, run forwards one
   * point at a time.
   */
  targetSpeed(index) {
    const tr = this.track;
    const n = tr.center.length;
    const A_LAT = this.aLat * 9.80665;
    const A_BRAKE = this.aBrake * 9.80665;
    let best = 34;
    for (let k = 0; k < 70; k++) {
      const j = tr.closed ? (index + k) % n : Math.min(n - 1, index + k);
      const kappa = Math.max(Math.abs(tr.curvature[j]), 1e-5);
      const vCorner = Math.sqrt(A_LAT / kappa);
      // The centreline is resampled to 1 m, so k is the distance in metres.
      const vHere = Math.sqrt(vCorner * vCorner + 2 * A_BRAKE * k);
      best = Math.min(best, vHere);
    }
    return Math.max(4, best);
  }

  step(car, dt) {
    const tr = this.track;
    const n = tr.center.length;
    this.biasClock += dt;
    if (this.biasClock > 2.2) {
      this.biasClock = 0;
      // A quarter of a metre: enough that no two laps are identical, not so
      // much that the robot posts a cone score instead of a lap time.
      this.biasTarget = (this.rng() - 0.5) * this.wander;
    }
    this.bias += (this.biasTarget - this.bias) * Math.min(1, dt * 1.2);

    const loc = tr.locate(car.X, car.Y, car.psi);
    const ahead = this.lookahead(car.speed);
    const i = tr.closed ? (loc.index + Math.round(ahead)) % n
                        : Math.min(n - 1, loc.index + Math.round(ahead));
    const target = tr.center[i];

    // Pure pursuit at the lookahead point, offset by the wandering bias.
    const hx = Math.cos(tr.heading[i]);
    const hy = Math.sin(tr.heading[i]);
    const tx = target[0] - hy * this.bias;
    const ty = target[1] + hx * this.bias;
    let bearing = Math.atan2(ty - car.Y, tx - car.X) - car.psi;
    while (bearing > Math.PI) bearing -= Math.PI * 2;
    while (bearing < -Math.PI) bearing += Math.PI * 2;
    const dist = Math.max(1, Math.hypot(tx - car.X, ty - car.Y));
    const curv = (2 * Math.sin(bearing)) / dist;
    let roadDeg = (Math.atan(curv * SDM26.wheelbaseM) * 180) / Math.PI;

    // Pure pursuit alone converges on the line but tolerates a standing
    // offset, and a standing offset on a 3.5 m corridor is a cone. A term on
    // the lateral error itself pulls it back to the middle.
    roadDeg -= clamp((loc.lateral - this.bias) * this.kLat, -6, 6);

    // Cap the demand at the angle that would ask for the lateral
    // acceleration budget and no more. Without this the geometry happily
    // commands 13 degrees of road wheel at 11 m/s, which is three times the
    // tyre's peak slip: the front bites, the rear lets go, and the robot
    // spins on its second corner. Kinematically ay = v^2 tan(d)/L, so the
    // angle that spends exactly the budget is atan(a L / v^2).
    const v = Math.max(3, car.speed);
    const limitDeg = Math.min(
      SDM26.maxSteerDeg,
      (Math.atan((this.aLat * 9.80665 * SDM26.wheelbaseM) / (v * v)) * 180) / Math.PI,
    );
    roadDeg = clamp(roadDeg, -limitDeg, limitDeg);

    // And rate-limit it. A step in steering angle is a step in front slip
    // angle, and the relaxation length means the tyre answers a step late --
    // which is the other half of how a controller with no hands spins a car.
    const maxStep = 240 * dt;
    roadDeg = clamp(roadDeg, this.lastRoadDeg - maxStep, this.lastRoadDeg + maxStep);
    this.lastRoadDeg = roadDeg;

    const steer = clamp(roadDeg / SDM26.maxSteerDeg, -1, 1);

    const want = this.targetSpeed(loc.index);
    const err = want - car.speed;
    // Asymmetric: squeeze the throttle on, stand on the brake. A gentle
    // brake gain leaves the car arriving too fast and understeering wide.
    let throttle = clamp(err * 0.55, 0, 1);
    const brake = clamp(-err * 0.55, 0, 1);
    // The traction circle, crudely: grip already spent sideways is not
    // available to the rear tyres, and a robot that floors it at the apex is
    // a robot that spins at the apex.
    const spent = Math.min(1, Math.abs(car.telemetry.ayG) / this.aLat);
    throttle *= clamp(1 - spent * spent, 0.12, 1);
    return { steer, throttle, brake };
  }
}

function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

/** Every editable vehicle parameter at its current value. Mirrors what
 *  `Game.beginRun` puts in a real run's manifest. */
function snapshotSetup() {
  const out = {};
  for (const path of Object.keys(PARAM_DEFAULTS)) {
    try {
      const v = readParam(path);
      if (typeof v === "number" && Number.isFinite(v)) out[path] = v;
    } catch { /* a parameter this build no longer has */ }
  }
  // The in-car adjuster's items are not in PARAM_DEFAULTS; see main.js.
  for (const path of ADJUSTABLE_PATHS) {
    const v = readParam(path);
    if (typeof v === "number" && Number.isFinite(v)) out[path] = v;
  }
  return out;
}

/** A small deterministic generator, so `--seed` reproduces a run exactly. */
function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// ------------------------------------------------------------------- run --

const trackFile = { autocross: "track-autocross.json", endurance: "track-endurance.json" }[TRACK];
if (!trackFile) {
  console.error(`make_sample_run: no course called "${TRACK}" (autocross or endurance)`);
  process.exit(1);
}
const track = new Track(JSON.parse(fs.readFileSync(path.join(DATA, trackFile), "utf8")));
const curve = JSON.parse(fs.readFileSync(path.join(DATA, "sdm26-torque.json"), "utf8"));

const powertrain = new Powertrain(SDM26, curve);
const car = new BicycleModel(SDM26, powertrain);
const timing = new Timing(track);
const rng = mulberry32(SEED);
const robot = new RobotDriver(track, rng, {
  aLat: Number(arg("alat", 1.15)),
  aBrake: Number(arg("abrake", 1.0)),
  wander: Number(arg("wander", 0.2)),
  kLat: Number(arg("klat", 2.5)),
  laBase: Number(arg("labase", 3.0)),
  laGain: Number(arg("lagain", 0.4)),
});

const runId = newRunId(TRACK);
const rec = new Recorder({
  runId,
  track: TRACK,
  trackName: track.name,
  trackKind: "course",
  trackLengthM: track.length,
  trackClosed: !!track.closed,
  trackSectors: Array.from(track.sectors ?? []),
  trackSource: track.source ?? null,
  datum: datumFor(TRACK),
  driver: DRIVER,
  session: "Generated by make_sample_run.mjs",
  profile: "robot",
  profileName: "Pure-pursuit robot",
  device: null,
  physics: "javascript",
  assists: { traction: false, abs: false, autoShift: true },
  // The live vehicle parameters, exactly as the game records them: a lap time
  // only means something next to the car that set it.
  setup: snapshotSetup(),
  etc: null,
  simVersion: "1.0.0",
  synthetic: true,
});
// A delta timer, so a generated run exercises the same path a driven one does
// and `sim.delta_s` is not permanently blank in every fixture. On a lapped
// course the first lap becomes the reference and the second is measured
// against it, which is exactly what happens on the rig.
const deltaTimer = new DeltaTimer(track.length);
timing.onLap = (entry, sectors) => {
  rec.recordLap(entry, sectors);
  if (deltaTimer.completeLap(entry.raw)) rec.setReference(deltaTimer.describeReference());
};

const start = track.startPose();
car.respawn(start.x, start.y, start.psi, 0);

const DT = 1 / 120;
// Long enough for the robot to finish, short enough that a bad seed cannot
// run forever: the length it has to cover at a pessimistic 7 m/s, plus a
// standing-start allowance.
const MAX_S = 30 + (track.length / 7) * (track.closed ? LAPS : 1);
let t = 0;
let spinF = 0;
let spinR = 0;
let lastGear = powertrain.gear + 1;
let wasOff = false;

const ctx = { assists: { traction: false, abs: false, autoShift: true } };

while (t < MAX_S) {
  const input = robot.step(car, DT);

  // Automatic gearbox, exactly as the game's auto-shift does it.
  if (powertrain.canShift()) {
    if (powertrain.engineRpm > powertrain.optimalUpshiftRpm()) powertrain.requestUpshift();
    else if (powertrain.gear > 0 && powertrain.engineRpm < 5200 &&
             powertrain.downshiftSafe(car.wR)) powertrain.requestDownshift();
  }

  car.step(DT, input);
  const loc = track.locate(car.X, car.Y, car.psi);
  const hits = track.strikeCones({ x: car.X, y: car.Y, psi: car.psi },
                                 { front: 1.5, rear: 1.1, halfWidth: 0.72 });
  // The same order the game runs in: the recorder is told where the frame
  // lands, timing scores against it, and only then is a row sampled. Getting
  // this order wrong here is how a fixture stops modelling the thing it is a
  // fixture for.
  rec.frameDt = DT;
  const wasStaged = timing.state === "staged";
  timing.update(DT, loc, car.speed > 0.6, hits);
  // THE GREEN FLAG. `Game.update` calls this on the staged -> running edge and
  // the recorder cannot infer it: without it the beacon's only rising edge is
  // at the chequered flag, so an autocross run -- one lap, one crossing --
  // arrives in Helios with no usable lap table at all. Every sample run
  // generated before this line had exactly that defect.
  if (wasStaged && timing.state === "running") rec.markLine();
  if (timing.state === "running") deltaTimer.update(loc.s, timing.lapTime);

  spinF = (spinF + car.wF * DT) % (Math.PI * 2);
  spinR = (spinR + car.wR * DT) % (Math.PI * 2);

  const tel = car.telemetry;
  ctx.t = tel;
  ctx.car = car;
  ctx.speed = car.speed;
  ctx.rpm = powertrain.engineRpm;
  ctx.gear = powertrain.gear + 1;
  ctx.gearRatio = powertrain.ratio();
  ctx.pedal = input.throttle;
  ctx.plate = input.throttle;
  ctx.brake = input.brake;
  ctx.brakeBiasFront = SDM26.brakeBiasFront;
  ctx.rimDeg = (car.delta * 180) / Math.PI * SDM26.steeringRatio;
  ctx.roadWheelDeg = tel.steerDeg ?? (car.delta * 180) / Math.PI;
  ctx.steerInput = input.steer;
  ctx.ffbCommand = clamp(tel.rimTorqueNm / 15, -1, 1);
  ctx.ffbClipped = Math.abs(tel.rimTorqueNm) > 15;
  ctx.wRL = car.wRL ?? car.wR;
  ctx.wRR = car.wRR ?? car.wR;
  ctx.spinFront = spinF;
  ctx.spinRear = spinR;
  ctx.s = loc.s;
  ctx.lateral = loc.lateral;
  ctx.headingErrorDeg = (loc.headingErrorRad * 180) / Math.PI;
  ctx.curvature = loc.curvature;
  ctx.onTrack = loc.onTrack;
  ctx.lap = timing.lap;
  ctx.lapTime = timing.lapTime;
  ctx.sector = timing.sectorIndex + 1;
  ctx.cones = timing.cones;
  ctx.offCourse = timing.offCourse;
  ctx.penaltyS = timing.penaltyS;
  ctx.deltaValid = deltaTimer.hasReference && deltaTimer.deltaValid;
  ctx.deltaS = ctx.deltaValid ? deltaTimer.delta : 0;
  ctx.launch = false;
  ctx.clutchSlipRpm = powertrain.clutchSlipRpm ?? 0;
  ctx.shifting = powertrain.shiftTimer > 0;

  rec.tick(DT, ctx);

  if (hits > 0) {
    rec.event("cone", { lap: timing.lap, n: hits, x: r3(car.X), y: r3(car.Y), s: r3(loc.s) });
  }
  if (!loc.onTrack && car.speed > 2 && !wasOff) {
    wasOff = true;
    rec.event("off-course", { lap: timing.lap, x: r3(car.X), y: r3(car.Y), s: r3(loc.s), lateral: r3(loc.lateral) });
  } else if (loc.onTrack && wasOff) {
    wasOff = false;
  }
  if (ctx.gear !== lastGear) {
    rec.event("shift", { from: lastGear, to: ctx.gear, rpm: Math.round(ctx.rpm), s: r3(loc.s) });
    lastGear = ctx.gear;
  }

  t += DT;
  if (timing.state === "finished") break;
  if (track.closed && timing.laps.length >= LAPS) break;
}
rec.finish(timing.state === "finished" ? "finished" : "ended");

function r3(v) { return Math.round(v * 1000) / 1000; }

// ----------------------------------------------------------------- write --

const dir = path.join(OUT ?? runsDir(), runId);
fs.mkdirSync(dir, { recursive: true });
const manifest = rec.toManifest();
fs.writeFileSync(path.join(dir, "run.json"), JSON.stringify(manifest, null, 2));
fs.writeFileSync(path.join(dir, "telemetry.csv"), rec.toCsv());

const st = manifest.stats;
console.log(`wrote ${dir}`);
console.log(`  ${manifest.trackName}, ${DRIVER}`);
console.log(`  ${rec.samples} samples over ${st.durationS.toFixed(2)} s, ${st.distanceM.toFixed(0)} m`);
console.log(`  laps: ${manifest.laps.map((l) => `L${l.lap} ${l.total.toFixed(3)}${l.cones ? ` (${l.cones}c)` : ""}`).join(", ") || "none"}`);
console.log(`  best ${st.bestLapS == null ? "-" : st.bestLapS.toFixed(3)} s, ` +
            `theoretical ${st.theoreticalBestS == null ? "-" : st.theoreticalBestS.toFixed(3)} s`);
console.log(`  peak ${st.peakSpeedKph.toFixed(0)} km/h, ${st.peakLatG.toFixed(2)} g lat, ` +
            `${st.totalCones} cones, ${st.totalOffCourse} off`);
