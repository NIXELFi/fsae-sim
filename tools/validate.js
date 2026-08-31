// Physics sanity harness. Runs the same model the game runs, headless, and
// checks it against the numbers Helios validated on real SDM26 runs.
//
//   node tools/validate.js
//
// These are not unit tests of the integrator -- they are the three events the
// team actually has stopwatch data for. If a change to the tyre or powertrain
// moves these, the change is wrong until proven otherwise.

import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

import { SDM26, gearVps } from "../src/vehicle/params.js";
import { Powertrain } from "../src/vehicle/powertrain.js";
import { BicycleModel } from "../src/vehicle/bicycle.js";
import { TIRE_INFO } from "../src/vehicle/tire.js";
import { EngineAudio, cbr600rrSdm26, Rng } from "../src/audio/engineAudio.js";

const here = dirname(fileURLToPath(import.meta.url));
const curve = JSON.parse(readFileSync(join(here, "..", "data", "sdm26-torque.json"), "utf8"));

const DT = 1 / 500;
let failures = 0;

function check(label, value, lo, hi, unit = "") {
  const ok = value >= lo && value <= hi;
  if (!ok) failures++;
  const mark = ok ? "PASS" : "FAIL";
  console.log(`  [${mark}] ${label.padEnd(34)} ${value.toFixed(3)}${unit}  (expect ${lo}-${hi}${unit})`);
}

function fresh() {
  const pt = new Powertrain(SDM26, curve);
  return { pt, car: new BicycleModel(SDM26, pt) };
}

// ---------------------------------------------------------------- skidpad ---
// FSAE skidpad: 15.25 m inner diameter, 3 m lane. Oracle solves the width-aware
// path radius; the lap sim uses 9.125 m for SDM26. Real comp run: 5.02 s.
console.log("\nSKIDPAD  (steady 9.125 m radius, real SDM26 run = 5.02 s)");
{
  const R = 9.125;
  // Constant-radius test: hold the 9.125 m circle with a PI steering
  // controller, raise the speed until the car can no longer hold the line.
  // Driven, not solved -- so the transient model has to actually settle.
  let bestV = 0;
  for (let vTarget = 8; vTarget <= 16; vTarget += 0.05) {
    const { car } = fresh();
    // Gear before respawn: respawn syncs the crank to the wheels, and it can
    // only do that against the gear it is actually in.
    car.pt.gear = 1;
    car.respawn(0, 0, 0, vTarget);
    // PI on yaw rate around an Ackermann feed-forward. The output is the steer
    // angle itself, NOT an increment -- accumulating into `steer` on top of an
    // integral term makes a double integrator that oscillates into a spin and
    // reports a grip limit the car never actually reached.
    const ff = car.p.wheelbaseM / R / ((car.p.maxSteerDeg * Math.PI) / 180);
    let integral = 0, sumR = 0, nR = 0, blew = false;
    for (let i = 0; i < 6000; i++) {
      const targetYaw = car.speed / R;
      const err = targetYaw - car.r;
      integral = Math.max(-0.5, Math.min(0.5, integral + err * DT));
      const steer = Math.max(-1, Math.min(1, ff + 6 * err + 4 * integral));
      const vErr = vTarget - car.speed;
      const thr = Math.max(0, Math.min(1, 0.3 + vErr * 0.6));
      car.step(DT, { steer, throttle: thr, brake: 0 });
      if (Math.abs(car.telemetry.bodySlipDeg) > 45) { blew = true; break; }
      if (i > 4000) {
        sumR += car.speed / Math.max(Math.abs(car.r), 1e-4);
        nR++;
      }
    }
    const meanR = nR ? sumR / nR : 1e9;
    const held = !blew && Math.abs(meanR - R) / R < 0.04 && Math.abs(car.speed - vTarget) < 0.5;
    if (held) bestV = vTarget;
  }
  const lap = (2 * Math.PI * R) / bestV;
  const g = (bestV * bestV) / R / 9.81;
  console.log(`  sustained ${bestV.toFixed(2)} m/s = ${g.toFixed(3)} g`);
  check("skidpad lap time", lap, 4.85, 5.35, " s");
  // Above the 1.368 tyre mu because 11.5 m/s is already worth ~250 N of
  // downforce on this aero package -- grip scales with it, mass does not.
  check("lateral acceleration", g, 1.35, 1.58, " g");
}

// ------------------------------------------------------------------ accel ---
// FSAE 75 m acceleration. Helios pins muLong so this lands ~4.2 s -- but the
// lap sim's launch is traction-limited by construction, so to compare like for
// like the launch here is throttle-modulated to hold slip ratio near the peak.
// Pinning the throttle open instead spins the rears to kappa ~2 for three
// seconds and costs over a second, which is correct behaviour for a 267 kg car
// with 918 N.m at the wheels in first, and is exactly the skill the game asks
// the driver for. That case is checked separately below.
console.log("\nACCELERATION  (75 m from standstill, Helios reference ~4.2 s)");
{
  const idealLaunch = (traction) => {
    const { car, pt } = fresh();
    car.respawn(0, 0, 0, 0);
    let t = 0;
    const shiftAt = [];
    while (t < 12 && car.X < 75) {
      if (pt.canShift() && pt.engineRpm > pt.optimalUpshiftRpm() && pt.gear < 5) {
        if (pt.requestUpshift()) {
          shiftAt.push({ t: +t.toFixed(2), gear: pt.gear + 2, rpm: Math.round(pt.engineRpm) });
        }
      }
      let throttle = 1;
      if (traction) {
        // Hold slip ratio just past the 0.11 peak, the way a good launch does.
        const over = car.telemetry.kappaR - 0.13;
        throttle = Math.max(0.15, Math.min(1, 1 - over * 8));
      }
      car.step(DT, { steer: 0, throttle, brake: 0 });
      t += DT;
    }
    return { t, car, shiftAt };
  };

  const good = idealLaunch(true);
  // Band is deliberately above the QSS 4.2 s. This model carries the driveline
  // rotational inertia (+94 kg apparent in first) that a quasi-steady lap sim
  // ignores entirely, so it SHOULD be a few tenths slower. If this ever comes
  // in at 4.2 s, something has stopped modelling the inertia.
  check("75 m time (managed launch)", good.t, 4.20, 4.95, " s");
  check("speed at 75 m", good.car.speed * 3.6, 95, 130, " km/h");
  console.log(`  shifts: ${good.shiftAt.map((s) => `${s.gear} @ ${s.t}s/${s.rpm}rpm`).join(", ")}`);

  const crude = idealLaunch(false);
  console.log(`  same run, throttle pinned open: ${crude.t.toFixed(2)} s ` +
              `(+${(crude.t - good.t).toFixed(2)} s lost to wheelspin)`);
}

// --------------------------------------------------------------- braking ---
console.log("\nBRAKING  (from 25 m/s, full pedal)");
{
  const { car } = fresh();
  car.pt.gear = 3;
  car.respawn(0, 0, 0, 25);
  let t = 0, x0 = car.X, peak = 0;
  while (t < 6 && car.speed > 0.5) {
    car.step(DT, { steer: 0, throttle: 0, brake: 1 });
    peak = Math.min(peak, car.telemetry.axG);
    t += DT;
  }
  check("stopping distance", car.X - x0, 20, 40, " m");
  check("peak deceleration", -peak, 1.3, 2.6, " g");
}

// ------------------------------------------------------------- powertrain ---
console.log("\nPOWERTRAIN  (from the Helios CFD sweep)");
{
  const pt = new Powertrain(SDM26, curve);
  check("peak torque", pt.peakTorque.torqueNm, 60, 65, " N.m");
  check("peak torque rpm", pt.peakTorque.rpm, 7500, 8500, " rpm");
  check("peak power", pt.peakPower.powerKW, 55, 60, " kW");
  const topGear = SDM26.gearRatios.length - 1;
  const vmax = gearVps(SDM26, topGear) * SDM26.revLimitRpm;
  check("geared top speed", vmax * 3.6, 110, 150, " km/h");
  console.log(`  motoring drag @ 10k rpm: ${pt.motoringTorque(10000).toFixed(1)} N.m`);
}

// -------------------------------------------------------------------- tyre ---
console.log("\nTYRE");
{
  const Fz = (SDM26.massKg * 9.81) / 4;
  const cs = TIRE_INFO.corneringStiffness(SDM26.muLat, Fz);
  check("cornering stiffness / tyre", cs / 57.3, 200, 380, " N/deg");
  check("peak slip angle", TIRE_INFO.peakSlipAngleDeg, 6, 11, " deg");
}

// --------------------------------------------------------------- ETC map ---
// The pedal map's contract is that it can never misbehave no matter where the
// breakpoints are dragged: monotone, in range, and exactly through the points.
console.log("\nETC MAP  (pedal -> throttle plate)");
{
  const { EtcMap, ETC_PRESETS } = await import("../src/vehicle/etcMap.js");

  // Linear preset must be the identity.
  const lin = new EtcMap(ETC_PRESETS.linear.points);
  let linErr = 0;
  for (let x = 0; x <= 100; x += 0.5) linErr = Math.max(linErr, Math.abs(lin.plateAt(x) - x));
  check("linear preset max error", linErr, 0, 0.001, " %");

  // Every preset: hits its own breakpoints, stays monotone and in range.
  let presetErr = 0;
  for (const key of Object.keys(ETC_PRESETS)) {
    const m = new EtcMap(ETC_PRESETS[key].points, key);
    for (const [px, py] of m.points) presetErr = Math.max(presetErr, Math.abs(m.plateAt(px) - py));
  }
  check("presets pass through breakpoints", presetErr, 0, 0.001, " %");

  // Randomised torture test. This is the one that matters: a Catmull-Rom or
  // natural cubic through the same points fails it, because it overshoots
  // between breakpoints and the plate briefly closes as the pedal opens.
  let worstBackstep = 0, worstOutOfRange = 0, worstBracket = 0, cases = 0;
  let seed = 12345;
  const rnd = () => (seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff;
  for (let trial = 0; trial < 4000; trial++) {
    const n = 2 + Math.floor(rnd() * 10);
    const xs = [];
    for (let i = 0; i < n; i++) xs.push(rnd() * 100);
    xs.sort((a, b) => a - b);
    const pts = xs.map((x) => [x, rnd() * 100]);
    const m = new EtcMap(pts);
    cases++;

    let prev = -1;
    for (let x = 0; x <= 100; x += 0.5) {
      const y = m.plateAt(x);
      worstBackstep = Math.max(worstBackstep, prev - y);
      worstOutOfRange = Math.max(worstOutOfRange, Math.max(0, -y, y - 100));
      prev = y;
    }
    // Between any two breakpoints the curve must stay inside their values.
    for (let i = 0; i < m.points.length - 1; i++) {
      const [x0, y0] = m.points[i], [x1, y1] = m.points[i + 1];
      const lo = Math.min(y0, y1), hi = Math.max(y0, y1);
      for (let s = 0; s <= 20; s++) {
        const y = m.plateAt(x0 + ((x1 - x0) * s) / 20);
        worstBracket = Math.max(worstBracket, lo - y, y - hi);
      }
    }
  }
  console.log(`  ${cases} random maps, 2-11 breakpoints each`);
  check("worst backwards step", worstBackstep, 0, 1e-9, " %");
  check("worst out-of-range", worstOutOfRange, 0, 1e-9, " %");
  check("worst overshoot past a breakpoint", worstBracket, 0, 1e-9, " %");

  // Editing keeps the map valid.
  const e = new EtcMap(ETC_PRESETS.linear.points);
  const idx = e.addPoint(50, 20);
  check("addPoint returns an index", idx, 1, 1, "");
  e.movePoint(idx, 50, -40);            // try to drag below the previous point
  check("drag below neighbour clamps", e.points[1][1], 0, 0, " %");
  e.movePoint(idx, 999, 130);           // try to drag past the end
  check("drag past neighbour clamps", e.points[1][0], 99, 99, " %");
  check("removePoint on an anchor refused", e.removePoint(0) ? 1 : 0, 0, 0, "");
  check("removePoint on interior works", e.removePoint(1) ? 1 : 0, 1, 1, "");
  check("anchors survive editing", e.points.length, 2, 2, "");

  // A plate-limited map: full pedal, less than full throttle.
  const wet = new EtcMap(ETC_PRESETS.wet.points, "wet");
  check("wet map plate at full pedal", wet.plateAt(100), 80, 84, " %");
  check("pedal 0 always closes the plate", lin.evaluate(0) + wet.evaluate(0), 0, 0, "");
}

// ------------------------------------------------------- engine audio -------
// The engine synthesiser exists twice: here, and as the `engine-audio` Rust
// crate the Bevy build uses. Two implementations of one model only stay honest
// if something checks them against each other, so the Rust side emits golden
// vectors and this compares against them.
//
// The comparison is in two parts on purpose. A short prefix of raw samples
// catches a coefficient typed wrong or a table indexed off by one -- anything
// wrong from the first sample. It cannot run long, because Rust computes in f32
// and JavaScript in f64, and a nonlinear model diverges from that eventually
// for a reason that does not matter. The spectral check survives that
// divergence and catches the mistakes that matter musically: wrong firing
// order, wrong cycle length, a pipe tuned to the wrong length.
//
// Regenerate after changing either side:
//   cargo run -p engine-audio --release --example golden_vectors \
//     > fsae-sim/data/engine-audio-golden.json
console.log("\nENGINE AUDIO  (vs golden vectors from the Rust crate)");
{
  const golden = JSON.parse(
    readFileSync(join(here, "..", "data", "engine-audio-golden.json"), "utf8"),
  );

  // The generator underpins every noise-driven stage; if it disagrees, nothing
  // downstream can be compared at all.
  const rng = new Rng(1);
  const seq = [0, 0, 0, 0, 0].map(() => rng.nextU32());
  const expected = [270369, 67634689, 2647435461, 307599695, 2398689233];
  check(
    "xorshift32 matches Rust",
    seq.every((v, i) => v === expected[i]) ? 1 : 0,
    1, 1, "",
  );

  const mkEngine = () =>
    new EngineAudio(cbr600rrSdm26(), {
      sampleRate: golden.sampleRate,
      irTaps: golden.irTaps,
      cabin: golden.cabin,
      seed: golden.seed,
    });

  const magAt = (buf, fs, f) => {
    let mean = 0;
    for (const v of buf) mean += v;
    mean /= buf.length;
    const w = (2 * Math.PI * f) / fs;
    let re = 0;
    let im = 0;
    for (let i = 0; i < buf.length; i++) {
      const x = buf[i] - mean;
      re += x * Math.cos(w * i);
      im += x * Math.sin(w * i);
    }
    return Math.hypot(re, im) / buf.length;
  };

  for (const c of golden.cases) {
    const e = mkEngine();
    e.setOperatingPoint(c.rpm, c.throttle, c.torqueNm);
    const prefix = new Float32Array(golden.prefixLength);
    e.render(prefix);

    let worst = 0;
    for (let i = 0; i < prefix.length; i++) {
      worst = Math.max(worst, Math.abs(prefix[i] - c.prefix[i]));
    }
    check(`${c.rpm} rpm: prefix vs Rust`, worst, 0, 5e-3, "");

    // Then settle and check where the energy sits.
    const warm = new Float32Array(48000);
    e.render(warm);
    const buf = new Float32Array(24000);
    e.render(buf);

    const fundamental = magAt(buf, golden.sampleRate, c.firingHz);
    check(`${c.rpm} rpm: firing fundamental present`, fundamental > 1e-3 ? 1 : 0, 1, 1, "");

    // Half order must be absent: an even-firing four has no once-per-cycle
    // component. If this appears, the firing angles or the 720-degree wrap are
    // wrong.
    const half = magAt(buf, golden.sampleRate, c.firingHz * 0.5);
    check(`${c.rpm} rpm: half-order rejection`, fundamental / Math.max(half, 1e-12), 20, 1e9, "x");

    // And the harmonics must be there, or it is a bare tone rather than an
    // engine.
    const h2 = magAt(buf, golden.sampleRate, c.firingHz * 2);
    check(`${c.rpm} rpm: second harmonic present`, h2 / fundamental, 0.05, 5, "");
  }

  // Silence has to be silent. The air-noise stage modulates rather than adds,
  // which is what makes this true however much noise is dialled in.
  {
    const e = mkEngine();
    e.setOperatingPoint(9000, 1, 60);
    const warm = new Float32Array(24000);
    e.render(warm);
    e.setRunning(false);
    e.render(warm); // ring-out
    const quiet = new Float32Array(48000);
    e.render(quiet);
    let rms = 0;
    for (const v of quiet) rms += v * v;
    rms = Math.sqrt(rms / quiet.length);
    check("stopped engine is silent", rms, 0, 1e-3, "");
  }

  // It has to keep ahead of the sound card with room to spare -- in the browser
  // this shares a core with the renderer.
  {
    const e = mkEngine();
    e.setOperatingPoint(12000, 1, 60);
    const seconds = 2;
    const buf = new Float32Array(golden.sampleRate * seconds);
    const t0 = process.hrtime.bigint();
    e.render(buf);
    const elapsed = Number(process.hrtime.bigint() - t0) / 1e9;
    check("render speed vs real time", seconds / elapsed, 4, 1e6, "x");
  }
}

console.log(`\n${failures === 0 ? "ALL CHECKS PASSED" : `${failures} CHECK(S) FAILED`}\n`);
process.exit(failures === 0 ? 0 : 1);
