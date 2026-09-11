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
import { buildCarFromGlb, parseGlb } from "../src/render/glbcar.js";

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
    // Gear before respawn: respawn keeps it when placing the car at speed and
    // syncs the crank to the wheels against it. (It used to be wiped by the
    // reset inside respawn, which put every rolling test in first gear.)
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

  // Past the peak a slick keeps most of its force -- a sharp drop would make
  // every slide unrecoverable. The fit holds ~94% at twice the peak slip and
  // ~89% at three times; these bands say it may not get much sharper.
  {
    const { tyreForces } = await import("../src/vehicle/tire.js");
    const fy = (deg) => tyreForces((deg * Math.PI) / 180, 0, Fz, SDM26.muLat, SDM26.muLong).fy;
    const fx = (k) => tyreForces(0, k, Fz, SDM26.muLat, SDM26.muLong).fx;
    const pk = TIRE_INFO.peakSlipAngleDeg, pkK = TIRE_INFO.peakSlipRatio;
    check("lateral force at 2x peak slip", (100 * fy(2 * pk)) / fy(pk), 85, 100, " %");
    check("lateral force at 3x peak slip", (100 * fy(3 * pk)) / fy(pk), 80, 100, " %");
    check("longitudinal force at 2x peak slip", (100 * fx(2 * pkK)) / fx(pkK), 80, 100, " %");
  }

  // Aligning torque: the trail must be longest at zero slip, gone once the
  // tyre is sliding, and grow with load. The constants are fitted to the raw
  // TTC Round 9 Mz channel for the R20 (sim/tools/ttc_trail.py): ~39 mm at
  // zero slip and 700 N, a fifth of that at the force peak, zero by 15 deg
  // (2x the model's peak slip), and the square root of load.
  const t0 = TIRE_INFO.pneumaticTrail(0, Fz);
  const tPeak = TIRE_INFO.pneumaticTrail(1, Fz);
  const tSlide = TIRE_INFO.pneumaticTrail(2.0, Fz);
  check("pneumatic trail at zero slip", t0 * 1000, 25, 50, " mm");
  check("trail at peak grip / trail at zero", tPeak / t0, 0.1, 0.3, "");
  check("trail once sliding", tSlide * 1000, 0, 1e-9, " mm");
  check("trail grows with load", TIRE_INFO.pneumaticTrail(0, 2 * Fz) / t0, 1.2, 1.6, "x");
}

// --------------------------------------------------------------- handling ---
// Why the car felt like it was on ice. None of the stopwatch events above see
// the limit BALANCE or the transient response, and both were wrong: with equal
// tyres front and rear the model was neutral to within 1% of force, the rear
// reached its peak first at every speed, and a 12 deg keyboard tap at 15 m/s
// spun it. These pin the fixes: front-limited at 10-20 m/s, a well-damped yaw
// mode, and a keyboard step to the speed-limited lock that pushes rather than
// spins.
console.log("\nHANDLING  (limit balance, yaw damping, keyboard inputs)");
{
  const { totalReduction } = await import("../src/vehicle/params.js");
  const { PROFILES, usableLockFrac, stepPedal } = await import("../src/game/controlProfiles.js");
  const { axleMu } = await import("../src/vehicle/tire.js");
  const D2R = Math.PI / 180;

  // Place the car at speed in whichever gear puts the engine near 9500 rpm --
  // what a driver would be in -- rather than first on the limiter.
  const place = (V) => {
    const { car } = fresh();
    car.respawn(0, 0, 0, V);
    let best = 0, bd = 1e9;
    for (let g = 0; g < SDM26.gearRatios.length; g++) {
      const rpm = (V / SDM26.tireRadiusM) * totalReduction(SDM26, g) * (60 / (2 * Math.PI));
      if (rpm < 14000 && Math.abs(rpm - 9500) < bd) { bd = Math.abs(rpm - 9500); best = g; }
    }
    car.pt.gear = best;
    car.pt.syncToWheel(car.wR);
    return car;
  };
  {
    const c = place(20);
    check("respawn at speed keeps the gear", c.pt.gear + 1, 3, 3, "");
    check("and the engine is off the limiter", c.pt.engineRpm, 8000, 12000, " rpm");
  }

  // Steady-state limit balance: constant speed, steer ramped slowly to 60% of
  // lock over 12 s. At the peak lateral acceleration the FRONT must be the
  // axle at its limit, with the rear holding something in hand, and the car
  // must push wide rather than spin as the steer keeps coming.
  for (const V of [10, 15, 20, 25, 28]) {
    const car = place(V);
    car.steeringServo = { maxRateDegPerS: 1e6, accelDegPerS2: 1e9, lagS: 0.01 };
    let t = 0, peakAy = 0, at = null, peakBeta = 0;
    while (t < 12) {
      const thr = Math.max(0, Math.min(1, 0.2 + (V - car.speed) * 0.8));
      car.step(DT, { steer: (t / 12) * 0.6, throttle: thr, brake: 0 });
      t += DT;
      const tel = car.telemetry;
      peakBeta = Math.max(peakBeta, Math.abs(tel.bodySlipDeg));
      if (tel.ayG > peakAy) { peakAy = tel.ayG; at = { uF: tel.utilF, uR: tel.utilR }; }
    }
    // Upper bound is a sanity cap, not a measurement: at 28 m/s the aero
    // package adds ~55% of the car's weight, and the TTC load sensitivity
    // (0.12, was 0.15 EST) leaves a little more of that as grip, 1.95 g.
    check(`${V} m/s: peak lateral`, peakAy, 1.35, 2.0, " g");
    check(`${V} m/s: front limits first (utilF - utilR)`, at.uF - at.uR, 0.08, 0.6, "");
    check(`${V} m/s: pushes, does not spin`, peakBeta, 0, 12, " deg slip");
  }

  // Linear 2-DOF yaw mode from the model's own cornering stiffnesses at
  // 15 m/s. An FSAE car is a stiff, light, low-inertia thing: a fast yaw mode
  // (2-4.5 Hz) and well damped. Below ~0.6 the car would hunt after every
  // input; this is the number that says the ice feel was NOT yaw damping.
  {
    const m = SDM26.massKg, I = SDM26.izzKgM2, L = SDM26.wheelbaseM, U = 15;
    const a = L * (1 - SDM26.weightDistFront), b = L * SDM26.weightDistFront;
    const Fz0 = (m * 9.81) / 4;
    const FzF = (m * 9.81 * b) / L, FzR = (m * 9.81 * a) / L;
    const Cf = 2 * TIRE_INFO.corneringStiffness(axleMu(SDM26.muLat * SDM26.frontGripFactor, FzF, 0, Fz0, SDM26.tireLoadSensitivity), FzF / 2);
    const Cr = 2 * TIRE_INFO.corneringStiffness(axleMu(SDM26.muLat, FzR, 0, Fz0, SDM26.tireLoadSensitivity), FzR / 2);
    const A = [[-(Cf + Cr) / (m * U), -(U + (a * Cf - b * Cr) / (m * U))],
               [-(a * Cf - b * Cr) / (I * U), -(a * a * Cf + b * b * Cr) / (I * U)]];
    const tr = A[0][0] + A[1][1], det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    const wn = Math.sqrt(det), zeta = -tr / (2 * wn);
    check("yaw natural frequency at 15 m/s", wn / (2 * Math.PI), 2.0, 4.5, " Hz");
    check("yaw damping ratio at 15 m/s", zeta, 0.6, 1.4, "");
    // Linear-range understeer gradient: near neutral, as the tyres are the
    // same front and rear; the limit balance above is what makes it push.
    check("linear understeer gradient", ((m / L) * (b / Cf - a / Cr) * 9.81) / D2R, -0.6, 1.5, " deg/g");
  }

  // A keyboard step. The key is held; the servo is the keyboard profile's
  // (180 deg/s, 700 deg/s^2, 0.10 s) and the lock is what the profile allows
  // at this speed. With the throttle held steady the car must reach the
  // tyre's limit and push, not spin.
  const kb = PROFILES.keyboard.steering;
  const lockAt = (V) => usableLockFrac(kb.speedSensitive, V, {
    wheelbaseM: SDM26.wheelbaseM, maxSteerDeg: SDM26.maxSteerDeg, peakSlipAngleDeg: TIRE_INFO.peakSlipAngleDeg,
  });
  check("keyboard lock at 5 m/s", lockAt(5), 1, 1, "");
  check("keyboard lock at 15 m/s", lockAt(15) * SDM26.maxSteerDeg, 15, 23, " deg");
  check("keyboard lock at 25 m/s", lockAt(25) * SDM26.maxSteerDeg, 13, 18, " deg");
  // A key held to the lock is a STEP to the usable lock, through the
  // keyboard profile's slip cap. At every speed the car must push, not
  // spin: this is the "front hooks round at the end of a fast corner"
  // complaint, and it needed both the slip cap and the 50% aero split.
  // At 25 m/s and up a key HELD at the limit for seconds still ends in a
  // spin: that is a driver holding 1.9 g at 90 km/h with the rear at its
  // margin, and the real car would go too. Steps are caught to 20 m/s.
  for (const V of [10, 15, 20]) {
    const car = place(V);
    car.steeringServo = { maxRateDegPerS: kb.maxRateDegPerS, accelDegPerS2: kb.accelDegPerS2, lagS: kb.lagS, slipCapDeg: kb.slipCapDeg, rateSpeedRefMps: kb.rateSpeedRefMps, rateSpeedExp: kb.rateSpeedExp };
    let t = 0, peakAy = 0, peakBeta = 0, maxSteer = 0;
    const steerLog = [];
    while (t < 3) {
      car.step(DT, { steer: lockAt(V), throttle: 0.25, brake: 0 });
      t += DT;
      const tel = car.telemetry;
      peakAy = Math.max(peakAy, tel.ayG);
      peakBeta = Math.max(peakBeta, Math.abs(tel.bodySlipDeg));
      maxSteer = Math.max(maxSteer, tel.steerDeg);
      steerLog.push([t, tel.steerDeg]);
    }
    // With the slip cap the road wheel settles wherever the front's peak is,
    // not at the lock; time to 90% of the angle it actually reached.
    const tLock = (steerLog.find(([, d]) => d >= 0.7 * maxSteer) ?? [0])[0];
    check(`${V} m/s: key held, steering builds in`, tLock * 1000, 60, 900, " ms");
    check(`${V} m/s: key held to the lock, peak lateral`, peakAy, 1.3, 1.9, " g");
    check(`${V} m/s: key held to the lock does not spin`, peakBeta, 0, 12, " deg slip");
  }

  // Keyboard pedals ramp instead of stepping.
  {
    const thr = PROFILES.keyboard.pedals.throttle;
    let v = 0, t = 0;
    while (v < 0.999 && t < 2) { v = stepPedal(v, 1, thr, 1 / 120); t += 1 / 120; }
    check("keyboard throttle 0 -> 1 in", t, 0.3, 0.6, " s");
    let d = 1; t = 0;
    while (d > 0.001 && t < 2) { d = stepPedal(d, 0, thr, 1 / 120); t += 1 / 120; }
    check("keyboard throttle 1 -> 0 in", t, 0.05, 0.2, " s");
    check("keyboard profile defaults traction control on", PROFILES.keyboard.assistDefaults?.traction ? 1 : 0, 1, 1, "");
    check("keyboard profile defaults ABS on", PROFILES.keyboard.assistDefaults?.abs ? 1 : 0, 1, 1, "");
    // A pad's trigger has a position: no ramp there.
    check("gamepad pedals are not ramped", PROFILES["gamepad-xbox"].pedals.throttle.rampUpPerS ? 1 : 0, 0, 0, "");
  }
}

// ------------------------------------------------------ Rust model parity ---
// The desktop build runs the vehicle model in Rust (native/crates/sim-core);
// the JS model is what a browser runs and what this script validates. They
// are ports of each other and must agree to floating-point noise on the same
// scripted drive. A divergence is a bug in one of them, not a modelling
// choice. Regenerate the golden file with:
//   cargo run --release -p sim-core --example golden_vehicle > sim/data/vehicle-golden.json
console.log("\nRUST PARITY  (sim-core golden vectors vs the JS model)");
{
  const golden = JSON.parse(readFileSync(join(here, "../data/vehicle-golden.json"), "utf8"));
  // Same drive as examples/golden_vehicle.rs, deliberately inside the tyre:
  // at the limit the model is chaotic and one-ulp libm differences grow into
  // centimetres, which says nothing about whether the models agree.
  const script = (t) =>
    t < 3 ? [0, 0.55, 0] : t < 5 ? [0.15, 0.4, 0] : t < 5.5 ? [0.05, 0, 0.25] : t < 9 ? [-0.12, 0.5, 0] : [0.08, 0.8, 0];
  const { car } = fresh();
  // Rolling start at 5 m/s in first, as in golden_vehicle.rs: from rest the
  // clutch bites into wheelspin, which is the chattering regime the drive
  // is meant to stay out of.
  car.respawn(0, 0, 0, 5);
  const dt = golden.dt;
  const shifts = new Set(golden.shiftFrames);
  let worstPos = 0, worstVel = 0, worstRpm = 0, worstRim = 0, worstTrail = 0;
  const byFrame = new Map(golden.rows.map((r) => [r.f, r]));
  for (let f = 0; f < 12 * 60; f++) {
    const [steer, throttle, brake] = script(f * dt);
    if (shifts.has(f)) car.pt.requestUpshift();
    car.step(dt, { steer, throttle, brake });
    const g = byFrame.get(f);
    if (!g) continue;
    const t = car.telemetry;
    worstPos = Math.max(worstPos, Math.abs(car.X - g.x), Math.abs(car.Y - g.y), Math.abs(car.psi - g.psi));
    worstVel = Math.max(worstVel, Math.abs(car.u - g.u), Math.abs(car.v - g.v), Math.abs(car.r - g.r));
    worstRpm = Math.max(worstRpm, Math.abs(car.pt.engineRpm - g.rpm));
    worstRim = Math.max(worstRim, Math.abs(t.rimTorqueNm - g.rim));
    worstTrail = Math.max(worstTrail, Math.abs(t.trailFm - g.trail));
    if (car.pt.gear !== g.gear) worstRpm = 1e9;
  }
  check("worst pose difference", worstPos, 0, 1e-6, " m|rad");
  check("worst velocity difference", worstVel, 0, 1e-6, " m/s|rad/s");
  check("worst engine rpm difference", worstRpm, 0, 1e-3, " rpm");
  check("worst rim torque difference", worstRim, 0, 1e-6, " N.m");
  check("worst front trail difference", worstTrail, 0, 1e-8, " m");
}

// ------------------------------------------------------------ wheel presets ---
console.log("\nWHEEL PRESETS");
{
  const { presetFor, defaultGainFor, presetPaths, WHEEL_PRESETS } = await import("../src/game/wheelPresets.js");
  check("R5 recognised", presetFor("MOZA R5 Base").ratedNm, 5.5, 5.5, " N.m");
  check("R9 beats generic MOZA", presetFor("MOZA R9 Base").ratedNm, 9, 9, " N.m");
  check("G29 recognised", presetFor("Logitech G29 Driving Force Racing Wheel USB").ratedNm, 2.2, 2.2, " N.m");
  check("DD2 recognised", presetFor("Fanatec Podium Wheel Base DD2").ratedNm, 25, 25, " N.m");
  check("Simucube Pro recognised", presetFor("Simucube 2 Pro").ratedNm, 25, 25, " N.m");
  check("unknown falls back", presetFor("Some Wheel Co Model X").ratedNm, 5, 5, " N.m");
  check("gain on a 5.5 N.m base", defaultGainFor(5.5), 0.45, 0.55, "");
  check("gain on a 12 N.m base", defaultGainFor(12), 1, 1, "");
  check("gain floor on a 2 N.m base", defaultGainFor(2.2), 0.3, 0.3, "");
  let bad = 0;
  for (const p of WHEEL_PRESETS) {
    const paths = presetPaths(p);
    if (!(p.ratedNm > 0 && p.rotationDeg >= 180 && Number.isInteger(p.steerAxis))) bad++;
    if (paths["forceFeedback.maxForceNm"] !== p.ratedNm) bad++;
  }
  check("every preset is well formed", bad, 0, 0, "");
}

// ----------------------------------------------------------- steering feel ---
// Rim torque out of the vehicle model, which is what force feedback plays.
// A left turn must produce a torque that tries to steer back right, it must
// grow with lateral g, and it must fall away as the front tyres start to
// slide even while the lateral force is still near its peak.
console.log("\nSTEERING FEEL  (rim torque for force feedback)");
{
  const { ForceFeedback } = await import("../src/game/forceFeedback.js");
  const settle = (steer, v) => {
    const { car } = fresh();
    car.pt.gear = 2;
    car.respawn(0, 0, 0, v);
    for (let i = 0; i < 400; i++) car.step(1 / 200, { steer, throttle: 0.25, brake: 0 });
    return car.telemetry;
  };
  const straight = settle(0, 15);
  const gentle = settle(0.08, 15);
  const hard = settle(0.16, 15);
  check("no torque going straight", Math.abs(straight.rimTorqueNm), 0, 0.05, " N.m");
  check("left turn pulls the rim back right", -gentle.rimTorqueNm, 0.5, 30, " N.m");
  check("harder turn, more torque", hard.rimTorqueNm / gentle.rimTorqueNm, 1.2, 5, "x");
  check("rim torque at ~1 g", -gentle.rimTorqueNm / Math.max(gentle.ayG, 0.1), 2, 20, " N.m/g");
  // The claim force feedback rests on: the rim goes light BEFORE the front
  // lets go. Straight from the tyre, on one front tyre at its static load:
  // sweep slip angle, and the aligning moment (lateral force through the
  // pneumatic plus mechanical trail) must peak at a smaller slip angle than
  // the lateral force does, and be well down by the time the force peaks.
  // A trail that only collapsed after the force peak would fail this.
  const { tyreForces } = await import("../src/vehicle/tire.js");
  const FzTyre = gentle.FzF / 2;
  const mech = gentle.mechTrailM;
  let fyPeak = { a: 0, v: 0 }, mzPeak = { a: 0, v: 0 }, mzAtFyPeak = 0, mzLow = 0, fyLow = 0;
  for (let deg = 0.25; deg <= 16; deg += 0.25) {
    const f = tyreForces((deg * Math.PI) / 180, 0, FzTyre, SDM26.muLat, SDM26.muLong);
    const mz = f.fy * (f.trail + mech);
    if (deg === 0.25) { mzLow = mz; fyLow = f.fy; }
    if (f.fy > fyPeak.v) { fyPeak = { a: deg, v: f.fy }; mzAtFyPeak = mz; }
    if (mz > mzPeak.v) mzPeak = { a: deg, v: mz };
  }
  check("lateral force peaks at", fyPeak.a, 7, 10, " deg");
  check("aligning torque peaks before the force does", fyPeak.a - mzPeak.a, 2, 7, " deg");
  check("torque per N at the grip peak vs low slip", (mzAtFyPeak / fyPeak.v) / (mzLow / fyLow), 0.35, 0.75, "");
  check("mechanical trail", mech * 1000, 10, 25, " mm");

  // The mixer: with a wheel profile, a resting rim at zero slip commands zero,
  // damping opposes rim motion, and the end stop pushes back toward the lock.
  const cfg = {
    enabled: true, gain: 1, alignTorqueGain: 1, roadTextureGain: 0.35, damping: 0.15,
    friction: 0.04, softLockGain: 1, minForce: 0, maxForceNm: 5.5, invert: false,
  };
  const feel = { spin: 0, lock: 0, offTrack: false, coneHit: 0 };
  const ffb = new ForceFeedback();
  const still = ffb.update(1 / 60, cfg, straight, { deg: 0, halfLockDeg: 56 }, feel);
  check("mixer: straight and still commands nothing", Math.abs(still.command), 0, 1e-6, "");
  const turned = ffb.update(1 / 60, cfg, gentle, { deg: -20, halfLockDeg: 56 }, feel);
  check("mixer: left turn -> clockwise command", turned.command, 0.05, 1, "");
  const ffb2 = new ForceFeedback();
  ffb2.update(1 / 60, cfg, straight, { deg: 0, halfLockDeg: 56 }, feel);
  const moving = ffb2.update(1 / 60, cfg, straight, { deg: 30, halfLockDeg: 56 }, feel);
  check("mixer: damping opposes rim motion", -moving.damping, 0.01, 6, " N.m");
  const over = new ForceFeedback().update(1 / 60, cfg, straight, { deg: 70, halfLockDeg: 56 }, feel);
  check("mixer: end stop pushes back from over-lock", -over.softLock, 4, 6, " N.m");
  const kicked = new ForceFeedback().update(1 / 60, cfg, straight, { deg: 0, halfLockDeg: 56 }, { ...feel, coneHit: 1 });
  check("mixer: a cone kicks", Math.abs(kicked.kickNm), 1, 6, " N.m");
  // On the straight-line telemetry: texture is limited to the headroom left
  // under the aligning torque, and mid-corner on a 5.5 N.m base there is
  // little of it, which is the motor's problem and not the mixer's.
  const spun = new ForceFeedback().update(1 / 60, cfg, straight, { deg: 0, halfLockDeg: 56 }, { ...feel, spin: 1 });
  check("mixer: wheelspin makes texture", spun.textureNm, 0.2, 3, " N.m");
  const inv = new ForceFeedback().update(1 / 60, { ...cfg, invert: true }, gentle, { deg: -20, halfLockDeg: 56 }, feel);
  check("mixer: invert flips the command", inv.command / turned.command, -1.0001, -0.9999, "");
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
//     > sim/data/engine-audio-golden.json
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

// ------------------------------- engine audio: tonal balance and loudness ---
// The checks above prove the model is *correct* -- energy on the firing
// harmonics, no half-order, matching the Rust build. They say nothing about
// whether it is bearable, and twice now it was not: first clipped flat and
// dominated by a 3 kHz limit cycle, then correct but with idle almost as loud
// as the limiter.
//
// Loudness is measured A-WEIGHTED, not as raw RMS. That distinction is the
// whole reason the second problem went unnoticed: by raw energy idle was
// already 2.7x quieter than full throttle, but 30% of its A-weighted energy sat
// above 1.5 kHz against 1% of its raw energy. Ears are not energy meters, so a
// metric that ignores the ear cannot answer "does this sound too loud".
console.log("\nENGINE AUDIO  (tonal balance and loudness)");
{
  const FS = 48000;

  // Standard A-weighting: about -30 dB at 50 Hz, roughly flat at 2-4 kHz.
  const aWeight = (f) => {
    const f2 = f * f;
    const num = 12194 * 12194 * f2 * f2;
    const den =
      (f2 + 20.6 * 20.6) *
      Math.sqrt((f2 + 107.7 * 107.7) * (f2 + 737.9 * 737.9)) *
      (f2 + 12194 * 12194);
    return Math.pow(10, (20 * Math.log10(num / den) + 2.0) / 20);
  };

  const measure = (rpm, throttle, torque) => {
    const e = new EngineAudio(cbr600rrSdm26(), {});
    e.setOperatingPoint(rpm, throttle, torque);
    // Long warm-up: the resonance compressor is deliberately slow.
    const warm = new Float32Array(96000);
    e.render(warm);
    const buf = new Float32Array(8192);
    e.render(buf);

    let peak = 0;
    for (const v of buf) peak = Math.max(peak, Math.abs(v));

    let aTotal = 0;
    let aHigh = 0;
    let rawTotal = 0;
    let rawLow = 0;
    const n = 110;
    for (let k = 0; k < n; k++) {
      const f = 40 * Math.pow(12000 / 40, k / (n - 1));
      const w = (2 * Math.PI * f) / FS;
      let re = 0;
      let im = 0;
      for (let i = 0; i < buf.length; i++) {
        re += buf[i] * Math.cos(w * i);
        im += buf[i] * Math.sin(w * i);
      }
      const p = (re * re + im * im) / (buf.length * buf.length);
      aTotal += p * aWeight(f) ** 2;
      rawTotal += p;
      if (f > 1500) aHigh += p * aWeight(f) ** 2;
      else rawLow += p;
    }
    return {
      peak,
      dBA: 10 * Math.log10(aTotal + 1e-30),
      aHighPct: (100 * aHigh) / (aTotal || 1e-30),
      lowPct: (100 * rawLow) / (rawTotal || 1e-30),
    };
  };

  const at = {};
  for (const [label, rpm, throttle, torque] of [
    ["idle 2000", 2000, 0.14, 5.6],
    ["2500 part", 2500, 0.3, 20],
    ["4000 WOT", 4000, 1.0, 48],
    ["7000 WOT", 7000, 1.0, 58],
    ["10000 WOT", 10000, 1.0, 62],
    ["13000 WOT", 13000, 1.0, 55],
    ["3000 overrun", 3000, 0.0, 0],
    ["9000 overrun", 9000, 0.0, 0],
  ]) {
    at[label] = measure(rpm, throttle, torque);
  }

  for (const [label, m] of Object.entries(at)) {
    // An exhaust note is a bass instrument. When this fell to 3% at 3000 rpm
    // the result was a shriek.
    check(`${label}: energy below 1.5 kHz`, m.lowPct, 70, 100, " %");
    // Clipping. A peak-following leveller that could not react to a blowdown
    // transient once left every pulse on the rail, at RMS 0.99.
    check(`${label}: peak (headroom)`, m.peak, 0, 0.85, "");
    // The perceptually weighted high end. Idle sat at 30% here while looking
    // fine by raw energy, and that is what "whiny at idle" measures as.
    check(`${label}: A-weighted energy >1.5 kHz`, m.aHighPct, 0, 25, " %");
  }

  // Loudness has to rise with how hard the engine is working. A real car idles
  // 25-35 dB below full throttle; normalising every operating point to one
  // level is what made idle sound louder than the rest of the rev range.
  const limiter = at["13000 WOT"].dBA;
  check("idle below the limiter", limiter - at["idle 2000"].dBA, 14, 32, " dB");
  check("part throttle below the limiter", limiter - at["2500 part"].dBA, 6, 24, " dB");
  check("overrun below the limiter", limiter - at["3000 overrun"].dBA, 14, 40, " dB");
  check(
    "overrun below full throttle at the same rpm",
    at["9000 WOT"] ? at["9000 WOT"].dBA - at["9000 overrun"].dBA : 99,
    -99, 99, " dB",
  );

  // And it must not swing wildly across the rev range. Tuned-length resonance
  // is real and worth hearing, but the raw waveguide varies by 12 dB purely on
  // which pipe mode the firing harmonics land on, which had 10000 rpm louder
  // than the limiter. The compressor exists to bound that.
  const wot = ["4000 WOT", "7000 WOT", "10000 WOT", "13000 WOT"].map((k) => at[k].dBA);
  check("spread across full throttle", Math.max(...wot) - Math.min(...wot), 0, 8, " dB");
}


// ------------------------------------------------------- measured idle ------
// Daniel measured the car idling near 2000 rpm with the throttle plate at 14%.
// Those are two independent numbers, and the model has to reproduce BOTH from
// one of them -- otherwise the idle is scripted rather than emergent.
//
// It also pins the low-rpm end of the torque curve, which used to be a pure
// guess (0.35 of peak below the sweep's first point, a value that could not
// sustain an idle at any plate opening). Requiring a 14% plate to balance
// friction at 2000 rpm forces wot(2000) = 34 N.m, or 0.56 of peak -- which
// independently lands in the 55-70% a naturally aspirated four really makes
// there.
console.log("\nIDLE  (measured: ~2000 rpm at ~14% throttle plate)");
{
  const { pt } = fresh();

  // Let a free crank find its own equilibrium with the pedal at rest.
  let rpm = 1500;
  const I = SDM26.engineInertiaKgM2;
  for (let i = 0; i < 40000; i++) {
    rpm += (pt.engineTorque(rpm, 0) / I) * DT * (60 / (2 * Math.PI));
    rpm = Math.max(200, rpm);
  }
  check("idle settles at", rpm, 1850, 2200, " rpm");
  check("plate held at idle", pt.platePosition(rpm, 0) * 100, 13, 15, " %");

  // Engine braking must survive: the plate has to close off-throttle, or
  // holding the idle opening across the range would delete most of it.
  check("plate off-throttle at 6000 rpm", pt.platePosition(6000, 0) * 100, 0, 0.5, " %");
  check("engine drag at 6000 rpm off-throttle", -pt.engineTorque(6000, 0), 3, 20, " N.m");

  // Indicated torque is positive at idle even though net torque is zero --
  // which is the whole reason the sound model takes indicated, not net.
  check("indicated torque at idle", pt.indicatedTorque(rpm, 0), 3, 9, " N.m");
  check("net torque at idle", Math.abs(pt.engineTorque(rpm, 0)), 0, 0.5, " N.m");
}

// ------------------------------------------------------------ CAD import ---
// The glTF loader is exercised against a .glb built here in memory rather than
// against a file on disk. A fixture file would have to be committed, and the
// one thing it must NOT be is present in `data/` -- that is where a real CAD
// export goes, and a stray fixture there would silently replace a good
// procedural car with a box.
//
// What is checked is the part that is easy to get wrong and invisible when it
// is: a wheel's geometry has to come out centred on its own origin, because
// the renderer spins it about that origin. Left at its world position, a wheel
// orbits the car instead of rotating.
console.log("\nCAD IMPORT  (glTF binary loader)");
{
  // A minimal but complete .glb: one body triangle and one wheel triangle,
  // each on its own named node, with a material apiece.
  const buildGlb = (wheelOffset = [0, 0, 0], frame = null) => {
    const [ox, oy, oz] = wheelOffset;
    // `rot` transforms mesh-local geometry; `place` transforms node
    // translations. They differ by the assembly origin offset.
    const rot = frame ? frame.rot : (p) => p;
    const place = frame ? frame.place : (p) => p;
    const tri = (pts) => pts.flatMap((p) => rot(p));
    const positions = new Float32Array([
      // body triangle, around the origin
      0, 0.3, 0, 1, 0.3, 0, 0, 0.9, 0,
      // wheel triangle, displaced from its node origin by `wheelOffset`
      -0.2 + ox, -0.2 + oy, 0 + oz, 0.2 + ox, -0.2 + oy, 0 + oz, 0 + ox, 0.2 + oy, 0 + oz,
    ]);
    const normals = new Float32Array([
      0, 0, 1, 0, 0, 1, 0, 0, 1,
      0, 0, 1, 0, 0, 1, 0, 0, 1,
    ]);
    // Both triples are 0,1,2: glTF indices are relative to the accessor they
    // are used with, not to the buffer. Writing 3,4,5 for the second mesh
    // indexes past the end of its own position accessor and produces NaN
    // geometry -- which is how this fixture was wrong the first time.
    const indices = new Uint32Array([0, 1, 2, 0, 1, 2]);

    const bin = new Uint8Array(
      positions.byteLength + normals.byteLength + indices.byteLength,
    );
    bin.set(new Uint8Array(positions.buffer), 0);
    bin.set(new Uint8Array(normals.buffer), positions.byteLength);
    bin.set(new Uint8Array(indices.buffer), positions.byteLength + normals.byteLength);

    const doc = {
      asset: { version: "2.0", generator: "validate.js" },
      scene: 0,
      scenes: [{ nodes: [0, 1] }],
      nodes: [
        { name: "body", mesh: 0 },
        // The hub is well away from the origin: if the loader fails to
        // re-centre the geometry, the wheel's bounding box gives it away.
        // +Z is to the RIGHT, matching carmesh.js, so FL is at negative Z.
        { name: "wheel_fl", mesh: 1, translation: place([0.788, 0.2, -0.604]) },
        { name: "wheel_fr", mesh: 1, translation: place([0.788, 0.2, 0.604]) },
        { name: "wheel_rl", mesh: 1, translation: place([-0.742, 0.2, -0.604]) },
        { name: "wheel_rr", mesh: 1, translation: place([-0.742, 0.2, 0.604]) },
      ],
      meshes: [
        { name: "b", primitives: [{ attributes: { POSITION: 0, NORMAL: 2 }, indices: 4, material: 0 }] },
        { name: "w", primitives: [{ attributes: { POSITION: 1, NORMAL: 3 }, indices: 5, material: 1 }] },
      ],
      materials: [
        { name: "carbon", pbrMetallicRoughness: { baseColorFactor: [0.1, 0.11, 0.13, 1] } },
        { name: "rubber", pbrMetallicRoughness: { baseColorFactor: [0.06, 0.06, 0.07, 1] } },
      ],
      accessors: [
        { bufferView: 0, componentType: 5126, count: 3, type: "VEC3" },
        { bufferView: 0, byteOffset: 36, componentType: 5126, count: 3, type: "VEC3" },
        { bufferView: 1, componentType: 5126, count: 3, type: "VEC3" },
        { bufferView: 1, byteOffset: 36, componentType: 5126, count: 3, type: "VEC3" },
        { bufferView: 2, componentType: 5125, count: 3, type: "SCALAR" },
        { bufferView: 2, byteOffset: 12, componentType: 5125, count: 3, type: "SCALAR" },
      ],
      bufferViews: [
        { buffer: 0, byteOffset: 0, byteLength: positions.byteLength },
        { buffer: 0, byteOffset: positions.byteLength, byteLength: normals.byteLength },
        { buffer: 0, byteOffset: positions.byteLength + normals.byteLength, byteLength: indices.byteLength },
      ],
      buffers: [{ byteLength: bin.byteLength }],
    };

    const enc = new TextEncoder();
    let json = enc.encode(JSON.stringify(doc));
    const jsonPad = (4 - (json.length % 4)) % 4;
    const jsonChunk = new Uint8Array(json.length + jsonPad).fill(0x20);
    jsonChunk.set(json, 0);
    const binPad = (4 - (bin.length % 4)) % 4;
    const binChunk = new Uint8Array(bin.length + binPad);
    binChunk.set(bin, 0);

    const total = 12 + 8 + jsonChunk.length + 8 + binChunk.length;
    const out = new Uint8Array(total);
    const dv = new DataView(out.buffer);
    dv.setUint32(0, 0x46546c67, true);
    dv.setUint32(4, 2, true);
    dv.setUint32(8, total, true);
    dv.setUint32(12, jsonChunk.length, true);
    dv.setUint32(16, 0x4e4f534a, true);
    out.set(jsonChunk, 20);
    dv.setUint32(20 + jsonChunk.length, binChunk.length, true);
    dv.setUint32(24 + jsonChunk.length, 0x004e4942, true);
    out.set(binChunk, 28 + jsonChunk.length);
    return out.buffer;
  };

  const glb = buildGlb();
  const { doc } = parseGlb(glb);
  check("parses the container", doc.asset.version === "2.0" ? 1 : 0, 1, 1, "");

  const car = buildCarFromGlb(glb);
  check("no import problems", car.stats.problems.length, 0, 0, "");
  check("body geometry present", car.body.count, 3, 3, " verts");
  check("all four hubs found", car.hubs ? car.hubs.length : 0, 4, 4, "");

  // Hub positions come from the file, not from the vehicle parameters.
  const fl = car.hubs.find((h) => h.name === "FL");
  check("front hub x from the file", fl.x, 0.787, 0.789, " m");
  check("FL is on the -Z side", fl.z, -0.606, -0.602, " m");
  check("front hub flagged front", fl.front ? 1 : 0, 1, 1, "");
  const rl = car.hubs.find((h) => h.name === "RL");
  check("rear hub flagged rear", rl.front ? 0 : 1, 1, 1, "");

  // The check that matters: wheel geometry centred on its own origin.
  let lo = [1e9, 1e9, 1e9];
  let hi = [-1e9, -1e9, -1e9];
  for (let i = 0; i < car.tire.position.length; i += 3) {
    for (let k = 0; k < 3; k++) {
      lo[k] = Math.min(lo[k], car.tire.position[i + k]);
      hi[k] = Math.max(hi[k], car.tire.position[i + k]);
    }
  }
  const offCentre = Math.max(...lo.map((v, k) => Math.abs((v + hi[k]) / 2)));
  check("wheel centred on its own origin", offCentre, 0, 1e-6, " m");

  // Material colours reach the vertices, or everything renders default grey.
  check("wheel takes its material colour", car.tire.color[0], 0.059, 0.061, "");
  check("body takes its material colour", car.body.color[1], 0.109, 0.111, "");

  // A wheel whose geometry is NOT centred on its node must still spin about
  // its own axle rather than orbiting the car.
  //
  // This is the check that lets a CAD export skip a fiddly step. Part origins
  // do not survive STEP as object origins, so a wheel usually arrives with its
  // geometry sitting wherever the assembly put it. Measuring the geometry and
  // correcting the hub to match means the model does not have to be right about
  // this, and the wheel still appears exactly where it was modelled.
  {
    const offset = [0.05, -0.03, 0.02];
    const shifted = buildGlb(offset);
    const car2 = buildCarFromGlb(shifted);

    check("measures the wheel's own offset", car2.wheelOffset[0], 0.049, 0.051, " m");

    let lo2 = [1e9, 1e9, 1e9];
    let hi2 = [-1e9, -1e9, -1e9];
    for (let i = 0; i < car2.tire.position.length; i += 3) {
      for (let k = 0; k < 3; k++) {
        lo2[k] = Math.min(lo2[k], car2.tire.position[i + k]);
        hi2[k] = Math.max(hi2[k], car2.tire.position[i + k]);
      }
    }
    const off2 = Math.max(...lo2.map((v, k) => Math.abs((v + hi2[k]) / 2)));
    check("off-centre wheel is re-centred", off2, 0, 1e-6, " m");

    // And the car must not MOVE because of it. The hub takes back what was
    // removed from the geometry, and the frame solve -- which runs afterwards
    // and reads the corrected hubs -- absorbs a uniform offset entirely. So the
    // wheel ends up exactly where it belongs rather than 50 mm forward of it,
    // which is the outcome that actually matters.
    const fl2 = car2.hubs.find((h) => h.name === "FL");
    check("an off-centre wheel does not move the car", fl2.x, 0.786, 0.790, " m");
  }


  // An export in ANY frame must come out in the simulator's.
  //
  // This is what makes the CAD requirements small enough to be worth meeting.
  // A SolidWorks assembly has no reason to share this simulator's idea of
  // forward, up, or where the origin belongs, and the four wheel hubs pin all
  // of that down exactly -- so the frame is solved from the file rather than
  // demanded of whoever exported it.
  {
    // Z-up, millimetres, facing +Y, origin displaced. About as unlike the
    // simulator's frame as a real export gets.
    const MM = 1000;
    const OFF = [250, -700, 0];
    // Rotation and scale only, for mesh-local geometry: the assembly origin
    // offset belongs on the node, not baked into the vertices. Putting it in
    // both is how this fixture was wrong the first time, and the loader
    // faithfully reported the resulting nonsense.
    const rot = (p) => [p[2] * MM, p[0] * MM, p[1] * MM];
    const place = (p) => rot(p).map((v, i) => v + OFF[i]);

    const glbSw = buildGlb([0, 0, 0], { rot, place });
    const fitted = buildCarFromGlb(glbSw);

    check("solves a foreign frame", fitted.hubs ? 1 : 0, 1, 1, "");
    check("detects the unit scale", fitted.frame.scale, 0.00099, 0.00101, "");

    const flf = fitted.hubs.find((h) => h.name === "FL");
    check("fitted front hub x", flf.x, 0.786, 0.790, " m");
    check("fitted front hub y", flf.y, 0.198, 0.202, " m");
    check("fitted FL still on the -Z side", flf.z, -0.606, -0.602, " m");
    const rrf = fitted.hubs.find((h) => h.name === "RR");
    check("fitted rear hub x", rrf.x, -0.744, -0.740, " m");
    check("fitted RR still on the +Z side", rrf.z, 0.602, 0.606, " m");

    // Up must come out up. Getting the cross-product handedness wrong produced
    // an up vector pointing at the ground, and a mirrored car -- which on a
    // symmetric model is entirely invisible.
    check("body sits above the ground", Math.min(...fitted.body.position.filter((_, i) => i % 3 === 1)), -0.01, 2, " m");
  }
  // A non-glb must be refused with a message, not parsed into nonsense.
  let refused = 0;
  try {
    parseGlb(new TextEncoder().encode('{"asset":{"version":"2.0"}}').buffer);
  } catch {
    refused = 1;
  }
  check("a .gltf (JSON) file is refused", refused, 1, 1, "");
}
console.log(`\n${failures === 0 ? "ALL CHECKS PASSED" : `${failures} CHECK(S) FAILED`}\n`);
process.exit(failures === 0 ? 0 : 1);
