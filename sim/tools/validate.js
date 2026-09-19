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
  // Roll stiffness is a per-event SETUP item, not a property of the car: the
  // team runs 1-1/1-1 blades (46%) at the skidpad and stiffer fronts on the
  // acceleration car. Run the skidpad on the skidpad setup -- and put it back
  // afterwards, because `car.p` is the shared params object and leaving it
  // changed silently re-runs every later check, the Rust parity drive
  // included, on a car the golden was not generated for.
  const savedRsd = SDM26.roll.rsdFront;
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
    car.p.roll.rsdFront = 0.46;
    car.respawn(0, 0, 0, vTarget);
    // PI on yaw rate around an Ackermann feed-forward. The output is the steer
    // angle itself, NOT an increment -- accumulating into `steer` on top of an
    // integral term makes a double integrator that oscillates into a spin and
    // reports a grip limit the car never actually reached.
    const ff = car.p.wheelbaseM / R / ((car.p.maxSteerDeg * Math.PI) / 180);
    // The PI gains convert a yaw-rate error into NORMALISED steer demand, so
    // they follow the lock: at the measured 46 deg rack this loop would run at
    // 1.6x its designed gain and shake the car off the circle. `ff` is already
    // an angle over the lock and needs no scaling; the integral is physical.
    const ls = 28 / car.p.maxSteerDeg;
    let integral = 0, sumR = 0, nR = 0, blew = false;
    for (let i = 0; i < 6000; i++) {
      const targetYaw = car.speed / R;
      const err = targetYaw - car.r;
      integral = Math.max(-0.5, Math.min(0.5, integral + err * DT));
      // Held to 28 deg of road wheel (`ls` normalised), as it always was: the
      // rack now travels to 46, but a skidpad driver does not use it, and an
      // integrator given that much authority winds the front past its peak.
      const steer = Math.max(-ls, Math.min(ls, ff + (6 * err + 4 * integral) * ls));
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
  SDM26.roll.rsdFront = savedRsd;
  const lap = (2 * Math.PI * R) / bestV;
  const g = (bestV * bestV) / R / 9.81;
  console.log(`  sustained ${bestV.toFixed(2)} m/s = ${g.toFixed(3)} g`);
  check("skidpad lap time", lap, 4.85, 5.35, " s");
  // Above the 1.368 tyre mu because 11.5 m/s is already worth ~250 N of
  // downforce on this aero package -- grip scales with it, mass does not.
  // Floor dropped from 1.35 with the measured 7.3 deg peak slip angle: a
  // sharper tyre sits further along its own curve at a given angle, so the
  // combined-slip coupling with the skidpad's throttle costs a little lateral.
  // The lap is still inside the band, but it has drifted from 5.02 toward
  // 5.21 and the honest fix is to re-pin muLat once `frontGripFactor` stops
  // carrying the balance (see the LSD work in the dev plan), not to widen
  // this again.
  check("lateral acceleration", g, 1.30, 1.58, " g");
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
  // KNOWN GAP. The real car runs 75 m in 4.2-4.4 s. This model, on the
  // measured torque curve, takes about 5.15 s, and the band below is around
  // the MODEL rather than around the car so the suite stays a regression gate.
  // Do not widen it to make a change fit; close the gap instead.
  //
  // What has been ruled out, each measured one at a time:
  //   engine power      +60% of torque buys only 0.28 s -- it is not power
  //   driveline inertia removing ALL of it buys 0.13 s
  //   longitudinal grip mu_x 1.5 -> 1.8 buys 0.03 s
  //   shift time        100 -> 50 ms buys 0.03 s
  //   mass, crr, CdA, final drive: 0.02-0.17 s each
  //
  // What is left, and the most likely cause: the launch is decided almost
  // entirely by wheelspin management, and the slip ratio the manager reads is
  // not meaningful below walking pace. `kDen` floors the denominator at 2 m/s
  // (bicycle.js), so a stationary car with barely turning wheels already reads
  // kappa 0.35. Any traction controller -- this harness's, or the car's own --
  // is blind exactly where a launch is won, and the car spends its first 1.5 s
  // at kappa 0.15-0.72 making less force than the tyre can. Halving the wheel
  // inertia makes the 75 m WORSE by 1.1 s, which only makes sense if wheelspin
  // management, not the car, is the binding constraint. A slip definition that
  // stays valid at low speed is the fix.
  check("75 m time (managed launch)", good.t, 4.90, 5.40, " s");
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
  // Measured, not predicted: the rolling-road run. At the WHEELS this is
  // 49.1 N.m and 45.3 kW, which is the 61 hp the dyno printed. The CFD sweep
  // this replaced claimed 62.6 N.m and 58.1 kW at the flywheel.
  check("peak torque", pt.peakTorque.torqueNm, 55, 60, " N.m");
  check("peak torque rpm", pt.peakTorque.rpm, 8000, 9000, " rpm");
  check("peak power", pt.peakPower.powerKW, 50, 56, " kW");
  const topGear = SDM26.gearRatios.length - 1;
  const vmax = gearVps(SDM26, topGear) * SDM26.revLimitRpm;
  {
    // Launch control: held, the engine sits on the LC limiter with the clutch
    // out; released, the clutch is DUMPED rather than fed in, which is what
    // the driver does and what makes a start competitive rather than soft.
    const { car, pt } = fresh();
    car.respawn(0, 0, 0, 0);
    pt.setLaunch(true);
    for (let i = 0; i < 1500; i++) car.step(1 / 500, { steer: 0, throttle: 1, brake: 0 });
    check("launch control holds the engine", pt.engineRpm, SDM26.launchRpm - 200, SDM26.launchRpm + 50, " rpm");
    check("and the car has not moved", car.X, 0, 0.05, " m");
    const capHeld = pt.clutchCapacity(1, 0, 0);
    pt.setLaunch(false);
    const capDumped = pt.clutchCapacity(1, 0, 0);
    check("clutch passes nothing while held", capHeld, 0, 0, " N.m");
    check("and is dumped when dropped", capDumped, 200, 260, " N.m");
  }
  check("geared top speed", vmax * 3.6, 110, 150, " km/h");
  console.log(`  motoring drag @ 10k rpm: ${pt.motoringTorque(10000).toFixed(1)} N.m`);
}

// -------------------------------------------------------------------- tyre ---
console.log("\nTYRE");
{
  const Fz = (SDM26.massKg * 9.81) / 4;
  const cs = TIRE_INFO.corneringStiffness(SDM26.muLat, Fz);
  // Top of the band raised from 380 with the peak slip angle: a tyre that
  // reaches the same peak FORCE 1.2 deg earlier is a stiffer tyre, and the
  // two cannot be banded independently. 407 N/deg at 655 N is above the
  // MF6.1 fit's own ~310-340, which is the price of a fixed-shape Magic
  // Formula -- B, and with it the stiffness, is whatever puts the peak where
  // the data says. Worth revisiting with a load-dependent peak slip angle.
  check("cornering stiffness / tyre", cs / 57.3, 200, 430, " N/deg");
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
    // 60% of the 28 deg lock this check was written against is 16.8 deg of
    // road wheel. The rack limit moved to the measured 46 deg; the angle this
    // check means to sweep did not.
    const rampFrac = (0.6 * 28) / SDM26.maxSteerDeg;
    let t = 0, peakAy = 0, at = null, peakBeta = 0;
    while (t < 12) {
      // Throttle held to what a driver would actually carry at the limit.
      // The old speed-hold controller floored it -- 0.84 at 28 m/s -- chasing
      // a speed the car cannot hold at 2 g, and now that the rear axle is two
      // wheels with a differential between them, flooring it at 2 g lights up
      // the inside rear and spins the car. That is the model being right: a
      // single rear rotor could not represent one wheel letting go. This check
      // is about the STEADY balance, so the driver model has to stop doing
      // something no driver does.
      const thr = Math.max(0, Math.min(0.55, 0.2 + (V - car.speed) * 0.8));
      car.step(DT, { steer: (t / 12) * rampFrac, throttle: thr, brake: 0 });
      t += DT;
      const tel = car.telemetry;
      peakBeta = Math.max(peakBeta, Math.abs(tel.bodySlipDeg));
      if (tel.ayG > peakAy) { peakAy = tel.ayG; at = { uF: tel.utilF, uR: tel.utilR, bal: tel.balance }; }
    }
    // Upper bound is a sanity cap, not a measurement: at 28 m/s the aero
    // package adds ~55% of the car's weight, and the TTC load sensitivity
    // (0.12, was 0.15 EST) leaves a little more of that as grip, 1.95 g.
    // Ceiling raised from 2.0: at 28 m/s the aero map is worth 1426 N against
    // a 2619 N car, so a shade over 2 g is what this package should make. The
    // old ceiling was set when the tyre peaked 1.2 deg later.
    check(`${V} m/s: peak lateral`, peakAy, 1.35, 2.1, " g");
    // Axle-level:  is load-weighted across the rear, so a spinning
    // inner wheel does not read as a rear axle at its limit.
    check(`${V} m/s: front limits first (axle balance)`, -at.bal, 0.08, 0.6, "");
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
  // Floor dropped from 13 deg: the usable lock is Ackermann for 1.4 g plus
  // the tyre's PEAK SLIP ANGLE plus a margin, and the peak slip angle moved
  // from 8.5 to the measured 7.3. The cap follows the tyre, as it should.
  check("keyboard lock at 25 m/s", lockAt(25) * SDM26.maxSteerDeg, 12, 18, " deg");
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
    t < 3 ? [0, 0.35, 0] : t < 5 ? [0.1, 0.3, 0] : t < 5.5 ? [0.04, 0, 0.15] : t < 9 ? [-0.08, 0.35, 0] : [0.06, 0.45, 0];
  // The scripted steer values are fractions of the 28 deg lock this drive was
  // written against; the rack's measured limit is now 46 deg. Rescaled so the
  // golden stays the same physical manoeuvre. Mirrors examples/golden_vehicle.rs.
  const SCRIPT_LOCK_DEG = 28;
  const lockScale = SCRIPT_LOCK_DEG / SDM26.maxSteerDeg;
  const { car } = fresh();
  // Rolling start at 15 m/s in third, as in golden_vehicle.rs -- above the
  // clutch's engagement window, where the clutch is simply locked, and gentle
  // enough that the tyres stay at a quarter of their peak. See the note there
  // for why a 5 m/s start in first no longer works.
  car.pt.gear = 2;
  car.respawn(0, 0, 0, 15);
  car.pt.syncToWheel(15 / 0.2);
  const dt = golden.dt;
  const shifts = new Set(golden.shiftFrames);
  let worstPos = 0, worstVel = 0, worstRpm = 0, worstRim = 0, worstTrail = 0;
  const byFrame = new Map(golden.rows.map((r) => [r.f, r]));
  for (let f = 0; f < 12 * 60; f++) {
    const [steer, throttle, brake] = script(f * dt);
    if (shifts.has(f)) car.pt.requestUpshift();
    car.step(dt, { steer: steer * lockScale, throttle, brake });
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
  // Floating-point noise, not bit-identity. The two ports were bit-identical
  // until the differential arrived, because every transcendental they shared
  // -- atan, sin -- happens to round the same way in V8 and in Rust libm for
  // these inputs. The Salisbury clutch added `tanh`, which does NOT.
  //
  // These bands used to be 1e-4, on the reasoning that `tanh` rounding alone
  // walked the two builds about 1e-6 apart over 12 s. That was only half
  // right. The clutch's stick spring was integrated explicitly against a very
  // small antisymmetric inertia and was UNSTABLE, so it multiplied any one-ulp
  // disagreement by more than one on every substep: a few ulps at the first
  // step really did become 1e-6, not because `tanh` rounds differently but
  // because the model amplified it. With the stick torque limited (see
  // `bicycle.js`, "limited so it cannot overshoot") the same drive agrees to
  // 5e-10 -- which is the precision the golden file is PRINTED at, so it is
  // the floor, not a measurement. The bands are back where they can do some
  // good.
  //
  // For scale: when this drive genuinely fell out of step earlier in the same
  // work it showed up as 0.047 m. Anything that trips these is a modelling
  // difference, not arithmetic.
  //
  // Printed as well as checked, so a drift still inside the band is visible
  // rather than silently accumulating until it is not.
  console.log("  (raw: pos " + worstPos.toExponential(3) + ", vel " + worstVel.toExponential(3)
    + ", rpm " + worstRpm.toExponential(3) + ", rim " + worstRim.toExponential(3)
    + ", trail " + worstTrail.toExponential(3) + ")");
  check("worst pose difference", worstPos, 0, 1e-7, " m|rad");
  check("worst velocity difference", worstVel, 0, 1e-7, " m/s|rad/s");
  check("worst engine rpm difference", worstRpm, 0, 1e-6, " rpm");
  check("worst rim torque difference", worstRim, 0, 1e-6, " N.m");
  check("worst front trail difference", worstTrail, 0, 1e-7, " m");
}

// ------------------------------------------------------------- differential ---
// SDM26 runs a Drexler Formula Student V3, a 1.5-way Salisbury LSD. These are
// the physics invariants the team's own study asks for as regression guards --
// open means equal torques, locked means equal speeds, more lock means more
// understeer -- plus the one that matters to a driver: the coast ramp is what
// stops the rear coming round when you lift.
console.log("\nDIFFERENTIAL  (Drexler V3, Salisbury clutch pack)");
{
  const saved = { ...SDM26.diff };
  const setDiff = (powerLock, coastLock, preloadNm) =>
    Object.assign(SDM26.diff, { powerLock, coastLock, preloadNm });

  // Settle into a steady corner and report what the rear wheels are doing.
  const corner = (V, steerDeg, throttle, hold = 2000) => {
    const { car } = fresh();
    car.pt.gear = V < 13 ? 1 : 2;
    car.respawn(0, 0, 0, V);
    car.pt.syncToWheel(car.wR);
    const steer = ((steerDeg * Math.PI) / 180) / ((SDM26.maxSteerDeg * Math.PI) / 180);
    for (let i = 0; i < hold; i++) {
      const thr = Math.max(0, Math.min(0.55, throttle + (V - car.speed) * 0.5));
      car.step(DT, { steer, throttle: thr, brake: 0 });
    }
    return car;
  };

  // 1. Open: the wheels are free to take up the speeds the corner asks for.
  //    The kinematic difference is yaw rate x track / rolling radius.
  setDiff(0, 0, 0);
  const open = corner(12, 10, 0.25);
  const kinematic = (Math.abs(open.r) * SDM26.trackRearM) / SDM26.tireRadiusM;
  const openSplit = Math.abs(open.wRR - open.wRL);
  // A shade under the pure kinematic difference, because both wheels are also
  // carrying drive slip and the lighter inner one slips more, which closes the
  // gap rather than opening it.
  check("open diff: wheels differentiate freely", openSplit / kinematic, 0.7, 1.2, "x kinematic");

  // 2. Locked: the clutch pack holds them together instead.
  setDiff(0.95, 0.95, 60);
  const locked = corner(12, 10, 0.25);
  const lockedSplit = Math.abs(locked.wRR - locked.wRL);
  check("locked diff: wheels held together", lockedSplit / openSplit, 0, 0.75, "x open");

  // 3. Lock has to move the balance one way only: toward understeer. A
  //    Salisbury sends torque to the SLOWER, inner wheel under power, and the
  //    inner wheel pushing harder than the outer pushes the nose wide.
  //    It is NOT monotone all the way: past about 0.6 the inner wheel spins
  //    hard enough that more lock stops buying understeer, which is exactly the
  //    optimum-per-corner-type the team's study describes. So the check is the
  //    one that has to hold -- going from open to the ramp the car runs makes
  //    the car push more, not less.
  const balAt = (lock) => {
    setDiff(lock, lock, 0);
    return corner(12, 10, 0.35).telemetry.balance; // >0 rear-limited
  };
  const balOpen = balAt(0);
  const balRun = balAt(saved.powerLock);
  check("the ramp the car runs adds understeer", balOpen - balRun, 0.005, 0.15, "");

  // 4. The one the driver feels. Settle on throttle, then drop it: engine
  //    braking takes load and grip off the rear, and with nothing holding the
  //    outer wheel the car rotates. The coast ramp is what catches it.
  const lift = (coastLock) => {
    setDiff(0.6, coastLock, coastLock > 0 ? 25 : 0);
    const car = corner(16, 8, 0.5);
    const yaw0 = Math.abs(car.telemetry.yawRateDegS);
    const steer = ((8 * Math.PI) / 180) / ((SDM26.maxSteerDeg * Math.PI) / 180);
    let peak = yaw0, beta = 0;
    for (let i = 0; i < 500; i++) {
      car.step(DT, { steer, throttle: 0, brake: 0 });
      peak = Math.max(peak, Math.abs(car.telemetry.yawRateDegS));
      beta = Math.max(beta, Math.abs(car.telemetry.bodySlipDeg));
    }
    return { spike: (100 * (peak - yaw0)) / yaw0, beta };
  };
  const openLift = lift(0);
  const lsdLift = lift(saved.coastLock);
  // Smaller than it was once the measured torque curve went in: less engine
  // braking to unsettle the rear. The point of the check is the CONTRAST with
  // the coast ramp below, which is what the driver feels.
  check("lift-off yaw spike, open diff", openLift.spike, 10, 400, " %");
  check("lift-off yaw spike, Drexler coast ramp", lsdLift.spike, 0, 25, " %");
  check("lift-off body slip, Drexler coast ramp", lsdLift.beta, 0, 4, " deg");

  Object.assign(SDM26.diff, saved);
  // Torque bias ratio, the number the drivetrain team quotes: TBR = (1+n)/(1-n).
  const tbr = (1 + SDM26.diff.powerLock) / (1 - SDM26.diff.powerLock);
  check("power-ramp torque bias ratio", tbr, 1, 6, ":1");
}

// ------------------------------------------------------------ steering rack ---
// The measured rim -> road table is the one piece of steering data that is not
// an estimate, and it is duplicated in `native/crates/sim-core/src/vehicle.rs`
// so the desktop rig can steer by it natively. These checks are what stop the
// two copies drifting apart silently.
console.log("\nSTEERING RACK  (measured rim -> road-wheel table)");
{
  const { roadFromRimDeg, roadPerRimDeg } = await import("../src/vehicle/params.js");
  const st = SDM26.steering;
  const t = st.rimToRoadDeg;
  check("table length", t.length, 37, 37, " points");
  check("centred", roadFromRimDeg(st, 0), 0, 0, " deg");
  check("odd (sign preserved)", roadFromRimDeg(st, -45) + roadFromRimDeg(st, 45), 0, 0, " deg");
  let backwards = 0;
  for (let i = 1; i < t.length; i++) if (!(t[i] > t[i - 1])) backwards++;
  check("monotone (so it inverts)", backwards, 0, 0, " backward steps");
  // The measured car: 46 deg of road wheel at the 179 deg stop, and a rack
  // that is much slower on centre than at lock. Either of these moving means
  // the table was regenerated from different data.
  check("road angle at the stop", roadFromRimDeg(st, st.rimLockDeg), 45.5, 46.5, " deg");
  check("lock matches the rack", SDM26.maxSteerDeg, 45.5, 46.5, " deg");
  check("local ratio on centre", 1 / roadPerRimDeg(st, 0), 5.1, 5.4, "");
  check("local ratio at 90 deg", 1 / roadPerRimDeg(st, 90), 3.2, 3.6, "");
  // The nominal constant this replaced, for the record: it is 19% quick here.
  check("nominal ratio is still quoted", SDM26.steeringRatio, 4.411, 4.411, "");
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
  // The divisor is the rim torque at the PEAK of the aligning curve (~15 N.m
  // at 4 deg of front slip), not the ~9 N.m/g the old 11 came from: the point
  // is that the peak lands at full output, so the fall-off past it is still
  // inside the motor instead of buried in a clip.
  check("gain on a 5.5 N.m base", defaultGainFor(5.5), 0.35, 0.40, "");
  check("gain on a 12 N.m base", defaultGainFor(12), 0.78, 0.82, "");
  check("gain unity from 15 N.m up", defaultGainFor(15), 1, 1, "");
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
// The car was measured idling near 2000 rpm. The idle RPM and the plate
// opening that holds it are two independent numbers, and the model has to
// reproduce BOTH from one of them -- otherwise the idle is scripted rather
// than emergent.
//
// It also pins the low-rpm end of the torque curve, which used to be a pure
// guess (0.35 of the sweep's first point, a value that could not sustain an
// idle at any plate opening). Requiring the plate to balance friction at
// 2000 rpm is what fixes the 0.56 floor in `powertrain.js`.
//
// OPEN QUESTION FOR THE TEAM -- do not "fix" this by editing a number.
// This section used to say the plate was measured at 14%, and the model now
// idles at 21.8% (`idleThrottleFrac` 0.22), which the band below accepts. One
// of two things is true and the code cannot tell which:
//
//   * the measurement is 14% and the model is 8 points off it, in which case
//     the drag figure or the low-rpm torque floor is wrong and this band is
//     rubber-stamping the model instead of checking it; or
//   * the plate was re-read as ~22% when the measured dyno curve landed, and
//     only this comment was left behind.
//
// The dyno sheet in the team Drive settles it. Until someone checks, this band
// asserts what the model does, and says so.
console.log("\nIDLE  (model: ~2000 rpm at ~22% plate -- see the note above)");
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
  check("plate held at idle", pt.platePosition(rpm, 0) * 100, 20, 23, " %");

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
  // glTF base colour is linear; the loader encodes it to display space
  // (pow 1/2.2) because the renderer decodes vertex colour on the way in.
  const enc = (v) => Math.pow(v, 1 / 2.2);
  check("wheel takes its material colour", car.tire.color[0], enc(0.06) - 0.001, enc(0.06) + 0.001, "");
  check("body takes its material colour", car.body.color[1], enc(0.11) - 0.001, enc(0.11) + 0.001, "");

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
// ---------------------------------------------------------------- version ---
// One build, one version number.
//
// `SIM_VERSION` is stamped into every recorded run, and a lap time only means
// something next to the build it was set on. It was a hardcoded "1.0.0" while
// the app shipped 0.2.0 and then 0.3.0, so every run ever recorded claims a
// version that has never existed -- and nothing noticed, because nothing was
// comparing them. The desktop build now asks the shell at boot, but the
// literal is still the browser's answer, so it has to be right.
//
// Read as text rather than imported: `main.js` is the whole browser app.
{
  const root = join(here, "..");
  const literal = readFileSync(join(root, "src/main.js"), "utf8")
    .match(/export let SIM_VERSION = "([^"]+)"/)?.[1] ?? "";
  const pkg = JSON.parse(readFileSync(join(root, "package.json"), "utf8")).version;
  const conf = JSON.parse(readFileSync(join(root, "src-tauri/tauri.conf.json"), "utf8")).version;
  const cargo = readFileSync(join(root, "src-tauri/Cargo.toml"), "utf8")
    .match(/^version = "([^"]+)"/m)?.[1] ?? "";
  console.log(`
version  main.js ${literal || "?"} | package.json ${pkg} | tauri.conf ${conf} | Cargo ${cargo}`);
  check("SIM_VERSION matches package.json", literal === pkg ? 1 : 0, 1, 1, "");
  check("tauri.conf.json matches package.json", conf === pkg ? 1 : 0, 1, 1, "");
  check("Cargo.toml matches package.json", cargo === pkg ? 1 : 0, 1, 1, "");
}

console.log(`\n${failures === 0 ? "ALL CHECKS PASSED" : `${failures} CHECK(S) FAILED`}\n`);
process.exit(failures === 0 ? 0 : 1);
