// Transient bicycle model.
//
// Twelve integrated states, semi-implicit at a fixed 500 Hz:
//
//   u, v      body-frame longitudinal / lateral velocity   (m/s)      2
//   r         yaw rate                                     (rad/s)    1
//   wF, wR    front / rear axle wheel speed                (rad/s)    2
//   aF, aR    relaxation-lagged slip angles                (rad)      2
//   delta     road-wheel steer angle                       (rad)      1
//   X, Y, psi global pose                                  (m, m, rad) 3
//   plus the crankshaft speed carried by Powertrain                   1
//
// Mechanically that is 3 chassis degrees of freedom (surge, sway, yaw), 2 wheel
// rotational DOF, and 1 driveline DOF that only exists while the clutch slips.
// Heave, pitch and roll are NOT degrees of freedom here -- they are applied as
// deg/g gradients, so there is no ride model.
//
// "Transient" is meant literally and shows up in four places:
//
//   1. The lateral equation keeps the m*u*r term, so the car has a real yaw
//      response with overshoot rather than a steady-state cornering balance.
//   2. Wheel speeds are states, so slip ratio is dynamic -- you can spin the
//      rears up and you can lock a wheel under braking.
//   3. Slip angles pass through a relaxation-length lag, so tyre force builds
//      over roughly the first third of a metre of travel after a steer input.
//   4. Load transfer is fed by the previous step's measured accelerations, so
//      it settles rather than teleporting.
//
// The bicycle model lumps each axle into one tyre, but grip is still computed
// from the LEFT/RIGHT load split: `axleMu` derates each axle by how much
// lateral transfer it is carrying, using the SDM26 roll-stiffness distribution.
// That keeps the balance sensitive to the ARB setting even though there are
// only two contact patches in the equations.

import { lengthToFrontAxle, lengthToRearAxle, nominalTyreLoad } from "./params.js";
import { axleMu, muAtLoad, pneumaticTrail, tyreForces } from "./tire.js";

const G = 9.81;
const SUBSTEP = 1 / 500;

/** Finite-difference step in slip ratio for the wheel update's implicit term. */
const KAPPA_H = 1e-4;
/** Speed above which the steering slip cap is at its full (narrow) width. */
const SLIP_CAP_FULL_MPS = 8;

export class BicycleModel {
  constructor(params, powertrain) {
    this.p = params;
    this.pt = powertrain;

    this.refresh();
    this.reset(0, 0, 0);
  }

  /**
   * Recompute everything derived from the parameters.
   *
   * Called every substep rather than once in the constructor, because the
   * parameters are live: mass, wheelbase, unsprung and the axle inertias can
   * all be edited while the car is moving. Caching these once meant a slider
   * appeared to do nothing. It is half a dozen divisions at 500 Hz.
   */
  refresh() {
    const p = this.p;
    this.a = lengthToFrontAxle(p);   // CG -> front axle
    this.b = lengthToRearAxle(p);    // CG -> rear axle
    this.Fz0 = nominalTyreLoad(p);
    this.mSprung = p.massKg - 2 * (p.unsprungFrontKg + p.unsprungRearKg);
    this.IwF = 2 * p.wheelInertiaFrontKgM2;
    this.IwR = 2 * p.wheelInertiaRearKgM2;
  }

  /**
   * Steering servo limits from the active control profile, or null to derive
   * them from the vehicle parameters. Configuration, so it survives a reset.
   * @type {{maxRateDegPerS:number, accelDegPerS2:number, lagS:number}|null}
   */
  steeringServo = null;

  reset(X, Y, psi) {
    this.u = 0; this.v = 0; this.r = 0;
    this.X = X; this.Y = Y; this.psi = psi;
    this.wF = 0;
    // Two rear wheel speeds, because a differential is the only thing between
    // them and it is what decides the car's balance on the throttle. The
    // front axle stays one unit: there is nothing between the front wheels.
    this.wRL = 0; this.wRR = 0;
    this.aF = 0; this.aR = 0;
    this.delta = 0;
    /**
     * Steering velocity, deg/s at the road wheel -- the servo's state.
     *
     * Reset here and not just in the constructor: `respawn` goes through
     * `reset`, and leaving a stale steering velocity behind is the same shape
     * of bug as leaving a stale crank speed behind, which put a 900 rad/s
     * clutch mismatch into the first substep after every respawn.
     *
     * `steeringServo` deliberately does NOT live here. It is configuration,
     * not state -- resetting it would silently drop the driver's control
     * profile every time they respawned.
     */
    this.steerRateDegPerS = 0;
    this.ax = 0; this.ay = 0;
    this.pt.gear = 0;
    this.pt.engineRpm = this.p.idleRpm;
    this.pt.shiftTimer = 0;
    this.pt.pendingGear = null;
    // Everything the clutch and the limiter were in the middle of. The launch
    // timer especially: it is free-running and pins the clutch at full
    // capacity for 0.8 s after a dump, so a restart inside that window used to
    // begin with the clutch locked regardless of throttle or engine speed.
    // The Rust port clears the same set -- see `Powertrain::reset`.
    this.pt.limiterCut = false;
    this.pt.slipping = true;
    this.pt.clutchSlipRpm = 0;
    this.pt.launchHeld = false;
    this.pt.launchDumpS = 0;
    this.telemetry = this.blankTelemetry();
  }

  blankTelemetry() {
    return {
      speed: 0, axG: 0, ayG: 0, bodySlipDeg: 0, yawRateDegS: 0,
      FzF: 0, FzR: 0, dFzLatF: 0, dFzLatR: 0,
      slipF: 0, slipR: 0, kappaF: 0, kappaR: 0,
      utilF: 0, utilR: 0, balance: 0,
      downforceN: 0, dragN: 0, driveForceN: 0,
      rollDeg: 0, pitchDeg: 0,
      // Steering feel. kingpinTorqueNm is the moment both front tyres put on
      // the steering axis; rimTorqueNm is what reaches the driver's hands.
      // Both left-positive like `delta`: positive tries to steer further left.
      kingpinTorqueNm: 0, rimTorqueNm: 0, trailFm: 0, mechTrailM: 0,
    };
  }

  get speed() { return Math.hypot(this.u, this.v); }

  /**
   * Advance by `dt` seconds using fixed 500 Hz substeps.
   * @param {number} dt
   * @param {{steer:number, throttle:number, brake:number}} input
   *        steer -1..1 (left positive), throttle/brake 0..1
   */
  step(dt, input) {
    let remaining = Math.min(dt, 0.1); // never simulate more than 100 ms of catch-up
    while (remaining > 1e-6) {
      const h = Math.min(SUBSTEP, remaining);
      this.substep(h, input);
      remaining -= h;
    }
  }

  substep(dt, input) {
    const p = this.p;
    this.refresh();

    // ---- steering: a rate- and acceleration-limited servo -------------------
    // Rate limit and first-order lag were always here. The acceleration limit
    // is new, and it is what lets a control device have its own steering
    // character: a keyboard key is a step input, and without a bound on how
    // fast the steering *speed* can change, a step in position becomes a step
    // in velocity, which no hand and no steering motor can produce.
    //
    // `steeringServo` is set by the game from the active control profile. The
    // default derived from the vehicle parameters uses an effectively infinite
    // acceleration, so with no profile attached this reduces exactly to the
    // previous behaviour and the validated numbers do not move.
    const cfg = this.steeringServo ?? {
      maxRateDegPerS: p.steerRateDegS,
      accelDegPerS2: 1e9,
      lagS: p.steerLagS,
      slipCapDeg: 0,
      rateSpeedRefMps: 0,
      rateSpeedExp: 1.5,
    };
    let targetDeg = clamp(input.steer, -1, 1) * p.maxSteerDeg;
    // Slip-capped steering, for devices with no feel. A key or a stick
    // commands an ANGLE, and past the tyre's peak more angle is less grip
    // and more yaw -- at 25 m/s the car's peak lateral comes at under 5 deg
    // of steer, and a keyboard "lock" of 13 deg threw the front three times
    // past it and hooked the car round. The cap holds the front slip angle
    // at or below `slipCapDeg`, measured against the car's actual velocity
    // and yaw rate, so at the limit the wheel angle follows the car (a
    // built-in easing off as the rear slides) instead of fighting it. The
    // band is centred on the car's motion, not on zero, so when the rear
    // steps out the front follows the slide: the assist counter-steers for
    // a driver who has no seat to feel it in. Off for a wheel: there the
    // driver has the tyre's own signal in their hands.
    if (cfg.slipCapDeg > 0) {
      const kinDeg = (Math.atan2(this.v + this.a * this.r, Math.max(Math.abs(this.u), 0.6)) * 180) / Math.PI;
      // The band only means something once the car is moving: at walking
      // pace the velocity direction swings with any leftover sideslip, and a
      // band centred on it commanded steer with no key held, so the car
      // wandered side to side by itself. Below SLIP_CAP_FULL_MPS the band
      // opens up, 10 deg per m/s, until it is no band at all.
      const capDeg = cfg.slipCapDeg + Math.max(0, SLIP_CAP_FULL_MPS - Math.abs(this.u)) * 10;
      const lo = kinDeg - capDeg, hi = kinDeg + capDeg;
      targetDeg = clamp(targetDeg, lo, hi);
      // The band follows the car's velocity, and in a spin that is 70 to 90
      // degrees off the nose; the rack still stops at lock.
      targetDeg = clamp(targetDeg, -p.maxSteerDeg, p.maxSteerDeg);
    }
    const angleDeg = (this.delta * 180) / Math.PI;

    // Speed-sensitive rate for devices with no feel: nobody flicks a wheel
    // at 90 km/h the way they do at 30, and a key press is the same step at
    // both. Scales the rate and acceleration limits by (ref / u)^exp above
    // the reference speed. Zero reference = off (wheels).
    let rateScale = 1;
    if (cfg.rateSpeedRefMps > 0) {
      rateScale = Math.min(1, Math.pow(cfg.rateSpeedRefMps / Math.max(Math.abs(this.u), 0.1), cfg.rateSpeedExp ?? 1.5));
    }
    let wantRate = (targetDeg - angleDeg) / Math.max(cfg.lagS, 1e-4);
    wantRate = clamp(wantRate, -cfg.maxRateDegPerS * rateScale, cfg.maxRateDegPerS * rateScale);
    const maxDelta = cfg.accelDegPerS2 * rateScale * dt;
    this.steerRateDegPerS += clamp(wantRate - this.steerRateDegPerS, -maxDelta, maxDelta);

    let nextDeg = angleDeg + this.steerRateDegPerS * dt;
    // Do not coast past the target: overshoot here is a discretisation
    // artefact, not modelled inertia.
    const err = targetDeg - angleDeg;
    if ((err > 0 && nextDeg > targetDeg) || (err < 0 && nextDeg < targetDeg)) {
      nextDeg = targetDeg;
      this.steerRateDegPerS = 0;
    }
    this.delta = (nextDeg * Math.PI) / 180;
    const d = this.delta;

    const u = this.u, v = this.v, r = this.r;
    const V = Math.hypot(u, v);
    // Speed floor in the slip-angle denominator. `alpha = atan(vy/vx)` is
    // singular as the car stops, and the floor is what keeps it finite -- but
    // the floor also sets the loop gain, because dFy/dv goes as C_alpha/uSafe.
    // At 0.6 m/s that gain is high enough for the lateral equation to ring: a
    // steady 35 deg of steer at 1-2 m/s buzzed v through 70-130 sign changes
    // in two seconds, roughly 30-85 Hz, and the driver felt the car "shifting
    // side to side" in the paddock. At 3.0 the ring is gone (72 flips -> 8)
    // and NOTHING above 3 m/s changes at all, because up there the floor is
    // not what is being used. The cost is that a tyre under 3 m/s reports less
    // slip angle than it has, so the car is a little soft at a crawl; the
    // standstill scrub and jacking terms in the force feedback carry the feel
    // down there instead.
    const uSafe = Math.max(Math.abs(u), 3.0);

    // ---- aero (2026 CFD map) ----
    const q = 0.5 * p.airDensityKgM3 * V * V;
    const downforce = q * p.claM2;
    const drag = q * p.cdaM2;

    // ---- normal loads: static + longitudinal transfer + aero ----
    const L = p.wheelbaseM;
    const W = p.massKg * G;
    let FzF = (W * this.b) / L - (p.massKg * this.ax * p.cgHeightM) / L + downforce * p.aeroFrontFrac;
    let FzR = (W * this.a) / L + (p.massKg * this.ax * p.cgHeightM) / L + downforce * (1 - p.aeroFrontFrac);
    FzF = Math.max(0, FzF);
    FzR = Math.max(0, FzR);

    // ---- lateral transfer per axle: elastic (roll stiffness) + geometric
    //      (roll centre) + unsprung. Only its MAGNITUDE matters here, because
    //      it is used to derate the axle's grip, not to steer the car.
    const tF = p.trackFrontM, tR = p.trackRearM;
    const msF = this.mSprung * p.weightDistFront;
    const msR = this.mSprung * (1 - p.weightDistFront);
    const unsprungF = 2 * p.unsprungFrontKg;
    const unsprungR = 2 * p.unsprungRearKg;
    const ay = this.ay;
    const dFzF =
      (this.mSprung * ay * p.roll.hRollArmM * p.roll.rsdFront) / tF +
      (msF * ay * p.roll.rcFrontM) / tF +
      (unsprungF * ay * p.tireRadiusM) / tF;
    const dFzR =
      (this.mSprung * ay * p.roll.hRollArmM * (1 - p.roll.rsdFront)) / tR +
      (msR * ay * p.roll.rcRearM) / tR +
      (unsprungR * ay * p.tireRadiusM) / tR;

    const muYF = axleMu(p.muLat, FzF, dFzF, this.Fz0, p.tireLoadSensitivity);
    const muYR = axleMu(p.muLat, FzR, dFzR, this.Fz0, p.tireLoadSensitivity);
    const muXF = axleMu(p.muLong, FzF, dFzF, this.Fz0, p.tireLoadSensitivity);
    const muXR = axleMu(p.muLong, FzR, dFzR, this.Fz0, p.tireLoadSensitivity);

    // ---- slip angles with relaxation-length lag ----
    const aFraw = d - Math.atan2(v + this.a * r, uSafe);
    const aRraw = -Math.atan2(v - this.b * r, uSafe);
    const relaxRate = Math.min(Math.abs(u) / p.relaxLengthM, 1 / dt); // stable at rest
    this.aF += (aFraw - this.aF) * Math.min(1, relaxRate * dt);
    this.aR += (aRraw - this.aR) * Math.min(1, relaxRate * dt);

    // ---- slip ratios from the wheel-speed states ----
    const kDen = Math.max(Math.abs(u), 2.0);
    const kF = (this.wF * p.tireRadiusM - u) / kDen;

    // ---- the rear axle, one wheel at a time ----
    // Each rear wheel carries its own load, its own forward speed and so its
    // own slip ratio; they share a slip angle. Splitting them is what makes a
    // differential mean anything: with one rotor there is no torque difference
    // across the track and so no yaw moment from the driven wheels at all.
    const halfR = FzR * 0.5;
    const shiftR = Math.min(Math.abs(dFzR), halfR); // the inner tyre lifts, it does not go negative
    const outerR = halfR + shiftR;
    const innerR = halfR - shiftR;
    // Positive ay is a LEFT turn, which loads the right-hand tyres.
    const FzRL = ay >= 0 ? innerR : outerR;
    const FzRR = ay >= 0 ? outerR : innerR;
    // Forward speed at each rear patch: a wheel at lateral offset y sees
    // u - r*y, and left is positive y.
    const halfTrackR = p.trackRearM * 0.5;
    const uRL = u - r * halfTrackR;
    const uRR = u + r * halfTrackR;
    const kRL = (this.wRL * p.tireRadiusM - uRL) / kDen;
    const kRR = (this.wRR * p.tireRadiusM - uRR) / kDen;
    const muYRL = muAtLoad(p.muLat, FzRL, this.Fz0, p.tireLoadSensitivity);
    const muXRL = muAtLoad(p.muLong, FzRL, this.Fz0, p.tireLoadSensitivity);
    const muYRR = muAtLoad(p.muLat, FzRR, this.Fz0, p.tireLoadSensitivity);
    const muXRR = muAtLoad(p.muLong, FzRR, this.Fz0, p.tireLoadSensitivity);
    const fRL = tyreForces(this.aR, kRL, FzRL, muYRL, muXRL);
    const fRR = tyreForces(this.aR, kRR, FzRR, muYRR, muXRR);

    const fF = tyreForces(this.aF, kF, FzF, muYF, muXF);
    const fR = {
      fx: fRL.fx + fRR.fx,
      fy: fRL.fy + fRR.fy,
      utilisation: Math.max(fRL.utilisation, fRR.utilisation),
    };
    // The front axle's lateral peak relative to the rear (params.js,
    // frontGripFactor). Applied to the force rather than to muYF because the
    // fitted curve is exactly linear in mu at a given slip, so this IS a mu
    // scaling -- and scaling the output keeps the Rust port bit-identical
    // without threading a second mu through the tyre. Utilisation (where on
    // the curve the tyre is) is unchanged by it, by construction.
    fF.fy *= p.frontGripFactor ?? 1;

    // ---- resolve front tyre forces through the steer angle ----
    const cd = Math.cos(d), sd = Math.sin(d);
    const FxFb = fF.fx * cd - fF.fy * sd;
    const FyFb = fF.fx * sd + fF.fy * cd;
    const FxRb = fR.fx;
    const FyRb = fR.fy;

    const rollRes = p.crr * (FzF + FzR) * Math.sign(u || 1);

    // ---- rigid-body equations of motion ----
    // The yaw moment the driven wheels make across the track -- the whole
    // point of modelling the differential. A force at lateral offset y
    // contributes -y*Fx and left is positive y, so the inner wheel pushing
    // harder than the outer pushes the nose wide. Under power a Salisbury LSD
    // sends torque to the SLOWER, inner wheel, which is why a locked car
    // understeers on throttle; on a lift it drags the faster, outer wheel,
    // which is what steadies the rear instead of letting it come round. With
    // one rear rotor both of those are exactly zero.
    const nDiff = halfTrackR * (fRR.fx - fRL.fx);

    const du = (FxFb + FxRb - drag - rollRes) / p.massKg + v * r;
    const dv = (FyFb + FyRb) / p.massKg - u * r;
    const dr = (this.a * FyFb - this.b * FyRb + nDiff) / p.izzKgM2;

    // ---- driveline ----
    // The carrier turns at the mean of the two side gears, so that is the
    // speed the gearbox sees.
    const wRmean = 0.5 * (this.wRL + this.wRR);
    const drive = this.pt.step(dt, input.throttle, wRmean, V);

    // ---- wheel dynamics; brake torque cannot drive a wheel backwards ----
    const brakeTotal = clamp(input.brake, 0, 1) * p.brakeTorqueMaxNm;
    const tbF = brakeTotal * p.brakeBiasFront;
    const tbR = brakeTotal * (1 - p.brakeBiasFront);

    const IwRside = p.wheelInertiaRearKgM2;

    // ---- the differential ----
    // Salisbury clutch pack: `tCap` is the largest torque DIFFERENCE the ramps
    // and the preload can hold across the two outputs, and the transfer is
    // half of it. See `params.diff` for where C and B come from. A 1.5-way
    // locks harder under power than on the overrun. Coulomb friction with a
    // soft sign, as everywhere else here: the clutch opposes the speed
    // difference and saturates, and inside the stick band it behaves as a
    // spring rather than switching between two branches. The spring is
    // integrated with a limit (below, once the axle's inertia is known) so it
    // cannot overshoot -- without that it is an explicit spring on a very
    // small inertia and it oscillates forever.
    const dfp = p.diff;
    const tIn = drive.wheelTorqueNm;
    const lockFrac = tIn >= 0 ? dfp.powerLock : dfp.coastLock;
    const tCap = lockFrac * Math.abs(tIn) + dfp.preloadNm;
    const dWrear = this.wRR - this.wRL;
    const tbRside = 0.5 * tbR;
    // Implicit in the tyre's longitudinal stiffness. Explicit Euler on
    // dw = -R Fx(kappa(w)) / I is only stable while dt < 2 I kDen / (R^2 dFx/dkappa),
    // which at 500 Hz is everything under about 4.5 m/s: the front wheels
    // chattered every substep at walking pace, +-0.33 in slip ratio, and ABS
    // read the chatter. Dividing by the linearised reaction as well is
    // unconditionally stable and lands on the same steady state. The slope is
    // taken numerically so the Rust port can do the identical operation.
    const R = p.tireRadiusM;
    const dFxF = Math.max(0, (tyreForces(this.aF, kF + KAPPA_H, FzF, muYF, muXF).fx - fF.fx) / KAPPA_H);
    const dFxRL = Math.max(0, (tyreForces(this.aR, kRL + KAPPA_H, FzRL, muYRL, muXRL).fx - fRL.fx) / KAPPA_H);
    const dFxRR = Math.max(0, (tyreForces(this.aR, kRR + KAPPA_H, FzRR, muYRR, muXRR).fx - fRR.fx) / KAPPA_H);
    const stiffF = dt * R * R * dFxF / kDen;
    const stiffRL = dt * R * R * dFxRL / kDen;
    const stiffRR = dt * R * R * dFxRR / kDen;
    let dwF = (-fF.fx * R - Math.sign(this.wF) * tbF) / (this.IwF + stiffF);

    // The two rear wheels, solved together. The driveline's reflected inertia
    // hangs on the CARRIER, which turns at the mean of the two side gears, so
    // it resists the wheels speeding up together and does nothing at all to
    // resist one speeding up while the other slows. Hanging half of it on each
    // wheel -- the obvious shortcut -- would make the axle behave far more
    // locked than the clutch pack actually makes it, which is precisely the
    // effect being modelled here.
    //
    //   (iL + qD) dwL +       qD dwR = Tin/2 + tLock - AL
    //        qD dwL + (iR + qD) dwR = Tin/2 - tLock - AR
    //
    // with qD = I_driveline / 4 and A the tyre and brake torques.
    const qD = 0.25 * drive.addedWheelInertia;
    const iL = IwRside + stiffRL;
    const iR = IwRside + stiffRR;
    const det = Math.max(iL * iR + qD * (iL + iR), 1e-9);

    // ---- the clutch pack's torque, limited so it cannot overshoot ----
    //
    // Feeding `tLock` into the pair above, the ANTISYMMETRIC mode obeys
    //
    //   d(wRR - wRL)/dt = -tLock * (iL + iR + 4 qD) / det   =   -tLock / J
    //
    // so `J` below is the inertia the clutch actually works against. It is
    // small -- a fraction of a kg m^2 -- and a spring of gain
    // `0.5 tCap / stickRadS` on it, integrated explicitly, is unstable
    // whenever `dt * 0.5 tCap / (stickRadS J) > 2`. At the shipped preload
    // alone that is true by more than an order of magnitude, and it showed:
    // once any corner had set the two rear wheels apart, they sat in a
    // permanent period-2 oscillation on dead-straight road, flipping sign
    // every substep for the rest of the run. It never decayed, it swung
    // `sim.kappa_rl` / `sim.kappa_rr` by half their value and put 0.065 deg/s
    // of peak-to-peak garbage into `imu.yaw_rate`, and at 100 Hz it aliased
    // into the log as noise nobody could account for.
    //
    // The limit is what a stick constraint actually does: never apply more
    // torque than would bring the relative speed to zero in this step. Inside
    // the band that turns the spring into a proper stick, and it is
    // unconditionally stable at any dt. Outside it -- once the pack is slipping
    // and `tanh` has saturated -- the limit is far larger than `0.5 tCap` and
    // nothing changes, so the modelled slip behaviour is untouched.
    const antiJ = det / Math.max(iL + iR + 4 * qD, 1e-9);
    const tSpring = 0.5 * tCap * Math.tanh(dWrear / Math.max(dfp.stickRadS, 1e-4));
    const tStop = antiJ * Math.abs(dWrear) / dt;
    const tLock = Math.sign(tSpring) * Math.min(Math.abs(tSpring), tStop);
    // Torque leaves the faster wheel and arrives at the slower one. These are
    // what the diff delivers BEFORE the driveline's own inertia is taken out
    // of them, which the coupled solve does.
    const tRL = 0.5 * tIn + tLock;
    const tRR = 0.5 * tIn - tLock;
    const pL = tRL - fRL.fx * R - Math.sign(this.wRL) * tbRside;
    const pR = tRR - fRR.fx * R - Math.sign(this.wRR) * tbRside;
    let dwRL = (pL * (iR + qD) - qD * pR) / det;
    let dwRR = (pR * (iL + qD) - qD * pL) / det;

    // Clamp so braking stops a wheel instead of reversing it inside one step.
    if (this.wF > 0 && this.wF + dwF * dt < 0 && tbF > 0) dwF = -this.wF / dt;
    if (this.wF < 0 && this.wF + dwF * dt > 0 && tbF > 0) dwF = -this.wF / dt;
    if (this.wRL > 0 && this.wRL + dwRL * dt < 0 && tbRside > 0 && tRL <= 0) {
      dwRL = -this.wRL / dt;
    }
    if (this.wRR > 0 && this.wRR + dwRR * dt < 0 && tbRside > 0 && tRR <= 0) {
      dwRR = -this.wRR / dt;
    }

    // ---- integrate ----
    this.u += du * dt;
    this.v += dv * dt;
    this.r += dr * dt;
    // The fronts may roll backwards (a spin carries the car backwards for a
    // moment); the rear stays non-negative because the driveline behind it
    // has no reverse and the clutch logic assumes it.
    this.wF = this.wF + dwF * dt;
    this.wRL = Math.max(0, this.wRL + dwRL * dt);
    this.wRR = Math.max(0, this.wRR + dwRR * dt);

    // The measured accelerations that feed next step's load transfer.
    this.ax = du - v * r;
    this.ay = dv + u * r;

    // ---- come to a genuine stop rather than creeping on numerical noise ----
    if (Math.abs(this.u) < 0.25 && input.throttle < 0.05 && this.speed < 0.4) {
      this.u = 0; this.v = 0; this.r = 0; this.wF = 0; this.wRL = 0; this.wRR = 0;
      this.ax = 0; this.ay = 0;
      // The lagged slip angles too. The relaxation rate is proportional to
      // speed, so at rest they never relax -- and a car stopped mid-corner
      // was left holding its full cornering slip, which put 8 N.m of
      // aligning torque on a stationary, centred steering wheel forever.
      this.aF = 0; this.aR = 0;
    }

    // No reverse gear, but a spinning car does travel backwards for a moment
    // and u must be allowed to say so. Pinning it at zero while v was left
    // alone forced the velocity perpendicular to the body every substep, and
    // the car orbited in a "tornado" instead of scrubbing off. The tyre
    // model is symmetric in |u| (slip angles use |u|), so it is valid here.

    this.X += (this.u * Math.cos(this.psi) - this.v * Math.sin(this.psi)) * dt;
    this.Y += (this.u * Math.sin(this.psi) + this.v * Math.cos(this.psi)) * dt;
    this.psi += this.r * dt;

    // ---- telemetry ----
    const t = this.telemetry;
    t.speed = this.speed;
    t.axG = this.ax / G;
    t.ayG = this.ay / G;
    t.bodySlipDeg = (Math.atan2(this.v, Math.max(Math.abs(this.u), 0.1)) * 180) / Math.PI;
    t.yawRateDegS = (this.r * 180) / Math.PI;
    t.FzF = FzF; t.FzR = FzR;
    t.dFzLatF = dFzF; t.dFzLatR = dFzR;
    t.slipF = (this.aF * 180) / Math.PI;
    t.slipR = (this.aR * 180) / Math.PI;
    t.kappaF = kF; t.kappaR = kRL; t.kappaRL = kRL; t.kappaRR = kRR;
    t.utilF = fF.utilisation; t.utilR = fR.utilisation;
    t.utilRL = fRL.utilisation; t.utilRR = fRR.utilisation;
    // Load-weighted across the rear, NOT the worse of the two wheels. With a
    // differential the lightly loaded inner wheel is allowed to spin in a tight
    // corner -- that is the diff doing its job -- and reading that one wheel as
    // "the rear axle is out of grip" is simply wrong: it is carrying almost no
    // load and almost none of the axle's lateral force.
    const utilRaxle = FzR > 1 ? (fRL.utilisation * FzRL + fRR.utilisation * FzRR) / FzR : 0;
    t.balance = utilRaxle - fF.utilisation; // >0 rear-limited (oversteer)
    t.downforceN = downforce;
    t.dragN = drag;
    t.driveForceN = FxRb;
    t.steerDeg = (d * 180) / Math.PI;
    t.rollDeg = t.ayG * p.rollGradientDegG;   // + = leaning right (left turn)
    t.pitchDeg = t.axG * p.pitchGradientDegG; // + = nose up (braking dives)
    t.locked = drive.locked;

    // ---- steering torque, for force feedback ----
    // The bicycle model has one front slip angle, but the two tyres carry
    // different loads once the car rolls, and the trail grows with load, so
    // the aligning torque is summed per tyre with the axle's force split by
    // load. The self-aligning moment opposes the slip angle, which is what
    // makes a wheel try to return to centre.
    const geo = p.steering;
    const mechTrail = p.tireRadiusM * Math.tan((geo.casterDeg * Math.PI) / 180) +
      geo.kingpinOffsetTrailM;
    let kingpin = 0;
    if (FzF > 1) {
      const half = FzF / 2;
      const shift = Math.min(Math.abs(dFzF), half);
      const outer = half + shift, inner = half - shift;
      const sF = fF.utilisation;
      const tO = pneumaticTrail(sF, outer), tI = pneumaticTrail(sF, inner);
      // Per-tyre share of the axle lateral force. The axle force was made
      // with the load-weighted mean mu, so each tyre's share is its own
      // mu * load -- which is exactly what evaluating the two tyres
      // separately (as the Rust port does) gives.
      const muO = muAtLoad(p.muLat, outer, this.Fz0, p.tireLoadSensitivity);
      const muI = muAtLoad(p.muLat, inner, this.Fz0, p.tireLoadSensitivity);
      const wO = muO * outer, wI = muI * inner;
      const fyO = fF.fy * (wO / (wO + wI)), fyI = fF.fy * (wI / (wO + wI));
      kingpin = -(fyO * (tO + mechTrail) + fyI * (tI + mechTrail));
      t.trailFm = (tO * outer + tI * inner) / FzF;
    } else {
      t.trailFm = 0;
    }
    t.mechTrailM = mechTrail;
    t.kingpinTorqueNm = kingpin;
    const torqueRatio = geo.torqueRatio ?? 1 / Math.max(p.steeringRatio, 1e-6);
    t.rimTorqueNm = kingpin * torqueRatio * geo.rackEfficiency;
  }

  /**
   * Put the car back on the road at a pose. `reset` leaves the engine at idle
   * in first, which is right for a standing start; if we are placing the car
   * at speed the crank has to come with it, or the clutch grabs against a
   * rolling wheel and brakes the rear axle to a stop on the first substep.
   */
  /**
   * Mean rear wheel speed (rad/s): what the differential carrier turns at, and
   * so what the gearbox, the rev counter and the rear wheel animation see.
   * Kept as a property because everything outside this class asked for `wR`
   * long before there were two of them.
   */
  get wR() {
    return 0.5 * (this.wRL + this.wRR);
  }

  respawn(X, Y, psi, speed = 0) {
    // `reset` puts the box in first for a standing start. Placed at speed the
    // gear the caller chose has to survive it, or every rolling respawn is in
    // first -- at 20 m/s that is 16 600 rpm, on the limiter, with engine
    // braking through a 17:1 reduction, which is what the validation harness
    // was unknowingly measuring above 15 m/s.
    const gear = this.pt.gear;
    this.reset(X, Y, psi);
    this.u = speed;
    this.wF = this.wRL = this.wRR = speed / this.p.tireRadiusM;
    if (speed > 0) {
      this.pt.gear = gear;
      this.pt.syncToWheel(this.wRL);
    }
  }
}

function clamp(x, lo, hi) { return x < lo ? lo : x > hi ? hi : x; }
