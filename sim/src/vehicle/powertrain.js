// Powertrain: the Helios CFD engine sweep, a real clutch, and the CBR600RR box.
//
// Torque comes straight from `data/sdm26-torque.json`, which is the SDM26
// characteristic-junction RPM sweep out of the Helios CFD module's 1-D FV
// engine solver. That curve is not a smooth dyno arc -- it has the wave-action
// bumps and holes the solver predicts (a hole at 6500, a spike at 8000, the
// second wind at 11000-11500 before the restrictor chokes it). Those features
// survive into the driving model, so gearing choices matter the way they do in
// the car.
//
// The clutch is modelled properly (locked / slipping with a torque capacity)
// rather than by pinning engine rpm to road speed. That is what makes launches,
// bogs, stalls and shift cuts behave.

import { totalReduction } from "./params.js";

const RPM_TO_RADS = (2 * Math.PI) / 60;
const RADS_TO_RPM = 60 / (2 * Math.PI);

/**
 * Crank torque to wheel torque through the ratio and the driveline
 * efficiency. Losses always oppose motion: drive is reduced by them, engine
 * braking is increased by them. Applying `eff` symmetrically made the car
 * coast 15% freer than it should.
 */
function toWheel(crankNm, n, eff) {
  return crankNm > 0 ? crankNm * n * eff : (crankNm * n) / eff;
}

function clamp(x, lo, hi) { return x < lo ? lo : x > hi ? hi : x; }

/** Width of the soft rev limiter, rpm below the limit. */
const LIMITER_BAND_RPM = 300;
/**
 * How long the clutch stays dumped after launch control is released, s.
 * Long enough to cover the engagement; after it the clutch is governed the
 * ordinary way again.
 */
const LAUNCH_DUMP_S = 0.8;

export class Powertrain {
  /**
   * @param {object} v      vehicle params (SDM26)
   * @param {object} curve  parsed data/sdm26-torque.json
   */
  constructor(v, curve) {
    this.v = v;
    this.points = curve.points;
    this.displacement = curve.displacementM3 ?? 599e-6;
    this.sourceName = curve.name;

    this.peakTorque = this.points.reduce((a, b) => (b.torqueNm > a.torqueNm ? b : a));
    this.peakPower = this.points.reduce((a, b) => (b.powerKW > a.powerKW ? b : a));

    this.gear = 0;              // 0-based index into gearRatios
    this.engineRpm = v.idleRpm;
    this.shiftTimer = 0;        // >0 while the driveline is open for a shift
    this.pendingGear = null;
    /** >0 while the torque is being fed back in after the cut; see `engineTorque`. */
    this.reintroTimer = 0;
    this.limiterCut = false;
    this.slipping = true;
    this.clutchSlipRpm = 0;
    /** Launch control: driver holding it, and how long since they dropped it. */
    this.launchHeld = false;
    this.launchDumpS = 0;
    this.stalled = false;
  }

  /** Linear-interpolated wide-open-throttle brake torque (N.m). */
  wotTorque(rpm) {
    const p = this.points;
    if (rpm <= p[0].rpm) {
      // Below the sweep's first point, fall away toward a plausible idle
      // torque rather than holding 61 N.m down to zero rpm.
      //
      // The 0.56 floor is not a guess. It is pinned by the measured idle
      // point: the engine idles at 2000 rpm with the plate at 22%
      // (`idleThrottleFrac`), so at 2000 rpm a 22% opening must exactly
      // balance friction. Solving
      //     drag / (wot + drag) = 0.22   with drag = 5.543 N.m
      // gives wot(2000) = 19.65 N.m. This floor is a fraction of the SWEEP'S
      // FIRST POINT -- 35.461 N.m at 4000 rpm, not the peak -- so
      // 19.65 / 35.461 = 0.554, which is the 0.56 here.
      //
      // That is 34% of the 57.82 N.m peak, lower than the 55-70% a naturally
      // aspirated four is usually quoted at 2000 rpm; this engine peaks near
      // 11k, so a low fraction down there is expected. Kept identical to the
      // Rust port in `powertrain.rs`.
      const f = Math.max(0, rpm - this.v.idleRpm) / Math.max(1, p[0].rpm - this.v.idleRpm);
      return p[0].torqueNm * (0.56 + 0.44 * Math.min(1, f));
    }
    const last = p[p.length - 1];
    if (rpm >= last.rpm) return last.torqueNm;
    for (let i = 1; i < p.length; i++) {
      if (rpm <= p[i].rpm) {
        const a = p[i - 1], b = p[i];
        const t = (rpm - a.rpm) / (b.rpm - a.rpm);
        return a.torqueNm + t * (b.torqueNm - a.torqueNm);
      }
    }
    return last.torqueNm;
  }

  /** Interpolated friction mean effective pressure (bar) at `rpm`. */
  fmepBar(rpm) {
    const p = this.points;
    if (rpm <= p[0].rpm) return p[0].fmepBar;
    const last = p[p.length - 1];
    if (rpm >= last.rpm) return last.fmepBar;
    for (let i = 1; i < p.length; i++) {
      if (rpm <= p[i].rpm) {
        const a = p[i - 1], b = p[i];
        const t = (rpm - a.rpm) / (b.rpm - a.rpm);
        return a.fmepBar + t * (b.fmepBar - a.fmepBar);
      }
    }
    return last.fmepBar;
  }

  /**
   * Motoring (engine-braking) torque, N.m, positive magnitude. Taken from the
   * sweep's own fmep: T = fmep * Vd / (4*pi) for a four-stroke. At 10k rpm the
   * sweep's 2.52 bar gives ~12 N.m of overrun drag, which is why lifting in a
   * low gear slows this car noticeably.
   */
  motoringTorque(rpm) {
    return (this.fmepBar(rpm) * 1e5 * this.displacement) / (4 * Math.PI);
  }

  /**
   * Throttle plate position, 0..1, for a driver demand.
   *
   * The plate does not fully close at idle: the ETC holds it open a little to
   * keep the engine alive, and on SDM26 that idle position is 22%
   * (`idleThrottleFrac`; see the note in `tools/validate.js` about how
   * that squares with the measured sheet). The floor
   * fades out as revs rise, because a real ETC *does* close on the overrun --
   * that is what engine braking is, and holding 14% all the way up the range
   * would delete most of it.
   */
  platePosition(rpm, demand) {
    const v = this.v;
    const nominal = v.idleThrottleFrac ?? 0;
    if (nominal <= 0) return demand;

    // Proportional idle-speed control, which is what an ETC idle circuit
    // actually is. A fixed opening is not enough: below idle speed the
    // wide-open torque curve is flat and so is friction, so a fixed plate makes
    // net torque very nearly zero at EVERY sub-idle rpm. That is a neutral
    // equilibrium, not a stable one -- the engine settled wherever it happened
    // to be, which in the running game was 982 rpm rather than 2000.
    //
    // The error term gives the restoring force. At the target the commanded
    // opening is exactly the measured 14%.
    const err = (v.idleRpm - rpm) / v.idleRpm;
    const commanded = Math.max(0, nominal * (1 + 3 * err));

    // Above idle the control backs out entirely, because a real ETC closes on
    // the overrun -- that is what engine braking is, and holding any opening
    // across the range would delete most of it.
    const fadeTop = v.idleRpm * 1.6;
    const scale = rpm <= v.idleRpm
      ? 1
      : Math.max(0, (fadeTop - rpm) / (fadeTop - v.idleRpm));

    return Math.max(demand, Math.min(commanded, nominal * 3) * scale);
  }

  /**
   * Indicated crankshaft torque, N.m -- the work combustion actually does,
   * before friction is subtracted.
   *
   * This is what the engine sound model wants: it solves its heat release to
   * reproduce this much work per cycle. Net torque is the wrong input there,
   * because an engine idling at zero net torque is still burning fuel and
   * still making noise.
   */
  indicatedTorque(rpm, demand) {
    // On the limiter the ignition is cut: no combustion, no note. Without
    // this the audio heard full-throttle combustion at 14,500 rpm.
    if (this.limiterCut) return 0;
    const plate = this.platePosition(rpm, demand);
    return plate * (this.wotTorque(rpm) + this.motoringTorque(rpm));
  }

  /** Net crankshaft torque for a throttle demand 0..1. */
  /**
   * Launch control. Held, the engine sits on the LC limiter with the clutch
   * out; released, the clutch is DUMPED rather than fed in, which is what the
   * driver does and what makes a competitive start.
   */
  setLaunch(held) {
    if (this.launchHeld && !held) this.launchDumpS = LAUNCH_DUMP_S;
    this.launchHeld = !!held;
  }

  /** The rev limit in force. Launch control lowers it while it is held. */
  limitRpm() {
    return this.launchHeld
      ? Math.min(this.v.launchRpm ?? 7000, this.v.revLimitRpm)
      : this.v.revLimitRpm;
  }

  /**
   * How much of the engine's torque is back after a shift, 0..1.
   *
   * A smoothstep over `shiftReintroS` from the moment the gear engages. The
   * cut is a hard zero (that is what an ignition cut is); the return is not,
   * because a quickshifter feeds the spark back over a few tens of
   * milliseconds, and a step from the cut straight to full torque was a kick
   * through the driveline -- and through the pitch camera -- on every shift.
   */
  reintroFraction() {
    const total = this.v.shiftReintroS ?? 0;
    if (total <= 0 || this.reintroTimer <= 0) return 1;
    const x = clamp(1 - this.reintroTimer / total, 0, 1);
    return x * x * (3 - 2 * x);
  }

  engineTorque(rpm, throttle) {
    if (this.shiftTimer > 0) return -this.motoringTorque(rpm) * 0.5; // ignition cut

    const wot = this.wotTorque(rpm);
    const drag = this.motoringTorque(rpm);
    // The idle plate floor replaces what used to be an ad-hoc torque added
    // below idle speed. Modelling it as a plate position rather than a torque
    // is both closer to what the ETC does and self-correcting: the engine
    // settles wherever that opening balances friction.
    const plate = this.platePosition(rpm, throttle);
    let t = plate * (wot + drag) - drag;

    // Soft limiter. A hard cut with 350 rpm of hysteresis bounced the car
    // between full torque and full motoring drag at ~8 Hz whenever a gear
    // was held on the limiter, and that pulse went straight into the pitch
    // camera. Over the last LIMITER_BAND_RPM the net torque blends toward
    // pure drag, so the engine settles where torque meets load instead of
    // cycling. At the limit itself it is the same full cut as before.
    const soft = clamp((rpm - (this.limitRpm() - LIMITER_BAND_RPM)) / LIMITER_BAND_RPM, 0, 1);
    if (soft > 0) t -= soft * (t + drag);
    // For the audio: the ignition is being cut once we are deep in the band.
    this.limiterCut = soft >= 0.5;
    // Coming back from a shift: blend from the cut's value to the full one.
    const f = this.reintroFraction();
    if (f < 1) {
      const cut = -this.motoringTorque(rpm) * 0.5;
      t = cut + (t - cut) * f;
    }
    return t;
  }

  ratio() { return totalReduction(this.v, this.gear); }

  /**
   * Is the driveline in a state where a shift makes sense? Not mid-shift, and
   * not still slipping the clutch back in from the last one -- otherwise the
   * engine rpm being read is the free-spinning value from the open driveline,
   * not the rpm the car is actually at, and an auto-shift will fire again
   * immediately and row through the whole box in half a second.
   */
  canShift() {
    return this.shiftTimer <= 0 && !this.slipping;
  }

  requestUpshift() {
    if (this.shiftTimer > 0) return false;
    if (this.gear >= this.v.gearRatios.length - 1) return false;
    this.pendingGear = this.gear + 1;
    this.shiftTimer = this.v.shiftTimeS;
    return true;
  }

  requestDownshift() {
    if (this.shiftTimer > 0) return false;
    if (this.gear <= 0) return false;
    this.pendingGear = this.gear - 1;
    this.shiftTimer = this.v.shiftTimeS;
    return true;
  }

  /** Would this downshift over-rev the engine? Used to block money-shifts. */
  downshiftSafe(wheelOmega) {
    if (this.gear <= 0) return false;
    const n = totalReduction(this.v, this.gear - 1);
    return wheelOmega * n * RADS_TO_RPM < this.v.revLimitRpm;
  }

  /**
   * Clutch torque capacity (N.m at the crank). Open during a shift; ramped
   * with throttle at low speed so the driver can slip it off the line; solid
   * once the car is rolling.
   */
  clutchCapacity(throttle, speed, clutchSideRpm) {
    if (this.shiftTimer > 0) return 0;
    const full = 220; // EST: well above peak torque, so it locks when rolling
    // Off the throttle with the driveline turning slower than the engine can
    // idle, the clutch comes in: that is what a driver does instead of
    // letting the engine stall, and what a slipper clutch does for them.
    // Without it a tall-gear roll-down dragged the engine to zero rpm.
    if (throttle < 0.05 && clutchSideRpm < this.v.idleRpm * 0.95) return 0;
    // The driveline turning the engine rather than the other way round: the
    // clutch is in, and this is engine braking, not a launch.
    if (clutchSideRpm >= this.engineRpm) return full;
    // Caught up: there is nothing left to slip.
    // 7000 to match `Powertrain::default()` in the Rust port: a fallback
    // that differs between the two builds is a divergence waiting for the
    // day something forgets to set the real value.
    const target = this.v.launchRpm ?? 7000;
    // Launch control held: the driver has the clutch in and the engine on the
    // LC limiter, waiting. Nothing goes through until the pedal comes up.
    if (this.launchHeld) return 0;
    // ...and once it does, it is DUMPED, not fed in. That is what the driver
    // actually does and it is worth several tenths: the clutch slams to full
    // capacity, the crank drops off the LC rpm into the tyres, and the rears
    // light up. The progressive engagement below is the soft start you get
    // when nobody is using launch control.
    if (this.launchDumpS > 0) return full;
    if (clutchSideRpm >= target) return full;
    // Rolling: the clutch is in. A launch in first has the driveline catching
    // the crank at about 11 m/s, so past 12 there is nothing left to slip and
    // the capacity must stop depending on engine speed -- both because a
    // slipping clutch in normal driving is wrong, and because that dependence
    // feeds back into the engine speed that computes it, which is how the two
    // builds drifted apart on the parity drive.
    if (speed > 12) return full;

    // Pulling away. The clutch is a torque the driver holds, not a switch.
    //
    // It used to pass the whole 220 N.m at full throttle, which is a dump:
    // against an engine making 20 N.m at idle it dragged the crank to about
    // 1000 rpm and took 1.4 s to climb back, so the car left the line on a
    // third of the traction it had.
    //
    // The fix has to be a FRACTION OF WHAT THE ENGINE IS MAKING RIGHT NOW, not
    // of what it would make at the launch rpm. Anchoring it to the launch rpm
    // and leaning on a proportional term gave zero capacity below 8500 and
    // 166 N.m at 10 000: the car got no drive at all until the crank came up,
    // then the clutch grabbed. At a crawl that snapped the car sideways.
    //
    // Below the target, pass about half of what the engine makes -- enough to
    // move the car, and the rest revs the crank toward the launch rpm. At the
    // target, pass all of it, so the engine sits steady and everything it
    // makes goes to the road. Above it, pass more than it makes and the crank
    // is pulled back down. That is what a left foot does.
    const avail = this.wotTorque(this.engineRpm) * Math.min(1, throttle * 1.15);
    const frac = 0.5 + 0.5 * (this.engineRpm / target);
    const slipping = Math.max(0, Math.min(full, avail * frac));
    // ...blended into the full capacity as the slip closes, because by then it
    // is not being slipped any more. The blend has to be CONTINUOUS: a plain
    // threshold here is a cliff with 200 N.m on the other side of it, and the
    // parity drive sits right on it, so one ulp of difference between JS Math
    // and Rust libm flipped the branch and the two builds walked apart.
    // ...blended into the full capacity as the DRIVELINE spins up toward the
    // launch rpm, which is what "the car has pulled away" actually means.
    // It must NOT key off the slip: in a low gear the engine sits a few
    // hundred rpm above the clutch at any normal throttle, and a slip-based
    // blend had the clutch giving way in ordinary driving -- which is also
    // what broke the JS/Rust parity, because the capacity then fed back into
    // the engine speed that computed it.
    const w = Math.max(0, Math.min(1, clutchSideRpm / target));
    return slipping + (full - slipping) * w;
  }

  /**
   * Advance the crank/driveline one step.
   *
   * The rear wheel is integrated by the caller (it needs the tyre force), so
   * this returns the torque the driveline delivers TO the rear wheel and the
   * effective inertia to add there when the clutch is locked.
   *
   * @returns {{wheelTorqueNm:number, addedWheelInertia:number, locked:boolean}}
   */
  step(dt, throttle, rearWheelOmega, speed) {
    if (this.launchDumpS > 0) this.launchDumpS = Math.max(0, this.launchDumpS - dt);
    if (this.shiftTimer > 0) {
      this.shiftTimer -= dt;
      if (this.shiftTimer <= 0 && this.pendingGear != null) {
        const downshift = this.pendingGear < this.gear;
        this.gear = this.pendingGear;
        this.pendingGear = null;
        this.shiftTimer = 0;
        this.reintroTimer = this.v.shiftReintroS ?? 0;
        // Auto-blip on a downshift (see powertrain.rs): rev-match before the
        // clutch comes back in, so it never lands by dragging the rear axle
        // down to crank speed.
        if (downshift) {
          const matched = rearWheelOmega * this.ratio() * RADS_TO_RPM;
          this.engineRpm = clamp(matched, this.v.idleRpm, this.v.revLimitRpm - LIMITER_BAND_RPM);
        }
      }
    }

    if (this.reintroTimer > 0) this.reintroTimer = Math.max(0, this.reintroTimer - dt);

    const v = this.v;
    const n = this.ratio();
    const eff = v.drivetrainEff;
    let omegaE = this.engineRpm * RPM_TO_RADS;
    const omegaClutchSide = rearWheelOmega * n;
    const slip = omegaE - omegaClutchSide;
    const cap = this.clutchCapacity(throttle, speed, omegaClutchSide * RADS_TO_RPM);

    const Te = this.engineTorque(this.engineRpm, throttle);

    // Try locked first: does the clutch have enough capacity to hold the
    // engine and driveline together at the acceleration that implies?
    // A clutch cannot be locked below idle speed. That is not a detail -- it is
    // the reason you slip a clutch pulling away, and without it the model
    // locked up at a standstill and pinned the engine to a stall-guard floor
    // rather than letting it idle. The car then sat at 1323 rpm instead of the
    // 2000 it actually idles at.
    const clutchSideRpm = omegaClutchSide * RADS_TO_RPM;
    const lockable =
      cap > 0 && Math.abs(slip) < 8 && n > 0 && clutchSideRpm >= v.idleRpm * 0.95;
    let locked = false;

    if (lockable) {
      // Engine torque required to drag its own inertia at the wheel's rate is
      // resolved by the caller; here we just check the clutch can carry Te.
      locked = Math.abs(Te) <= cap;
    }

    if (locked) {
      this.slipping = false;
      this.clutchSlipRpm = 0;
      // Engine is tied to the wheel: rpm follows the wheel, and the wheel
      // inherits the engine's reflected inertia.
      // Floor at idle, not below it: a running engine cannot be dragged under
      // its governed idle speed -- the clutch gives up first.
      this.engineRpm = Math.max(v.idleRpm, omegaClutchSide * RADS_TO_RPM);
      this.stalled = this.engineRpm < 900;
      // Reflected driveline inertia, each component at its own speed ratio:
      // the crank turns `n` times the wheel, everything downstream of the
      // primary only `n / primaryReduction`.
      const nGbox = n / v.primaryReduction;
      return {
        wheelTorqueNm: toWheel(Te, n, eff),
        addedWheelInertia:
          v.engineInertiaKgM2 * n * n + v.gearboxInertiaKgM2 * nGbox * nGbox,
        locked: true,
      };
    }

    // Slipping: the clutch passes at most `cap`, in the direction that closes
    // the slip. The engine accelerates on its own inertia with the remainder.
    this.slipping = true;
    const dir = slip > 0 ? 1 : slip < 0 ? -1 : 0;
    // Stiction first: the torque that lands the engine exactly on the
    // clutch-side speed this step. Passing the full capacity whenever the
    // slip was non-zero yanked the engine hundreds of rpm in one substep on
    // every upshift and made a tall-gear coast-down flip the drive force
    // sign every step. Only when the slip needs more than the clutch has
    // does it slip at capacity.
    // Both sides move: the crank at Ie and the wheel side at IwSide (the
    // rear wheels plus the post-primary gearbox, at the wheel), coupled
    // through n. Ignoring the tyre reaction for this one step is fine; the
    // next step corrects it. Treating the wheel side as fixed is not: in
    // first, n^2 Ie is ten times the wheel inertia and the landing torque
    // would stop the wheel dead every step.
    const nGbox = n / v.primaryReduction;
    const IwSide = 2 * v.wheelInertiaRearKgM2 + v.gearboxInertiaKgM2 * nGbox * nGbox;
    const stick = (slip / dt + Te / v.engineInertiaKgM2) / (1 / v.engineInertiaKgM2 + (n * n) / IwSide);
    const passed = Math.abs(stick) <= cap
      ? stick
      : dir === 0 ? Math.max(-cap, Math.min(cap, Te)) : dir * cap;

    const domegaE = (Te - passed) / v.engineInertiaKgM2;
    omegaE += domegaE * dt;
    this.engineRpm = Math.max(0, omegaE * RADS_TO_RPM);
    this.clutchSlipRpm = Math.abs(slip) * RADS_TO_RPM;
    this.stalled = this.engineRpm < 700 && speed < 1;
    if (this.stalled) this.engineRpm = v.idleRpm * 0.85; // auto-restart, this is a game

    return {
      wheelTorqueNm: toWheel(passed, n, eff),
      addedWheelInertia: 0,
      locked: false,
    };
  }

  /**
   * Put the crank where the current gear says it should be for this wheel
   * speed. Needed whenever the car is placed at speed: leaving the engine at
   * idle behind a rolling wheel is a ~900 rad/s clutch mismatch, and the
   * clutch then dumps its full capacity into the driveline as a brake, which
   * locks the rear axle instantly.
   */
  syncToWheel(wheelOmega) {
    const n = this.ratio();
    this.engineRpm = Math.max(this.v.idleRpm, wheelOmega * n * RADS_TO_RPM);
    this.slipping = false;
  }

  /** Engine rpm this gear would show at road speed `v` (m/s). Diagnostics. */
  rpmAtSpeed(speedMps, gear = this.gear) {
    const n = totalReduction(this.v, gear);
    return (speedMps / this.v.tireRadiusM) * n * RADS_TO_RPM;
  }

  /**
   * Optimal upshift rpm: the lowest rpm above which the next gear ALREADY makes
   * more wheel force than this one. Scanned downward from the limiter rather
   * than upward, because the CFD curve is lumpy -- an upward scan would latch
   * onto the 6500 rpm torque hole and shift far too early.
   */
  optimalUpshiftRpm() {
    const v = this.v;
    if (this.gear >= v.gearRatios.length - 1) return v.revLimitRpm;
    const n0 = totalReduction(v, this.gear);
    const n1 = totalReduction(v, this.gear + 1);
    let best = v.revLimitRpm;
    for (let rpm = v.revLimitRpm; rpm >= 5000; rpm -= 25) {
      const after = rpm * (n1 / n0);
      if (this.wotTorque(after) * n1 >= this.wotTorque(rpm) * n0) best = rpm;
      else break;
    }
    return best;
  }
}

/** Fetch and parse the CFD sweep. */
export async function loadTorqueCurve(url = "./data/sdm26-torque.json") {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`torque curve ${url}: ${res.status}`);
  return res.json();
}
