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
    this.limiterCut = false;
    this.slipping = true;
    this.clutchSlipRpm = 0;
    this.stalled = false;
  }

  /** Linear-interpolated wide-open-throttle brake torque (N.m). */
  wotTorque(rpm) {
    const p = this.points;
    if (rpm <= p[0].rpm) {
      // Below the sweep's first point, fall away toward a plausible idle
      // torque rather than holding 61 N.m down to zero rpm.
      const f = Math.max(0, rpm - this.v.idleRpm) / Math.max(1, p[0].rpm - this.v.idleRpm);
      return p[0].torqueNm * (0.35 + 0.65 * Math.min(1, f));
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

  /** Net crankshaft torque for a throttle demand 0..1. */
  engineTorque(rpm, throttle) {
    if (this.shiftTimer > 0) return -this.motoringTorque(rpm) * 0.5; // ignition cut
    if (rpm >= this.v.revLimitRpm) this.limiterCut = true;
    if (this.limiterCut && rpm < this.v.revLimitRpm - 350) this.limiterCut = false;
    if (this.limiterCut) return -this.motoringTorque(rpm);

    const wot = this.wotTorque(rpm);
    const drag = this.motoringTorque(rpm);
    let t = throttle * (wot + drag) - drag;

    // Idle air control: keep the engine alive when the driver is off it.
    if (rpm < this.v.idleRpm) t += (this.v.idleRpm - rpm) * 0.02;
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
  clutchCapacity(throttle, speed) {
    if (this.shiftTimer > 0) return 0;
    const full = 220; // EST: well above peak torque, so it locks when rolling
    if (speed > 4) return full;
    const launch = 0.12 + 0.88 * Math.min(1, throttle * 1.15);
    return full * Math.max(0.1, launch);
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
    if (this.shiftTimer > 0) {
      this.shiftTimer -= dt;
      if (this.shiftTimer <= 0 && this.pendingGear != null) {
        this.gear = this.pendingGear;
        this.pendingGear = null;
        this.shiftTimer = 0;
      }
    }

    const v = this.v;
    const n = this.ratio();
    const eff = v.drivetrainEff;
    const cap = this.clutchCapacity(throttle, speed);

    let omegaE = this.engineRpm * RPM_TO_RADS;
    const omegaClutchSide = rearWheelOmega * n;
    const slip = omegaE - omegaClutchSide;

    const Te = this.engineTorque(this.engineRpm, throttle);

    // Try locked first: does the clutch have enough capacity to hold the
    // engine and driveline together at the acceleration that implies?
    const lockable = cap > 0 && Math.abs(slip) < 8 && n > 0;
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
      this.engineRpm = Math.max(v.idleRpm * 0.6, omegaClutchSide * RADS_TO_RPM);
      this.stalled = this.engineRpm < 900;
      // Reflected driveline inertia, each component at its own speed ratio:
      // the crank turns `n` times the wheel, everything downstream of the
      // primary only `n / primaryReduction`.
      const nGbox = n / v.primaryReduction;
      return {
        wheelTorqueNm: Te * n * eff,
        addedWheelInertia:
          v.engineInertiaKgM2 * n * n + v.gearboxInertiaKgM2 * nGbox * nGbox,
        locked: true,
      };
    }

    // Slipping: the clutch passes at most `cap`, in the direction that closes
    // the slip. The engine accelerates on its own inertia with the remainder.
    this.slipping = true;
    const dir = slip > 0 ? 1 : slip < 0 ? -1 : 0;
    const passed = dir === 0 ? Math.max(-cap, Math.min(cap, Te)) : dir * cap;

    const domegaE = (Te - passed) / v.engineInertiaKgM2;
    omegaE += domegaE * dt;
    this.engineRpm = Math.max(0, omegaE * RADS_TO_RPM);
    this.clutchSlipRpm = Math.abs(slip) * RADS_TO_RPM;
    this.stalled = this.engineRpm < 700 && speed < 1;
    if (this.stalled) this.engineRpm = v.idleRpm * 0.85; // auto-restart, this is a game

    return {
      wheelTorqueNm: passed * n * eff,
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
