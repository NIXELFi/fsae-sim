// The car, when the physics runs natively.
//
// In the desktop build the vehicle model lives in the Rust rig thread
// (src-tauri/src/rig.rs) at 1 kHz, next to the steering wheel and the force
// feedback. This class stands where `BicycleModel` stood so the rest of the
// game -- HUD, renderer, audio, timing, cones -- does not know the difference:
// it has the same fields and the same `step`, `respawn` and `telemetry`.
//
// The exchange is one IPC round trip per rendered frame: `step` sends the
// frame's inputs and the previous frame's snapshot is what the game reads
// back. That is one frame of display latency on top of a 1 kHz loop, and
// nothing the driver's hands can feel -- the wheel never goes through here.
//
// The JS `Powertrain` instance is kept for its pure lookups (torque curve,
// peaks, indicated torque for the audio) and has its STATE overwritten from
// the snapshot every frame, so `powertrain.engineRpm` keeps meaning what it
// always meant.

import { rigNative } from "../game/desktop.js";
import { lengthToFrontAxle, lengthToRearAxle, nominalTyreLoad } from "./params.js";

export class NativeCar {
  /**
   * @param params      the live SDM26 params object (edited in place by the UI)
   * @param powertrain  a JS Powertrain, for its lookups; its state mirrors the rig
   */
  constructor(params, powertrain) {
    this.p = params;
    this.native = true;
    this.jsPt = powertrain;
    this.pt = new NativePowertrainProxy(powertrain, this);

    this.X = 0; this.Y = 0; this.psi = 0;
    this.u = 0; this.v = 0; this.r = 0;
    this.wF = 0; this.wR = 0;
    this.delta = 0;
    this.ax = 0; this.ay = 0;
    this.telemetry = blankTelemetry();
    /** What the rig actually applied after driver aids, for the HUD. */
    this.applied = { steer: 0, throttle: 0, brake: 0, nativeSteer: false };
    /** The rig's force feedback mix, for the settings panel. */
    this.ffb = { command: 0, torqueNm: 0, align: 0, damping: 0, friction: 0, softLock: 0, textureNm: 0, clipped: false, kickNm: 0 };
    /** The natively read wheel, if any, for the input layer. */
    this.device = { present: false, axes: [], buttons: 0, pov: -1, rimDeg: 0, halfLockDeg: 0 };
    this.stats = { ticks: 0, tickUsAvg: 0, tickUsMax: 0, overruns: 0, rateHz: 0 };
    this.boundaryHit = false;
    this.moneyShiftBlocked = false;

    /**
     * Per-frame context the game sets before `step`: driver aids, what the
     * course says, and the wheel as the webview sees it.
     */
    this.frame = {
      traction: false, abs: false, autoShift: false, ffbEnabled: true,
      offTrack: false, coneHits: 0, rimDeg: 0, halfLockDeg: 56,
    };
    this._shiftUp = false;
    this._shiftDown = false;
    this._inflight = false;
    this._servo = null;
    this.refresh();
  }

  get speed() { return Math.hypot(this.u, this.v); }

  refresh() {
    const p = this.p;
    this.a = lengthToFrontAxle(p);
    this.b = lengthToRearAxle(p);
    this.Fz0 = nominalTyreLoad(p);
  }

  /** Servo limits from the control profile, forwarded as parameter overrides. */
  get steeringServo() { return this._servo; }
  set steeringServo(cfg) {
    // The game sets this every frame; only a real change is worth a message.
    const old = this._servo;
    if (old && cfg && old.maxRateDegPerS === cfg.maxRateDegPerS &&
        old.accelDegPerS2 === cfg.accelDegPerS2 && old.lagS === cfg.lagS) return;
    if (!old && !cfg) return;
    this._servo = cfg;
    this.pushParams();
  }

  /**
   * Send every live parameter to the rig. Called after any edit (spec sheet,
   * setup nudge, reset) and on start. Cheap: one small JSON message.
   */
  pushParams() {
    const p = this.p;
    const servo = this._servo;
    rigNative.command({
      kind: "params",
      massKg: p.massKg, weightDistFront: p.weightDistFront, cgHeightM: p.cgHeightM,
      wheelbaseM: p.wheelbaseM, trackFrontM: p.trackFrontM, trackRearM: p.trackRearM,
      tireRadiusM: p.tireRadiusM, izzKgM2: p.izzKgM2,
      unsprungFrontKg: p.unsprungFrontKg, unsprungRearKg: p.unsprungRearKg,
      wheelInertiaFrontKgM2: p.wheelInertiaFrontKgM2, wheelInertiaRearKgM2: p.wheelInertiaRearKgM2,
      crr: p.crr, cdaM2: p.cdaM2, claM2: p.claM2, aeroFrontFrac: p.aeroFrontFrac,
      airDensityKgM3: p.airDensityKgM3,
      rsdFront: p.roll.rsdFront, hRollArmM: p.roll.hRollArmM, rcFrontM: p.roll.rcFrontM, rcRearM: p.roll.rcRearM,
      brakeTorqueMaxNm: p.brakeTorqueMaxNm, brakeBiasFront: p.brakeBiasFront,
      maxSteerDeg: p.maxSteerDeg,
      steerLagS: servo ? servo.lagS : p.steerLagS,
      steerRateDegS: servo ? servo.maxRateDegPerS : p.steerRateDegS,
      steerAccelDegS2: servo ? servo.accelDegPerS2 : 1e9,
      steeringRatio: p.steeringRatio,
      casterDeg: p.steering.casterDeg, kingpinOffsetTrailM: p.steering.kingpinOffsetTrailM,
      rackEfficiency: p.steering.rackEfficiency, torqueRatio: p.steering.torqueRatio ?? undefined,
      muLat: p.muLat, muLong: p.muLong, tireLoadSensitivity: p.tireLoadSensitivity, relaxLengthM: p.relaxLengthM,
      gearRatios: p.gearRatios, primaryReduction: p.primaryReduction, finalDrive: p.finalDrive,
      drivetrainEff: p.drivetrainEff, revLimitRpm: p.revLimitRpm, idleRpm: p.idleRpm,
      idleThrottleFrac: p.idleThrottleFrac, shiftTimeS: p.shiftTimeS,
      engineInertiaKgM2: p.engineInertiaKgM2, gearboxInertiaKgM2: p.gearboxInertiaKgM2,
    });
    this.refresh();
  }

  /** The course's barrier, if it has one (the oval). Cleared otherwise. */
  pushBoundary(track) {
    if (track && track.constrain && Array.isArray(track.center) && Number.isFinite(track.barrierOffset)) {
      rigNative.command({ kind: "boundary", centre: track.center, offsetM: track.barrierOffset });
    } else {
      rigNative.command({ kind: "clearBoundary" });
    }
  }

  /** The wheel profile and the force feedback block, to the rig. */
  pushControls(profile, etcPoints) {
    const ffb = profile.forceFeedback || {};
    rigNative.command({
      kind: "ffb",
      enabled: ffb.enabled !== false,
      gain: ffb.gain ?? 0.55, alignTorqueGain: ffb.alignTorqueGain ?? 1,
      roadTextureGain: ffb.roadTextureGain ?? 0.35, damping: ffb.damping ?? 0.15,
      friction: ffb.friction ?? 0.04, softLockGain: ffb.softLockGain ?? 1,
      minForce: ffb.minForce ?? 0, maxForceNm: ffb.maxForceNm ?? 5.5, invert: !!ffb.invert,
    });
    const isWheel = profile.kind === "wheel";
    const w = profile.wheel || {};
    const pedal = (cal) =>
      cal && cal.isAxis && cal.source != null
        ? {
            axis: cal.source, rawMin: cal.rawMin, rawMax: cal.rawMax,
            deadzoneLow: cal.deadzoneLow ?? 0, deadzoneHigh: cal.deadzoneHigh ?? 0,
            gamma: cal.gamma ?? 1, invert: !!cal.invert,
          }
        : null;
    rigNative.command({
      kind: "wheel",
      enabled: isWheel,
      steerAxis: profile.axes?.steer ?? 0,
      rotationDeg: w.rotationDeg ?? 900,
      mapping: w.mapping ?? "match-car",
      softLock: w.softLock !== false,
      centreTrimDeg: w.centreTrimDeg ?? 0,
      carRimHalfDeg: (this.p.maxSteerDeg * this.p.steeringRatio) / 2,
      throttle: isWheel ? pedal(profile.pedals?.throttle) : null,
      brake: isWheel ? pedal(profile.pedals?.brake) : null,
      etcPoints: etcPoints ?? [[0, 0], [100, 100]],
    });
  }

  /**
   * One frame: send the inputs, adopt the latest snapshot. `dt` is unused --
   * the rig keeps its own clock -- and kept for interface parity.
   */
  step(_dt, input) {
    this.exchange({ ...input, paused: false });
  }

  /** While the menu, pause or an editor is up: keep the rig informed and still. */
  hold() {
    this.exchange({ steer: 0, throttle: 0, brake: 0, paused: true });
  }

  exchange(input) {
    const f = this.frame;
    const msg = {
      steer: input.steer ?? 0,
      throttle: input.throttle ?? 0,
      brake: input.brake ?? 0,
      rimDeg: f.rimDeg,
      halfLockDeg: f.halfLockDeg,
      traction: !!f.traction,
      abs: !!f.abs,
      autoShift: !!f.autoShift,
      ffbEnabled: f.ffbEnabled !== false,
      paused: !!input.paused,
      offTrack: !!f.offTrack,
      coneHits: f.coneHits | 0,
      shiftUp: this._shiftUp,
      shiftDown: this._shiftDown,
    };
    this._shiftUp = false;
    this._shiftDown = false;
    f.coneHits = 0;
    // One exchange in flight at a time. If the previous one has not come
    // back (a hitch), this frame's inputs are simply the next ones sent.
    if (this._inflight) return;
    this._inflight = true;
    rigNative.frame(msg).then((snap) => {
      this._inflight = false;
      if (snap) this.apply(snap);
    }).catch(() => { this._inflight = false; });
  }

  apply(s) {
    const st = s.state, t = s.tel;
    this.X = st.x; this.Y = st.y; this.psi = st.psi;
    this.u = st.u; this.v = st.v; this.r = st.r;
    this.wF = st.wF; this.wR = st.wR;
    this.delta = st.delta;
    this.ax = t.axG * 9.81; this.ay = t.ayG * 9.81;
    const tel = this.telemetry;
    Object.assign(tel, t);
    tel.rollDeg = t.ayG * this.p.rollGradientDegG;
    tel.pitchDeg = t.axG * this.p.pitchGradientDegG;
    this.applied = s.applied;
    this.ffb = { ...s.ffb, kickNm: 0 };
    this.device = s.device;
    this.stats = s.stats;
    this.boundaryHit = s.boundaryHit;
    this.moneyShiftBlocked = s.moneyShiftBlocked;
    this.pt.adopt(s.pt);
  }

  respawn(x, y, psi, speed = 0) {
    // Locally too, so the course sees the new pose this frame rather than
    // one round trip later.
    this.X = x; this.Y = y; this.psi = psi;
    this.u = speed; this.v = 0; this.r = 0;
    this.wF = this.wR = speed / this.p.tireRadiusM;
    this.delta = 0;
    this.telemetry = blankTelemetry();
    rigNative.command({ kind: "respawn", x, y, psi, speed });
  }
}

/**
 * The `Powertrain` surface the game uses, backed by the rig's state and the
 * JS instance's curve lookups.
 */
class NativePowertrainProxy {
  constructor(js, car) {
    this.js = js;
    this.car = car;
    this._canShift = false;
    this._shiftRpm = js.optimalUpshiftRpm();
    this._downshiftSafe = false;
    this.indicatedTorqueNm = 0;
    this.plate = 0;
    this.slipping = true;
  }

  adopt(p) {
    const js = this.js;
    js.engineRpm = p.engineRpm;
    js.gear = p.gear;
    js.shiftTimer = p.shifting ? 1 : 0;
    js.slipping = p.slipping;
    this.slipping = p.slipping;
    this._canShift = p.canShift;
    this._shiftRpm = p.shiftRpm;
    this._downshiftSafe = p.downshiftSafe;
    this.indicatedTorqueNm = p.indicatedTorqueNm;
    this.plate = p.plate;
  }

  get engineRpm() { return this.js.engineRpm; }
  set engineRpm(v) { this.js.engineRpm = v; }
  get gear() { return this.js.gear; }
  set gear(v) { this.js.gear = v; }
  get shiftTimer() { return this.js.shiftTimer; }
  set shiftTimer(v) { this.js.shiftTimer = v; }
  get pendingGear() { return null; }
  set pendingGear(_v) {}
  get peakTorque() { return this.js.peakTorque; }
  get peakPower() { return this.js.peakPower; }

  canShift() { return this._canShift; }
  requestUpshift() { this.car._shiftUp = true; return true; }
  requestDownshift() { this.car._shiftDown = true; return true; }
  downshiftSafe() { return this._downshiftSafe; }
  optimalUpshiftRpm() { return this._shiftRpm; }
  wotTorque(rpm) { return this.js.wotTorque(rpm); }
  motoringTorque(rpm) { return this.js.motoringTorque(rpm); }
  indicatedTorque(rpm, demand) { return this.js.indicatedTorque(rpm, demand); }
  platePosition(rpm, demand) { return this.js.platePosition(rpm, demand); }
  rpmAtSpeed(v, gear) { return this.js.rpmAtSpeed(v, gear); }
  syncToWheel() {}
}

function blankTelemetry() {
  return {
    speed: 0, axG: 0, ayG: 0, bodySlipDeg: 0, yawRateDegS: 0,
    FzF: 0, FzR: 0, dFzLatF: 0, dFzLatR: 0,
    slipF: 0, slipR: 0, kappaF: 0, kappaR: 0,
    utilF: 0, utilR: 0, balance: 0,
    downforceN: 0, dragN: 0, driveForceN: 0, steerDeg: 0,
    rollDeg: 0, pitchDeg: 0, locked: false,
    kingpinTorqueNm: 0, rimTorqueNm: 0, trailFm: 0, mechTrailM: 0,
  };
}
