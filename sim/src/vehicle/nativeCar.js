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
    this.ffb = { command: 0, torqueNm: 0, align: 0, damping: 0, friction: 0, oversteer: 0, softLock: 0, textureNm: 0, asphaltNm: 0, clipped: false, kickNm: 0 };
    /** The natively read wheel, if any, for the input layer. */
    this.device = { present: false, forceFeedback: false, axes: [], buttons: 0, pov: -1, rimDeg: 0, halfLockDeg: 0 };
    this.stats = { ticks: 0, tickUsAvg: 0, tickUsMax: 0, overruns: 0, rateHz: 0 };
    this.boundaryHit = false;
    this.moneyShiftBlocked = false;
    /**
     * The respawn token we last sent, against the one coming back.
     *
     * `respawn` sets the pose here immediately so the course sees it this
     * frame, but `apply` then overwrites the whole state from the next
     * snapshot to arrive -- which, for the frame or two an IPC round trip
     * takes, was computed BEFORE the rig drained the respawn command. The
     * effect was a car put on the start line that reported the speed it was
     * doing when the driver hit restart, which started the lap clock on the
     * spot. A snapshot carrying a different token is from before the respawn
     * and is dropped.
     *
     * The rig echoes this back unchanged rather than keeping a count of its
     * own, so the two agree exactly whatever either side has been through --
     * a reloaded page against a rig that never stopped, included.
     */
    this._respawnSeq = 0;
    /**
     * ...but never for longer than this. A respawn command is fire and
     * forget, and one that never arrives would otherwise leave the car frozen
     * on the line forever while the rig carried on somewhere else. Zero to
     * begin with, so the first snapshot after a page load simply adopts
     * whatever the rig is echoing instead of waiting for a token it never
     * sent.
     */
    this._respawnWaitUntil = 0;

    /**
     * Per-frame context the game sets before `step`: driver aids, what the
     * course says, and the wheel as the webview sees it.
     */
    this.frame = {
      traction: false, abs: false, autoShift: false, ffbEnabled: true,
      offTrack: false, coneHits: 0, rimDeg: 0, halfLockDeg: 179, launch: false,
    };
    this._shiftUp = false;
    this._shiftDown = false;
    this._inflight = false;
    this._servo = null;
    /**
     * When the pose currently in X/Y/psi arrived, `performance.now()` ms,
     * and whether the rig was being HELD when the last frame went out.
     *
     * The rig copies its snapshot out at whatever phase of the 1 kHz loop
     * the IPC lands on, and the page draws it a frame later: a +-0.5 ms tick
     * quantisation plus 0.2-2 ms of scheduling jitter on top of a 7 ms frame,
     * which at 25 m/s is a 30% frame-to-frame irregularity in the apparent
     * motion -- visible micro-stutter on close cones. `Game.render` uses
     * this stamp to dead-reckon the DRAWN pose forward by the snapshot's
     * age; the raw state stays what the rig said, for the timing, the log
     * and the telemetry. Zero until a snapshot has been adopted, and reset
     * to zero by `respawn` so nothing extrapolates a pose the rig has not
     * confirmed yet.
     */
    this.appliedAt = 0;
    this.held = false;
    /** The frame message, reused: one IPC send a frame, same shape every time. */
    this._msg = {
      steer: 0, throttle: 0, brake: 0, rimDeg: 0, halfLockDeg: 179,
      traction: false, abs: false, autoShift: false, ffbEnabled: true,
      paused: false, offTrack: false, coneHits: 0,
      shiftUp: false, shiftDown: false, launch: false,
    };
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
        old.accelDegPerS2 === cfg.accelDegPerS2 && old.lagS === cfg.lagS &&
        (old.slipCapDeg ?? 0) === (cfg.slipCapDeg ?? 0) &&
        (old.rateSpeedRefMps ?? 0) === (cfg.rateSpeedRefMps ?? 0)) return;
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
      rsdFront: p.roll.rsdFront, rcFrontM: p.roll.rcFrontM, rcRearM: p.roll.rcRearM,
      brakeTorqueMaxNm: p.brakeTorqueMaxNm, brakeBiasFront: p.brakeBiasFront,
      maxSteerDeg: p.maxSteerDeg,
      steerLagS: servo ? servo.lagS : p.steerLagS,
      steerRateDegS: servo ? servo.maxRateDegPerS : p.steerRateDegS,
      steerAccelDegS2: servo ? servo.accelDegPerS2 : 1e9,
      steerSlipCapDeg: servo ? (servo.slipCapDeg ?? 0) : 0,
      steerRateSpeedRefMps: servo ? (servo.rateSpeedRefMps ?? 0) : 0,
      steerRateSpeedExp: servo ? (servo.rateSpeedExp ?? 1.5) : 1.5,
      steeringRatio: p.steeringRatio,
      casterDeg: p.steering.casterDeg, kingpinOffsetTrailM: p.steering.kingpinOffsetTrailM,
      rackEfficiency: p.steering.rackEfficiency, torqueRatio: p.steering.torqueRatio ?? undefined,
      diffPowerLock: p.diff?.powerLock, diffCoastLock: p.diff?.coastLock,
      diffPreloadNm: p.diff?.preloadNm,
      muLat: p.muLat, muLong: p.muLong, tireLoadSensitivity: p.tireLoadSensitivity, relaxLengthM: p.relaxLengthM,
      frontGripFactor: p.frontGripFactor,
      gearRatios: p.gearRatios, primaryReduction: p.primaryReduction, finalDrive: p.finalDrive,
      drivetrainEff: p.drivetrainEff, revLimitRpm: p.revLimitRpm, idleRpm: p.idleRpm,
      idleThrottleFrac: p.idleThrottleFrac, launchRpm: p.launchRpm, shiftTimeS: p.shiftTimeS,
      shiftReintroS: p.shiftReintroS,
      revLimitHystRpm: p.revLimitHystRpm, launchHystRpm: p.launchHystRpm,
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
      gain: ffb.gain ?? 0.37, alignTorqueGain: ffb.alignTorqueGain ?? 1,
      roadTextureGain: ffb.roadTextureGain ?? 0.35, damping: ffb.damping ?? 0.10,
      friction: ffb.friction ?? 0.04, softLockGain: ffb.softLockGain ?? 1,
      minForce: ffb.minForce ?? 0, maxForceNm: ffb.maxForceNm ?? 5.5, invert: !!ffb.invert,
      // Defaulted here as well as in the profile: a settings file written
      // before these existed must still get the compressor, not a hard clip.
      gamma: ffb.gamma ?? 0.75, knee: ffb.knee ?? 0.6,
      parkFriction: ffb.parkFriction ?? 0.10, stopDamping: ffb.stopDamping ?? 0.35,
      understeerEffect: ffb.understeerEffect ?? 0, oversteerEffect: ffb.oversteerEffect ?? 0,
      asphaltVibration: ffb.asphaltVibration ?? 0,
      // 1 = v1, 2 = v2, 3 = v2.1 (see `FfbConfig::model` in rig.rs).
      model: ffb.model === 2 || ffb.model === 3 ? ffb.model : 1,
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
      deviceName: w.deviceName ?? "",
      // The rack's MEASURED stop at the rim, 179 deg for SDM26. Not
      // maxSteerDeg x steeringRatio: the real rack is progressive, so the
      // nominal ratio does not put the stop anywhere near the right place.
      carRimHalfDeg: this.p.steering.rimLockDeg ?? 179,
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
    this.held = false;
    this.exchange(input, false);
  }

  /** While the menu, pause or an editor is up: keep the rig informed and still. */
  hold() {
    this.held = true;
    this.exchange(HOLD_INPUT, true);
  }

  exchange(input, paused) {
    const f = this.frame;
    // One exchange in flight at a time. If the previous one has not come
    // back yet, this frame's continuous inputs are simply superseded by the
    // next frame's -- but the EVENTS (a shift, a cone) must not be: they
    // stay pending until a frame actually goes out. Clearing them here
    // before this check silently ate most gear changes, because a 60 Hz
    // frame and an IPC round trip are about the same length.
    if (this._inflight) return;
    // The one message object, filled in place: `invoke` serialises it
    // synchronously, so nothing holds on to it after this call.
    const msg = this._msg;
    msg.steer = input.steer ?? 0;
    msg.throttle = input.throttle ?? 0;
    msg.brake = input.brake ?? 0;
    msg.rimDeg = f.rimDeg;
    msg.halfLockDeg = f.halfLockDeg;
    msg.traction = !!f.traction;
    msg.abs = !!f.abs;
    msg.autoShift = !!f.autoShift;
    msg.ffbEnabled = f.ffbEnabled !== false;
    msg.paused = !!paused;
    msg.offTrack = !!f.offTrack;
    msg.coneHits = f.coneHits | 0;
    msg.shiftUp = this._shiftUp;
    msg.shiftDown = this._shiftDown;
    msg.launch = !!f.launch;
    this._shiftUp = false;
    this._shiftDown = false;
    f.coneHits = 0;
    this._inflight = true;
    rigNative.frame(msg).then((snap) => {
      this._inflight = false;
      if (snap) this.apply(snap);
    }).catch(() => { this._inflight = false; });
  }

  apply(s) {
    // The device is the driver's HANDS, and it arrives in the same snapshot as
    // the car. Adopted before anything below can decide to drop the rest:
    // gating it behind the respawn check froze the wheel, the pedals and the
    // paddles for as long as the gate lasted -- two frames normally, and the
    // whole timeout when a respawn command went missing.
    this.device = s.device;
    this.stats = s.stats;

    // From before the respawn we asked for; see `_respawnSeq`.
    if (typeof s.respawnSeq === "number" && s.respawnSeq !== this._respawnSeq) {
      if (performance.now() < this._respawnWaitUntil) return;
      // Gave up waiting: adopt it and re-sync, so one lost command cannot
      // wedge the car permanently.
      this._respawnSeq = s.respawnSeq;
    }
    const st = s.state, t = s.tel;
    this.X = st.x; this.Y = st.y; this.psi = st.psi;
    this.u = st.u; this.v = st.v; this.r = st.r;
    this.appliedAt = performance.now();
    this.wF = st.wF; this.wR = st.wR;
    this.delta = st.delta;
    this.ax = t.axG * 9.81; this.ay = t.ayG * 9.81;
    const tel = this.telemetry;
    Object.assign(tel, t);
    tel.rollDeg = t.ayG * this.p.rollGradientDegG;
    tel.pitchDeg = t.axG * this.p.pitchGradientDegG;
    this.applied = s.applied;
    // Into the one ffb record rather than a fresh spread a frame; the
    // settings panel reads it live. The native mix has no cone kick.
    Object.assign(this.ffb, s.ffb);
    this.ffb.kickNm = 0;
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
    this.appliedAt = 0;
    // Everything the rig sends back until it has applied this is from the
    // drive we just ended. 400 ms is many round trips at any frame rate.
    this._respawnSeq = (this._respawnSeq + 1) >>> 0;
    this._respawnWaitUntil = performance.now() + 400;
    rigNative.command({ kind: "respawn", x, y, psi, speed, seq: this._respawnSeq });
  }
}

/** What `hold` sends: no driver input, and the rig told to stand still. */
const HOLD_INPUT = Object.freeze({ steer: 0, throttle: 0, brake: 0 });

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
    this.limiterCut = false;
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
    // Undefined from a rig older than the hard-cut limiter; see liveAudioState.
    this.limiterCut = p.limiterCut;
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
    kappaRL: 0, kappaRR: 0, utilRL: 0, utilRR: 0, diffNm: 0,
    kingpinTorqueNm: 0, rimTorqueNm: 0, trailFm: 0, mechTrailM: 0, scrubMomentNm: 0,
  };
}
