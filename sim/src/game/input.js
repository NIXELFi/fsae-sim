// Driver input, through whichever device is plugged in.
//
// What each device *is* -- axis and button assignments, pedal calibration,
// steering curve and steering dynamics -- lives in `controlProfiles.js`. This
// file reads whatever the active profile says. Adding a device means adding a
// profile, not editing this.
//
// The Xbox layout below is still the reference for anything reporting the
// Gamepad API's "standard" mapping, which is what an Xbox One/Series pad gives
// on Windows over both USB and Bluetooth. Wheels do NOT use standard mapping
// and their axis assignments vary by vendor, which is why they get a
// calibration flow rather than a fixed table:
//
//   axes[0]  left stick X      steering
//   axes[1]  left stick Y      (unused)
//   buttons[6] LT              brake      -- analog, read .value not .pressed
//   buttons[7] RT              throttle   -- analog
//   buttons[4] LB              downshift
//   buttons[5] RB              upshift
//   buttons[0] A               (hold) clutch-free launch assist
//   buttons[1] B               reset to the last point on course
//   buttons[2] X               toggle traction control
//   buttons[3] Y               change camera
//   buttons[8] View            restart the run
//   buttons[9] Menu            pause
//
// Steering gets a small deadzone and a squared response curve. The curve is not
// a gimmick: full lock is 28 degrees and the tyre peaks at 8.5 degrees of slip,
// so a linear stick makes the useful travel about a third of the range. Squaring
// it puts the precision where the grip is.

import {
  ControlSettings,
  applyPedal,
  applySteeringCurve,
  detectProfile, usableLockFrac, stepPedal } from "./controlProfiles.js";
import { presetFor, presetPaths } from "./wheelPresets.js";

// Only used to decide whether the pad is being touched at all; the real
// deadzone comes from the active profile.
const ACTIVITY_THRESHOLD = 0.10;

export class Input {
  constructor() {
    this.settings = new ControlSettings();
    this.profile = this.settings.active();
    /** Set when the driver picks a profile by hand, so detection stops overriding it. */
    this.pinned = !!this.settings.pinned;
    /** Raw pedal readings, for the calibration UI to watch. */
    this.rawAxes = [];
    this.mouseSteer = 0;
    this.keys = new Set();
    this.padIndex = null;
    this.padName = "";
    this.usingPad = false;

    // Edge-triggered actions, consumed once per frame by the game.
    this.edges = {
      upshift: false, downshift: false, reset: false, restart: false,
      pause: false, camera: false, traction: false, mapEditor: false,
      home: false,
      setupPrev: false, setupNext: false, setupUp: false, setupDown: false,
    };
    // D-pad up/down auto-repeat. At 0.1% a press, walking roll stiffness a few
    // points would be dozens of taps, so held presses repeat and then speed up.
    this._holdSince = { setupUp: 0, setupDown: 0 };
    this._nextRepeat = { setupUp: 0, setupDown: 0 };
    /** Multiplier the game applies to the step size while a d-pad is held. */
    this.setupHoldScale = 1;
    this._prevButtons = [];
    this._prevKeys = new Set();

    this.state = { steer: 0, throttle: 0, brake: 0, launch: false };
    /**
     * What the game tells the input layer about the car each frame, for the
     * keyboard's speed-sensitive lock: speed and the numbers that turn a
     * lateral g into a steer angle. Zero speed means full lock, so a fresh
     * Input without a game behind it behaves as before.
     */
    this.carSpeed = 0;
    this.carWheelbaseM = 1.53;
    this.carPeakSlipDeg = 8.5;
    /** Ramped keyboard pedals (see controlProfiles.stepPedal) and their clock. */
    this.kbPedal = { throttle: 0, brake: 0 };
    this._kbPedalAt = 0;
    /**
     * The rim, for force feedback: measured angle in degrees (right positive,
     * as the device reports it) and the car's lock at the rim. Only meaningful
     * on a wheel profile; zero otherwise.
     */
    this.rim = { deg: 0, halfLockDeg: 0 };
    /**
     * A wheel read natively by the desktop rig (DirectInput), presented here
     * as if it were a Gamepad API device so profiles, calibration and the
     * live axis monitor all work unchanged. Set by the game each frame;
     * null in a browser. While present it takes precedence over the
     * Gamepad API, which cannot see an exclusively acquired device anyway.
     */
    this.nativeDevice = null;
    this.nativeName = "wheel (native)";

    // Typing a throttle-map value into a number field must not also stand on
    // the throttle, so keys aimed at a form control never reach the car.
    const isTyping = (e) => /^(INPUT|TEXTAREA|SELECT)$/.test(e.target?.tagName ?? "");

    addEventListener("keydown", (e) => {
      if (e.repeat || isTyping(e)) return;
      this.keys.add(e.code);
      // Stop the page scrolling out from under the game.
      if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "Space"].includes(e.code)) {
        e.preventDefault();
      }
    });
    addEventListener("keyup", (e) => this.keys.delete(e.code));
    addEventListener("blur", () => this.keys.clear());

    // Mouse steering, for the keyboard profile. Uses movementX so it keeps
    // working under pointer lock and does not stop at the edge of the window.
    addEventListener("mousemove", (e) => {
      const m = this.profile.mouse;
      if (!m || !m.enabled) return;
      const span = Math.max(m.pixelsForFullLock, 1);
      this.mouseSteer = Math.max(-1, Math.min(1, this.mouseSteer + (-e.movementX * 2) / span));
    });

    addEventListener("gamepadconnected", (e) => {
      this.padIndex = e.gamepad.index;
      this.padName = e.gamepad.id;
      // Pick a profile from the vendor string unless the driver has chosen one
      // by hand. The Gamepad API gives a free-form id and nothing else, so this
      // is pattern matching and will sometimes be wrong -- it only sets the
      // starting point, and the choice is remembered once overridden.
      if (!this.pinned) {
        const guess = detectProfile(e.gamepad.id);
        this.settings.setActive(guess);
        this.profile = this.settings.active();
        this.onProfileChange?.(guess, e.gamepad.id);
      }
      this.onPadChange?.(true, e.gamepad.id);
    });
    addEventListener("gamepaddisconnected", (e) => {
      if (e.gamepad.index === this.padIndex) {
        this.padIndex = null;
        this.usingPad = false;
        this.onPadChange?.(false, "");
      }
    });
  }

  /** Switch profile by hand. Stops device detection from overriding it. */
  setProfile(id) {
    this.pinned = true;
    this.settings.pinned = true;
    this.settings.setActive(id);
    this.profile = this.settings.active();
    this.onProfileChange?.(id, this.padName);
  }

  /** Re-read the profile after the settings UI changes a value. */
  refreshProfile() {
    this.profile = this.settings.active();
  }

  /**
   * Servo limits for `BicycleModel.steeringServo`.
   *
   * These are per-device on purpose: on a pad they describe how fast the
   * software is willing to move the steering, and on a wheel how fast arms and
   * a rack actually can.
   */
  steeringServo() {
    const st = this.profile.steering;
    return {
      maxRateDegPerS: st.maxRateDegPerS,
      accelDegPerS2: st.accelDegPerS2,
      lagS: st.lagS,
      slipCapDeg: st.slipCapDeg ?? 0,
      rateSpeedRefMps: st.rateSpeedRefMps ?? 0,
      rateSpeedExp: st.rateSpeedExp ?? 1.5,
    };
  }

  pad() {
    const nd = this.nativeDevice;
    if (nd && nd.present) return this.syntheticPad(nd);
    const pads = navigator.getGamepads?.() ?? [];
    if (this.padIndex != null && pads[this.padIndex]) return pads[this.padIndex];
    // Some browsers only populate the array after the first button press, so
    // adopt whatever shows up.
    for (const p of pads) {
      if (p && p.connected) {
        this.padIndex = p.index;
        this.padName = p.id;
        return p;
      }
    }
    return null;
  }

  /**
   * The rig's device state in Gamepad API shape. Buttons are a bitmask; the
   * hat becomes the standard-mapping d-pad indices 12-15 so the setup
   * controls work from a wheel's hat.
   */
  syntheticPad(nd) {
    // Buttons: 32 per device, base first, so a button box's buttons sit at
    // 32 and up. The base's hat becomes the standard d-pad indices 12-15.
    const words = Array.isArray(nd.buttons) ? nd.buttons : [nd.buttons | 0];
    const buttons = [];
    for (const w of words) {
      for (let i = 0; i < 32; i++) buttons.push({ pressed: !!(w & (1 << i)), value: w & (1 << i) ? 1 : 0 });
    }
    if (nd.pov >= 0) {
      const dir = Math.round(nd.pov / 9000) % 4; // 0 up, 1 right, 2 down, 3 left
      buttons[12] = { pressed: dir === 0, value: dir === 0 ? 1 : 0 };
      buttons[15] = { pressed: dir === 1, value: dir === 1 ? 1 : 0 };
      buttons[13] = { pressed: dir === 2, value: dir === 2 ? 1 : 0 };
      buttons[14] = { pressed: dir === 3, value: dir === 3 ? 1 : 0 };
    }
    if (this._nativeAnnounced !== this.nativeName) {
      this._nativeAnnounced = this.nativeName;
      this.padName = this.nativeName;
      if (!this.pinned) {
        this.settings.setActive("wheel");
        this.profile = this.settings.active();
      }
      this.applyWheelPreset(this.nativeName);
      this.onProfileChange?.(this.settings.activeId, this.nativeName);
      this.onPadChange?.(true, this.nativeName);
    }
    return { id: this.nativeName, index: -1, connected: true, mapping: "", axes: Array.from(nd.axes), buttons };
  }

  /**
   * First time a given base is seen: rated torque, rotation, axis guesses
   * and a gain that fits the motor, from `wheelPresets.js`. Only once per
   * base, so the driver's own calibration survives every later launch.
   * @returns the preset applied, or null if this base was already set up
   */
  applyWheelPreset(name) {
    const s = this.settings;
    if (s.read("wheel", "wheel.presetApplied") === name) return null;
    const preset = presetFor(name);
    for (const [path, value] of Object.entries(presetPaths(preset))) s.set("wheel", path, value);
    s.set("wheel", "wheel.presetApplied", name);
    this.lastPreset = preset;
    this.profile = s.active();
    return preset;
  }

  /** Rumble. Ignored silently on pads or browsers without an actuator. */
  rumble(strong, weak, ms) {
    const p = this.pad();
    const act = p?.vibrationActuator;
    if (!act?.playEffect) return;
    act.playEffect("dual-rumble", {
      duration: ms,
      strongMagnitude: Math.max(0, Math.min(1, strong)),
      weakMagnitude: Math.max(0, Math.min(1, weak)),
    }).catch(() => {});
  }

  /**
   * Turn a wheel's rim reading into a normalised steering command.
   *
   * The mapping choice matters more than any other wheel setting.
   *
   * "match-car" is the honest one: the rim turns through the car's real
   * steering ratio, so a driver's hand position corresponds to an actual front
   * wheel angle. For SDM26 that is 28 degrees of lock through the measured
   * 4.411 ratio, or 247 degrees at the rim lock to lock -- set the wheel's
   * driver software to 247 and what you feel is what the tyres are doing.
   * Leave a 900-degree wheel on 900 and this mapping correctly uses only the
   * first 27% of its travel,
   * which feels wrong because the *wheel* is configured wrong.
   *
   * "scale-to-lock" spreads whatever rotation the wheel is set to across full
   * lock. Nobody has to reconfigure anything, but the ratio becomes a fiction
   * and the steering is eight times slower than the real car's.
   */
  wheelSteer(raw, prof) {
    const w = prof.wheel || {};
    const rotation = Math.max(w.rotationDeg || 900, 1);

    // Rim angle in degrees, from the -1..1 axis, with the centre trim applied.
    const rimDeg = raw * (rotation / 2) - (w.centreTrimDeg || 0);

    const carRimHalf = (this.carLockDeg ?? 28) * (this.carSteeringRatio ?? 4) * 0.5;
    let norm;
    if (w.mapping === "match-car") {
      // The car's rim travel is lock * ratio; anything beyond it is over-lock.
      norm = rimDeg / Math.max(carRimHalf, 1e-6);
      this.rim.halfLockDeg = carRimHalf;
    } else {
      norm = rimDeg / (rotation / 2);
      this.rim.halfLockDeg = rotation / 2;
    }
    this.rim.deg = rimDeg;

    if (w.softLock !== false) norm = Math.max(-1, Math.min(1, norm));
    // A wheel measures hand position directly, so the curve is a no-op unless
    // the driver has deliberately dialled one in.
    return -applySteeringCurve(prof.steering, Math.max(-1, Math.min(1, norm)));
  }

  /** Read the pad and keyboard into `state` and `edges`. Call once per frame. */
  poll() {
    for (const k of Object.keys(this.edges)) this.edges[k] = false;

    const p = this.pad();
    const prof = this.profile;
    let steer = 0, throttle = 0, brake = 0, launch = false;

    if (p) {
      this.rawAxes = Array.from(p.axes);
      const axes = prof.axes || { steer: 0 };
      const raw = p.axes[axes.steer] ?? 0;

      if (prof.kind === "wheel") {
        steer = this.wheelSteer(raw, prof);
      } else {
        // Negative because pushing the stick left must steer left, and the
        // vehicle model takes left as positive.
        steer = -applySteeringCurve(prof.steering, raw);
      }

      const readPedal = (which) => {
        const cal = prof.pedals?.[which];
        if (!cal || cal.source === null) return 0;
        const rawValue = cal.isAxis
          ? (p.axes[cal.source] ?? 0)
          : (p.buttons[cal.source]?.value ?? 0);
        return applyPedal(cal, rawValue);
      };
      throttle = readPedal("throttle");
      brake = readPedal("brake");

      const B = prof.buttons || {};
      const pressed = (i) => i != null && !!p.buttons[i]?.pressed;
      const wasPressed = (i) => i != null && !!this._prevButtons[i];
      const edge = (i) => pressed(i) && !wasPressed(i);

      launch = pressed(B.launch);
      this.edges.upshift = edge(B.upshift);
      this.edges.downshift = edge(B.downshift);
      this.edges.reset = edge(B.reset);
      this.edges.traction = edge(B.traction);
      this.edges.camera = edge(B.camera);
      this.edges.restart = edge(B.restart);
      this.edges.pause = edge(B.pause);
      this.edges.home = edge(B.home);

      // D-pad: left/right pick the setting, up/down move it.
      this.edges.setupPrev = edge(B.dpadLeft);
      this.edges.setupNext = edge(B.dpadRight);
      this.applyRepeat("setupUp", pressed(B.dpadUp), edge(B.dpadUp));
      this.applyRepeat("setupDown", pressed(B.dpadDown), edge(B.dpadDown));

      this._prevButtons = p.buttons.map((b) => b.pressed);
      if (throttle > 0.02 || brake > 0.02 || Math.abs(raw) > ACTIVITY_THRESHOLD ||
          p.buttons.some((b) => b.pressed)) {
        this.usingPad = true;
      }
    }

    // Keyboard: always live, so the game is playable (and testable) without a
    // pad. Pad input wins whenever it is non-zero.
    const k = this.keys;
    const kSteer = (k.has("ArrowLeft") || k.has("KeyA") ? 1 : 0) -
                   (k.has("ArrowRight") || k.has("KeyD") ? 1 : 0);
    const kThrottle = k.has("ArrowUp") || k.has("KeyW") ? 1 : 0;
    const kBrake = k.has("ArrowDown") || k.has("KeyS") ? 1 : 0;

    // Digital steering only gets the lock the car can use at this speed; see
    // controlProfiles.usableLockFrac. The same cap applies to the mouse, which
    // has a position but no stop.
    const lockFrac = usableLockFrac(prof.steering?.speedSensitive, this.carSpeed, {
      wheelbaseM: this.carWheelbaseM,
      maxSteerDeg: this.carLockDeg ?? 28,
      peakSlipAngleDeg: this.carPeakSlipDeg,
    });
    if (kSteer !== 0 && steer === 0) steer = kSteer * lockFrac;

    // Mouse steering, when the keyboard profile has it switched on. It decays
    // back to centre like a self-centring wheel, otherwise the car holds a
    // steering angle forever after the mouse stops moving.
    const mouse = prof.mouse;
    if (mouse && mouse.enabled && steer === 0) {
      steer = this.mouseSteer * lockFrac;
      if (mouse.selfCentre) {
        const decay = (mouse.selfCentreRateDegPerS / Math.max(1, 28)) * (1 / 60);
        this.mouseSteer -= Math.sign(this.mouseSteer) * Math.min(Math.abs(this.mouseSteer), decay);
      }
    }
    // Keyboard pedals are ramped rather than stepped, at the rates the
    // profile sets (no rates: a plain step, as before). Timed off the wall
    // clock because poll() is called once per rendered frame.
    {
      const now = performance.now();
      const dt = this._kbPedalAt ? Math.min(0.1, (now - this._kbPedalAt) / 1000) : 0;
      this._kbPedalAt = now;
      const thrCfg = prof.pedals?.throttle ?? {};
      const brkCfg = prof.pedals?.brake ?? {};
      this.kbPedal.throttle = stepPedal(this.kbPedal.throttle, kThrottle, thrCfg, dt);
      this.kbPedal.brake = stepPedal(this.kbPedal.brake, kBrake, brkCfg, dt);
    }
    if (this.kbPedal.throttle > 0 && throttle === 0) throttle = this.kbPedal.throttle;
    if (this.kbPedal.brake > 0 && brake === 0) brake = this.kbPedal.brake;
    if (k.has("Space")) launch = true;

    const kEdge = (code) => k.has(code) && !this._prevKeys.has(code);
    if (kEdge("ShiftRight") || kEdge("KeyE")) this.edges.upshift = true;
    if (kEdge("ShiftLeft") || kEdge("KeyQ")) this.edges.downshift = true;
    if (kEdge("KeyR")) this.edges.reset = true;
    if (kEdge("Backspace")) this.edges.restart = true;
    if (kEdge("KeyC")) this.edges.camera = true;
    if (kEdge("KeyT")) this.edges.traction = true;
    if (kEdge("KeyM")) this.edges.mapEditor = true;
    if (kEdge("KeyH")) this.edges.home = true;
    // Walkaround camera nudges, live rather than edge-triggered so holding a
    // key sweeps smoothly.
    this.walkaround = {
      // Not Q/E: those are the gearshift. Comma and period sit next to each
      // other and are otherwise unused.
      turn: (k.has("Comma") ? 1 : 0) - (k.has("Period") ? 1 : 0),
      rise: (k.has("KeyR") ? 1 : 0) - (k.has("KeyF") ? 1 : 0),
      zoom: (k.has("BracketRight") ? 1 : 0) - (k.has("BracketLeft") ? 1 : 0),
    };
    if (kEdge("Escape") || kEdge("KeyP")) this.edges.pause = true;

    // Keyboard stand-in for the d-pad: [ ] pick the setting, - = move it.
    if (kEdge("BracketLeft")) this.edges.setupPrev = true;
    if (kEdge("BracketRight")) this.edges.setupNext = true;
    if (!p) {
      this.applyRepeat("setupUp", k.has("Equal"), kEdge("Equal"));
      this.applyRepeat("setupDown", k.has("Minus"), kEdge("Minus"));
    }
    this._prevKeys = new Set(k);

    this.state.steer = clamp(steer, -1, 1);
    this.state.throttle = clamp(throttle, 0, 1);
    this.state.brake = clamp(brake, 0, 1);
    this.state.launch = launch;
    return this.state;
  }
}

/**
 * Turn a held button into a repeating edge: fires immediately, pauses, then
 * repeats, then speeds up. `setupHoldScale` lets the game take bigger steps
 * once the driver has clearly committed to a big change, while a single tap
 * always moves the minimum 0.1%.
 */
Input.prototype.applyRepeat = function applyRepeat(name, held, justPressed) {
  const now = performance.now();
  if (justPressed) {
    this.edges[name] = true;
    this._holdSince[name] = now;
    this._nextRepeat[name] = now + 420;
    this.setupHoldScale = 1;
    return;
  }
  if (!held) { this._holdSince[name] = 0; return; }
  if (now >= this._nextRepeat[name]) {
    this.edges[name] = true;
    const heldFor = now - this._holdSince[name];
    this.setupHoldScale = heldFor > 2200 ? 5 : 1;
    this._nextRepeat[name] = now + (heldFor > 2200 ? 70 : 85);
  }
};

function analog(button) {
  if (!button) return 0;
  // Triggers report through .value; some drivers only set .pressed.
  const v = typeof button.value === "number" ? button.value : 0;
  return v > 0.02 ? Math.min(1, v) : button.pressed ? 1 : 0;
}

// NaN fails both comparisons and would pass straight through into the car.
function clamp(x, lo, hi) { return Number.isFinite(x) ? (x < lo ? lo : x > hi ? hi : x) : 0; }
