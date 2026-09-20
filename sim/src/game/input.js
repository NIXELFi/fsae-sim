// Driver input, through whichever device is plugged in.
//
// What each device *is* -- axis and button assignments, pedal calibration,
// steering curve and steering dynamics -- lives in `controlProfiles.js`. This
// file reads whatever the active profile says. Adding a device means adding a
// profile, not editing this.
//
// Nothing here names a key or a button any more. Every control is a binding in
// the active profile, the settings panel rebinds them by watching what you
// press, and the table of what the controls ARE lives in `controlBindings.js`.
// What follows is the shipped default for a pad, not a rule.
//
// The Xbox layout below is the reference for anything reporting the Gamepad
// API's "standard" mapping, which is what an Xbox One/Series pad gives on
// Windows over both USB and Bluetooth. Wheels do NOT use standard mapping and
// their axis assignments vary by vendor, which is why they get a detect-and-
// calibrate flow rather than a fixed table:
//
//   axes[0]  left stick X      steering
//   axes[1]  left stick Y      (unused)
//   buttons[6] LT              brake -- analog, read .value not .pressed
//   buttons[7] RT              throttle -- analog
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
// a gimmick: full lock is 46 degrees and the tyre peaks at 7.3 degrees of slip,
// so a linear stick makes the useful travel about a third of the range. Squaring
// it puts the precision where the grip is.

import {
  ControlSettings,
  applyPedal,
  applySteeringCurve,
  detectProfile, usableLockFrac, stepPedal } from "./controlProfiles.js";
import { roadFromRimDeg, rimFromRoadDeg } from "../vehicle/params.js";
import { presetFor, presetPaths, PRESET_VERSION } from "./wheelPresets.js";
import { ACTIONS, buttonSlot } from "./controlBindings.js";

/**
 * Held, not tapped. These are read as a level every frame and never appear in
 * `edges`; the pedals and the steering keys are the obvious ones, and the
 * launch assist is held because letting go is how you release the clutch.
 */
const HOLD_ACTIONS = new Set(["steerLeft", "steerRight", "throttle", "brake", "launch"]);
/** Auto-repeat while held; see `applyRepeat`. */
const REPEAT_ACTIONS = new Set(["setupUp", "setupDown"]);
/** Everything else: one edge per press. */
const EDGE_ACTIONS = ACTIONS.filter(
  (a) => !HOLD_ACTIONS.has(a.id) && !REPEAT_ACTIONS.has(a.id),
);

/**
 * Below this a steering command is noise -- a stick inside its deadzone, a key
 * ramp with a millimetre left to unwind -- and crediting it to a device would
 * let a controller left on the desk reclassify a whole session on a wheel.
 */
const SOURCE_THRESHOLD = 0.02;

/** First virtual button index for a native base's hat switch: above 4 devices x 32 real buttons. */
export const HAT_BASE = 128;
/** Hat direction (0 up, 1 right, 2 down, 3 left) for each virtual hat button (up, down, left, right). */
const HAT_DIRS = [0, 2, 3, 1];

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

    // Edge-triggered actions, consumed once per frame by the game. Built from
    // the action table rather than listed here, so an action that the settings
    // panel offers to bind cannot be one the game never reads.
    this.edges = {};
    for (const a of ACTIONS) if (!HOLD_ACTIONS.has(a.id)) this.edges[a.id] = false;
    /**
     * Menu navigation from the device, for the screens the driving edges do
     * not reach: the home screen, the pause and finish cards, the replay
     * transport. Read off the same bound buttons -- the d-pad or hat moves,
     * launch (A) selects, recover (B) backs out, the shift paddles switch
     * tabs and pause (Menu) starts the engine -- so a wheel with its own
     * mapping navigates with the same buttons it drives with.
     */
    this.menu = { up: false, down: false, left: false, right: false,
      accept: false, back: false, prevTab: false, nextTab: false, start: false };
    // D-pad up/down auto-repeat. At 0.1% a press, walking roll stiffness a few
    // points would be dozens of taps, so held presses repeat and then speed up.
    this._holdSince = { setupUp: 0, setupDown: 0 };
    this._nextRepeat = { setupUp: 0, setupDown: 0 };
    /** Multiplier the game applies to the step size while a d-pad is held. */
    this.setupHoldScale = 1;
    this._prevButtons = [];
    this._prevKeys = new Set();
    /** Walkaround camera nudges from the keyboard; see `poll`. */
    this.walkaround = { turn: 0, rise: 0, zoom: 0 };

    this.state = { steer: 0, throttle: 0, brake: 0, launch: false };
    /**
     * What the game tells the input layer about the car each frame, for the
     * keyboard's speed-sensitive lock: speed and the numbers that turn a
     * lateral g into a steer angle. Zero speed means full lock, so a fresh
     * Input without a game behind it behaves as before.
     */
    this.carSpeed = 0;
    this.carWheelbaseM = 1.53;
    // Overwritten at boot from the tyre model (`main.js`). The fallback is
    // the tyre's actual peak, so a path that somehow skips that boot step
    // behaves like the car rather than like a car from two revisions ago.
    this.carPeakSlipDeg = 7.3;
    /** Ramped keyboard pedals (see controlProfiles.stepPedal) and their clock. */
    this.kbPedal = { throttle: 0, brake: 0 };
    this._kbPedalAt = 0;
    /** Ramped key steering command, -1..1, before the speed lock. */
    this.kbSteer = 0;
    /** Filtered mouse rim position; `mouseSteer` is the raw accumulator. */
    this.mouseSteerFiltered = 0;
    this._mouseAt = 0;
    /** Car yaw rate, deg/s, set by the game for the keyboard damping assist. */
    this.carYawRateDegS = 0;
    /** Set by the game: the run is live (not menu, pause, editor). Gates the mouse. */
    this.driving = false;
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
    /** Every device the rig is reading, base first. Set by the game. */
    this.nativeDeviceNames = [];
    /**
     * WHAT ACTUALLY STEERED THE CAR THIS FRAME.
     *
     * "wheel" | "pad" | "key" | "mouse" | "none", written by whichever branch
     * of `poll` produced the steering command, and classified from the DEVICE
     * rather than from the profile. See `steerDeviceIsWheel`.
     *
     * This exists because the leaderboards are separated by device class and
     * the profile is a dropdown. Separating boards by the profile separates
     * them by what people SAY they drove with, which is worth nothing on a
     * board anybody wants to top: pick "Controller", steer with the wheel
     * anyway, and the controller record is yours. This is what the car was
     * steered by.
     *
     * It is not tamper proof, and nothing that runs on the driver's own
     * machine can be -- the run file is a JSON manifest they own. It removes
     * the cheat that costs nothing, which is the only one that would actually
     * happen on a student team.
     */
    this.steerSource = "none";

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
      // Only while driving: a hand crossing the settings panel is not steering.
      if (!this.driving) return;
      const span = Math.max(m.pixelsForFullLock, 1);
      this.mouseSteer = Math.max(-1, Math.min(1, this.mouseSteer + (-e.movementX * 2) / span));
      this._mouseAt = performance.now();
    });

    addEventListener("gamepadconnected", (e) => {
      if (this.adoptPad(e.gamepad)) this.onPadChange?.(true, this.padName);
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

  /**
   * Switch profile for this session only -- no save, no pin.
   *
   * `setProfile` is the driver choosing at the rig, and it should stick.
   * This is a launcher describing a run, and it should not.
   */
  useProfileForSession(id) {
    this.settings.useForSession(id);
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
    const m = this.profile.mouse;
    // A mouse rim is a hand on a wheel: fast, no slip cap. It takes over the
    // servo whenever the mouse is the thing steering (moved recently and no
    // key held); the keys keep their slower, capped servo.
    if (this.usingMouse && m?.servo) {
      const ms = m.servo;
      return {
        maxRateDegPerS: ms.maxRateDegPerS, accelDegPerS2: ms.accelDegPerS2, lagS: ms.lagS,
        slipCapDeg: ms.slipCapDeg ?? 0, rateSpeedRefMps: ms.rateSpeedRefMps ?? 0, rateSpeedExp: 1.5,
      };
    }
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
    const cur = this.padIndex != null ? pads[this.padIndex] : null;
    // Keep what we have, unless it is an unrecognised device: browsers only
    // populate `getGamepads()` after the first input, so the first thing to
    // show up can easily be the SpaceMouse rather than the wheel, and nothing
    // else would ever look again.
    if (cur && padRank(cur.id) > 1) return cur;
    let best = cur ?? null;
    for (const p of pads) {
      if (!p || !p.connected) continue;
      if (!best || padRank(p.id) > padRank(best.id)) best = p;
    }
    if (best) {
      if (best !== cur) this.adoptPad(best);
      return best;
    }
    return null;
  }

  /**
   * Start reading this device, and guess a profile from it.
   *
   * Refuses to displace something better. A rig has more plugged in than a
   * wheel -- a button box, a handbrake, a SpaceMouse on the CAD machine -- and
   * every one of them enumerates as a gamepad. Taking the newest to announce
   * itself is how a MOZA base ends up behind an "Unknown Gamepad" and the
   * profile switches to a controller nobody is holding, which is exactly what
   * this rig does with three devices attached.
   *
   * @returns true if the device was adopted
   */
  adoptPad(g) {
    if (!g) return false;
    // The rig is reading a wheel natively and `pad()` never looks at the
    // Gamepad API while it is. Without this, a button box or a pad
    // enumerating mid-session switched the live profile off `wheel` -- losing
    // the force feedback, the rim mapping and the wheel's own bindings --
    // while DirectInput carried on being the thing actually steering the car.
    if (this.nativeDevice?.present) return false;
    const pads = navigator.getGamepads?.() ?? [];
    const cur = this.padIndex != null ? pads[this.padIndex] : null;
    if (cur && cur.connected && cur.index !== g.index && padRank(cur.id) >= padRank(g.id)) {
      return false;
    }
    this.padIndex = g.index;
    this.padName = g.id;
    // Pick a profile from the vendor string unless the driver has chosen one
    // by hand. The Gamepad API gives a free-form id and nothing else, so this
    // is pattern matching and will sometimes be wrong -- it only sets the
    // starting point, and the choice is remembered once overridden.
    if (!this.pinned) {
      const guess = detectProfile(g.id);
      if (guess !== this.settings.activeId) {
        this.settings.setActive(guess);
        this.profile = this.settings.active();
        this.onProfileChange?.(guess, g.id);
      }
    }
    return true;
  }

  /**
   * The rig's device state in Gamepad API shape. Buttons are a bitmask; the
   * hat becomes the standard-mapping d-pad indices 12-15 so the setup
   * controls work from a wheel's hat.
   */
  syntheticPad(nd) {
    // Buttons: 32 per device, base first, so a button box's buttons sit at
    // 32 and up. The base's hat becomes four virtual buttons ABOVE every
    // real one (HAT_BASE..HAT_BASE+3: up, down, left, right) -- it used to
    // overwrite indices 12-15, which on a MOZA base are the shift paddles,
    // so a paddle pull stepped the setup menu instead of shifting.
    //
    // The pad object and its 132 button objects are persistent and updated
    // in place: this runs every frame on the rig, and building them fresh
    // was ~140 allocations a frame for a state that changes a bit at a time.
    // The profiles only ever read `.pressed` / `.value` off them.
    const words = Array.isArray(nd.buttons) ? nd.buttons : [nd.buttons | 0];
    const syn = this._syn ??= {
      id: "", index: -1, connected: true, mapping: "", axes: [], buttons: [],
    };
    const buttons = syn.buttons;
    const real = Math.max(words.length * 32, HAT_BASE);
    const total = HAT_BASE + 4;
    // A device came or went: resize once, filling with fresh button records.
    if (buttons.length !== total || real !== syn.real) {
      buttons.length = 0;
      for (let i = 0; i < total; i++) buttons.push({ pressed: false, value: 0 });
      syn.real = real;
    }
    for (let d = 0; d < words.length; d++) {
      const w = words[d];
      for (let i = 0; i < 32; i++) {
        const b = buttons[d * 32 + i];
        const on = (w & (1 << i)) !== 0;
        b.pressed = on; b.value = on ? 1 : 0;
      }
    }
    // Slots above the last real device, below the hat: always off.
    for (let i = words.length * 32; i < HAT_BASE; i++) {
      const b = buttons[i];
      b.pressed = false; b.value = 0;
    }
    const dir = nd.pov >= 0 ? Math.round(nd.pov / 9000) % 4 : -1; // 0 up, 1 right, 2 down, 3 left
    for (let k = 0; k < 4; k++) {
      const on = dir === HAT_DIRS[k];
      const b = buttons[HAT_BASE + k];
      b.pressed = on; b.value = on ? 1 : 0;
    }
    // The rig's axes arrive as a fresh JSON array each frame; hand it over
    // rather than copying it.
    syn.axes = nd.axes;
    syn.id = this.nativeName;
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
    return syn;
  }

  /**
   * First time a given base is seen: rated torque, rotation, axis guesses
   * and a gain that fits the motor, from `wheelPresets.js`. Only once per
   * base, so the driver's own calibration survives every later launch.
   * @returns the preset applied, or null if this base was already set up
   */
  applyWheelPreset(name) {
    const s = this.settings;
    const stamp = `${name}@${PRESET_VERSION}`;
    if (s.read("wheel", "wheel.presetApplied") === stamp) return null;
    const preset = presetFor(name);
    for (const [path, value] of Object.entries(presetPaths(preset))) s.set("wheel", path, value);
    s.set("wheel", "wheel.presetApplied", stamp);
    this.lastPreset = preset;
    this.profile = s.active();
    return preset;
  }

  /**
   * Is the thing currently supplying the steering axis a wheel?
   *
   * Asked of the DEVICE, never of the profile. A driver is free to steer a
   * MOZA base through the controller profile -- it reads an axis and the base
   * has axes -- and that has to come out as a wheel, because it was one.
   *
   * Two pieces of evidence, in order:
   *  - the device the rig opened natively has a force feedback actuator. A
   *    base whose vendor string nobody has seen before still lands here.
   *    NOT merely "the rig opened it": the rig opens whatever the driver
   *    picks in the controls panel, and that picker lists every game
   *    controller -- so an Xbox pad chosen there was "present", and this
   *    called it a wheel and put a pad run on the wheel board, which is the
   *    one thing this exists to prevent. A real base whose effect failed to
   *    start still enumerates with its actuator, so it is still a wheel.
   *  - the vendor string says wheel. `detectProfile` already carries the
   *    brand list, and it is the same list that picks the starting profile.
   *    This is what catches a base the rig read without force feedback --
   *    a wheel in a console mode -- and a wheel on the Gamepad API.
   *
   * Both are things the hardware reports about itself. Neither is something
   * the driver picks from a menu, which is the whole point.
   */
  steerDeviceIsWheel() {
    const nd = this.nativeDevice;
    if (nd?.present && nd.forceFeedback) return true;
    return detectProfile(this.padName || "") === "wheel";
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
   * wheel angle. For SDM26 that is 46 degrees of lock, which the MEASURED
   * rack (the toe-vs-rim table in `params.js`) puts at 179 degrees of rim
   * either side -- 358 lock to lock. The nominal 4.411 ratio would say 203
   * either side, and the difference is the rack's non-linearity, which is why
   * the table is what the car steers through -- set the wheel's
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

    // Rim travel to one side is the rack's MEASURED stop: 179 deg for SDM26,
    // so 358 lock to lock. Not lock x a nominal constant ratio -- the real
    // rack is progressive and the constant puts the stop 24 deg out.
    const carRimHalf = this.carRimHalfDeg ?? 179;
    let norm;
    if (w.mapping === "match-car") {
      // Through the car's measured rim -> road table, the same one the desktop
      // rig steers by natively. Anything past the stop is over-lock.
      const maxRoad = Math.max(this.carLockDeg ?? 46, 1e-6);
      const road = roadFromRimDeg(this.carSteering, rimDeg);
      norm = road / maxRoad;
      // The end stop goes where the SOFT LOCK bites, which is where the road
      // wheel reaches the car's live lock -- not at the rack's stop. They are
      // the same place only at the default lock; see `rimFromRoadDeg`. Same
      // reasoning and same formula as `rig.rs` on the desktop path.
      this.rim.halfLockDeg = Math.min(
        Math.abs(rimFromRoadDeg(this.carSteering, maxRoad)),
        carRimHalf,
      );
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
    for (const k in this.edges) this.edges[k] = false;
    for (const k in this.menu) this.menu[k] = false;

    const p = this.pad();
    const prof = this.profile;
    let steer = 0, throttle = 0, brake = 0, launch = false;
    // Written by whichever branch below ends up owning `steer`. See
    // `this.steerSource`, which it is published to at the bottom.
    let source = "none";

    // How much of the car's lock is usable at this speed; see
    // `controlProfiles.usableLockFrac`. It applies to every control that has
    // no physical stop of its own -- keyboard, mouse AND stick.
    //
    // It used to be computed below the pad branch and therefore reached only
    // the keyboard and the mouse, while both gamepad profiles declared
    // `speedSensitive` and got nothing from it. That went unnoticed while full
    // lock was 28 degrees; it went to 46 in the same change that added the
    // declaration, so a full stick deflection at speed now asks for far more
    // steering than the front axle can use.
    const lockFrac = usableLockFrac(prof.steering?.speedSensitive, this.carSpeed, {
      wheelbaseM: this.carWheelbaseM,
      maxSteerDeg: this.carLockDeg ?? 46,
      peakSlipAngleDeg: this.carPeakSlipDeg,
    });

    if (p) {
      // Copied into the persistent array rather than `Array.from` each
      // frame; the calibration UI reads it live.
      const ra = this.rawAxes;
      ra.length = p.axes.length;
      for (let i = 0; i < ra.length; i++) ra[i] = p.axes[i];
      const axes = prof.axes || { steer: 0 };
      const raw = p.axes[axes.steer] ?? 0;

      if (prof.kind === "wheel") {
        // A wheel has a real stop at a real angle and its own soft lock, so it
        // is not capped here: the driver can see and feel where the lock is.
        steer = this.wheelSteer(raw, prof);
      } else {
        // Negative because pushing the stick left must steer left, and the
        // vehicle model takes left as positive.
        steer = -applySteeringCurve(prof.steering, raw) * lockFrac;
      }
      // An axis is steering. WHICH axis is a question about the hardware, and
      // deliberately not about `prof.kind` -- that is the dropdown.
      if (Math.abs(steer) > SOURCE_THRESHOLD) {
        source = this.steerDeviceIsWheel() ? "wheel" : "pad";
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
      // A negative index is "not bound to anything on this device" -- see
      // `UNBOUND`. It has to be rejected here rather than relied on to index
      // nothing, because `buttons[-1]` is undefined only by luck.
      const bound = (i) => typeof i === "number" && i >= 0;
      const pressed = (i) => bound(i) && !!p.buttons[i]?.pressed;
      const wasPressed = (i) => bound(i) && !!this._prevButtons[i];
      const edge = (i) => pressed(i) && !wasPressed(i);

      launch = pressed(B.launch);
      // Every edge action the device has a button for. `=` would be wrong:
      // the keyboard is read after this and must be able to add to it.
      for (const a of EDGE_ACTIONS) {
        if (edge(B[buttonSlot(a)])) this.edges[a.id] = true;
      }
      const M = this.menu;
      M.up = edge(B.dpadUp);
      M.down = edge(B.dpadDown);
      M.left = edge(B.dpadLeft);
      M.right = edge(B.dpadRight);
      M.accept = edge(B.launch);
      M.back = edge(B.reset);
      M.prevTab = edge(B.downshift);
      M.nextTab = edge(B.upshift);
      M.start = edge(B.pause);

      // D-pad: left/right pick the setting (above, as setupPrev/setupNext),
      // up/down move it and repeat while held.
      this.applyRepeat("setupUp", pressed(B.dpadUp), edge(B.dpadUp));
      this.applyRepeat("setupDown", pressed(B.dpadDown), edge(B.dpadDown));

      // Last frame's buttons for the edge detection, kept in one array
      // rather than `map`ped fresh; and whether anything at all is pressed.
      const pb = this._prevButtons;
      const nb = p.buttons.length;
      pb.length = nb;
      let anyPressed = false;
      for (let i = 0; i < nb; i++) {
        const on = !!p.buttons[i].pressed;
        pb[i] = on;
        if (on) anyPressed = true;
      }
      if (throttle > 0.02 || brake > 0.02 || Math.abs(raw) > ACTIVITY_THRESHOLD || anyPressed) {
        this.usingPad = true;
      }
    }

    // Keyboard: always live, so the game is playable (and testable) without a
    // pad. Pad input wins whenever it is non-zero.
    const k = this.keys;
    // The profile's key bindings. Every profile carries a set, because the
    // keyboard stays live whatever is plugged in: a driver on a wheel still
    // pauses and still goes home from the keyboard.
    const K = prof.keys || {};
    const held = (codes) => Array.isArray(codes) && codes.some((c) => k.has(c));
    const kEdge = (code) => k.has(code) && !this._prevKeys.has(code);
    const anyEdge = (codes) => Array.isArray(codes) && codes.some(kEdge);

    const kSteer = (held(K.steerLeft) ? 1 : 0) - (held(K.steerRight) ? 1 : 0);
    const kThrottle = held(K.throttle) ? 1 : 0;
    const kBrake = held(K.brake) ? 1 : 0;

    // Wall-clock step for everything ramped below: poll() runs once per
    // rendered frame, and a ramp must not depend on the frame rate.
    const now = performance.now();
    const dt = this._kbPedalAt ? Math.min(0.1, (now - this._kbPedalAt) / 1000) : 0;
    this._kbPedalAt = now;

    // ---- keys: a ramped command with a yaw-rate reflex ----
    // The key is a step; the command it produces is not. It ramps up over
    // keyRampUpS and back over keyRampDownS (release is the safe direction,
    // so it is quicker), and the car's yaw rate feeds back as a small
    // counter-steer, the reflex a driver has and a key does not. Both are
    // profile settings; zero ramps and zero damping give the old step.
    const stc = prof.steering ?? {};
    const lock = Math.max(1, this.carLockDeg ?? 46);
    {
      const up = Math.max(stc.keyRampUpS ?? 0, 1e-3);
      const down = Math.max(stc.keyRampDownS ?? 0, 1e-3);
      const target = kSteer;
      const towardCentre = target === 0 || Math.sign(target) !== Math.sign(this.kbSteer);
      const rate = (towardCentre ? 1 / down : 1 / up) * dt;
      this.kbSteer += Math.max(-rate, Math.min(rate, target - this.kbSteer));
      if (Math.abs(this.kbSteer) < 1e-4) this.kbSteer = 0;
    }
    const keySteering = kSteer !== 0 || this.kbSteer !== 0;
    if (keySteering && steer === 0) {
      let cmd = this.kbSteer * lockFrac;
      // Yaw damping: yaw rate is positive turning left, steer is positive
      // left, so the term opposes the rotation. Only while the car moves.
      // Off below 4 m/s, full by 12: at walking pace the reflex only fought
      // the driver's own full lock.
      const damp = (stc.yawDampPerDegS ?? 0) * Math.min(1, Math.max(0, ((this.carSpeed ?? 0) - 4) / 8));
      cmd -= (this.carYawRateDegS ?? 0) * damp;
      steer = clamp(cmd, -1, 1);
      // A key is down, or the ramp it left behind is still unwinding. Either
      // way the keyboard is what is steering: the ramp is this branch's own
      // output, not a second device.
      if (Math.abs(steer) > SOURCE_THRESHOLD || kSteer !== 0) source = "key";
    }

    // ---- mouse: a virtual rim ----
    // Horizontal movement turns a rim that has a position and a lock, like
    // the real one. The position is low-pass filtered (a hand on a mouse
    // jitters), and when the mouse is still the rim self-centres at a rate
    // that scales with speed, the way aligning torque does: standing still
    // it stays where you put it, at speed it comes back on its own. Keys
    // win while one is held.
    const mouse = prof.mouse;
    // The mouse owns the servo whenever it is enabled and no key is held.
    // Switching back to the capped key servo after a quiet second made the
    // wheel jump from full lock to the slip-cap band the moment the hand
    // stopped moving in a tight low-speed turn.
    this.usingMouse = !!(mouse && mouse.enabled && !keySteering);
    if (mouse && mouse.enabled && steer === 0 && !keySteering) {
      const idle = now - this._mouseAt > 60;
      if (mouse.selfCentre && idle && dt > 0) {
        const speedScale = Math.min(1, (this.carSpeed ?? 0) / 8);
        const rate = ((mouse.selfCentreRateDegPerS ?? 0) / lock) * speedScale * dt;
        this.mouseSteer -= Math.sign(this.mouseSteer) * Math.min(Math.abs(this.mouseSteer), rate);
      }
      const tau = Math.max(mouse.smoothingS ?? 0, 0);
      const k = tau > 0 && dt > 0 ? 1 - Math.exp(-dt / tau) : 1;
      this.mouseSteerFiltered += (this.mouseSteer - this.mouseSteerFiltered) * k;
      // Expo: gentle near centre, still full lock at full travel. At low
      // speed the usable lock is the whole rack and a linear map was twitchy.
      const ex = mouse.expo ?? 1;
      const f = this.mouseSteerFiltered;
      steer = Math.sign(f) * Math.pow(Math.abs(f), ex) * lockFrac;
      if (Math.abs(steer) > SOURCE_THRESHOLD) source = "mouse";
    } else if (keySteering) {
      // A key took over: fold the rim back to where the car is pointed so
      // letting go does not snap to a stale mouse position.
      this.mouseSteer = this.kbSteer;
      this.mouseSteerFiltered = this.kbSteer;
    }

    // Keyboard pedals are ramped rather than stepped, at the rates the
    // profile sets (no rates: a plain step, as before).
    {
      const thrCfg = prof.pedals?.throttle ?? {};
      const brkCfg = prof.pedals?.brake ?? {};
      this.kbPedal.throttle = stepPedal(this.kbPedal.throttle, kThrottle, thrCfg, dt);
      this.kbPedal.brake = stepPedal(this.kbPedal.brake, kBrake, brkCfg, dt);
    }
    if (this.kbPedal.throttle > 0 && throttle === 0) throttle = this.kbPedal.throttle;
    if (this.kbPedal.brake > 0 && brake === 0) brake = this.kbPedal.brake;
    if (held(K.launch)) launch = true;

    for (const a of EDGE_ACTIONS) {
      if (anyEdge(K[a.id])) this.edges[a.id] = true;
    }
    if (!p) {
      this.applyRepeat("setupUp", held(K.setupUp), anyEdge(K.setupUp));
      this.applyRepeat("setupDown", held(K.setupDown), anyEdge(K.setupDown));
    }

    // Walkaround camera nudges, live rather than edge-triggered so holding a
    // key sweeps smoothly.
    //
    // NOT rebindable, because in that camera the mouse already does all three
    // -- drag turns and raises, the wheel zooms -- and these are the
    // keyboard's copy of that, not a second set of controls.
    //
    // But they ARE live at the same time as the driving controls, and the
    // claim that they are not is what made the old choice of keys wrong: `R`
    // was also "put me back on course", so raising the orbit camera teleported
    // the car onto the centreline, and `[` / `]` also stepped the live setup
    // menu while you zoomed. These keys are now in `RESERVED_KEYS` and the
    // test suite checks no shipped binding lands on one.
    const wa = this.walkaround;
    wa.turn = (k.has("Comma") ? 1 : 0) - (k.has("Period") ? 1 : 0);
    wa.rise = (k.has("KeyG") ? 1 : 0) - (k.has("KeyF") ? 1 : 0);
    wa.zoom = (k.has("Quote") ? 1 : 0) - (k.has("Semicolon") ? 1 : 0);
    // The same Set, refilled, rather than a new one a frame.
    const pk = this._prevKeys;
    pk.clear();
    for (const c of k) pk.add(c);

    this.steerSource = source;
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

/**
 * How likely a connected device is to be the one being driven.
 *
 * A wheel beats a named controller beats anything that merely enumerated.
 * Only used to choose between several at once; with one device plugged in it
 * changes nothing.
 */
function padRank(id) {
  if (detectProfile(id) === "wheel") return 3;
  if (/xbox|xinput|microsoft|045e|dualshock|dualsense|playstation|sony|054c|wireless controller/i
    .test(id || "")) {
    return 2;
  }
  return 1;
}

// NaN fails both comparisons and would pass straight through into the car.
function clamp(x, lo, hi) { return Number.isFinite(x) ? (x < lo ? lo : x > hi ? hi : x) : 0; }
