// Control devices, and what each one does to the steering.
//
// Three families, and they are not variations on a theme -- they differ in what
// the driver is physically able to command, which is why each gets its own
// steering dynamics rather than a shared curve with different numbers:
//
//   keyboard   A digital switch. There is no steering *position* to read, only
//              "left" or "not left", so the angle has to be ramped in software.
//              Every number here is the software standing in for a hand.
//
//   gamepad    A proportional stick, but a short one: full deflection is about
//              20 mm of thumb travel for the car's whole steering range. The
//              limit is resolution, so it gets an expo curve to put precision
//              where the grip is.
//
//   wheel      The driver's hands *are* the input. The rim angle is measured
//              directly, so software smoothing is a defect rather than a
//              feature: deadzone 0, linearity 1, and rate limits set high
//              enough that the human is the slowest element in the loop.
//
// Steering is modelled as a rate- and acceleration-limited servo following the
// commanded angle. Both limits are per-profile and adjustable, because they
// mean genuinely different things per device: on a pad they describe how fast
// the *software* is willing to move the wheel; on a real wheel they describe
// how fast arms and a steering rack actually can.
//
// Force feedback is not implemented. The fields are reserved and documented so
// the steer-force calculation can be dropped in without reshaping the profile
// format, and `forceFeedback.enabled` is false everywhere until it exists.

const STORAGE_KEY = "fsae-sim.controls.v1";

/**
 * Steering servo limits.
 *
 * All angles are at the ROAD WHEEL, matching `BicycleModel.delta` and
 * `SDM26.steerRateDegS`. Divide by `steeringRatio` for hand-wheel figures --
 * with SDM26's 4.0, 360 deg/s at the road wheel is 1440 deg/s at the rim,
 * which is about as fast as anyone moves a wheel in a save.
 */
function steering(over) {
  return {
    /** Fastest the road wheel can be moved, deg/s. */
    maxRateDegPerS: 360,
    /** How fast that rate can change, deg/s^2. Bounds the snap of a step input. */
    accelDegPerS2: 4000,
    /** First-order lag on the position command, s. Hands plus rack compliance. */
    lagS: 0.06,
    /** Fraction of travel ignored around centre. */
    deadzone: 0.1,
    /** Response curve exponent. 1.0 is linear; above 1 softens around centre. */
    expo: 1.7,
    ...over,
  };
}

/** One pedal axis. Every real pedal set needs all of these. */
function pedal(over) {
  return {
    /** Which gamepad axis or button index carries it. */
    source: null,
    /** Is `source` an axis (true) or an analog button (false)? */
    isAxis: true,
    /** Raw reading at rest and at full travel. Real pedals rarely span -1..1. */
    rawMin: -1,
    rawMax: 1,
    /** Ignored at the top and bottom of travel, after calibration. */
    deadzoneLow: 0.02,
    deadzoneHigh: 0.02,
    /** Output curve. 1.0 linear. */
    gamma: 1,
    invert: false,
    ...over,
  };
}

/**
 * Apply a pedal calibration to a raw reading, returning 0..1.
 *
 * Calibration is not optional decoration for pedals. A Logitech G29 brake rests
 * around -1 and tops out near +1, a Fanatec load cell rests at -1 and may never
 * reach +1 at any force a person can apply, and a gamepad trigger reports 0..1
 * as a button. Without per-axis min/max the same code reads full brake at rest
 * on one device and half brake at the stop on another.
 */
export function applyPedal(cal, raw) {
  const span = cal.rawMax - cal.rawMin;
  let v = Math.abs(span) < 1e-6 ? 0 : (raw - cal.rawMin) / span;
  if (cal.invert) v = 1 - v;
  v = Math.min(Math.max(v, 0), 1);

  // Trim the ends, then rescale so the usable travel still spans 0..1.
  const lo = cal.deadzoneLow;
  const hi = 1 - cal.deadzoneHigh;
  v = hi - lo < 1e-6 ? 0 : (v - lo) / (hi - lo);
  v = Math.min(Math.max(v, 0), 1);

  return cal.gamma === 1 ? v : Math.pow(v, cal.gamma);
}

/** Steering input curve: deadzone, then expo, sign preserved. */
export function applySteeringCurve(cfg, raw) {
  const mag = Math.abs(raw);
  if (mag <= cfg.deadzone) return 0;
  const scaled = (mag - cfg.deadzone) / (1 - cfg.deadzone);
  return Math.sign(raw) * Math.pow(scaled, cfg.expo);
}

/**
 * Advance a rate- and acceleration-limited steering servo.
 *
 * @param state  {angleDeg, rateDegPerS} mutated in place
 * @param targetDeg  commanded road-wheel angle
 * @param cfg  a profile's `steering` block
 * @param dt   seconds
 *
 * The acceleration limit is the part that did not exist before. Without it a
 * step input produces a step in steering *velocity*, which no hand and no
 * steering motor can do -- and on a keyboard, where every input is a step, it
 * is the difference between the car darting and the car being steered.
 */
export function stepSteering(state, targetDeg, cfg, dt) {
  const error = targetDeg - state.angleDeg;

  // Proportional command, which is the first-order lag written as a rate.
  let wantRate = error / Math.max(cfg.lagS, 1e-4);
  wantRate = Math.min(Math.max(wantRate, -cfg.maxRateDegPerS), cfg.maxRateDegPerS);

  const maxDelta = cfg.accelDegPerS2 * dt;
  const dRate = Math.min(Math.max(wantRate - state.rateDegPerS, -maxDelta), maxDelta);
  state.rateDegPerS += dRate;

  const next = state.angleDeg + state.rateDegPerS * dt;

  // Do not let the servo drive past the target and oscillate. Overshoot here is
  // an artefact of discretising the controller, not modelled inertia.
  if ((error > 0 && next > targetDeg) || (error < 0 && next < targetDeg)) {
    state.angleDeg = targetDeg;
    state.rateDegPerS = 0;
  } else {
    state.angleDeg = next;
  }
  return state.angleDeg;
}

// ---------------------------------------------------------------------------
// Profiles
// ---------------------------------------------------------------------------

export const PROFILES = {
  keyboard: {
    id: "keyboard",
    label: "Keyboard & mouse",
    kind: "keyboard",
    // Slower and smoother than any analog device, on purpose. A key is a step
    // input; the software ramp is the only thing standing between the driver
    // and full lock in one frame.
    steering: steering({
      maxRateDegPerS: 180,
      accelDegPerS2: 700,
      lagS: 0.10,
      deadzone: 0,
      expo: 1,
    }),
    // Mouse steering: horizontal movement maps to steering angle. Off by
    // default because it is a different skill, not a better one.
    mouse: {
      enabled: false,
      /** Screen pixels for full lock. */
      pixelsForFullLock: 420,
      /** Recentres when not moving, like a self-centring wheel. */
      selfCentre: true,
      selfCentreRateDegPerS: 90,
    },
    pedals: {
      throttle: pedal({ source: "KeyW", isAxis: false }),
      brake: pedal({ source: "KeyS", isAxis: false }),
      clutch: pedal({ source: null }),
    },
    forceFeedback: { enabled: false, supported: false },
  },

  "gamepad-xbox": {
    id: "gamepad-xbox",
    label: "Controller — Xbox",
    kind: "gamepad",
    // Standard Gamepad API mapping, which is what an Xbox One/Series pad
    // reports on Windows over USB and Bluetooth alike.
    steering: steering({
      maxRateDegPerS: 300,
      accelDegPerS2: 2200,
      lagS: 0.06,
      deadzone: 0.10,
      // Full lock is 28 degrees and the tyre peaks at 8.5 degrees of slip, so a
      // linear stick puts the useful travel in the first third. Squaring it
      // moves the resolution to where the grip is.
      expo: 1.7,
    }),
    axes: { steer: 0 },
    pedals: {
      throttle: pedal({ source: 7, isAxis: false, rawMin: 0, rawMax: 1 }),
      brake: pedal({ source: 6, isAxis: false, rawMin: 0, rawMax: 1 }),
      clutch: pedal({ source: null }),
    },
    buttons: {
      upshift: 5, downshift: 4, launch: 0, reset: 1, traction: 2, camera: 3,
      restart: 8, pause: 9, home: 10,
      dpadUp: 12, dpadDown: 13, dpadLeft: 14, dpadRight: 15,
    },
    labels: { launch: "A", reset: "B", traction: "X", camera: "Y", upshift: "RB", downshift: "LB" },
    forceFeedback: { enabled: false, supported: false },
  },

  "gamepad-ps": {
    id: "gamepad-ps",
    label: "Controller — PlayStation",
    kind: "gamepad",
    // DualShock 4 and DualSense report the same standard mapping; what changes
    // is what the buttons are called, and the triggers are physically longer,
    // which is worth a slightly smaller deadzone.
    steering: steering({
      maxRateDegPerS: 300,
      accelDegPerS2: 2200,
      lagS: 0.06,
      deadzone: 0.08,
      expo: 1.7,
    }),
    axes: { steer: 0 },
    pedals: {
      throttle: pedal({ source: 7, isAxis: false, rawMin: 0, rawMax: 1 }),
      brake: pedal({ source: 6, isAxis: false, rawMin: 0, rawMax: 1 }),
      clutch: pedal({ source: null }),
    },
    buttons: {
      upshift: 5, downshift: 4, launch: 0, reset: 1, traction: 2, camera: 3,
      restart: 8, pause: 9, home: 10,
      dpadUp: 12, dpadDown: 13, dpadLeft: 14, dpadRight: 15,
    },
    labels: {
      launch: "Cross", reset: "Circle", traction: "Square", camera: "Triangle",
      upshift: "R1", downshift: "L1",
    },
    forceFeedback: { enabled: false, supported: false },
  },

  wheel: {
    id: "wheel",
    label: "Wheel & pedals",
    kind: "wheel",
    // The driver's hands are the input, so the software should get out of the
    // way. Deadzone zero and linearity one are not tuning choices -- either one
    // set otherwise is a defect on a device that measures rim angle directly.
    steering: steering({
      maxRateDegPerS: 720,
      accelDegPerS2: 12000,
      lagS: 0.015,
      deadzone: 0,
      expo: 1,
    }),
    wheel: {
      /**
       * Physical rotation the wheel is configured for, lock to lock, degrees.
       * Set this to match the driver software (Logitech G HUB, Fanatec, MOZA
       * Pit House). If it disagrees, everything else is calibrated wrong.
       */
      rotationDeg: 900,

      /**
       * How the rim angle becomes a road-wheel angle. This is the single most
       * important setting for a wheel and the one most often got wrong.
       *
       * "match-car" is the honest option: the rim turns through the car's real
       * ratio, so SDM26's 28 degrees of lock through a 4.0 ratio is 112 degrees
       * rim, lock to lock. Set `rotationDeg` to 112 in the driver and the wheel
       * and the car agree exactly -- what you feel is what the front tyres are
       * doing.
       *
       * "scale-to-lock" maps whatever rotation the wheel is set to onto full
       * lock. More forgiving on a 900-degree wheel nobody wants to reconfigure,
       * but the ratio is then a fiction: a 900-degree rim on a 112-degree car
       * is eight times slower than the real steering.
       */
      mapping: "match-car",

      /**
       * Resist beyond the car's lock rather than continuing to read angle.
       * Without force feedback this is only a clamp; with it, it is a stop.
       */
      softLock: true,

      /** Centre offset, degrees, if the wheel does not zero perfectly. */
      centreTrimDeg: 0,
    },
    axes: {
      // Sensible defaults for a Logitech G-series on Windows. Wheels do NOT use
      // the standard gamepad mapping and every vendor differs, so these are a
      // starting point for the calibration flow, not a claim about your device.
      steer: 0,
      throttle: 1,
      brake: 2,
      clutch: 3,
    },
    pedals: {
      // Wheel pedals rest at one end of the axis and are usually inverted.
      throttle: pedal({ source: 1, isAxis: true, rawMin: 1, rawMax: -1 }),
      brake: pedal({
        source: 2,
        isAxis: true,
        rawMin: 1,
        rawMax: -1,
        // A load cell measures force, not travel, and force rises much faster
        // than displacement near the stop. Above 1 makes early travel less
        // sensitive, which is what makes threshold braking possible.
        gamma: 1.6,
      }),
      clutch: pedal({ source: 3, isAxis: true, rawMin: 1, rawMax: -1 }),
    },
    buttons: {
      upshift: 4, downshift: 5, launch: 0, reset: 1, traction: 2, camera: 3,
      restart: 8, pause: 9, home: 10,
      dpadUp: 12, dpadDown: 13, dpadLeft: 14, dpadRight: 15,
    },
    forceFeedback: {
      /**
       * Reserved. Nothing here is implemented yet.
       *
       * The plan is to drive it from the steer torque the vehicle model can
       * already almost produce: front tyre lateral force through the pneumatic
       * trail and the mechanical trail from caster gives self-aligning torque,
       * which is the signal a driver actually reads through a wheel. That needs
       * the tyre model to return Mz (it currently returns Fy only) and a
       * kingpin geometry block in the parameters, neither of which exists.
       *
       * The web platform has no force-feedback API beyond dual-rumble, so the
       * desktop build will need a native path. Left declared so the shape of
       * the profile does not have to change when it lands.
       */
      enabled: false,
      supported: false,
      gain: 1.0,
      /** Self-aligning torque from the front tyres. The main effect. */
      alignTorqueGain: 1.0,
      /** Kerb and surface texture. */
      roadTextureGain: 0.35,
      /** Damping, to stop the wheel oscillating when unloaded. */
      damping: 0.15,
      /** Friction, standing in for the rack. */
      friction: 0.05,
      /** Force at the soft lock. */
      softLockGain: 1.0,
      minForce: 0.0,
      maxForceNm: 8.0,
    },
  },
};

/**
 * Guess which profile a gamepad id belongs to.
 *
 * The Gamepad API gives a free-form vendor string and nothing else useful, so
 * this is pattern matching and will not be right for every device. It only
 * picks the starting profile -- the driver can always override, and the choice
 * is remembered.
 */
export function detectProfile(padId) {
  const id = (padId || "").toLowerCase();
  if (!id) return "keyboard";

  // Wheels first: several report vendor names that also contain "gamepad".
  if (
    /logitech|g29|g920|g923|g27|g25|driving force|thrustmaster|t300|t150|tmx|t248|t500|fanatec|clubsport|csl|podium|moza|simucube|simagic|cammus|wheel|racing/.test(
      id,
    )
  ) {
    return "wheel";
  }
  if (/dualshock|dualsense|playstation|sony|054c|wireless controller/.test(id)) {
    return "gamepad-ps";
  }
  if (/xbox|xinput|microsoft|045e/.test(id)) return "gamepad-xbox";
  // Anything else that enumerated as a gamepad: standard mapping is the safest
  // assumption, and that is the Xbox layout.
  return "gamepad-xbox";
}

/** Deep clone, so editing a profile never mutates the shipped defaults. */
function clone(v) {
  return typeof structuredClone === "function"
    ? structuredClone(v)
    : JSON.parse(JSON.stringify(v));
}

/**
 * Recursively overlay `patch` onto `base`, in place, ignoring keys the base
 * does not have.
 *
 * Ignoring unknown keys is what makes a saved override survive this file
 * changing: a setting that has been renamed or removed is dropped rather than
 * reappearing as a stray property nothing reads.
 */
function overlay(base, patch) {
  if (!patch || typeof patch !== "object") return base;
  for (const k of Object.keys(patch)) {
    if (!(k in base)) continue;
    const b = base[k];
    const p = patch[k];
    if (b && typeof b === "object" && !Array.isArray(b) && p && typeof p === "object") {
      overlay(b, p);
    } else if (typeof b === typeof p || b === null) {
      base[k] = p;
    }
  }
  return base;
}

/**
 * Holds the active profile and the driver's edits.
 *
 * Overrides are stored as a diff against the shipped defaults, not as a full
 * copy. That way improving a default later actually reaches anyone who never
 * touched it, instead of being silently pinned to whatever shipped the day
 * they first ran the game.
 */
export class ControlSettings {
  constructor() {
    this.activeId = "keyboard";
    /** @type {Record<string, object>} profile id -> sparse override tree */
    this.overrides = {};
    this.load();
  }

  ids() {
    return Object.keys(PROFILES);
  }

  /** The active profile with the driver's edits applied. */
  active() {
    return this.get(this.activeId);
  }

  get(id) {
    const base = clone(PROFILES[id] || PROFILES.keyboard);
    return overlay(base, this.overrides[id]);
  }

  setActive(id) {
    if (PROFILES[id]) {
      this.activeId = id;
      this.save();
    }
  }

  /**
   * Set one value by dotted path, e.g. `steering.maxRateDegPerS`.
   * Stored as an override, so a reset restores the shipped default.
   */
  set(id, path, value) {
    if (!PROFILES[id]) return;
    const parts = path.split(".");
    let node = (this.overrides[id] ||= {});
    for (let i = 0; i < parts.length - 1; i++) node = node[parts[i]] ||= {};
    node[parts[parts.length - 1]] = value;
    this.save();
  }

  read(id, path) {
    let node = this.get(id);
    for (const part of path.split(".")) {
      if (node == null) return undefined;
      node = node[part];
    }
    return node;
  }

  /** Is this value currently different from the shipped default? */
  isOverridden(id, path) {
    let node = this.overrides[id];
    for (const part of path.split(".")) {
      if (node == null || !(part in node)) return false;
      node = node[part];
    }
    return true;
  }

  resetPath(id, path) {
    const parts = path.split(".");
    let node = this.overrides[id];
    const chain = [];
    for (let i = 0; i < parts.length - 1; i++) {
      if (!node) return;
      chain.push([node, parts[i]]);
      node = node[parts[i]];
    }
    if (!node) return;
    delete node[parts[parts.length - 1]];
    // Prune empty branches so `isOverridden` stays honest.
    for (let i = chain.length - 1; i >= 0; i--) {
      const [parent, key] = chain[i];
      if (parent[key] && Object.keys(parent[key]).length === 0) delete parent[key];
    }
    this.save();
  }

  resetProfile(id) {
    delete this.overrides[id];
    this.save();
  }

  /** Store a completed pedal calibration. */
  calibratePedal(id, which, rawMin, rawMax) {
    this.set(id, `pedals.${which}.rawMin`, rawMin);
    this.set(id, `pedals.${which}.rawMax`, rawMax);
  }

  save() {
    try {
      localStorage.setItem(
        STORAGE_KEY,
        JSON.stringify({ activeId: this.activeId, overrides: this.overrides }),
      );
    } catch {
      // Private browsing, or storage disabled. Settings just do not persist.
    }
  }

  load() {
    try {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (!raw) return;
      const data = JSON.parse(raw);
      if (data && PROFILES[data.activeId]) this.activeId = data.activeId;
      if (data && data.overrides && typeof data.overrides === "object") {
        this.overrides = data.overrides;
      }
    } catch {
      // Corrupt or from an incompatible version: fall back to defaults rather
      // than refusing to start.
    }
  }
}

/**
 * Which settings are editable, and over what range.
 *
 * Same shape as `paramMeta.js` uses for the vehicle, and for the same reason:
 * the UI should be generated from one description rather than hand-built and
 * then drifting from what the code reads.
 */
export function editableSettings(profile) {
  const rows = [
    {
      group: "Steering",
      items: [
        {
          path: "steering.maxRateDegPerS",
          label: "Max steering speed",
          unit: " deg/s",
          min: 60, max: 1200, step: 10,
          note: "At the road wheel. Divide by the steering ratio for the rim.",
        },
        {
          path: "steering.accelDegPerS2",
          label: "Steering acceleration",
          unit: " deg/s²",
          min: 200, max: 20000, step: 100,
          note: "How fast the steering speed itself can change.",
        },
        {
          path: "steering.lagS",
          label: "Steering lag",
          unit: " s",
          min: 0, max: 0.25, step: 0.005,
          note: "Hands and rack compliance.",
        },
        {
          path: "steering.deadzone",
          label: "Deadzone",
          unit: "",
          min: 0, max: 0.4, step: 0.01,
          note: profile.kind === "wheel" ? "Should be zero on a wheel." : "",
        },
        {
          path: "steering.expo",
          label: "Response curve",
          unit: "",
          min: 1, max: 3, step: 0.05,
          note: profile.kind === "wheel" ? "Should be 1.00 on a wheel." : "1.0 is linear.",
        },
      ],
    },
  ];

  if (profile.kind === "wheel") {
    rows.push({
      group: "Wheel",
      items: [
        {
          path: "wheel.rotationDeg",
          label: "Wheel rotation",
          unit: " deg",
          min: 90, max: 1440, step: 10,
          note: "Must match the setting in your wheel's driver software.",
        },
        {
          path: "wheel.centreTrimDeg",
          label: "Centre trim",
          unit: " deg",
          min: -20, max: 20, step: 0.5,
        },
      ],
    });
  }

  const pedalRows = [];
  for (const which of ["throttle", "brake", "clutch"]) {
    const cal = profile.pedals && profile.pedals[which];
    // Anything with real travel: an axis on a pedal set, or a gamepad trigger,
    // which is an analog button rather than an axis but still has a curve worth
    // shaping. Excluded is a keyboard key, whose `source` is a key code -- it
    // has no travel, so a gamma or a deadzone on it cannot do anything.
    if (!cal || cal.source === null || typeof cal.source === "string") continue;
    pedalRows.push(
      {
        path: `pedals.${which}.gamma`,
        label: `${which[0].toUpperCase()}${which.slice(1)} curve`,
        unit: "",
        min: 0.4, max: 3, step: 0.05,
        note: which === "brake" ? "Above 1 suits a load cell." : "",
      },
      {
        path: `pedals.${which}.deadzoneLow`,
        label: `${which[0].toUpperCase()}${which.slice(1)} deadzone (rest)`,
        unit: "",
        min: 0, max: 0.3, step: 0.005,
      },
      {
        path: `pedals.${which}.deadzoneHigh`,
        label: `${which[0].toUpperCase()}${which.slice(1)} deadzone (stop)`,
        unit: "",
        min: 0, max: 0.3, step: 0.005,
      },
    );
  }
  if (pedalRows.length) rows.push({ group: "Pedals", items: pedalRows });

  // Only when it is switched on -- the toggle itself lives in the panel, and a
  // sensitivity group above a disabled feature reads as a bug.
  if (profile.kind === "keyboard" && profile.mouse && profile.mouse.enabled) {
    rows.push({
      group: "Mouse sensitivity",
      items: [
        {
          path: "mouse.pixelsForFullLock",
          label: "Pixels for full lock",
          unit: " px",
          min: 120, max: 1600, step: 20,
        },
        {
          path: "mouse.selfCentreRateDegPerS",
          label: "Self-centre rate",
          unit: " deg/s",
          min: 0, max: 400, step: 5,
        },
      ],
    });
  }

  return rows;
}
