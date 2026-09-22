// Live setup adjustments the driver can make from the wheel.
//
// These write straight into the vehicle parameters the physics is already
// holding a reference to, so a change takes effect on the next substep -- no
// restart, no reload. That is the point: you feel the balance move while the
// car is still moving.
//
// Steps are finer than anything you could actually set on the car (roll
// stiffness comes in bar holes, brake bias in turns of a bar, final drive in
// sprocket teeth). That is deliberate for a simulator: it lets you find where
// the balance actually changes before deciding what the nearest real setting is.

/**
 * Everything the driver can move from inside the car, in the order the d-pad
 * menu walks them. Each one is a real between-runs change on SDM26 -- a bar
 * blade, a bias-bar turn, a diff shim, an ECU number, a sprocket
 * -- so a test day in the sim can be run the way the team runs one.
 *
 *   path     dotted path into the params object (see paramMeta.readParam)
 *   factor   stored -> shown: fractions are shown as percentages
 *   min/max  SHOWN units; `set` clamps into them
 *   step     one tap, in shown units; a held button repeats and then goes 5x
 *   decimals how the HUD prints it; also sets what counts as "at baseline"
 *
 * Every item also gets its own bindable up/down pair (controlBindings.js
 * builds them from this list), so the two a driver reaches for most can sit
 * on the rim without going through the menu.
 */
export const ADJUSTMENTS = [
  {
    id: "rsd",
    label: "Roll stiffness, front",
    bindLabel: "Roll stiffness (RSD)",
    short: "RSD-F",
    unit: "%",
    // Front share of total roll stiffness. More front = more lateral load
    // transfer at the front = less front grip = more understeer.
    path: "roll.rsdFront", factor: 100,
    min: 30, max: 70, step: 0.1, decimals: 1,
    effect: "up = more understeer",
  },
  {
    id: "bbias",
    label: "Brake bias, front",
    bindLabel: "Brake bias (BBAL)",
    short: "BB-F",
    unit: "%",
    // More front bias locks the fronts first (stable, understeers on entry);
    // less locks the rears first (rotates, then spins).
    path: "brakeBiasFront", factor: 100,
    min: 45, max: 75, step: 0.1, decimals: 1,
    effect: "up = more stable on entry",
  },
  {
    id: "diffPre",
    label: "Diff preload",
    bindLabel: "Diff preload",
    short: "PRELD",
    unit: "Nm",
    // Breakaway torque across the Drexler. Adds entry understeer and kills
    // the lock-up lag on exit; the adjustable unit covers 0-75.
    path: "diff.preloadNm", factor: 1,
    min: 0, max: 75, step: 5, decimals: 0,
    effect: "up = steadier entry, less rotation",
  },
  {
    id: "diffCoast",
    label: "Diff lock, coast",
    bindLabel: "Diff coast lock",
    short: "COAST",
    unit: "",
    // Coast-ramp lock fraction. Drexler's table: 60 deg 0.29, 50 deg 0.42
    // (as shipped), 45 deg 0.51, 40 deg 0.60. The number that steadies the
    // rear on a lift, which is exactly the handling the drivers talk about.
    path: "diff.coastLock", factor: 1,
    min: 0, max: 0.95, step: 0.01, decimals: 2,
    effect: "up = steadier on a lift",
  },
  {
    id: "launch",
    label: "Launch control",
    bindLabel: "Launch control rpm",
    short: "LC",
    unit: "rpm",
    // Where the LC holds the crank. A number the team changes between
    // accel runs on the laptop; this makes it a wheel click.
    path: "launchRpm", factor: 1,
    min: 4000, max: 12000, step: 100, decimals: 0,
    effect: "up = more wheelspin off the line",
  },
  {
    id: "final",
    label: "Final drive",
    bindLabel: "Final drive",
    short: "FINAL",
    unit: "",
    // Sprocket ratio. A step is 0.05, finer than a tooth on the rear
    // sprocket, for the same reason RSD is finer than a blade hole: find
    // where it stops helping, then pick the nearest sprocket.
    path: "finalDrive", factor: 1,
    min: 2.5, max: 4.0, step: 0.05, decimals: 2,
    effect: "up = shorter gearing",
  },
];

/**
 * The parameter paths the driver can move, in menu order.
 *
 * The failure this list prevents is silent: the run recorder snapshots the
 * car's setup from the spec sheet's editable list, and RSD and brake bias
 * were on no slider. A run's manifest claimed to record "the setup this was
 * driven with" while omitting the only two things a driver could change from
 * inside the car. Add an item to ADJUSTMENTS and the recorder and the setup
 * file both pick it up from here.
 */
export const ADJUSTABLE_PATHS = ADJUSTMENTS.map((a) => a.path);

/**
 * @param {object} v  the live SDM26 params object
 */
export function buildAdjustments(v) {
  return ADJUSTMENTS.map((a) => {
    const keys = a.path.split(".");
    const last = keys.pop();
    const holder = () => keys.reduce((o, k) => o[k], v);
    const get = () => holder()[last] * a.factor;
    return {
      ...a,
      get,
      set: (shown) => { holder()[last] = clamp(shown, a.min, a.max) / a.factor; },
      baseline: get(),
    };
  });
}

export class SetupAdjuster {
  constructor(params) {
    this.items = buildAdjustments(params);
    this.index = 0;
    this.lastChanged = -1e9;
  }

  get current() { return this.items[this.index]; }

  select(delta) {
    const n = this.items.length;
    this.index = ((this.index + delta) % n + n) % n;
    return this.current;
  }

  /**
   * Nudge the selected value.
   * @param direction  +1 or -1
   * @param scale      step multiplier while a d-pad is held (1 for a tap)
   * @param nowSeconds elapsed time, for the HUD's "recently touched" highlight
   */
  nudge(direction, scale, nowSeconds) {
    const item = this.current;
    // Snap to the step grid so repeated presses cannot drift onto 51.29998.
    // toFixed as well: 61 * 0.05 is 3.0500000000000003, and that is what the
    // setup file and the run manifest would otherwise record.
    const stepped = Math.round((item.get() + direction * item.step * scale) / item.step) * item.step;
    item.set(Number(stepped.toFixed(6)));
    this.lastChanged = nowSeconds;
    return item;
  }

  /**
   * Nudge one item by id, from its own binding rather than the menu. It also
   * becomes the menu's selection, so the HUD highlights what just moved and
   * the d-pad carries on from there.
   */
  nudgeId(id, direction, scale, nowSeconds) {
    const i = this.items.findIndex((it) => it.id === id);
    if (i < 0) return null;
    this.index = i;
    return this.nudge(direction, scale, nowSeconds);
  }

  resetAll() {
    for (const item of this.items) item.set(item.baseline);
  }

  /** Snapshot for the HUD. */
  state(nowSeconds) {
    return {
      index: this.index,
      active: nowSeconds - this.lastChanged < 4,
      items: this.items.map((it, i) => ({
        short: it.short,
        label: it.label,
        unit: it.unit,
        decimals: it.decimals,
        value: it.get(),
        baseline: it.baseline,
        effect: it.effect,
        selected: i === this.index,
      })),
    };
  }
}

/** "48.0%", "25 Nm", "7000 rpm", "3.00": the HUD's and the callout's form. */
export function formatSetupValue(value, item) {
  const n = value.toFixed(item.decimals ?? 1);
  if (!item.unit) return n;
  return item.unit === "%" ? `${n}%` : `${n} ${item.unit}`;
}

function clamp(x, lo, hi) { return x < lo ? lo : x > hi ? hi : x; }
