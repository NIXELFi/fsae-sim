// Live setup adjustments the driver can make from the wheel.
//
// These write straight into the vehicle parameters the physics is already
// holding a reference to, so a change takes effect on the next substep -- no
// restart, no reload. That is the point: you feel the balance move while the
// car is still moving.
//
// Steps are 0.1 percentage points, which is finer than anything you could
// actually set on the car (roll stiffness comes in bar holes, brake bias in
// turns of a bar). That is deliberate for a simulator: it lets you find where
// the balance actually changes before deciding what the nearest real setting is.

/**
 * @param {object} v  the live SDM26 params object
 */
export function buildAdjustments(v) {
  return [
    {
      id: "rsd",
      label: "Roll stiffness, front",
      short: "RSD-F",
      unit: "%",
      // Front share of total roll stiffness. More front = more lateral load
      // transfer at the front = less front grip = more understeer.
      get: () => v.roll.rsdFront * 100,
      set: (pct) => { v.roll.rsdFront = clamp(pct, 30, 70) / 100; },
      min: 30,
      max: 70,
      step: 0.1,
      baseline: v.roll.rsdFront * 100,
      effect: "up = more understeer",
    },
    {
      id: "bbias",
      label: "Brake bias, front",
      short: "BB-F",
      unit: "%",
      // More front bias locks the fronts first (stable, understeers on entry);
      // less locks the rears first (rotates, then spins).
      get: () => v.brakeBiasFront * 100,
      set: (pct) => { v.brakeBiasFront = clamp(pct, 45, 75) / 100; },
      min: 45,
      max: 75,
      step: 0.1,
      baseline: v.brakeBiasFront * 100,
      effect: "up = more stable on entry",
    },
  ];
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
    const stepped = Math.round((item.get() + direction * item.step * scale) / item.step) * item.step;
    item.set(stepped);
    this.lastChanged = nowSeconds;
    return item;
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
        value: it.get(),
        baseline: it.baseline,
        effect: it.effect,
        selected: i === this.index,
      })),
    };
  }
}

function clamp(x, lo, hi) { return x < lo ? lo : x > hi ? hi : x; }
