// Electronic throttle control map: accelerator pedal position (APPS) to
// throttle plate position (TPS).
//
// Any number of breakpoints, interpolated with a MONOTONE cubic Hermite spline
// (Fritsch-Carlson / PCHIP). That choice is the whole point of this file, so
// it is worth being explicit about why it is not a Catmull-Rom or a natural
// cubic:
//
//   A natural cubic or Catmull-Rom through the same breakpoints will overshoot
//   between them. On a throttle map an overshoot means the plate opens FURTHER
//   than the next breakpoint and then comes back -- so somewhere in that span,
//   pushing the pedal harder closes the throttle. That is a driveability
//   hazard, it is exactly what a pedal-plausibility check exists to catch, and
//   it can put the plate outside 0-100% entirely.
//
//   Monotone cubic Hermite is smooth (C1, no kinks at the breakpoints like
//   straight linear interpolation) and provably cannot overshoot: if the
//   breakpoints rise, the curve rises. So the map is always physically valid
//   no matter where the points are dragged.
//
// Breakpoints are stored in percent, because that is how the map is discussed
// and how it would be typed into a Motec or PE3. `evaluate` takes and returns
// 0..1 because that is what the rest of the sim uses.

const MIN_GAP = 1.0; // %, closest two breakpoints may sit on the pedal axis

export const ETC_PRESETS = {
  linear: {
    label: "Linear",
    note: "1:1. The reference every other map is judged against.",
    points: [[0, 0], [100, 100]],
  },
  progressive: {
    label: "Progressive",
    note: "Soft off the bottom. Buys pedal travel where corner exit lives, " +
          "at the cost of feeling lazy on a straight.",
    points: [[0, 0], [25, 14], [50, 36], [75, 66], [100, 100]],
  },
  aggressive: {
    label: "Aggressive",
    note: "Most of the plate in the first half of the pedal. Feels urgent, " +
          "and makes a slippery exit much harder to meter.",
    points: [[0, 0], [20, 34], [45, 64], [70, 86], [100, 100]],
  },
  wet: {
    label: "Wet",
    note: "Very soft, and capped short of full plate. Deliberately takes the " +
          "top of the torque curve away from the driver.",
    points: [[0, 0], [30, 12], [60, 33], [85, 62], [100, 82]],
  },
  endurance: {
    label: "Endurance",
    note: "Soft through the mid-pedal where a driver spends most of a stint, " +
          "full plate still available at the top.",
    points: [[0, 0], [35, 20], [65, 48], [90, 80], [100, 100]],
  },
};

export class EtcMap {
  /** @param {Array<[number, number]>} points  [pedal%, throttle%] pairs */
  constructor(points = ETC_PRESETS.linear.points, name = "linear") {
    this.name = name;
    this.setPoints(points);
  }

  /** Normalise, sort, clamp and re-establish monotonicity. */
  setPoints(points) {
    let pts = points
      .map(([x, y]) => [clamp(+x, 0, 100), clamp(+y, 0, 100)])
      .filter(([x, y]) => isFinite(x) && isFinite(y))
      .sort((a, b) => a[0] - b[0]);

    if (pts.length < 2) pts = [[0, 0], [100, 100]];

    // The pedal-closed anchor is not negotiable: 0% pedal is a shut plate.
    // Idle air is the engine's job (see Powertrain.engineTorque), not the map's.
    if (pts[0][0] > 0) pts.unshift([0, 0]);
    pts[0] = [0, 0];
    if (pts[pts.length - 1][0] < 100) pts.push([100, pts[pts.length - 1][1]]);
    pts[pts.length - 1][0] = 100;

    // Drop breakpoints that crowd their neighbour on the pedal axis.
    const spaced = [pts[0]];
    for (let i = 1; i < pts.length - 1; i++) {
      if (pts[i][0] - spaced[spaced.length - 1][0] >= MIN_GAP &&
          100 - pts[i][0] >= MIN_GAP) spaced.push(pts[i]);
    }
    spaced.push(pts[pts.length - 1]);

    // Enforce non-decreasing throttle. See the header: a map where more pedal
    // gives less plate is not a tuning choice, it is a fault.
    for (let i = 1; i < spaced.length; i++) {
      if (spaced[i][1] < spaced[i - 1][1]) spaced[i][1] = spaced[i - 1][1];
    }

    this.points = spaced;
    this._buildSlopes();
    return this.points;
  }

  /** Fritsch-Carlson tangents: smooth, and guaranteed not to overshoot. */
  _buildSlopes() {
    const p = this.points, n = p.length;
    const h = [], d = [];
    for (let i = 0; i < n - 1; i++) {
      h[i] = p[i + 1][0] - p[i][0];
      d[i] = h[i] > 0 ? (p[i + 1][1] - p[i][1]) / h[i] : 0;
    }

    const m = new Array(n).fill(0);
    if (n === 2) {
      m[0] = m[1] = d[0];
    } else {
      for (let i = 1; i < n - 1; i++) {
        if (d[i - 1] * d[i] <= 0) {
          m[i] = 0; // a flat spot or a turn: pin the tangent so it cannot bulge
        } else {
          const w1 = 2 * h[i] + h[i - 1];
          const w2 = h[i] + 2 * h[i - 1];
          m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
        }
      }
      m[0] = endpointSlope(h[0], h[1], d[0], d[1]);
      m[n - 1] = endpointSlope(h[n - 2], h[n - 3], d[n - 2], d[n - 3]);
    }
    this._m = m;
    this._h = h;
  }

  /**
   * Pedal (0..1) to plate (0..1).
   * Hot path -- called every physics substep, so it stays allocation-free.
   */
  evaluate(pedal01) {
    const x = clamp(pedal01, 0, 1) * 100;
    const p = this.points, n = p.length;
    if (x <= 0) return 0;
    if (x >= 100) return clamp(p[n - 1][1] / 100, 0, 1);

    let i = 0;
    while (i < n - 2 && x > p[i + 1][0]) i++;

    const h = this._h[i];
    if (h <= 0) return clamp(p[i][1] / 100, 0, 1);
    const t = (x - p[i][0]) / h;
    const t2 = t * t, t3 = t2 * t;
    const h00 = 2 * t3 - 3 * t2 + 1;
    const h10 = t3 - 2 * t2 + t;
    const h01 = -2 * t3 + 3 * t2;
    const h11 = t3 - t2;
    const y = h00 * p[i][1] + h10 * h * this._m[i] +
              h01 * p[i + 1][1] + h11 * h * this._m[i + 1];
    return clamp(y / 100, 0, 1);
  }

  /** Plate % at pedal % -- the units the editor works in. */
  plateAt(pedalPct) { return this.evaluate(pedalPct / 100) * 100; }

  /** `n` evenly spaced [pedal%, plate%] samples, for plotting. */
  sample(n = 101) {
    const out = [];
    for (let i = 0; i < n; i++) {
      const x = (i / (n - 1)) * 100;
      out.push([x, this.plateAt(x)]);
    }
    return out;
  }

  // ------------------------------------------------------------ editing ---

  /** Insert a breakpoint. Returns its index, or -1 if it would not fit. */
  addPoint(pedalPct, platePct) {
    const x = clamp(pedalPct, MIN_GAP, 100 - MIN_GAP);
    for (const [px] of this.points) if (Math.abs(px - x) < MIN_GAP) return -1;
    const pts = this.points.map((p) => [p[0], p[1]]);
    pts.push([x, clamp(platePct, 0, 100)]);
    this.setPoints(pts);
    return this.points.findIndex((p) => p[0] === x);
  }

  /** Remove an interior breakpoint. The two anchors cannot be removed. */
  removePoint(index) {
    if (index <= 0 || index >= this.points.length - 1) return false;
    const pts = this.points.filter((_, i) => i !== index).map((p) => [p[0], p[1]]);
    this.setPoints(pts);
    return true;
  }

  /**
   * Move a breakpoint, keeping the map valid: pedal stays strictly between its
   * neighbours, plate stays between theirs so the curve cannot go backwards.
   * The 0% anchor is fixed; the 100% anchor may only move vertically, which is
   * how you build a plate-limited map.
   */
  movePoint(index, pedalPct, platePct) {
    const pts = this.points.map((p) => [p[0], p[1]]);
    const last = pts.length - 1;
    if (index < 0 || index > last) return false;
    if (index === 0) return false;

    const loY = pts[index - 1][1];
    const hiY = index < last ? pts[index + 1][1] : 100;
    const y = clamp(platePct, loY, hiY);

    if (index === last) {
      pts[last][1] = clamp(platePct, loY, 100);
    } else {
      const loX = pts[index - 1][0] + MIN_GAP;
      const hiX = pts[index + 1][0] - MIN_GAP;
      if (hiX < loX) return false;
      pts[index] = [clamp(pedalPct, loX, hiX), y];
    }
    this.setPoints(pts);
    return true;
  }

  loadPreset(key) {
    const p = ETC_PRESETS[key];
    if (!p) return false;
    this.name = key;
    this.setPoints(p.points.map((q) => [q[0], q[1]]));
    return true;
  }

  /** Descriptive numbers for the editor readout. */
  describe() {
    // Initial gain: plate per pedal over the first 10% of travel. Below 1 the
    // map is soft off the bottom, above 1 it is sharp.
    const initialGain = this.plateAt(10) / 10;
    let maxDev = 0, atPedal = 0;
    for (let x = 0; x <= 100; x += 1) {
      const dev = this.plateAt(x) - x;
      if (Math.abs(dev) > Math.abs(maxDev)) { maxDev = dev; atPedal = x; }
    }
    return {
      points: this.points.length,
      initialGain: +initialGain.toFixed(2),
      maxDeviation: +maxDev.toFixed(1),
      maxDeviationAtPedal: atPedal,
      plateAtFullPedal: +this.plateAt(100).toFixed(1),
      character: initialGain < 0.85 ? "soft" : initialGain > 1.15 ? "sharp" : "linear",
    };
  }

  toJSON() { return { name: this.name, points: this.points.map((p) => [p[0], p[1]]) }; }

  static fromJSON(o) {
    if (!o || !Array.isArray(o.points)) return new EtcMap();
    return new EtcMap(o.points, o.name ?? "custom");
  }
}

function endpointSlope(h0, h1, d0, d1) {
  // Three-point one-sided estimate, guarded so the end span cannot overshoot.
  if (h1 == null || d1 == null || !isFinite(h1)) return d0;
  const m = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
  if (m * d0 <= 0) return 0;
  if (d0 * d1 <= 0 && Math.abs(m) > Math.abs(3 * d0)) return 3 * d0;
  return m;
}

function clamp(x, lo, hi) { return x < lo ? lo : x > hi ? hi : x; }

// ------------------------------------------------------------- persistence ---

const STORAGE_KEY = "fsae-sim.etc";

export function saveEtc(map) {
  try { localStorage.setItem(STORAGE_KEY, JSON.stringify(map.toJSON())); } catch { /* ignore */ }
}

export function loadEtc() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) return EtcMap.fromJSON(JSON.parse(raw));
  } catch { /* ignore */ }
  return new EtcMap();
}
