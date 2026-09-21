// Procedural FSAE courses: a course nobody has driven, from a seed.
//
// The rulebook describes a course as a sequence of elements -- straights,
// constant-radius turns, hairpins, slaloms, chicanes, decreasing-radius
// turns -- each with dimensions, so that is exactly what this builds. A
// seeded generator draws elements from that grammar, chains them nose to
// tail as constant-curvature pieces, and keeps only the chains that stay
// clear of themselves, fit on a pad, and (for endurance) close back onto
// the start line. Everything downstream -- timing, cones, off-course,
// minimap, run manifests -- reads the same JSON the traced Michigan
// courses use, so a generated course is a first-class course.
//
// Determinism is the point: two drivers who type the same seed get the same
// course, byte for byte, on any machine, so a seed is something you can put
// in a group chat. Everything random comes from one seeded PRNG consumed in
// a fixed order; nothing reads Math.random or the clock.
//
// Dimensions are FSAE Rules 2021 V1, D.11.1.1 (autocross) and D.12.2.2
// (endurance), quoted in EVENTS below. Where the rules give no number
// ("Miscellaneous: chicanes, multiple turns, decreasing radius turns, etc.")
// the ranges are what a Michigan course actually uses.

// ------------------------------------------------------------- the rules ---

const G = 9.81;

/**
 * Per-event dimensions. Diameters are as the rules state them; the
 * generator works in centreline radius, which for a hairpin is the outside
 * diameter less the track width, halved.
 */
export const EVENTS = {
  autocross: {
    id: "autocross",
    short: "ax",
    label: "Autocross",
    rule: "FSAE 2021 D.11.1.1",
    closed: false,
    widthM: 3.5,                    // g. minimum track width
    lengthM: [740, 860],            // h. "approximately 0.80 km"
    straightHairpinMaxM: 60,        // a. with hairpins at both ends
    straightWideMaxM: 45,           // b. with wide turns on the ends
    turnDiaM: [23, 45],             // c. constant turns
    hairpinOutsideDiaM: [9, 15],    // d. 9 m minimum outside diameter
    slalomSpacingM: [7.62, 12.19],  // e.
    slalomCones: [3, 6],
    avgSpeedKmh: [40, 48],          // "average speeds should be 40 to 48 km/h"
    topSpeedKmh: null,
    footprintM: [330, 170],         // the Michigan autocross pad is 350 x 40
    passingZones: [0, 0],
    sectors: 3,
    startStraightM: [30, 45],
  },
  endurance: {
    id: "endurance",
    short: "en",
    label: "Endurance",
    rule: "FSAE 2021 D.12.2.2",
    closed: true,
    widthM: 4.5,                    // g.
    lengthM: [1150, 1400],          // one lap; the event is ~22 km of them
    straightHairpinMaxM: 77,        // a.
    straightWideMaxM: 61,           // b.
    turnDiaM: [30, 54],             // c.
    hairpinOutsideDiaM: [9, 18],    // d.
    slalomSpacingM: [9, 15],        // e.
    slalomCones: [3, 6],
    avgSpeedKmh: [48, 57],          // "average speed should be 48 to 57 km/h"
    topSpeedKmh: 105,               // "top speeds of approximately 105 km/h"
    footprintM: [420, 300],
    passingZones: [2, 3],           // h. "designated passing zones at several locations"
    sectors: 4,
    startStraightM: [40, 60],
  },
};

/**
 * How close two stretches of course may pass, centreline to centreline,
 * by how far apart they are along the course:
 *
 *  - closer than SELF_GAP_M along: neighbours on the same bend, not a pass;
 *  - up to NEAR_GAP_M along: the two legs of a hairpin, or an ess back on
 *    itself. Their cone rows may all but touch, as a real hairpin's do --
 *    a 9 m outside diameter on a 3.5 m course leaves 2 m between the legs;
 *  - further than that: an unrelated crossing, which gets the width plus
 *    two rows of cones and a car's worth of daylight, so a driver on a
 *    course they have never seen is never in doubt about which corridor
 *    is theirs.
 */
const SELF_GAP_M = 24;
const NEAR_GAP_M = 70;
const NEAR_EXTRA_M = 1.0;
const CLEARANCE_EXTRA_M = 4.0;

/**
 * A slalom is a straight line of cones, and the course through it is that
 * line: the driver supplies the weave. Two things follow. The corridor opens
 * up around the line, by SLALOM_ROOM_M in all, so there is room to weave --
 * a real slalom sits in a pen of boundary cones, not in a 3.5 m lane -- and
 * each cone carries a gate that says which side of the line the car must
 * pass it on, because a straight corridor cannot tell a slalom from a
 * straight. SLALOM_AMPLITUDE_M is how far the driver is expected to weave,
 * used only to estimate the speed the slalom is worth.
 */
const SLALOM_ROOM_M = 3.0;
const SLALOM_AMPLITUDE_M = { autocross: 1.35, endurance: 1.5 };

/** A pointer cone lies on its side beside each slalom cone, on the side the
 *  car must pass, tip pointing that way; its base sits this far out. */
const POINTER_OFFSET_M = 0.45;

/** A passing zone is a second lane: the course opens to twice its width. */
const PASSING_WIDTH_FACTOR = 2.0;
const PASSING_TAPER_M = 10;

const STEP_M = 0.5;   // build step; the output is resampled to 1 m like the traced courses

/** Why attempts were thrown away, for tuning the walk. Reset per call. */
export const diagnostics = { reasons: {} };
const reject = (why) => { diagnostics.reasons[why] = (diagnostics.reasons[why] ?? 0) + 1; return null; };

// ------------------------------------------------------------ randomness ---

/** xmur3: a string to a 32-bit seed. */
function hashSeed(str) {
  let h = 1779033703 ^ str.length;
  for (let i = 0; i < str.length; i++) {
    h = Math.imul(h ^ str.charCodeAt(i), 3432918353);
    h = (h << 13) | (h >>> 19);
  }
  h = Math.imul(h ^ (h >>> 16), 2246822507);
  h = Math.imul(h ^ (h >>> 13), 3266489909);
  return (h ^= h >>> 16) >>> 0;
}

/** mulberry32: small, fast, and identical in every JS engine. */
function mulberry32(a) {
  return function () {
    a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

class Rng {
  constructor(seed32) { this.next = mulberry32(seed32); }
  u32() { return Math.floor(this.next() * 4294967296) >>> 0; }
  float(lo, hi) { return lo + (hi - lo) * this.next(); }
  int(lo, hi) { return lo + Math.floor(this.next() * (hi - lo + 1)); } // inclusive
  pick(arr) { return arr[Math.floor(this.next() * arr.length)]; }
  /** Weighted choice over [{w, ...}]. */
  weighted(items) {
    let total = 0;
    for (const it of items) total += it.w;
    let r = this.next() * total;
    for (const it of items) { r -= it.w; if (r <= 0) return it; }
    return items[items.length - 1];
  }
}

// ----------------------------------------------------------------- seeds ---

const SEED_ALPHABET = "ABCDEFGHJKMNPQRSTUVWXYZ23456789"; // no 0/O/1/I/L

/** Canonical seed: upper case, letters/digits/dashes only, at most 12. */
export function normaliseSeed(s) {
  return String(s ?? "").toUpperCase().replace(/[^A-Z0-9-]/g, "").replace(/^-+|-+$/g, "").slice(0, 12);
}

/** A fresh 4-character seed. The one place randomness is allowed to be
 *  unseeded, because picking a seed is the user's act, not the course's. */
export function randomSeed(rand = Math.random) {
  let s = "";
  for (let i = 0; i < 4; i++) s += SEED_ALPHABET[Math.floor(rand() * SEED_ALPHABET.length)];
  return s;
}

/** The track id a generated course is filed under: `gen-ax-K7Q2`. */
export function generatedTrackId(event, seed) {
  const ev = EVENTS[event];
  if (!ev) throw new Error(`unknown event ${event}`);
  return `gen-${ev.short}-${normaliseSeed(seed) || "SDM26"}`;
}

/** `gen-ax-K7Q2` -> { event: "autocross", seed: "K7Q2" }, or null. */
export function parseGeneratedId(id) {
  const m = /^gen-(ax|en)-([A-Z0-9-]{1,12})$/i.exec(String(id ?? ""));
  if (!m) return null;
  const event = m[1].toLowerCase() === "ax" ? "autocross" : "endurance";
  return { event, seed: normaliseSeed(m[2]) };
}

// ------------------------------------------------------------- geometry ---

const TAU = Math.PI * 2;
const wrap = (a) => { while (a > Math.PI) a -= TAU; while (a < -Math.PI) a += TAU; return a; };
const mod2pi = (a) => ((a % TAU) + TAU) % TAU;

/** Advance a pose along a constant-curvature piece (kappa > 0 turns left). */
function advance(pose, kappa, len) {
  if (Math.abs(kappa) < 1e-9) {
    return { x: pose.x + Math.cos(pose.h) * len, y: pose.y + Math.sin(pose.h) * len, h: pose.h };
  }
  const h1 = pose.h + kappa * len;
  return {
    x: pose.x + (Math.sin(h1) - Math.sin(pose.h)) / kappa,
    y: pose.y - (Math.cos(h1) - Math.cos(pose.h)) / kappa,
    h: h1,
  };
}

/**
 * Sample a list of pieces [{kappa, len}] from `pose` about every STEP_M,
 * returning the points (not including the start point), each point's exact
 * distance along the element, and the end pose.
 */
function samplePieces(pose, pieces) {
  const pts = [], ss = [];
  let p = pose, along = 0;
  for (const piece of pieces) {
    const n = Math.max(1, Math.round(piece.len / STEP_M));
    const ds = piece.len / n;
    for (let i = 0; i < n; i++) {
      p = advance(p, piece.kappa, ds);
      along += ds;
      pts.push([p.x, p.y]);
      ss.push(along);
    }
  }
  return { pts, ss, end: p };
}

function piecesLength(pieces) { let L = 0; for (const p of pieces) L += p.len; return L; }

// ------------------------------------------------------------- elements ---
//
// Each maker returns { type, pieces, ...details }. Signed direction `dir` is
// +1 for a left turn. Radii are centreline radii.

function straight(len, extra = {}) {
  return { type: "straight", pieces: [{ kappa: 0, len }], lengthM: len, ...extra };
}

function arc(R, angleRad, dir) { return { kappa: dir / R, len: R * angleRad }; }

function turn(ev, rng, dir) {
  const dia = rng.float(ev.turnDiaM[0], ev.turnDiaM[1]);
  const R = dia / 2;
  const angle = (rng.float(35, 150) * Math.PI) / 180;
  return { type: "turn", pieces: [arc(R, angle, dir)], diameterM: dia, angleDeg: (angle * 180) / Math.PI, dir };
}

function hairpin(ev, rng, dir) {
  const outsideDia = rng.float(ev.hairpinOutsideDiaM[0], ev.hairpinOutsideDiaM[1]);
  const R = outsideDia / 2 - ev.widthM / 2;
  const angle = (rng.float(160, 200) * Math.PI) / 180;
  return { type: "hairpin", pieces: [arc(R, angle, dir)], outsideDiaM: outsideDia, angleDeg: (angle * 180) / Math.PI, dir };
}

/** Esses: a turn one way then the other, with a breath between. */
function chicane(ev, rng, dir) {
  const rMin = ev.hairpinOutsideDiaM[0] / 2 - ev.widthM / 2 + 3; // never tighter than a roomy hairpin
  const rMax = ev.turnDiaM[0] / 2 + 4;
  const R1 = rng.float(rMin, rMax), R2 = rng.float(rMin, rMax);
  const a1 = (rng.float(35, 75) * Math.PI) / 180, a2 = (rng.float(35, 75) * Math.PI) / 180;
  const gap = rng.float(0, 8);
  const pieces = [arc(R1, a1, dir)];
  if (gap > 0.5) pieces.push({ kappa: 0, len: gap });
  pieces.push(arc(R2, a2, -dir));
  return { type: "chicane", pieces, radiiM: [R1, R2], dir };
}

/** A constant turn that tightens: the rulebook's "decreasing radius turn". */
function decreasing(ev, rng, dir) {
  const dia1 = rng.float(ev.turnDiaM[0] + 6, ev.turnDiaM[1]);
  const R1 = dia1 / 2;
  const R2 = Math.max(ev.hairpinOutsideDiaM[0] / 2 - ev.widthM / 2 + 2, R1 * rng.float(0.45, 0.7));
  const a1 = (rng.float(40, 80) * Math.PI) / 180, a2 = (rng.float(30, 70) * Math.PI) / 180;
  return { type: "decreasing", pieces: [arc(R1, a1, dir), arc(R2, a2, dir)], radiiM: [R1, R2], dir };
}

/**
 * A slalom: cones in a straight line, with a straight run in and out.
 *
 * The line is the course; the cones sit on it every `d` metres; the first
 * cone is passed on the left (dir = +1) or the right, and the sides
 * alternate from there. `weaveR` is the radius of the arcs a driver weaving
 * SLALOM_AMPLITUDE_M either side would drive -- an S of two equal arcs over
 * a run d and a rise 2A has tan(theta/2) = 2A/d and R = d / (2 sin theta) --
 * and it is what the lap estimate charges the slalom at.
 */
function slalom(ev, rng, dir) {
  const d = rng.float(ev.slalomSpacingM[0], ev.slalomSpacingM[1]);
  const n = rng.int(ev.slalomCones[0], ev.slalomCones[1]);
  const A = SLALOM_AMPLITUDE_M[ev.id];
  const lead = 0.75 * d;
  const len = 2 * lead + (n - 1) * d;
  const coneAlong = [];
  for (let i = 0; i < n; i++) coneAlong.push(lead + i * d);
  const theta = 2 * Math.atan((2 * A) / d);
  const weaveR = d / (2 * Math.sin(theta));
  return { type: "slalom", pieces: [{ kappa: 0, len }], spacingM: d, cones: n, coneAlong, leadM: lead, amplitudeM: A, weaveR, dir };
}

// -------------------------------------------------------- dubins closure ---
//
// The shortest curve-straight-curve path of radius R from one pose to
// another (Dubins 1957, in the form LaValle gives it). Used to bring an
// endurance course back to its own start line with legal turns.

function dubinsCSC(from, to, R) {
  const dx = to.x - from.x, dy = to.y - from.y;
  const D = Math.hypot(dx, dy);
  const d = D / R;
  const phi = Math.atan2(dy, dx);
  const a = mod2pi(from.h - phi), b = mod2pi(to.h - phi);
  const sa = Math.sin(a), sb = Math.sin(b), ca = Math.cos(a), cb = Math.cos(b);
  const cab = Math.cos(a - b);
  const out = [];
  // LSL
  {
    const tmp = d + sa - sb;
    const p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb);
    if (p2 >= 0) {
      const at = Math.atan2(cb - ca, tmp);
      const t = mod2pi(-a + at), p = Math.sqrt(p2), q = mod2pi(b - at);
      out.push({ word: "LSL", t, p, q, dirs: [1, 0, 1] });
    }
  }
  // RSR
  {
    const tmp = d - sa + sb;
    const p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa);
    if (p2 >= 0) {
      const at = Math.atan2(ca - cb, tmp);
      const t = mod2pi(a - at), p = Math.sqrt(p2), q = mod2pi(-b + at);
      out.push({ word: "RSR", t, p, q, dirs: [-1, 0, -1] });
    }
  }
  // LSR
  {
    const p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb);
    if (p2 >= 0) {
      const p = Math.sqrt(p2);
      const tmp = Math.atan2(-ca - cb, d + sa + sb) - Math.atan2(-2, p);
      const t = mod2pi(-a + tmp), q = mod2pi(-mod2pi(b) + tmp);
      out.push({ word: "LSR", t, p, q, dirs: [1, 0, -1] });
    }
  }
  // RSL
  {
    const p2 = d * d - 2 + 2 * cab - 2 * d * (sa + sb);
    if (p2 >= 0) {
      const p = Math.sqrt(p2);
      const tmp = Math.atan2(ca + cb, d - sa - sb) - Math.atan2(2, p);
      const t = mod2pi(a - tmp), q = mod2pi(b - tmp);
      out.push({ word: "RSL", t, p, q, dirs: [-1, 0, 1] });
    }
  }
  return out.map((o) => ({
    ...o,
    R,
    lengthM: (o.t + o.q) * R + o.p * R,
    straightM: o.p * R,
    pieces: [
      { kappa: o.dirs[0] / R, len: o.t * R },
      { kappa: 0, len: o.p * R },
      { kappa: o.dirs[2] / R, len: o.q * R },
    ].filter((pc) => pc.len > 1e-6),
  }));
}

// -------------------------------------------------------------- the walk ---

/** Bounding box of a point list, incrementally. */
class Bounds {
  constructor() { this.minX = Infinity; this.maxX = -Infinity; this.minY = Infinity; this.maxY = -Infinity; }
  add(x, y) {
    if (x < this.minX) this.minX = x; if (x > this.maxX) this.maxX = x;
    if (y < this.minY) this.minY = y; if (y > this.maxY) this.maxY = y;
  }
  with(pts) {
    const b = new Bounds();
    b.minX = this.minX; b.maxX = this.maxX; b.minY = this.minY; b.maxY = this.maxY;
    for (const [x, y] of pts) b.add(x, y);
    return b;
  }
  get w() { return this.maxX - this.minX; }
  get h() { return this.maxY - this.minY; }
  /** Fits a W x H pad in either orientation. */
  fits(W, H) { return (this.w <= W && this.h <= H) || (this.w <= H && this.h <= W); }
}

/**
 * The growing course: sample points with their along-course distance, a
 * spatial hash over them, and the tests a candidate element has to pass.
 *
 * Points are pushed and popped in order, element by element, so the hash
 * can be a stack too: the last point in any cell is always the most
 * recently pushed one, and popping an element pops its points off the ends
 * of their cells.
 */
class Chain {
  constructor(ev) {
    this.ev = ev;
    this.pts = [];       // [x, y]
    this.s = [];         // along-course distance of each point
    this.room = [];      // extra half-width each point needs (a slalom's pen)
    this.elements = [];  // { ...element, s0, s1, poseIn, poseOut, count }
    this.pose = { x: 0, y: 0, h: 0 };
    this.length = 0;
    this.bounds = new Bounds();
    this.clearance = ev.widthM + CLEARANCE_EXTRA_M;
    this.nearClearance = ev.widthM + NEAR_EXTRA_M;
    this.cell = this.clearance; // a 3x3 block of cells covers the clearance
    this.grid = new Map();
  }

  key(x, y) { return `${Math.floor(x / this.cell)},${Math.floor(y / this.cell)}`; }

  /**
   * Every point of `pts` (each `ss[k]` along the element that starts at
   * along-distance s0) clears the course laid so far. Points within
   * SELF_GAP_M behind are neighbours, not a crossing. For the path that
   * closes a lap, the last SELF_GAP_M of it is also allowed beside the first
   * SELF_GAP_M of the course: that is the join.
   */
  clear(pts, ss, s0, closing = false, extra = 0) {
    const total = ss[ss.length - 1];
    for (let k = 0; k < pts.length; k++) {
      const [x, y] = pts[k];
      const sk = s0 + ss[k];
      const atTail = closing && total - ss[k] < SELF_GAP_M;
      const cx = Math.floor(x / this.cell), cy = Math.floor(y / this.cell);
      for (let i = -1; i <= 1; i++) {
        for (let j = -1; j <= 1; j++) {
          const bucket = this.grid.get(`${cx + i},${cy + j}`);
          if (!bucket) continue;
          for (const idx of bucket) {
            const gap = sk - this.s[idx];
            if (gap < SELF_GAP_M) continue;
            const dx = this.pts[idx][0] - x, dy = this.pts[idx][1] - y;
            const d2 = dx * dx + dy * dy;
            if (atTail && this.s[idx] < SELF_GAP_M) continue;
            // The join of a lap is a hairpin-like pass too: the closing
            // path's tail beside the first bend of the course. Either side
            // may be a slalom, whose pen needs its extra room.
            const near = gap < NEAR_GAP_M || (atTail && this.s[idx] < NEAR_GAP_M);
            const need = (near ? this.nearClearance : this.clearance) + this.room[idx] + extra;
            if (d2 < need * need) return false;
          }
        }
      }
    }
    return true;
  }

  push(element, pts, ss, end) {
    const s0 = this.length;
    const room = element.type === "slalom" ? SLALOM_ROOM_M / 2 : 0;
    for (let k = 0; k < pts.length; k++) {
      const p = pts[k];
      const idx = this.pts.length;
      this.pts.push(p);
      this.s.push(s0 + ss[k]);
      this.room.push(room);
      this.bounds.add(p[0], p[1]);
      const key = this.key(p[0], p[1]);
      let bucket = this.grid.get(key);
      if (!bucket) this.grid.set(key, (bucket = []));
      bucket.push(idx);
    }
    this.length = s0 + (ss.length ? ss[ss.length - 1] : 0);
    this.elements.push({ ...element, s0, s1: this.length, poseIn: this.pose, poseOut: end, count: pts.length });
    this.pose = end;
  }

  pop() {
    const el = this.elements.pop();
    if (!el) return null;
    for (let k = 0; k < el.count; k++) {
      const p = this.pts.pop();
      this.s.pop();
      this.room.pop();
      const bucket = this.grid.get(this.key(p[0], p[1]));
      bucket.pop();
    }
    this.length = el.s0;
    this.pose = el.poseIn;
    this.bounds = new Bounds();
    for (const [x, y] of this.pts) this.bounds.add(x, y);
    return el;
  }

  get last() { return this.elements[this.elements.length - 1] ?? null; }
  countOf(type) { let c = 0; for (const e of this.elements) if (e.type === type) c++; return c; }
}

/** Which element comes next, by what came before. Weights, not rules: the
 *  rules are enforced on the result. */
function chooseType(rng, chain, ev, targetLen) {
  const last = chain.last?.type ?? "start";
  const frac = chain.length / targetLen;
  const hairpins = chain.countOf("hairpin"), slaloms = chain.countOf("slalom");
  const items = [];
  const add = (type, w) => { if (w > 0) items.push({ type, w }); };
  if (last === "straight") {
    add("turn", 4.5);
    add("hairpin", hairpins < 5 ? 2.2 : 0.6);
    add("chicane", 1.6);
    add("decreasing", 0.9);
    add("slalom", (slaloms === 0 && frac > 0.3 ? 3.0 : 1.4) - (slaloms >= 4 ? 1.0 : 0));
  } else if (last === "slalom") {
    add("straight", 4);
    add("turn", 2);
    add("hairpin", 0.8);
  } else {
    // after any corner
    add("straight", 5);
    add("turn", 1.8);
    add("chicane", 1.0);
    add("slalom", slaloms < 3 ? 0.9 : 0.2);
    add("hairpin", last === "hairpin" ? 0 : 0.5);
  }
  return rng.weighted(items).type;
}

function makeElement(type, ev, rng, chain) {
  const dir = rng.next() < 0.5 ? -1 : 1;
  switch (type) {
    case "straight": {
      const afterHairpin = chain.last?.type === "hairpin";
      // a. a straight up to the long limit only with hairpins at both ends;
      // b. otherwise the short limit. The next element is forced to match.
      const long = afterHairpin && rng.next() < 0.35;
      const max = long ? ev.straightHairpinMaxM : ev.straightWideMaxM;
      const len = rng.float(8, max);
      return straight(len, { mustFollow: len > ev.straightWideMaxM ? "hairpin" : null });
    }
    case "turn": return turn(ev, rng, dir);
    case "hairpin": return hairpin(ev, rng, dir);
    case "chicane": return chicane(ev, rng, dir);
    case "decreasing": return decreasing(ev, rng, dir);
    case "slalom": return slalom(ev, rng, dir);
    default: throw new Error(`no such element ${type}`);
  }
}

/** Both directions of a turning element, as candidates. */
function withMirror(el) {
  if (el.type === "straight") return [el];
  const m = { ...el, dir: -el.dir, pieces: el.pieces.map((p) => ({ kappa: -p.kappa, len: p.len })) };
  return [el, m];
}

/** How much a candidate's end pose wants to be picked: pointing back toward
 *  the course's centre keeps the walk compact, and toward the start line
 *  when it is time to come home. */
function steerScore(end, target) {
  if (!target) return 1;
  const bearing = Math.atan2(target.y - end.y, target.x - end.x);
  const err = Math.abs(wrap(bearing - end.h));
  return 0.35 + Math.exp(-(err * err) / (1.1 * 1.1));
}

/**
 * One attempt at a course from one RNG stream. Returns the chain, or null.
 */
function attempt(ev, rng) {
  const chain = new Chain(ev);
  const targetLen = rng.float(ev.lengthM[0], ev.lengthM[1]);
  const [W, H] = ev.footprintM;

  // A lap needs a shape to go round, or the walk curls into its own middle
  // and boxes itself in short of the start. The shape is a ring through the
  // start line, tangent to it, of about half the lap's length: the course
  // meanders around it -- hairpins, slaloms and esses spend length without
  // spending ring -- and arrives home pointing down the start straight,
  // where a short legal closing path finishes the job. Which way round is
  // the seed's choice.
  let ring = null;
  if (ev.closed) {
    const sign = rng.next() < 0.5 ? 1 : -1;
    const frac = rng.float(0.45, 0.62);
    const r = Math.min(Math.min(W, H) / 2 - 30, (frac * targetLen) / TAU);
    ring = {
      sign, r,
      at: (s) => {
        const phi = Math.min(TAU, (TAU * s) / targetLen);
        return { x: r * Math.sin(phi), y: sign * r * (1 - Math.cos(phi)) };
      },
    };
  }

  // The start: a straight, so the launch and the start/finish line are on one.
  {
    const el = straight(rng.float(ev.startStraightM[0], ev.startStraightM[1]), { start: true });
    const { pts, ss, end } = samplePieces(chain.pose, el.pieces);
    chain.push(el, pts, ss, end);
  }

  let backtracks = 0;
  let closure = null;
  for (let guard = 0; guard < 400; guard++) {
    // Endurance: try to close once there is room for the closing path to
    // land inside the length band.
    // (A straight that was drawn long on the promise of a hairpin after
    // it keeps that promise: no closing path until the hairpin is laid.)
    if (ev.closed && chain.length >= ev.lengthM[0] - 240 && !chain.last?.mustFollow) {
      closure = tryClose(ev, rng, chain);
      if (closure) break;
      if (chain.length > ev.lengthM[1] - 40) {
        // Too long to close legally: back up a couple of elements and go
        // another way.
        chain.pop(); chain.pop();
        if (++backtracks > 100 || !chain.last) return reject("closure: out of backtracks");
        continue;
      }
    }
    if (!ev.closed && chain.length >= targetLen) break;

    const forced = chain.last?.mustFollow ?? null;
    const candidates = [];
    for (let k = 0; k < 10; k++) {
      const type = forced ?? chooseType(rng, chain, ev, targetLen);
      const el = makeElement(type, ev, rng, chain);
      for (const c of withMirror(el)) {
        const { pts, ss, end } = samplePieces(chain.pose, c.pieces);
        if (!chain.bounds.with(pts).fits(W, H)) continue;
        if (!chain.clear(pts, ss, chain.length, false, c.type === "slalom" ? SLALOM_ROOM_M / 2 : 0)) continue;
        // Steer: a lap follows its ring, a little ahead of where it is; a
        // run steers toward the middle of what is laid, which keeps it
        // compact on the pad.
        const sEnd = chain.length + ss[ss.length - 1];
        const target = ring
          ? ring.at(sEnd + 0.07 * targetLen)
          : { x: (chain.bounds.minX + chain.bounds.maxX) / 2, y: (chain.bounds.minY + chain.bounds.maxY) / 2 };
        let w = steerScore(end, target);
        if (ring) {
          // And stay near the ring: far off it, the lap will not close.
          const off = Math.abs(Math.hypot(end.x, end.y - ring.sign * ring.r) - ring.r);
          w *= off > 60 ? 0.25 : off > 35 ? 0.6 : 1;
        }
        candidates.push({ el: c, pts, ss, end, w });
      }
      if (forced && candidates.length) break;
    }
    if (!candidates.length) {
      if (!chain.pop() || !chain.last || ++backtracks > 100) return reject("walk: out of backtracks");
      continue;
    }
    const pick = rng.weighted(candidates);
    chain.push(pick.el, pick.pts, pick.ss, pick.end);
  }

  if (ev.closed) {
    if (!closure) return reject("closure: never found");
    for (const el of closure) {
      const { pts, ss, end } = samplePieces(chain.pose, el.pieces);
      chain.push(el, pts, ss, end);
    }
  } else {
    // An autocross run ends with a short straight to the finish line, as
    // every real one does. A long straight owed a hairpin cannot be it.
    while (chain.last?.mustFollow) chain.pop();
    if (!chain.last) return reject("finish straight");
    const last = chain.last;
    if (last.type !== "straight") {
      const el = straight(rng.float(10, 25), { finish: true });
      const { pts, ss, end } = samplePieces(chain.pose, el.pieces);
      if (!chain.bounds.with(pts).fits(W, H) || !chain.clear(pts, ss, chain.length)) return reject("finish straight");
      chain.push(el, pts, ss, end);
    } else last.finish = true;
  }

  // The rules want a varied course, and so does a driver.
  const minimum = { hairpin: 1, slalom: 1, turn: 2, chicane: 1 };
  for (const [type, n] of Object.entries(minimum)) if (chain.countOf(type) < n) return reject(`missing ${type}`);
  if (chain.length < ev.lengthM[0] || chain.length > ev.lengthM[1]) return reject("length");
  return chain;
}

/**
 * Close an endurance course: a Dubins path from the current pose to the
 * start pose, with legal constant-turn radii. A closing straight longer than
 * the wide-turn limit gets a slalom in it, which is what a real course does
 * with a long straight. Returns the closing path as ordinary elements --
 * turns and straights -- or null.
 */
function tryClose(ev, rng, chain) {
  const room = ev.lengthM[1] - chain.length;
  const need = ev.lengthM[0] - chain.length;
  const radii = [];
  for (let k = 0; k < 6; k++) radii.push((ev.turnDiaM[0] + ((ev.turnDiaM[1] - ev.turnDiaM[0]) * k) / 5) / 2);
  const start = { x: 0, y: 0, h: 0 };
  const minArc = (8 * Math.PI) / 180, maxArc = (270 * Math.PI) / 180;
  const landed = (end) => Math.hypot(end.x, end.y) <= 0.05 && Math.abs(wrap(end.h)) <= 1e-3;
  let best = null;
  for (const R of radii) {
    for (const cand of dubinsCSC(chain.pose, start, R)) {
      // The closing path has to land the lap inside the length band.
      if (cand.lengthM > room || cand.lengthM < need) continue;
      // Each arc must be a real turn (the rules have no "kink"): under 8
      // degrees is a wobble, not a corner -- unless it is nothing at all.
      const arcs = cand.pieces.filter((p) => p.kappa !== 0);
      if (arcs.some((p) => p.len / R < minArc || p.len / R > maxArc)) continue;

      const asElement = (p) => (p.kappa === 0
        ? straight(p.len, { closing: true })
        : { type: "turn", pieces: [p], diameterM: 2 * R, angleDeg: (p.len / R) * (180 / Math.PI), dir: Math.sign(p.kappa), closing: true });
      let elements;
      if (cand.straightM <= ev.straightWideMaxM) {
        elements = cand.pieces.map(asElement);
      } else {
        // A slalom in the middle of the straight. Its entry and exit return
        // to the line, so the join still lands on the start.
        const sl = slalom(ev, rng, rng.next() < 0.5 ? 1 : -1);
        const lead = (cand.straightM - piecesLength(sl.pieces)) / 2;
        if (lead < 6 || lead > ev.straightWideMaxM) continue;
        elements = [];
        for (const p of cand.pieces) {
          if (p.kappa !== 0) elements.push(asElement(p));
          else elements.push(straight(lead, { closing: true }), sl, straight(lead, { closing: true }));
        }
      }
      const { pts, ss, end } = samplePieces(chain.pose, elements.flatMap((e) => e.pieces));
      if (!landed(end)) continue;
      if (!chain.bounds.with(pts).fits(ev.footprintM[0], ev.footprintM[1])) continue;
      const penRoom = elements.some((e) => e.type === "slalom") ? SLALOM_ROOM_M / 2 : 0;
      if (!chain.clear(pts, ss, chain.length, true, penRoom)) continue;
      if (!best || cand.lengthM < best.lengthM) best = { lengthM: cand.lengthM, elements };
    }
  }
  return best ? best.elements : null;
}

// ----------------------------------------------------------- finishing ---

/** Arc-length resample to `step`, the same routine prepare_data.py uses. */
function resample(points, closed, step) {
  const pts = closed ? [...points, points[0]] : points;
  const out = [pts[0].slice()];
  let carry = 0;
  for (let i = 0; i < pts.length - 1; i++) {
    const [ax, ay] = pts[i], [bx, by] = pts[i + 1];
    const seg = Math.hypot(bx - ax, by - ay);
    if (seg <= 1e-9) continue;
    let t = step - carry;
    while (t <= seg) {
      const f = t / seg;
      out.push([ax + f * (bx - ax), ay + f * (by - ay)]);
      t += step;
    }
    carry = (carry + seg) % step;
  }
  if (closed && out.length > 1 && Math.hypot(out[out.length - 1][0] - out[0][0], out[out.length - 1][1] - out[0][1]) < step * 0.5) out.pop();
  return out;
}

/** Cumulative distance, heading and signed curvature per point. */
function geometry(center, closed) {
  const n = center.length;
  const s = new Array(n).fill(0), heading = new Array(n).fill(0), curv = new Array(n).fill(0);
  for (let i = 1; i < n; i++) s[i] = s[i - 1] + Math.hypot(center[i][0] - center[i - 1][0], center[i][1] - center[i - 1][1]);
  const at = (i) => (closed ? ((i % n) + n) % n : Math.max(0, Math.min(n - 1, i)));
  for (let i = 0; i < n; i++) {
    const p = center[at(i - 1)], q = center[at(i + 1)];
    heading[i] = Math.atan2(q[1] - p[1], q[0] - p[0]);
  }
  for (let i = 0; i < n; i++) {
    const a = heading[at(i - 1)], b = heading[at(i + 1)];
    const ds = (s[Math.min(i + 1, n - 1)] - s[Math.max(i - 1, 0)]) || 1;
    curv[i] = wrap(b - a) / ds;
  }
  return { s, heading, curv };
}

/** Edge cones on both sides, spacing tightening with the local radius as
 *  prepare_data.py does, skipping spans a slalom owns. */
function placeEdgeCones(center, heading, curv, s, halfAt, skip) {
  const cones = [];
  let nextS = 0;
  for (let i = 0; i < center.length; i++) {
    if (s[i] < nextS) continue;
    if (skip.some(([a, b]) => s[i] >= a && s[i] <= b)) continue;
    const radius = Math.abs(curv[i]) > 1e-4 ? 1 / Math.abs(curv[i]) : 1e6;
    const spacing = Math.max(3, Math.min(7, 1.6 + 0.3 * radius));
    nextS = s[i] + spacing;
    const nx = -Math.sin(heading[i]), ny = Math.cos(heading[i]);
    const [cx, cy] = center[i];
    const half = halfAt(i);
    cones.push([r3(cx + nx * half), r3(cy + ny * half), 0]);
    cones.push([r3(cx - nx * half), r3(cy - ny * half), 1]);
  }
  return cones;
}

const r3 = (v) => Math.round(v * 1000) / 1000;

/**
 * A quasi-static lap-time estimate: point mass on a friction circle with a
 * power limit, forward and backward passes over the curvature profile. Not
 * the vehicle model -- a few percent optimistic against it -- but enough to
 * hold a generated course to the rulebook's average-speed band, which is the
 * only thing the rules say about speed.
 */
export function estimateLap(curvature, s, closed, opts = {}) {
  const aLat = (opts.aLatG ?? 1.35) * G, aAcc = (opts.aAccG ?? 0.85) * G, aBrk = (opts.aBrkG ?? 1.4) * G;
  const vTop = (opts.topKmh ?? 112) / 3.6;
  const powerW = opts.powerW ?? 46000 * 0.85, mass = opts.massKg ?? 267 + 75;
  const n = curvature.length;
  const vLim = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const k = Math.abs(curvature[i]);
    vLim[i] = Math.min(vTop, k > 1e-6 ? Math.sqrt(aLat / k) : vTop);
  }
  const ds = (i) => (i + 1 < n ? s[i + 1] - s[i] : (closed ? s[1] - s[0] : 0));
  const v = new Float64Array(n);
  // Forward: accelerate within what the friction circle leaves over lateral.
  const fwd = (v0) => {
    v[0] = Math.min(vLim[0], v0);
    for (let i = 0; i + 1 < n; i++) {
      const k = Math.abs(curvature[i]);
      const latUsed = v[i] * v[i] * k;
      const spare = Math.max(0, 1 - (latUsed / aLat) ** 2);
      const aTr = Math.min(aAcc * Math.sqrt(spare), v[i] > 1 ? powerW / (mass * v[i]) : aAcc);
      v[i + 1] = Math.min(vLim[i + 1], Math.sqrt(v[i] * v[i] + 2 * aTr * ds(i)));
    }
  };
  fwd(closed ? vLim[0] : 0);
  if (closed) fwd(Math.min(v[n - 1], vLim[0]));  // a flying lap starts at the speed it ends
  // Backward: brake.
  for (let pass = 0; pass < (closed ? 2 : 1); pass++) {
    for (let i = n - 2; i >= 0; i--) {
      const k = Math.abs(curvature[i + 1]);
      const latUsed = v[i + 1] * v[i + 1] * k;
      const spare = Math.max(0, 1 - (latUsed / aLat) ** 2);
      const aB = aBrk * Math.sqrt(spare);
      v[i] = Math.min(v[i], Math.sqrt(v[i + 1] * v[i + 1] + 2 * aB * ds(i)));
    }
    if (closed) v[n - 1] = Math.min(v[n - 1], Math.sqrt(v[0] * v[0] + 2 * aBrk * ds(n - 1)));
  }
  let t = 0, top = 0;
  for (let i = 0; i + 1 < n; i++) {
    const vm = Math.max(0.5, (v[i] + v[i + 1]) / 2);
    t += ds(i) / vm;
    if (v[i] > top) top = v[i];
  }
  if (closed) t += ds(n - 1) / Math.max(0.5, (v[n - 1] + v[0]) / 2);
  const L = closed ? s[n - 1] + ds(n - 1) : s[n - 1];
  return { timeS: t, avgKmh: (L / t) * 3.6, topKmh: top * 3.6 };
}

/** The course as the game wants it, from a finished chain. */
function finish(ev, chain, seed, attemptNo) {
  const closed = ev.closed;
  // Stitch: for a closed course the last sample sits on the start point.
  let raw = [[0, 0], ...chain.pts];
  if (closed) {
    while (raw.length > 1 && Math.hypot(raw[raw.length - 1][0], raw[raw.length - 1][1]) < STEP_M * 0.75) raw.pop();
  }
  const centre = resample(raw, closed, 1.0);

  // Recentre on the centroid, like the traced courses: world coordinates
  // stay small near the car.
  let ox = 0, oy = 0;
  for (const [x, y] of centre) { ox += x; oy += y; }
  ox /= centre.length; oy /= centre.length;
  const center = centre.map(([x, y]) => [r3(x - ox), r3(y - oy)]);
  const shift = ([x, y]) => [x - ox, y - oy];

  const { s, heading, curv } = geometry(center, closed);
  const length = s[s.length - 1] + (closed ? Math.hypot(center[0][0] - center[center.length - 1][0], center[0][1] - center[center.length - 1][1]) : 0);

  // Per-point width: nominal, opened up through slaloms and passing zones.
  const zones = [];
  const widths = new Array(center.length).fill(ev.widthM);
  const openUp = (a, b, w, taper) => {
    for (let i = 0; i < center.length; i++) {
      const inner = Math.min(s[i] - a, b - s[i]);
      if (inner < -taper) continue;
      const f = Math.max(0, Math.min(1, (inner + taper) / taper));
      widths[i] = Math.max(widths[i], r3(ev.widthM + (w - ev.widthM) * f));
    }
  };
  const slaloms = chain.elements.filter((e) => e.type === "slalom");
  for (const e of slaloms) {
    // The pen runs from the first cone to the last, tapering open over the
    // lead-in and closed over the lead-out.
    openUp(e.s0 + e.leadM, e.s1 - e.leadM, ev.widthM + SLALOM_ROOM_M, e.leadM);
  }
  if (ev.passingZones[1] > 0) {
    const straights = chain.elements
      .filter((e) => e.type === "straight" && !e.start && e.lengthM >= 30)
      .sort((a, b) => b.lengthM - a.lengthM);
    // A zone is twice as wide, so its cones reach a width further out than
    // the walk allowed for: only a straight with that much room on both
    // sides -- against every stretch that is not its own neighbourhood --
    // can be one.
    const roomNeeded = (ev.widthM * PASSING_WIDTH_FACTOR) / 2 + ev.widthM / 2 + NEAR_EXTRA_M;
    const hasRoom = (e) => {
      for (let i = 0; i < center.length; i++) {
        if (s[i] < e.s0 - PASSING_TAPER_M || s[i] > e.s1 + PASSING_TAPER_M) continue;
        for (let j = 0; j < center.length; j++) {
          let gap = Math.abs(s[j] - s[i]);
          if (closed) gap = Math.min(gap, length - gap);
          if (gap < SELF_GAP_M) continue;
          const dx = center[j][0] - center[i][0], dy = center[j][1] - center[i][1];
          if (dx * dx + dy * dy < roomNeeded * roomNeeded) return false;
        }
      }
      return true;
    };
    for (const e of straights) {
      if (zones.length >= ev.passingZones[1]) break;
      if (hasRoom(e)) zones.push([e.s0, e.s1]);
    }
    for (const [a, b] of zones) openUp(a + PASSING_TAPER_M, b - PASSING_TAPER_M, ev.widthM * PASSING_WIDTH_FACTOR, PASSING_TAPER_M);
  }
  const halfAt = (i) => widths[i] / 2;

  // Cones: the edges everywhere -- through a slalom the edge IS the pen --
  // then the slalom cones on their lines, each with its gate.
  const cones = placeEdgeCones(center, heading, curv, s, halfAt, []);
  slaloms.forEach((e, group) => {
    const c = Math.cos(e.poseIn.h), sn = Math.sin(e.poseIn.h);
    e.coneAlong.forEach((along, i) => {
      const [x, y] = shift([e.poseIn.x + c * along, e.poseIn.y + sn * along]);
      const pass = e.dir * (i % 2 === 0 ? 1 : -1);
      cones.push([r3(x), r3(y), 2, r3(c), r3(sn), pass, group]);
      // Its pointer: [x, y, 3, tipX, tipY], lying on the pass side.
      const px = -sn * pass, py = c * pass;
      cones.push([r3(x + px * POINTER_OFFSET_M), r3(y + py * POINTER_OFFSET_M), 3, r3(px), r3(py)]);
    });
  });

  const elements = chain.elements.map((e) => {
    const out = { type: e.type, s0: r3(e.s0), s1: r3(e.s1), lengthM: r3(e.s1 - e.s0) };
    for (const k of ["dir", "diameterM", "outsideDiaM", "angleDeg", "radiiM", "spacingM", "cones", "leadM"]) {
      if (e[k] !== undefined) out[k] = Array.isArray(e[k]) ? e[k].map(r3) : (typeof e[k] === "number" ? r3(e[k]) : e[k]);
    }
    if (e.start) out.start = true;
    if (e.finish) out.finish = true;
    if (e.closing) out.closing = true;
    return out;
  });

  // The line through a slalom is straight; the car's path is not. Charge
  // the estimate the weave's curvature through each one.
  const curvForEstimate = curv.slice();
  for (const e of slaloms) {
    for (let i = 0; i < center.length; i++) {
      if (s[i] >= e.s0 + e.leadM / 2 && s[i] <= e.s1 - e.leadM / 2) curvForEstimate[i] = 1 / e.weaveR;
    }
  }
  const estimate = estimateLap(curvForEstimate, s, closed, { topKmh: ev.topSpeedKmh ? ev.topSpeedKmh + 7 : 112 });

  const count = (t) => elements.filter((e) => e.type === t).length;
  const straights = elements.filter((e) => e.type === "straight");
  const stats = {
    hairpins: count("hairpin"),
    turns: count("turn") + count("decreasing"),
    decreasing: count("decreasing"),
    chicanes: count("chicane"),
    slaloms: count("slalom"),
    longestStraightM: r3(Math.max(0, ...straights.map((e) => e.lengthM))),
    passingZones: zones.length,
    footprintM: [r3(chain.bounds.w), r3(chain.bounds.h)],
    attempt: attemptNo,
  };

  const sectors = [];
  for (let i = 1; i < ev.sectors; i++) sectors.push(r3((length * i) / ev.sectors));

  const data = {
    name: `${ev.label} ${seed}`,
    closed,
    lengthM: r3(length),
    widthM: ev.widthM,
    source: `Procedural course, seed ${seed}, built to ${ev.rule}`,
    centerline: center,
    heading: heading.map((h) => Math.round(h * 1e5) / 1e5),
    curvature: curv.map((c) => Math.round(c * 1e6) / 1e6),
    s: s.map(r3),
    cones,
    sectors,
    generated: {
      seed,
      event: ev.id,
      id: generatedTrackId(ev.id, seed),
      rule: ev.rule,
      elements,
      stats,
      passingZones: zones.map(([a, b]) => [r3(a), r3(b)]),
      estimate: { timeS: r3(estimate.timeS), avgKmh: r3(estimate.avgKmh), topKmh: r3(estimate.topKmh) },
    },
  };
  data.widths = widths;
  return data;
}

// ------------------------------------------------------------- the entry ---

/**
 * Generate a course.
 *
 * @param {object} opts
 * @param {"autocross"|"endurance"} opts.event
 * @param {string} opts.seed   any text; normalised to the canonical seed
 * @returns the track JSON (see prepare_data.py for the schema) plus a
 *          `generated` block describing what was built and why.
 */
export function generateTrack({ event = "autocross", seed = "" } = {}) {
  const ev = EVENTS[event];
  if (!ev) throw new Error(`unknown event "${event}"`);
  const code = normaliseSeed(seed) || "SDM26";
  const master = new Rng(hashSeed(`${ev.id}:${code}`));

  // Attempts draw from their own streams, so an attempt that fails early
  // does not shift the randomness of every later one: a small change to the
  // generator changes fewer seeds' courses.
  const streams = [];
  for (let i = 0; i < 400; i++) streams.push(master.u32());

  diagnostics.reasons = {};
  const band = ev.avgSpeedKmh;
  const pad = 4; // km/h either side: the estimator is not the vehicle model
  let fallback = null, fallbackErr = Infinity;
  for (let i = 0; i < streams.length; i++) {
    const chain = attempt(ev, new Rng(streams[i]));
    if (!chain) continue;
    const data = finish(ev, chain, code, i + 1);
    const avg = data.generated.estimate.avgKmh;
    if (avg >= band[0] - pad && avg <= band[1] + pad) return data;
    reject(avg < band[0] ? "too slow" : "too fast");
    const err = avg < band[0] ? band[0] - avg : avg - band[1];
    if (err < fallbackErr) { fallback = data; fallbackErr = err; }
  }
  if (fallback) return fallback;
  throw new Error(`could not lay out a ${ev.label} course for seed ${code}`);
}

/** A one-line description for menus: "4 hairpins, 6 turns, 3 slaloms...". */
export function describeGenerated(gen) {
  if (!gen) return "";
  const st = gen.stats;
  const bits = [
    `${st.hairpins} hairpin${st.hairpins === 1 ? "" : "s"}`,
    `${st.turns} turn${st.turns === 1 ? "" : "s"}${st.decreasing ? ` (${st.decreasing} tightening)` : ""}`,
    `${st.chicanes} chicane${st.chicanes === 1 ? "" : "s"}`,
    `${st.slaloms} slalom${st.slaloms === 1 ? "" : "s"}`,
    `longest straight ${Math.round(st.longestStraightM)} m`,
  ];
  if (st.passingZones) bits.push(`${st.passingZones} passing zone${st.passingZones === 1 ? "" : "s"}`);
  return bits.join(", ");
}
