// Track model: where the car is on the course, and what it just hit.
//
// The geometry is the real traced 2026 FSAE Michigan autocross and endurance
// courses from the Helios lap sim, resampled to 1 m. Everything the game needs
// -- progress, lateral error, off-course, cone strikes -- comes off that
// polyline. Both lookups go through a uniform spatial hash because the
// endurance course is 2123 points and 586 cones and this runs every frame.

import { SDM26 } from "../vehicle/params.js";
import { bodyBoxFor } from "../render/carmesh.js";
import { generateTrack, parseGeneratedId, EVENTS as GEN_EVENTS } from "./generate.js";

const CELL = 12; // m, spatial hash cell size

/** How far from the car a slalom cone's gate is watched. Comfortably more
 *  than a frame's travel, and a cone further away than this cannot be the
 *  one the car is passing. */
const GATE_RANGE = 24;

/**
 * How far to the side of a slalom cone the car can be and still be passing
 * it. A slalom's pen is the course width plus 3 m, so its half is 3.25 m on
 * autocross and 3.75 m on endurance; a car further out than this is not
 * running the slalom at all -- it is on a neighbouring stretch of the
 * course, which on a generated course can run within GATE_RANGE. Judging
 * those crossings called missed gates on cars that were nowhere near the
 * cones. Wider than the pen by a car, so a driver hanging a wheel over the
 * pen's edge is still judged (and is off course anyway).
 */
const GATE_JUDGE_LATERAL_M = 5.0;

// Cell keys are a single number, not a "cx,cy" string. The renderer asks for
// every cone within 280 m each frame, which is a 49 x 49 block of cells;
// building 2401 template-literal keys for that was over half of all the
// garbage the page produced (V8 sampled ~12 MB/s of it). Cell indices are
// offset into the positive range and packed as cx * STRIDE + cy, which stays
// an exact integer well inside 2^53 for any course we will ever load.
const KEY_OFFSET = 32768;
const KEY_STRIDE = 65536;
function cellKey(cx, cy) { return (cx + KEY_OFFSET) * KEY_STRIDE + (cy + KEY_OFFSET); }

class Grid {
  constructor(cell) { this.cell = cell; this.map = new Map(); }
  key(x, y) { return cellKey(Math.floor(x / this.cell), Math.floor(y / this.cell)); }
  add(x, y, item) {
    const k = this.key(x, y);
    let a = this.map.get(k);
    if (!a) this.map.set(k, (a = []));
    a.push(item);
  }
  /** Every item in the 3x3 cell block around (x, y). */
  near(x, y) {
    const cx = Math.floor(x / this.cell), cy = Math.floor(y / this.cell);
    const out = [];
    for (let i = -1; i <= 1; i++) {
      for (let j = -1; j <= 1; j++) {
        const a = this.map.get(cellKey(cx + i, cy + j));
        if (a) out.push(...a);
      }
    }
    return out;
  }
}

export class Track {
  constructor(data) {
    this.name = data.name;
    this.closed = data.closed;
    this.length = data.lengthM;
    this.width = data.widthM;
    // Optional per-point width. A generated endurance course opens up
    // through its passing zones; everywhere else, and on every traced
    // course, the width is the one number.
    this.widths = Array.isArray(data.widths) && data.widths.length === data.centerline.length ? data.widths : null;
    this.source = data.source;
    // What a procedural course was built from, for the menu; null otherwise.
    this.generated = data.generated ?? null;
    this.center = data.centerline;
    this.heading = data.heading;
    this.curvature = data.curvature;
    this.s = data.s;
    this.sectors = data.sectors;

    // Cones: [x, y, side]. `down` is set when the car knocks one over --
    // FSAE scores a downed-or-displaced cone at +2 s, so we track them.
    //
    // Side 2 is a slalom cone, and it carries a GATE: the direction of the
    // slalom's line (dx, dy) and which side of the line the car must be on
    // when it passes -- `pass` +1 for the left, -1 for the right -- plus the
    // slalom it belongs to. `along` is the gate's own state: where the car
    // was relative to the cone's plane last frame. See `checkGates`.
    //
    // Side 3 is a POINTER: a cone laid on its side beside a slalom cone with
    // its tip pointing the way the car must go, as a real course marks it.
    // It is born down and stays down -- `resetCones` leaves it, a strike
    // never counts it (a down cone is skipped), and the renderer draws it
    // lying along its tip direction exactly as it draws a struck cone.
    this.cones = data.cones.map((c) => {
      const [x, y, side] = c;
      const cone = { x, y, side, down: false };
      if (side === 2 && c.length >= 7) {
        cone.gate = { dx: c[3], dy: c[4], pass: c[5], group: c[6], along: null };
      }
      if (side === 3) {
        cone.pointer = true;
        cone.down = true;
        cone.downAt = null;
        cone.downDir = Math.atan2(c[4] ?? 0, c[3] ?? 1);
      }
      return cone;
    });
    this.slaloms = new Set(this.cones.filter((c) => c.gate).map((c) => c.gate.group)).size;

    this.centerGrid = new Grid(CELL);
    for (let i = 0; i < this.center.length; i++) {
      this.centerGrid.add(this.center[i][0], this.center[i][1], i);
    }
    this.coneGrid = new Grid(CELL);
    this.cones.forEach((c, i) => this.coneGrid.add(c.x, c.y, i));

    this.lastIndex = 0;
    this._nearOut = null; // conesNear cache, see below
  }

  /** Course width at centreline index `i`: the nominal width, or the local
   *  one where the course has been opened up. */
  widthAt(i) { return this.widths ? this.widths[i] : this.width; }

  /** Pose of the starting grid slot: on the centreline, facing down the course. */
  startPose() {
    const [x, y] = this.center[0];
    return { x, y, psi: this.heading[0] };
  }

  /** Pose a given distance back along the course from index `i`. */
  poseAt(index) {
    const i = Math.max(0, Math.min(this.center.length - 1, Math.round(index)));
    return { x: this.center[i][0], y: this.center[i][1], psi: this.heading[i] };
  }

  /**
   * Nearest centreline point to (x, y). Searches the spatial hash, but tries a
   * window around the last result first -- the car moves a metre per frame at
   * most, so that hits almost every time.
   */
  nearestIndex(x, y) {
    const n = this.center.length;
    let best = -1, bestD = Infinity;

    const consider = (i) => {
      const p = this.center[i];
      const dx = p[0] - x, dy = p[1] - y;
      const d = dx * dx + dy * dy;
      if (d < bestD) { bestD = d; best = i; }
    };

    for (let k = -25; k <= 25; k++) {
      let i = this.lastIndex + k;
      if (this.closed) i = ((i % n) + n) % n;
      else if (i < 0 || i >= n) continue;
      consider(i);
    }
    if (bestD > 100) { // lost the thread (respawn, big off) -- do the full lookup
      for (const i of this.centerGrid.near(x, y)) consider(i);
    }
    if (best < 0) { // still nothing nearby: brute force, rare
      for (let i = 0; i < n; i++) consider(i);
    }
    this.lastIndex = best;
    return { index: best, distance: Math.sqrt(bestD) };
  }

  /**
   * Where the car is relative to the course.
   * @returns {{index, s, lateral, onTrack, headingErrorRad, curvature}}
   *          `lateral` is signed: positive is left of the centreline.
   */
  locate(x, y, psi) {
    const { index } = this.nearestIndex(x, y);
    const p = this.center[index];
    const h = this.heading[index];
    const dx = x - p[0], dy = y - p[1];
    const lateral = -Math.sin(h) * dx + Math.cos(h) * dy;
    // Distance along the course, PROJECTED onto the local heading rather than
    // snapped to the nearest node.
    //
    // The centreline is resampled to 1 m, so returning `this.s[index]` gave a
    // staircase: the value held still for seven or eight frames and then
    // jumped a whole metre. Everything downstream treats it as continuous --
    // the live delta, the ghost gap, `timeAtDistanceInLap`, and the
    // `sim.track_s_m` channel in the log -- so the delta ramped and snapped
    // once per metre. Replaying a lap against its OWN reference, where the
    // answer is zero everywhere, gave a 0.58 s peak-to-peak sawtooth, worst in
    // the slowest corners (a metre at 3 m/s is a third of a second) which is
    // exactly where a driver reads the number.
    const along = Math.cos(h) * dx + Math.sin(h) * dy;
    let s = this.s[index] + along;
    // Keep it on the course. A closed lap wraps; an open one is clamped, so a
    // car past the finish line does not report a distance the course does not
    // have.
    if (this.closed) {
      const L = this.length;
      s = ((s % L) + L) % L;
    } else {
      s = Math.max(0, Math.min(this.length, s));
    }
    let he = psi - h;
    while (he > Math.PI) he -= 2 * Math.PI;
    while (he < -Math.PI) he += 2 * Math.PI;
    // Off course is FSAE D.8.1.7.b: "all four wheels outside the course
    // boundary". The car is located by its CG, so the test moves out by the
    // reach of the tyre nearest the course -- half the track plus half a
    // tyre, from the footprint the cone test uses -- and a wheel with any of
    // its contact patch still on the line is on the course. This was a bare
    // 0.6, which is half the track and nothing else: it called the car off
    // with the outer 9.5 cm of its inside tyres still on the line. Read live
    // rather than cached because the track width is a setup adjustment.
    //
    // Yaw is ignored, and the error is on the strict side: a car sideways
    // across the line has a wheel further out than this assumes and is
    // called off up to 30 cm early. The node heading is not steady enough to
    // place wheels by -- it swings 40 degrees between adjacent 1 m nodes in
    // a hairpin -- so a yaw-aware test would flicker exactly where it
    // matters.
    const wheelReach = bodyBoxFor(SDM26).halfWidth;
    return {
      index,
      s,
      lateral,
      onTrack: Math.abs(lateral) <= this.widthAt(index) / 2 + wheelReach,
      headingErrorRad: he,
      curvature: this.curvature[index],
    };
  }

  /**
   * Knock over any cone the chassis overlaps.
   *
   * This is a rectangle-versus-circle test, not a few sample points with a
   * fudge radius around them. Sample points need slop to stop cones slipping
   * between them along the side of the car, and that slop inflates the car's
   * effective width -- with 1.05 m of clearance each side of a 1.39 m car in a
   * 3.5 m corridor, half a metre of slop turns a clean lap into a cone farm.
   * The exact test lets the driver use the real width of the course.
   *
   * @param pose  {x, y, psi} chassis centre and heading
   * @param box   {front, rear, halfWidth} extents from the CG, metres
   * @param coneRadius  cone base radius (0.155 m for an 18 in course cone)
   * @returns number of cones newly knocked down
   */
  strikeCones(pose, box, coneRadius = 0.155) {
    const c = Math.cos(pose.psi), s = Math.sin(pose.psi);
    // A cone can only be reached within this radius of the CG.
    const reach = Math.hypot(Math.max(box.front, box.rear), box.halfWidth) + coneRadius;
    let hits = 0;
    for (const idx of this.coneGrid.near(pose.x, pose.y)) {
      const cone = this.cones[idx];
      if (cone.down) continue;
      const dx = cone.x - pose.x, dy = cone.y - pose.y;
      if (dx * dx + dy * dy > reach * reach) continue;

      // Into the chassis frame, then closest point on the rectangle.
      const bx = c * dx + s * dy;
      const by = -s * dx + c * dy;
      const qx = clamp(bx, -box.rear, box.front);
      const qy = clamp(by, -box.halfWidth, box.halfWidth);
      const ex = bx - qx, ey = by - qy;
      if (ex * ex + ey * ey < coneRadius * coneRadius) {
        cone.down = true;
        // For the renderer's tumble: when, and which way it fell -- away
        // from the car, along the line from the CG to the cone.
        cone.downAt = typeof performance !== "undefined" ? performance.now() : 0;
        cone.downDir = Math.atan2(dy, dx);
        hits++;
      }
    }
    return hits;
  }

  resetCones() {
    for (const c of this.cones) {
      if (c.pointer) continue; // lies there by design
      c.down = false; c.downAt = null;
    }
    this.resetGates();
  }

  /**
   * Forget where the car was relative to every gate. After a respawn or a
   * recover the car has jumped, and a jump across a cone's plane is not a
   * pass through its gate in either direction: the next frame re-arms the
   * gates from wherever the car now is, without judging.
   */
  resetGates() { for (const c of this.cones) if (c.gate) c.gate.along = null; }

  /**
   * Slalom gates.
   *
   * D.8.1.7.a: an off course is "the vehicle did not pass through a gate in
   * the required direction", and D.11.3.2 / D.12.12.2 score missing one or
   * more gates of a slalom as ONE off course. A driver who straightlines a
   * slalom stays inside the corridor's width, so the edge test never sees
   * it; this does.
   *
   * Each slalom cone is judged the moment the car's CG crosses the plane
   * through the cone perpendicular to the slalom's line, going forward. On
   * the wrong side of the line at that moment is a missed gate. Crossing
   * backwards is ignored, and a cone the car never gets near -- along the
   * line or, past GATE_JUDGE_LATERAL_M, beside it -- is not judged, which
   * is fine, because a car that far from the line is off course by the
   * width rule anyway if it is on this stretch, and on another stretch of
   * the course if it is not.
   *
   * @param pose {x, y}
   * @returns the slalom ids whose gate was just missed, one per cone
   */
  checkGates(pose) {
    const missed = [];
    if (!this.slaloms) return missed;
    for (const cone of this.conesNear(pose.x, pose.y, GATE_RANGE)) {
      const g = cone.gate;
      if (!g) continue;
      const rx = pose.x - cone.x, ry = pose.y - cone.y;
      const along = rx * g.dx + ry * g.dy;
      const prev = g.along;
      g.along = along;
      if (prev == null || !(prev < 0 && along >= 0)) continue;
      const lateral = -g.dy * rx + g.dx * ry; // left of the line is positive
      if (Math.abs(lateral) > GATE_JUDGE_LATERAL_M) continue; // not this slalom's car
      if ((lateral >= 0 ? 1 : -1) !== g.pass) missed.push(g.group);
    }
    return missed;
  }

  /**
   * Cones within `range` metres of (x, y) -- what the renderer needs to draw.
   *
   * Returns the SAME array on every call until the query moves to a different
   * cell or asks for a different range: cones never move (a struck cone only
   * flips its `down` flags on the object the array already holds), so the
   * answer for a given cell block is fixed and the renderer, which calls this
   * every frame, only needs a fresh list when the car crosses a 12 m cell
   * boundary. Callers must treat the result as read-only.
   */
  conesNear(x, y, range) {
    const cx = Math.floor(x / CELL), cy = Math.floor(y / CELL);
    if (this._nearOut && cx === this._nearCx && cy === this._nearCy && range === this._nearRange) {
      return this._nearOut;
    }
    const out = [];
    const cells = Math.ceil(range / CELL);
    for (let i = -cells; i <= cells; i++) {
      for (let j = -cells; j <= cells; j++) {
        const a = this.coneGrid.map.get(cellKey(cx + i, cy + j));
        if (a) for (const idx of a) out.push(this.cones[idx]);
      }
    }
    this._nearCx = cx; this._nearCy = cy; this._nearRange = range;
    this._nearOut = out;
    return out;
  }
}

function clamp(x, lo, hi) { return x < lo ? lo : x > hi ? hi : x; }

export async function loadTrack(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`track ${url}: ${res.status}`);
  return new Track(await res.json());
}

/** A procedural course from its id (`gen-ax-K7Q2`), built on the spot. */
export function generatedTrack(id) {
  const g = parseGeneratedId(id);
  if (!g) throw new Error(`not a generated course id: ${id}`);
  return new Track(generateTrack(g));
}

export const TRACKS = [
  { id: "autocross", label: "Autocross 2026", url: "./data/track-autocross.json" },
  { id: "endurance", label: "Endurance 2026", url: "./data/track-endurance.json" },
  { id: "mis", label: "Michigan International Speedway", url: "./data/venue-mis.json",
    kind: "venue" },
];

/**
 * What a track id names: one of the TRACKS entries, or a generated course
 * (`kind: "generated"`, with its event and seed). Null for anything else,
 * which is how a run manifest from a build that had a course this one
 * does not is kept off the selector.
 */
export function trackSpec(id) {
  const fixed = TRACKS.find((t) => t.id === id);
  if (fixed) return fixed;
  const g = parseGeneratedId(id);
  if (!g) return null;
  return {
    id: `gen-${GEN_EVENTS[g.event].short}-${g.seed}`,
    label: `${GEN_EVENTS[g.event].label} ${g.seed}`,
    kind: "generated",
    event: g.event,
    seed: g.seed,
  };
}

export const isTrackId = (id) => trackSpec(id) !== null;
