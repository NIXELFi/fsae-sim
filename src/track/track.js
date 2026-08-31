// Track model: where the car is on the course, and what it just hit.
//
// The geometry is the real traced 2026 FSAE Michigan autocross and endurance
// courses from the Helios lap sim, resampled to 1 m. Everything the game needs
// -- progress, lateral error, off-course, cone strikes -- comes off that
// polyline. Both lookups go through a uniform spatial hash because the
// endurance course is 2123 points and 586 cones and this runs every frame.

const CELL = 12; // m, spatial hash cell size

class Grid {
  constructor(cell) { this.cell = cell; this.map = new Map(); }
  key(x, y) { return `${Math.floor(x / this.cell)},${Math.floor(y / this.cell)}`; }
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
        const a = this.map.get(`${cx + i},${cy + j}`);
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
    this.source = data.source;
    this.center = data.centerline;
    this.heading = data.heading;
    this.curvature = data.curvature;
    this.s = data.s;
    this.sectors = data.sectors;

    // Cones: [x, y, side]. `down` is set when the car knocks one over --
    // FSAE scores a downed-or-displaced cone at +2 s, so we track them.
    this.cones = data.cones.map(([x, y, side]) => ({ x, y, side, down: false }));

    this.centerGrid = new Grid(CELL);
    for (let i = 0; i < this.center.length; i++) {
      this.centerGrid.add(this.center[i][0], this.center[i][1], i);
    }
    this.coneGrid = new Grid(CELL);
    this.cones.forEach((c, i) => this.coneGrid.add(c.x, c.y, i));

    this.lastIndex = 0;
  }

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
    let he = psi - h;
    while (he > Math.PI) he -= 2 * Math.PI;
    while (he < -Math.PI) he += 2 * Math.PI;
    return {
      index,
      s: this.s[index],
      lateral,
      onTrack: Math.abs(lateral) <= this.width / 2 + 0.6,
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
        hits++;
      }
    }
    return hits;
  }

  resetCones() { for (const c of this.cones) c.down = false; }

  /** Cones within `range` metres of (x, y) -- what the renderer needs to draw. */
  conesNear(x, y, range) {
    const out = [];
    const cells = Math.ceil(range / CELL);
    const cx = Math.floor(x / CELL), cy = Math.floor(y / CELL);
    for (let i = -cells; i <= cells; i++) {
      for (let j = -cells; j <= cells; j++) {
        const a = this.coneGrid.map.get(`${cx + i},${cy + j}`);
        if (a) for (const idx of a) out.push(this.cones[idx]);
      }
    }
    return out;
  }
}

function clamp(x, lo, hi) { return x < lo ? lo : x > hi ? hi : x; }

export async function loadTrack(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`track ${url}: ${res.status}`);
  return new Track(await res.json());
}

export const TRACKS = [
  { id: "autocross", label: "Autocross 2026", url: "./data/track-autocross.json" },
  { id: "endurance", label: "Endurance 2026", url: "./data/track-endurance.json" },
  { id: "mis", label: "Michigan International Speedway", url: "./data/venue-mis.json",
    kind: "venue" },
];
