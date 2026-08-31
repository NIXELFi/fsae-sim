// A venue: a bounded area you drive around in, rather than a course you are
// timed on.
//
// It deliberately exposes the same interface the game already uses from
// `Track` -- startPose, locate, strikeCones, conesNear, resetCones, plus the
// centerline arrays -- so main.js, the HUD and the renderer need no branches.
// The differences are that there are no cones, no sectors, and `locate` is
// answering "are you still inside the barrier" rather than "how far along the
// course are you".

const CELL = 40; // m, spatial hash for the barrier lookup

export class Venue {
  constructor(data) {
    this.raw = data;
    this.name = data.name;
    this.kind = "venue";
    this.closed = true;
    this.length = data.lengthM;
    this.width = data.widthM;
    this.provenance = data.provenance;

    this.center = data.centerline;
    this.heading = data.heading;
    this.curvature = data.curvature;
    this.s = data.s;
    this.bankDeg = data.bankDeg;
    this.riseM = data.riseM;
    this.segment = data.segment;

    this.rings = {
      apronInner: data.apronInner,
      bankInner: data.bankInner,
      bankOuter: data.bankOuter,
      wallLine: data.wallLine,
      roadOuter: data.roadOuter,
      roadInner: data.roadInner,
    };
    this.paddock = data.paddock;
    this.wallHeight = data.wallHeightM;
    this.fenceHeight = data.fenceHeightM;
    this.apronWidth = data.apronWidthM;

    // Offsets are outward-positive. Driveable means "no further out than the
    // foot of the banking".
    this.barrierOffset = data.barrierOffsetM;
    this.spawn = data.spawn;

    // Nothing to knock over and nothing to time.
    this.cones = [];
    this.sectors = [];

    this.grid = new Map();
    for (let i = 0; i < this.center.length; i++) {
      const k = this.key(this.center[i][0], this.center[i][1]);
      let a = this.grid.get(k);
      if (!a) this.grid.set(k, (a = []));
      a.push(i);
    }
    this.lastIndex = 0;
  }

  key(x, y) {
    return `${Math.floor(x / CELL)},${Math.floor(y / CELL)}`;
  }

  startPose() {
    return { x: this.spawn.x, y: this.spawn.y, psi: this.spawn.headingRad };
  }

  poseAt(index) {
    const i = Math.max(0, Math.min(this.center.length - 1, Math.round(index)));
    return { x: this.center[i][0], y: this.center[i][1], psi: this.heading[i] };
  }

  /**
   * Nearest centreline point. The infield is 1.3 km across, so unlike a course
   * the car can be hundreds of metres from the centreline and the windowed
   * search around the last hit is useless -- go through the spatial hash, and
   * widen the search until something is found.
   */
  nearestIndex(x, y) {
    let best = -1;
    let bestD = Infinity;
    const consider = (i) => {
      const p = this.center[i];
      const dx = p[0] - x;
      const dy = p[1] - y;
      const d = dx * dx + dy * dy;
      if (d < bestD) { bestD = d; best = i; }
    };

    const cx = Math.floor(x / CELL);
    const cy = Math.floor(y / CELL);
    for (let r = 1; r <= 40 && best < 0; r++) {
      for (let i = -r; i <= r; i++) {
        for (let j = -r; j <= r; j++) {
          if (Math.max(Math.abs(i), Math.abs(j)) !== r) continue; // ring only
          const a = this.grid.get(`${cx + i},${cy + j}`);
          if (a) for (const idx of a) consider(idx);
        }
      }
    }
    if (best < 0) for (let i = 0; i < this.center.length; i++) consider(i);
    this.lastIndex = best;
    return { index: best, distance: Math.sqrt(bestD) };
  }

  /**
   * Where the car is relative to the oval.
   * `lateral` is outward-positive, matching the ring offsets.
   */
  locate(x, y, psi) {
    const { index } = this.nearestIndex(x, y);
    const p = this.center[index];
    const h = this.heading[index];
    // Outward normal for this counterclockwise loop.
    const nx = Math.sin(h);
    const ny = -Math.cos(h);
    const lateral = (x - p[0]) * nx + (y - p[1]) * ny;
    let he = psi - h;
    while (he > Math.PI) he -= 2 * Math.PI;
    while (he < -Math.PI) he += 2 * Math.PI;
    return {
      index,
      s: this.s[index],
      lateral,
      onTrack: lateral <= this.barrierOffset,
      headingErrorRad: he,
      curvature: this.curvature[index],
      bankDeg: this.bankDeg[index],
      normal: [nx, ny],
    };
  }

  /**
   * Keep the car inside the barrier at the foot of the banking.
   *
   * Rather than model a wall collision, this clamps the position back to the
   * barrier and removes the outward component of velocity -- the car slides
   * along the wall instead of sticking to or bouncing off it, which is what
   * you want when the wall exists only to stop you climbing a bank you cannot
   * drive on.
   *
   * @returns true if the car was actually pushed back
   */
  constrain(car) {
    const loc = this.locate(car.X, car.Y, car.psi);
    const over = loc.lateral - this.barrierOffset;
    if (over <= 0) return false;

    const [nx, ny] = loc.normal;
    car.X -= nx * over;
    car.Y -= ny * over;

    // Strip the outward component of the world-frame velocity.
    const cp = Math.cos(car.psi);
    const sp = Math.sin(car.psi);
    let vx = car.u * cp - car.v * sp;
    let vy = car.u * sp + car.v * cp;
    const outward = vx * nx + vy * ny;
    if (outward > 0) {
      vx -= nx * outward;
      vy -= ny * outward;
      car.u = vx * cp + vy * sp;
      car.v = -vx * sp + vy * cp;
    }
    return true;
  }

  // ---- course interface the rest of the game expects, made inert ----
  strikeCones() { return 0; }
  resetCones() {}
  conesNear() { return []; }
}

export async function loadVenue(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`venue ${url}: ${res.status}`);
  return new Venue(await res.json());
}
