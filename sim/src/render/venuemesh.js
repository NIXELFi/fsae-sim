// Venue geometry: the banked oval, its wall and fence, and the infield.
//
// Built as vertex-coloured triangles so it can go through the same shader the
// car uses -- that program already does two-sided lighting and distance haze,
// which is exactly what a 1.3 km bowl needs.
//
// Coordinate mapping matches the rest of the renderer: world (x, y) becomes
// GL (x, height, -y).

// Palette in the renderer's display space, NOT linear. The rest of this
// renderer works the same way -- its procedural asphalt is 0.30, not 0.03 --
// so converting these to linear first renders the whole venue near-black.
const GRASS = [0.306, 0.392, 0.212];
const GRASS_2 = [0.345, 0.439, 0.235];
const ASPHALT = [0.243, 0.251, 0.263];
const ASPHALT_DK = [0.208, 0.216, 0.229];
const CONCRETE = [0.560, 0.560, 0.540];
const WALL = [0.640, 0.640, 0.620];
const WALL_TOP = [0.780, 0.780, 0.760];
const FENCE = [0.400, 0.420, 0.440];
const PADDOCK = [0.298, 0.306, 0.322];
const LINE = [0.880, 0.870, 0.810];

class Builder {
  constructor() { this.p = []; this.n = []; this.c = []; }

  tri(a, b, c, colour) {
    const ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
    const vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
    let nx = uy * vz - uz * vy;
    let ny = uz * vx - ux * vz;
    let nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1;
    nx /= l; ny /= l; nz /= l;
    for (const v of [a, b, c]) {
      this.p.push(v[0], v[1], v[2]);
      this.n.push(nx, ny, nz);
      this.c.push(colour[0], colour[1], colour[2]);
    }
  }

  quad(a, b, c, d, colour) {
    this.tri(a, b, c, colour);
    this.tri(a, c, d, colour);
  }

  /** Close a strip between two rings of equal length. */
  strip(inner, outer, colourAt) {
    const n = inner.length;
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      this.quad(inner[i], outer[i], outer[j], inner[j], colourAt(i));
    }
  }

  mesh() {
    return {
      position: new Float32Array(this.p),
      normal: new Float32Array(this.n),
      color: new Float32Array(this.c),
      count: this.p.length / 3,
    };
  }
}

/** World (x, y, height) -> GL. */
const w2g = (x, y, h) => [x, h, -y];

/** A ring of [x, y] at a constant height. */
const lift = (ring, h) => ring.map(([x, y]) => w2g(x, y, h));

export function buildVenueMesh(venue) {
  const b = new Builder();
  const R = venue.rings;

  // ---- infield grass -------------------------------------------------------
  // The infield is a stadium, so it is convex and a fan from the centre is
  // valid. Alternate the colour by fan wedge to suggest mowing stripes.
  const inner = lift(R.roadInner, 0.0);
  const cx = R.roadInner.reduce((a, p) => a + p[0], 0) / R.roadInner.length;
  const cy = R.roadInner.reduce((a, p) => a + p[1], 0) / R.roadInner.length;
  const hub = w2g(cx, cy, 0.0);
  for (let i = 0; i < inner.length; i++) {
    const j = (i + 1) % inner.length;
    // Stripe by world x so the bands run across the infield, not radially.
    const stripe = Math.floor(R.roadInner[i][0] / 26) % 2 === 0;
    b.tri(hub, inner[i], inner[j], stripe ? GRASS : GRASS_2);
  }

  // ---- paved paddock -------------------------------------------------------
  const p = venue.paddock;
  b.quad(
    w2g(-p.halfX, -p.halfY, 0.012), w2g(p.halfX, -p.halfY, 0.012),
    w2g(p.halfX, p.halfY, 0.012), w2g(-p.halfX, p.halfY, 0.012),
    PADDOCK,
  );

  // ---- infield access road -------------------------------------------------
  b.strip(lift(R.roadInner, 0.014), lift(R.roadOuter, 0.014), () => ASPHALT);

  // Grass shoulder between the road and the apron.
  b.strip(lift(R.roadOuter, 0.010), lift(R.apronInner, 0.010),
          (i) => (Math.floor(R.roadOuter[i][0] / 26) % 2 === 0 ? GRASS : GRASS_2));

  // ---- apron: flat concrete, driveable ------------------------------------
  b.strip(lift(R.apronInner, 0.016), lift(R.bankInner, 0.016), () => CONCRETE);

  // ---- the banking ---------------------------------------------------------
  // Rises from the foot to the top by the local bank angle. This is the part
  // the car is barred from, but it is most of what you see.
  const footRing = lift(R.bankInner, 0.018);
  const topRing = R.bankOuter.map(([x, y], i) => w2g(x, y, venue.riseM[i] + 0.018));
  b.strip(footRing, topRing, () => ASPHALT_DK);

  // A painted line at the foot of the banking, where the barrier is.
  const lineOut = R.bankInner.map(([x, y], i) => {
    const h = venue.heading[i];
    return w2g(x + Math.sin(h) * 0.15, y - Math.cos(h) * 0.15, 0.020);
  });
  b.strip(lift(R.bankInner, 0.020), lineOut, () => LINE);

  // ---- wall on top of the banking -----------------------------------------
  const wallBase = R.wallLine.map(([x, y], i) => w2g(x, y, venue.riseM[i] + 0.018));
  const wallTop = R.wallLine.map(([x, y], i) =>
    w2g(x, y, venue.riseM[i] + 0.018 + venue.wallHeight));
  // Inner face, seen from the track.
  b.strip(topRing, wallBase, () => ASPHALT_DK);
  b.strip(wallBase, wallTop, () => WALL);
  // Cap, so the top reads as a solid edge rather than a paper wall.
  const capOut = R.wallLine.map(([x, y], i) => {
    const h = venue.heading[i];
    return w2g(x + Math.sin(h) * 0.45, y - Math.cos(h) * 0.45,
               venue.riseM[i] + 0.018 + venue.wallHeight);
  });
  b.strip(wallTop, capOut, () => WALL_TOP);

  // ---- catchfence ----------------------------------------------------------
  // Posts and a top rail rather than a mesh sheet: there is no alpha blending
  // in this renderer, and a solid panel would wall the sky off completely.
  const step = 6;
  for (let i = 0; i < R.wallLine.length; i += step) {
    const [x, y] = R.wallLine[i];
    const h = venue.heading[i];
    const base = venue.riseM[i] + 0.018 + venue.wallHeight;
    const top = base + venue.fenceHeight;
    const t = 0.10;
    const ax = x - Math.sin(h) * t, ay = y + Math.cos(h) * t;
    const bx = x + Math.sin(h) * t, by = y - Math.cos(h) * t;
    b.quad(w2g(ax, ay, base), w2g(bx, by, base),
           w2g(bx, by, top), w2g(ax, ay, top), FENCE);
  }
  // Top rail.
  const railLo = R.wallLine.map(([x, y], i) =>
    w2g(x, y, venue.riseM[i] + 0.018 + venue.wallHeight + venue.fenceHeight - 0.25));
  const railHi = R.wallLine.map(([x, y], i) =>
    w2g(x, y, venue.riseM[i] + 0.018 + venue.wallHeight + venue.fenceHeight));
  b.strip(railLo, railHi, () => FENCE);

  return b.mesh();
}
