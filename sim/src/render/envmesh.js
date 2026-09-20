// Distant environment: woods, a tree line on the horizon, a couple of low
// hills, a grandstand and a row of paddock buildings around the course so the
// lot sits somewhere rather than floating in a void.
//
// FSAE Michigan runs on the paddock lot at Michigan International Speedway,
// so the grandstand on one side is not decoration -- it is what you actually
// see from the autocross grid. Everything is low-poly and vertex coloured,
// built once per track and drawn in a single call through the lit static
// shader (CAR_FS with a matte material); the fog does the rest.
//
// Colours are authored in DISPLAY space like every other mesh; the shader
// decodes them to linear before lighting. A mid green is 0.25-0.35 here, not
// 0.6. Faces are wound to agree with their normals (the Builder enforces it)
// because the renderer draws this mesh with back-face culling on and negates
// the normal on back faces, so a face wound the wrong way is either culled or
// lit from inside.
//
// Layout around the course: everything is placed on an offset loop around
// the course bounding box (a rounded rectangle at a constant clearance), so a
// thin autocross lot and a wide endurance lot both get scenery just off the
// pavement rather than an ellipse that cuts across the corners of the lot.
//
// Coordinates: world (x, y) -> GL (x, height, -y), like everything else.

// ---- palette (display space) ----------------------------------------------
const TRUNK = [0.24, 0.18, 0.125];
const BROADLEAF = [
  [0.25, 0.31, 0.15],   // olive
  [0.21, 0.31, 0.16],
  [0.17, 0.27, 0.19],   // blue-green
  [0.23, 0.33, 0.17],
  [0.16, 0.25, 0.15],   // dark
  [0.28, 0.32, 0.17],   // yellow olive
];
const CONIFER = [
  [0.15, 0.25, 0.17],
  [0.13, 0.22, 0.16],
  [0.18, 0.27, 0.16],
  [0.16, 0.26, 0.21],
];
const FAR_TREES = [0.13, 0.18, 0.13];
const FAR_TREES_2 = [0.15, 0.20, 0.15];
const HILL = [0.19, 0.25, 0.15];

const STAND_SEATS = [
  [0.30, 0.37, 0.52],   // blue-grey (most)
  [0.42, 0.44, 0.48],   // grey
  [0.25, 0.35, 0.60],   // MIS blue
];
const STAND_TEAM = [[0.48, 0.12, 0.20], [0.78, 0.60, 0.18]];  // maroon, gold
const STAND_AISLE = [0.50, 0.50, 0.51];
const STAND_RISER = [0.38, 0.39, 0.41];
const STAND_STRUCT = [0.22, 0.22, 0.24];
const STAND_WALL = [0.50, 0.49, 0.48];
const STAND_ROOF = [0.74, 0.75, 0.77];
const STAND_ROOF_UNDER = [0.42, 0.43, 0.46];
const STAND_FASCIA = [0.82, 0.82, 0.84];
const STAND_POST = [0.40, 0.41, 0.43];

const WALLS = [
  [0.62, 0.60, 0.56],   // beige block
  [0.66, 0.66, 0.65],   // light grey
  [0.60, 0.55, 0.48],   // tan
  [0.72, 0.71, 0.68],   // off-white cladding
];
const BASE_COURSE = [0.36, 0.35, 0.34];
const ROOF = [0.40, 0.40, 0.42];
const PARAPET = [0.56, 0.56, 0.57];
const METAL_ROOF = [0.34, 0.36, 0.40];
const GLASS = [0.10, 0.13, 0.17];
const FRAME = [0.30, 0.30, 0.32];
const ROLLUP = [0.52, 0.53, 0.54];
const DOOR = [0.26, 0.28, 0.32];
const UNIT = [0.56, 0.57, 0.58];
const UNIT_TOP = [0.48, 0.49, 0.50];
const CANOPY = [0.44, 0.45, 0.47];

// ---- small maths -----------------------------------------------------------
function faceNormal(a, b, c) {
  const ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
  const vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
  const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
  const l = Math.hypot(nx, ny, nz);
  return l > 0 ? [nx / l, ny / l, nz / l] : [0, 0, 0];
}
function norm3(x, y, z) { const l = Math.hypot(x, y, z) || 1; return [x / l, y / l, z / l]; }
function scale(c, k) { return [c[0] * k, c[1] * k, c[2] * k]; }
function smoothstep(e0, e1, x) {
  const t = Math.min(1, Math.max(0, (x - e0) / (e1 - e0)));
  return t * t * (3 - 2 * t);
}

/** Small deterministic PRNG so the scenery is the same every launch. */
function rng(seed) {
  let s = seed >>> 0 || 1;
  return () => {
    s ^= s << 13; s >>>= 0;
    s ^= s >>> 17;
    s ^= s << 5; s >>>= 0;
    return (s >>> 0) / 4294967296;
  };
}

// ---- builder ----------------------------------------------------------------
class Builder {
  constructor() { this.p = []; this.n = []; this.c = []; this.xf = null; }

  /** Everything emitted until `unplace()` is rotated about y by `ang` and moved to (x, z). */
  place(x, z, ang) { this.xf = { x, z, c: Math.cos(ang), s: Math.sin(ang) }; }
  unplace() { this.xf = null; }

  emit(v, n, col) {
    let x = v[0], y = v[1], z = v[2], nx = n[0], ny = n[1], nz = n[2];
    const t = this.xf;
    if (t) {
      const rx = x * t.c + z * t.s, rz = -x * t.s + z * t.c;
      x = rx + t.x; z = rz + t.z;
      const rnx = nx * t.c + nz * t.s, rnz = -nx * t.s + nz * t.c;
      nx = rnx; nz = rnz;
    }
    this.p.push(x, y, z);
    this.n.push(nx, ny, nz);
    this.c.push(col[0], col[1], col[2]);
  }

  /** Flat triangle. The winding is made to agree with the normal. */
  tri(a, b, c, colour, normal) {
    const fn = faceNormal(a, b, c);
    const n = normal ?? fn;
    if (fn[0] * n[0] + fn[1] * n[1] + fn[2] * n[2] < 0) { const t = b; b = c; c = t; }
    this.emit(a, n, colour); this.emit(b, n, colour); this.emit(c, n, colour);
  }

  /** Smooth triangle: a normal and a colour per vertex, wound to agree with the mean normal. */
  triS(a, b, c, na, nb, nc, ca, cb = ca, cc = ca) {
    const fn = faceNormal(a, b, c);
    const d = fn[0] * (na[0] + nb[0] + nc[0]) + fn[1] * (na[1] + nb[1] + nc[1]) + fn[2] * (na[2] + nb[2] + nc[2]);
    if (d < 0) {
      let t = b; b = c; c = t;
      t = nb; nb = nc; nc = t;
      t = cb; cb = cc; cc = t;
    }
    this.emit(a, na, ca); this.emit(b, nb, cb); this.emit(c, nc, cc);
  }

  quad(a, b, c, d, colour, normal) {
    const n = normal ?? faceNormal(a, b, c);
    this.tri(a, b, c, colour, n);
    this.tri(a, c, d, colour, n);
  }

  /** Axis-aligned box, base at y0. No bottom face unless a colour is given for it. */
  box(cx, y0, cz, w, h, d, colour, top = colour, bottom = null) {
    const x0 = cx - w / 2, x1 = cx + w / 2, z0 = cz - d / 2, z1 = cz + d / 2, y1 = y0 + h;
    this.quad([x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1], colour, [0, 0, 1]);
    this.quad([x1, y0, z0], [x0, y0, z0], [x0, y1, z0], [x1, y1, z0], colour, [0, 0, -1]);
    this.quad([x1, y0, z1], [x1, y0, z0], [x1, y1, z0], [x1, y1, z1], colour, [1, 0, 0]);
    this.quad([x0, y0, z0], [x0, y0, z1], [x0, y1, z1], [x0, y1, z0], colour, [-1, 0, 0]);
    this.quad([x0, y1, z1], [x1, y1, z1], [x1, y1, z0], [x0, y1, z0], top, [0, 1, 0]);
    if (bottom) this.quad([x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1], bottom, [0, -1, 0]);
  }

  /**
   * A cone with smooth sides and a flat, darker base disc, base at y0. The
   * disc is what you see when a conifer tier is above eye level; without it
   * the culled inside of the cone would leave a hole.
   */
  cone(cx, y0, cz, r, h, segs, phase, colour, under) {
    const slope = r / h;
    const apex = [cx, y0 + h, cz], apexN = [0, 1, 0];
    for (let i = 0; i < segs; i++) {
      const a0 = phase + (i / segs) * Math.PI * 2, a1 = phase + ((i + 1) / segs) * Math.PI * 2;
      const p0 = [cx + Math.cos(a0) * r, y0, cz + Math.sin(a0) * r];
      const p1 = [cx + Math.cos(a1) * r, y0, cz + Math.sin(a1) * r];
      const n0 = norm3(Math.cos(a0), slope, Math.sin(a0));
      const n1 = norm3(Math.cos(a1), slope, Math.sin(a1));
      this.triS(p0, apex, p1, n0, apexN, n1, colour);
      this.tri(p0, p1, [cx, y0, cz], under, [0, -1, 0]);
    }
  }

  /** Ellipsoid with analytic normals; `colourAt(ny)` shades by the normal's y. */
  ellipsoid(cx, cy, cz, rx, ry, rz, lon, lat, phase, colourAt) {
    const P = (t, p) => [cx + rx * Math.cos(t) * Math.cos(p), cy + ry * Math.sin(t), cz + rz * Math.cos(t) * Math.sin(p)];
    const N = (t, p) => norm3(Math.cos(t) * Math.cos(p) / rx, Math.sin(t) / ry, Math.cos(t) * Math.sin(p) / rz);
    for (let j = 0; j < lat; j++) {
      const t0 = -Math.PI / 2 + (j / lat) * Math.PI, t1 = -Math.PI / 2 + ((j + 1) / lat) * Math.PI;
      for (let i = 0; i < lon; i++) {
        const p0 = phase + (i / lon) * Math.PI * 2, p1 = phase + ((i + 1) / lon) * Math.PI * 2;
        const v00 = P(t0, p0), v10 = P(t0, p1), v11 = P(t1, p1), v01 = P(t1, p0);
        const n00 = N(t0, p0), n10 = N(t0, p1), n11 = N(t1, p1), n01 = N(t1, p0);
        if (j > 0) this.triS(v00, v10, v11, n00, n10, n11, colourAt(n00[1]), colourAt(n10[1]), colourAt(n11[1]));
        if (j < lat - 1) this.triS(v00, v11, v01, n00, n11, n01, colourAt(n00[1]), colourAt(n11[1]), colourAt(n01[1]));
      }
    }
  }

  /** Stacked frustums around a vertical axis: profile is [[y, r], ...] ascending in y. */
  frustums(x, z, profile, sides, phase, colourAt) {
    for (let k = 0; k + 1 < profile.length; k++) {
      const [y0, r0] = profile[k], [y1, r1] = profile[k + 1];
      const tilt = (r0 - r1) / Math.max(y1 - y0, 1e-3);
      const c0 = colourAt(y0), c1 = colourAt(y1);
      for (let i = 0; i < sides; i++) {
        const a0 = phase + (i / sides) * Math.PI * 2, a1 = phase + ((i + 1) / sides) * Math.PI * 2;
        const n0 = norm3(Math.cos(a0), tilt, Math.sin(a0)), n1 = norm3(Math.cos(a1), tilt, Math.sin(a1));
        const b0 = [x + Math.cos(a0) * r0, y0, z + Math.sin(a0) * r0];
        const b1 = [x + Math.cos(a1) * r0, y0, z + Math.sin(a1) * r0];
        const t0 = [x + Math.cos(a0) * r1, y1, z + Math.sin(a0) * r1];
        const t1 = [x + Math.cos(a1) * r1, y1, z + Math.sin(a1) * r1];
        this.triS(b0, b1, t1, n0, n1, n1, c0, c0, c1);
        this.triS(b0, t1, t0, n0, n1, n0, c0, c1, c1);
      }
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

// ---- the offset loop --------------------------------------------------------
// A rounded rectangle at clearance D around a box of half-extents (hx, hy),
// parametrised by arc length s from the south-west corner, counter-clockwise.
// Returns [x, y, nx, ny] relative to the box centre, with the outward normal.
function loopLength(hx, hy, D) { return 4 * hx + 4 * hy + 2 * Math.PI * D; }

function loopPoint(hx, hy, D, s) {
  const total = loopLength(hx, hy, D), arc = Math.PI * D / 2;
  s = ((s % total) + total) % total;
  const arcPt = (ax, ay, phi) => [ax + D * Math.cos(phi), ay + D * Math.sin(phi), Math.cos(phi), Math.sin(phi)];
  if (s < 2 * hx) return [-hx + s, -hy - D, 0, -1]; s -= 2 * hx;
  if (s < arc) return arcPt(hx, -hy, -Math.PI / 2 + s / D); s -= arc;
  if (s < 2 * hy) return [hx + D, -hy + s, 1, 0]; s -= 2 * hy;
  if (s < arc) return arcPt(hx, hy, s / D); s -= arc;
  if (s < 2 * hx) return [hx - s, hy + D, 0, 1]; s -= 2 * hx;
  if (s < arc) return arcPt(-hx, hy, Math.PI / 2 + s / D); s -= arc;
  if (s < 2 * hy) return [-hx - D, hy - s, -1, 0]; s -= 2 * hy;
  return arcPt(-hx, -hy, Math.PI + Math.min(s, arc) / D);
}

// ---- trees ------------------------------------------------------------------
function trunk(b, x, z, h, r, sides, phase) {
  const flare = Math.min(0.5, h * 0.2);
  const profile = [[0, r * 1.8], [flare, r * 1.15], [h * 0.45, r * 0.95], [h, r * 0.6]];
  // Darker at the foot, where the bark is wet and shaded.
  b.frustums(x, z, profile, sides, phase, (y) => scale(TRUNK, 0.7 + 0.3 * smoothstep(0, h * 0.5, y)));
}

/** Conifer: 3-4 offset canopy tiers of decreasing radius with a slight lean. */
function conifer(b, x, z, h, rand) {
  const c = scale(CONIFER[Math.floor(rand() * CONIFER.length)], 0.88 + rand() * 0.24);
  const under = scale(c, 0.5);
  const tiers = 3 + (rand() < 0.5 ? 1 : 0);
  const leanA = rand() * Math.PI * 2, lean = rand() * 0.05;
  const r0 = h * (0.17 + rand() * 0.06);
  trunk(b, x, z, h * 0.45, h * 0.02 + 0.06, 7, rand() * 6.28);
  let y = h * 0.16;
  for (let t = 0; t < tiers; t++) {
    const f = 1 - t / tiers;
    const r = r0 * (0.4 + 0.6 * f) * (0.9 + 0.2 * rand());
    const th = h * (0.34 + 0.06 * rand()) * (0.7 + 0.3 * f);
    const lx = Math.cos(leanA) * lean * y + (rand() - 0.5) * r * 0.3;
    const lz = Math.sin(leanA) * lean * y + (rand() - 0.5) * r * 0.3;
    b.cone(x + lx, y, z + lz, r, th, 9, rand() * 6.28, c, under);
    y += th * (0.42 + 0.1 * rand());
  }
}

/** Broadleaf: a lumpy crown of 5-8 overlapping ellipsoids on a trunk. */
function broadleaf(b, x, z, h, rand) {
  const c = scale(BROADLEAF[Math.floor(rand() * BROADLEAF.length)], 0.9 + rand() * 0.2);
  const R = h * (0.30 + rand() * 0.12);
  const yC = h - R;
  trunk(b, x, z, yC + R * 0.2, h * 0.026 + 0.08, 8, rand() * 6.28);
  const n = 5 + Math.floor(rand() * 4);
  for (let k = 0; k < n; k++) {
    const big = k === 0;
    const s = big ? 0.85 : 0.45 + rand() * 0.3;
    const dir = rand() * Math.PI * 2, dist = big ? 0 : R * (0.35 + rand() * 0.4);
    const dy = big ? 0 : (rand() - 0.35) * R * 0.7;
    const cc = scale(c, 0.92 + rand() * 0.16);
    b.ellipsoid(
      x + Math.cos(dir) * dist, yC + dy, z + Math.sin(dir) * dist,
      R * s * (0.9 + 0.2 * rand()), R * s * (0.75 + 0.2 * rand()), R * s * (0.9 + 0.2 * rand()),
      8, 4, rand() * 6.28,
      // Darker underside so the crown rounds even in flat light.
      (ny) => scale(cc, 0.55 + 0.45 * smoothstep(-0.6, 0.5, ny)),
    );
  }
}

function tree(b, x, z, h, rand, coniferBias = 0.5) {
  if (rand() < coniferBias) conifer(b, x, z, h, rand);
  else broadleaf(b, x, z, h, rand);
}

// ---- horizon ----------------------------------------------------------------
/**
 * A ring of jagged tree-silhouette strip around the loop at clearance D,
 * facing inward. Heights come from a few sine terms that divide the loop's
 * length, so the ring closes without a seam, plus per-vertex jitter.
 */
function horizonBand(b, cx, cy, hx, hy, D, spacing, colour, hScale, rand) {
  const total = loopLength(hx, hy, D);
  const n = Math.max(16, Math.round(total / spacing));
  const k1 = Math.max(1, Math.round(total / 160));
  const k2 = Math.max(1, Math.round(total / 48));
  const k3 = Math.max(1, Math.round(total / 17));
  const ph = [rand() * 6.28, rand() * 6.28, rand() * 6.28];
  const pts = [];
  for (let i = 0; i < n; i++) {
    const s = (i / n) * total, t = (s / total) * Math.PI * 2;
    let h = 8 + 3.2 * Math.sin(k1 * t + ph[0]) + 2.2 * Math.sin(k2 * t + ph[1]) + 1.6 * Math.sin(k3 * t + ph[2]) + rand() * 1.8;
    h = Math.max(2.5, h) * hScale;
    const [px, py, nx, ny] = loopPoint(hx, hy, D, s);
    pts.push([cx + px, cy + py, h, nx, ny]);
  }
  for (let i = 0; i < n; i++) {
    const p = pts[i], q = pts[(i + 1) % n];
    // Inward in world is (-nx, -ny); GL z is -world y.
    const nrm = norm3(-(p[3] + q[3]) / 2, 0.35, (p[4] + q[4]) / 2);
    b.quad([p[0], -0.6, -p[1]], [q[0], -0.6, -q[1]], [q[0], q[2], -q[1]], [p[0], p[2], -p[1]], colour, nrm);
  }
}

/** A shallow ridge centred at world (wx, wy), axis at `ang`, length L, width W, height H. */
function hill(b, wx, wy, ang, L, W, H, rand) {
  const nu = 22, nv = 8, ph = rand() * 6.28;
  const ca = Math.cos(ang), sa = Math.sin(ang);
  const P = (u, v) => {
    const env = Math.pow(Math.max(0, Math.sin(Math.PI * u)), 1.4) * (0.82 + 0.18 * Math.sin(u * 11 + ph));
    const bump = Math.pow(Math.max(0, Math.cos(v * Math.PI / 2)), 1.6);
    const h = H * env * bump - 0.5;
    const lx = (u - 0.5) * L, ly = v * W / 2;
    return [wx + lx * ca - ly * sa, h, -(wy + lx * sa + ly * ca)];
  };
  const N = (u, v) => {
    const e = 1e-3, p = P(u, v), pu = P(u + e, v), pv = P(u, v + e);
    const tu = [pu[0] - p[0], pu[1] - p[1], pu[2] - p[2]], tv = [pv[0] - p[0], pv[1] - p[1], pv[2] - p[2]];
    let nx = tu[1] * tv[2] - tu[2] * tv[1], ny = tu[2] * tv[0] - tu[0] * tv[2], nz = tu[0] * tv[1] - tu[1] * tv[0];
    if (ny < 0) { nx = -nx; ny = -ny; nz = -nz; }
    return norm3(nx, ny, nz);
  };
  const C = (p) => scale(HILL, 0.85 + 0.35 * Math.min(1, Math.max(0, p[1] / H)));
  for (let i = 0; i < nu; i++) {
    for (let j = 0; j < nv; j++) {
      const u0 = i / nu, u1 = (i + 1) / nu, v0 = -1 + 2 * j / nv, v1 = -1 + 2 * (j + 1) / nv;
      const a = P(u0, v0), bb = P(u1, v0), c = P(u1, v1), d = P(u0, v1);
      const na = N(u0, v0), nb = N(u1, v0), nc = N(u1, v1), nd = N(u0, v1);
      b.triS(a, bb, c, na, nb, nc, C(a), C(bb), C(c));
      b.triS(a, c, d, na, nc, nd, C(a), C(c), C(d));
    }
  }
}

// ---- grandstand -------------------------------------------------------------
/**
 * Stepped seating bank in a local frame: along x, the lowest row at z = 0
 * facing +z, rows stepping back toward -z as they rise. Roof on posts with a
 * fascia, a back wall, and a dark enclosed under-structure. The caller
 * `place()`s it.
 */
function grandstand(b, L, rand) {
  const rows = 11, tread = 1.0, rise = 0.72, yF = 2.4, walk = 2.4;
  const zTop = -rows * tread, zBack = zTop - walk;
  const yTop = yF + rows * rise, yWall = yTop + 2.6, wallT = 0.3;
  const x0 = -L / 2, x1 = L / 2;

  // Front (concourse) wall.
  b.quad([x0, 0, 0], [x1, 0, 0], [x1, yF, 0], [x0, yF, 0], STAND_STRUCT, [0, 0, 1]);

  // Seat sections: mostly blue-grey and grey, an aisle every eighth, team
  // colours in a few blocks only.
  const nSec = Math.max(1, Math.round(L / 4.5)), sw = L / nSec;
  const secCol = [];
  for (let s = 0; s < nSec; s++) {
    if (s % 8 === 4) { secCol.push(null); continue; }
    const r = rand();
    if (r < 0.05) secCol.push(STAND_TEAM[0]);
    else if (r < 0.09) secCol.push(STAND_TEAM[1]);
    else if (r < 0.55) secCol.push(STAND_SEATS[0]);
    else if (r < 0.82) secCol.push(STAND_SEATS[1]);
    else secCol.push(STAND_SEATS[2]);
  }
  // From the lot you see the seat backs (the risers), not the treads, so
  // the section colour goes on the riser; the tread is the walkway.
  for (let i = 0; i < rows; i++) {
    const y0 = yF + i * rise, y1 = y0 + rise, zf = -i * tread, zb = zf - tread;
    b.quad([x0, y1, zf], [x1, y1, zf], [x1, y1, zb], [x0, y1, zb], STAND_RISER, [0, 1, 0]);
    for (let s = 0; s < nSec; s++) {
      const sx0 = x0 + s * sw, sx1 = sx0 + sw;
      const col = secCol[s] ?? STAND_AISLE;
      const tone = 0.95 + 0.1 * ((i + s) % 2);
      b.quad([sx0, y0, zf], [sx1, y0, zf], [sx1, y1, zf], [sx0, y1, zf], scale(col, tone), [0, 0, 1]);
    }
  }
  // Top walkway, and the back wall (a slab, so its top reads from the front).
  b.quad([x0, yTop, zTop], [x1, yTop, zTop], [x1, yTop, zBack + wallT], [x0, yTop, zBack + wallT], STAND_RISER, [0, 1, 0]);
  b.quad([x0, 0, zBack], [x1, 0, zBack], [x1, yWall, zBack], [x0, yWall, zBack], STAND_WALL, [0, 0, -1]);
  b.quad([x0, yTop, zBack + wallT], [x1, yTop, zBack + wallT], [x1, yWall, zBack + wallT], [x0, yWall, zBack + wallT],
    scale(STAND_WALL, 0.85), [0, 0, 1]);
  b.quad([x0, yWall, zBack], [x1, yWall, zBack], [x1, yWall, zBack + wallT], [x0, yWall, zBack + wallT], STAND_WALL, [0, 1, 0]);

  // End walls: a fan over the stepped profile from the back-bottom corner,
  // which sees every point of the staircase.
  for (const s of [-1, 1]) {
    const x = s * L / 2;
    const prof = [[0, 0], [yF, 0]];
    for (let i = 0; i < rows; i++) {
      const y1 = yF + (i + 1) * rise;
      prof.push([y1, -i * tread], [y1, -(i + 1) * tread]);
    }
    prof.push([yTop, zBack + wallT], [yWall, zBack + wallT], [yWall, zBack]);
    const p0 = [x, 0, zBack];
    for (let k = 0; k + 1 < prof.length; k++) {
      b.tri(p0, [x, prof[k][0], prof[k][1]], [x, prof[k + 1][0], prof[k + 1][1]], STAND_STRUCT, [s, 0, 0]);
    }
  }

  // Roof: a slab higher at the front, overhanging the first row, with a
  // fascia along the leading edge.
  const zF = 2.2, zR = zBack - 0.5, yFr = yTop + 5.4, yRr = yTop + 4.0, T = 0.4;
  const top = [[x0, yFr, zF], [x1, yFr, zF], [x1, yRr, zR], [x0, yRr, zR]];
  const nTop = faceNormal(top[0], top[1], top[2]);
  const nUnder = [-nTop[0], -nTop[1], -nTop[2]];
  b.quad(top[0], top[1], top[2], top[3], STAND_ROOF, nTop);
  b.quad([x0, yFr - T, zF], [x1, yFr - T, zF], [x1, yRr - T, zR], [x0, yRr - T, zR], STAND_ROOF_UNDER, nUnder);
  b.quad([x0, yFr - T - 0.7, zF], [x1, yFr - T - 0.7, zF], [x1, yFr, zF], [x0, yFr, zF], STAND_FASCIA, [0, 0, 1]);
  b.quad([x0, yFr - T - 0.7, zF + 0.02], [x1, yFr - T - 0.7, zF + 0.02], [x1, yFr - T - 0.45, zF + 0.02],
    [x0, yFr - T - 0.45, zF + 0.02], STAND_TEAM[0], [0, 0, 1]);
  b.quad([x0, yRr - T, zR], [x1, yRr - T, zR], [x1, yRr, zR], [x0, yRr, zR], STAND_FASCIA, [0, 0, -1]);
  for (const s of [-1, 1]) {
    const x = s * L / 2;
    b.quad([x, yFr, zF], [x, yRr, zR], [x, yRr - T, zR], [x, yFr - T, zF], STAND_FASCIA, [s, 0, 0]);
  }
  // Posts behind the top row, and the beam they carry.
  const nPosts = Math.max(4, Math.min(6, Math.round(L / 40)));
  const zP = zBack + 1.4;
  const yBeam = yFr - T + (yRr - yFr) * (zP - zF) / (zR - zF);
  b.box(0, yBeam - 0.6, zP, L, 0.6, 0.6, STAND_POST, STAND_POST, STAND_STRUCT);
  for (let k = 0; k < nPosts; k++) {
    const px = x0 + (k + 0.5) * L / nPosts;
    b.box(px, 0, zP, 0.7, yBeam - 0.6, 0.7, STAND_POST);
  }
}

// ---- buildings --------------------------------------------------------------
// Local frame: footprint w (x) by d (z), the front face at z = +d/2 faces +z
// toward the lot. The caller `place()`s the building with its front toward
// the course.

/** Front wall with openings: reveals and a recessed panel (glass or door). */
function facade(b, w, h, zf, colour, openings) {
  const n = [0, 0, 1];
  // Wall: split into the elementary x-intervals between every opening edge,
  // and fill each interval between the openings that cover it (windows on
  // two storeys share a column, so a per-opening above/below split would
  // paper over the other storey's window).
  const xs = [...new Set([-w / 2, w / 2, ...openings.flatMap((o) => [o.x0, o.x1])])].sort((p, q) => p - q);
  for (let i = 0; i + 1 < xs.length; i++) {
    const xa = xs[i], xb = xs[i + 1];
    if (xb - xa < 1e-6) continue;
    const xm = (xa + xb) / 2;
    const cover = openings.filter((o) => o.x0 <= xm && o.x1 >= xm).sort((p, q) => p.y0 - q.y0);
    let y = 0;
    for (const o of cover) {
      if (o.y0 > y) b.quad([xa, y, zf], [xb, y, zf], [xb, o.y0, zf], [xa, o.y0, zf], colour, n);
      y = Math.max(y, o.y1);
    }
    if (y < h) b.quad([xa, y, zf], [xb, y, zf], [xb, h, zf], [xa, h, zf], colour, n);
  }
  // Reveals and the recessed panel of each opening.
  for (const o of openings) {
    const zb = zf - o.depth;
    const fr = o.frame ?? FRAME;
    if (o.y0 > 0) b.quad([o.x0, o.y0, zf], [o.x1, o.y0, zf], [o.x1, o.y0, zb], [o.x0, o.y0, zb], fr, [0, 1, 0]);
    b.quad([o.x0, o.y1, zf], [o.x1, o.y1, zf], [o.x1, o.y1, zb], [o.x0, o.y1, zb], fr, [0, -1, 0]);
    b.quad([o.x0, o.y0, zf], [o.x0, o.y1, zf], [o.x0, o.y1, zb], [o.x0, o.y0, zb], fr, [1, 0, 0]);
    b.quad([o.x1, o.y0, zf], [o.x1, o.y1, zf], [o.x1, o.y1, zb], [o.x1, o.y0, zb], fr, [-1, 0, 0]);
    b.quad([o.x0, o.y0, zb], [o.x1, o.y0, zb], [o.x1, o.y1, zb], [o.x0, o.y1, zb], o.colour, n);
  }
}

/** Back, sides and (optionally) the flat roof of a rectangular shell. */
function shell(b, w, h, d, colour, roof) {
  const x0 = -w / 2, x1 = w / 2, z0 = -d / 2, z1 = d / 2;
  b.quad([x1, 0, z0], [x0, 0, z0], [x0, h, z0], [x1, h, z0], colour, [0, 0, -1]);
  b.quad([x1, 0, z1], [x1, 0, z0], [x1, h, z0], [x1, h, z1], colour, [1, 0, 0]);
  b.quad([x0, 0, z0], [x0, 0, z1], [x0, h, z1], [x0, h, z0], colour, [-1, 0, 0]);
  if (roof) b.quad([x0, h, z1], [x1, h, z1], [x1, h, z0], [x0, h, z0], roof, [0, 1, 0]);
}

function baseCourse(b, w, d) { b.box(0, 0, 0, w + 0.16, 0.7, d + 0.16, BASE_COURSE); }

function parapet(b, w, h, d) {
  const t = 0.36, ph = 0.6;
  b.box(0, h, d / 2 - t / 2, w, ph, t, PARAPET, scale(PARAPET, 1.08));
  b.box(0, h, -d / 2 + t / 2, w, ph, t, PARAPET, scale(PARAPET, 1.08));
  b.box(w / 2 - t / 2, h, 0, t, ph, d - 2 * t, PARAPET, scale(PARAPET, 1.08));
  b.box(-w / 2 + t / 2, h, 0, t, ph, d - 2 * t, PARAPET, scale(PARAPET, 1.08));
}

function rooftop(b, w, h, d, rand) {
  const n = 1 + (rand() < 0.6 ? 1 : 0);
  for (let i = 0; i < n; i++) {
    const ux = (rand() - 0.5) * (w - 6), uz = (rand() - 0.5) * (d - 5);
    b.box(ux, h, uz, 2.4, 1.3, 1.9, UNIT, UNIT_TOP);
    b.box(ux + 0.4, h + 1.3, uz - 0.2, 0.9, 0.35, 0.9, UNIT_TOP, scale(UNIT_TOP, 0.8));
  }
  b.box((rand() - 0.5) * (w - 4), h, (rand() - 0.5) * (d - 4), 0.5, 1.7, 0.5, scale(UNIT, 0.8));
}

/** Garage / workshop: roll-up doors along the front, a people door, small windows. */
function garage(b, w, h, d, wall, rand) {
  shell(b, w, h, d, wall, ROOF);
  const ops = [];
  const doorH = Math.min(4.2, h - 1.6), bay = 8;
  for (let x = -w / 2 + 1.2; x + 4.4 < w / 2 - 3.2; x += bay) {
    ops.push({ x0: x, x1: x + 4.4, y0: 0, y1: doorH, depth: 0.3, colour: scale(ROLLUP, 0.9 + rand() * 0.2) });
    if (x + 7.4 < w / 2 - 3.2) ops.push({ x0: x + 5.6, x1: x + 7.0, y0: 2.1, y1: 3.2, depth: 0.25, colour: GLASS });
  }
  ops.push({ x0: w / 2 - 2.6, x1: w / 2 - 1.5, y0: 0, y1: 2.3, depth: 0.25, colour: DOOR });
  facade(b, w, h, d / 2, wall, ops);
  baseCourse(b, w, d);
  parapet(b, w, h, d);
  rooftop(b, w, h, d, rand);
}

/** Office block: rows of windows per storey, a glazed door under a canopy. */
function office(b, w, h, d, wall, rand) {
  shell(b, w, h, d, wall, ROOF);
  const ops = [];
  const storeys = h >= 7.2 ? 2 : 1, sh = h / storeys;
  for (let st = 0; st < storeys; st++) {
    const y0 = st * sh + 1.0, y1 = Math.min(y0 + 1.5, st * sh + sh - 0.6);
    for (let x = -w / 2 + 1.5; x + 1.5 < w / 2 - 1.0; x += 2.6) {
      if (st === 0 && Math.abs(x + 0.75) < 2.0) continue;   // the door slot
      ops.push({ x0: x, x1: x + 1.5, y0, y1, depth: 0.28, colour: GLASS });
    }
  }
  ops.push({ x0: -0.9, x1: 0.9, y0: 0, y1: 2.5, depth: 0.35, colour: scale(GLASS, 1.3) });
  facade(b, w, h, d / 2, wall, ops);
  b.box(0, 2.6, d / 2 + 0.7, 3.2, 0.2, 1.5, CANOPY, CANOPY, scale(CANOPY, 0.8));
  baseCourse(b, w, d);
  parapet(b, w, h, d);
  rooftop(b, w, h, d, rand);
}

/** Gabled shed: pitched metal roof with the ridge along x, roll-up doors on the eave side. */
function shed(b, w, h, d, wall, rand) {
  shell(b, w, h, d, wall, null);
  const ridge = h + d * 0.28;
  for (const s of [-1, 1]) {
    const x = s * w / 2;
    b.tri([x, h, -d / 2], [x, h, d / 2], [x, ridge, 0], wall, [s, 0, 0]);
  }
  const ops = [];
  const doorH = Math.min(3.8, h - 1.2);
  for (let x = -w / 2 + 1.5; x + 4.0 < w / 2 - 1.5; x += 7.5) {
    ops.push({ x0: x, x1: x + 4.0, y0: 0, y1: doorH, depth: 0.3, colour: scale(ROLLUP, 0.9 + rand() * 0.2) });
  }
  if (ops.length && ops[ops.length - 1].x1 + 3.0 < w / 2 - 1.0) {
    const x = ops[ops.length - 1].x1 + 1.2;
    ops.push({ x0: x, x1: x + 1.4, y0: 1.6, y1: 2.6, depth: 0.25, colour: GLASS });
  }
  facade(b, w, h, d / 2, wall, ops);
  // Roof slopes with an overhang, an underside and an eave fascia.
  const ov = 0.6, ox = 0.4, t = 0.18;
  const yE = h - (ridge - h) * ov / (d / 2);
  for (const s of [-1, 1]) {
    const ze = s * (d / 2 + ov);
    const a = [-w / 2 - ox, yE, ze], bb = [w / 2 + ox, yE, ze], c = [w / 2 + ox, ridge, 0], dd = [-w / 2 - ox, ridge, 0];
    const n = faceNormal(a, bb, c);
    const nn = n[1] < 0 ? [-n[0], -n[1], -n[2]] : n;
    b.quad(a, bb, c, dd, METAL_ROOF, nn);
    b.quad([a[0], yE - t, ze], [bb[0], yE - t, ze], [c[0], ridge - t, 0], [dd[0], ridge - t, 0],
      scale(METAL_ROOF, 0.7), [-nn[0], -nn[1], -nn[2]]);
    b.quad([a[0], yE - t, ze], [bb[0], yE - t, ze], [bb[0], yE, ze], [a[0], yE, ze], scale(wall, 1.1), [0, 0, s]);
  }
  baseCourse(b, w, d);
}

const SHAPES = [garage, office, shed];

/**
 * Build the scenery for a course or venue.
 * @param bounds {minX, maxX, minY, maxY} of the driveable geometry, world (x, y)
 * @param kind   "course" or "venue"
 */
export function buildEnvironmentMesh(bounds, kind = "course") {
  const b = new Builder();
  const venue = kind === "venue";
  const rand = rng(venue ? 7331 : 1337);
  const cx = (bounds.minX + bounds.maxX) / 2;
  const cy = (bounds.minY + bounds.maxY) / 2;
  const hx = (bounds.maxX - bounds.minX) / 2;
  const hy = (bounds.maxY - bounds.minY) / 2;

  // Footprints the trees must keep out of, world (x, y).
  const blocked = [];
  const isBlocked = (x, y) => blocked.some((r) => x > r.x0 && x < r.x1 && y > r.y0 && y < r.y1);

  // Facing is a GL xz direction the front should point along.
  const placeBuilding = (x, y, fx, fz, shape, w, h, d) => {
    const wall = scale(WALLS[Math.floor(rand() * WALLS.length)], 0.92 + rand() * 0.16);
    b.place(x, -y, Math.atan2(fx, fz));
    shape(b, w, h, d, wall, rand);
    b.unplace();
    const alongX = Math.abs(fx) < 0.5;
    const ex = (alongX ? w : d) / 2 + 6, ey = (alongX ? d : w) / 2 + 6;
    blocked.push({ x0: x - ex, x1: x + ex, y0: y - ey, y1: y + ey });
  };

  // ---- built things first, so the trees can keep clear of them -------------
  if (!venue) {
    // Grandstand on the north side, the way MIS sits behind the lot: just
    // past the paved apron (which is the course box + 110 m), facing south.
    const L = Math.min(260, Math.max(130, hx));
    b.place(cx, -(bounds.maxY + 128), 0);
    grandstand(b, L, rand);
    b.unplace();
    blocked.push({ x0: cx - L / 2 - 10, x1: cx + L / 2 + 10, y0: bounds.maxY + 110, y1: bounds.maxY + 150 });

    // Paddock buildings along the south edge of the apron, fronts to the lot.
    const nS = 5;
    for (let i = 0; i < nS; i++) {
      const x = bounds.minX - 10 + (i + 0.2 + rand() * 0.6) * ((hx * 2 + 20) / nS);
      const y = bounds.minY - (118 + rand() * 22);
      placeBuilding(x, y, 0, -1, SHAPES[i % 3], 22 + rand() * 28, 6 + rand() * 5, 14 + rand() * 9);
    }
    // A smaller group east of the lot, facing west.
    const pitch = Math.max(hy * 2 / 3, 34);
    for (let i = 0; i < 3; i++) {
      const x = bounds.maxX + 122 + rand() * 20;
      const y = cy + (i - 1) * pitch + (rand() - 0.5) * 8;
      placeBuilding(x, y, -1, 0, SHAPES[(i + 1) % 3], 16 + rand() * 18, 5 + rand() * 4, 12 + rand() * 6);
    }
  } else {
    // A venue already has its grandstand; a garage row behind the paddock.
    for (let i = 0; i < 4; i++) {
      const x = bounds.minX + (i + 0.5) * (hx * 2 / 4);
      placeBuilding(x, bounds.minY - 45, 0, -1, i === 1 ? office : garage, 40, 8, 18);
    }
  }

  // ---- woods: clumps and loners on the offset loop, with gaps --------------
  const D0 = venue ? 70 : 165;
  const total0 = loopLength(hx, hy, D0);
  const nA = Math.min(170, Math.max(60, Math.round(total0 / 12)));
  const step = total0 / nA;
  for (let i = 0; i < nA; i++) {
    const s = (i + rand() * 0.7) * step;
    const t = (s / total0) * Math.PI * 2;
    // Leave openings so the horizon shows through rather than a solid wall.
    const g = Math.sin(3 * t + 0.8) + 0.6 * Math.sin(7 * t + 2.1);
    if (g > 0.9 && rand() < 0.8) continue;
    const [px, py, nx, ny] = loopPoint(hx, hy, D0, s);
    const out = rand() * rand() * 40;
    const k = 1 + Math.floor(rand() * rand() * 4);
    for (let j = 0; j < k; j++) {
      const x = cx + px + nx * out + (rand() - 0.5) * 12;
      const y = cy + py + ny * out + (rand() - 0.5) * 12;
      if (isBlocked(x, y)) continue;
      tree(b, x, -y, 6 + rand() * 8, rand, 0.5);
    }
  }
  // A sparser band further out, for depth.
  const D1 = D0 + 60;
  const total1 = loopLength(hx, hy, D1);
  const nB = Math.min(90, Math.round(total1 / 40));
  for (let i = 0; i < nB; i++) {
    const s = (i + rand()) * (total1 / nB);
    const [px, py, nx, ny] = loopPoint(hx, hy, D1, s);
    const out = rand() * 90;
    const k = 1 + Math.floor(rand() * 3);
    for (let j = 0; j < k; j++) {
      const x = cx + px + nx * out + (rand() - 0.5) * 14;
      const y = cy + py + ny * out + (rand() - 0.5) * 14;
      if (isBlocked(x, y)) continue;
      tree(b, x, -y, 7 + rand() * 7, rand, 0.55);
    }
  }
  // Loners on the grass between the apron and the woods.
  if (!venue) {
    for (let i = 0; i < 14; i++) {
      const D = 118 + rand() * 35;
      const [px, py] = loopPoint(hx, hy, D, rand() * loopLength(hx, hy, D));
      const x = cx + px, y = cy + py;
      if (isBlocked(x, y)) continue;
      tree(b, x, -y, 7 + rand() * 6, rand, 0.35);
    }
  }

  // ---- horizon: two silhouette rings and a couple of low hills -------------
  // The renderer's fog is 1 - exp(-(d/520)^1.6): 0.5 at 400 m, 0.85 at
  // 700 m, 0.96 at 950 m. A ring at 1.2 km would be sky-coloured, so the
  // rings sit where the fog still leaves them a faint dark band across an
  // autocross-sized lot. If the fog scale grows, push HORIZON out with it
  // (about 0.75x the fog's scale distance keeps the same look).
  const HORIZON = venue ? 300 : 380;
  horizonBand(b, cx, cy, hx, hy, HORIZON, 5, FAR_TREES, 1.0, rand);
  horizonBand(b, cx, cy, hx, hy, HORIZON + 70, 6, FAR_TREES_2, 1.3, rand);
  const DH = HORIZON + 130;
  const totalH = loopLength(hx, hy, DH);
  const fracs = venue ? [0.06, 0.38, 0.70] : [0.10, 0.62];
  for (const f of fracs) {
    const [px, py, nx, ny] = loopPoint(hx, hy, DH, f * totalH);
    hill(b, cx + px, cy + py, Math.atan2(nx, -ny), 600 + rand() * 300, 260, 38 + rand() * 20, rand);
  }

  return b.mesh();
}
