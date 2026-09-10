// Distant environment: a tree line and a grandstand around the course so the
// lot sits somewhere rather than floating in a void.
//
// FSAE Michigan runs on the paddock lot at Michigan International Speedway,
// so the grandstand on one side is not decoration -- it is what you actually
// see from the autocross grid. Everything is low-poly and vertex coloured,
// built once per track and drawn in a single call through the lit static
// shader; the fog does the rest.
//
// Coordinates: world (x, y) -> GL (x, height, -y), like everything else.

const TRUNK = [0.30, 0.24, 0.18];
const CANOPIES = [
  [0.18, 0.34, 0.16],
  [0.22, 0.40, 0.18],
  [0.16, 0.30, 0.14],
  [0.26, 0.42, 0.17],
  [0.20, 0.36, 0.20],
];
const STAND_SEAT = [0.28, 0.40, 0.66];   // MIS blue seating
const STAND_STEP = [0.40, 0.41, 0.44];
const STAND_ROOF = [0.78, 0.79, 0.80];
const STAND_POST = [0.52, 0.53, 0.55];
const BUILDING = [0.62, 0.60, 0.56];
const BUILDING_DK = [0.50, 0.48, 0.45];
const ROOF = [0.42, 0.42, 0.44];

class Builder {
  constructor() { this.p = []; this.n = []; this.c = []; }

  tri(a, b, c, colour, normal) {
    let nx, ny, nz;
    if (normal) {
      [nx, ny, nz] = normal;
    } else {
      const ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
      const vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
      nx = uy * vz - uz * vy; ny = uz * vx - ux * vz; nz = ux * vy - uy * vx;
      const l = Math.hypot(nx, ny, nz) || 1;
      nx /= l; ny /= l; nz /= l;
    }
    for (const v of [a, b, c]) {
      this.p.push(v[0], v[1], v[2]);
      this.n.push(nx, ny, nz);
      this.c.push(colour[0], colour[1], colour[2]);
    }
  }

  quad(a, b, c, d, colour, normal) {
    this.tri(a, b, c, colour, normal);
    this.tri(a, c, d, colour, normal);
  }

  /** Axis-aligned box in GL space, base at y0. */
  box(cx, y0, cz, w, h, d, colour, top = colour) {
    const x0 = cx - w / 2, x1 = cx + w / 2, z0 = cz - d / 2, z1 = cz + d / 2, y1 = y0 + h;
    this.quad([x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1], colour, [0, 0, 1]);
    this.quad([x1, y0, z0], [x0, y0, z0], [x0, y1, z0], [x1, y1, z0], colour, [0, 0, -1]);
    this.quad([x1, y0, z1], [x1, y0, z0], [x1, y1, z0], [x1, y1, z1], colour, [1, 0, 0]);
    this.quad([x0, y0, z0], [x0, y0, z1], [x0, y1, z1], [x0, y1, z0], colour, [-1, 0, 0]);
    this.quad([x0, y1, z1], [x1, y1, z1], [x1, y1, z0], [x0, y1, z0], top, [0, 1, 0]);
  }

  /** A cone (tree canopy tier) with soft-ish normals, base at y0. */
  cone(cx, y0, cz, r, h, segs, colour) {
    for (let i = 0; i < segs; i++) {
      const a0 = (i / segs) * Math.PI * 2, a1 = ((i + 1) / segs) * Math.PI * 2;
      const am = (a0 + a1) / 2;
      const slope = r / h;
      const nl = Math.hypot(1, slope);
      const n = [Math.cos(am) / nl, slope / nl, Math.sin(am) / nl];
      this.tri(
        [cx + Math.cos(a0) * r, y0, cz + Math.sin(a0) * r],
        [cx, y0 + h, cz],
        [cx + Math.cos(a1) * r, y0, cz + Math.sin(a1) * r],
        colour, n,
      );
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

function tree(b, x, z, h, rand) {
  const canopy = CANOPIES[Math.floor(rand() * CANOPIES.length)];
  const shade = 0.9 + rand() * 0.2;
  const c = [canopy[0] * shade, canopy[1] * shade, canopy[2] * shade];
  const trunkH = h * 0.18;
  b.box(x, 0, z, h * 0.06, trunkH, h * 0.06, TRUNK);
  // Two stacked tiers read as a conifer; a broad one as a deciduous crown.
  if (rand() < 0.55) {
    b.cone(x, trunkH, z, h * 0.32, h * 0.55, 7, c);
    b.cone(x, trunkH + h * 0.35, z, h * 0.24, h * 0.47, 7, c);
  } else {
    b.cone(x, trunkH, z, h * 0.42, h * 0.82, 8, c);
  }
}

/**
 * Grandstand: a stepped wedge facing the course, with a roof on posts.
 * Built along the x axis at the given z (GL), length L, facing -z (toward
 * smaller |z|, i.e. toward the caller's centre when placed on the +y side).
 */
function grandstand(b, cx, cz, L, facing) {
  const depth = 22, height = 15, steps = 6;
  const f = facing; // +1 or -1: which way the seating faces in z
  for (let i = 0; i < steps; i++) {
    const t0 = i / steps, t1 = (i + 1) / steps;
    const y0 = t0 * height, y1 = t1 * height;
    const z0 = cz + f * t0 * depth, z1 = cz + f * t1 * depth;
    // riser
    b.quad([cx - L / 2, y0, z0], [cx + L / 2, y0, z0], [cx + L / 2, y1, z0], [cx - L / 2, y1, z0],
      STAND_STEP, [0, 0, -f]);
    // tread (seats)
    b.quad([cx - L / 2, y1, z0], [cx + L / 2, y1, z0], [cx + L / 2, y1, z1], [cx - L / 2, y1, z1],
      STAND_SEAT, [0, 1, 0]);
  }
  // back wall and ends
  const zb = cz + f * depth;
  b.quad([cx - L / 2, 0, zb], [cx + L / 2, 0, zb], [cx + L / 2, height, zb], [cx - L / 2, height, zb],
    STAND_STEP, [0, 0, f]);
  for (const s of [-1, 1]) {
    const x = cx + s * L / 2;
    b.tri([x, 0, cz], [x, 0, zb], [x, height, zb], STAND_STEP, [s, 0, 0]);
  }
  // roof canopy on posts, overhanging the front row
  const roofY = height + 6;
  b.quad([cx - L / 2, roofY, cz - f * 3], [cx + L / 2, roofY, cz - f * 3],
    [cx + L / 2, roofY + 1.2, zb + f * 1], [cx - L / 2, roofY + 1.2, zb + f * 1], STAND_ROOF, [0, 1, 0]);
  b.quad([cx - L / 2, roofY, cz - f * 3], [cx - L / 2, roofY + 1.2, zb + f * 1],
    [cx + L / 2, roofY + 1.2, zb + f * 1], [cx + L / 2, roofY, cz - f * 3], [0.55, 0.56, 0.58], [0, -1, 0]);
  for (let x = cx - L / 2 + 6; x < cx + L / 2; x += 18) {
    b.box(x, 0, zb - f * 2, 0.7, roofY, 0.7, STAND_POST);
  }
}

function building(b, x, z, w, h, d, rand) {
  const tone = 0.9 + rand() * 0.2;
  const c = [BUILDING[0] * tone, BUILDING[1] * tone, BUILDING[2] * tone];
  b.box(x, 0, z, w, h, d, c, ROOF);
  // A darker plinth so it reads as sitting on the ground.
  b.box(x, 0, z, w + 0.6, 1.2, d + 0.6, BUILDING_DK);
}

/**
 * Build the scenery for a course or venue.
 * @param bounds {minX, maxX, minY, maxY} of the driveable geometry, world (x, y)
 * @param kind   "course" or "venue"
 */
export function buildEnvironmentMesh(bounds, kind = "course") {
  const b = new Builder();
  const rand = rng(kind === "venue" ? 7331 : 1337);
  const cx = (bounds.minX + bounds.maxX) / 2;
  const cy = (bounds.minY + bounds.maxY) / 2;
  const hx = (bounds.maxX - bounds.minX) / 2;
  const hy = (bounds.maxY - bounds.minY) / 2;
  const w2g = (x, y) => [x, -y];

  // Distances scale with the course so a 2 km oval gets its trees further out.
  const pad = kind === "venue" ? 60 : 150;
  const rx = hx + pad, ry = hy + pad;

  // ---- tree line: clumps around the ellipse, with gaps ---------------------
  const N = kind === "venue" ? 260 : 190;
  for (let i = 0; i < N; i++) {
    const a = (i / N) * Math.PI * 2;
    // Leave the grandstand side thinner, and open a couple of gaps so the
    // horizon shows through instead of a solid green wall.
    const gap = Math.sin(a * 3.0 + 0.8) > 0.72;
    if (gap && rand() < 0.75) continue;
    const jitterR = 1 + rand() * 0.35;
    const x = cx + Math.cos(a) * rx * jitterR + (rand() - 0.5) * 18;
    const y = cy + Math.sin(a) * ry * jitterR + (rand() - 0.5) * 18;
    const [gx, gz] = w2g(x, y);
    tree(b, gx, gz, 7 + rand() * 9, rand);
    // Occasionally a second tree close by, for clumping.
    if (rand() < 0.4) tree(b, gx + (rand() - 0.5) * 9, gz + (rand() - 0.5) * 9, 6 + rand() * 8, rand);
  }

  if (kind === "course") {
    // ---- grandstand on the north side, the way MIS sits behind the lot -----
    // North of the course is GL -z, and the seating must rise AWAY from the
    // course, so the stand faces +z (toward the lot).
    const L = Math.max(140, hx * 1.3);
    const [gx, gz] = w2g(cx, bounds.maxY + pad * 1.25);
    grandstand(b, gx, gz, L, -1);

    // ---- a few paddock buildings / garages on the south and east -----------
    for (let i = 0; i < 5; i++) {
      const x = bounds.minX - 20 + rand() * (hx * 2 + 40);
      const y = bounds.minY - pad * (0.55 + rand() * 0.3);
      const [bx, bz] = w2g(x, y);
      building(b, bx, bz, 24 + rand() * 30, 6 + rand() * 5, 14 + rand() * 10, rand);
    }
    for (let i = 0; i < 3; i++) {
      const x = bounds.maxX + pad * (0.5 + rand() * 0.35);
      const y = bounds.minY + rand() * hy * 2;
      const [bx, bz] = w2g(x, y);
      building(b, bx, bz, 16 + rand() * 20, 5 + rand() * 4, 12 + rand() * 8, rand);
    }
  } else {
    // A venue already has its grandstand; put a garage row behind the paddock.
    for (let i = 0; i < 4; i++) {
      const x = bounds.minX + (i + 0.5) * (hx * 2 / 4);
      const y = bounds.minY - pad * 0.5;
      const [bx, bz] = w2g(x, y);
      building(b, bx, bz, 40, 8, 18, rand);
    }
  }

  return b.mesh();
}
