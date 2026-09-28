// Buildings, grandstands, the scoring pylon and the light poles of the real
// MIS site (render/sitemesh.js), from the traced outlines in mis-site.json.
//
// Everything is emitted through the caller's `put(tri, colour, mat)` in SITE
// coordinates with a height: `v = [siteX, siteY, z]` where z is metres above
// the drawn ground at that site point (the caller adds the ground and carries
// the vertex into the course frame). `mat` is the per-vertex material the car
// program reads: null for plain matte scenery, EMISSIVE for the pylon's LEDs.

export const EMISSIVE = [0.5, 0.0, 2.0];

const CONCRETE = [0.56, 0.56, 0.54];
const CLADDING = [[0.62, 0.62, 0.60], [0.55, 0.56, 0.58], [0.66, 0.63, 0.58], [0.50, 0.52, 0.55]];
const GLASS = [0.07, 0.09, 0.12];
const ROOF_UNIT = [0.60, 0.61, 0.62];
const SEATS = [[0.26, 0.40, 0.66], [0.30, 0.46, 0.74], [0.42, 0.45, 0.52], [0.28, 0.43, 0.70]];
const RISER = [0.66, 0.66, 0.64];
const PYLON_BODY = [0.12, 0.12, 0.13];
const LED_PANEL = [0.025, 0.025, 0.03];
const LED_AMBER = [1.0, 0.72, 0.18];
const LED_WHITE = [0.95, 0.95, 0.92];
const LED_RED = [1.0, 0.16, 0.12];
const POLE = [0.52, 0.53, 0.55];
const LAMP = [0.78, 0.78, 0.74];

/**
 * @param meta  mis-site.json
 * @param put   (a, b, c, colour, mat) -> emits one triangle; a/b/c = [sx, sy, z]
 * @param groundAt (sx, sy) -> drawn ground height, for seating a building level
 * @param rand  seeded random in [0, 1)
 */
export function buildStructures(meta, putAbs, groundAt, rand) {
  // Every object is built from z = 0 at its own base; G is that base's drawn
  // height (a building sits level on the lowest ground under it).
  let G = 0;
  const put = (a, b, c, col, mat) => putAbs([a[0], a[1], a[2] + G], [b[0], b[1], b[2] + G], [c[0], c[1], c[2] + G], col, mat);
  const quad = (a, b, c, d, col, mat = null) => { put(a, b, c, col, mat); put(a, c, d, col, mat); };
  const stats = { buildings: 0, stands: 0, tris: 0 };

  for (const bld of meta.buildings?.list ?? []) {
    const stand = bld.kind === "stand";
    // Seat the whole building at the lowest ground under its footprint.
    let g = Infinity;
    for (const poly of bld.tiers[0].p) for (const [x, y] of poly) g = Math.min(g, groundAt(x, y));
    G = g;
    const clad = CLADDING[Math.floor(rand() * CLADDING.length)];
    let below = -0.3;
    bld.tiers.forEach((tier, ti) => {
      const top = tier.z, last = ti === bld.tiers.length - 1;
      const tread = stand ? SEATS[ti % SEATS.length] : last ? bld.roof : clad;
      for (const raw of tier.p) {
        const poly = ccw(raw);
        // Walls from the tier below up to this one.
        for (let e = 0; e < poly.length; e++) {
          const [ax, ay] = poly[e], [bx, by] = poly[(e + 1) % poly.length];
          const wallCol = stand ? RISER : clad;
          quad([ax, ay, below], [bx, by, below], [bx, by, top], [ax, ay, top], wallCol);
          // Windows: a glass band per 3.6 m storey on buildings, the suites'
          // glass on a grandstand's top tier.
          const L = Math.hypot(bx - ax, by - ay);
          if (L < 3) continue;
          const ux = (bx - ax) / L, uy = (by - ay) / L, nx = uy * 0.03, ny = -ux * 0.03;   // outward (poly is CCW)
          const m0 = Math.min(0.6, L * 0.1);
          const a0 = [ax + ux * m0 + nx, ay + uy * m0 + ny], b0 = [bx - ux * m0 + nx, by - uy * m0 + ny];
          if (!stand && top - below >= 3) {
            for (let f = Math.max(0, below) + 1.0; f + 1.2 <= top - 0.4; f += 3.6) {
              quad([a0[0], a0[1], f], [b0[0], b0[1], f], [b0[0], b0[1], f + 1.2], [a0[0], a0[1], f + 1.2], GLASS);
            }
          } else if (stand && last && top - below > 1.2) {
            quad([a0[0], a0[1], below + 0.3], [b0[0], b0[1], below + 0.3], [b0[0], b0[1], top - 0.3], [a0[0], a0[1], top - 0.3], GLASS);
          }
        }
        // The tread / roof on top.
        for (const [i, j, k] of triangulate(poly)) put([...poly[i], top], [...poly[j], top], [...poly[k], top], tread, null);
        // A grandstand's aisles: concrete stairs across each tread every ~28 m
        // along its longest edges.
        if (stand) {
          for (let e = 0; e < poly.length; e++) {
            const [ax, ay] = poly[e], [bx, by] = poly[(e + 1) % poly.length];
            const L = Math.hypot(bx - ax, by - ay);
            if (L < 20) continue;
            const ux = (bx - ax) / L, uy = (by - ay) / L, ix = -uy * 2.2, iy = ux * 2.2;   // 2.2 m in from the edge
            for (let d = 14; d < L - 4; d += 28) {
              const px = ax + ux * d, py = ay + uy * d;
              quad([px - ux * 0.7, py - uy * 0.7, top + 0.02], [px + ux * 0.7, py + uy * 0.7, top + 0.02],
                [px + ux * 0.7 + ix, py + uy * 0.7 + iy, top + 0.02], [px - ux * 0.7 + ix, py - uy * 0.7 + iy, top + 0.02], RISER);
            }
          }
        }
        if (!stand && last) {
          parapet(poly, top, clad, quad);
          const A = Math.abs(area(poly));
          for (let u = 0; u < Math.min(4, Math.floor(A / 120)); u++) {
            const p = pointIn(poly, rand);
            if (p) box(p[0], p[1], top, 1 + rand() * 1.5, 1 + rand() * 1.5, 0.9 + rand() * 0.8, rand() * Math.PI, ROOF_UNIT, quad);
          }
        }
      }
      below = top;
    });
    if (stand) stats.stands++; else stats.buildings++;
  }

  if (meta.pylon) { G = groundAt(meta.pylon.x, meta.pylon.y); pylon(meta.pylon, quad, rand); }
  for (const [x, y, h] of meta.poles?.xyh ?? []) { G = groundAt(x, y); lightPole(x, y, Math.min(h, 25), rand() * Math.PI, quad); }
  return stats;
}

// ---- the scoring pylon ------------------------------------------------------
// A slim column carrying a tall LED cabinet: a red header with MIS on it,
// then the running order -- position on the left in white, car number on the
// right in amber -- on all four faces, lit.
function pylon(p, quad, rand) {
  const a = (p.angDeg * Math.PI) / 180, ca = Math.cos(a), sa = Math.sin(a);
  const W = Math.max(5, p.w), D = Math.max(3, p.d * 0.7), H = p.h;
  const at = (u, v) => [p.x + u * ca - v * sa, p.y + u * sa + v * ca];
  box(p.x, p.y, 0, W * 0.35, D * 0.45, H * 0.3, a, PYLON_BODY, quad);
  const z0 = H * 0.3, z1 = H - 0.6;
  box(p.x, p.y, z0, W, D, z1 - z0, a, PYLON_BODY, quad);
  box(p.x, p.y, z1, W + 0.4, D + 0.4, 0.6, a, [0.7, 0.7, 0.72], quad);
  // One face: centre offset along its normal, its width axis (u) and span.
  const faces = [
    { n: [0, -1], u: [1, 0], span: W, off: D / 2 }, { n: [0, 1], u: [-1, 0], span: W, off: D / 2 },
    { n: [1, 0], u: [0, 1], span: D, off: W / 2 }, { n: [-1, 0], u: [0, -1], span: D, off: W / 2 },
  ];
  const cars = shuffled([2, 3, 5, 8, 9, 11, 12, 14, 17, 19, 20, 22, 23, 24, 31, 34, 38, 41, 42, 43, 45, 47, 48, 51, 54, 71, 77, 99], rand);
  for (const f of faces) {
    const out = 0.04;
    // Face point: s along the face (-span/2..span/2), z height.
    const F = (s, z, lift = 0) => {
      const u = f.u[0] * s + f.n[0] * (f.off + out + lift), v = f.u[1] * s + f.n[1] * (f.off + out + lift);
      return [...at(u, v), z];
    };
    const rect = (s0, s1, za, zb, col, mat, lift = 0) => quad(F(s0, za, lift), F(s1, za, lift), F(s1, zb, lift), F(s0, zb, lift), col, mat);
    const m = f.span * 0.06;
    rect(-f.span / 2 + m, f.span / 2 - m, z0 + 0.4, z1 - 0.3, LED_PANEL, null);
    // Header: a red band with MIS in white.
    const hz0 = z1 - 0.3 - 3.2, hz1 = z1 - 0.5;
    rect(-f.span / 2 + m * 1.5, f.span / 2 - m * 1.5, hz0, hz1, LED_RED, EMISSIVE, 0.01);
    word("MIS", 0, (hz0 + hz1) / 2, Math.min(2.2, (hz1 - hz0) * 0.7), f.span * 0.7, (s0, s1, za, zb) => rect(s0, s1, za, zb, LED_WHITE, EMISSIVE, 0.02));
    // The running order.
    const rowH = 1.25, rows = Math.floor((hz0 - 0.4 - (z0 + 0.6)) / rowH);
    const dh = rowH * 0.62, dw = dh * 0.55;
    for (let r = 0; r < rows; r++) {
      const zc = hz0 - 0.5 - (r + 0.5) * rowH;
      const pos = String(r + 1), car = String(cars[r % cars.length]);
      digits(pos, -f.span / 2 + m * 2 + dw, zc, dw, dh, (s0, s1, za, zb) => rect(s0, s1, za, zb, LED_WHITE, EMISSIVE, 0.02));
      digits(car, f.span / 2 - m * 2 - dw * (car.length * 1.35) + dw * 0.35, zc, dw, dh, (s0, s1, za, zb) => rect(s0, s1, za, zb, LED_AMBER, EMISSIVE, 0.02));
    }
  }
}

// Seven-segment digits: segment rectangles in face coordinates.
const SEG = {
  0: "abcdef", 1: "bc", 2: "abdeg", 3: "abcdg", 4: "bcfg", 5: "acdfg", 6: "acdefg", 7: "abc", 8: "abcdefg", 9: "abcdfg",
};
function digits(str, s0, zc, w, h, rect) {
  const t = w * 0.18;
  let s = s0;
  for (const ch of str) {
    const segs = SEG[ch] ?? "";
    const L = s, R = s + w, T = zc + h / 2, B = zc - h / 2, M = zc;
    const seg = {
      a: [L + t, R - t, T - t, T], d: [L + t, R - t, B, B + t], g: [L + t, R - t, M - t / 2, M + t / 2],
      f: [L, L + t, M, T - t], b: [R - t, R, M, T - t], e: [L, L + t, B + t, M], c: [R - t, R, B + t, M],
    };
    for (const k of segs) { const [x0, x1, z0, z1] = seg[k]; rect(x0, x1, z0, z1); }
    s += w * 1.35;
  }
}
// Block capitals for the header (only the letters it needs).
const GLYPH = {
  M: ["10001", "11011", "10101", "10001", "10001"],
  I: ["111", "010", "010", "010", "111"],
  S: ["0111", "1000", "0110", "0001", "1110"],
};
function word(str, sc, zc, h, maxW, rect) {
  const px = h / 5;
  const widths = [...str].map((c) => GLYPH[c][0].length);
  let total = (widths.reduce((a, b) => a + b, 0) + str.length - 1) * px;
  const k = total > maxW ? maxW / total : 1;
  const p = px * k; total *= k;
  let s = sc - total / 2;
  [...str].forEach((c, i) => {
    GLYPH[c].forEach((row, r) => {
      [...row].forEach((bit, q) => {
        if (bit !== "1") return;
        const z1 = zc + h / 2 - r * px, z0 = z1 - px * 0.9;
        rect(s + q * p, s + (q + 0.9) * p, z0, z1);
      });
    });
    s += (widths[i] + 1) * p;
  });
}

function lightPole(x, y, h, ang, quad) {
  box(x, y, 0, 0.25, 0.25, h, ang, POLE, quad);
  const c = Math.cos(ang), s = Math.sin(ang);
  box(x, y, h - 0.3, 2.6, 0.18, 0.18, ang, POLE, quad);
  for (const o of [-1.1, 1.1]) box(x + c * o, y + s * o, h - 0.55, 0.7, 0.45, 0.28, ang, LAMP, quad);
}

// ---- geometry helpers -------------------------------------------------------
/** An oriented box: centre (x, y), base z0, size w x d x h, turned by ang. */
function box(x, y, z0, w, d, h, ang, col, quad) {
  const c = Math.cos(ang), s = Math.sin(ang);
  const P = (u, v, z) => [x + u * c - v * s, y + u * s + v * c, z];
  const hw = w / 2, hd = d / 2, z1 = z0 + h;
  const side = [col[0] * 0.9, col[1] * 0.9, col[2] * 0.9];
  quad(P(-hw, -hd, z1), P(hw, -hd, z1), P(hw, hd, z1), P(-hw, hd, z1), col);
  quad(P(-hw, -hd, z0), P(-hw, -hd, z1), P(-hw, hd, z1), P(-hw, hd, z0), side);
  quad(P(hw, hd, z0), P(hw, hd, z1), P(hw, -hd, z1), P(hw, -hd, z0), side);
  quad(P(hw, -hd, z0), P(hw, -hd, z1), P(-hw, -hd, z1), P(-hw, -hd, z0), side);
  quad(P(-hw, hd, z0), P(-hw, hd, z1), P(hw, hd, z1), P(hw, hd, z0), side);
}
function parapet(poly, top, col, quad) {
  for (let e = 0; e < poly.length; e++) {
    const [ax, ay] = poly[e], [bx, by] = poly[(e + 1) % poly.length];
    const L = Math.hypot(bx - ax, by - ay); if (L < 0.5) continue;
    const nx = (by - ay) / L * 0.25, ny = -(bx - ax) / L * 0.25;   // inward for a CCW poly is -n; the lip sits on the edge
    quad([ax, ay, top + 0.45], [bx, by, top + 0.45], [bx - nx, by - ny, top + 0.45], [ax - nx, ay - ny, top + 0.45], col);
    quad([ax, ay, top], [bx, by, top], [bx, by, top + 0.45], [ax, ay, top + 0.45], col);
    quad([bx - nx, by - ny, top], [ax - nx, ay - ny, top], [ax - nx, ay - ny, top + 0.45], [bx - nx, by - ny, top + 0.45], col);
  }
}
function area(poly) {
  let a = 0;
  for (let i = 0; i < poly.length; i++) { const [x0, y0] = poly[i], [x1, y1] = poly[(i + 1) % poly.length]; a += x0 * y1 - x1 * y0; }
  return a / 2;
}
function ccw(poly) { return area(poly) < 0 ? [...poly].reverse() : poly; }
function inPoly(poly, x, y) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}
function pointIn(poly, rand) {
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const [x, y] of poly) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
  for (let t = 0; t < 20; t++) {
    const x = x0 + rand() * (x1 - x0), y = y0 + rand() * (y1 - y0);
    if (inPoly(poly, x, y) && inPoly(poly, x + 2, y) && inPoly(poly, x - 2, y) && inPoly(poly, x, y + 2) && inPoly(poly, x, y - 2)) return [x, y];
  }
  return null;
}
function shuffled(a, rand) { const b = [...a]; for (let i = b.length - 1; i > 0; i--) { const j = Math.floor(rand() * (i + 1)); [b[i], b[j]] = [b[j], b[i]]; } return b; }
/** Ear clipping for a simple CCW polygon: index triples. */
function triangulate(poly) {
  const idx = poly.map((_, i) => i), out = [];
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  let guard = 0;
  while (idx.length > 3 && guard++ < 10000) {
    let clipped = false;
    for (let k = 0; k < idx.length; k++) {
      const i = idx[(k + idx.length - 1) % idx.length], j = idx[k], l = idx[(k + 1) % idx.length];
      const A = poly[i], B = poly[j], C = poly[l];
      if (cross(A, B, C) <= 1e-9) continue;
      let ear = true;
      for (const q of idx) {
        if (q === i || q === j || q === l) continue;
        const Pq = poly[q];
        if (cross(A, B, Pq) >= 0 && cross(B, C, Pq) >= 0 && cross(C, A, Pq) >= 0) { ear = false; break; }
      }
      if (!ear) continue;
      out.push([i, j, l]); idx.splice(k, 1); clipped = true; break;
    }
    if (!clipped) break;   // degenerate: give up on the rest rather than loop
  }
  if (idx.length === 3) out.push([idx[0], idx[1], idx[2]]);
  return out;
}
