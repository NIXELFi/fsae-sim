// SDM26 geometry, built procedurally in the chassis frame.
//
// Frame: +X forward, +Y up, +Z to the driver's right. The origin sits on the
// ground directly under the CG, so every station below is a real dimension off
// the car: the front axle is at x = +0.788 and the rear at x = -0.742 because
// that is what a 1.53 m wheelbase at 48.5% front actually gives you.
//
// Parts come out separately rather than as one blob because they move
// independently -- the road wheels steer and spin, the steering wheel turns,
// and the rims fade out at speed so the spokes do not strobe.
//
// Nothing here is culled by winding: the renderer draws the car with two-sided
// lighting, so a face pointing the wrong way still shades correctly. Inside a
// cockpit you are looking at the back of half the bodywork by definition.

import { SDM26, lengthToFrontAxle, lengthToRearAxle } from "../vehicle/params.js";

// ---- palette ----
//
// Every colour here is in DISPLAY (sRGB-ish, gamma 2.2) space, the same
// convention as the asphalt and the venue: the renderer's `toLinear` decodes
// it before lighting. That is the number you would read off a swatch or a
// photograph, NOT a physical albedo. The two differ by the gamma curve, and
// at the dark end the difference is huge: a real slick has a linear albedo of
// about 0.02-0.03, which is 0.16-0.19 in display space. Author 0.03 here and
// the shader decodes it to 0.0005 -- a black hole with no readable shape,
// which is exactly what the tyres, dash and grips were until 2026-09-20.
//
// So the rule for the dark materials is: nothing below ~0.16 display. Rubber
// sits at 0.17-0.20, carbon and black mouldings at 0.19-0.24, and the tonal
// separation between a tread and its sidewall, or a plate and its recess, is
// carried by a difference of 0.03-0.05 in display space (30-40% in linear),
// not by pushing one of them toward zero.
//
// ASU maroon is #8C1D40 = [0.55, 0.11, 0.25] as a swatch, but paint under a
// clearcoat has to be able to carry the sky in its shading, and a green
// channel of 0.114 decodes to 0.0085 linear -- effectively zero, so the paint
// could take no blue-sky tint and read as a dead brown-red. The green is
// lifted to 0.12 (0.0095 linear) and the blue trimmed a touch so the swatch
// still reads as ASU maroon rather than magenta.
const MAROON = [0.55, 0.12, 0.20];
const MAROON_DK = [0.40, 0.085, 0.145];
const GOLD = [1.0, 0.776, 0.153];
/** Bare carbon-fibre laminate: ~0.02 linear under its gloss. */
const CARBON = [0.20, 0.205, 0.215];
const CARBON_LT = [0.25, 0.255, 0.27];
/** The tread: a slick's 0.02-0.03 linear albedo, in display space. */
const TIRE = [0.175, 0.178, 0.185];
/** Sidewall, a touch lighter so the shoulder line is visible at all. */
const TIRE_WALL = [0.215, 0.218, 0.226];
const RIM = [0.60, 0.61, 0.64];
const RIM_FACE = [0.46, 0.47, 0.51];
const METAL = [0.42, 0.44, 0.48];
const DASH_LCD = [0.16, 0.20, 0.185];
// Steering wheel, matched to the team's wheel asset. Carbon reads darker in
// the cockpit's shadow than in a studio render, so the plate sits at the
// light end of the carbon range.
const CARBON_PLATE = [0.235, 0.24, 0.25];
/** Recesses and the lightening slot -- the same material, in shadow. */
const CARBON_DARK = [0.175, 0.178, 0.188];
/** The AiM case: a matte black moulding, ~0.025 linear. */
const DASH_CASE = [0.19, 0.192, 0.20];
const DASH_BEZEL = [0.16, 0.162, 0.17];
const DASH_BUTTON = [0.30, 0.31, 0.33];
/** Suede grip: the darkest thing on the wheel, but still not a void. */
const GRIP = [0.165, 0.168, 0.176];
const AMBER = [0.95, 0.62, 0.12];
const AMBER_LIT = [1.0, 0.80, 0.34];
/** The domed face of a button, which catches the light the barrel does not. */
const GOLD_LIT = [1.0, 0.88, 0.42];
const EMBLEM = [0.86, 0.87, 0.88];

// ---- the driver ----
// Same display-space rule as the rest: the suit and the harness are the
// darkest cloth on the car, and Nomex under sunlight is about 0.03 linear,
// not zero. The suit carries a little of the team's maroon so it separates
// from the carbon around it; the webbing is navy so it separates from the
// suit.
const SUIT = [0.205, 0.17, 0.18];
const GLOVE = [0.30, 0.13, 0.17];
const STRAP = [0.17, 0.185, 0.26];
/** Helmet shell: gloss white, with the maroon below the visor and up the crown. */
const HELMET = [0.90, 0.895, 0.88];
const HELMET_TRIM = MAROON;
/** Visor: a dark tint under gloss. The liner is the matte black inside it. */
const VISOR = [0.17, 0.185, 0.215];
const HELMET_LINER = [0.19, 0.19, 0.20];

// Real SDM26 dimensions (Helios spec sheet).
export const GEO = {
  frontAxle: 0.788,
  rearAxle: -0.742,
  trackFront: 1.207,
  trackRear: 1.194,
  tireRadius: 0.20,
  tireHalfWidth: 0.095,   // 7.5 in section
  rimRadius: 0.127,       // 10 in wheel
  noseTip: 1.25,
  tail: -1.05,
  // The foremost and rearmost points of the CAR, which are the wings and not
  // the bodywork. The cone hitbox is derived from these: a front wing that
  // reaches past the hitbox lets cones pass straight through it, which is both
  // wrong and the opposite of the real car's problem -- the front wing is what
  // actually hits cones.
  frontWingTip: 1.42,
  rearWingTip: -1.16,
  // Driver's eye, and where the wheel sits relative to it.
  // Wheel centre sits low enough that the driver looks OVER the rim, which is
  // where it is in the car -- put it at eye height and it blocks the course.
  // Dropped 30 mm when the dash went on the column: the dash sits above the
  // wheel, the eye is at 0.70, and with the column at 0.50 the panel's top
  // edge came within 60 mm of the driver's eye line and filled the windscreen.
  // Lower the column, the whole assembly follows, and you look DOWN at the
  // dash the way you do in the car.
  // 2026-09-23: back up 30 mm now the dash is where the car has it, up under
  // the front hoop (below), rather than on the column where it crowded the
  // eye line.
  steerCentre: [0.28, 0.465, 0],
  steerTiltRad: (22 * Math.PI) / 180,
  // The team's actual wheel (see packages/widgets/src/steering-wheel/assets):
  // a carbon plate with two kidney cut-outs, grips wrapping their outer edge,
  // 1.4:1 wide. 208 x 148 mm here -- the reference proportions at the compact
  // size the round wheel was cut down to.
  steerHalfWidth: 0.104,
  steerHalfHeight: 0.074,
  /** Steering ratio: 28 deg of lock becomes ~112 deg of wheel rotation. */
  steeringRatio: 4.0,

  // ---- the dash ----
  //
  // An AiM MXS Strada, at its real size. The CASE is 137 x 84 mm and the
  // display inside it is a 5-inch 800x480 panel, 108 x 65 mm. Those two are
  // different rectangles and the difference matters: the screen quad shows
  // only the display, and the bezel around it is geometry.
  //
  // It is on the column shroud ABOVE the wheel and does not turn with it.
  // Positions are in the steering column's own frame: +x across the wheel
  // face, +y up it, +z away from the driver.
  //
  // NO extra rake. The column's own 22 degrees already lean the face back and
  // up toward a driver whose eye is 100 mm above it and behind it; adding more
  // points the screen past them at the sky, which is what the first attempt
  // did and it looked exactly as wrong as it was.
  // 2026-09-23: moved to where the car has it -- up under the front roll
  // hoop's top tube and the cowl (the team's SolidWorks assembly), not on the
  // column. The hoop's top tube crosses the centreline 0.648 m ahead of the
  // origin with its underside 0.605 m up (chassis CAD); in the column frame
  // (22 deg) this puts the display centre at (0.595, 0.535) in the chassis,
  // the case's top ~10 mm under the tube and its back clear of it.
  dashCentre: [0, 0.183, 0.266],
  /** The DISPLAY, 108 x 65 mm. What the screen quad is sized from. */
  dashHalfWidth: 0.054,
  dashHalfHeight: 0.0324,
  /** The CASE, 137 x 84 mm. */
  dashCaseHalfWidth: 0.0685,
  dashCaseHalfHeight: 0.042,
  dashTiltRad: 0,
};

// Shading: `tri`/`quad` give every face one flat normal, which is right for
// boxes, plates and wing planes -- their edges are real. The curved
// primitives (`loft` with a normal grid, `cylZ` smooth, `tube`, `sweep`) put
// an analytic normal on each vertex instead, from the parametric angle the
// builder already knows, so a 40-facet tyre shades as a cylinder and not as
// forty flat strips. The normals are per-vertex-per-triangle (the buffers are
// unindexed), so a smooth patch and a flat one can share one mesh.
class Builder {
  constructor() { this.p = []; this.n = []; this.c = []; }

  tri(a, b, c, color, normal) {
    const nn = normal ?? faceNormal(a, b, c);
    for (const v of [a, b, c]) {
      this.p.push(v[0], v[1], v[2]);
      this.n.push(nn[0], nn[1], nn[2]);
      this.c.push(color[0], color[1], color[2]);
    }
  }

  /**
   * Triangle with a normal per vertex.
   *
   * Each vertex normal is turned to lie on the same side as the triangle's
   * own winding normal. The car is lit two-sided: `CAR_FS` negates the normal
   * on back faces, and "back" is decided by winding, so a smooth normal that
   * disagrees with the winding would be flipped exactly when it should not
   * be and the surface would light from the wrong side. The primitives wind
   * whichever way their authors found natural, and their vertex normals are
   * computed "outward" -- this reconciles the two without every caller
   * having to reason about it. A zero normal (degenerate patch) falls back
   * to the flat one.
   */
  triN(a, b, c, na, nb, nc, color) {
    const fn = faceNormal(a, b, c);
    const pts = [a, b, c], nrm = [na, nb, nc];
    for (let k = 0; k < 3; k++) {
      const v = pts[k];
      let n = nrm[k];
      const d = n[0] * fn[0] + n[1] * fn[1] + n[2] * fn[2];
      if (d < 0) n = [-n[0], -n[1], -n[2]];
      else if (n[0] === 0 && n[1] === 0 && n[2] === 0) n = fn;
      this.p.push(v[0], v[1], v[2]);
      this.n.push(n[0], n[1], n[2]);
      this.c.push(color[0], color[1], color[2]);
    }
  }

  quad(a, b, c, d, color) {
    const nn = faceNormal(a, b, c);
    this.tri(a, b, c, color, nn);
    this.tri(a, c, d, color, nn);
  }

  quadN(a, b, c, d, na, nb, nc, nd, color) {
    this.triN(a, b, c, na, nb, nc, color);
    this.triN(a, c, d, na, nc, nd, color);
  }

  /** Axis-aligned box by centre and full size. */
  box(cx, cy, cz, sx, sy, sz, color) {
    const x0 = cx - sx / 2, x1 = cx + sx / 2;
    const y0 = cy - sy / 2, y1 = cy + sy / 2;
    const z0 = cz - sz / 2, z1 = cz + sz / 2;
    const v = [
      [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
      [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
    ];
    this.quad(v[4], v[5], v[6], v[7], color); // +Z
    this.quad(v[1], v[0], v[3], v[2], color); // -Z
    this.quad(v[5], v[1], v[2], v[6], color); // +X
    this.quad(v[0], v[4], v[7], v[3], color); // -X
    this.quad(v[3], v[7], v[6], v[2], color); // +Y
    this.quad(v[0], v[1], v[5], v[4], color); // -Y
  }

  /**
   * Connect a chain of cross-sections (each an equal-length ring of points)
   * into a surface. Used for the nose, the tub and the engine cover, where a
   * box would read as a shoebox and a real car tapers.
   */
  loft(sections, color, capFirst = true, capLast = true, normals = null) {
    for (let s = 0; s < sections.length - 1; s++) {
      const A = sections[s], B = sections[s + 1];
      const NA = normals?.[s], NB = normals?.[s + 1];
      for (let i = 0; i < A.length; i++) {
        const j = (i + 1) % A.length;
        if (normals) this.quadN(A[i], B[i], B[j], A[j], NA[i], NB[i], NB[j], NA[j], color);
        else this.quad(A[i], B[i], B[j], A[j], color);
      }
    }
    // Caps stay flat: a cap is a real edge on every shape that has one.
    if (capFirst) this.cap(sections[0], color);
    if (capLast) this.cap(sections[sections.length - 1], color);
  }

  /** `loft` with normals derived from the section grid -- see `gridNormals`. */
  smoothLoft(sections, color, capFirst = true, capLast = true) {
    this.loft(sections, color, capFirst, capLast, gridNormals(sections));
  }

  cap(ring, color) {
    const c = [0, 0, 0];
    for (const p of ring) { c[0] += p[0]; c[1] += p[1]; c[2] += p[2]; }
    c[0] /= ring.length; c[1] /= ring.length; c[2] /= ring.length;
    for (let i = 0; i < ring.length; i++) {
      this.tri(c, ring[i], ring[(i + 1) % ring.length], color);
    }
  }

  /**
   * Cylinder with its axis along Z -- the road-wheel and hub primitive.
   * `smooth` puts the radial normal [cos a, sin a, 0] on each vertex so the
   * barrel shades round; off, every facet is flat (the default, unchanged).
   */
  cylZ(cx, cy, cz, r, halfLen, segs, color, smooth = false) {
    for (let i = 0; i < segs; i++) {
      const a0 = (i / segs) * Math.PI * 2, a1 = ((i + 1) / segs) * Math.PI * 2;
      const p = (a, z) => [cx + Math.cos(a) * r, cy + Math.sin(a) * r, cz + z];
      if (smooth) {
        const n0 = [Math.cos(a0), Math.sin(a0), 0], n1 = [Math.cos(a1), Math.sin(a1), 0];
        this.quadN(p(a0, -halfLen), p(a0, halfLen), p(a1, halfLen), p(a1, -halfLen), n0, n0, n1, n1, color);
      } else {
        this.quad(p(a0, -halfLen), p(a0, halfLen), p(a1, halfLen), p(a1, -halfLen), color);
      }
    }
  }

  /** Flat ring in the XY plane, at z. */
  annulusZ(cx, cy, cz, r0, r1, segs, color) {
    for (let i = 0; i < segs; i++) {
      const a0 = (i / segs) * Math.PI * 2, a1 = ((i + 1) / segs) * Math.PI * 2;
      const p = (a, r) => [cx + Math.cos(a) * r, cy + Math.sin(a) * r, cz];
      this.quad(p(a0, r0), p(a0, r1), p(a1, r1), p(a1, r0), color);
    }
  }

  discZ(cx, cy, cz, r, segs, color) {
    for (let i = 0; i < segs; i++) {
      const a0 = (i / segs) * Math.PI * 2, a1 = ((i + 1) / segs) * Math.PI * 2;
      this.tri([cx, cy, cz],
        [cx + Math.cos(a0) * r, cy + Math.sin(a0) * r, cz],
        [cx + Math.cos(a1) * r, cy + Math.sin(a1) * r, cz], color);
    }
  }

  /**
   * Round tube between two points -- suspension links, pylons, pushrods.
   * Every use is a round bar, so the barrel is always smooth-shaded: the
   * normal at angle a is u cos a + v sin a, the radial direction itself.
   * The end caps stay flat.
   */
  tube(p0, p1, r, segs, color) {
    const axis = norm(sub(p1, p0));
    const helper = Math.abs(axis[1]) > 0.9 ? [1, 0, 0] : [0, 1, 0];
    const u = norm(cross(helper, axis));
    const v = cross(axis, u);
    const normals = [];
    const ring = (p) => {
      const out = [];
      for (let i = 0; i < segs; i++) {
        const a = (i / segs) * Math.PI * 2;
        const c = Math.cos(a), s = Math.sin(a);
        out.push([p[0] + (u[0] * c + v[0] * s) * r, p[1] + (u[1] * c + v[1] * s) * r, p[2] + (u[2] * c + v[2] * s) * r]);
        if (normals.length < segs) normals.push([u[0] * c + v[0] * s, u[1] * c + v[1] * s, u[2] * c + v[2] * s]);
      }
      return out;
    };
    this.loft([ring(p0), ring(p1)], color, true, true, [normals, normals]);
  }

  polyTube(points, r, segs, color) {
    for (let i = 0; i < points.length - 1; i++) this.tube(points[i], points[i + 1], r, segs, color);
  }

  /**
   * Sweep a circular section along a path that lies in one plane, given by
   * that plane's unit normal -- the grips (XY plane) and the roll hoops
   * (a plane of constant x).
   *
   * `polyTube` cannot do this: it builds each segment as its own `tube`, so
   * every segment picks its own rotational frame from its own axis and gets
   * its own end caps. The result is a chain of separately-capped sausages
   * whose rings do not line up -- fine for a straight suspension link, wrong
   * for anything that curves, where it reads as a row of blocks.
   *
   * Here the frame is continuous: at each point the section is swept in the
   * plane of the path's in-plane normal and the plane normal, so consecutive
   * rings share an orientation and the whole thing lofts as one surface with
   * two caps. The vertex normal is the same radial direction the point was
   * placed along, so the bar shades round.
   */
  sweep(path, r, segs, color, planeN) {
    const rings = [], normals = [];
    path.forEach((pt, i) => {
      const prev = path[Math.max(0, i - 1)];
      const next = path[Math.min(path.length - 1, i + 1)];
      // In-plane normal: the tangent turned a quarter turn within the plane.
      const n1 = norm(cross(planeN, sub(next, prev)));
      const ring = [], rn = [];
      for (let k = 0; k < segs; k++) {
        const a = (k / segs) * Math.PI * 2;
        const c = Math.cos(a), d = Math.sin(a);
        const n = [n1[0] * c + planeN[0] * d, n1[1] * c + planeN[1] * d, n1[2] * c + planeN[2] * d];
        ring.push([pt[0] + n[0] * r, pt[1] + n[1] * r, pt[2] + n[2] * r]);
        rn.push(n);
      }
      rings.push(ring);
      normals.push(rn);
    });
    this.loft(rings, color, true, true, normals);
  }

  /** `sweep` along a path in the XY plane. */
  sweepXY(path, r, segs, color) {
    this.sweep(path, r, segs, color, [0, 0, 1]);
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

// ------------------------------------------------------------------ helpers ---

function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}
function norm(v) {
  const l = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / l, v[1] / l, v[2] / l];
}
function faceNormal(a, b, c) { return norm(cross(sub(b, a), sub(c, a))); }

/**
 * Smooth normals for a loft, one per section point, from the section grid
 * itself: the cross product of the tangent along the loft (central
 * difference between neighbouring sections, one-sided at the ends) and the
 * tangent around the ring (central difference, the ring being closed). That
 * is the analytic surface normal of the bilinear patch grid, and for the
 * superellipse tub sections it is what makes a 12-point ring read as a
 * continuous monocoque rather than twelve flat panels.
 *
 * Returned in the winding order `loft` uses, so it agrees with the face
 * normals; `triN` re-checks anyway.
 */
function gridNormals(sections) {
  const S = sections.length;
  return sections.map((ring, s) => {
    const n = ring.length;
    const prev = sections[Math.max(0, s - 1)], next = sections[Math.min(S - 1, s + 1)];
    return ring.map((_, i) => {
      const tS = sub(next[i], prev[i]);
      const tI = sub(ring[(i + 1) % n], ring[(i - 1 + n) % n]);
      return norm(cross(tS, tI));
    });
  });
}

/**
 * Rings of a surface of revolution about Z, with their analytic normals.
 * `profile` is a list of [z, r] pairs, in order along the meridian; the
 * normal at each is the meridian tangent turned a quarter turn, then spun
 * round the axis with the point, so the tyre shades as one continuous
 * curve from the crown round the shoulder and down the sidewall.
 */
function revolveZ(profile, segs) {
  const rings = [], normals = [];
  const last = profile.length - 1;
  profile.forEach(([z, r], k) => {
    const [z0, r0] = profile[Math.max(0, k - 1)], [z1, r1] = profile[Math.min(last, k + 1)];
    // Meridian tangent (dr, dz) -> outward normal (dz, -dr), normalised.
    const dr = r1 - r0, dz = z1 - z0;
    const l = Math.hypot(dr, dz) || 1;
    const nr = dz / l, nz = -dr / l;
    const ring = [], rn = [];
    for (let i = 0; i < segs; i++) {
      const a = (i / segs) * Math.PI * 2;
      const c = Math.cos(a), s = Math.sin(a);
      ring.push([c * r, s * r, z]);
      rn.push([c * nr, s * nr, nz]);
    }
    rings.push(ring);
    normals.push(rn);
  });
  return { rings, normals };
}

/** Rounded-rectangle cross-section at station x, for lofting bodywork. */
function section(x, halfW, yBot, yTop, round = 0.35) {
  const pts = [];
  const cy = (yBot + yTop) / 2, hy = (yTop - yBot) / 2;
  const n = 12;
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2;
    // Superellipse: `round` 0 -> rectangle, 1 -> ellipse.
    const ca = Math.cos(a), sa = Math.sin(a);
    const p = 2 / (1 + round * 1.6);
    const sx = Math.sign(ca) * Math.pow(Math.abs(ca), p);
    const sy = Math.sign(sa) * Math.pow(Math.abs(sa), p);
    pts.push([x, cy + sy * hy, sx * halfW]);
  }
  return pts;
}

/**
 * A cambered aerofoil section, as a closed loop in the chord plane.
 *
 * Returned in chord-normalised coordinates: x from 0 at the leading edge to 1
 * at the trailing edge, y positive upward. Thickness follows the NACA 4-digit
 * distribution, which is a shape everyone recognises even at this size, applied
 * about a circular-arc camber line.
 *
 * This exists because a wing drawn as a flat plate is the single thing that
 * most makes a formula car look unfinished. A real wing has a blunt leading
 * edge, a sharp trailing edge and visible curvature, and at the angles a
 * Formula Student car runs -- steep enough to be obvious -- you see all three
 * from any angle.
 */
function aerofoil(thickness = 0.12, camber = 0.055, n = 16) {
  const yt = (x) =>
    (thickness / 0.2) *
    (0.2969 * Math.sqrt(x) - 0.1260 * x - 0.3516 * x * x +
     0.2843 * x * x * x - 0.1015 * x * x * x * x);
  // Circular-arc camber line: peak `camber` at mid-chord, zero at both ends.
  const yc = (x) => camber * 4 * x * (1 - x);
  const dyc = (x) => camber * 4 * (1 - 2 * x);

  const upper = [];
  const lower = [];
  for (let i = 0; i <= n; i++) {
    // Cosine spacing: points bunch at the leading edge, where the curvature is.
    const x = 0.5 * (1 - Math.cos((Math.PI * i) / n));
    const t = yt(x);
    const c = yc(x);
    const theta = Math.atan(dyc(x));
    upper.push([x - t * Math.sin(theta), c + t * Math.cos(theta)]);
    lower.push([x + t * Math.sin(theta), c - t * Math.cos(theta)]);
  }
  // One closed loop, trailing edge -> upper -> leading edge -> lower.
  return upper.concat(lower.slice(1, -1).reverse());
}

/**
 * A wing endplate: a solid plate with thickness, from an outline in the XY
 * plane.
 *
 * Built as a closed prism rather than two facing quads. Two quads have no
 * edges, so from any oblique angle you see straight through the gap between
 * them and the plate reads as a pair of flags rather than a piece of carbon.
 */
function endplate(b, z, side, outline, color) {
  const t = 0.012;
  const inner = outline.map(([x, y]) => [x, y, z]);
  const outer = outline.map(([x, y]) => [x, y, z + side * t]);
  b.loft([inner, outer], color, true, true);
}

/**
 * Lay an aerofoil section into the car's frame as a spanwise wing element.
 *
 * The chord runs backwards (-X) from the leading edge, the span runs along Z,
 * and a positive angle of attack pitches the leading edge up -- which for a
 * downforce wing means the section is upside down relative to an aircraft's.
 */
function wingElement(b, opts) {
  const {
    xLead, y, chord, span, aoaDeg = 0, thickness = 0.12, camber = 0.055,
    zCentre = 0, color = CARBON_LT, taper = 1,
  } = opts;
  const profile = aerofoil(thickness, camber);
  const a = (aoaDeg * Math.PI) / 180;
  const ca = Math.cos(a);
  const sa = Math.sin(a);

  const at = (zFrac) => {
    const z = zCentre + zFrac * span / 2;
    // Taper toward the tips, which is what stops a wing reading as a slab.
    const c = chord * (1 - (1 - taper) * Math.abs(zFrac));
    return profile.map(([px, py]) => {
      // Downforce wing: flip the section so camber points down.
      const cx = px * c;
      const cy = -py * c;
      return [xLead - (cx * ca - cy * sa), y + (cx * sa + cy * ca), z];
    });
  };
  b.loft([at(-1), at(-0.55), at(0), at(0.55), at(1)], color, true, true);
}

// -------------------------------------------------------------------- parts ---

/**
 * Everything rigidly attached to the chassis: tub, nose, wings, pods, hoops,
 * dash and the suspension wishbones. The wishbones are static on purpose --
 * an A-arm does not move when you steer, only the upright rotates about the
 * kingpin, and that rotation lives on the wheel transform.
 */
function buildBody() {
  const b = new Builder();
  const fa = GEO.frontAxle, ra = GEO.rearAxle;

  // ---- tub ----
  // Lofted rather than built from slabs. The nose and the engine cover were
  // already lofted, and the box between them is what made the car read as a
  // stack of panels: a monocoque is a continuous surface that narrows toward
  // the bulkhead, and the join is the whole reason it looks like one piece.
  //
  // The top of this loft sits below the cockpit rim, so the rim and its padding
  // are still added on top and the cockpit stays open.
  //
  // The bodywork lofts are smooth-shaded (`smoothLoft`): the sections are
  // superellipses, and flat facets on a 12-point ring were what made the
  // tub read as a folded-paper model from the chase camera.
  b.smoothLoft([
    section(0.58, 0.295, 0.045, 0.33, 0.45),
    section(0.30, 0.315, 0.040, 0.34, 0.50),
    section(-0.05, 0.325, 0.038, 0.35, 0.55),
    section(-0.35, 0.315, 0.040, 0.36, 0.50),
    section(-0.58, 0.300, 0.045, 0.38, 0.45),
  ], MAROON, false, false);

  // Floor, flat and slightly proud of the tub underside so it catches light.
  b.box(-0.15, 0.038, 0, 1.45, 0.014, 0.60, CARBON);

  for (const side of [-1, 1]) {
    // Cockpit edge is dark foam padding, which is both what the real car has
    // and what keeps the driver's peripheral vision quiet -- a gold rail here
    // sits right where the front wheel is and swamps it.
    b.box(-0.15, 0.372, side * 0.315, 1.45, 0.028, 0.052, CARBON_LT);
    // Livery stripe on the OUTER face instead, where only the chase camera
    // and the outside world see it.
    b.box(-0.15, 0.155, side * 0.340, 1.30, 0.045, 0.010, GOLD);
  }

  // ---- nose: bulkhead tapering to the tip ----
  b.smoothLoft([
    section(0.56, 0.30, 0.05, 0.44, 0.35),
    section(0.78, 0.29, 0.05, 0.40, 0.40),
    section(1.00, 0.22, 0.05, 0.30, 0.50),
    section(1.16, 0.14, 0.05, 0.23, 0.65),
    section(GEO.noseTip, 0.055, 0.06, 0.155, 0.9),
  ], MAROON, true, true);
  // Gold stripe down the spine of the nose.
  b.box(0.92, 0.315, 0, 0.62, 0.02, 0.075, GOLD);

  // ---- engine cover behind the driver ----
  b.smoothLoft([
    section(-0.60, 0.30, 0.05, 0.42, 0.35),
    section(-0.80, 0.30, 0.05, 0.55, 0.35),
    section(-0.98, 0.26, 0.05, 0.48, 0.45),
    section(GEO.tail, 0.20, 0.06, 0.36, 0.6),
  ], MAROON_DK, false, true);

  // ---- side pods ----
  for (const side of [-1, 1]) {
    b.smoothLoft([
      section(0.12, 0.10, 0.09, 0.30, 0.4).map((p) => [p[0], p[1], p[2] + side * 0.44]),
      section(-0.20, 0.13, 0.08, 0.34, 0.4).map((p) => [p[0], p[1], p[2] + side * 0.46]),
      section(-0.52, 0.12, 0.08, 0.32, 0.4).map((p) => [p[0], p[1], p[2] + side * 0.45]),
    ], MAROON, true, true);
  }

  // ---- front wing: main plane, flap and endplates ----
  // Ahead of the nose, which is where a Formula Student front wing sits. It
  // used to start behind the nose tip, so the nose poked through it and the
  // whole front of the car read as one maroon mass.
  wingElement(b, { xLead: GEO.frontWingTip, y: 0.105, chord: 0.22, span: 1.10,
                   aoaDeg: 6, thickness: 0.13, camber: 0.075, taper: 0.92 });
  wingElement(b, { xLead: 1.22, y: 0.175, chord: 0.14, span: 1.02,
                   aoaDeg: 22, thickness: 0.11, camber: 0.085 });
  for (const side of [-1, 1]) {
    endplate(b, side * 0.565, side, [
      [GEO.frontWingTip + 0.02, 0.060], [GEO.frontWingTip + 0.02, 0.175],
      [1.14, 0.250], [1.12, 0.055],
    ], MAROON);
  }

  // ---- rear wing on twin pylons ----
  // Swan-neck mounts: they meet the UPPER surface, which is what a real one
  // does so the pylon does not disturb the working side of the wing.
  for (const side of [-1, 1]) {
    b.polyTube([
      [-0.90, 0.40, side * 0.16],
      [-0.94, 0.72, side * 0.16],
      [-0.90, 0.90, side * 0.16],
    ], 0.016, 6, CARBON);
  }
  wingElement(b, { xLead: -0.83, y: 0.855, chord: 0.26, span: 1.00,
                   aoaDeg: 14, thickness: 0.12, camber: 0.06 });
  wingElement(b, { xLead: -1.00, y: 0.945, chord: 0.17, span: 1.00,
                   aoaDeg: 30, thickness: 0.10, camber: 0.09 });
  for (const side of [-1, 1]) {
    endplate(b, side * 0.505, side, [
      [-0.78, 0.760], [-0.78, 0.905],
      [GEO.rearWingTip, 1.010], [GEO.rearWingTip, 0.735],
    ], MAROON);
  }

  // ---- diffuser ----
  // The floor has to end somewhere, and a flat cut-off is the one thing that
  // most says "this model stopped here". A ramp reads as a car.
  b.quad([-0.80, 0.038, -0.30], [-0.80, 0.038, 0.30],
         [GEO.tail, 0.20, 0.26], [GEO.tail, 0.20, -0.26], CARBON_LT);
  for (const side of [-1, 1]) {
    b.quad([-0.80, 0.038, side * 0.30], [GEO.tail, 0.20, side * 0.26],
           [GEO.tail, 0.05, side * 0.26], [-0.80, 0.030, side * 0.30], CARBON);
  }

  // ---- headrest ----
  // Required by the rules, and from outside it is the thing that turns an open
  // box into a cockpit with someone sitting in it.
  b.smoothLoft([
    section(-0.36, 0.135, 0.42, 0.60, 0.8),
    section(-0.46, 0.150, 0.42, 0.62, 0.8),
    section(-0.54, 0.130, 0.42, 0.58, 0.8),
  ], CARBON_LT, true, true);

  // ---- roll hoops ----
  // Each hoop is one bent tube in a plane of constant x, so it is swept as
  // one continuous surface (see `sweep`) rather than a chain of capped
  // segments: no seams at the joints, and a round normal all the way round.
  const HOOP_PLANE = [1, 0, 0];
  const mainHoop = [];
  for (let i = 0; i <= 14; i++) {
    const t = i / 14, a = Math.PI * t;
    mainHoop.push([-0.40, 0.30 + Math.sin(a) * 0.78, -Math.cos(a) * 0.285]);
  }
  b.sweep(mainHoop, 0.024, 8, METAL, HOOP_PLANE);
  b.tube([-0.40, 0.98, -0.16], [-0.80, 0.42, -0.20], 0.018, 6, METAL);
  b.tube([-0.40, 0.98, 0.16], [-0.80, 0.42, 0.20], 0.018, 6, METAL);

  // Front hoop peaks at 0.56 m -- below the 0.66 m eye point, so the driver
  // looks over it rather than through it.
  const frontHoop = [];
  for (let i = 0; i <= 12; i++) {
    const t = i / 12, a = Math.PI * t;
    frontHoop.push([0.52, 0.18 + Math.sin(a) * 0.38, -Math.cos(a) * 0.275]);
  }
  b.sweep(frontHoop, 0.020, 8, METAL, HOOP_PLANE);

  // ---- dash panel and display, angled back toward the driver ----
  for (const side of [-1, 1]) {
    b.tri([0.44, 0.40, side * 0.27], [0.56, 0.46, side * 0.27], [0.44, 0.50, side * 0.27], CARBON);
  }
  b.box(0.495, 0.455, 0, 0.16, 0.035, 0.54, CARBON);
  // Small, unlit instrument panel. It used to be a 220 mm bright green slab,
  // which glowed straight through the steering wheel's cut-outs and read as a
  // giant screen bolted to the car. An FSAE dash is a little dark display.
  b.box(0.47, 0.487, 0, 0.055, 0.010, 0.085, DASH_LCD);

  // ---- front suspension: upper and lower wishbones, pushrod, upright ----
  for (const side of [-1, 1]) {
    const zOut = side * (GEO.trackFront / 2 - 0.075);
    const lowOut = [fa, 0.115, zOut];
    const upOut = [fa, 0.275, side * (GEO.trackFront / 2 - 0.10)];
    b.tube([fa + 0.20, 0.10, side * 0.28], lowOut, 0.013, 6, METAL);
    b.tube([fa - 0.22, 0.10, side * 0.28], lowOut, 0.013, 6, METAL);
    b.tube([fa + 0.17, 0.30, side * 0.20], upOut, 0.011, 6, METAL);
    b.tube([fa - 0.19, 0.30, side * 0.20], upOut, 0.011, 6, METAL);
    b.tube(lowOut, [fa - 0.10, 0.46, side * 0.13], 0.011, 6, GOLD); // pushrod
    b.tube(lowOut, upOut, 0.017, 6, CARBON_LT);                     // upright
  }

  // ---- rear suspension ----
  for (const side of [-1, 1]) {
    const zOut = side * (GEO.trackRear / 2 - 0.075);
    const lowOut = [ra, 0.115, zOut];
    const upOut = [ra, 0.275, side * (GEO.trackRear / 2 - 0.10)];
    b.tube([ra + 0.24, 0.10, side * 0.24], lowOut, 0.013, 6, METAL);
    b.tube([ra - 0.20, 0.10, side * 0.24], lowOut, 0.013, 6, METAL);
    b.tube([ra + 0.20, 0.30, side * 0.19], upOut, 0.011, 6, METAL);
    b.tube([ra - 0.18, 0.30, side * 0.19], upOut, 0.011, 6, METAL);
    b.tube(lowOut, upOut, 0.017, 6, CARBON_LT);
  }

  return b.mesh();
}

/** Tyre carcass. Separate from the rim so the rim can blur at speed. */
function buildTire() {
  const b = new Builder();
  const R = GEO.tireRadius, HW = GEO.tireHalfWidth;
  // 40 facets: with smooth normals the shading is already round at 28, but
  // the silhouette against the road is a polygon whichever way it is shaded,
  // and at 40 the vertices are 31 mm apart on a 200 mm tyre -- under a pixel
  // from the driver's seat. Four tyres at 40 are ~5100 triangles.
  const SEG = 40;

  // A slick's cross-section is a continuous curve: crowned across the tread,
  // rolling into the shoulder, then a sidewall that bulges before it meets the
  // rim. The previous version was a cylinder with one chamfer at each end,
  // which from any angle reads as a machined edge rather than rubber -- and
  // there are four of these filling the frame, so it is the most-seen surface
  // on the car.
  //
  // Each entry is [z as a fraction of half-width, radius as a fraction of R].
  const profile = [
    [0.00, 1.000],
    [0.45, 0.997],   // crown: the tread is very slightly domed
    [0.70, 0.988],
    [0.85, 0.965],   // shoulder
    [0.94, 0.925],
    [1.00, 0.870],   // sidewall bulge
    [0.97, 0.760],
    [0.90, 0.660],
  ];

  for (const side of [-1, 1]) {
    // The whole meridian is revolved at once so the normal is continuous
    // through the shoulder, where the tread loft meets the sidewall loft.
    const { rings, normals } = revolveZ(profile.map(([zf, rf]) => [side * HW * zf, R * rf]), SEG);
    // Tread and shoulder in tyre black; the sidewall a touch lighter, which is
    // what makes the shoulder line visible at all.
    b.loft(rings.slice(0, 4), TIRE, false, false, normals.slice(0, 4));
    b.loft(rings.slice(3), TIRE_WALL, false, false, normals.slice(3));
    // Close the sidewall onto the rim.
    b.annulusZ(0, 0, side * HW * 0.90, GEO.rimRadius, R * 0.660, SEG, TIRE_WALL);
  }
  return b.mesh();
}

/**
 * Rim and spokes. This is the part that makes wheel rotation legible -- a bare
 * black cylinder spinning at 125 rad/s looks identical to a stationary one.
 */
function buildRim() {
  const b = new Builder();
  const RR = GEO.rimRadius, HW = GEO.tireHalfWidth;
  // The barrel is smooth-shaded; 32 facets so the rim's edge, which is seen
  // against the sidewall from every angle, is as round as the tyre's.
  const SEG = 32;
  b.cylZ(0, 0, 0, RR, HW * 0.95, SEG, RIM, true);
  for (const s of [-1, 1]) {
    const z = s * HW * 0.93;
    b.discZ(0, 0, z, RR * 0.99, SEG, RIM_FACE);
    for (let i = 0; i < 5; i++) {
      const a = (i / 5) * Math.PI * 2;
      const ca = Math.cos(a), sa = Math.sin(a);
      const r0 = 0.030, r1 = RR * 0.93, w = 0.027;
      const px = -sa * w, py = ca * w;
      b.quad(
        [ca * r0 - px, sa * r0 - py, z], [ca * r1 - px, sa * r1 - py, z],
        [ca * r1 + px, sa * r1 + py, z], [ca * r0 + px, sa * r0 + py, z], GOLD);
    }
    b.discZ(0, 0, z + s * 0.004, 0.036, 12, [0.20, 0.21, 0.23]); // hub nut
  }
  return b.mesh();
}

/**
 * Steering wheel, built in its own frame with the rotation axis along +Z and
 * the wheel face in XY. The renderer tilts that frame onto the column.
 * Includes the driver's gloves at 9 and 3 -- an FSAE driver never takes a hand
 * off a 260 mm wheel, and having them there sells the cockpit.
 */
function buildSteeringWheel() {
  const b = new Builder();

  // Modelled from the team's own wheel -- the render in
  // `helios/packages/widgets/src/steering-wheel/assets/wheel.png`, which is
  // the wheel that is actually bolted to the car.
  //
  // Reading that image: a near-square carbon plate with two big kidney
  // cut-outs, a suede D-grip wrapping the outside of each cut-out, two gold
  // buttons in the top corners, two amber rockers either side of the centre,
  // the Sparky mark in the middle, and three gold rotaries along the bottom.
  // There is NO screen on it -- the dash is a separate Strada unit on the
  // scuttle, which is why the HUD draws one there and not here.
  //
  // Everything is in metres in the wheel's own plane: +x right, +y up, and -z
  // toward the driver, so a part that stands proud has a more negative z.
  const HW = GEO.steerHalfWidth;    // 104 mm: the outside of the grips
  const HH = GEO.steerHalfHeight;   //  72 mm: the top of the plate
  const PT = 0.009;                 // plate thickness
  const PZ = -PT / 2;

  // The grips sit at the outer edge; the plate's bars run out to their centres
  // so there is no gap between plate and grip.
  const GRIP_R = 0.016;
  const GRIP_CX = HW - GRIP_R;
  const PLATE_HW = GRIP_CX;

  // The cut-outs. Everything else about the plate is the material left around
  // them, which keeps this low-poly and needs no CSG.
  const COL_HW = 0.026;   // centre column half-width
  const TOP_Y = 0.036;    // underside of the top bar
  const BOT_Y = -0.042;   // top of the bottom bar

  const bar = (cx, cy, w, h, colour) => b.box(cx, cy, PZ, w, h, PT, colour);

  bar(0, (TOP_Y + HH) / 2, PLATE_HW * 2, HH - TOP_Y, CARBON_PLATE);       // top
  bar(0, (-HH + BOT_Y) / 2, PLATE_HW * 2, BOT_Y + HH, CARBON_PLATE);      // bottom
  bar(0, (BOT_Y + TOP_Y) / 2, COL_HW * 2, TOP_Y - BOT_Y, CARBON_PLATE);   // centre

  // The lightening slot across the top, and the shallow scallop under the
  // logo -- both are recesses in the reference, so both are darker insets
  // rather than raised parts.
  b.box(0, HH - 0.013, PZ - PT * 0.30, 0.080, 0.010, PT * 0.55, CARBON_DARK);
  b.box(0, BOT_Y + 0.008, PZ - PT * 0.30, 0.062, 0.007, PT * 0.55, CARBON_DARK);

  // ---- grips ----
  //
  // A D in plan: an arc bulging outward from the cut-out, its ends turning
  // back in toward the centre column, which is the shape your hand actually
  // wraps. Swept as one continuous surface -- see `sweepXY`, and see the
  // comment there for why the obvious `polyTube` is wrong for a curve.
  const GZ = -0.016;          // stands proud toward the driver
  const ARC_A = 0.0231;       // outward bulge
  // Sized so the grip's top ends stop just under the plate's top bar: in the
  // reference the two gold buttons sit on that bar, OUTBOARD and above the
  // grips, and a taller arc swallows them.
  const ARC_B = 0.0496;       // vertical half-extent
  const ARC_CX = GRIP_CX - ARC_A;
  const SPAN = (130 * Math.PI) / 180;
  const N = 15;
  for (const side of [-1, 1]) {
    const path = [];
    for (let i = 0; i < N; i++) {
      const th = SPAN - (2 * SPAN * i) / (N - 1);
      path.push([side * (ARC_CX + ARC_A * Math.cos(th)), ARC_B * Math.sin(th) - 0.004, GZ]);
    }
    b.sweepXY(side < 0 ? path.slice().reverse() : path, GRIP_R, 12, GRIP);
  }

  // ---- gold buttons, top corners ----
  // Out at the corners of the top bar, clear of the grips below them -- which
  // is where they are on the real wheel and, less romantically, the only place
  // a 23 mm button is not swallowed by a 32 mm grip.
  for (const side of [-1, 1]) {
    b.cylZ(side * 0.076, HH - 0.018, -0.013, 0.0115, 0.005, 12, GOLD, true);
    b.discZ(side * 0.076, HH - 0.018, -0.018, 0.0115, 12, GOLD_LIT);
  }

  // ---- amber rockers, either side of the mark ----
  for (const side of [-1, 1]) {
    b.box(side * 0.021, 0.021, -0.011, 0.011, 0.021, 0.005, AMBER);
    // The lit face, so they read as switches and not as two orange smudges.
    b.box(side * 0.021, 0.021, -0.014, 0.008, 0.017, 0.001, AMBER_LIT);
  }

  // ---- three gold rotaries along the bottom ----
  for (const side of [-1, 0, 1]) {
    b.cylZ(side * 0.034, -0.058, -0.015, 0.0125, 0.007, 12, GOLD, true);
    b.discZ(side * 0.034, -0.058, -0.022, 0.0125, 12, GOLD_LIT);
  }

  // ---- the Sparky mark ----
  //
  // A pitchfork, not a blank plate. It is about forty pixels on screen, so
  // the actual devil is hopeless, but three tines and a shaft are instantly
  // the right mark and a white rectangle is instantly nothing.
  const MZ = -0.012;
  const tineH = 0.019;
  for (const side of [-1, 0, 1]) {
    b.box(side * 0.0105, 0.006 + tineH / 2, MZ, 0.0045, tineH, 0.003, EMBLEM);
  }
  b.box(0, 0.005, MZ, 0.027, 0.005, 0.003, EMBLEM);     // crossbar
  b.box(0, -0.008, MZ, 0.0055, 0.022, 0.003, EMBLEM);   // shaft

  return b.mesh();
}

/**
 * The driver's gloves: fists at 9 and 3 wrapped round the grips, in the
 * WHEEL's frame so they stay on the grips at any lock. A mesh of their own,
 * drawn with the wheel's transform, because the driver is for the outside
 * cameras only -- from the seat the real driver's own hands are on the real
 * rim, and a second pair on the screen was worse than none.
 *
 * The forearms are NOT here either: a forearm rigid with the wheel swings
 * up into the sky at 90 degrees of lock. They are separate meshes
 * (`buildForearm`, `buildUpperArm`) that the renderer poses every frame
 * from the glove positions `handsInWheelFrame` reports and the shoulders
 * in `driverPose`, so the elbow stays down in the tub whatever the wheel
 * is doing.
 */
function buildGloves() {
  const b = new Builder();
  const hands = handsInWheelFrame();
  for (const [gx, gy, gz] of hands) {
    b.smoothLoft([
      ovalRing(gy - 0.047, gx, gz, 0.012, 0.012, 2.4, 12),
      ovalRing(gy - 0.041, gx, gz, 0.024, 0.024, 2.4, 12),
      ovalRing(gy - 0.015, gx, gz, 0.029, 0.028, 2.6, 12),
      ovalRing(gy + 0.017, gx, gz, 0.029, 0.028, 2.6, 12),
      ovalRing(gy + 0.041, gx, gz, 0.024, 0.024, 2.4, 12),
      ovalRing(gy + 0.047, gx, gz, 0.012, 0.012, 2.4, 12),
    ], GLOVE, true, true);
  }
  return b.mesh();
}

// ------------------------------------------------------------------ driver ---

/**
 * Where the driver sits, in the chassis frame. Everything about the figure
 * hangs off the eye point the cockpit camera uses, so the helmet is built
 * AROUND the camera: the eye is 30 mm below and 40 mm ahead of the helmet's
 * centre, inside the shell, and the cockpit view skips the `driver` and
 * `helmet` meshes so it never sees the inside of its own head.
 *
 * The arms are not in either mesh. The gloves ride on the steering wheel;
 * the upper arm and forearm are canonical bones (`buildUpperArm`,
 * `buildForearm`) that the renderer places every frame with a two-bone IK
 * from the shoulders here to the gloves wherever the wheel has taken them,
 * elbow pulled toward `elbowHint`. `buildCarMeshes` hands those numbers to
 * the renderer as `arms` so nothing about the pose is typed twice.
 */
/** The eye point the driver figure (torso, neck, straps) is authored around. */
const FIGURE_EYE = [-0.15, 0.70];

function driverPose(params) {
  const eyeX = params?.eyeAheadOfCgM ?? SDM26.eyeAheadOfCgM;
  const eyeY = params?.eyeHeightM ?? SDM26.eyeHeightM;
  // The figure was authored around an eye at (-0.15, 0.70). Wherever the eye
  // is now -- the eye-height slider, the CAD car's seat -- the torso, neck
  // and shoulders go with it, so the helmet never floats off the body.
  const off = [eyeX - FIGURE_EYE[0], eyeY - FIGURE_EYE[1]];
  return {
    figureOffset: off,
    helmetCentre: [eyeX - 0.04, eyeY + 0.03, 0],
    /** Shell half-sizes: a road helmet is longer than it is wide. */
    helmetRadii: [0.135, 0.13, 0.12],
    /** Shoulder joint, driver's right; mirror z for the left. Inside the
     *  torso, so the upper arm's root is buried and only the sleeve shows. */
    shoulder: [-0.115 + off[0], 0.485 + off[1], 0.19],
    /** Bone lengths: shoulder to elbow, and elbow to the centre of the fist
     *  (the last 60 mm of the forearm is inside the glove). Together they
     *  are 40 mm longer than the farthest the wheel can carry a hand, so
     *  the elbow always has somewhere to bend to. */
    lUpper: 0.28,
    lFore: 0.30,
    /** Where the elbow goes: down and a little outboard, against the tub.
     *  z is for the driver's right; the renderer mirrors it. */
    elbowHint: [0, -1, 0.35],
    /** Neck: a short thick tube from the shoulders up into the shell. With
     *  a HANS on, almost none of a driver's neck shows. */
    neckBase: [-0.17, 0.53, 0],
    neckTop: [-0.18, 0.645, 0],
  };
}

/**
 * The centres of the two fists in the steering wheel's own frame,
 * [left, right]. Wheel +x is the driver's LEFT; the grips sit at
 * +-(steerHalfWidth - grip radius), and the fist is a touch outboard of
 * the grip centre and proud of it toward the driver (-z).
 */
function handsInWheelFrame() {
  const gx = GEO.steerHalfWidth - 0.016 + 0.004;
  return [[gx, -0.005, -0.024], [-gx, -0.005, -0.024]];
}

/**
 * Horizontal superellipse ring at height y, centred on (cx, cz), for
 * lofting a body: `e` is the exponent, 2 for an ellipse, higher for a
 * squarer section (a chest is about 2.6).
 */
function ovalRing(y, cx, cz, hx, hz, e, n = 16) {
  const pts = [];
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2;
    const c = Math.cos(a), s = Math.sin(a);
    pts.push([cx + hx * Math.sign(c) * Math.pow(Math.abs(c), 2 / e), y,
              cz + hz * Math.sign(s) * Math.pow(Math.abs(s), 2 / e)]);
  }
  return pts;
}

/**
 * A patch of an ellipsoid, with its analytic normal (x/rx^2, y/ry^2, z/rz^2)
 * on every vertex. Latitude is measured from the equator toward +Y,
 * longitude from +X toward +Z. A longitude span of a full turn closes the
 * ring; a latitude that reaches a pole collapses that row to triangles
 * instead of emitting degenerate quads. `offset` pushes the patch out along
 * its normal, which is how the visor and the trim sit proud of the shell
 * without z-fighting it. Returns the point grid, row 0 first.
 */
function ellipsoidPatch(b, c, r, lat0, lat1, lon0, lon1, nLat, nLon, color, offset = 0) {
  const closed = Math.abs(lon1 - lon0 - Math.PI * 2) < 1e-9;
  const cols = closed ? nLon : nLon + 1;
  const P = [], N = [];
  for (let i = 0; i <= nLat; i++) {
    const la = lat0 + ((lat1 - lat0) * i) / nLat;
    const row = [], rn = [];
    for (let j = 0; j < cols; j++) {
      const lo = lon0 + ((lon1 - lon0) * j) / nLon;
      const ux = Math.cos(la) * Math.cos(lo), uy = Math.sin(la), uz = Math.cos(la) * Math.sin(lo);
      const n = norm([ux / r[0], uy / r[1], uz / r[2]]);
      row.push([c[0] + ux * r[0] + n[0] * offset, c[1] + uy * r[1] + n[1] * offset, c[2] + uz * r[2] + n[2] * offset]);
      rn.push(n);
    }
    P.push(row);
    N.push(rn);
  }
  const atPole = (la) => Math.abs(Math.abs(la) - Math.PI / 2) < 1e-9;
  for (let i = 0; i < nLat; i++) {
    for (let j = 0; j < nLon; j++) {
      const k = (j + 1) % cols;
      if (i === nLat - 1 && atPole(lat1)) {
        b.triN(P[i][j], P[i][k], P[i + 1][k], N[i][j], N[i][k], N[i + 1][k], color);
      } else if (i === 0 && atPole(lat0)) {
        b.triN(P[i][j], P[i + 1][k], P[i + 1][j], N[i][j], N[i + 1][k], N[i + 1][j], color);
      } else {
        b.quadN(P[i][j], P[i][k], P[i + 1][k], P[i + 1][j], N[i][j], N[i][k], N[i + 1][k], N[i + 1][j], color);
      }
    }
  }
  return P;
}

/**
 * A round bar that changes radius along its length -- a sleeve, a forearm,
 * a neck. Same frame construction as `tube`; the normal is the radial
 * direction leaned along the axis by the cone's half-angle, so a taper
 * shades as a cone and not as a cylinder with a kink at each end.
 */
function taperTube(b, p0, p1, r0, r1, segs, color) {
  const d = sub(p1, p0);
  const L = Math.hypot(d[0], d[1], d[2]) || 1;
  const axis = [d[0] / L, d[1] / L, d[2] / L];
  const helper = Math.abs(axis[1]) > 0.9 ? [1, 0, 0] : [0, 1, 0];
  const u = norm(cross(helper, axis));
  const v = cross(axis, u);
  const lean = Math.atan2(r0 - r1, L);
  const cl = Math.cos(lean), sl = Math.sin(lean);
  const ring = (p, r) => {
    const pts = [], ns = [];
    for (let i = 0; i < segs; i++) {
      const a = (i / segs) * Math.PI * 2;
      const c = Math.cos(a), s = Math.sin(a);
      const rad = [u[0] * c + v[0] * s, u[1] * c + v[1] * s, u[2] * c + v[2] * s];
      pts.push([p[0] + rad[0] * r, p[1] + rad[1] * r, p[2] + rad[2] * r]);
      ns.push([rad[0] * cl + axis[0] * sl, rad[1] * cl + axis[1] * sl, rad[2] * cl + axis[2] * sl]);
    }
    return { pts, ns };
  };
  const A = ring(p0, r0), B = ring(p1, r1);
  b.loft([A.pts, B.pts], color, true, true, [A.ns, B.ns]);
}

/**
 * The torso, as horizontal sections from well inside the tub up to the
 * base of the neck. The line of the sections leans back: a Formula
 * Student driver reclines at about 45 degrees, so the chest at the
 * cockpit rim is ahead of the shoulders, and the shoulders are ahead of
 * the headrest.
 *
 * Kept as data so the harness can be laid on the surface it describes.
 */
const TORSO = [
  { y: 0.26, cx: 0.05, hx: 0.11, hz: 0.18 },
  { y: 0.37, cx: 0.00, hx: 0.115, hz: 0.20 },
  { y: 0.44, cx: -0.05, hx: 0.115, hz: 0.215 },
  { y: 0.495, cx: -0.10, hx: 0.11, hz: 0.225 },
  { y: 0.52, cx: -0.125, hx: 0.095, hz: 0.21 },
  { y: 0.535, cx: -0.145, hx: 0.075, hz: 0.15 },
  { y: 0.545, cx: -0.16, hx: 0.06, hz: 0.08 },
];
const TORSO_E = 2.6;

/** The x of the torso's FRONT surface at height y, `z` off the centreline. */
function torsoFront(y, z) {
  let lo = TORSO[0], hi = TORSO[TORSO.length - 1];
  for (let i = 0; i < TORSO.length - 1; i++) {
    if (y >= TORSO[i].y && y <= TORSO[i + 1].y) { lo = TORSO[i]; hi = TORSO[i + 1]; break; }
  }
  const t = hi.y === lo.y ? 0 : (y - lo.y) / (hi.y - lo.y);
  const cx = lo.cx + (hi.cx - lo.cx) * t;
  const hx = lo.hx + (hi.hx - lo.hx) * t;
  const hz = lo.hz + (hi.hz - lo.hz) * t;
  const f = Math.max(0, 1 - Math.pow(Math.min(1, Math.abs(z) / hz), TORSO_E));
  return cx + hx * Math.pow(f, 1 / TORSO_E);
}

/**
 * A flat strip of webbing along a path in a plane of constant z, lifted
 * `lift` off the surface it follows. Lofted as rectangular rings with flat
 * normals, because a strap's edges are real edges and a smooth normal on a
 * 5 mm rectangle would shade it as a tube.
 */
function strap(b, path, z, width, lift, color) {
  const rings = path.map((p, i) => {
    const prev = path[Math.max(0, i - 1)], next = path[Math.min(path.length - 1, i + 1)];
    const t = norm(sub(next, prev));
    const n = norm(cross([0, 0, 1], t));      // in-plane normal, "outward"
    const c = [p[0] + n[0] * lift, p[1] + n[1] * lift, z];
    const th = 0.0025, hw = width / 2;
    return [
      [c[0] - n[0] * th, c[1] - n[1] * th, z - hw],
      [c[0] - n[0] * th, c[1] - n[1] * th, z + hw],
      [c[0] + n[0] * th, c[1] + n[1] * th, z + hw],
      [c[0] + n[0] * th, c[1] + n[1] * th, z - hw],
    ];
  });
  b.loft(rings, color, true, true);
}

/**
 * The helmet, on its own so the renderer can give it a clearcoat: the shell
 * is the one glossy thing on the driver, and a suit with a clearcoat reads
 * as vinyl.
 *
 * An ellipsoid, a touch longer than it is wide, open at the bottom where
 * the neck goes in. The bottom band below the visor line is maroon, which
 * from the chase camera is what reads as a chin bar; the visor is a dark
 * patch across the eye line, proud of the shell by 3 mm; a maroon stripe
 * runs from above the visor over the crown to the nape. The open bottom
 * gets a flat liner-black cap so the shell does not read as hollow from
 * the walkaround camera at a low angle.
 */
function buildHelmet(params) {
  const b = new Builder();
  const { helmetCentre: c, helmetRadii: r } = driverPose(params);
  const D = Math.PI / 180;
  const TAU = Math.PI * 2;
  const LON = 20;
  const BOTTOM = -62 * D, BAND = -30 * D;

  // Shell: maroon band, then white to the crown.
  const bottomRing = ellipsoidPatch(b, c, r, BOTTOM, BAND, 0, TAU, 2, LON, HELMET_TRIM)[0];
  ellipsoidPatch(b, c, r, BAND, 90 * D, 0, TAU, 9, LON, HELMET);
  b.cap(bottomRing, HELMET_LINER);

  // Visor: centred on the eye line, 30 mm below the shell's centre, across
  // the front 140 degrees.
  ellipsoidPatch(b, c, r, -33 * D, 7 * D, -70 * D, 70 * D, 3, 8, VISOR, 0.003);

  // Crown stripe: two meridians at z = +-18 mm from just above the visor,
  // over the top, down to the nape.
  const zs = 0.018;
  const k = Math.sqrt(1 - (zs / r[2]) ** 2);
  const rows = [];
  for (const z of [-zs, zs]) {
    const row = [], rn = [];
    for (let i = 0; i <= 14; i++) {
      const phi = 14 * D + ((168 - 14) * D * i) / 14;
      const ux = k * Math.cos(phi), uy = k * Math.sin(phi), uz = z / r[2];
      const n = norm([ux / r[0], uy / r[1], uz / r[2]]);
      row.push([c[0] + ux * r[0] + n[0] * 0.003, c[1] + uy * r[1] + n[1] * 0.003, c[2] + z + n[2] * 0.003]);
      rn.push(n);
    }
    rows.push({ row, rn });
  }
  for (let i = 0; i < 14; i++) {
    const A = rows[0], B = rows[1];
    b.quadN(A.row[i], A.row[i + 1], B.row[i + 1], B.row[i],
            A.rn[i], A.rn[i + 1], B.rn[i + 1], B.rn[i], HELMET_TRIM);
  }
  return b.mesh();
}

/**
 * The figure in the seat, minus the helmet and minus the forearms: torso,
 * neck, HANS collar, shoulder straps and the upper-arm sleeves. All matte,
 * all rigid with the chassis, all skipped by the cockpit camera.
 */
function buildDriver(params) {
  const b = new Builder();
  const pose = driverPose(params);

  // ---- torso ----
  // Open at the bottom (inside the tub, never seen) and at the neck (the
  // neck tube plugs it).
  b.smoothLoft(TORSO.map((s) => ovalRing(s.y, s.cx, 0, s.hx, s.hz, TORSO_E)), SUIT, false, false);

  // ---- neck, up into the shell ----
  taperTube(b, pose.neckBase, pose.neckTop, 0.066, 0.058, 12, SUIT);

  // ---- HANS collar ----
  // The yoke round the back of the neck, sitting on the shoulders, and its
  // two legs forward over them. A horizontal-plane sweep, so it lofts as one
  // surface.
  const hansC = [pose.neckBase[0] - 0.005, 0.552, 0];
  const yoke = [];
  for (let i = 0; i <= 10; i++) {
    const a = (95 + (170 * i) / 10) * (Math.PI / 180);   // 95 deg round the back to 265
    yoke.push([hansC[0] + Math.cos(a) * 0.085, hansC[1], hansC[2] + Math.sin(a) * 0.085]);
  }
  b.sweep(yoke, 0.021, 8, CARBON, [0, 1, 0]);
  for (const side of [-1, 1]) {
    b.box(-0.19, 0.544, side * 0.105, 0.11, 0.016, 0.05, CARBON);
  }

  // ---- shoulder straps ----
  // Over the shoulders from the harness bar behind the seat, then down the
  // chest to the lap. The chest run is laid on the torso's front surface.
  for (const side of [-1, 1]) {
    const z = side * 0.09;
    const path = [
      [-0.35, 0.485, z],
      [-0.25, 0.52, z],
      [-0.17, 0.54, z],
      [torsoFront(0.53, z), 0.53, z],
      [torsoFront(0.49, z), 0.49, z],
      [torsoFront(0.45, z), 0.45, z],
      [torsoFront(0.40, z), 0.40, z],
      [torsoFront(0.34, z), 0.34, z],
    ];
    strap(b, path, z, 0.075, 0.004, STRAP);
  }

  // Move the whole figure with the eye (see driverPose).
  const m = b.mesh();
  const [dx, dy] = pose.figureOffset;
  if (dx || dy) for (let i = 0; i < m.position.length; i += 3) { m.position[i] += dx; m.position[i + 1] += dy; }
  return m;
}

/**
 * The upper arm as a canonical bone: shoulder joint at the origin, elbow at
 * x = lUpper, along +X. A sleeve that is thicker at the shoulder, with a
 * ball at the elbow so the forearm can meet it at any angle without a
 * visible corner. The renderer orients it per frame.
 */
function buildUpperArm(params) {
  const b = new Builder();
  const { lUpper } = driverPose(params);
  const R_SHOULDER = 0.052, R_ELBOW = 0.040;
  taperTube(b, [0, 0, 0], [lUpper, 0, 0], R_SHOULDER, R_ELBOW, 12, SUIT);
  ellipsoidPatch(b, [lUpper, 0, 0], [R_ELBOW, R_ELBOW, R_ELBOW],
                 -Math.PI / 2, Math.PI / 2, 0, Math.PI * 2, 6, 8, SUIT);
  return b.mesh();
}

/**
 * The forearm as a canonical bone: elbow at the origin, the centre of the
 * fist at x = lFore, along +X. Suit sleeve tapering to the wrist, then the
 * glove's cuff for the last 60 mm, which is the part inside the fist on the
 * wheel.
 */
function buildForearm(params) {
  const b = new Builder();
  const { lFore } = driverPose(params);
  const CUFF = 0.06;
  taperTube(b, [0, 0, 0], [lFore - CUFF, 0, 0], 0.038, 0.029, 12, SUIT);
  taperTube(b, [lFore - CUFF, 0, 0], [lFore, 0, 0], 0.031, 0.027, 12, GLOVE);
  return b.mesh();
}

/**
 * What the renderer needs to pose the arms: shoulders in the chassis
 * frame, fists in the STEERING WHEEL's local frame (they move with it),
 * bone lengths and the elbow hint, each as [left, right].
 */
function armRig(params) {
  const pose = driverPose(params);
  const [sx, sy, sz] = pose.shoulder;
  const [hx, hy, hz] = pose.elbowHint;
  return {
    shoulders: [[sx, sy, -sz], [sx, sy, sz]],
    hands: handsInWheelFrame(),
    lUpper: pose.lUpper,
    lFore: pose.lFore,
    elbowHints: [[hx, hy, -hz], [hx, hy, hz]],
  };
}

/**
 * The dash unit's CASE -- the black shell the screen sits in.
 *
 * Only the case is here. The screen is a textured quad the renderer draws
 * with its own unlit program, because it is a backlit LCD: shading it with the
 * sun would make it darker in shadow, which is precisely wrong.
 */
function buildDashCase() {
  const b = new Builder();
  const sw = GEO.dashHalfWidth;        // the display
  const sh = GEO.dashHalfHeight;
  const cw = GEO.dashCaseHalfWidth;    // the case around it
  const ch = GEO.dashCaseHalfHeight;
  const D = 0.024;                     // 24 mm deep, as the real one is

  // Shell, just behind the display plane.
  b.box(0, 0, D / 2 + 0.001, cw * 2, ch * 2, D, DASH_CASE);

  // The bezel: four bars of case material around the display, standing a
  // fraction proud so the glass is recessed rather than painted on. The side
  // bars are wide -- 14.5 mm -- because that is where the buttons live.
  const lip = 0.0022;
  const sideW = cw - sw;
  const topH = ch - sh;
  b.box(0, sh + topH / 2, -lip / 2, cw * 2, topH, lip, DASH_BEZEL);
  b.box(0, -sh - topH / 2, -lip / 2, cw * 2, topH, lip, DASH_BEZEL);
  for (const side of [-1, 1]) {
    b.box(side * (sw + sideW / 2), 0, -lip / 2, sideW, sh * 2, lip, DASH_BEZEL);
    // Three buttons down each side, which is what those bars are for.
    for (let i = -1; i <= 1; i++) {
      b.box(side * (sw + sideW / 2), i * 0.016, -lip - 0.0015,
            sideW * 0.55, 0.005, 0.003, DASH_BUTTON);
    }
  }

  // The stalk down to the column shroud.
  b.box(0, -ch - 0.020, D * 0.55, 0.024, 0.026, 0.016, DASH_CASE);
  return b.mesh();
}

/** The stations the body is authored at, before any stretch. */
const BASE = {
  a: GEO.frontAxle,
  b: -GEO.rearAxle,
  trackFront: GEO.trackFront,
  trackRear: GEO.trackRear,
};

/**
 * Stretch the authored body onto a different wheelbase and track.
 *
 * The body is modelled once at SDM26's real stations, then mapped: forward of
 * the CG everything scales by a/a0 and behind it by b/b0, so moving the
 * wheelbase OR the weight distribution moves the nose and tail the way the
 * axles moved. Laterally only the outboard half scales -- widening the track
 * lengthens the wishbones and pushes the wheels out, it does not make the tub
 * wider, which is what actually happens when a team changes track.
 *
 * This is a rendering approximation, not a re-body: at large changes the
 * bodywork is stretched rather than redesigned. It keeps the car coherent with
 * the physics, which is the point.
 */
function stretchBody(mesh, p) {
  const a = lengthToFrontAxle(p);
  const b = lengthToRearAxle(p);
  const fx = a / BASE.a;
  const rx = b / BASE.b;
  const fz = p.trackFrontM / BASE.trackFront;
  const rz = p.trackRearM / BASE.trackRear;
  const INBOARD = 0.33; // tub half-width: anything inside this does not scale

  const pos = mesh.position;
  for (let i = 0; i < pos.length; i += 3) {
    const x = pos[i];
    const zScale = x >= 0 ? fz : rz;
    pos[i] = x >= 0 ? x * fx : x * rx;
    const z = pos[i + 2];
    if (Math.abs(z) > INBOARD) {
      pos[i + 2] = Math.sign(z) * (INBOARD + (Math.abs(z) - INBOARD) * zScale);
    }
  }
  return mesh;
}

/** Wheel hub positions for the live parameters. */
export function hubsFor(p) {
  const a = lengthToFrontAxle(p);
  const b = -lengthToRearAxle(p);
  return [
    { name: "FL", x: a, y: p.tireRadiusM, z: -p.trackFrontM / 2, front: true },
    { name: "FR", x: a, y: p.tireRadiusM, z: p.trackFrontM / 2, front: true },
    { name: "RL", x: b, y: p.tireRadiusM, z: -p.trackRearM / 2, front: false },
    { name: "RR", x: b, y: p.tireRadiusM, z: p.trackRearM / 2, front: false },
  ];
}

/** Chassis footprint for cone strikes, derived from the live geometry. */
export function bodyBoxFor(p) {
  return {
    front: lengthToFrontAxle(p) + FRONT_OVERHANG,
    rear: lengthToRearAxle(p) + REAR_OVERHANG,
    // Track is between tyre centrelines, so overall width adds one tyre.
    halfWidth: Math.max(p.trackFrontM, p.trackRearM) / 2 + GEO.tireHalfWidth,
  };
}

// Overhang beyond each axle, from the authored body.
const FRONT_OVERHANG = Math.max(GEO.noseTip, GEO.frontWingTip) - GEO.frontAxle;
const REAR_OVERHANG = GEO.rearAxle - Math.min(GEO.tail, GEO.rearWingTip);

export function buildCarMeshes(params) {
  const body = buildBody();
  return {
    body: params ? stretchBody(body, params) : body,
    tire: buildTire(),
    rim: buildRim(),
    steeringWheel: buildSteeringWheel(),
    gloves: buildGloves(),
    dashCase: buildDashCase(),
    driver: buildDriver(params),
    helmet: buildHelmet(params),
    upperArm: buildUpperArm(params),
    forearm: buildForearm(params),
    // Not a mesh: the pose data the renderer's arm IK reads (see `armRig`).
    arms: armRig(params),
  };
}

/** Baseline hub positions. `hubsFor(params)` is the live version. */
export const HUBS = [
  { name: "FL", x: GEO.frontAxle, y: GEO.tireRadius, z: -GEO.trackFront / 2, front: true },
  { name: "FR", x: GEO.frontAxle, y: GEO.tireRadius, z: GEO.trackFront / 2, front: true },
  { name: "RL", x: GEO.rearAxle, y: GEO.tireRadius, z: -GEO.trackRear / 2, front: false },
  { name: "RR", x: GEO.rearAxle, y: GEO.tireRadius, z: GEO.trackRear / 2, front: false },
];
