// Suspension kinematics for a CAD car: the arms, pushrod, rocker and
// coilover follow the wheel.
//
// The renderer hangs the wheels off a level "axle" frame and rolls and pitches
// the body about its CG over them, so every corner sees the wheel move
// relative to the body. A CAD car baked as one rigid mesh then tears its own
// suspension apart in every corner: the wishbones roll with the tub while the
// wheels stay on the road. This solves the linkage from the team's OptimumK
// hardpoints each frame instead, so the parts stay joined:
//
//   upright     translates with the wheel's hub (and steers, at the front)
//   wishbones   rotate about their chassis pivot axes to meet the upright
//   tie rod     aims from its chassis end at the upright's steering arm
//   pushrod     rides its wishbone at the outboard end, meets the rocker
//   rocker      turns about its pivot until the pushrod is its own length
//   coilover    body stays on its chassis eye, aims at the rocker; the shaft
//               slides out of the body; the spring is scaled between them
//   drop link   (front ARB) aims from the blade at the rocker
//
// All in the chassis frame the body is drawn in (sim: +x forward, +y up,
// +z right). Every result is a 4x4 column-major model matrix that takes the
// part from its as-modelled (static) position to where it is now.

const V = {
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  mul: (a, k) => [a[0] * k, a[1] * k, a[2] * k],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  len: (a) => Math.hypot(a[0], a[1], a[2]),
  unit: (a) => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; },
};

/** 3x3 rotation about unit axis `k` by `th` (Rodrigues), row-major. */
function axisAngle(k, th) {
  const c = Math.cos(th), s = Math.sin(th), t = 1 - c;
  const [x, y, z] = k;
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c,
  ];
}
const IDENT3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
/** The shortest rotation taking direction `a` onto direction `b`. */
function between(a, b) {
  const u = V.unit(a), v = V.unit(b);
  const ax = V.cross(u, v);
  const s = V.len(ax), c = V.dot(u, v);
  if (s < 1e-9) return IDENT3;
  return axisAngle(V.mul(ax, 1 / s), Math.atan2(s, c));
}
const apply3 = (R, p) => [R[0] * p[0] + R[1] * p[1] + R[2] * p[2], R[3] * p[0] + R[4] * p[1] + R[5] * p[2], R[6] * p[0] + R[7] * p[1] + R[8] * p[2]];
const mul3 = (A, B) => {
  const o = new Array(9);
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) o[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
  return o;
};

/** out = the affine map p -> to + L(p - from), as a column-major mat4. */
function place(out, L, from, to) {
  const t = V.sub(to, apply3(L, from));
  out[0] = L[0]; out[1] = L[3]; out[2] = L[6]; out[3] = 0;
  out[4] = L[1]; out[5] = L[4]; out[6] = L[7]; out[7] = 0;
  out[8] = L[2]; out[9] = L[5]; out[10] = L[8]; out[11] = 0;
  out[12] = t[0]; out[13] = t[1]; out[14] = t[2]; out[15] = 1;
  return out;
}
const pt = (m, p) => [
  m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
  m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
  m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
];

/**
 * Turn a wishbone about its chassis pickups a->b until `err(where its ball
 * joint p0 now is)` is zero: Newton on the angle, warm-started from the last
 * frame's (`st[key]`). Writes the wishbone's matrix and returns the joint.
 */
function solveHinge(out, a, b, p0, err, st, key) {
  const k = V.unit(V.sub(b, a)), r0 = V.sub(p0, a);
  const at = (th) => V.add(a, apply3(axisAngle(k, th), r0));
  let th = st[key] ?? 0;
  for (let i = 0; i < 10; i++) {
    const y = err(at(th));
    if (Math.abs(y) < 1e-8) break;
    const dy = (err(at(th + 1e-5)) - y) / 1e-5;
    if (!Number.isFinite(dy) || Math.abs(dy) < 1e-9) break;
    th = Math.max(-0.7, Math.min(0.7, th - y / dy));
  }
  if (Number.isFinite(th)) st[key] = th;
  place(out, axisAngle(k, st[key]), a, a);
  return at(st[key]);
}

export const RIG_ROLES = ["upright", "uca", "lca", "tie", "pushrod", "rocker", "damper_top", "damper_rod", "spring", "droplink"];

export class SuspensionRig {
  /**
   * @param corners {fl: {UF, UA, LF, LA, UB, LB, TC, TU, PP, ATT, PIV, ROD,
   *   COI, WC, pushOn, DLB?, DLT?}, fr, rl, rr} in the chassis frame.
   */
  constructor(corners) {
    this.corners = corners;
    this.state = {};
    for (const [name, hp] of Object.entries(corners)) {
      const n = V.unit(V.cross(V.sub(hp.ROD, hp.PIV), V.sub(hp.COI, hp.PIV)));
      this.state[name] = {
        phi: 0,
        rockerAxis: n,
        pushLen: V.len(V.sub(hp.ROD, hp.PP)),
        tieLen: V.len(V.sub(hp.TU, hp.TC)),
        thL: 0, thU: 0,
        coilLen: V.len(V.sub(hp.COI, hp.ATT)),
        mats: Object.fromEntries(RIG_ROLES.map((r) => [r, new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])])),
        /** Coilover length now over static: < 1 is compression. */
        coilRatio: 1,
        rockerDeg: 0,
      };
    }
  }

  /**
   * Pose one corner.
   * @param name   "fl" | "fr" | "rl" | "rr"
   * @param d      where the wheel centre has moved to relative to the body,
   *               minus where it was modelled: [dx, dy, dz], chassis frame
   * @param steer  road-wheel steer angle about the vertical (rad), front only
   */
  solve(name, d, steer = 0) {
    const hp = this.corners[name], st = this.state[name], M = st.mats;
    // Lower wishbone sets the height: turn it about its pickups until its
    // ball joint has risen with the wheel. The upper one then turns until the
    // two ball joints are their own distance apart again, and the upright
    // rotates between them -- so the joints stay exact and the camber gain the
    // geometry gives is what is drawn.
    const LB1 = solveHinge(M.lca, hp.LF, hp.LA, hp.LB, (p) => p[1] - hp.LB[1] - d[1], st, "thL");
    const kingpin = V.len(V.sub(hp.UB, hp.LB));
    const UB1 = solveHinge(M.uca, hp.UF, hp.UA, hp.UB, (p) => V.len(V.sub(p, LB1)) - kingpin, st, "thU");
    // Steering turns the upright about the vertical through the hub, as the
    // renderer turns the wheel; then the kingpin's own rotation.
    const Ry = steer ? axisAngle([0, 1, 0], steer) : IDENT3;
    const Rk = between(V.sub(hp.UB, hp.LB), V.sub(UB1, LB1));
    place(M.upright, mul3(Rk, Ry), hp.WC, V.add(LB1, apply3(Rk, V.sub(hp.WC, hp.LB))));
    const TU1 = pt(M.upright, hp.TU);
    // Tie rod: its inboard end is on the rack, which slides across the car
    // to keep the rod its own length (the front's steering; the rear's toe
    // link has a fixed chassis end and only swings).
    let TC1 = hp.TC;
    if (steer) {
      const v = V.sub(TU1, hp.TC), L = st.tieLen;
      const disc = L * L - v[0] * v[0] - v[1] * v[1];
      if (disc > 0) {
        const r = Math.sqrt(disc);
        const s1 = v[2] - r, s2 = v[2] + r;
        TC1 = V.add(hp.TC, [0, 0, Math.abs(s1) < Math.abs(s2) ? s1 : s2]);
      }
    }
    place(M.tie, between(V.sub(hp.TU, hp.TC), V.sub(TU1, TC1)), hp.TC, TC1);
    // Pushrod's outboard end rides its wishbone.
    const PP1 = pt(hp.pushOn === "uca" ? M.uca : M.lca, hp.PP);
    // Rocker: turn about its pivot until the pushrod is its own length again.
    // Newton on the angle, warm-started from last frame.
    const n = st.rockerAxis, r0 = V.sub(hp.ROD, hp.PIV);
    let phi = st.phi;
    const f = (a) => V.len(V.sub(V.add(hp.PIV, apply3(axisAngle(n, a), r0)), PP1)) - st.pushLen;
    for (let i = 0; i < 8; i++) {
      const y = f(phi);
      if (Math.abs(y) < 1e-7) break;
      const dy = (f(phi + 1e-4) - y) / 1e-4;
      if (!Number.isFinite(dy) || Math.abs(dy) < 1e-9) break;
      phi = Math.max(-0.8, Math.min(0.8, phi - y / dy));
    }
    if (Number.isFinite(phi)) st.phi = phi;
    st.rockerDeg = (st.phi * 180) / Math.PI;
    place(M.rocker, axisAngle(n, st.phi), hp.PIV, hp.PIV);
    const ROD1 = pt(M.rocker, hp.ROD), COI1 = pt(M.rocker, hp.COI);
    place(M.pushrod, between(V.sub(hp.ROD, hp.PP), V.sub(ROD1, PP1)), hp.PP, PP1);
    // Coilover: the body stays on its chassis eye and aims at the rocker; the
    // shaft stays on the rocker's eye and slides; the spring spans the two,
    // scaled along its own axis by how much the unit has shortened.
    const axis0 = V.sub(hp.COI, hp.ATT), axis1 = V.sub(COI1, hp.ATT);
    const R = between(axis0, axis1);
    place(M.damper_top, R, hp.ATT, hp.ATT);
    place(M.damper_rod, R, hp.COI, COI1);
    const k = V.len(axis1) / st.coilLen;
    st.coilRatio = k;
    const u = V.unit(axis0);
    // S = I + (k - 1) u u^T: stretch along the axis only, the coils keep
    // their diameter.
    const S = [
      1 + (k - 1) * u[0] * u[0], (k - 1) * u[0] * u[1], (k - 1) * u[0] * u[2],
      (k - 1) * u[1] * u[0], 1 + (k - 1) * u[1] * u[1], (k - 1) * u[1] * u[2],
      (k - 1) * u[2] * u[0], (k - 1) * u[2] * u[1], 1 + (k - 1) * u[2] * u[2],
    ];
    place(M.spring, mul3(R, S), hp.ATT, hp.ATT);
    // ARB drop link (front): from the blade to the rocker.
    if (hp.DLB && hp.DLT) {
      const top1 = pt(M.rocker, hp.DLT);
      place(M.droplink, between(V.sub(hp.DLT, hp.DLB), V.sub(top1, hp.DLB)), hp.DLB, hp.DLB);
    }
    return st;
  }

  matrix(name, role) {
    return this.state[name]?.mats[role] ?? null;
  }
}
