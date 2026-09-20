// Minimal column-major 4x4 matrix helpers -- only what the renderer uses.
// Column-major so the arrays go straight to uniformMatrix4fv without transpose.

export function mat4() {
  return new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
}

export function perspective(out, fovYRad, aspect, near, far) {
  const f = 1 / Math.tan(fovYRad / 2);
  const nf = 1 / (near - far);
  out[0] = f / aspect; out[1] = 0; out[2] = 0; out[3] = 0;
  out[4] = 0; out[5] = f; out[6] = 0; out[7] = 0;
  out[8] = 0; out[9] = 0; out[10] = (far + near) * nf; out[11] = -1;
  out[12] = 0; out[13] = 0; out[14] = 2 * far * near * nf; out[15] = 0;
  return out;
}

/** View matrix from an eye point and an orthonormal-ish (forward, up) pair. */
export function lookAlong(out, eye, forward, up) {
  const f = normalize([...forward]);
  let s = cross(f, up);
  s = normalize(s);
  const u = cross(s, f);

  out[0] = s[0]; out[1] = u[0]; out[2] = -f[0]; out[3] = 0;
  out[4] = s[1]; out[5] = u[1]; out[6] = -f[1]; out[7] = 0;
  out[8] = s[2]; out[9] = u[2]; out[10] = -f[2]; out[11] = 0;
  out[12] = -dot(s, eye); out[13] = -dot(u, eye); out[14] = dot(f, eye); out[15] = 1;
  return out;
}

export function identity(out) {
  out.set([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
  return out;
}

export function translation(out, x, y, z) {
  identity(out);
  out[12] = x; out[13] = y; out[14] = z;
  return out;
}

export function scale(out, x, y, z) {
  identity(out);
  out[0] = x;
  out[5] = y;
  out[10] = z;
  return out;
}

export function rotX(out, a) {
  const c = Math.cos(a), s = Math.sin(a);
  out.set([1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1]);
  return out;
}

export function rotY(out, a) {
  const c = Math.cos(a), s = Math.sin(a);
  out.set([c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, 0, 0, 0, 1]);
  return out;
}

export function rotZ(out, a) {
  const c = Math.cos(a), s = Math.sin(a);
  out.set([c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
  return out;
}

/** Transform a direction (w = 0) by a matrix. */
export function transformDir(m, v) {
  return [
    m[0] * v[0] + m[4] * v[1] + m[8] * v[2],
    m[1] * v[0] + m[5] * v[1] + m[9] * v[2],
    m[2] * v[0] + m[6] * v[1] + m[10] * v[2],
  ];
}

/** Transform a point (w = 1) by a matrix. */
export function transformPoint(m, v) {
  return [
    m[0] * v[0] + m[4] * v[1] + m[8] * v[2] + m[12],
    m[1] * v[0] + m[5] * v[1] + m[9] * v[2] + m[13],
    m[2] * v[0] + m[6] * v[1] + m[10] * v[2] + m[14],
  ];
}

export function multiply(out, a, b) {
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      out[c * 4 + r] =
        a[r] * b[c * 4] +
        a[4 + r] * b[c * 4 + 1] +
        a[8 + r] * b[c * 4 + 2] +
        a[12 + r] * b[c * 4 + 3];
    }
  }
  return out;
}

export function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

export function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

export function normalize(v) {
  const l = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / l, v[1] / l, v[2] / l];
}

/** Orthographic projection, column-major, GL clip conventions. */
export function ortho(out, l, r, b, t, n, f) {
  out.fill(0);
  out[0] = 2 / (r - l);
  out[5] = 2 / (t - b);
  out[10] = -2 / (f - n);
  out[12] = -(r + l) / (r - l);
  out[13] = -(t + b) / (t - b);
  out[14] = -(f + n) / (f - n);
  out[15] = 1;
  return out;
}

/** Inverse of a rigid transform (rotation + translation), column-major. */
export function invertRigid(out, m) {
  const r0 = m[0], r1 = m[1], r2 = m[2];
  const r4 = m[4], r5 = m[5], r6 = m[6];
  const r8 = m[8], r9 = m[9], r10 = m[10];
  const tx = m[12], ty = m[13], tz = m[14];
  out[0] = r0; out[1] = r4; out[2] = r8; out[3] = 0;
  out[4] = r1; out[5] = r5; out[6] = r9; out[7] = 0;
  out[8] = r2; out[9] = r6; out[10] = r10; out[11] = 0;
  out[12] = -(r0 * tx + r1 * ty + r2 * tz);
  out[13] = -(r4 * tx + r5 * ty + r6 * tz);
  out[14] = -(r8 * tx + r9 * ty + r10 * tz);
  out[15] = 1;
  return out;
}

/**
 * A rigid frame from three axis vectors and an origin, column-major: the
 * matrix that carries local +X, +Y, +Z onto `x`, `y`, `z` and the local
 * origin onto `origin`. The axes are written as given -- orthonormalise
 * them first if the frame has to be rigid. In place, no allocation.
 */
export function basisFromAxes(out, x, y, z, origin) {
  out[0] = x[0]; out[1] = x[1]; out[2] = x[2]; out[3] = 0;
  out[4] = y[0]; out[5] = y[1]; out[6] = y[2]; out[7] = 0;
  out[8] = z[0]; out[9] = z[1]; out[10] = z[2]; out[11] = 0;
  out[12] = origin[0]; out[13] = origin[1]; out[14] = origin[2]; out[15] = 1;
  return out;
}
