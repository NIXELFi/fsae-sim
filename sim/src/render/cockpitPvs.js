// EXPERIMENT (exp/hd-cockpit): which of the car's triangles can the driver see?
//
// A potentially-visible set, baked once on the GPU: every triangle of the
// body, the suspension/pedal parts and the four tyres is drawn with its own
// ID into an offscreen target, from a box of eye positions covering head
// motion and the whole eye-height slider, through a frustum wider than any
// screen's. Whatever ID lands in a pixel is visible. The cockpit view then
// draws only those triangles (plus a one-ring margin), and skips moving parts
// that never showed at all. Shadows and outside cameras still draw everything.

const VS = `#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in uint aId;
uniform mat4 uMVP;
flat out uint vId;
void main() { vId = aId; gl_Position = uMVP * vec4(aPos, 1.0); }`;
const FS = `#version 300 es
precision highp float;
flat in uint vId;
out vec4 o;
void main() {
  o = vec4(float(vId & 255u), float((vId >> 8) & 255u), float((vId >> 16) & 255u), 255.0) / 255.0;
}`;

function compile(gl, type, src) {
  const s = gl.createShader(type);
  gl.shaderSource(s, src);
  gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
  return s;
}

function mul(a, b) {
  const o = new Float32Array(16);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    let s = 0;
    for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
    o[c * 4 + r] = s;
  }
  return o;
}
function persp(fovY, aspect, near, far) {
  const f = 1 / Math.tan(fovY / 2), o = new Float32Array(16);
  o[0] = f / aspect; o[5] = f; o[10] = (far + near) / (near - far); o[11] = -1; o[14] = (2 * far * near) / (near - far);
  return o;
}
const trans = (x, y, z) => { const o = new Float32Array(16); o[0] = o[5] = o[10] = o[15] = 1; o[12] = x; o[13] = y; o[14] = z; return o; };

/**
 * @param gl         the renderer's WebGL2 context
 * @param items      [{ mesh: {position, index?, count}, model: mat4 (chassis-local) }]
 * @param viewLocal  the cockpit view matrix times the chassis matrix (chassis-local -> eye)
 * @param eyeOffsets chassis-local eye displacements to sample, [[dx, dy, dz], ...]
 * @returns per item, a Uint8Array of per-triangle visibility (1 = seen)
 */
export function bakeCockpitPvs(gl, items, viewLocal, eyeOffsets, opts = {}) {
  const W = opts.width ?? 2560, H = opts.height ?? 1440;
  const fovY = ((opts.fovYDeg ?? 78) * Math.PI) / 180;
  const t0 = performance.now();
  const prog = gl.createProgram();
  gl.attachShader(prog, compile(gl, gl.VERTEX_SHADER, VS));
  gl.attachShader(prog, compile(gl, gl.FRAGMENT_SHADER, FS));
  gl.linkProgram(prog);
  const uMVP = gl.getUniformLocation(prog, "uMVP");

  // De-indexed geometry with a flat per-triangle ID (IDs start at 1; 0 = sky).
  let nextId = 1;
  const gpu = items.map((it) => {
    const P = it.mesh.position, I = it.mesh.index;
    const tris = I ? I.length / 3 : P.length / 9;
    const pos = new Float32Array(tris * 9), ids = new Uint32Array(tris * 3);
    for (let t = 0; t < tris; t++) {
      for (let k = 0; k < 3; k++) {
        const v = I ? I[t * 3 + k] : t * 3 + k;
        pos[t * 9 + k * 3] = P[v * 3]; pos[t * 9 + k * 3 + 1] = P[v * 3 + 1]; pos[t * 9 + k * 3 + 2] = P[v * 3 + 2];
        ids[t * 3 + k] = nextId + t;
      }
    }
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const pb = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, pb);
    gl.bufferData(gl.ARRAY_BUFFER, pos, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    const ib = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, ib);
    gl.bufferData(gl.ARRAY_BUFFER, ids, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribIPointer(1, 1, gl.UNSIGNED_INT, 0, 0);
    gl.bindVertexArray(null);
    const g = { vao, pb, ib, first: nextId, tris, model: it.model };
    nextId += tris;
    return g;
  });
  if (nextId >= 1 << 24) throw new Error("too many triangles for a 24-bit ID");
  const seen = new Uint8Array(nextId);

  const fbo = gl.createFramebuffer();
  const col = gl.createRenderbuffer(), dep = gl.createRenderbuffer();
  gl.bindRenderbuffer(gl.RENDERBUFFER, col);
  gl.renderbufferStorage(gl.RENDERBUFFER, gl.RGBA8, W, H);
  gl.bindRenderbuffer(gl.RENDERBUFFER, dep);
  gl.renderbufferStorage(gl.RENDERBUFFER, gl.DEPTH_COMPONENT24, W, H);
  gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
  gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.RENDERBUFFER, col);
  gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.RENDERBUFFER, dep);
  gl.viewport(0, 0, W, H);
  gl.useProgram(prog);
  gl.enable(gl.DEPTH_TEST);
  gl.disable(gl.CULL_FACE);
  gl.disable(gl.BLEND);
  gl.colorMask(true, true, true, true);
  gl.depthMask(true);
  const proj = persp(fovY, W / H, 0.12, 50);
  const px = new Uint8Array(W * H * 4);
  for (const [dx, dy, dz] of eyeOffsets) {
    const vp = mul(proj, mul(viewLocal, trans(-dx, -dy, -dz)));
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    for (const g of gpu) {
      gl.uniformMatrix4fv(uMVP, false, mul(vp, g.model));
      gl.bindVertexArray(g.vao);
      gl.drawArrays(gl.TRIANGLES, 0, g.tris * 3);
    }
    gl.readPixels(0, 0, W, H, gl.RGBA, gl.UNSIGNED_BYTE, px);
    for (let i = 0; i < px.length; i += 4) {
      const id = px[i] | (px[i + 1] << 8) | (px[i + 2] << 16);
      if (id) seen[id] = 1;
    }
  }
  gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  gl.bindVertexArray(null);
  gl.deleteFramebuffer(fbo); gl.deleteRenderbuffer(col); gl.deleteRenderbuffer(dep);
  for (const g of gpu) { gl.deleteVertexArray(g.vao); gl.deleteBuffer(g.pb); gl.deleteBuffer(g.ib); }
  gl.deleteProgram(prog);
  gl.clearColor(0, 0, 0, 1);

  const out = gpu.map((g) => seen.subarray(g.first, g.first + g.tris).slice());
  return { vis: out, ms: Math.round(performance.now() - t0), views: eyeOffsets.length };
}

/** Visible triangles plus every triangle sharing a vertex with one (a margin
 *  against sub-pixel slivers the bake stepped over), as a new index array. */
export function visibleIndex(mesh, vis, ring = 1) {
  const I = mesh.index, nv = mesh.position.length / 3;
  let keep = vis;
  for (let r = 0; r < ring; r++) {
    const mark = new Uint8Array(nv);
    for (let t = 0; t < keep.length; t++) if (keep[t]) { mark[I[t * 3]] = 1; mark[I[t * 3 + 1]] = 1; mark[I[t * 3 + 2]] = 1; }
    const next = new Uint8Array(keep.length);
    for (let t = 0; t < keep.length; t++) next[t] = keep[t] || mark[I[t * 3]] || mark[I[t * 3 + 1]] || mark[I[t * 3 + 2]] ? 1 : 0;
    keep = next;
  }
  let n = 0;
  for (let t = 0; t < keep.length; t++) n += keep[t];
  const out = new Uint32Array(n * 3);
  let o = 0;
  for (let t = 0; t < keep.length; t++) if (keep[t]) { out[o++] = I[t * 3]; out[o++] = I[t * 3 + 1]; out[o++] = I[t * 3 + 2]; }
  return out;
}
