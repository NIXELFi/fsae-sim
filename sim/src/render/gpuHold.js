// GPU clock hold.
//
// The renderer draws a 2560x1080 frame in about 1.5 ms of GPU time, which is
// the problem: a laptop GeForce sitting at 15-20 % busy drops to its lowest
// power state (P4-P8, 800-1000 MHz on the RTX 4070 that this was measured on)
// and has to ramp a clock every time a frame arrives. Roughly one frame in
// forty then misses its vsync and lands 7-25 ms late -- a visible stutter at
// 144 Hz that no amount of tidying in the renderer can touch, because a page
// that only clears its canvas hitches at exactly the same rate inside the
// same window (measured 2026-09-20: 63 late frames in 2828 for a bare
// gl.clear, 67 for the whole simulator).
//
// What removes them is keeping the GPU busy. With a fragment-heavy pass
// bringing the frame to ~3 ms of GPU time the late frames fell from 63-69 to
// 20 per 2800, three runs out of three, and the p99 frame went from 14.5 ms to
// 8.9. That is the same thing the driver's "prefer maximum performance"
// setting does, done from inside the page where the user does not have to
// find the NVIDIA control panel.
//
// This class draws that pass: a full-screen triangle into a small offscreen
// target, running a transcendental loop whose length is adjusted every few
// frames so the WHOLE frame (the renderer's work plus this) lands on a target
// GPU time. As the real rendering gets heavier the filler shrinks to nothing
// on its own. It measures with EXT_disjoint_timer_query_webgl2 and does not
// run at all without it, since without a measurement it could only guess.

/** GPU milliseconds per frame the frame is padded out to. About 40 % of a
 *  144 Hz frame: enough to hold the clocks, well short of a real load. */
const TARGET_MS = 3.0;
/** Offscreen target size. Small on purpose: the cost should be ALU, not
 *  bandwidth, so the pass leaves memory alone for the real frame. */
const SIZE = 512;
const ITER_MIN = 0, ITER_MAX = 40000;
/** How many frames a measurement is averaged over before the loop length
 *  moves. Short enough to follow a camera change, long enough not to hunt. */
const ADJUST_EVERY = 12;

const VS = `#version 300 es
void main() {
  vec2 p = vec2(gl_VertexID == 1 ? 3.0 : -1.0, gl_VertexID == 2 ? 3.0 : -1.0);
  gl_Position = vec4(p, 0.0, 1.0);
}`;

// The loop bound is a uniform: GLSL ES 3.00 allows a dynamic condition, and it
// is what lets the pass be resized without relinking. The seed keeps the
// compiler from hoisting anything out of the loop.
const FS = `#version 300 es
precision highp float;
uniform int uIter;
uniform float uSeed;
out vec4 o;
void main() {
  float a = uSeed + gl_FragCoord.x * 0.001 + gl_FragCoord.y * 0.0007;
  for (int i = 0; i < uIter; i++) a = sin(a * 1.7 + float(i) * 0.01) + cos(a);
  o = vec4(a, 0.0, 0.0, 1.0);
}`;

export class GpuHold {
  /**
   * @param {WebGL2RenderingContext} gl the renderer's own context, so the
   *   pass is queued behind the frame it pads.
   */
  constructor(gl) {
    this.gl = gl;
    this.enabled = false;
    this.ext = gl.getExtension("EXT_disjoint_timer_query_webgl2");
    /** Whether the machine can run this at all. */
    this.supported = !!this.ext;
    this.iter = 400;
    /** Last measured whole-frame GPU time, ms; for the panel. */
    this.frameMs = 0;
    this._pending = [];
    this._acc = 0;
    this._n = 0;
    this._open = false;
    this._seed = 0;
    if (!this.supported) return;

    const sh = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
        throw new Error("gpuHold shader: " + gl.getShaderInfoLog(s));
      }
      return s;
    };
    const prog = gl.createProgram();
    gl.attachShader(prog, sh(gl.VERTEX_SHADER, VS));
    gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FS));
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) {
      throw new Error("gpuHold link: " + gl.getProgramInfoLog(prog));
    }
    this.prog = prog;
    this.uIter = gl.getUniformLocation(prog, "uIter");
    this.uSeed = gl.getUniformLocation(prog, "uSeed");
    this.vao = gl.createVertexArray();
    this.tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, this.tex);
    gl.texStorage2D(gl.TEXTURE_2D, 1, gl.RGBA8, SIZE, SIZE);
    gl.bindTexture(gl.TEXTURE_2D, null);
    this.fbo = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.tex, 0);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  }

  /** Call before the frame's first draw. Opens the timer over the frame. */
  begin() {
    if (!this.enabled || !this.supported || this._open) return;
    const gl = this.gl, ext = this.ext;
    this._drain();
    const q = gl.createQuery();
    gl.beginQuery(ext.TIME_ELAPSED_EXT, q);
    this._pending.push(q);
    this._open = true;
  }

  /**
   * Call after the frame's last draw. Draws the filler and closes the timer.
   * Leaves the default framebuffer bound with the full-canvas viewport, and
   * depth test / blending as the renderer's next frame expects to find them
   * (it sets its own state on entry anyway).
   */
  end() {
    if (!this._open) return;
    const gl = this.gl;
    if (this.iter > 0) {
      gl.bindFramebuffer(gl.FRAMEBUFFER, this.fbo);
      gl.viewport(0, 0, SIZE, SIZE);
      gl.disable(gl.DEPTH_TEST);
      gl.disable(gl.BLEND);
      gl.useProgram(this.prog);
      gl.bindVertexArray(this.vao);
      gl.uniform1i(this.uIter, this.iter);
      gl.uniform1f(this.uSeed, (this._seed = (this._seed + 0.37) % 100));
      gl.drawArrays(gl.TRIANGLES, 0, 3);
      gl.bindVertexArray(null);
      gl.bindFramebuffer(gl.FRAMEBUFFER, null);
      gl.viewport(0, 0, gl.canvas.width, gl.canvas.height);
      gl.enable(gl.DEPTH_TEST);
    }
    gl.endQuery(this.ext.TIME_ELAPSED_EXT);
    this._open = false;
  }

  /** Read back finished timers and steer the loop length toward the target. */
  _drain() {
    const gl = this.gl, ext = this.ext;
    while (this._pending.length && gl.getQueryParameter(this._pending[0], gl.QUERY_RESULT_AVAILABLE)) {
      const q = this._pending.shift();
      if (!gl.getParameter(ext.GPU_DISJOINT_EXT)) {
        const ms = gl.getQueryParameter(q, gl.QUERY_RESULT) / 1e6;
        this.frameMs = ms;
        this._acc += ms;
        this._n++;
      }
      gl.deleteQuery(q);
    }
    if (this._n >= ADJUST_EVERY) {
      const mean = this._acc / this._n;
      this._acc = 0; this._n = 0;
      // Proportional step on the loop length: the pass costs ~linearly in
      // iterations, so scale by the shortfall, damped so it settles rather
      // than rings when the real frame cost jumps (camera change, menu).
      const err = TARGET_MS - mean;
      const step = Math.round(this.iter * (err / TARGET_MS) * 0.5) + (err > 0 ? 20 : -20);
      this.iter = Math.max(ITER_MIN, Math.min(ITER_MAX, this.iter + step));
    }
  }
}
