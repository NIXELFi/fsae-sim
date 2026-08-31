// First-person WebGL2 renderer.
//
// The world is a flat asphalt lot -- which is what FSAE Michigan actually is --
// so almost all of the sense of speed has to come from surface detail rather
// than scenery. Hence the procedurally textured ground with faded parking-stall
// lines: they are authentic to the venue AND they are the optical flow that
// tells you how fast you are going at 25 m/s two feet off the deck.
//
// Coordinate mapping: the vehicle model works in (x east, y north). GL is
// y-up, so world (x, y) maps to GL (x, height, -y) throughout.

import {
  mat4, perspective, lookAlong, multiply, normalize, identity,
  translation, rotX, rotY, rotZ, transformDir, transformPoint,
} from "./math.js";
import { buildCarMeshes, hubsFor, GEO, HUBS } from "./carmesh.js";
import { buildVenueMesh } from "./venuemesh.js";

const CONE_DRAW_RANGE = 140; // m

// ---------------------------------------------------------------- shaders ---

const SKY_VS = `#version 300 es
in vec2 aPos;
out vec2 vNdc;
void main() { vNdc = aPos; gl_Position = vec4(aPos, 0.999, 1.0); }`;

const SKY_FS = `#version 300 es
precision highp float;
in vec2 vNdc;
uniform float uHorizon;   // NDC y of the horizon
out vec4 frag;
void main() {
  float t = clamp((vNdc.y - uHorizon) / (1.0 - uHorizon + 1e-3), 0.0, 1.0);
  vec3 low  = vec3(0.72, 0.78, 0.85);
  vec3 high = vec3(0.28, 0.45, 0.72);
  vec3 c = mix(low, high, pow(t, 0.85));
  // Haze band right at the horizon so the ground plane does not end abruptly.
  c = mix(vec3(0.80, 0.83, 0.86), c, smoothstep(0.0, 0.16, t));
  frag = vec4(c, 1.0);
}`;

const GROUND_VS = `#version 300 es
in vec2 aPos;                 // unit quad, -1..1
uniform mat4 uViewProj;
uniform vec2 uCamXZ;
uniform float uExtent;
uniform float uDrop;
out vec2 vWorld;
void main() {
  vec2 p = uCamXZ + aPos * uExtent;
  vWorld = p;
  gl_Position = uViewProj * vec4(p.x, -uDrop, p.y, 1.0);
}`;

const GROUND_FS = `#version 300 es
precision highp float;
in vec2 vWorld;
uniform vec2 uCamXZ;
out vec4 frag;

float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float noise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x),
             mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}

void main() {
  float dist = length(vWorld - uCamXZ);

  // Aggregate-sized speckle, faded out with distance so it does not alias into
  // a shimmering mess at the horizon.
  float detail = clamp(1.0 - dist / 70.0, 0.0, 1.0);
  float n = noise(vWorld * 18.0) * 0.55 + noise(vWorld * 60.0) * 0.45;
  float coarse = noise(vWorld * 1.3);

  vec3 asphalt = vec3(0.30, 0.305, 0.315);
  asphalt *= 0.86 + 0.28 * coarse;
  asphalt += (n - 0.5) * 0.13 * detail;

  // Faded parking-stall lines: 2.75 m bays, 5.5 m deep. Real lot markings, and
  // the main optical-flow cue at speed.
  vec2 g = abs(fract(vWorld / vec2(2.75, 5.5)) - 0.5) * vec2(2.75, 5.5);
  float line = min(g.x, g.y);
  float paint = (1.0 - smoothstep(0.04, 0.09, line)) * 0.35 * clamp(1.0 - dist / 110.0, 0.0, 1.0);
  paint *= 0.4 + 0.6 * noise(vWorld * 3.0);   // worn and patchy
  asphalt = mix(asphalt, vec3(0.62, 0.62, 0.60), paint);

  // Distance haze into the sky colour.
  float haze = clamp(dist / 260.0, 0.0, 1.0);
  asphalt = mix(asphalt, vec3(0.80, 0.83, 0.86), pow(haze, 1.4));
  frag = vec4(asphalt, 1.0);
}`;

const RIBBON_VS = `#version 300 es
in vec3 aPos;
in float aS;
in float aSide;
uniform mat4 uViewProj;
out float vS;
out float vSide;
out vec3 vWorld;
void main() {
  vS = aS; vSide = aSide; vWorld = aPos;
  gl_Position = uViewProj * vec4(aPos, 1.0);
}`;

const RIBBON_FS = `#version 300 es
precision highp float;
in float vS;
in float vSide;
in vec3 vWorld;
uniform vec3 uCam;
uniform float uLength;
uniform float uClosed;
out vec4 frag;

float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float noise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x),
             mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}

void main() {
  float dist = length(vWorld - uCam);
  // Rubbered-in racing surface: darker than the surrounding lot, darkest in
  // the middle where the cars actually run.
  float mid = 1.0 - abs(vSide);
  vec3 c = vec3(0.245, 0.248, 0.256) - 0.045 * mid;
  c += (noise(vWorld.xz * 14.0) - 0.5) * 0.05 * clamp(1.0 - dist / 60.0, 0.0, 1.0);

  // Start/finish: a painted chequer band. Autocross also gets one at the end.
  float atStart = 1.0 - smoothstep(0.0, 1.2, vS);
  float atEnd = uClosed > 0.5 ? 0.0 : 1.0 - smoothstep(0.0, 1.2, uLength - vS);
  float band = max(atStart, atEnd);
  if (band > 0.0) {
    float sq = mod(floor(vSide * 6.0) + floor(vS / 0.6), 2.0);
    c = mix(c, mix(vec3(0.08), vec3(0.88), sq), band);
  }

  float haze = clamp(dist / 260.0, 0.0, 1.0);
  c = mix(c, vec3(0.80, 0.83, 0.86), pow(haze, 1.4));
  frag = vec4(c, 1.0);
}`;

const PROP_VS = `#version 300 es
in vec3 aPos;
in vec3 aNormal;
in vec3 aColor;
in vec3 iOffset;
in float iDown;
in float iTint;
uniform mat4 uViewProj;
out vec3 vColor;
out vec3 vNormal;
out vec3 vWorld;
void main() {
  vec3 p = aPos;
  vec3 n = aNormal;
  if (iDown > 0.5) {
    // Knocked over: rotate -90 deg about X so the axis lies horizontal, then
    // lift by the base radius so it rests on the deck instead of half-sunk.
    p = vec3(p.x, p.z, -p.y);
    n = vec3(n.x, n.z, -n.y);
    p.y += 0.15;
  }
  vec3 world = p + iOffset;
  vWorld = world;
  vNormal = n;
  vColor = aColor * iTint;
  gl_Position = uViewProj * vec4(world, 1.0);
}`;

const PROP_FS = `#version 300 es
precision highp float;
in vec3 vColor;
in vec3 vNormal;
in vec3 vWorld;
uniform vec3 uCam;
out vec4 frag;
void main() {
  vec3 n = normalize(vNormal);
  vec3 sun = normalize(vec3(0.45, 0.82, 0.35));
  float lambert = max(dot(n, sun), 0.0);
  vec3 c = vColor * (0.52 + 0.60 * lambert);
  float dist = length(vWorld - uCam);
  float haze = clamp(dist / 260.0, 0.0, 1.0);
  c = mix(c, vec3(0.80, 0.83, 0.86), pow(haze, 1.4));
  frag = vec4(c, 1.0);
}`;

const CAR_VS = `#version 300 es
in vec3 aPos;
in vec3 aNormal;
in vec3 aColor;
uniform mat4 uViewProj;
uniform mat4 uModel;
out vec3 vColor;
out vec3 vNormal;
out vec3 vWorld;
void main() {
  vec4 w = uModel * vec4(aPos, 1.0);
  vWorld = w.xyz;
  vNormal = mat3(uModel) * aNormal;
  vColor = aColor;
  gl_Position = uViewProj * w;
}`;

const CAR_FS = `#version 300 es
precision highp float;
in vec3 vColor;
in vec3 vNormal;
in vec3 vWorld;
uniform vec3 uCam;
uniform vec4 uOverride;   // rgb to blend toward, alpha = how much
out vec4 frag;
void main() {
  // Two-sided: inside a cockpit you are looking at the back of half the
  // bodywork, and an unlit black shell there ruins the whole effect.
  vec3 n = normalize(vNormal);
  if (!gl_FrontFacing) n = -n;

  vec3 sun = normalize(vec3(0.45, 0.82, 0.35));
  float lambert = max(dot(n, sun), 0.0);
  float sky = 0.5 + 0.5 * n.y;               // sky above, ground bounce below

  vec3 base = mix(vColor, uOverride.rgb, uOverride.a);
  vec3 c = base * (0.26 + 0.32 * sky + 0.60 * lambert);

  vec3 v = normalize(uCam - vWorld);
  vec3 h = normalize(v + sun);
  c += vec3(1.0) * pow(max(dot(n, h), 0.0), 46.0) * 0.20;   // clearcoat glint

  float haze = clamp(length(vWorld - uCam) / 260.0, 0.0, 1.0);
  c = mix(c, vec3(0.80, 0.83, 0.86), pow(haze, 1.4));
  frag = vec4(c, 1.0);
}`;

// ------------------------------------------------------------------ meshes ---

/** An FSAE course cone: 18 in tall, orange, with the white reflective band. */
function coneMesh() {
  const pos = [], nrm = [], col = [];
  const H = 0.46, R = 0.145, SEG = 12;
  const orange = [0.95, 0.32, 0.05];
  const white = [0.92, 0.92, 0.90];
  const baseCol = [0.16, 0.16, 0.17];

  // Body: stacked rings so the white band gets its own colour.
  const rings = 6;
  for (let r = 0; r < rings; r++) {
    const t0 = r / rings, t1 = (r + 1) / rings;
    const y0 = t0 * H, y1 = t1 * H;
    const r0 = R * (1 - t0 * 0.88), r1 = R * (1 - t1 * 0.88);
    const c0 = t0 > 0.52 && t0 < 0.78 ? white : orange;
    const c1 = t1 > 0.52 && t1 < 0.78 ? white : orange;
    for (let s = 0; s < SEG; s++) {
      const a0 = (s / SEG) * Math.PI * 2, a1 = ((s + 1) / SEG) * Math.PI * 2;
      const p = (rr, y, a) => [Math.cos(a) * rr, y, Math.sin(a) * rr];
      const n = (a) => [Math.cos(a) * 0.9, 0.34, Math.sin(a) * 0.9];
      const quad = [
        [p(r0, y0, a0), n(a0), c0], [p(r1, y1, a0), n(a0), c1], [p(r1, y1, a1), n(a1), c1],
        [p(r0, y0, a0), n(a0), c0], [p(r1, y1, a1), n(a1), c1], [p(r0, y0, a1), n(a1), c0],
      ];
      for (const [P, N, C] of quad) { pos.push(...P); nrm.push(...N); col.push(...C); }
    }
  }
  // Square base plate.
  const B = 0.155;
  const plate = [
    [-B, 0.012, -B], [B, 0.012, -B], [B, 0.012, B],
    [-B, 0.012, -B], [B, 0.012, B], [-B, 0.012, B],
  ];
  for (const P of plate) { pos.push(...P); nrm.push(0, 1, 0); col.push(...baseCol); }

  return {
    position: new Float32Array(pos),
    normal: new Float32Array(nrm),
    color: new Float32Array(col),
    count: pos.length / 3,
  };
}

/** A box, used for the start/finish gate posts and the lot's light poles. */
function boxMesh(w, h, d, color) {
  const x = w / 2, z = d / 2;
  const faces = [
    [[-x, 0, z], [x, 0, z], [x, h, z], [-x, h, z], [0, 0, 1]],
    [[x, 0, -z], [-x, 0, -z], [-x, h, -z], [x, h, -z], [0, 0, -1]],
    [[x, 0, z], [x, 0, -z], [x, h, -z], [x, h, z], [1, 0, 0]],
    [[-x, 0, -z], [-x, 0, z], [-x, h, z], [-x, h, -z], [-1, 0, 0]],
    [[-x, h, z], [x, h, z], [x, h, -z], [-x, h, -z], [0, 1, 0]],
  ];
  const pos = [], nrm = [], col = [];
  for (const [a, b, c, d2, n] of faces) {
    for (const P of [a, b, c, a, c, d2]) { pos.push(...P); nrm.push(...n); col.push(...color); }
  }
  return {
    position: new Float32Array(pos),
    normal: new Float32Array(nrm),
    color: new Float32Array(col),
    count: pos.length / 3,
  };
}

// ---------------------------------------------------------------- renderer ---

export class Renderer {
  constructor(canvas) {
    const gl = canvas.getContext("webgl2", {
      antialias: true, alpha: false, depth: true, powerPreference: "high-performance",
    });
    if (!gl) throw new Error("WebGL2 is required and is not available in this browser.");
    this.gl = gl;
    this.canvas = canvas;

    gl.enable(gl.DEPTH_TEST);
    gl.cullFace(gl.BACK);
    // Culling is switched on only around the solid props. The sky quad, the
    // ground quad and the course ribbon are all single-sided sheets whose
    // winding depends on which side you view them from, so culling them
    // globally makes the world disappear from half the approaches.

    this.progSky = program(gl, SKY_VS, SKY_FS);
    this.progGround = program(gl, GROUND_VS, GROUND_FS);
    this.progRibbon = program(gl, RIBBON_VS, RIBBON_FS);
    this.progProp = program(gl, PROP_VS, PROP_FS);
    this.progCar = program(gl, CAR_VS, CAR_FS);

    this.quad = quadVao(gl, this.progSky, "aPos");
    this.groundQuad = quadVao(gl, this.progGround, "aPos");

    this.cone = this.makeInstanced(coneMesh(), 4096);
    this.post = this.makeInstanced(boxMesh(0.12, 2.1, 0.12, [0.85, 0.85, 0.88]), 8);
    this.pole = this.makeInstanced(boxMesh(0.35, 9.0, 0.35, [0.32, 0.33, 0.36]), 64);

    /**
     * A CAD model, once one is loaded. Held separately from the procedural
     * meshes so that editing a vehicle parameter -- which rebuilds the
     * procedural car -- does not silently throw the imported model away.
     */
    this.carModel = null;

    const carMeshes = buildCarMeshes(null);
    this.car = {
      body: this.makeMesh(carMeshes.body),
      tire: this.makeMesh(carMeshes.tire),
      rim: this.makeMesh(carMeshes.rim),
      steeringWheel: this.makeMesh(carMeshes.steeringWheel),
    };

    this.viewProj = mat4();
    this.proj = mat4();
    this.view = mat4();
    this.chassis = mat4();
    this.camFrame = mat4();
    this.model = mat4();
    this._a = mat4(); this._b = mat4();
    this._t = [mat4(), mat4(), mat4(), mat4(), mat4()];
    this.fovDeg = 78;
  }

  /**
   * Re-stretch the body onto new geometry.
   *
   * The bodywork is authored once at SDM26's real stations and mapped onto the
   * live wheelbase and track, so changing either moves the car you see as well
   * as the car you drive. Only the body needs it -- the wheels are placed by
   * their own transforms, which already read the live hub positions.
   */
  /**
   * Draw a CAD-imported car instead of the procedural one.
   *
   * The hub positions come from the file rather than from the vehicle
   * parameters, so the wheels are drawn where the model puts them. That is the
   * right way round: if the two disagree, the fix is the model, and seeing it
   * in the wrong place is how you find out.
   *
   * Pass null to go back to the procedural body, which is worth being able to
   * do -- comparing a half-finished export against known-good geometry is
   * exactly what you want while getting an export right.
   */
  useCarModel(car) {
    const gl = this.gl;
    for (const mesh of Object.values(this.car)) {
      if (mesh?.vao) gl.deleteVertexArray(mesh.vao);
      for (const b of Object.values(mesh?.buffers ?? {})) gl.deleteBuffer(b);
    }
    if (car) {
      this.car = {
        body: this.makeMesh(car.body),
        tire: this.makeMesh(car.tire),
        rim: this.makeMesh(car.rim),
        steeringWheel: this.makeMesh(car.steeringWheel),
      };
      this.carModel = { hubs: car.hubs, steerCentre: car.steerCentre };
    } else {
      const meshes = buildCarMeshes(this.carParams ?? null);
      this.car = {
        body: this.makeMesh(meshes.body),
        tire: this.makeMesh(meshes.tire),
        rim: this.makeMesh(meshes.rim),
        steeringWheel: this.makeMesh(meshes.steeringWheel),
      };
      this.carModel = null;
    }
  }

  /**
   * Replace just the wheel, keeping whatever body is in use.
   *
   * Separate from `useCarModel` because a wheel is a far easier thing to
   * supply than a whole car -- one sub-assembly, identical on all four
   * corners, and four of the largest objects on screen. Nobody should have to
   * model a complete car to stop looking at a procedural tyre.
   */
  useWheelModel(wheel) {
    const gl = this.gl;
    for (const key of ["tire", "rim"]) {
      const mesh = this.car[key];
      if (mesh?.vao) gl.deleteVertexArray(mesh.vao);
      for (const b of Object.values(mesh?.buffers ?? {})) gl.deleteBuffer(b);
    }
    if (wheel) {
      this.car.tire = this.makeMesh(wheel.tire);
      this.car.rim = this.makeMesh(wheel.rim);
      this.wheelModel = true;
    } else {
      const meshes = buildCarMeshes(this.carParams ?? null);
      this.car.tire = this.makeMesh(meshes.tire);
      this.car.rim = this.makeMesh(meshes.rim);
      this.wheelModel = false;
    }
  }

  /**
   * Replace only the bodywork, keeping the wheels.
   *
   * The pairing a CFD assembly needs: aero surfaces from one file, wheels from
   * another. Neither file has to know about the other.
   */
  useBodyModel(body) {
    const gl = this.gl;
    const mesh = this.car.body;
    if (mesh?.vao) gl.deleteVertexArray(mesh.vao);
    for (const b of Object.values(mesh?.buffers ?? {})) gl.deleteBuffer(b);
    if (body) {
      this.car.body = this.makeMesh(body);
      this.bodyModel = true;
    } else {
      this.car.body = this.makeMesh(buildCarMeshes(this.carParams ?? null).body);
      this.bodyModel = false;
    }
  }

  rebuildCar(params) {
    this.carParams = params;
    // A CAD model is not built from the vehicle parameters, so stretching the
    // wheelbase must not quietly replace it with procedural geometry.
    if (this.carModel) return;
    if (this.bodyModel) {
      // An imported body is not built from the vehicle parameters either.
      const meshes = buildCarMeshes(params);
      const gl = this.gl;
      const sw = this.car.steeringWheel;
      if (sw?.vao) gl.deleteVertexArray(sw.vao);
      for (const b of Object.values(sw?.buffers ?? {})) gl.deleteBuffer(b);
      this.car.steeringWheel = this.makeMesh(meshes.steeringWheel);
      if (!this.wheelModel) {
        for (const key of ["tire", "rim"]) {
          const m = this.car[key];
          if (m?.vao) gl.deleteVertexArray(m.vao);
          for (const b of Object.values(m?.buffers ?? {})) gl.deleteBuffer(b);
          this.car[key] = this.makeMesh(meshes[key]);
        }
      }
      return;
    }
    if (this.wheelModel) {
      // Same for an imported wheel: rebuild the body, keep the wheel.
      const meshes = buildCarMeshes(params);
      const gl = this.gl;
      for (const key of ["body", "steeringWheel"]) {
        const mesh = this.car[key];
        if (mesh?.vao) gl.deleteVertexArray(mesh.vao);
        for (const b of Object.values(mesh?.buffers ?? {})) gl.deleteBuffer(b);
      }
      this.car.body = this.makeMesh(meshes.body);
      this.car.steeringWheel = this.makeMesh(meshes.steeringWheel);
      return;
    }
    const gl = this.gl;
    const body = buildCarMeshes(params).body;
    gl.bindBuffer(gl.ARRAY_BUFFER, this.car.body.buffers.position);
    gl.bufferData(gl.ARRAY_BUFFER, body.position, gl.STATIC_DRAW);
    this.car.body.count = body.count;
    this.carParams = params;
  }

  /** Static mesh drawn with its own model matrix (the car parts). */
  makeMesh(mesh) {
    const gl = this.gl;
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const attach = (data, name, size) => {
      const buf = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
      const loc = gl.getAttribLocation(this.progCar, name);
      if (loc >= 0) {
        gl.enableVertexAttribArray(loc);
        gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 0, 0);
      }
      return buf;
    };
    const buffers = {
      position: attach(mesh.position, "aPos", 3),
      normal: attach(mesh.normal, "aNormal", 3),
      color: attach(mesh.color, "aColor", 3),
    };
    gl.bindVertexArray(null);
    return { vao, count: mesh.count, buffers };
  }

  /** out = m0 * m1 * ... , without aliasing. */
  chain(out, mats) {
    let a = this._a, b = this._b;
    identity(a);
    for (const m of mats) {
      multiply(b, a, m);
      const t = a; a = b; b = t;
    }
    out.set(a);
    return out;
  }

  makeInstanced(mesh, maxInstances) {
    const gl = this.gl;
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);

    const attach = (data, name, size) => {
      const buf = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
      const loc = gl.getAttribLocation(this.progProp, name);
      if (loc >= 0) {
        gl.enableVertexAttribArray(loc);
        gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 0, 0);
      }
      return buf;
    };
    attach(mesh.position, "aPos", 3);
    attach(mesh.normal, "aNormal", 3);
    attach(mesh.color, "aColor", 3);

    // Interleaved per-instance data: offset(3), down(1), tint(1)
    const stride = 5 * 4;
    const instBuf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, instBuf);
    gl.bufferData(gl.ARRAY_BUFFER, maxInstances * stride, gl.DYNAMIC_DRAW);
    const bindInst = (name, size, offset) => {
      const loc = gl.getAttribLocation(this.progProp, name);
      if (loc < 0) return;
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, size, gl.FLOAT, false, stride, offset);
      gl.vertexAttribDivisor(loc, 1);
    };
    bindInst("iOffset", 3, 0);
    bindInst("iDown", 1, 12);
    bindInst("iTint", 1, 16);

    gl.bindVertexArray(null);
    return {
      vao, instBuf, count: mesh.count, maxInstances,
      data: new Float32Array(maxInstances * 5),
    };
  }

  /** Build the course ribbon and the static props for a track. */
  setTrack(track) {
    const gl = this.gl;
    this.track = track;
    this.venue = null;

    // A venue supplies its own surfaces -- banking, apron, infield, wall and
    // fence -- so there is no course ribbon and no gate posts to place.
    if (track.kind === "venue") {
      this.venue = this.makeMesh(buildVenueMesh(track));
      this.ribbon = null;
      this.gatePosts = [];
      this.poles = [];
      return;
    }

    const n = track.center.length;
    const pos = [], sArr = [], side = [];
    const half = track.width / 2;
    const push = (i) => {
      const [x, y] = track.center[i];
      const h = track.heading[i];
      const nx = -Math.sin(h), ny = Math.cos(h);
      for (const sgn of [1, -1]) {
        pos.push(x + nx * half * sgn, 0.012, -(y + ny * half * sgn));
        sArr.push(track.s[i]);
        side.push(sgn);
      }
    };
    for (let i = 0; i < n; i++) push(i);
    if (track.closed) push(0);

    this.ribbon = {
      vao: gl.createVertexArray(),
      count: pos.length / 3,
    };
    gl.bindVertexArray(this.ribbon.vao);
    const attach = (arr, name, size) => {
      const buf = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(arr), gl.STATIC_DRAW);
      const loc = gl.getAttribLocation(this.progRibbon, name);
      if (loc >= 0) {
        gl.enableVertexAttribArray(loc);
        gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 0, 0);
      }
    };
    attach(pos, "aPos", 3);
    attach(sArr, "aS", 1);
    attach(side, "aSide", 1);
    gl.bindVertexArray(null);

    // Start/finish gate posts, and light poles scattered around the lot for
    // depth cues (placed outside the course bounding box so they never block).
    const gate = [];
    const addGate = (i) => {
      const [x, y] = track.center[i];
      const h = track.heading[i];
      const nx = -Math.sin(h), ny = Math.cos(h);
      for (const sgn of [1, -1]) {
        gate.push({ x: x + nx * (half + 0.6) * sgn, y: y + ny * (half + 0.6) * sgn });
      }
    };
    addGate(0);
    if (!track.closed) addGate(n - 1);
    this.gatePosts = gate;

    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const [x, y] of track.center) {
      minX = Math.min(minX, x); maxX = Math.max(maxX, x);
      minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    }
    this.poles = [];
    for (let i = 0; i < 26; i++) {
      const a = (i / 26) * Math.PI * 2;
      const rx = (maxX - minX) / 2 + 45, ry = (maxY - minY) / 2 + 45;
      this.poles.push({
        x: (minX + maxX) / 2 + Math.cos(a) * rx,
        y: (minY + maxY) / 2 + Math.sin(a) * ry,
      });
    }
  }

  resize() {
    const dpr = Math.min(devicePixelRatio || 1, 2);
    // A hidden or not-yet-laid-out canvas reports zero client size. Sizing the
    // drawing buffer to zero makes the projection divide by an aspect of 0 and
    // the whole frame silently vanishes, so fall back to a sane default.
    const cw = this.canvas.clientWidth > 0 ? this.canvas.clientWidth : 1280;
    const ch = this.canvas.clientHeight > 0 ? this.canvas.clientHeight : 720;
    const w = Math.max(1, Math.floor(cw * dpr));
    const h = Math.max(1, Math.floor(ch * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w; this.canvas.height = h;
    }
    this.gl.viewport(0, 0, w, h);
    return w / h;
  }

  /**
   * Draw one frame.
   * @param {object} s
   *   car    {x, y, psi, rollRad, pitchRad}   chassis pose
   *   view   {ahead, height, pitchOffset, rigid}  eye point in the chassis frame
   *   wheels {steerRad, spinFront, spinRear, rimFade}
   *   heaveM, fovBoost
   */
  draw(s) {
    const gl = this.gl;
    const aspect = this.resize();
    const T = this._t;
    const cam = s.car;

    // ---- chassis frame: everything bolted to the car rides on this ----
    this.chain(this.chassis, [
      translation(T[0], cam.x, 0, -cam.y),
      rotY(T[1], cam.psi),
      rotZ(T[2], cam.pitchRad),
      rotX(T[3], cam.rollRad),
    ]);

    // The cockpit and nose cameras are RIGIDLY bolted to that frame. That is
    // the whole point: the dash and wheel must not move relative to the
    // driver's head, so the roll you see is the world rolling, not the car.
    // Chase damps roll and pitch, because a chase camera that rolls with the
    // car is unwatchable.
    if (s.view.rigid) {
      this.camFrame.set(this.chassis);
    } else {
      this.chain(this.camFrame, [
        translation(T[0], cam.x, 0, -cam.y),
        rotY(T[1], cam.psi),
        rotZ(T[2], cam.pitchRad * 0.30),
        rotX(T[3], cam.rollRad * 0.25),
      ]);
    }

    let eye;
    let forward;
    let up;
    if (s.view.orbit) {
      // Walkaround: circle the car at a fixed radius, looking at it.
      //
      // Every other camera sits on the car's centreline and looks along it,
      // which is right for driving and useless for judging the car -- you can
      // never see it from the side. This one is for looking at the model, and
      // it is the only view that can show whether an imported CAD body is the
      // right shape, the right way round, or sitting at the right height.
      const a = s.view.orbitAngle || 0;
      const r = s.view.radius ?? 3.4;
      // The car sits at (x, -y) in GL space; orbit in that plane.
      const cx = cam.x;
      const cz = -cam.y;
      const focusY = s.view.focusHeight ?? 0.45;
      eye = [cx + Math.cos(a) * r, s.view.height, cz + Math.sin(a) * r];
      const to = [cx - eye[0], focusY - eye[1], cz - eye[2]];
      forward = normalize(to);
      // World up, re-orthogonalised against the view direction.
      const dotUp = forward[1];
      up = normalize([-forward[0] * dotUp, 1 - dotUp * dotUp, -forward[2] * dotUp]);
    } else {
      eye = transformPoint(this.camFrame, [s.view.ahead, s.view.height + s.heaveM, 0]);
      const pitched = this.chain(T[4], [this.camFrame, rotZ(T[0], s.view.pitchOffset || 0)]);
      forward = normalize(transformDir(pitched, [1, 0, 0]));
      up = normalize(transformDir(pitched, [0, 1, 0]));
    }

    perspective(this.proj, ((this.fovDeg + (s.fovBoost || 0)) * Math.PI) / 180, aspect, 0.05, 700);
    lookAlong(this.view, eye, forward, up);
    multiply(this.viewProj, this.proj, this.view);

    gl.clearColor(0.8, 0.83, 0.86, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    // --- sky ---
    gl.depthMask(false);
    gl.useProgram(this.progSky);
    // Project the horizon into NDC so the gradient sits where the ground ends.
    const horizonNdc = -forward[1] * 1.6;
    gl.uniform1f(gl.getUniformLocation(this.progSky, "uHorizon"), Math.max(-0.9, Math.min(0.9, horizonNdc)));
    gl.bindVertexArray(this.quad);
    gl.drawArrays(gl.TRIANGLES, 0, 6);
    gl.depthMask(true);

    // --- ground ---
    gl.useProgram(this.progGround);
    uMat(gl, this.progGround, "uViewProj", this.viewProj);
    gl.uniform2f(gl.getUniformLocation(this.progGround, "uCamXZ"), eye[0], eye[2]);
    gl.uniform1f(gl.getUniformLocation(this.progGround, "uExtent"), 320);
    // The venue paves its own ground; sink the procedural lot below it.
    gl.uniform1f(gl.getUniformLocation(this.progGround, "uDrop"), this.venue ? 0.35 : 0.0);
    gl.bindVertexArray(this.groundQuad);
    gl.drawArrays(gl.TRIANGLES, 0, 6);

    // --- venue surfaces ---
    if (this.venue) {
      gl.useProgram(this.progCar);
      uMat(gl, this.progCar, "uViewProj", this.viewProj);
      gl.uniform3f(gl.getUniformLocation(this.progCar, "uCam"), eye[0], eye[1], eye[2]);
      gl.uniform4f(gl.getUniformLocation(this.progCar, "uOverride"), 0, 0, 0, 0);
      identity(this._a);
      uMat(gl, this.progCar, "uModel", this._a);
      gl.bindVertexArray(this.venue.vao);
      gl.drawArrays(gl.TRIANGLES, 0, this.venue.count);
    }

    // --- course ribbon ---
    if (this.ribbon) {
      gl.useProgram(this.progRibbon);
      uMat(gl, this.progRibbon, "uViewProj", this.viewProj);
      gl.uniform3f(gl.getUniformLocation(this.progRibbon, "uCam"), eye[0], eye[1], eye[2]);
      gl.uniform1f(gl.getUniformLocation(this.progRibbon, "uLength"), this.track.length);
      gl.uniform1f(gl.getUniformLocation(this.progRibbon, "uClosed"), this.track.closed ? 1 : 0);
      gl.bindVertexArray(this.ribbon.vao);
      gl.drawArrays(gl.TRIANGLE_STRIP, 0, this.ribbon.count);
    }

    // --- instanced props ---
    gl.enable(gl.CULL_FACE);
    gl.useProgram(this.progProp);
    uMat(gl, this.progProp, "uViewProj", this.viewProj);
    gl.uniform3f(gl.getUniformLocation(this.progProp, "uCam"), eye[0], eye[1], eye[2]);

    const cones = this.track.conesNear(s.car.x, s.car.y, CONE_DRAW_RANGE);
    this.drawInstances(this.cone, cones.map((c) => ({
      x: c.x, y: c.y, h: 0, down: c.down ? 1 : 0, tint: c.down ? 0.72 : 1,
    })));

    this.drawInstances(this.post, this.gatePosts.map((p) => ({ x: p.x, y: p.y, h: 0, down: 0, tint: 1 })));
    this.drawInstances(this.pole, this.poles.map((p) => ({ x: p.x, y: p.y, h: 0, down: 0, tint: 1 })));
    gl.disable(gl.CULL_FACE);

    this.drawCar(s, eye);

    gl.bindVertexArray(null);
  }

  /** The car itself: body, four wheels that steer and spin, steering wheel. */
  drawCar(s, eye) {
    const gl = this.gl;
    const T = this._t;
    const prog = this.progCar;

    gl.useProgram(prog);
    uMat(gl, prog, "uViewProj", this.viewProj);
    gl.uniform3f(gl.getUniformLocation(prog, "uCam"), eye[0], eye[1], eye[2]);
    const ovLoc = gl.getUniformLocation(prog, "uOverride");
    const modelLoc = gl.getUniformLocation(prog, "uModel");

    const part = (mesh, model, ov) => {
      gl.uniformMatrix4fv(modelLoc, false, model);
      if (ov) gl.uniform4f(ovLoc, ov[0], ov[1], ov[2], ov[3]);
      else gl.uniform4f(ovLoc, 0, 0, 0, 0);
      gl.bindVertexArray(mesh.vao);
      gl.drawArrays(gl.TRIANGLES, 0, mesh.count);
    };

    part(this.car.body, this.chassis, null);

    // Wheels. Steer rotates about the kingpin (local Y); spin is about the hub
    // axis (local Z) AFTER the steer, so a steered wheel rolls about its own
    // steered axis rather than the car's.
    const w = s.wheels;
    for (const hub of (this.carModel?.hubs ?? s.hubs ?? HUBS)) {
      this.chain(this.model, [
        this.chassis,
        translation(T[0], hub.x, hub.y, hub.z),
        rotY(T[1], hub.front ? w.steerRad : 0),
        rotZ(T[2], -(hub.front ? w.spinFront : w.spinRear)),
      ]);
      part(this.car.tire, this.model, null);
      // Fade the gold spokes toward the tyre as the wheel speeds up. Five
      // spokes at 20 rev/s would otherwise strobe into a stationary-looking
      // mess at 60 Hz; this reads as motion blur instead.
      part(this.car.rim, this.model, [0.15, 0.16, 0.18, w.rimFade]);
    }

    // Steering wheel: its own frame has the rotation axis on +Z, so tilt that
    // frame onto the column before spinning it.
    const tilt = GEO.steerTiltRad;
    const basis = T[3];
    basis.set([
      0, 0, -1, 0,
      Math.sin(tilt), Math.cos(tilt), 0, 0,
      Math.cos(tilt), -Math.sin(tilt), 0, 0,
      0, 0, 0, 1,
    ]);
    this.chain(this.model, [
      this.chassis,
      translation(T[0], ...(this.carModel?.steerCentre ?? GEO.steerCentre)),
      basis,
      rotZ(T[1], -w.steerRad * (w.steerRatio ?? GEO.steeringRatio)),
    ]);
    part(this.car.steeringWheel, this.model, null);
  }

  drawInstances(mesh, list) {
    const gl = this.gl;
    const n = Math.min(list.length, mesh.maxInstances);
    if (n === 0) return;
    const d = mesh.data;
    for (let i = 0; i < n; i++) {
      const it = list[i];
      d[i * 5 + 0] = it.x;
      d[i * 5 + 1] = it.h;
      d[i * 5 + 2] = -it.y;
      d[i * 5 + 3] = it.down;
      d[i * 5 + 4] = it.tint;
    }
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.instBuf);
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, d, 0, n * 5);
    gl.bindVertexArray(mesh.vao);
    gl.drawArraysInstanced(gl.TRIANGLES, 0, mesh.count, n);
  }
}

// ------------------------------------------------------------------ helpers ---

function compile(gl, type, src) {
  const sh = gl.createShader(type);
  gl.shaderSource(sh, src);
  gl.compileShader(sh);
  if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
    throw new Error(`shader: ${gl.getShaderInfoLog(sh)}\n${src}`);
  }
  return sh;
}

function program(gl, vs, fs) {
  const p = gl.createProgram();
  gl.attachShader(p, compile(gl, gl.VERTEX_SHADER, vs));
  gl.attachShader(p, compile(gl, gl.FRAGMENT_SHADER, fs));
  gl.linkProgram(p);
  if (!gl.getProgramParameter(p, gl.LINK_STATUS)) {
    throw new Error(`link: ${gl.getProgramInfoLog(p)}`);
  }
  return p;
}

function quadVao(gl, prog, attr) {
  const vao = gl.createVertexArray();
  gl.bindVertexArray(vao);
  const buf = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, buf);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
    -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1,
  ]), gl.STATIC_DRAW);
  const loc = gl.getAttribLocation(prog, attr);
  gl.enableVertexAttribArray(loc);
  gl.vertexAttribPointer(loc, 2, gl.FLOAT, false, 0, 0);
  gl.bindVertexArray(null);
  return vao;
}

function uMat(gl, prog, name, m) {
  gl.uniformMatrix4fv(gl.getUniformLocation(prog, name), false, m);
}
