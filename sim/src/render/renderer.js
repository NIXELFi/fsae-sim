// First-person WebGL2 renderer.
//
// The world is a flat asphalt lot -- which is what FSAE Michigan actually is --
// so almost all of the sense of speed has to come from surface detail rather
// than scenery. Hence the procedurally textured ground with faded parking-stall
// lines: they are authentic to the venue AND they are the optical flow that
// tells you how fast you are going at 25 m/s two feet off the deck.
//
// Lighting is one model shared by every surface: a sun direction, a
// sky/ground hemisphere for ambient, Blinn specular, an orthographic shadow map
// that follows the car, and a sun-tinted distance fog whose colour is exactly
// the sky's horizon colour so the ground dissolves into the sky rather than
// stopping at an edge. Colours are in display space throughout (the asphalt
// albedo is 0.30, not 0.03), which every mesh builder relies on.
//
// Coordinate mapping: the vehicle model works in (x east, y north). GL is
// y-up, so world (x, y) maps to GL (x, height, -y) throughout.

import {
  mat4, perspective, lookAlong, multiply, normalize, identity, ortho,
  translation, rotX, rotY, rotZ, scale, transformDir, transformPoint,
} from "./math.js";
import { buildCarMeshes, GEO, HUBS } from "./carmesh.js";
import { buildVenueMesh } from "./venuemesh.js";
import { buildEnvironmentMesh } from "./envmesh.js";

const CONE_DRAW_RANGE = 140; // m
const SHADOW_SIZE = 2048;     // texels
const SHADOW_HALF = 16;       // m, half-extent of the shadow box around the car

// Attribute locations are fixed so one VAO can be drawn by the lit program
// and by the depth-only program alike.
const A_POS = 0, A_NORMAL = 1, A_COLOR = 2, I_OFFSET = 3, I_DOWN = 4, I_TINT = 5;

// Sun: mid-afternoon, from the south-west, high enough that a 15 m grandstand
// throws a shadow without the cones throwing 3 m ones.
const SUN = normalize([0.42, 0.74, 0.52]);
const HORIZON = [0.74, 0.81, 0.89];   // haze colour; also the fog colour
const ZENITH = [0.22, 0.44, 0.78];

// ---------------------------------------------------------------- shaders ---

/** Noise, lighting, shadow and fog shared by every fragment shader. */
const COMMON_FS = `
uniform vec3 uSun;
uniform vec3 uCam;
uniform vec3 uHorizon;
uniform vec3 uZenith;
uniform highp sampler2DShadow uShadow;
uniform mat4 uShadowMat;

float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float noise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x),
             mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}
float fbm(vec2 p) {
  return noise(p) * 0.5 + noise(p * 2.03) * 0.25 + noise(p * 4.11) * 0.125 + noise(p * 8.3) * 0.0625;
}

// 1 = lit, 0 = in shadow. Fades out at the edge of the shadow box so the
// boundary never shows as a line on the ground.
float shadowAt(vec3 world, vec3 n) {
  vec4 sp = uShadowMat * vec4(world + n * 0.02, 1.0);
  vec3 p = sp.xyz / sp.w;
  vec2 e = min(p.xy, 1.0 - p.xy);
  float inside = smoothstep(0.0, 0.06, min(e.x, e.y));
  if (inside <= 0.0 || p.z > 1.0) return 1.0;
  // Hardware compare with linear filtering: a 2x2 PCF tap for free. Four of
  // them, offset half a texel, soften the edge further.
  float t = 0.5 / ${SHADOW_SIZE}.0;
  float s = texture(uShadow, vec3(p.xy + vec2(-t, -t), p.z))
          + texture(uShadow, vec3(p.xy + vec2( t, -t), p.z))
          + texture(uShadow, vec3(p.xy + vec2(-t,  t), p.z))
          + texture(uShadow, vec3(p.xy + vec2( t,  t), p.z));
  return mix(1.0, s * 0.25, inside);
}

// Ambient from a sky/ground hemisphere plus direct sun.
vec3 lighting(vec3 albedo, vec3 n, vec3 world, float shadow, float specStrength, float gloss) {
  float up = n.y * 0.5 + 0.5;
  vec3 ambient = mix(vec3(0.24, 0.23, 0.21), vec3(0.50, 0.56, 0.66), up);
  float lambert = max(dot(n, uSun), 0.0);
  vec3 sunCol = vec3(1.0, 0.96, 0.88);
  vec3 c = albedo * (ambient + sunCol * 0.78 * lambert * shadow);
  vec3 v = normalize(uCam - world);
  vec3 h = normalize(v + uSun);
  float spec = pow(max(dot(n, h), 0.0), gloss) * specStrength * shadow;
  return c + sunCol * spec;
}

// Fog toward the horizon colour, warmed when looking into the sun.
vec3 applyFog(vec3 c, vec3 world) {
  vec3 d = world - uCam;
  float dist = length(d);
  float f = 1.0 - exp(-pow(dist / 520.0, 1.6));
  float sunAmt = pow(max(dot(d / dist, uSun), 0.0), 6.0);
  vec3 fc = mix(uHorizon, vec3(0.93, 0.86, 0.72), sunAmt * 0.45);
  return mix(c, fc, f);
}
`;

const SKY_VS = `#version 300 es
layout(location = 0) in vec2 aPos;
out vec2 vNdc;
void main() { vNdc = aPos; gl_Position = vec4(aPos, 0.9999, 1.0); }`;

const SKY_FS = `#version 300 es
precision highp float;
in vec2 vNdc;
uniform vec3 uRight, uUp, uFwd;   // camera basis
uniform vec2 uTan;                // tan(fov/2) * aspect, tan(fov/2)
uniform float uTime;
uniform vec3 uSun, uHorizon, uZenith;
out vec4 frag;

float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float noise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x),
             mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}

void main() {
  vec3 dir = normalize(uFwd + uRight * (vNdc.x * uTan.x) + uUp * (vNdc.y * uTan.y));
  float y = dir.y;

  // Gradient: haze at the horizon, saturating blue overhead.
  float t = clamp(y, 0.0, 1.0);
  vec3 c = mix(uHorizon, uZenith, pow(t, 0.55));

  // Sun: a hot disc, a tight glow, and a broad warm wash.
  float cosSun = dot(dir, uSun);
  c += vec3(1.0, 0.95, 0.85) * pow(max(cosSun, 0.0), 1400.0) * 3.0;
  c += vec3(1.0, 0.85, 0.60) * pow(max(cosSun, 0.0), 40.0) * 0.28;
  c += vec3(0.95, 0.80, 0.55) * pow(max(cosSun, 0.0), 5.0) * 0.10;

  // Thin high cloud: noise on the sky projected to a plane, drifting slowly.
  if (y > 0.02) {
    vec2 uv = dir.xz / (y + 0.15) * 1.6 + vec2(uTime * 0.004, uTime * 0.0015);
    float n = noise(uv) * 0.5 + noise(uv * 2.1 + 3.7) * 0.3 + noise(uv * 4.3 + 9.1) * 0.2;
    float cloud = smoothstep(0.52, 0.78, n) * smoothstep(0.02, 0.22, y) * 0.55;
    c = mix(c, vec3(0.97, 0.97, 0.99), cloud);
  }

  // Below the horizon there is nothing but the ground plane, which is fogged
  // to the same colour; a slightly darker band there hides any seam.
  if (y < 0.0) c = mix(uHorizon, uHorizon * 0.96, clamp(-y * 8.0, 0.0, 1.0));
  frag = vec4(c, 1.0);
}`;

const GROUND_VS = `#version 300 es
layout(location = 0) in vec2 aPos;   // unit quad, -1..1
uniform mat4 uViewProj;
uniform vec2 uCamXZ;
uniform float uExtent;
uniform float uDrop;
out vec3 vWorld;
void main() {
  vec2 p = uCamXZ + aPos * uExtent;
  vWorld = vec3(p.x, -uDrop, p.y);
  gl_Position = uViewProj * vec4(vWorld, 1.0);
}`;

/** Contact darkening under the chassis and at the tyres, shared by lot and ribbon. */
const CONTACT_GLSL = `
uniform vec2 uCarXZ;
uniform vec2 uCarFwd;     // unit, GL xz
uniform vec2 uCarHalf;    // half length, half width
uniform vec2 uHubXZ[4];
float contactAO(vec2 p) {
  vec2 d = p - uCarXZ;
  vec2 local = vec2(dot(d, uCarFwd), dot(d, vec2(-uCarFwd.y, uCarFwd.x)));
  vec2 q = abs(local) - uCarHalf;
  float box = max(q.x, q.y);
  float body = 1.0 - smoothstep(-0.25, 0.55, box);   // soft under the floor
  float tyre = 0.0;
  for (int i = 0; i < 4; i++) {
    float r = length(p - uHubXZ[i]);
    tyre = max(tyre, 1.0 - smoothstep(0.08, 0.34, r));
  }
  return 1.0 - body * 0.22 - tyre * 0.45;
}`;

const GROUND_FS = `#version 300 es
precision highp float;
in vec3 vWorld;
uniform vec2 uLotCentre;
uniform vec2 uLotHalf;
${COMMON_FS}
${CONTACT_GLSL}
out vec4 frag;

void main() {
  vec2 p = vWorld.xz;
  float dist = length(vWorld - uCam);
  float detail = clamp(1.0 - dist / 55.0, 0.0, 1.0);

  // ---- asphalt: aggregate speckle, coarse patching, a few seal-coat seams --
  float fine = noise(p * 22.0) * 0.55 + noise(p * 64.0) * 0.45;
  float wear = fbm(p * 0.09);
  float seam = noise(p * 0.7) * 0.6 + noise(p * 1.9) * 0.4;
  vec3 asphalt = vec3(0.30, 0.305, 0.315);
  asphalt *= 0.80 + 0.36 * wear;
  asphalt *= 0.93 + 0.14 * smoothstep(0.55, 0.62, seam);
  asphalt += (fine - 0.5) * 0.14 * detail;

  // Faded parking-stall lines: 2.75 m bays, 5.5 m deep. Real lot markings, and
  // the main optical-flow cue at speed.
  vec2 g = abs(fract(p / vec2(2.75, 5.5)) - 0.5) * vec2(2.75, 5.5);
  float line = min(g.x, g.y);
  float paint = (1.0 - smoothstep(0.04, 0.10, line)) * 0.42 * clamp(1.0 - dist / 130.0, 0.0, 1.0);
  paint *= 0.35 + 0.65 * noise(p * 3.0);   // worn and patchy
  asphalt = mix(asphalt, vec3(0.66, 0.65, 0.62), paint);

  // ---- grass beyond the lot, with a concrete kerb along the edge ----------
  vec2 q = abs(p - uLotCentre) - uLotHalf;
  float edge = max(q.x, q.y);              // <0 inside the lot
  vec3 albedo = asphalt;
  if (edge > -0.5) {
    float grassN = fbm(p * 0.35) * 0.6 + noise(p * 6.0) * 0.4;
    vec3 grass = mix(vec3(0.27, 0.36, 0.17), vec3(0.42, 0.50, 0.22), grassN);
    grass *= 0.9 + 0.2 * noise(p * 0.05);
    vec3 kerb = vec3(0.60, 0.60, 0.57) * (0.9 + 0.2 * noise(p * 9.0));
    albedo = mix(asphalt, kerb, smoothstep(-0.05, 0.12, edge));
    albedo = mix(albedo, grass, smoothstep(0.45, 0.9, edge + 0.4 * (grassN - 0.5)));
  }

  // ---- lighting: flat plane, so the normal is up ----
  vec3 n = vec3(0.0, 1.0, 0.0);
  float sh = shadowAt(vWorld, n);
  float ao = contactAO(p);
  float grazing = 1.0 - smoothstep(0.0, 0.6, abs(normalize(uCam - vWorld).y));
  vec3 c = lighting(albedo, n, vWorld, sh * ao, 0.10 + 0.18 * grazing, 30.0);
  c *= ao;
  frag = vec4(applyFog(c, vWorld), 1.0);
}`;

const RIBBON_VS = `#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aS;
layout(location = 2) in float aSide;
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
uniform float uLength;
uniform float uClosed;
${COMMON_FS}
${CONTACT_GLSL}
out vec4 frag;

void main() {
  vec2 p = vWorld.xz;
  float dist = length(vWorld - uCam);
  float detail = clamp(1.0 - dist / 70.0, 0.0, 1.0);

  // Rubbered-in racing surface: darker than the surrounding lot, darkest in
  // the middle where the cars actually run, with marbles at the edges.
  float mid = 1.0 - abs(vSide);
  vec3 c = vec3(0.245, 0.248, 0.256) - 0.05 * smoothstep(0.1, 0.9, mid);
  c *= 0.86 + 0.28 * fbm(p * 0.12);
  c += (noise(p * 18.0) - 0.5) * 0.06 * detail;
  float marbles = smoothstep(0.86, 1.0, abs(vSide)) * noise(p * 30.0) * detail;
  c = mix(c, vec3(0.14, 0.14, 0.15), marbles * 0.5);

  // Start/finish: a painted chequer band. Autocross also gets one at the end.
  float atStart = 1.0 - smoothstep(0.0, 1.2, vS);
  float atEnd = uClosed > 0.5 ? 0.0 : 1.0 - smoothstep(0.0, 1.2, uLength - vS);
  float band = max(atStart, atEnd);
  if (band > 0.0) {
    float sq = mod(floor(vSide * 6.0) + floor(vS / 0.6), 2.0);
    c = mix(c, mix(vec3(0.08), vec3(0.88), sq), band);
  }

  vec3 n = vec3(0.0, 1.0, 0.0);
  float sh = shadowAt(vWorld, n);
  float ao = contactAO(p);
  float grazing = 1.0 - smoothstep(0.0, 0.6, abs(normalize(uCam - vWorld).y));
  c = lighting(c, n, vWorld, sh * ao, 0.12 + 0.22 * grazing, 34.0);
  c *= ao;
  frag = vec4(applyFog(c, vWorld), 1.0);
}`;

const PROP_VS = `#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
layout(location = 3) in vec3 iOffset;
layout(location = 4) in float iDown;
layout(location = 5) in float iTint;
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
${COMMON_FS}
out vec4 frag;
void main() {
  vec3 n = normalize(vNormal);
  float sh = shadowAt(vWorld, n);
  // The base of a cone sits in its own contact shadow.
  float ao = 0.72 + 0.28 * clamp(vWorld.y / 0.35, 0.0, 1.0);
  vec3 c = lighting(vColor, n, vWorld, sh, 0.25, 22.0) * ao;
  frag = vec4(applyFog(c, vWorld), 1.0);
}`;

const CAR_VS = `#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
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
uniform vec4 uOverride;   // rgb to blend toward, alpha = how much
uniform float uGloss;     // 1 = painted bodywork, 0 = matte scenery
${COMMON_FS}
out vec4 frag;
void main() {
  // Two-sided: inside a cockpit you are looking at the back of half the
  // bodywork, and an unlit black shell there ruins the whole effect.
  vec3 n = normalize(vNormal);
  if (!gl_FrontFacing) n = -n;

  vec3 base = mix(vColor, uOverride.rgb, uOverride.a);
  float sh = shadowAt(vWorld, n);
  // Darker, glossier paint takes a sharper highlight than a matte tyre.
  float lum = dot(base, vec3(0.3, 0.5, 0.2));
  float gloss = mix(18.0, 70.0, uGloss);
  float specStrength = mix(0.08, 0.30 + 0.25 * (1.0 - lum), uGloss);
  vec3 c = lighting(base, n, vWorld, sh, specStrength, gloss);

  // Fresnel rim from the sky, which is what makes a curved panel read as
  // curved rather than flat-shaded.
  vec3 v = normalize(uCam - vWorld);
  float fres = pow(1.0 - max(dot(n, v), 0.0), 4.0);
  c += uHorizon * fres * (0.06 + 0.12 * uGloss);
  frag = vec4(applyFog(c, vWorld), 1.0);
}`;

// Depth-only programs for the shadow pass. Same attribute layout as the lit
// programs, so the VAOs are shared.
const DEPTH_CAR_VS = `#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
uniform mat4 uModel;
void main() { gl_Position = uViewProj * (uModel * vec4(aPos, 1.0)); }`;

const DEPTH_PROP_VS = `#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 3) in vec3 iOffset;
layout(location = 4) in float iDown;
uniform mat4 uViewProj;
void main() {
  vec3 p = aPos;
  if (iDown > 0.5) { p = vec3(p.x, p.z, -p.y); p.y += 0.15; }
  gl_Position = uViewProj * vec4(p + iOffset, 1.0);
}`;

const DEPTH_FS = `#version 300 es
precision mediump float;
out vec4 frag;
void main() { frag = vec4(1.0); }`;

// ------------------------------------------------------------------ meshes ---

/** An FSAE course cone: 18 in tall, orange, with the white reflective band. */
function coneMesh() {
  const pos = [], nrm = [], col = [];
  const H = 0.46, R = 0.145, SEG = 12;
  const orange = [0.96, 0.34, 0.06];
  const white = [0.93, 0.93, 0.91];
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

/** A light pole with a head, so the lot's poles read as poles, not sticks. */
function poleMesh() {
  const a = boxMesh(0.30, 9.0, 0.30, [0.36, 0.37, 0.40]);
  const head = boxMesh(1.6, 0.35, 0.5, [0.30, 0.31, 0.33]);
  const pos = new Float32Array(a.position.length + head.position.length);
  const nrm = new Float32Array(pos.length);
  const col = new Float32Array(pos.length);
  pos.set(a.position); nrm.set(a.normal); col.set(a.color);
  for (let i = 0; i < head.position.length; i += 3) {
    pos[a.position.length + i] = head.position[i] + 0.55;
    pos[a.position.length + i + 1] = head.position[i + 1] + 8.8;
    pos[a.position.length + i + 2] = head.position[i + 2];
  }
  nrm.set(head.normal, a.normal.length);
  col.set(head.color, a.color.length);
  return { position: pos, normal: nrm, color: col, count: pos.length / 3 };
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
    this.progDepthCar = program(gl, DEPTH_CAR_VS, DEPTH_FS);
    this.progDepthProp = program(gl, DEPTH_PROP_VS, DEPTH_FS);

    // Uniform locations, looked up once: getUniformLocation every frame is
    // both slow and a string allocation per call.
    this.u = {};
    for (const [name, prog] of Object.entries({
      sky: this.progSky, ground: this.progGround, ribbon: this.progRibbon,
      prop: this.progProp, car: this.progCar, depthCar: this.progDepthCar, depthProp: this.progDepthProp,
    })) {
      const map = {};
      const n = gl.getProgramParameter(prog, gl.ACTIVE_UNIFORMS);
      for (let i = 0; i < n; i++) {
        const info = gl.getActiveUniform(prog, i);
        const base = info.name.replace(/\[0\]$/, "");
        map[base] = gl.getUniformLocation(prog, info.name);
      }
      this.u[name] = map;
    }

    this.quad = quadVao(gl);
    this.groundQuad = quadVao(gl);

    this.cone = this.makeInstanced(coneMesh(), 4096);
    this.post = this.makeInstanced(boxMesh(0.12, 2.1, 0.12, [0.85, 0.85, 0.88]), 8);
    this.pole = this.makeInstanced(poleMesh(), 64);

    this.shadow = this.makeShadowMap(SHADOW_SIZE);

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
    this.lightView = mat4();
    this.lightProj = mat4();
    this.lightViewProj = mat4();
    this.shadowMat = mat4();
    this.biasMat = new Float32Array([0.5, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0.5, 0, 0.5, 0.5, 0.5, 1]);
    this._a = mat4(); this._b = mat4();
    this._t = [mat4(), mat4(), mat4(), mat4(), mat4()];
    this._hubXZ = new Float32Array(8);
    this._wheelMats = [mat4(), mat4(), mat4(), mat4()];
    this.fovDeg = 78;
    this.time = 0;

    this.lot = { cx: 0, cz: 0, hx: 400, hz: 400 };
    this.env = null;
    this.venue = null;
    this.ribbon = null;
    this.gatePosts = [];
    this.poles = [];

    /** Per-frame counters, for the report and the console. */
    this.stats = { drawCalls: 0, cones: 0, triangles: 0 };
  }

  // ------------------------------------------------------------ car meshes ---

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
  useBodyModel(body, axles = null, track = null) {
    const gl = this.gl;
    const mesh = this.car.body;
    if (mesh?.vao) gl.deleteVertexArray(mesh.vao);
    for (const b of Object.values(mesh?.buffers ?? {})) gl.deleteBuffer(b);
    if (body) {
      this.car.body = this.makeMesh(body);
      this.bodyModel = true;
      // Draw the wheels in the bays the bodywork actually has, rather than at
      // the stations in the vehicle parameters. When the two disagree the model
      // is the thing you can see, and wheels floating outside their arches look
      // broken in a way that a slightly wrong wheelbase does not.
      this.bodyHubs = axles
        ? [
            { name: "FL", x: axles.front, y: track.tireRadius, z: -track.front / 2, front: true },
            { name: "FR", x: axles.front, y: track.tireRadius, z: track.front / 2, front: true },
            { name: "RL", x: axles.rear, y: track.tireRadius, z: -track.rear / 2, front: false },
            { name: "RR", x: axles.rear, y: track.tireRadius, z: track.rear / 2, front: false },
          ]
        : null;
    } else {
      this.bodyHubs = null;
      this.car.body = this.makeMesh(buildCarMeshes(this.carParams ?? null).body);
      this.bodyModel = false;
    }
  }

  /**
   * Re-stretch the body onto new geometry.
   *
   * The bodywork is authored once at SDM26's real stations and mapped onto the
   * live wheelbase and track, so changing either moves the car you see as well
   * as the car you drive. Only the body needs it -- the wheels are placed by
   * their own transforms, which already read the live hub positions.
   */
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

  /** Static mesh drawn with its own model matrix (the car parts, scenery). */
  makeMesh(mesh) {
    const gl = this.gl;
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const attach = (data, loc, size) => {
      const buf = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 0, 0);
      return buf;
    };
    const buffers = {
      position: attach(mesh.position, A_POS, 3),
      normal: attach(mesh.normal, A_NORMAL, 3),
      color: attach(mesh.color, A_COLOR, 3),
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

    const attach = (data, loc, size) => {
      const buf = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 0, 0);
      return buf;
    };
    attach(mesh.position, A_POS, 3);
    attach(mesh.normal, A_NORMAL, 3);
    attach(mesh.color, A_COLOR, 3);

    // Interleaved per-instance data: offset(3), down(1), tint(1)
    const stride = 5 * 4;
    const instBuf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, instBuf);
    gl.bufferData(gl.ARRAY_BUFFER, maxInstances * stride, gl.DYNAMIC_DRAW);
    const bindInst = (loc, size, offset) => {
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, size, gl.FLOAT, false, stride, offset);
      gl.vertexAttribDivisor(loc, 1);
    };
    bindInst(I_OFFSET, 3, 0);
    bindInst(I_DOWN, 1, 12);
    bindInst(I_TINT, 1, 16);

    gl.bindVertexArray(null);
    return {
      vao, instBuf, count: mesh.count, maxInstances,
      data: new Float32Array(maxInstances * 5),
      n: 0,
    };
  }

  /** Depth texture plus framebuffer for the sun's shadow map. */
  makeShadowMap(size) {
    const gl = this.gl;
    const tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.texStorage2D(gl.TEXTURE_2D, 1, gl.DEPTH_COMPONENT24, size, size);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_COMPARE_MODE, gl.COMPARE_REF_TO_TEXTURE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_COMPARE_FUNC, gl.LEQUAL);
    const fbo = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.TEXTURE_2D, tex, 0);
    gl.drawBuffers([gl.NONE]);
    gl.readBuffer(gl.NONE);
    const ok = gl.checkFramebufferStatus(gl.FRAMEBUFFER) === gl.FRAMEBUFFER_COMPLETE;
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.bindTexture(gl.TEXTURE_2D, null);
    if (!ok) console.warn("shadow map framebuffer incomplete; shadows disabled");
    return { tex, fbo, size, ok };
  }

  // ----------------------------------------------------------------- track ---

  /** Build the course ribbon and the static props for a track. */
  setTrack(track) {
    const gl = this.gl;
    this.track = track;
    if (this.venue) this.deleteMesh(this.venue);
    if (this.env) this.deleteMesh(this.env);
    if (this.ribbon) { gl.deleteVertexArray(this.ribbon.vao); }
    this.venue = null;
    this.env = null;
    this.ribbon = null;

    // A venue supplies its own surfaces -- banking, apron, infield, wall and
    // fence -- so there is no course ribbon and no gate posts to place.
    if (track.kind === "venue") {
      this.venue = this.makeMesh(buildVenueMesh(track));
      this.gatePosts = [];
      this.poles = [];
      const b = bounds(track.rings.wallLine);
      this.env = this.makeMesh(buildEnvironmentMesh(b, "venue"));
      // Grass everywhere under the venue: the lot is a zero-size rectangle.
      this.lot = { cx: (b.minX + b.maxX) / 2, cz: -(b.minY + b.maxY) / 2, hx: 0, hz: 0 };
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
    const attach = (arr, loc, size) => {
      const buf = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(arr), gl.STATIC_DRAW);
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 0, 0);
    };
    attach(pos, 0, 3);
    attach(sArr, 1, 1);
    attach(side, 2, 1);
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

    const b = bounds(track.center);
    this.poles = [];
    for (let i = 0; i < 26; i++) {
      const a = (i / 26) * Math.PI * 2;
      const rx = (b.maxX - b.minX) / 2 + 45, ry = (b.maxY - b.minY) / 2 + 45;
      this.poles.push({
        x: (b.minX + b.maxX) / 2 + Math.cos(a) * rx,
        y: (b.minY + b.maxY) / 2 + Math.sin(a) * ry,
      });
    }

    // The paved lot: the course plus a generous apron, grass beyond it.
    this.lot = {
      cx: (b.minX + b.maxX) / 2, cz: -(b.minY + b.maxY) / 2,
      hx: (b.maxX - b.minX) / 2 + 110, hz: (b.maxY - b.minY) / 2 + 110,
    };
    this.env = this.makeMesh(buildEnvironmentMesh(b, "course"));
  }

  deleteMesh(mesh) {
    const gl = this.gl;
    if (mesh?.vao) gl.deleteVertexArray(mesh.vao);
    for (const b of Object.values(mesh?.buffers ?? {})) gl.deleteBuffer(b);
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

  // ----------------------------------------------------------------- frame ---

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
    this.stats.drawCalls = 0;
    this.stats.triangles = 0;
    this.time = performance.now() / 1000;

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
      const a = s.view.orbitAngle || 0;
      const r = s.view.radius ?? 3.4;
      const cx = cam.x;
      const cz = -cam.y;
      const focusY = s.view.focusHeight ?? 0.45;
      eye = [cx + Math.cos(a) * r, s.view.height, cz + Math.sin(a) * r];
      const to = [cx - eye[0], focusY - eye[1], cz - eye[2]];
      forward = normalize(to);
      const dotUp = forward[1];
      up = normalize([-forward[0] * dotUp, 1 - dotUp * dotUp, -forward[2] * dotUp]);
    } else {
      eye = transformPoint(this.camFrame, [s.view.ahead, s.view.height + s.heaveM, 0]);
      const pitched = this.chain(T[4], [this.camFrame, rotZ(T[0], s.view.pitchOffset || 0)]);
      forward = normalize(transformDir(pitched, [1, 0, 0]));
      up = normalize(transformDir(pitched, [0, 1, 0]));
    }

    const fovRad = ((this.fovDeg + (s.fovBoost || 0)) * Math.PI) / 180;
    // Near 0.06 m: the steering wheel is 0.28 m ahead of the eye, and the
    // walkaround floors its radius at 1.2 m. Far 1100 m clears the tree line
    // on the far side of the endurance course with the sky quad behind it.
    perspective(this.proj, fovRad, aspect, 0.06, 1100);
    lookAlong(this.view, eye, forward, up);
    multiply(this.viewProj, this.proj, this.view);

    // ---- wheel transforms and hub positions, used by both passes ----
    this.placeWheels(s);

    // ---- shadow pass ----
    this.drawShadowMap(s, cam);

    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clearColor(HORIZON[0], HORIZON[1], HORIZON[2], 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    // --- sky ---
    gl.depthMask(false);
    gl.useProgram(this.progSky);
    const us = this.u.sky;
    const right = normalize(cross3(forward, up));
    const tanH = Math.tan(fovRad / 2);
    gl.uniform3f(us.uRight, right[0], right[1], right[2]);
    gl.uniform3f(us.uUp, up[0], up[1], up[2]);
    gl.uniform3f(us.uFwd, forward[0], forward[1], forward[2]);
    gl.uniform2f(us.uTan, tanH * aspect, tanH);
    gl.uniform1f(us.uTime, this.time);
    gl.uniform3f(us.uSun, SUN[0], SUN[1], SUN[2]);
    gl.uniform3f(us.uHorizon, HORIZON[0], HORIZON[1], HORIZON[2]);
    gl.uniform3f(us.uZenith, ZENITH[0], ZENITH[1], ZENITH[2]);
    gl.bindVertexArray(this.quad);
    this.drawArrays(gl.TRIANGLES, 0, 6);
    gl.depthMask(true);

    // Shadow map on unit 0 for every lit program.
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.shadow.tex);

    // --- ground ---
    gl.useProgram(this.progGround);
    const ug = this.u.ground;
    this.setCommon(ug, eye);
    this.setContact(ug, cam);
    gl.uniformMatrix4fv(ug.uViewProj, false, this.viewProj);
    gl.uniform2f(ug.uCamXZ, eye[0], eye[2]);
    gl.uniform1f(ug.uExtent, 900);
    // The venue paves its own ground; sink the procedural lot below it.
    gl.uniform1f(ug.uDrop, this.venue ? 0.35 : 0.0);
    gl.uniform2f(ug.uLotCentre, this.lot.cx, this.lot.cz);
    gl.uniform2f(ug.uLotHalf, this.lot.hx, this.lot.hz);
    gl.bindVertexArray(this.groundQuad);
    // The course ribbon lies 12 mm above this plane; push the plane back in
    // depth so the two never fight at distance.
    gl.enable(gl.POLYGON_OFFSET_FILL);
    gl.polygonOffset(1.0, 2.0);
    this.drawArrays(gl.TRIANGLES, 0, 6);
    gl.disable(gl.POLYGON_OFFSET_FILL);

    // --- venue surfaces and the distant environment (matte, static) ---
    if (this.venue || this.env) {
      gl.useProgram(this.progCar);
      const uc = this.u.car;
      this.setCommon(uc, eye);
      gl.uniformMatrix4fv(uc.uViewProj, false, this.viewProj);
      gl.uniform4f(uc.uOverride, 0, 0, 0, 0);
      gl.uniform1f(uc.uGloss, 0.0);
      identity(this._a);
      gl.uniformMatrix4fv(uc.uModel, false, this._a);
      if (this.venue) {
        gl.bindVertexArray(this.venue.vao);
        this.drawArrays(gl.TRIANGLES, 0, this.venue.count);
      }
      if (this.env) {
        gl.enable(gl.CULL_FACE);
        gl.bindVertexArray(this.env.vao);
        this.drawArrays(gl.TRIANGLES, 0, this.env.count);
        gl.disable(gl.CULL_FACE);
      }
    }

    // --- course ribbon ---
    if (this.ribbon) {
      gl.useProgram(this.progRibbon);
      const ur = this.u.ribbon;
      this.setCommon(ur, eye);
      this.setContact(ur, cam);
      gl.uniformMatrix4fv(ur.uViewProj, false, this.viewProj);
      gl.uniform1f(ur.uLength, this.track.length);
      gl.uniform1f(ur.uClosed, this.track.closed ? 1 : 0);
      gl.bindVertexArray(this.ribbon.vao);
      this.drawArrays(gl.TRIANGLE_STRIP, 0, this.ribbon.count);
    }

    // --- instanced props ---
    gl.enable(gl.CULL_FACE);
    gl.useProgram(this.progProp);
    const upr = this.u.prop;
    this.setCommon(upr, eye);
    gl.uniformMatrix4fv(upr.uViewProj, false, this.viewProj);
    this.drawInstanced(this.cone);
    this.drawInstanced(this.post);
    this.drawInstanced(this.pole);
    gl.disable(gl.CULL_FACE);

    this.drawCar(s, eye);

    gl.bindVertexArray(null);
  }

  /** Uniforms every lit program shares. */
  setCommon(u, eye) {
    const gl = this.gl;
    gl.uniform3f(u.uSun, SUN[0], SUN[1], SUN[2]);
    gl.uniform3f(u.uCam, eye[0], eye[1], eye[2]);
    gl.uniform3f(u.uHorizon, HORIZON[0], HORIZON[1], HORIZON[2]);
    gl.uniform3f(u.uZenith, ZENITH[0], ZENITH[1], ZENITH[2]);
    gl.uniform1i(u.uShadow, 0);
    gl.uniformMatrix4fv(u.uShadowMat, false, this.shadowMat);
  }

  /** Contact-shadow uniforms for the ground and ribbon. */
  setContact(u, cam) {
    const gl = this.gl;
    const fx = Math.cos(cam.psi), fz = -Math.sin(cam.psi);
    gl.uniform2f(u.uCarXZ, cam.x, -cam.y);
    gl.uniform2f(u.uCarFwd, fx, fz);
    gl.uniform2f(u.uCarHalf, (GEO.frontWingTip - GEO.rearWingTip) / 2 * 0.9, GEO.trackFront / 2 - 0.05);
    gl.uniform2fv(u.uHubXZ, this._hubXZ);
  }

  /**
   * Wheel model matrices, and the hub contact points in world xz. Computed
   * once per frame and used by the shadow pass, the main pass and the
   * contact darkening on the ground.
   */
  placeWheels(s) {
    const T = this._t;
    const w = s.wheels;
    const hubs = this.carModel?.hubs ?? this.bodyHubs ?? s.hubs ?? HUBS;
    for (let i = 0; i < 4; i++) {
      const hub = hubs[i];
      // Steer rotates about the kingpin (local Y); spin is about the hub axis
      // (local Z) AFTER the steer. The left pair is mirrored across the wheel
      // plane so the dished rim faces outboard on both sides (see carmesh).
      const mirrored = hub.z < 0;
      const m = this._wheelMats[i];
      multiply(this._a, this.chassis, translation(T[0], hub.x, hub.y, hub.z));
      multiply(this._b, this._a, rotY(T[1], hub.front ? w.steerRad : 0));
      multiply(this._a, this._b, rotZ(T[2], -(hub.front ? w.spinFront : w.spinRear)));
      if (mirrored) multiply(m, this._a, scale(T[3], 1, 1, -1));
      else m.set(this._a);
      this._hubXZ[i * 2] = m[12];
      this._hubXZ[i * 2 + 1] = m[14];
    }
  }

  /** Render the sun's view of the car and nearby cones into the depth map. */
  drawShadowMap(s, cam) {
    const gl = this.gl;
    const sm = this.shadow;

    // Orthographic box around the car, looking along the sun.
    const cx = cam.x, cz = -cam.y;
    const eye = [cx + SUN[0] * 60, SUN[1] * 60, cz + SUN[2] * 60];
    lookAlong(this.lightView, eye, [-SUN[0], -SUN[1], -SUN[2]], [0, 1, 0]);
    ortho(this.lightProj, -SHADOW_HALF, SHADOW_HALF, -SHADOW_HALF, SHADOW_HALF, 1, 140);
    multiply(this.lightViewProj, this.lightProj, this.lightView);
    multiply(this.shadowMat, this.biasMat, this.lightViewProj);

    // Fill the cone instance buffer once for both passes.
    const cones = this.track.conesNear(s.car.x, s.car.y, CONE_DRAW_RANGE);
    this.fillCones(this.cone, cones);
    this.fillPoints(this.post, this.gatePosts);
    this.fillPoints(this.pole, this.poles);
    this.stats.cones = this.cone.n;

    if (!sm.ok) return;
    gl.bindFramebuffer(gl.FRAMEBUFFER, sm.fbo);
    gl.viewport(0, 0, sm.size, sm.size);
    gl.clear(gl.DEPTH_BUFFER_BIT);
    gl.colorMask(false, false, false, false);
    gl.enable(gl.POLYGON_OFFSET_FILL);
    gl.polygonOffset(2.0, 4.0);

    // Car: body and tyres. Rims sit inside the tyre silhouette and the
    // steering wheel is inside the body, so neither adds anything here.
    gl.useProgram(this.progDepthCar);
    const ud = this.u.depthCar;
    gl.uniformMatrix4fv(ud.uViewProj, false, this.lightViewProj);
    gl.uniformMatrix4fv(ud.uModel, false, this.chassis);
    gl.bindVertexArray(this.car.body.vao);
    this.drawArrays(gl.TRIANGLES, 0, this.car.body.count);
    for (let i = 0; i < 4; i++) {
      gl.uniformMatrix4fv(ud.uModel, false, this._wheelMats[i]);
      gl.bindVertexArray(this.car.tire.vao);
      this.drawArrays(gl.TRIANGLES, 0, this.car.tire.count);
    }

    // Cones and posts. The instance buffer holds everything within draw
    // range; the ortho box clips the rest away for free.
    gl.useProgram(this.progDepthProp);
    gl.uniformMatrix4fv(this.u.depthProp.uViewProj, false, this.lightViewProj);
    this.drawInstanced(this.cone);
    this.drawInstanced(this.post);
    this.drawInstanced(this.pole);

    gl.disable(gl.POLYGON_OFFSET_FILL);
    gl.colorMask(true, true, true, true);
  }

  /** The car itself: body, four wheels that steer and spin, steering wheel. */
  drawCar(s, eye) {
    const gl = this.gl;
    const T = this._t;
    const prog = this.progCar;
    const uc = this.u.car;

    gl.useProgram(prog);
    this.setCommon(uc, eye);
    gl.uniformMatrix4fv(uc.uViewProj, false, this.viewProj);

    const part = (mesh, model, ov, gloss) => {
      gl.uniformMatrix4fv(uc.uModel, false, model);
      if (ov) gl.uniform4f(uc.uOverride, ov[0], ov[1], ov[2], ov[3]);
      else gl.uniform4f(uc.uOverride, 0, 0, 0, 0);
      gl.uniform1f(uc.uGloss, gloss);
      gl.bindVertexArray(mesh.vao);
      this.drawArrays(gl.TRIANGLES, 0, mesh.count);
    };

    part(this.car.body, this.chassis, null, 1.0);

    const w = s.wheels;
    for (let i = 0; i < 4; i++) {
      part(this.car.tire, this._wheelMats[i], null, 0.15);
      // Fade the gold spokes toward the tyre as the wheel speeds up. Five
      // spokes at 20 rev/s would otherwise strobe into a stationary-looking
      // mess at 60 Hz; this reads as motion blur instead.
      part(this.car.rim, this._wheelMats[i], this._rimOverride(w.rimFade), 0.8);
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
    const sc = this.carModel?.steerCentre ?? GEO.steerCentre;
    this.chain(this.model, [
      this.chassis,
      translation(T[0], sc[0], sc[1], sc[2]),
      basis,
      rotZ(T[1], -w.steerRad * (w.steerRatio ?? GEO.steeringRatio)),
    ]);
    part(this.car.steeringWheel, this.model, null, 0.5);
  }

  _rimOverride(fade) {
    const o = this._rimOv ?? (this._rimOv = [0.15, 0.16, 0.18, 0]);
    o[3] = fade;
    return o;
  }

  /** Write cone instances straight from the track's cone records. */
  fillCones(mesh, list) {
    const n = Math.min(list.length, mesh.maxInstances);
    const d = mesh.data;
    for (let i = 0; i < n; i++) {
      const c = list[i];
      d[i * 5 + 0] = c.x;
      d[i * 5 + 1] = 0;
      d[i * 5 + 2] = -c.y;
      d[i * 5 + 3] = c.down ? 1 : 0;
      d[i * 5 + 4] = c.down ? 0.72 : 1;
    }
    this.uploadInstances(mesh, n);
  }

  fillPoints(mesh, list) {
    const n = Math.min(list.length, mesh.maxInstances);
    const d = mesh.data;
    for (let i = 0; i < n; i++) {
      d[i * 5 + 0] = list[i].x;
      d[i * 5 + 1] = 0;
      d[i * 5 + 2] = -list[i].y;
      d[i * 5 + 3] = 0;
      d[i * 5 + 4] = 1;
    }
    this.uploadInstances(mesh, n);
  }

  uploadInstances(mesh, n) {
    const gl = this.gl;
    mesh.n = n;
    if (n === 0) return;
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.instBuf);
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, mesh.data, 0, n * 5);
  }

  drawInstanced(mesh) {
    if (mesh.n === 0) return;
    const gl = this.gl;
    gl.bindVertexArray(mesh.vao);
    gl.drawArraysInstanced(gl.TRIANGLES, 0, mesh.count, mesh.n);
    this.stats.drawCalls++;
    this.stats.triangles += (mesh.count / 3) * mesh.n;
  }

  drawArrays(mode, first, count) {
    this.gl.drawArrays(mode, first, count);
    this.stats.drawCalls++;
    this.stats.triangles += mode === this.gl.TRIANGLE_STRIP ? count - 2 : count / 3;
  }
}

// ------------------------------------------------------------------ helpers ---

function bounds(points) {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const [x, y] of points) {
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
  }
  return { minX, maxX, minY, maxY };
}

function cross3(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

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

function quadVao(gl) {
  const vao = gl.createVertexArray();
  gl.bindVertexArray(vao);
  const buf = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, buf);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
    -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1,
  ]), gl.STATIC_DRAW);
  gl.enableVertexAttribArray(A_POS);
  gl.vertexAttribPointer(A_POS, 2, gl.FLOAT, false, 0, 0);
  gl.bindVertexArray(null);
  return vao;
}
