// First-person WebGL2 renderer.
//
// The world is a flat asphalt lot -- which is what FSAE Michigan actually is --
// so almost all of the sense of speed has to come from surface detail rather
// than scenery. Hence the procedurally textured ground with faded parking-stall
// lines: they are authentic to the venue AND they are the optical flow that
// tells you how fast you are going at 25 m/s two feet off the deck.
//
// Lighting is one model shared by every surface: a sun whose colour follows
// its elevation, an analytic sky gradient that is at once the backdrop, the
// ambient term, the fog colour and the reflection in the paint, GGX materials
// with a roughness per surface, two cascaded orthographic shadow maps that
// follow the car, and a distance fog toward the sky's own colour so the ground
// dissolves into the horizon rather than stopping at an edge.
//
// Colours are AUTHORED in display space (the asphalt albedo is 0.30, not
// 0.03), which every mesh builder relies on; the shaders decode them to linear,
// light there, and encode back out through exposure, a filmic curve and sRGB.
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

// Far enough that a cone arrives out of the fog rather than popping in on
// the endurance straights; the instance buffer holds 4096, plenty.
const CONE_DRAW_RANGE = 280; // m
/** How solid the replay ghost is drawn. */
const GHOST_ALPHA = 0.55;
/** Skid marks kept on the surface: segments in a ring, oldest overwritten. */
const SKID_MAX = 8000;
/** Floats per segment: two triangles of (x, y, z, alpha). */
const SKID_FLOATS = 6 * 4;
/** Half-width of a mark, metres -- a bit under the slick's 7 in, since only
 *  the loaded shoulder really scrubs. */
const SKID_HALF_W = 0.07;
/** Height of the marks above the deck: over the ribbon, under a cone's plate. */
const SKID_Y = 0.018;
const SHADOW_SIZE = 2048;     // texels, per cascade
// Two cascades: a tight box for the car's own shadow and a wide one so the
// cones down the course carry shadows instead of popping into them.
const SHADOW_HALF = [14, 64]; // m, half-extent of each cascade
const SHADOW_DEPTH = 90;      // m, half depth range of each cascade along the sun

// Attribute locations are fixed so one VAO can be drawn by the lit program
// and by the depth-only program alike.
const A_POS = 0, A_NORMAL = 1, A_COLOR = 2, I_OFFSET = 3, I_DOWN = 4, I_TINT = 5, I_DIR = 6;
/** Floats per prop instance: offset(3), down(1), tint(1), dir(1). */
const INST_FLOATS = 6;
/** How long a struck cone takes to land, ms. */
const CONE_TUMBLE_MS = 320;

/** The dash panel's texture, at the DISPLAY's own 108:65 -- not the case's.
 *  Sized so the smallest type is still a couple of pixels tall from the
 *  driver's seat, which is what decides it, not the panel's physical size. */
const DASH_TEX_W = 768, DASH_TEX_H = 462;

/** How bright the panel is, in the scene's linear units. A dash in daylight
 *  is about as bright as sunlit white paper -- enough to read against the
 *  sky, not so much that it glows. */
const DASH_NITS = 1.9;

// Sun: mid-afternoon, from the south-west, high enough that a 15 m grandstand
// throws a shadow without the cones throwing 3 m ones.
const SUN = normalize([0.42, 0.74, 0.52]);

/**
 * Sun and sky radiance from the sun's elevation.
 *
 * One tiny atmosphere: the beam loses light to Rayleigh scattering along its
 * air mass (Kasten-Young), so the sun goes warm as it drops; what the beam
 * loses is what colours the sky, blue at the zenith and washing out to a warm
 * haze at the horizon where the path is longest. Everything is in linear
 * radiance, normalised so that a white Lambertian surface facing the sun
 * returns the sun value -- the shaders then apply exposure, a filmic curve and
 * the sRGB transfer at the very end.
 *
 * The ground bounce is what lights the underside of the car: the lot is grey
 * asphalt lit by the same sun and sky, and it is a hemisphere's worth of it.
 */
function sunSky(sun) {
  const elev = Math.asin(Math.max(-1, Math.min(1, sun[1])));
  const zenDeg = 90 - (elev * 180) / Math.PI;
  const cosZ = Math.max(sun[1], 0.02);
  const airMass = 1 / (cosZ + 0.15 * Math.pow(Math.max(93.885 - zenDeg, 0.1), -1.253));
  // Optical depth per channel at one air mass: Rayleigh plus a little aerosol.
  const tau = [0.062, 0.118, 0.255];
  const T = tau.map((t) => Math.exp(-t * airMass));
  const SUN_SCALE = 3.4;
  const sunCol = T.map((t) => t * SUN_SCALE);

  // Sky brightness follows the sun's height; its hue is the scattered
  // complement of the beam, so it goes deeper blue as the sun climbs.
  const day = Math.pow(Math.max(sun[1], 0.0), 0.6);
  const zenith = [0.135, 0.285, 0.64].map((c) => c * (0.25 + 0.90 * day));
  // Horizon: the long path scatters everything, so it is brighter and far
  // less saturated, tinted by the (already warmed) sun.
  const horizon = [0, 1, 2].map((i) => zenith[i] * 0.55 + (0.28 + 0.20 * T[i]) * (0.30 + 0.80 * day));
  const skyAvg = [0, 1, 2].map((i) => zenith[i] * 0.45 + horizon[i] * 0.55);
  const groundAlbedo = [0.085, 0.087, 0.090]; // linear, the lot
  const ground = [0, 1, 2].map((i) => groundAlbedo[i] * (sunCol[i] * Math.max(sun[1], 0) + skyAvg[i]));
  return { sunCol, zenith, horizon, ground };
}

const SKY = sunSky(SUN);

/** [roughness, metalness, clearcoat] per car part. */
const MAT = {
  paint: [0.38, 0.0, 1.0],   // vertex-coloured bodywork under a gloss clearcoat
  tyre: [0.82, 0.0, 0.0],    // matte rubber
  rim: [0.42, 0.75, 0.0],    // cast wheel, metal
  wheel: [0.55, 0.0, 0.25],  // carbon plate with a light lacquer
  // The dash case: a matte black moulding with no clearcoat. Drawn with
  // `wheel` it caught the sky and read as bare aluminium, which is the one
  // thing an AiM case is not.
  dash: [0.88, 0.0, 0.0],
};

/** Linear radiance -> display encoding, for the clear colour only. */
function displayEncode(c, exposure) {
  return c.map((v) => {
    const x = v * exposure;
    const t = Math.min(1, Math.max(0, (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14)));
    return Math.pow(t, 1 / 2.2);
  });
}

// ---------------------------------------------------------------- shaders ---

/**
 * Colour pipeline, sky, GGX lighting, shadow and fog shared by every fragment
 * shader.
 *
 * Vertex colours and the procedural albedos are authored in display space
 * (asphalt is 0.30, the maroon is 0.55) because that is what people can
 * reason about; `toLinear` takes them into linear radiance where the lighting
 * maths is actually valid, and `finish` brings the result back out through
 * exposure, an ACES-style filmic curve and the sRGB transfer.
 */
const COMMON_FS = `
uniform vec3 uSun;
uniform vec3 uCam;
uniform vec3 uSunCol;     // linear, diffuse-normalised
uniform vec3 uZenith;     // linear sky radiance overhead
uniform vec3 uHorizon;    // linear sky radiance at the horizon
uniform vec3 uGround;     // linear radiance bounced off the lot
uniform float uExposure;
uniform vec2 uInvRes;
uniform highp sampler2DShadow uShadow0;
uniform highp sampler2DShadow uShadow1;
uniform mat4 uShadowMat0;
uniform mat4 uShadowMat1;
uniform vec2 uShadowTexel;  // metres per texel, cascade 0 and 1

const float PI = 3.14159265;

vec3 toLinear(vec3 c) { return pow(max(c, 0.0), vec3(2.2)); }

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

// ---- sky ---------------------------------------------------------------
// The one gradient the sky quad, the ambient term, the fog and the paint's
// reflection all read, so they cannot disagree with each other.
vec3 skyRadiance(vec3 d) {
  float t = clamp(d.y, 0.0, 1.0);
  vec3 c = mix(uHorizon, uZenith, sqrt(t));
  // Aureole: forward-scattered haze around the sun.
  float cs = max(dot(d, uSun), 0.0);
  float cs2 = cs * cs, cs4 = cs2 * cs2;
  c += uSunCol * (0.018 * cs4 * cs4 + 0.006 * cs2);
  // Below the horizon the reflection sees the lot.
  if (d.y < 0.0) c = mix(uHorizon, uGround, clamp(-d.y * 3.0, 0.0, 1.0));
  return c;
}

// Diffuse irradiance for a normal: sky above, the bounced lot below, and the
// horizon band for anything facing sideways.
vec3 ambientFor(vec3 n) {
  float up = n.y * 0.5 + 0.5;
  vec3 sky = mix(uHorizon, uZenith, 0.45);
  vec3 amb = mix(uGround, sky, up);
  return mix(amb, uHorizon, (1.0 - abs(n.y)) * 0.25);
}

// ---- shadows -----------------------------------------------------------
// 1 = lit, 0 = in shadow. Four rotated-grid taps on top of the hardware 2x2
// compare, a normal-offset bias scaled to the cascade's texel so the ground
// never acnes and the car's underside never peels off its shadow, and a
// fade at each cascade edge so no boundary ever draws as a line.
const vec2 POISSON[4] = vec2[4](
  vec2(-0.94, 0.34), vec2(0.34, 0.94), vec2(0.94, -0.34), vec2(-0.34, -0.94));

float cascade(highp sampler2DShadow tex, mat4 m, vec3 world, vec3 n, float texel, float ndl, out float inside) {
  // Normal offset grows as the light grazes the surface, which is where
  // self-shadowing goes wrong.
  vec3 wp = world + n * texel * (1.2 + 2.0 * (1.0 - ndl));
  vec3 p = (m * vec4(wp, 1.0)).xyz;
  vec2 e = min(p.xy, 1.0 - p.xy);
  inside = smoothstep(0.0, 0.08, min(e.x, e.y));
  if (inside <= 0.0 || p.z > 1.0) return 1.0;
  float r = 1.4 / ${SHADOW_SIZE}.0;
  float z = p.z - 0.00025;
  float s = 0.0;
  for (int i = 0; i < 4; i++) s += texture(tex, vec3(p.xy + POISSON[i] * r, z));
  return s * 0.25;
}

float shadowAt(vec3 world, vec3 n) {
  float ndl = max(dot(n, uSun), 0.0);
  float in0, in1;
  float s0 = cascade(uShadow0, uShadowMat0, world, n, uShadowTexel.x, ndl, in0);
  if (in0 >= 1.0) return s0;
  float s1 = cascade(uShadow1, uShadowMat1, world, n, uShadowTexel.y, ndl, in1);
  return mix(mix(1.0, s1, in1), s0, in0);
}

// ---- material ----------------------------------------------------------
// GGX / Smith / Schlick, plus an analytic reflection of the sky gradient in
// place of an environment map. \`clearcoat\` lays a second, near-mirror
// dielectric layer over the base -- that is what paint is, and the sky
// sliding across a curved panel is most of what makes a car read as a car.
vec3 shade(vec3 albedo, vec3 n, vec3 world, float shadow,
           float rough, float metal, float clearcoat) {
  vec3 v = normalize(uCam - world);
  vec3 h = normalize(v + uSun);
  float ndl = max(dot(n, uSun), 0.0);
  float ndv = max(dot(n, v), 1e-3);
  float ndh = max(dot(n, h), 0.0);
  float vdh = max(dot(v, h), 0.0);

  rough = clamp(rough, 0.04, 1.0);
  float a = rough * rough, a2 = a * a;
  float dd = ndh * ndh * (a2 - 1.0) + 1.0;
  float D = a2 / (PI * dd * dd);
  float k = (rough + 1.0) * (rough + 1.0) * 0.125;
  float G = (ndv / (ndv * (1.0 - k) + k)) * (ndl / (ndl * (1.0 - k) + k));
  vec3 F0 = mix(vec3(0.04), albedo, metal);
  float w1 = 1.0 - vdh, w2 = w1 * w1;
  float fw = w2 * w2 * w1;
  vec3 F = F0 + (1.0 - F0) * fw;
  vec3 spec = D * G * F / max(4.0 * ndv * ndl, 1e-3);

  vec3 diffuse = albedo * (1.0 - metal) * (1.0 - F0);
  vec3 sun = uSunCol * ndl * shadow;
  vec3 c = (diffuse + spec * PI) * sun;

  // Ambient: hemisphere diffuse, and the sky reflected off the surface with a
  // Fresnel that a rough surface sees less of.
  vec3 amb = ambientFor(n);
  c += diffuse * amb;
  vec3 r = reflect(-v, n);
  float v1 = 1.0 - ndv, v2 = v1 * v1;
  float fv = v2 * v2 * v1;
  // A rough surface sees far less of the grazing Fresnel boost: the lobe is
  // wide, and most of it lands below the horizon of the surface.
  float gl = (1.0 - rough) * (1.0 - rough);
  vec3 Fenv = F0 + (max(vec3(gl), F0) - F0) * fv;
  vec3 sky = skyRadiance(r);
  vec3 env = mix(sky, amb, rough * rough);
  c += env * Fenv * (1.0 - 0.5 * rough);

  if (clearcoat > 0.0) {
    float ca = 0.06 * 0.06;
    float cdd = ndh * ndh * (ca - 1.0) + 1.0;
    float cD = ca / (PI * cdd * cdd);
    float cF = 0.04 + 0.96 * fw;
    float cG = (ndv / (ndv * 0.95 + 0.05)) * (ndl / (ndl * 0.95 + 0.05));
    c += clearcoat * cD * cG * cF / max(4.0 * ndv * ndl, 1e-3) * PI * sun;
    c += clearcoat * sky * (0.03 + 0.72 * fv);
  }
  return c;
}

// ---- fog ---------------------------------------------------------------
// Toward the sky's own colour in that direction, so the ground dissolves
// into the horizon rather than stopping at an edge.
vec3 applyFog(vec3 c, vec3 world) {
  vec3 d = world - uCam;
  float dist = length(d);
  float f = 1.0 - exp(-pow(dist / 520.0, 1.6));
  vec3 dir = d / dist;
  vec3 fc = skyRadiance(vec3(dir.x, max(dir.y, 0.0) * 0.5, dir.z));
  return mix(c, fc, f);
}

// ---- output ------------------------------------------------------------
vec3 aces(vec3 x) {
  return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}
vec4 finish(vec3 lin) {
  // Gentle vignette: draws the eye to the road, and sells the lens.
  vec2 q = gl_FragCoord.xy * uInvRes - 0.5;
  float vig = 1.0 - 0.32 * pow(dot(q, q) * 2.6, 1.3);
  vec3 c = aces(lin * uExposure * vig);
  return vec4(pow(c, vec3(1.0 / 2.2)), 1.0);
}
`;

// ---- the dash screen -------------------------------------------------------
//
// A backlit LCD, so it is UNLIT: shading it with the sun would make it darker
// in shadow, which is the opposite of what a screen does. It still goes
// through the same exposure, tonemap and vignette as everything else, because
// a panel that skipped them would sit on top of the image rather than in it.
//
// The tonemap is repeated here rather than pulled from `COMMON_FS`: that chunk
// also declares the shadow samplers, the sky uniforms and the fog, none of
// which a lit panel has any use for, and a shader that declares uniforms
// nobody sets is one that breaks the first time somebody tidies up.
const SCREEN_VS = `#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUv;
uniform mat4 uViewProj;
uniform mat4 uModel;
out vec2 vUv;
void main() {
  vUv = aUv;
  gl_Position = uViewProj * uModel * vec4(aPos, 1.0);
}`;

const SCREEN_FS = `#version 300 es
precision highp float;
in vec2 vUv;
uniform sampler2D uPanel;
uniform float uNits;
uniform vec2 uInvRes;
uniform float uExposure;
out vec4 frag;

vec3 aces(vec3 x) {
  return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

void main() {
  // The panel is authored in sRGB on a 2D canvas; the scene is linear.
  vec3 lin = pow(texture(uPanel, vUv).rgb, vec3(2.2)) * uNits;
  vec2 q = gl_FragCoord.xy * uInvRes - 0.5;
  float vig = 1.0 - 0.32 * pow(dot(q, q) * 2.6, 1.3);
  frag = vec4(pow(aces(lin * uExposure * vig), vec3(1.0 / 2.2)), 1.0);
}`;

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
${COMMON_FS}
out vec4 frag;

void main() {
  vec3 dir = normalize(uFwd + uRight * (vNdc.x * uTan.x) + uUp * (vNdc.y * uTan.y));
  float y = dir.y;
  vec3 c = skyRadiance(vec3(dir.x, max(y, 0.0), dir.z));

  // Sun: a hot disc and a tight glow. The filmic curve does the bloom-free
  // glare; the disc is simply far above white.
  float cosSun = dot(dir, uSun);
  c += uSunCol * (pow(max(cosSun, 0.0), 1600.0) * 12.0 + pow(max(cosSun, 0.0), 60.0) * 0.12);

  // Thin high cloud: noise on the sky projected to a plane, drifting slowly.
  // Lit by the sun, so it is brighter than the blue behind it.
  if (y > 0.02) {
    vec2 uv = dir.xz / (y + 0.15) * 1.6 + vec2(uTime * 0.004, uTime * 0.0015);
    float n = noise(uv) * 0.5 + noise(uv * 2.1 + 3.7) * 0.3 + noise(uv * 4.3 + 9.1) * 0.2;
    float cloud = smoothstep(0.52, 0.78, n) * smoothstep(0.02, 0.22, y) * 0.6;
    vec3 cloudCol = mix(uHorizon, uSunCol * 0.42, 0.55) * (0.85 + 0.25 * pow(max(cosSun, 0.0), 3.0));
    c = mix(c, cloudCol, cloud);
  }

  // Below the horizon there is nothing but the ground plane, which is fogged
  // to the same colour; a slightly darker band there hides any seam.
  if (y < 0.0) c = mix(uHorizon, uHorizon * 0.96, clamp(-y * 8.0, 0.0, 1.0));
  frag = finish(c);
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
  vec3 asphalt = vec3(0.27, 0.275, 0.285);
  asphalt *= 0.86 + 0.24 * wear;
  asphalt *= 0.95 + 0.10 * smoothstep(0.55, 0.62, seam);
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
  float rough = 0.78;
  if (edge > -0.5) {
    float grassN = fbm(p * 0.35) * 0.6 + noise(p * 6.0) * 0.4;
    vec3 grass = mix(vec3(0.27, 0.36, 0.17), vec3(0.42, 0.50, 0.22), grassN);
    grass *= 0.9 + 0.2 * noise(p * 0.05);
    vec3 kerb = vec3(0.60, 0.60, 0.57) * (0.9 + 0.2 * noise(p * 9.0));
    albedo = mix(asphalt, kerb, smoothstep(-0.05, 0.12, edge));
    float grassAmt = smoothstep(0.45, 0.9, edge + 0.4 * (grassN - 0.5));
    albedo = mix(albedo, grass, grassAmt);
    rough = mix(rough, 0.95, grassAmt);
  }

  // ---- lighting: flat plane, so the normal is up ----
  // Worn asphalt goes glossier where the aggregate is polished, which is why
  // a lot glares when you look toward the sun and not otherwise; GGX at this
  // roughness does exactly that at grazing angles.
  rough -= 0.10 * smoothstep(0.55, 0.62, seam) + 0.06 * wear;
  vec3 n = vec3(0.0, 1.0, 0.0);
  float sh = shadowAt(vWorld, n);
  float ao = contactAO(p);
  vec3 c = shade(toLinear(albedo), n, vWorld, sh * ao, rough, 0.0, 0.0);
  c *= ao;
  frag = finish(applyFog(c, vWorld));
}`;

// Rubber on the road. A strip of dark quads laid where a tyre was sliding,
// blended over whatever surface is there, fading with distance so the far
// end of a long lap does not turn into a black smear.
const SKID_VS = `#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aAlpha;
uniform mat4 uViewProj;
out float vA;
out vec3 vWorld;
void main() {
  vA = aAlpha;
  vWorld = aPos;
  gl_Position = uViewProj * vec4(aPos, 1.0);
}`;

const SKID_FS = `#version 300 es
precision highp float;
in float vA;
in vec3 vWorld;
uniform vec3 uEye;
out vec4 frag;
void main() {
  float d = length(vWorld - uEye);
  float fade = 1.0 - smoothstep(50.0, 140.0, d);
  frag = vec4(0.03, 0.03, 0.035, vA * fade);
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
  // Laid rubber is smoother than the lot around it.
  float rough = 0.70 - 0.08 * smoothstep(0.1, 0.9, mid);

  // Start/finish: a painted chequer band. Autocross also gets one at the end.
  float atStart = 1.0 - smoothstep(0.0, 1.2, vS);
  float atEnd = uClosed > 0.5 ? 0.0 : 1.0 - smoothstep(0.0, 1.2, uLength - vS);
  float band = max(atStart, atEnd);
  if (band > 0.0) {
    float sq = mod(floor(vSide * 6.0) + floor(vS / 0.6), 2.0);
    c = mix(c, mix(vec3(0.08), vec3(0.88), sq), band);
    rough = mix(rough, 0.45, band);
  }

  vec3 n = vec3(0.0, 1.0, 0.0);
  float sh = shadowAt(vWorld, n);
  float ao = contactAO(p);
  vec3 lit = shade(toLinear(c), n, vWorld, sh * ao, rough, 0.0, 0.0);
  lit *= ao;
  frag = finish(applyFog(lit, vWorld));
}`;

const PROP_VS = `#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
layout(location = 3) in vec3 iOffset;
layout(location = 4) in float iDown;
layout(location = 5) in float iTint;
layout(location = 6) in float iDir;
uniform mat4 uViewProj;
out vec3 vColor;
out vec3 vNormal;
out vec3 vWorld;
void main() {
  vec3 p = aPos;
  vec3 n = aNormal;
  if (iDown > 0.001) {
    // Knocked over: tip about the base edge on the far side from the strike,
    // by up to 90 degrees, eased so it lands rather than hinges. Used to be
    // an instant flop about world X whichever way the car hit it, so every
    // downed cone on the course lay the same way.
    float a = 1.5707963 * (1.0 - (1.0 - iDown) * (1.0 - iDown));
    float ca = cos(a), sa = sin(a);
    // The fall direction in world xz (world z is -y of the course frame).
    vec3 f = vec3(cos(iDir), 0.0, -sin(iDir));
    vec3 k = vec3(f.z, 0.0, -f.x);          // horizontal axis, perpendicular
    vec3 pivot = f * 0.155;                 // the base edge it tips over
    vec3 q = p - pivot;
    p = q * ca + cross(k, q) * sa + k * dot(k, q) * (1.0 - ca) + pivot;
    n = n * ca + cross(k, n) * sa + k * dot(k, n) * (1.0 - ca);
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
  // A degenerate triangle (CAD slivers are routine) has a zero normal, and
  // normalize(0) is NaN: a black or undefined pixel. Fall back to up.
  float nl = length(vNormal);
  vec3 n = nl > 1e-6 ? vNormal / nl : vec3(0.0, 1.0, 0.0);
  float sh = shadowAt(vWorld, n);
  // The base of a cone sits in its own contact shadow.
  float ao = 0.72 + 0.28 * clamp(vWorld.y / 0.35, 0.0, 1.0);
  // Satin PVC: a broad soft highlight, no mirror.
  vec3 c = shade(toLinear(vColor), n, vWorld, sh, 0.55, 0.0, 0.0) * ao;
  frag = finish(applyFog(c, vWorld));
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
uniform vec3 uMaterial;   // roughness, metalness, clearcoat
uniform float uAlpha;     // 1 for the car; less for a ghost, which is blended
${COMMON_FS}
out vec4 frag;
void main() {
  // Two-sided: inside a cockpit you are looking at the back of half the
  // bodywork, and an unlit black shell there ruins the whole effect.
  // A degenerate triangle (CAD slivers are routine) has a zero normal, and
  // normalize(0) is NaN: a black or undefined pixel. Fall back to up.
  float nl = length(vNormal);
  vec3 n = nl > 1e-6 ? vNormal / nl : vec3(0.0, 1.0, 0.0);
  if (!gl_FrontFacing) n = -n;

  vec3 base = toLinear(mix(vColor, uOverride.rgb, uOverride.a));
  float sh = shadowAt(vWorld, n);
  vec3 c = shade(base, n, vWorld, sh, uMaterial.x, uMaterial.y, uMaterial.z);
  frag = finish(applyFog(c, vWorld));
  frag.a = uAlpha;
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
layout(location = 6) in float iDir;
uniform mat4 uViewProj;
void main() {
  vec3 p = aPos;
  vec3 n = vec3(0.0, 1.0, 0.0);
  if (iDown > 0.001) {
    // Knocked over: tip about the base edge on the far side from the strike,
    // by up to 90 degrees, eased so it lands rather than hinges. Used to be
    // an instant flop about world X whichever way the car hit it, so every
    // downed cone on the course lay the same way.
    float a = 1.5707963 * (1.0 - (1.0 - iDown) * (1.0 - iDown));
    float ca = cos(a), sa = sin(a);
    // The fall direction in world xz (world z is -y of the course frame).
    vec3 f = vec3(cos(iDir), 0.0, -sin(iDir));
    vec3 k = vec3(f.z, 0.0, -f.x);          // horizontal axis, perpendicular
    vec3 pivot = f * 0.155;                 // the base edge it tips over
    vec3 q = p - pivot;
    p = q * ca + cross(k, q) * sa + k * dot(k, q) * (1.0 - ca) + pivot;
    n = n * ca + cross(k, n) * sa + k * dot(k, n) * (1.0 - ca);
  }
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
  // Square base plate. Its top face sits 24 mm up: the course ribbon is at
  // 12 mm, and a plate at the same height z-fought it at every cone on the
  // ribbon's edge. A real cone base is about that thick anyway.
  const B = 0.155;
  const PY = 0.024;
  const plate = [
    // Wound counter-clockwise seen from above, or culling removes it.
    [-B, PY, -B], [B, PY, B], [B, PY, -B],
    [-B, PY, -B], [-B, PY, B], [B, PY, B],
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
    // A GPU reset kills the context. Everything here (programs, VAOs, shadow
    // maps, the track ribbon) would need rebuilding; a reload is the honest
    // way to get all of it back, and the rig's watchdog holds the car meanwhile.
    this.lost = false;
    canvas.addEventListener("webglcontextlost", (e) => {
      e.preventDefault();
      this.lost = true;
      console.error("WebGL context lost; reloading when it is restored");
    });
    canvas.addEventListener("webglcontextrestored", () => location.reload());

    gl.enable(gl.DEPTH_TEST);
    gl.cullFace(gl.BACK);
    // Culling is switched on only around the solid props. The sky quad, the
    // ground quad and the course ribbon are all single-sided sheets whose
    // winding depends on which side you view them from, so culling them
    // globally makes the world disappear from half the approaches.

    this.progSky = program(gl, SKY_VS, SKY_FS);
    this.progGround = program(gl, GROUND_VS, GROUND_FS);
    this.progRibbon = program(gl, RIBBON_VS, RIBBON_FS);
    this.progSkid = program(gl, SKID_VS, SKID_FS);
    this.progProp = program(gl, PROP_VS, PROP_FS);
    this.progCar = program(gl, CAR_VS, CAR_FS);
    this.progScreen = program(gl, SCREEN_VS, SCREEN_FS);
    this.progDepthCar = program(gl, DEPTH_CAR_VS, DEPTH_FS);
    this.progDepthProp = program(gl, DEPTH_PROP_VS, DEPTH_FS);

    // Uniform locations, looked up once: getUniformLocation every frame is
    // both slow and a string allocation per call.
    this.u = {};
    for (const [name, prog] of Object.entries({
      sky: this.progSky, ground: this.progGround, ribbon: this.progRibbon,
      prop: this.progProp, car: this.progCar, depthCar: this.progDepthCar, depthProp: this.progDepthProp,
      skid: this.progSkid,
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
    this.skid = this.makeSkidBuffer();

    this.cone = this.makeInstanced(coneMesh(), 4096);
    this.post = this.makeInstanced(boxMesh(0.12, 2.1, 0.12, [0.85, 0.85, 0.88]), 8);
    this.pole = this.makeInstanced(poleMesh(), 64);

    this.shadow = [this.makeShadowMap(SHADOW_SIZE), this.makeShadowMap(SHADOW_SIZE)];
    /** Scene exposure: linear radiance is scaled by this before the filmic curve. */
    this.exposure = 0.45;

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
      dashCase: this.makeMesh(carMeshes.dashCase),
    };

    // The dash screen: a textured quad, and the canvas the HUD draws into.
    this.dashScreen = this.makeScreenQuad(GEO.dashHalfWidth, GEO.dashHalfHeight);
    this.dashPanel = this.makePanelTexture(DASH_TEX_W, DASH_TEX_H);
    this.dashModel = mat4();

    this.viewProj = mat4();
    this.proj = mat4();
    this.view = mat4();
    this.chassis = mat4();
    this.camFrame = mat4();
    this.model = mat4();
    this.lightView = mat4();
    this.lightProj = mat4();
    this.lightViewProj = [mat4(), mat4()];
    this.shadowMat = [mat4(), mat4()];
    this.shadowTexel = [0, 0];
    this.biasMat = new Float32Array([0.5, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0.5, 0, 0.5, 0.5, 0.5, 1]);
    this._a = mat4(); this._b = mat4();
    this._t = [mat4(), mat4(), mat4(), mat4(), mat4()];
    this._hubXZ = new Float32Array(8);
    this._wheelMirrored = [false, false, false, false];
    this._wheelMats = [mat4(), mat4(), mat4(), mat4()];
    // A second car -- the replay ghost -- gets its own frame and wheel set,
    // because `placeWheels` writes the live car's into shared scratch.
    this._ghostChassis = mat4();
    this._ghostAxles = mat4();
    this.axleFrame = mat4();
    this._ghostWheelMats = [mat4(), mat4(), mat4(), mat4()];
    this._ghostMirrored = [false, false, false, false];
    this._ghostOn = false;
    this._ghostOv = [0.24, 0.62, 0.95, 0.8];
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

  /**
   * A flat quad in the XY plane with UVs, for the dash screen.
   *
   * Its own tiny VAO rather than a `makeMesh`: the car meshes carry a normal
   * and a vertex colour, and this needs neither -- it needs a UV, at the
   * attribute slot the car's normal uses.
   */
  makeScreenQuad(hw, hh) {
    const gl = this.gl;
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const pos = new Float32Array([
      -hw, -hh, 0, hw, -hh, 0, hw, hh, 0,
      -hw, -hh, 0, hw, hh, 0, -hw, hh, 0,
    ]);
    // Both axes flipped, for two different reasons.
    //
    // v, because a canvas's origin is top-left and GL's is bottom-left.
    //
    // u, because the steering column's frame has local +x pointing to the
    // driver's LEFT: the basis maps it onto the car's -Z. The wheel does not
    // care -- it is very nearly symmetric -- but a panel full of text does,
    // and without this the dash renders as a mirror image.
    const uv = new Float32Array([
      1, 1, 0, 1, 0, 0,
      1, 1, 0, 0, 1, 0,
    ]);
    const attach = (data, loc, size) => {
      const buf = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 0, 0);
      return buf;
    };
    const buffers = { position: attach(pos, A_POS, 3), uv: attach(uv, A_NORMAL, 2) };
    gl.bindVertexArray(null);
    return { vao, count: 6, buffers };
  }

  /** An offscreen canvas and the texture it is uploaded to each frame. */
  makePanelTexture(w, h) {
    const gl = this.gl;
    const canvas = document.createElement("canvas");
    canvas.width = w;
    canvas.height = h;
    const ctx = canvas.getContext("2d");
    const tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, tex);
    // No mips: the panel is redrawn every frame and regenerating a chain each
    // time costs more than the aliasing it would save at this size.
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.bindTexture(gl.TEXTURE_2D, null);
    return { canvas, ctx, tex, w, h, dirty: true };
  }

  /**
   * Hand the dash a fresh face.
   *
   * Called by the game with the same `drawDash` the overlay uses, so the panel
   * on the car and the panel on the screen can never disagree -- there is one
   * renderer and one layout.
   */
  updateDashPanel(draw) {
    const p = this.dashPanel;
    if (!p?.ctx) return;
    p.ctx.clearRect(0, 0, p.w, p.h);
    draw(p.ctx, 0, 0, p.w, p.h);
    p.dirty = true;
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

    // Interleaved per-instance data: offset(3), down(1), tint(1), dir(1)
    const stride = INST_FLOATS * 4;
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
    bindInst(I_DIR, 1, 20);

    gl.bindVertexArray(null);
    return {
      vao, instBuf, count: mesh.count, maxInstances,
      data: new Float32Array(maxInstances * INST_FLOATS),
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
    this.clearSkids();
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
   *   ghost  {x, y, psi, rollRad, pitchRad, steerRad, spinFront, spinRear,
   *           color, tint} or null -- a second car, for replays
   *   heaveM, fovBoost
   */
  draw(s) {
    if (this.lost) return;
    const gl = this.gl;
    const aspect = this.resize();
    const T = this._t;
    const cam = s.car;
    this.stats.drawCalls = 0;
    this.stats.triangles = 0;
    this.time = performance.now() / 1000;

    // ---- chassis frame: everything bolted to the car rides on this ----
    //
    // Roll and pitch are about the CG, not about a point on the ground: the
    // body sits on its springs and the wheels stay on the road. Rotating
    // about the ground origin put the outside tyres into the asphalt and
    // floated the inside ones at every corner, and hid the one thing a
    // rolling body shows -- the travel between the wheel and the arch.
    const h = cam.cgHeight ?? 0;
    this.chain(this.chassis, [
      translation(T[0], cam.x, h, -cam.y),
      rotY(T[1], cam.psi),
      rotZ(T[2], cam.pitchRad),
      rotX(T[3], cam.rollRad),
      translation(T[4], 0, -h, 0),
    ]);
    // The unsprung frame the wheels hang off: position and heading only.
    this.chain(this.axleFrame, [
      translation(T[0], cam.x, 0, -cam.y),
      rotY(T[1], cam.psi),
    ]);

    // The cockpit and nose cameras are RIGIDLY bolted to that frame. That is
    // the whole point: the dash and wheel must not move relative to the
    // driver's head, so the roll you see is the world rolling, not the car.
    // Chase damps roll and pitch, because a chase camera that rolls with the
    // car is unwatchable.
    if (s.view.rigid) {
      this.camFrame.set(this.chassis);
    } else {
      // A chase camera has its own heading (`view.yaw`, a damped follower
      // in main.js), so the car can yaw inside the frame: that is how slip
      // angle is seen from behind. Welded to `cam.psi` it never could.
      this.chain(this.camFrame, [
        translation(T[0], cam.x, 0, -cam.y),
        rotY(T[1], s.view.yaw ?? cam.psi),
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
      // The driver's head is not bolted to the chassis: it leans outboard
      // (`lateral`, +z is the driver's right) and the eyes lead into the
      // corner (`yawOffset`, positive looks left). Both were computed every
      // frame and dropped here, so only the fore-aft slide ever showed.
      eye = transformPoint(this.camFrame,
        [s.view.ahead, s.view.height + s.heaveM, s.view.lateral || 0]);
      const pitched = this.chain(T[4], [
        this.camFrame,
        rotY(T[0], s.view.yawOffset || 0),
        rotZ(T[1], s.view.pitchOffset || 0),
      ]);
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
    this.placeGhost(s);
    if (s.skid) this.addSkids(s.skid);

    // ---- shadow pass ----
    this.drawShadowMap(s, cam);

    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    const clear = displayEncode(SKY.horizon, this.exposure);
    gl.clearColor(clear[0], clear[1], clear[2], 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    // Shadow cascades on units 0 and 1 for every lit program.
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.shadow[0].tex);
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, this.shadow[1].tex);


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
      gl.uniform1f(uc.uAlpha, 1);
      gl.uniform3f(uc.uMaterial, 0.88, 0.0, 0.0);   // matte scenery
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

    // --- rubber on the surface, over the ribbon and the lot ---
    this.drawSkids(eye);

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

    // The ghost AFTER the live car, because it is translucent and has to
    // blend over whatever is behind it. `drawCar` ends with the dash screen,
    // which leaves its texture on unit 0 where the lit programs expect the
    // near shadow cascade, so `drawGhost` rebinds the cascades first.
    this.drawCar(s, eye);
    this.drawGhost(s, eye);

    // --- sky, last: its quad sits at z = 0.9999, so wherever anything was
    // drawn the depth test rejects it before the (expensive) sky shader runs.
    // Drawn first it ran on every pixel the ground and car then covered. ---
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
    this.setCommon(us, eye);
    gl.bindVertexArray(this.quad);
    this.drawArrays(gl.TRIANGLES, 0, 6);
    gl.depthMask(true);

    gl.bindVertexArray(null);
  }

  /** Uniforms every lit program shares. */
  setCommon(u, eye) {
    const gl = this.gl;
    gl.uniform3f(u.uSun, SUN[0], SUN[1], SUN[2]);
    gl.uniform3f(u.uCam, eye[0], eye[1], eye[2]);
    gl.uniform3fv(u.uSunCol, SKY.sunCol);
    gl.uniform3fv(u.uZenith, SKY.zenith);
    gl.uniform3fv(u.uHorizon, SKY.horizon);
    gl.uniform3fv(u.uGround, SKY.ground);
    gl.uniform1f(u.uExposure, this.exposure);
    gl.uniform2f(u.uInvRes, 1 / this.canvas.width, 1 / this.canvas.height);
    gl.uniform1i(u.uShadow0, 0);
    gl.uniform1i(u.uShadow1, 1);
    gl.uniformMatrix4fv(u.uShadowMat0, false, this.shadowMat[0]);
    gl.uniformMatrix4fv(u.uShadowMat1, false, this.shadowMat[1]);
    gl.uniform2f(u.uShadowTexel, this.shadowTexel[0], this.shadowTexel[1]);
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
    this._placeWheelSet(this.axleFrame, s.wheels, s.hubs, this._wheelMats, this._wheelMirrored, this._hubXZ);
  }

  makeSkidBuffer() {
    const gl = this.gl;
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const buf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferData(gl.ARRAY_BUFFER, SKID_MAX * SKID_FLOATS * 4, gl.DYNAMIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 16, 0);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 1, gl.FLOAT, false, 16, 12);
    gl.bindVertexArray(null);
    return {
      vao, buf, seg: new Float32Array(SKID_FLOATS), n: 0, head: 0,
      prev: [null, null, null, null],
    };
  }

  /** Forget every mark: a new course, a fresh surface. */
  clearSkids() {
    this.skid.n = 0;
    this.skid.head = 0;
    this.skid.prev = [null, null, null, null];
  }

  /**
   * Lay rubber under any wheel that is sliding this frame.
   *
   * `intensity` is one number per wheel, 0..1, decided by the game from the
   * slip the tyre is at; here it only sets how dark the mark is. Each wheel
   * contributes one quad from where it was last frame to where it is now,
   * so a mark is continuous at any frame rate. A jump (a respawn, a seek)
   * breaks the strip rather than drawing a streak across the course.
   */
  addSkids(intensity) {
    const gl = this.gl;
    const sk = this.skid;
    for (let i = 0; i < 4; i++) {
      const x = this._hubXZ[i * 2];
      const z = this._hubXZ[i * 2 + 1];
      const prev = sk.prev[i];
      const a = intensity[i] ?? 0;
      if (prev && a > 0.15) {
        const dx = x - prev.x, dz = z - prev.z;
        const len = Math.hypot(dx, dz);
        if (len > 0.015 && len < 2.5) {
          const nx = (-dz / len) * SKID_HALF_W, nz = (dx / len) * SKID_HALF_W;
          // Faint at the onset, never black: rubber on asphalt is a shade
          // darker, not paint.
          const alpha = Math.min(0.38, 0.04 + a * 0.34);
          const v = sk.seg;
          const put = (k, px, pz) => { v[k] = px; v[k + 1] = SKID_Y; v[k + 2] = pz; v[k + 3] = alpha; };
          put(0, prev.x + nx, prev.z + nz); put(4, prev.x - nx, prev.z - nz); put(8, x - nx, z - nz);
          put(12, prev.x + nx, prev.z + nz); put(16, x - nx, z - nz); put(20, x + nx, z + nz);
          gl.bindBuffer(gl.ARRAY_BUFFER, sk.buf);
          gl.bufferSubData(gl.ARRAY_BUFFER, sk.head * SKID_FLOATS * 4, v);
          sk.head = (sk.head + 1) % SKID_MAX;
          sk.n = Math.min(sk.n + 1, SKID_MAX);
        }
      }
      if (prev) { prev.x = x; prev.z = z; } else sk.prev[i] = { x, z };
    }
  }

  drawSkids(eye) {
    const sk = this.skid;
    if (!sk.n) return;
    const gl = this.gl;
    gl.useProgram(this.progSkid);
    const u = this.u.skid;
    gl.uniformMatrix4fv(u.uViewProj, false, this.viewProj);
    gl.uniform3f(u.uEye, eye[0], eye[1], eye[2]);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.depthMask(false);
    // Six millimetres above the ribbon is nothing at 80 m; pull the marks
    // toward the camera in depth so they win the tie.
    gl.enable(gl.POLYGON_OFFSET_FILL);
    gl.polygonOffset(-1.0, -2.0);
    gl.bindVertexArray(sk.vao);
    this.drawArrays(gl.TRIANGLES, 0, sk.n * 6);
    gl.disable(gl.POLYGON_OFFSET_FILL);
    gl.depthMask(true);
    gl.disable(gl.BLEND);
  }

  /**
   * Wheel matrices for one car -- the live one or the ghost -- hung off the
   * given chassis frame.
   *
   * @param hubXZ  where to write the hub contact points, or null for a car
   *               that does not darken the ground under itself
   */
  _placeWheelSet(chassis, w, hubsIn, mats, mirroredOut, hubXZ) {
    const T = this._t;
    const hubs = this.carModel?.hubs ?? this.bodyHubs ?? hubsIn ?? HUBS;
    for (let i = 0; i < 4; i++) {
      const hub = hubs[i];
      // Steer rotates about the kingpin (local Y); spin is about the hub axis
      // (local Z) AFTER the steer. The left pair is mirrored across the wheel
      // plane so the dished rim faces outboard on both sides (see carmesh).
      const mirrored = hub.z < 0;
      const m = mats[i];
      multiply(this._a, chassis, translation(T[0], hub.x, hub.y, hub.z));
      multiply(this._b, this._a, rotY(T[1], hub.front ? w.steerRad : 0));
      multiply(this._a, this._b, rotZ(T[2], -(hub.front ? w.spinFront : w.spinRear)));
      if (mirrored) multiply(m, this._a, scale(T[3], 1, 1, -1));
      else m.set(this._a);
      // A reflection reverses the winding, so gl_FrontFacing inverts and the
      // two-sided shader would flip these normals inward; drawCar swaps the
      // front-face rule for the mirrored pair.
      mirroredOut[i] = mirrored;
      if (hubXZ) {
        hubXZ[i * 2] = m[12];
        hubXZ[i * 2 + 1] = m[14];
      }
    }
  }

  /**
   * The replay ghost's frame and wheels, from the pose `main.js` sampled out
   * of the other run's log. Nothing else about it is different: same body,
   * same tyres, tinted so the two cars can be told apart.
   */
  placeGhost(s) {
    const g = s.ghost;
    this._ghostOn = !!g;
    if (!g) return;
    const T = this._t;
    const h = g.cgHeight ?? 0;
    this.chain(this._ghostChassis, [
      translation(T[0], g.x, h, -g.y),
      rotY(T[1], g.psi),
      rotZ(T[2], g.pitchRad || 0),
      rotX(T[3], g.rollRad || 0),
      translation(T[4], 0, -h, 0),
    ]);
    this.chain(this._ghostAxles, [
      translation(T[0], g.x, 0, -g.y),
      rotY(T[1], g.psi),
    ]);
    this._placeWheelSet(this._ghostAxles,
      { steerRad: g.steerRad || 0, spinFront: g.spinFront || 0, spinRear: g.spinRear || 0 },
      s.hubs, this._ghostWheelMats, this._ghostMirrored, null);
    const c = g.color;
    if (c) { this._ghostOv[0] = c[0]; this._ghostOv[1] = c[1]; this._ghostOv[2] = c[2]; }
    this._ghostOv[3] = g.tint ?? 0.8;
  }

  /**
   * Body and wheels of the ghost, tinted and translucent. No dash, no
   * steering wheel.
   *
   * Translucent, and drawn even when it overlaps the live car. It used to be
   * hidden inside a car's length, on the reasoning that two cars in one
   * place is z-fighting -- but two laps by the same driver on the same
   * course ARE in one place for most of the lap, and a ghost that vanishes
   * whenever it is close and sits behind the chase camera whenever it is
   * slower reads as no ghost at all. Blended at half strength it shows
   * through the live car instead, which is what a ghost is for.
   */
  drawGhost(s, eye) {
    if (!this._ghostOn) return;
    const gl = this.gl;
    const uc = this.u.car;
    gl.useProgram(this.progCar);
    // The dash screen left its texture on unit 0; the cascades go back.
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.shadow[0].tex);
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, this.shadow[1].tex);
    this.setCommon(uc, eye);
    gl.uniformMatrix4fv(uc.uViewProj, false, this.viewProj);
    gl.uniform1f(uc.uAlpha, GHOST_ALPHA);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    // Depth is tested, not written: the ghost's own back faces would
    // otherwise punch holes in its front ones, and the sky quad's depth
    // trick (drawn last, rejected where anything was written) must not see
    // a translucent car as solid.
    gl.depthMask(false);
    const ov = this._ghostOv;
    gl.uniform4f(uc.uOverride, ov[0], ov[1], ov[2], ov[3]);
    const part = (mesh, model, mat) => {
      gl.uniformMatrix4fv(uc.uModel, false, model);
      gl.uniform3f(uc.uMaterial, mat[0], mat[1], mat[2]);
      gl.bindVertexArray(mesh.vao);
      this.drawArrays(gl.TRIANGLES, 0, mesh.count);
    };
    part(this.car.body, this._ghostChassis, MAT.paint);
    for (let i = 0; i < 4; i++) {
      gl.frontFace(this._ghostMirrored[i] ? gl.CW : gl.CCW);
      part(this.car.tire, this._ghostWheelMats[i], MAT.tyre);
      part(this.car.rim, this._ghostWheelMats[i], MAT.rim);
    }
    gl.frontFace(gl.CCW);
    gl.depthMask(true);
    gl.disable(gl.BLEND);
    gl.uniform1f(uc.uAlpha, 1);
    gl.uniform4f(uc.uOverride, 0, 0, 0, 0);
  }

  /**
   * Render the sun's view of the car and nearby cones into the two depth
   * cascades.
   *
   * Each cascade is an orthographic box centred on the car and looking along
   * the sun. The box is snapped to whole shadow texels in light space, so as
   * the car moves the rasterised silhouette lands on the same texel grid
   * every frame and the shadow edge stops swimming.
   */
  drawShadowMap(s, cam) {
    const gl = this.gl;

    // Fill the instance buffers once for both passes.
    const cones = this.track.conesNear(s.car.x, s.car.y, CONE_DRAW_RANGE);
    this.fillCones(this.cone, cones);
    this.fillPoints(this.post, this.gatePosts);
    this.fillPoints(this.pole, this.poles);
    this.stats.cones = this.cone.n;

    // Light view with the eye at the origin: only the ortho window moves,
    // which is what makes the texel snap possible.
    lookAlong(this.lightView, [0, 0, 0], [-SUN[0], -SUN[1], -SUN[2]], [0, 1, 0]);
    const c = transformPoint(this.lightView, [cam.x, 0.3, -cam.y]);
    for (let i = 0; i < 2; i++) {
      const half = SHADOW_HALF[i];
      const texel = (2 * half) / SHADOW_SIZE;
      this.shadowTexel[i] = texel;
      const sx = Math.round(c[0] / texel) * texel;
      const sy = Math.round(c[1] / texel) * texel;
      const dist = -c[2];
      ortho(this.lightProj, sx - half, sx + half, sy - half, sy + half,
            dist - SHADOW_DEPTH, dist + SHADOW_DEPTH);
      multiply(this.lightViewProj[i], this.lightProj, this.lightView);
      multiply(this.shadowMat[i], this.biasMat, this.lightViewProj[i]);
    }

    gl.colorMask(false, false, false, false);
    gl.enable(gl.POLYGON_OFFSET_FILL);
    gl.polygonOffset(2.0, 4.0);
    for (let i = 0; i < 2; i++) {
      const sm = this.shadow[i];
      if (!sm.ok) continue;
      gl.bindFramebuffer(gl.FRAMEBUFFER, sm.fbo);
      gl.viewport(0, 0, sm.size, sm.size);
      gl.clear(gl.DEPTH_BUFFER_BIT);

      // Car: body and tyres. Rims sit inside the tyre silhouette and the
      // steering wheel is inside the body, so neither adds anything here.
      gl.useProgram(this.progDepthCar);
      const ud = this.u.depthCar;
      gl.uniformMatrix4fv(ud.uViewProj, false, this.lightViewProj[i]);
      gl.uniformMatrix4fv(ud.uModel, false, this.chassis);
      gl.bindVertexArray(this.car.body.vao);
      this.drawArrays(gl.TRIANGLES, 0, this.car.body.count);
      for (let k = 0; k < 4; k++) {
        gl.uniformMatrix4fv(ud.uModel, false, this._wheelMats[k]);
        gl.bindVertexArray(this.car.tire.vao);
        this.drawArrays(gl.TRIANGLES, 0, this.car.tire.count);
      }
      // The ghost throws a shadow too, or it reads as a hologram.
      if (this._ghostOn) {
        gl.uniformMatrix4fv(ud.uModel, false, this._ghostChassis);
        gl.bindVertexArray(this.car.body.vao);
        this.drawArrays(gl.TRIANGLES, 0, this.car.body.count);
        for (let k = 0; k < 4; k++) {
          gl.uniformMatrix4fv(ud.uModel, false, this._ghostWheelMats[k]);
          gl.bindVertexArray(this.car.tire.vao);
          this.drawArrays(gl.TRIANGLES, 0, this.car.tire.count);
        }
      }

      // Cones, posts and poles. The instance buffer holds everything within
      // draw range; the ortho box clips the rest away for free.
      gl.useProgram(this.progDepthProp);
      gl.uniformMatrix4fv(this.u.depthProp.uViewProj, false, this.lightViewProj[i]);
      this.drawInstanced(this.cone);
      this.drawInstanced(this.post);
      this.drawInstanced(this.pole);
    }
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
    gl.uniform1f(uc.uAlpha, 1);

    // Material per part: roughness, metalness, clearcoat.
    const part = (mesh, model, ov, mat) => {
      gl.uniformMatrix4fv(uc.uModel, false, model);
      if (ov) gl.uniform4f(uc.uOverride, ov[0], ov[1], ov[2], ov[3]);
      else gl.uniform4f(uc.uOverride, 0, 0, 0, 0);
      gl.uniform3f(uc.uMaterial, mat[0], mat[1], mat[2]);
      gl.bindVertexArray(mesh.vao);
      this.drawArrays(gl.TRIANGLES, 0, mesh.count);
    };

    part(this.car.body, this.chassis, null, MAT.paint);

    const w = s.wheels;
    for (let i = 0; i < 4; i++) {
      gl.frontFace(this._wheelMirrored[i] ? gl.CW : gl.CCW);
      part(this.car.tire, this._wheelMats[i], null, MAT.tyre);
      // Fade the gold spokes toward the tyre as the wheel speeds up. Five
      // spokes at 20 rev/s would otherwise strobe into a stationary-looking
      // mess at 60 Hz; this reads as motion blur instead.
      part(this.car.rim, this._wheelMats[i], this._rimOverride(w.rimFade), MAT.rim);
    }
    gl.frontFace(gl.CCW);

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

    // The dash, FIRST, and without the wheel's rotation.
    //
    // It is bolted to the column shroud, not to the wheel, so it shares the
    // column's frame and its tilt but not its spin -- which is the whole
    // reason a grip sweeps across it at 90 degrees of lock instead of the
    // dash swinging with the driver's hands. Drawn before the wheel so the
    // depth buffer does the occluding; nothing here needs to know where the
    // grips are.
    const dc = GEO.dashCentre;
    this.chain(this.model, [
      this.chassis,
      translation(T[0], sc[0], sc[1], sc[2]),
      basis,
      translation(T[1], dc[0], dc[1], dc[2]),
      rotX(T[2], GEO.dashTiltRad),
    ]);
    this.dashModel.set(this.model);
    part(this.car.dashCase, this.model, null, MAT.dash);

    this.chain(this.model, [
      this.chassis,
      translation(T[0], sc[0], sc[1], sc[2]),
      basis,
      rotZ(T[1], -w.steerRad * (w.steerRatio ?? GEO.steeringRatio)),
    ]);
    part(this.car.steeringWheel, this.model, null, MAT.wheel);

    // The screen last of the car's parts: it is the only thing drawn with a
    // different program, and switching back and forth per object costs more
    // than doing it once.
    this.drawDashScreen();
  }

  /**
   * The lit face of the dash.
   *
   * Its own program and its own VAO, so it is one state change and six
   * vertices. The texture is only uploaded when the panel has actually been
   * redrawn -- `updateDashPanel` sets the flag -- because `texImage2D` from a
   * canvas is a GPU copy of a megabyte and doing it on a frame where nothing
   * changed is pure waste.
   */
  drawDashScreen() {
    const gl = this.gl;
    const p = this.dashPanel;
    const q = this.dashScreen;
    if (!p || !q) return;

    gl.useProgram(this.progScreen);
    const u = this._screenU ?? (this._screenU = {
      uViewProj: gl.getUniformLocation(this.progScreen, "uViewProj"),
      uModel: gl.getUniformLocation(this.progScreen, "uModel"),
      uPanel: gl.getUniformLocation(this.progScreen, "uPanel"),
      uNits: gl.getUniformLocation(this.progScreen, "uNits"),
      uInvRes: gl.getUniformLocation(this.progScreen, "uInvRes"),
      uExposure: gl.getUniformLocation(this.progScreen, "uExposure"),
    });

    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, p.tex);
    if (p.dirty) {
      gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, p.canvas);
      p.dirty = false;
    }
    gl.uniform1i(u.uPanel, 0);
    gl.uniformMatrix4fv(u.uViewProj, false, this.viewProj);
    gl.uniformMatrix4fv(u.uModel, false, this.dashModel);
    gl.uniform1f(u.uNits, DASH_NITS);
    gl.uniform2f(u.uInvRes, 1 / this.canvas.width, 1 / this.canvas.height);
    gl.uniform1f(u.uExposure, this.exposure);

    gl.bindVertexArray(q.vao);
    gl.drawArrays(gl.TRIANGLES, 0, q.count);
    gl.bindVertexArray(null);
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
    const now = performance.now();
    for (let i = 0; i < n; i++) {
      const c = list[i];
      // How far through its tumble a struck cone is. A cone with no strike
      // time (a replay's, or an older record) is simply down.
      const prog = !c.down ? 0
        : c.downAt == null ? 1
        : Math.min(1, (now - c.downAt) / CONE_TUMBLE_MS);
      const o = i * INST_FLOATS;
      d[o + 0] = c.x;
      d[o + 1] = 0;
      d[o + 2] = -c.y;
      d[o + 3] = prog;
      d[o + 4] = 1 - 0.28 * prog;
      d[o + 5] = c.downDir ?? 0;
    }
    this.uploadInstances(mesh, n);
  }

  fillPoints(mesh, list) {
    const n = Math.min(list.length, mesh.maxInstances);
    const d = mesh.data;
    for (let i = 0; i < n; i++) {
      const o = i * INST_FLOATS;
      d[o + 0] = list[i].x;
      d[o + 1] = 0;
      d[o + 2] = -list[i].y;
      d[o + 3] = 0;
      d[o + 4] = 1;
      d[o + 5] = 0;
    }
    this.uploadInstances(mesh, n);
  }

  uploadInstances(mesh, n) {
    const gl = this.gl;
    mesh.n = n;
    if (n === 0) return;
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.instBuf);
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, mesh.data, 0, n * INST_FLOATS);
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
