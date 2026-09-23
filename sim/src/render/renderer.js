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
  mat4, perspective, lookAlong, multiply, normalize, identity, ortho, axisAngle,
  translation, rotX, rotY, rotZ, scale, transformDir, transformPoint,
  basisFromAxes, invertRigid,
} from "./math.js";
import { buildCarMeshes, GEO, HUBS } from "./carmesh.js";
import { SDM26, CAD_EYE_AHEAD_OF_CG_M } from "../vehicle/params.js";
import { SuspensionRig } from "./suspensionRig.js";
import { buildVenueMesh } from "./venuemesh.js";
import { buildEnvironmentMesh } from "./envmesh.js";
import { PRESETS } from "./quality.js";

// Far enough that a cone arrives out of the fog rather than popping in on
// the endurance straights; the instance buffer holds 4096, plenty.
const CONE_DRAW_RANGE = 280; // m
/** How solid the replay ghost is drawn. */
const GHOST_ALPHA = 0.55;
/** Skid marks kept on the surface: segments in a ring, oldest overwritten. */
const SKID_MAX = 8000;
/** Floats per segment: two triangles of (x, y, z, alpha, side). */
const SKID_FLOATS = 6 * 5;
/** Half-width of a mark, metres -- a bit under the slick's 7 in, since only
 *  the loaded shoulder really scrubs. */
const SKID_HALF_W = 0.07;
/** Height of the marks above the deck: over the ribbon, under a cone's plate. */
const SKID_Y = 0.018;
// Two cascades: a tight box for the car's own shadow and a wide one so the
// cones down the course carry shadows instead of popping into them.
const SHADOW_HALF = [14, 64]; // m, half-extent of each cascade
const SHADOW_DEPTH = 90;      // m, half depth range of each cascade along the sun

// Attribute locations are fixed so one VAO can be drawn by the lit program
// and by the depth-only program alike.
const A_POS = 0, A_NORMAL = 1, A_COLOR = 2, I_OFFSET = 3, I_DOWN = 4, I_CONE = 5, I_DIR = 6;
/** Floats per prop instance: offset(3), down(1), cone(1), dir(1). The cone
 *  flag tells PROP_FS which props get the reflective collar; it used to be a
 *  tint that darkened a knocked cone by 28%, which a falling cone does not do. */
const INST_FLOATS = 6;
/** How long a struck cone takes to land, ms. */
const CONE_TUMBLE_MS = 320;
/** The cone's white collar, metres up an 18 in (0.46 m) cone: 52-78% of the
 *  height, drawn by PROP_FS from the vertex height so it is an edge. */
const CONE_BAND_LO = 0.24, CONE_BAND_HI = 0.36;

/** The dash panel's texture, at the DISPLAY's own 108:65 -- not the case's.
 *  Sized so the smallest type is still a couple of pixels tall from the
 *  driver's seat, which is what decides it, not the panel's physical size. */
const DASH_TEX_W = 768, DASH_TEX_H = 462;

/** How bright the panel is, in the scene's linear units. A dash in daylight
 *  is about as bright as sunlit white paper -- enough to read against the
 *  sky, not so much that it glows. */
const DASH_NITS = 1.9;

// Sun: mid-afternoon, from the south-EAST (GL +z is world south, so a
// positive z component is a sun on the south side and positive x is east),
// high enough that a 15 m grandstand throws a shadow without the cones
// throwing 3 m ones.
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
  // Horizon: the long path scatters everything, so it is brighter and less
  // saturated than the zenith, tinted by the (already warmed) sun. The haze
  // term is held down (0.55) and the zenith's share up (0.70): with the haze
  // at full strength the horizon came out nearly neutral grey, and since it
  // is also the fog colour and a third of the frame, the whole image went
  // milky. A clear sky at this elevation is bluer than that.
  const horizon = [0, 1, 2].map((i) => zenith[i] * 0.70 + (0.28 + 0.20 * T[i]) * 0.55 * (0.30 + 0.80 * day));
  const skyAvg = [0, 1, 2].map((i) => zenith[i] * 0.45 + horizon[i] * 0.55);
  // Linear, the lot: exactly `toLinear` of the asphalt albedo GROUND_FS
  // authors (0.27, 0.275, 0.285 in display space), so the bounce that lights
  // the underside of the car is the lot it is sitting on and not a lighter one.
  const groundAlbedo = [0.056, 0.058, 0.063];
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
  // The driver: a Nomex suit and belts are cloth, about as matte as it gets.
  suit: [0.92, 0.0, 0.0],
  // The helmet is a lacquered shell, the one part of the driver that gleams.
  helmet: [0.28, 0.0, 0.9],
};

/** How hard the display-space S-curve in `finish` pulls at contrast.
 *  Shared with the JS clear-colour encode below so the two agree. */
const GRADE_CONTRAST = 0.25;

/** Linear radiance -> display encoding, for the clear colour only. Mirrors
 *  `finish` in the shaders: exposure, ACES, sRGB, then the same S-curve. */
function displayEncode(c, exposure) {
  return c.map((v) => {
    const x = v * exposure;
    const t = Math.min(1, Math.max(0, (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14)));
    const g = Math.pow(t, 1 / 2.2);
    return g + GRADE_CONTRAST * (g * g * (3 - 2 * g) - g);
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
// The fragment default for integers is mediump; the lattice hash below mixes
// 32-bit words and needs all of them.
precision highp int;
uniform vec3 uSun;
uniform vec3 uCam;
uniform float uTime;      // seconds; drifts the cloud layer
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

// Lattice hash by integer bit mixing (pcg2d). The usual fract(sin(dot()))
// hash feeds sin() an argument of ~1e7 once the lattice coordinate is a
// world position at the far end of the MIS lot (p * 64 at 1300 m), where a
// float32 has no fractional bits left, and the noise there collapsed into
// axis-aligned streaks. Integers do not lose precision with distance.
float hash(vec2 p) {
  uvec2 v = uvec2(ivec2(p)) * uvec2(1664525u, 1013904223u);
  v.x += v.y * 1664525u; v.y += v.x * 1664525u;
  v ^= v >> 16u;
  v.x += v.y * 1664525u; v.y += v.x * 1664525u;
  v ^= v >> 16u;
  return float(v.x ^ v.y) * (1.0 / 4294967296.0);
}
float noise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x),
             mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}
float fbm(vec2 p) {
  return noise(p) * 0.5 + noise(p * 2.03) * 0.25 + noise(p * 4.11) * 0.125 + noise(p * 8.3) * 0.0625;
}

// ---- footprint-faded noise, for surfaces --------------------------------
// \`px\` is how many metres of surface one pixel covers (fwidth of the
// world coordinate, taken ONCE per fragment outside any branch, since
// derivatives are undefined inside non-uniform control flow) and \`freq\` is
// the octave's cells per metre, so px * freq is cells per pixel. Past about
// half a cell the octave is above Nyquist and the only thing it can add is
// sparkle, so it is faded to its mean instead of sampled. The old fade was
// by distance, which cannot work: the footprint depends on the grazing
// angle, and from a seat 0.7 m off the deck the 16 mm and 45 mm octaves
// were past Nyquist from 4 m out while the fade still had them at 90%.
// That was the shimmer over most of the lot.
float octaveFade(float px, float freq) { return 1.0 - smoothstep(0.3, 0.8, px * freq); }
float anoise(vec2 p, float freq, float px) {
  float k = octaveFade(px, freq);
  // Skipping the lattice lookups once an octave is faded out is what keeps
  // the far half of the ground cheaper than it was.
  return k > 0.0 ? mix(0.5, noise(p * freq), k) : 0.5;
}
float afbm(vec2 p, float freq, float px) {
#if Q_DETAIL == 0
  // Two octaves, renormalised so the mean and the spread stay put.
  return (anoise(p, freq, px) * 0.5 + anoise(p, freq * 2.03, px) * 0.25) / 0.75;
#else
  return anoise(p, freq, px) * 0.5 + anoise(p, freq * 2.03, px) * 0.25
       + anoise(p, freq * 4.11, px) * 0.125 + anoise(p, freq * 8.3, px) * 0.0625;
#endif
}

// ---- lot relief --------------------------------------------------------
// The seal-coat patching of the lot as a height field, and its normal by
// finite difference. Asphalt is not a plane: it settles and is patched at
// the metre scale, and that gentle undulation is what breaks the sun's glare
// into the mottled sheen a real lot has. On a perfectly flat plane the GGX
// lobe is one uniform smear, which is what read as a plastic sheet. Two
// octaves and a few centimetres of relief, so it stays subtle; the octaves
// fade with footprint like everything else, so far away it is flat again.
float lotHeight(vec2 p, float px) {
  return anoise(p, 0.7, px) * 0.6 + anoise(p, 1.9, px) * 0.4;
}
vec3 lotNormal(vec2 p, float px, float h0) {
#if Q_DETAIL == 0
  return vec3(0.0, 1.0, 0.0);
#else
  float e = max(0.12, px);         // never step less than a pixel: that aliases too
  float hx = lotHeight(p + vec2(e, 0.0), px);
  float hz = lotHeight(p + vec2(0.0, e), px);
  const float AMP = 0.035;         // metres of relief per unit of the field
  return normalize(vec3((h0 - hx) * AMP / e, 1.0, (h0 - hz) * AMP / e));
#endif
}

// ---- sky ---------------------------------------------------------------
// The one sky the sky quad, the ambient term, the fog and the paint's
// reflection all read, so they cannot disagree with each other. The
// gradient alone is what a rough surface sees; \`skyRadiance\` adds the
// cloud layer on top for everything glossy enough to resolve it.
vec3 skyGradient(vec3 d) {
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

// Thin high cloud: noise on the sky projected to a plane, drifting slowly,
// lit by the sun so it is brighter than the blue behind it -- about 1.7x the
// horizon, which is where sunlit cloud sits against a clear sky; at the
// horizon's own radiance it was grey. It lives HERE and not in the sky
// shader so that the clearcoat reflection and the fog see the same sky as
// the backdrop: a paint reflecting a featureless gradient has nothing to
// slide across a curved panel, and read as matte however glossy it was.
vec3 skyRadiance(vec3 d) {
  vec3 c = skyGradient(d);
  if (d.y > 0.02) {
    vec2 uv = d.xz / (d.y + 0.15) * 1.6 + vec2(uTime * 0.004, uTime * 0.0015);
    float n = noise(uv) * 0.5 + noise(uv * 2.1 + 3.7) * 0.3 + noise(uv * 4.3 + 9.1) * 0.2;
    float cloud = smoothstep(0.52, 0.78, n) * smoothstep(0.02, 0.22, d.y) * 0.6;
    float cs = max(dot(d, uSun), 0.0);
    // Paler than the sky behind it (mostly neutral at the horizon's
    // luminance, a touch warm) and brighter, more so toward the sun.
    float hl = dot(uHorizon, vec3(0.30, 0.59, 0.11));
    vec3 cloudCol = mix(uHorizon, vec3(hl) * vec3(1.03, 1.0, 0.95), 0.7)
                  * 1.7 * (0.85 + 0.25 * cs * cs * cs);
    c = mix(c, cloudCol, cloud);
  }
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
  // Constant bias: 0.00004 of the 180 m depth range is 7 mm along the sun.
  // It was 0.00025, which is 45 mm -- enough to lift every cone's shadow
  // clear of its 24 mm base plate and float the car's off its tyres. The
  // normal offset above (1.2-3.2 texels, 16-44 mm in the near cascade) is
  // what actually holds off acne on the casters, and the ground cannot acne
  // at all: it is never drawn into the map, only shadowed by it.
  float z = p.z - 0.00004;
#if Q_SHADOWS < 2
  // Medium: the hardware 2x2 compare alone. Harder-edged, a quarter of the taps.
  return texture(tex, vec3(p.xy, z));
#else
  float r = 1.4 / SHADOW_RES;
  float s = 0.0;
  for (int i = 0; i < 4; i++) s += texture(tex, vec3(p.xy + POISSON[i] * r, z));
  return s * 0.25;
#endif
}

float shadowAt(vec3 world, vec3 n) {
#if Q_SHADOWS == 0
  return 1.0;
#else
  float ndl = max(dot(n, uSun), 0.0);
  float in0, in1;
  float s0 = cascade(uShadow0, uShadowMat0, world, n, uShadowTexel.x, ndl, in0);
  if (in0 >= 1.0) return s0;
  float s1 = cascade(uShadow1, uShadowMat1, world, n, uShadowTexel.y, ndl, in1);
  return mix(mix(1.0, s1, in1), s0, in0);
#endif
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
  // A rough surface's lobe is wider than a cloud, so it sees the gradient
  // and skips the three noise lookups; anything glossy reflects the clouds.
#if Q_DETAIL == 0
  vec3 sky = skyGradient(r);
#else
  vec3 sky = rough < 0.5 ? skyRadiance(r) : skyGradient(r);
#endif
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
  // A mild S-curve in DISPLAY space, pivoting at display 0.5 (linear 0.22,
  // near mid-grey): a little more contrast through the mid-tones with the
  // ends untouched. ACES on its own left the frame grey-on-grey. Applied
  // after the transfer so the pivot is perceptual mid-grey; in linear it
  // would sit at display 0.73 and darken everything below.
  vec3 g = pow(c, vec3(1.0 / 2.2));
  g = mix(g, g * g * (3.0 - 2.0 * g), ${GRADE_CONTRAST});
  return vec4(g, 1.0);
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
  vec3 g = pow(aces(lin * uExposure * vig), vec3(1.0 / 2.2));
  // The same display-space S-curve as \`finish\`, for the same reason the
  // exposure and vignette are here: the panel must sit IN the image.
  g = mix(g, g * g * (3.0 - 2.0 * g), ${GRADE_CONTRAST});
  frag = vec4(g, 1.0);
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
${COMMON_FS}
out vec4 frag;

void main() {
  vec3 dir = normalize(uFwd + uRight * (vNdc.x * uTan.x) + uUp * (vNdc.y * uTan.y));
  float y = dir.y;
  // Gradient, aureole and the cloud layer, all from the shared sky so the
  // backdrop is the same sky the paint reflects and the fog fades to.
  vec3 c = skyRadiance(vec3(dir.x, max(y, 0.0), dir.z));

  // Sun: a hot disc and a tight glow. The filmic curve does the bloom-free
  // glare; the disc is simply far above white. Added over the cloud, which
  // is thin enough that the sun shows through it.
  float cosSun = dot(dir, uSun);
  c += uSunCol * (pow(max(cosSun, 0.0), 1600.0) * 12.0 + pow(max(cosSun, 0.0), 60.0) * 0.12);

  // Below the horizon there is nothing but the ground plane, which is fogged
  // to the same colour; a slightly darker band there hides any seam.
  if (y < 0.0) c = uHorizon;
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
  // Surface metres per pixel, per axis and overall, for the footprint fades
  // and the stripe filters. Once, here, outside every branch: derivatives
  // are undefined inside non-uniform control flow (the grass block below).
  // Floored so the smoothstep edges below can never coincide.
  vec2 dp = max(fwidth(p), vec2(1e-4));
  float px = max(dp.x, dp.y);

  // ---- asphalt: aggregate speckle, coarse patching, a few seal-coat seams --
  // Every octave is faded by its own footprint (see anoise), to its mean, so
  // the average albedo -- and therefore the exposure -- is the same near and
  // far; only the sparkle goes.
#if Q_DETAIL == 0
  // The 64/m octave held at its mean, so the speckle keeps its contrast
  // instead of the coarse octave taking the whole weight.
  float fine = anoise(p, 22.0, px) * 0.55 + 0.5 * 0.45;
#else
  float fine = anoise(p, 22.0, px) * 0.55 + anoise(p, 64.0, px) * 0.45;
#endif
  float wear = afbm(p, 0.09, px);
  float seam = lotHeight(p, px);
  vec3 asphalt = vec3(0.27, 0.275, 0.285);
  // Kept gentle: a lot is one pour, and the coarse patching read as tiles.
  asphalt *= 0.92 + 0.13 * wear;
  asphalt *= 0.97 + 0.06 * smoothstep(0.55, 0.62, seam);
  asphalt += (fine - 0.5) * 0.14;

  // Faded parking-stall lines. Real lot markings, and the main optical-flow
  // cue at speed: 2.75 m bays, 5.5 m deep, in double rows (two bays nose to
  // nose) with a 7 m drive aisle between the rows -- an 18 m module. The
  // stripes run ONE way, across the row, and stop at the aisle. Drawing the
  // 2.75 x 5.5 grid on both axes made the whole lot read as a tiled floor.
  //
  // Antialiased as coverage: each stripe edge is box-filtered over the
  // pixel's footprint along the stripe's own axis, so a 100 mm stripe that
  // is a tenth of a pixel wide draws as a tenth of its paint rather than
  // sparkling in and out. Beyond the point where the 2.75 m pitch itself
  // is a pixel or two the rows moire, so the paint fades out there.
  float ax = abs(fract(p.x / 2.75) - 0.5) * 2.75;            // distance across to the nearest stripe
  float stripe = clamp((0.05 - ax) / dp.x + 0.5, 0.0, 1.0) - clamp((-0.05 - ax) / dp.x + 0.5, 0.0, 1.0);
  float m = mod(p.y, 18.0);                                   // position along the row's depth
  float aisle = smoothstep(5.5 - dp.y, 5.5 + dp.y, m) - smoothstep(12.5 - dp.y, 12.5 + dp.y, m);
  float paint = stripe * (1.0 - aisle) * 0.42 * (1.0 - smoothstep(0.9, 2.0, dp.x));
  paint *= 0.35 + 0.65 * anoise(p, 3.0, px);   // worn and patchy
  asphalt = mix(asphalt, vec3(0.66, 0.65, 0.62), paint);

  // ---- grass beyond the lot, with a concrete kerb along the edge ----------
  vec2 q = abs(p - uLotCentre) - uLotHalf;
  float edge = max(q.x, q.y);              // <0 inside the lot
  vec3 albedo = asphalt;
  float rough = 0.78;
  if (edge > -0.5) {
    float grassN = afbm(p, 0.35, px) * 0.6 + anoise(p, 6.0, px) * 0.4;
    vec3 grass = mix(vec3(0.27, 0.36, 0.17), vec3(0.42, 0.50, 0.22), grassN);
    grass *= 0.9 + 0.2 * anoise(p, 0.05, px);
    vec3 kerb = vec3(0.60, 0.60, 0.57) * (0.9 + 0.2 * anoise(p, 9.0, px));
    albedo = mix(asphalt, kerb, smoothstep(-0.05, 0.12, edge));
    float grassAmt = smoothstep(0.45, 0.9, edge + 0.4 * (grassN - 0.5));
    albedo = mix(albedo, grass, grassAmt);
    rough = mix(rough, 0.95, grassAmt);
  }

  // ---- lighting ----
  // Worn asphalt goes glossier where the aggregate is polished, which is why
  // a lot glares when you look toward the sun and not otherwise; GGX at this
  // roughness does exactly that at grazing angles. The shading normal
  // carries the lot's relief (lotNormal) so that glare is mottled rather
  // than one sheet; the shadow lookup keeps the true plane, since the
  // normal offset there is about the receiver's geometry, not its finish.
  rough -= 0.10 * smoothstep(0.55, 0.62, seam) + 0.06 * wear;
  vec3 up = vec3(0.0, 1.0, 0.0);
  vec3 n = lotNormal(p, px, seam);
  float sh = shadowAt(vWorld, up);
  // Contact darkening once, on the whole result: the sun under the car is
  // already the shadow map's job, and applying the AO to the direct term as
  // well as the total squared it under the tyres.
  float ao = contactAO(p);
  vec3 c = shade(toLinear(albedo), n, vWorld, sh, rough, 0.0, 0.0) * ao;
  frag = finish(applyFog(c, vWorld));
}`;

// Rubber on the road. A mitred strip laid where a tyre was sliding, drawn
// AFTER the lit ground with a multiplicative blend (dst *= 1 - alpha): the
// mark darkens whatever the surface already is, so it is lit, shadowed and
// fogged exactly as the asphalt under it. Blending a flat grey over the top
// put a display-space swatch on the road that ignored the car's shadow and
// stayed the same grey into the fog. Fades with distance so the far end of
// a long lap does not turn into a black smear.
const SKID_VS = `#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aAlpha;
layout(location = 2) in float aSide;   // -1 at one edge of the mark, +1 at the other
uniform mat4 uViewProj;
out float vA;
out float vSide;
out vec3 vWorld;
void main() {
  vA = aAlpha;
  vSide = aSide;
  vWorld = aPos;
  gl_Position = uViewProj * vec4(aPos, 1.0);
}`;

const SKID_FS = `#version 300 es
precision highp float;
in float vA;
in float vSide;
in vec3 vWorld;
uniform vec3 uEye;
out vec4 frag;
void main() {
  float d = length(vWorld - uEye);
  float fade = 1.0 - smoothstep(50.0, 140.0, d);
  // Soft across the width: the loaded shoulder scrubs hardest at the middle
  // of the contact patch and barely at its edges. Widened by the pixel
  // footprint so a mark a few pixels wide still has an edge, not a stair.
  float w = fwidth(vSide);
  float edge = 1.0 - smoothstep(0.45 - w, 1.0, abs(vSide));
  // Only alpha matters under (ZERO, ONE_MINUS_SRC_ALPHA).
  frag = vec4(0.0, 0.0, 0.0, vA * fade * edge);
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
  // Surface metres per pixel, for the footprint fades (see GROUND_FS).
  vec2 dp = fwidth(p);
  float px = max(dp.x, dp.y);

  // Rubbered-in racing surface: darker than the surrounding lot, darkest in
  // the middle where the cars actually run, with marbles at the edges. The
  // speckle fades to its mean by footprint; the marbles fade out entirely,
  // since they are a near-field detail with no mean worth keeping.
  float mid = 1.0 - abs(vSide);
  vec3 c = vec3(0.245, 0.248, 0.256) - 0.05 * smoothstep(0.1, 0.9, mid);
  c *= 0.86 + 0.28 * afbm(p, 0.12, px);
  c += (anoise(p, 18.0, px) - 0.5) * 0.06;
  float marbles = smoothstep(0.86, 1.0, abs(vSide)) * anoise(p, 30.0, px) * octaveFade(px, 30.0);
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

  // The same relief as the lot around it, so the glare is continuous across
  // the ribbon's edge; the shadow lookup keeps the true plane.
  vec3 up = vec3(0.0, 1.0, 0.0);
#if Q_DETAIL == 0
  vec3 n = up;
#else
  vec3 n = lotNormal(p, px, lotHeight(p, px));
#endif
  float sh = shadowAt(vWorld, up);
  // Contact darkening once, on the whole result (see GROUND_FS).
  float ao = contactAO(p);
  vec3 lit = shade(toLinear(c), n, vWorld, sh, rough, 0.0, 0.0) * ao;
  frag = finish(applyFog(lit, vWorld));
}`;

const PROP_VS = `#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
layout(location = 3) in vec3 iOffset;
layout(location = 4) in float iDown;
layout(location = 5) in float iCone;   // 1 for a cone (gets the reflective collar), 0 for other props
layout(location = 6) in float iDir;
uniform mat4 uViewProj;
out vec3 vColor;
out vec3 vNormal;
out vec3 vWorld;
out float vLocalY;   // height up the prop BEFORE any tumble, metres
out float vCone;
void main() {
  vec3 p = aPos;
  vec3 n = aNormal;
  vLocalY = aPos.y;
  vCone = iCone;
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
  vColor = aColor;
  gl_Position = uViewProj * vec4(world, 1.0);
}`;

const PROP_FS = `#version 300 es
precision highp float;
in vec3 vColor;
in vec3 vNormal;
in vec3 vWorld;
in float vLocalY;
in float vCone;
${COMMON_FS}
out vec4 frag;
void main() {
  // A degenerate triangle (CAD slivers are routine) has a zero normal, and
  // normalize(0) is NaN: a black or undefined pixel. Fall back to up.
  float nl = length(vNormal);
  vec3 n = nl > 1e-6 ? vNormal / nl : vec3(0.0, 1.0, 0.0);

  // The retro-reflective collar: a crisp band on the pre-tumble height,
  // antialiased over one pixel of it. It used to be vertex colour on a
  // six-ring mesh, which interpolated it into a 150 mm blur -- and the band
  // is the thing you aim at, so it has to be an edge.
  // Floored: on a face of constant height (the base plate, a post's top)
  // the derivative is zero, and smoothstep with equal edges is undefined.
  float e = max(fwidth(vLocalY), 1e-4);
  float band = vCone * (smoothstep(${CONE_BAND_LO} - e, ${CONE_BAND_LO} + e, vLocalY)
                      - smoothstep(${CONE_BAND_HI} - e, ${CONE_BAND_HI} + e, vLocalY));
  vec3 albedo = mix(vColor, vec3(0.93, 0.93, 0.91), band);

  float sh = shadowAt(vWorld, n);
  // The base of a cone sits in its own contact shadow.
  float ao = 0.72 + 0.28 * clamp(vWorld.y / 0.35, 0.0, 1.0);
  // Satin PVC: a broad soft highlight, no mirror. The sheeting is glossier.
  vec3 c = shade(toLinear(albedo), n, vWorld, sh, mix(0.55, 0.32, band), 0.0, 0.0) * ao;

  // Retro-reflective sheeting sends light back where it came from, so the
  // band flares when the sun is behind the camera -- the same reason a road
  // sign lights up in headlights. A hint of it, on the sunlit side only.
  if (band > 0.0) {
    vec3 v = normalize(uCam - vWorld);
    float ndl = max(dot(n, uSun), 0.0);
    float retro = pow(max(dot(v, uSun), 0.0), 8.0) * smoothstep(0.0, 0.25, ndl) * sh;
    c += band * uSunCol * retro * 0.30;
  }
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

/** An FSAE course cone: 18 in tall, orange; PROP_FS paints the white
 *  reflective collar from the height (CONE_BAND_LO..HI). */
function coneMesh() {
  const pos = [], nrm = [], col = [];
  // Twenty-four segments: at twelve the silhouette at gate distance was a
  // visible dodecagon. One ring of quads -- a frustum is linear, and the
  // collar no longer needs rings of its own -- so this is still fewer
  // vertices than the old six-ring, twelve-segment body.
  const H = 0.46, R = 0.145, SEG = 24;
  const orange = [0.96, 0.34, 0.06];
  const baseCol = [0.16, 0.16, 0.17];

  const r0 = R, r1 = R * 0.12;
  for (let s = 0; s < SEG; s++) {
    const a0 = (s / SEG) * Math.PI * 2, a1 = ((s + 1) / SEG) * Math.PI * 2;
    const p = (rr, y, a) => [Math.cos(a) * rr, y, Math.sin(a) * rr];
    const n = (a) => [Math.cos(a) * 0.9, 0.34, Math.sin(a) * 0.9];
    const quad = [
      [p(r0, 0, a0), n(a0)], [p(r1, H, a0), n(a0)], [p(r1, H, a1), n(a1)],
      [p(r0, 0, a0), n(a0)], [p(r1, H, a1), n(a1)], [p(r0, 0, a1), n(a1)],
    ];
    for (const [P, N] of quad) { pos.push(...P); nrm.push(...N); col.push(...orange); }
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
  /**
   * @param canvas
   * @param quality a preset from quality.js (`resolvePreset`); High if omitted.
   *   Its `msaa` is fixed here for the life of the context; `setQuality`
   *   changes everything else live.
   */
  constructor(canvas, quality = { id: "high", ...PRESETS.high }) {
    const gl = canvas.getContext("webgl2", {
      antialias: !!quality.msaa, alpha: false, depth: true, powerPreference: "high-performance",
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

    /** Whether the context was created multisampled; see `setQuality`. */
    this.msaa = !!quality.msaa;
    this.quality = null;
    this.shadow = [];
    this.setQuality(quality);

    this.quad = quadVao(gl);
    this.groundQuad = quadVao(gl);
    this.skid = this.makeSkidBuffer();

    this.cone = this.makeInstanced(coneMesh(), 4096);
    this.post = this.makeInstanced(boxMesh(0.12, 2.1, 0.12, [0.85, 0.85, 0.88]), 8);
    this.pole = this.makeInstanced(poleMesh(), 64);

    /** Scene exposure: linear radiance is scaled by this before the filmic
     *  curve. 0.55: at 0.45 the sunlit lot sat at display 0.45 and the whole
     *  frame read as overcast under a clear sky. */
    this.exposure = 0.55;

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
      gloves: carMeshes.gloves ? this.makeMesh(carMeshes.gloves) : null,
      dashCase: this.makeMesh(carMeshes.dashCase),
      // The driver: helmet, shoulders, belts and upper arms. Always the
      // procedural one, even under a CAD body -- a CAD export has no driver.
      // Skipped from the cockpit camera, which sits inside the helmet.
      driver: carMeshes.driver ? this.makeMesh(carMeshes.driver) : null,
      helmet: carMeshes.helmet ? this.makeMesh(carMeshes.helmet) : null,
      // The arms: canonical bones along +X, posed per frame by `placeArms`
      // from the shoulders to the gloves on the wheel. Drawn from every
      // camera -- from the seat they are the point.
      upperArm: carMeshes.upperArm ? this.makeMesh(carMeshes.upperArm) : null,
      forearm: carMeshes.forearm ? this.makeMesh(carMeshes.forearm) : null,
    };
    /** Shoulders, glove positions, bone lengths -- see `armRig` in carmesh. */
    this.arms = carMeshes.arms ?? null;

    // The dash screen: a textured quad, and the canvas the HUD draws into.
    this.dashScreen = this.makeScreenQuad(GEO.dashHalfWidth, GEO.dashHalfHeight);
    this.dashPanel = this.makePanelTexture(DASH_TEX_W, DASH_TEX_H);
    // The dash and steering wheel frames, placed once per frame BEFORE the
    // shadow pass (`placeCockpit`), because both are drawn into the near
    // cascade as well as the main pass.
    this.dashModel = mat4();
    this.steerModel = mat4();
    // The wheel's own frame in CHASSIS space (steerModel without the
    // chassis in front of it), kept so the arm IK can find the gloves
    // without inverting the chassis every frame. One for the ghost too.
    this.wheelLocal = mat4();
    this._ghostWheelLocal = mat4();
    this._ghostGloveModel = mat4();
    // Arm model matrices: [upper L, fore L, upper R, fore R].
    this._armMats = [mat4(), mat4(), mat4(), mat4()];
    this._ghostArmMats = [mat4(), mat4(), mat4(), mat4()];
    this._armT = mat4();
    this._armV = Array.from({ length: 8 }, () => new Float64Array(3));

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
    // The moving suspension of the last CAD car, if any.
    for (const rp of this.rigParts ?? []) {
      if (rp.mesh?.vao) gl.deleteVertexArray(rp.mesh.vao);
      for (const b of Object.values(rp.mesh?.buffers ?? {})) gl.deleteBuffer(b);
    }
    this.rigParts = null;
    this.susp = null;
    if (car?.rig) {
      this.rigParts = car.rig.parts.map((p) => ({ corner: p.corner, role: p.role, mesh: this.makeMesh(p.mesh), model: mat4() }));
      this.susp = new SuspensionRig(car.rig.corners);
      this._rigInv = mat4();
    }
    // The pedals and the hand clutch ride in the same list, so every pass
    // that draws the suspension (main, shadows, ghost) draws them too.
    if (car?.controls) {
      this.rigParts = (this.rigParts ?? []).concat(car.controls.map((c) => ({
        control: c.name, pivot: c.pivot, axis: normalize([...c.axis]), curve: c.curve,
        mesh: this.makeMesh(c.mesh), model: mat4(),
      })));
    }
    for (const mesh of Object.values(this.car)) {
      if (mesh?.vao) gl.deleteVertexArray(mesh.vao);
      for (const b of Object.values(mesh?.buffers ?? {})) gl.deleteBuffer(b);
    }
    // The dash case is not part of any import: it is the same AiM unit on
    // every car, so it always comes from the procedural builder. Dropping it
    // here left `drawCar` binding an undefined mesh the first time a CAD
    // model was loaded.
    const meshes = buildCarMeshes(car?.cockpit ? this.cadDriverParams() : this.carParams ?? null);
    if (car) {
      this.car = {
        body: this.makeMesh(car.body),
        tire: this.makeMesh(car.tire),
        rim: this.makeMesh(car.rim),
        steeringWheel: this.makeMesh(this.steerOverride ?? car.steeringWheel),
        gloves: meshes.gloves ? this.makeMesh(meshes.gloves) : null,
        dashCase: this.makeMesh(this.dashOverride ?? meshes.dashCase),
        driver: meshes.driver ? this.makeMesh(meshes.driver) : null,
        helmet: meshes.helmet ? this.makeMesh(meshes.helmet) : null,
        upperArm: meshes.upperArm ? this.makeMesh(meshes.upperArm) : null,
        forearm: meshes.forearm ? this.makeMesh(meshes.forearm) : null,
      };
      this.arms = meshes.arms ?? null;
      this.carModel = { hubs: car.hubs, steerCentre: car.steerCentre, cockpit: car.cockpit ?? null };
    } else {
      this.car = {
        body: this.makeMesh(meshes.body),
        tire: this.makeMesh(meshes.tire),
        rim: this.makeMesh(meshes.rim),
        steeringWheel: this.makeMesh(this.steerOverride ?? meshes.steeringWheel),
        gloves: meshes.gloves ? this.makeMesh(meshes.gloves) : null,
        dashCase: this.makeMesh(this.dashOverride ?? meshes.dashCase),
        driver: meshes.driver ? this.makeMesh(meshes.driver) : null,
        helmet: meshes.helmet ? this.makeMesh(meshes.helmet) : null,
        upperArm: meshes.upperArm ? this.makeMesh(meshes.upperArm) : null,
        forearm: meshes.forearm ? this.makeMesh(meshes.forearm) : null,
      };
      this.arms = meshes.arms ?? null;
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
  /**
   * Draw the team's steering wheel (`loadSteeringWheelModel`) instead of the
   * procedural one, on every body -- procedural, imported body, or whole CAD
   * car -- and across rebuilds: each place a steering wheel mesh is made reads
   * `steerOverride` first.
   */
  /** The real dash case (`loadDashModel`), kept across rebuilds like the wheel. */
  useDashModel(mesh) {
    this.dashOverride = mesh ?? null;
    if (!this.car || !mesh) return;
    const gl = this.gl;
    const d = this.car.dashCase;
    if (d?.vao) gl.deleteVertexArray(d.vao);
    for (const b of Object.values(d?.buffers ?? {})) gl.deleteBuffer(b);
    this.car.dashCase = this.makeMesh(mesh);
  }

  useSteeringWheelModel(mesh) {
    this.steerOverride = mesh ?? null;
    if (!this.car || !mesh) return;
    const gl = this.gl;
    const sw = this.car.steeringWheel;
    if (sw?.vao) gl.deleteVertexArray(sw.vao);
    for (const b of Object.values(sw?.buffers ?? {})) gl.deleteBuffer(b);
    this.car.steeringWheel = this.makeMesh(mesh);
  }

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
    // wheelbase must not quietly replace it with procedural geometry. Its
    // driver is, though: the eye height moves the helmet and the shoulders.
    if (this.carModel) {
      if (this.carModel.cockpit) this.rebuildDriver(buildCarMeshes(this.cadDriverParams()));
      return;
    }
    if (this.bodyModel) {
      // An imported body is not built from the vehicle parameters either.
      const meshes = buildCarMeshes(params);
      const gl = this.gl;
      const sw = this.car.steeringWheel;
      if (sw?.vao) gl.deleteVertexArray(sw.vao);
      for (const b of Object.values(sw?.buffers ?? {})) gl.deleteBuffer(b);
      this.car.steeringWheel = this.makeMesh(this.steerOverride ?? meshes.steeringWheel);
      this.arms = meshes.arms ?? null;
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
      for (const key of ["body", "steeringWheel", "driver", "helmet", "upperArm", "forearm"]) {
        const mesh = this.car[key];
        if (mesh?.vao) gl.deleteVertexArray(mesh.vao);
        for (const b of Object.values(mesh?.buffers ?? {})) gl.deleteBuffer(b);
      }
      this.car.body = this.makeMesh(meshes.body);
      this.car.steeringWheel = this.makeMesh(this.steerOverride ?? meshes.steeringWheel);
      this.car.gloves = meshes.gloves ? this.makeMesh(meshes.gloves) : null;
      this.car.driver = meshes.driver ? this.makeMesh(meshes.driver) : null;
      this.car.helmet = meshes.helmet ? this.makeMesh(meshes.helmet) : null;
      this.car.upperArm = meshes.upperArm ? this.makeMesh(meshes.upperArm) : null;
      this.car.forearm = meshes.forearm ? this.makeMesh(meshes.forearm) : null;
      this.arms = meshes.arms ?? null;
      return;
    }
    const gl = this.gl;
    const rebuilt = buildCarMeshes(params);
    const body = rebuilt.body;
    this.arms = rebuilt.arms ?? null;   // the shoulders follow the eye point
    gl.bindBuffer(gl.ARRAY_BUFFER, this.car.body.buffers.position);
    gl.bufferData(gl.ARRAY_BUFFER, body.position, gl.STATIC_DRAW);
    this.car.body.count = body.count;
    this.carParams = params;
  }

  /** The vehicle parameters with the driver seated where the CAD car's
   *  head restraint puts them (CAD_EYE_AHEAD_OF_CG_M). */
  cadDriverParams() {
    return { ...(this.carParams ?? SDM26), eyeAheadOfCgM: CAD_EYE_AHEAD_OF_CG_M };
  }

  /** Replace the driver, helmet, gloves and arms, keeping everything else. */
  rebuildDriver(meshes) {
    const gl = this.gl;
    for (const key of ["driver", "helmet", "gloves", "upperArm", "forearm"]) {
      const mesh = this.car[key];
      if (mesh?.vao) gl.deleteVertexArray(mesh.vao);
      for (const b of Object.values(mesh?.buffers ?? {})) gl.deleteBuffer(b);
      this.car[key] = meshes[key] ? this.makeMesh(meshes[key]) : null;
    }
    this.arms = meshes.arms ?? null;
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

  /** An offscreen canvas and the texture it is uploaded to when it changes. */
  makePanelTexture(w, h) {
    const gl = this.gl;
    const canvas = document.createElement("canvas");
    canvas.width = w;
    canvas.height = h;
    const ctx = canvas.getContext("2d");
    const tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, tex);
    // Immutable storage with a full mip chain, allocated once. Uploads go
    // through texSubImage2D into it rather than a texImage2D that reallocates
    // the texture every time the panel changes.
    //
    // Mips, because the panel is MINIFIED: from the seat the 768 x 462 canvas
    // lands on about 320 x 150 pixels of screen, 2.4x under, and from the
    // chase camera 30x under. With a plain LINEAR filter every 1 px tick and
    // peak marker sparkled and the small type was noise. Regenerating the
    // chain (1.4 MB) is ~0.1 ms at the rate the panel actually redraws.
    const levels = Math.floor(Math.log2(Math.max(w, h))) + 1;
    gl.texStorage2D(gl.TEXTURE_2D, levels, gl.RGBA8, w, h);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    // The panel is viewed at ~40 degrees from the seat, where a trilinear
    // lookup blurs along the foreshortened axis; anisotropic filtering keeps
    // the type sharp there. Optional: absent, the mips alone still help.
    const aniso = gl.getExtension("EXT_texture_filter_anisotropic");
    if (aniso) {
      const max = gl.getParameter(aniso.MAX_TEXTURE_MAX_ANISOTROPY_EXT) || 1;
      gl.texParameterf(gl.TEXTURE_2D, aniso.TEXTURE_MAX_ANISOTROPY_EXT, Math.min(8, max));
    }
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
    bindInst(I_CONE, 1, 16);
    bindInst(I_DIR, 1, 20);

    gl.bindVertexArray(null);
    return {
      vao, instBuf, count: mesh.count, maxInstances,
      data: new Float32Array(maxInstances * INST_FLOATS),
      n: 0,
    };
  }

  /**
   * Apply a graphics preset (quality.js). Live, except MSAA, which belongs to
   * the context: a change there is kept in `quality` and takes effect on the
   * next load. Recompiles the programs only when a shader define changed and
   * reallocates the shadow maps only when their size did.
   */
  setQuality(q) {
    const gl = this.gl;
    const old = this.quality;
    this.quality = { ...q };
    if (!old || old.shadows !== q.shadows || old.detail !== q.detail || old.shadowSize !== q.shadowSize) {
      const defs = `#define Q_SHADOWS ${q.shadows}\n#define Q_DETAIL ${q.detail}\n#define SHADOW_RES ${q.shadowSize}.0\n`;
      const progs = {
        sky: [SKY_VS, SKY_FS], ground: [GROUND_VS, GROUND_FS], ribbon: [RIBBON_VS, RIBBON_FS],
        skid: [SKID_VS, SKID_FS], prop: [PROP_VS, PROP_FS], car: [CAR_VS, CAR_FS],
        screen: [SCREEN_VS, SCREEN_FS], depthCar: [DEPTH_CAR_VS, DEPTH_FS], depthProp: [DEPTH_PROP_VS, DEPTH_FS],
      };
      const built = {};
      for (const [name, [vs, fs]] of Object.entries(progs)) built[name] = program(gl, vs, withDefines(fs, defs));
      // Only once every program has compiled: a failure above leaves the
      // previous set drawing rather than half of each.
      for (const p of [this.progSky, this.progGround, this.progRibbon, this.progSkid, this.progProp,
        this.progCar, this.progScreen, this.progDepthCar, this.progDepthProp]) if (p) gl.deleteProgram(p);
      this.progSky = built.sky; this.progGround = built.ground; this.progRibbon = built.ribbon;
      this.progSkid = built.skid; this.progProp = built.prop; this.progCar = built.car;
      this.progScreen = built.screen; this.progDepthCar = built.depthCar; this.progDepthProp = built.depthProp;
      this._screenU = null; // the dash screen's lazily cached locations belong to the old program

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
    }
    if (!old || old.shadowSize !== q.shadowSize) {
      for (const sm of this.shadow) { gl.deleteTexture(sm.tex); gl.deleteFramebuffer(sm.fbo); }
      this.shadow = [this.makeShadowMap(q.shadowSize), this.makeShadowMap(q.shadowSize)];
    }
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
      // The local half width: a generated endurance course opens up through
      // its passing zones, and the ribbon has to follow the cones out.
      const hw = track.widthAt ? track.widthAt(i) / 2 : half;
      for (const sgn of [1, -1]) {
        pos.push(x + nx * hw * sgn, 0.012, -(y + ny * hw * sgn));
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
    // The preset's render scale, on top of a cap on the display's pixel
    // ratio; the canvas is stretched to the window by CSS either way.
    const q = this.quality;
    const dpr = Math.min(devicePixelRatio || 1, q.maxDpr) * q.scale;
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
      // Bolted to the car in every way but roll: the horizon stays level
      // and the cockpit is what tilts. The in-car views never roll.
      const h = cam.cgHeight ?? 0;
      this.chain(this.camFrame, [
        translation(T[0], cam.x, h, -cam.y),
        rotY(T[1], cam.psi),
        rotZ(T[2], cam.pitchRad),
        translation(T[4], 0, -h, 0),
      ]);
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
    // Near 0.12 m. Nothing is closer to any eye than that: from the seat
    // (0.70 m up, 0.15 m behind the CG) the nearest thing is the top of the
    // steering wheel, 0.43 m ahead and 0.2 m down, ~0.4 m away even with
    // the grips standing proud and the head leaning in; the nose camera has
    // the nose tip behind and below it; the walkaround floors its radius at
    // 1.2 m. It was 0.06, and a 24-bit depth buffer's resolution goes as
    // the near plane, so halving it doubled the z-fighting between the
    // ribbon, the cone plates and the venue's 2-4 mm layers at range. Far
    // 1100 m clears the tree line on the far side of the endurance course
    // with the sky quad behind it.
    perspective(this.proj, fovRad, aspect, 0.12, 1100);
    lookAlong(this.view, eye, forward, up);
    multiply(this.viewProj, this.proj, this.view);

    // ---- wheel and cockpit transforms, used by both passes ----
    this.placeWheels(s);
    this.placeRig(s);
    this.placeGhost(s);
    this.placeCockpit(s);
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

    // --- course ribbon, FIRST of the surfaces: it lies on top of the lot,
    // so with its depth already written the early-Z test throws away every
    // ground fragment under it before the (expensive) ground shader runs. ---
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

    // --- ground ---
    gl.useProgram(this.progGround);
    const ug = this.u.ground;
    this.setCommon(ug, eye);
    this.setContact(ug, cam);
    gl.uniformMatrix4fv(ug.uViewProj, false, this.viewProj);
    gl.uniform2f(ug.uCamXZ, eye[0], eye[2]);
    // Out to where the fog is complete (3 km is 1 - exp(-(3000/520)^1.6),
    // i.e. all of it); at 900 m the edge was 9 % unfogged and drew a line.
    gl.uniform1f(ug.uExtent, 3000);
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

    // --- rubber on the surface: darkens the lit ribbon and lot ---
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

    this.drawCar(s, eye);

    // --- sky, after every OPAQUE pass: its quad sits at z = 0.9999, so
    // wherever anything was drawn the depth test rejects it before the
    // (expensive) sky shader runs. Drawn first it ran on every pixel the
    // ground and car then covered. But it must come BEFORE the translucent
    // ghost: the ghost writes no depth, so a sky drawn after it passed the
    // depth test wherever the ghost stood against the sky and painted over
    // it -- from the seat, everything on the ghost above eye height. ---
    gl.depthMask(false);
    gl.useProgram(this.progSky);
    const us = this.u.sky;
    const right = normalize(cross3(forward, up));
    const tanH = Math.tan(fovRad / 2);
    gl.uniform3f(us.uRight, right[0], right[1], right[2]);
    gl.uniform3f(us.uUp, up[0], up[1], up[2]);
    gl.uniform3f(us.uFwd, forward[0], forward[1], forward[2]);
    gl.uniform2f(us.uTan, tanH * aspect, tanH);
    this.setCommon(us, eye);
    gl.bindVertexArray(this.quad);
    this.drawArrays(gl.TRIANGLES, 0, 6);
    gl.depthMask(true);

    // The ghost last, because it is translucent and has to blend over
    // whatever is behind it -- the sky included.
    this.drawGhost(s, eye);

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
    // The cloud drift, for every program that reads the sky. A program the
    // compiler pruned it from has no location, and a null location is a
    // no-op by spec, not an error.
    gl.uniform1f(u.uTime, this.time);
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

  /**
   * Pose the CAD car's suspension (suspensionRig.js): each wheel's hub, which
   * rides the level axle frame, is taken into the body's frame; how far it
   * has moved from where it was modelled is the corner's travel, and the
   * linkage is solved to meet it.
   */
  placeRig(s) {
    for (const rp of this.rigParts ?? []) {
      if (rp.control) this.placeControl(rp, s.pedals?.[rp.control] ?? 0);
    }
    if (!this.susp || !this.rigParts || !this.carModel?.hubs) return;
    const inv = invertRigid(this._rigInv, this.chassis);
    for (const hub of this.carModel.hubs) {
      const name = hub.name.toLowerCase();
      const local = transformPoint(inv, transformPoint(this.axleFrame, [hub.x, hub.y, hub.z]));
      this.susp.solve(name, [local[0] - hub.x, local[1] - hub.y, local[2] - hub.z], hub.front ? (s.wheels?.steerRad ?? 0) : 0);
    }
    for (const rp of this.rigParts) {
      if (rp.control) continue;
      const m = this.susp.matrix(rp.corner, rp.role);
      if (m) multiply(rp.model, this.chassis, m);
      else rp.model.set(this.chassis);
    }
  }

  /**
   * A pedal or the hand clutch, turned about its pivot by the driver's input
   * through the part's own travel curve (input 0..1 -> degrees, piecewise
   * linear). Positive pushes the top forward, as a foot or a hand does.
   */
  placeControl(rp, input) {
    const x = Math.max(0, Math.min(1, input || 0));
    const c = rp.curve;
    let deg = c[c.length - 1][1];
    for (let i = 1; i < c.length; i++) {
      if (x <= c[i][0]) {
        const [x0, d0] = c[i - 1], [x1, d1] = c[i];
        deg = d0 + (d1 - d0) * (x1 > x0 ? (x - x0) / (x1 - x0) : 0);
        break;
      }
    }
    const T = this._t, p = rp.pivot;
    this.chain(rp.model, [
      this.chassis,
      translation(T[0], p[0], p[1], p[2]),
      axisAngle(T[1], rp.axis, (deg * Math.PI) / 180),
      translation(T[2], -p[0], -p[1], -p[2]),
    ]);
  }

  makeSkidBuffer() {
    const gl = this.gl;
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const buf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferData(gl.ARRAY_BUFFER, SKID_MAX * SKID_FLOATS * 4, gl.DYNAMIC_DRAW);
    const stride = 5 * 4;   // x, y, z, alpha, side
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, stride, 0);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 1, gl.FLOAT, false, stride, 12);
    gl.enableVertexAttribArray(2);
    gl.vertexAttribPointer(2, 1, gl.FLOAT, false, stride, 16);
    gl.bindVertexArray(null);
    return {
      vao, buf, seg: new Float32Array(SKID_FLOATS), n: 0, head: 0,
      strip: [this._newStrip(), this._newStrip(), this._newStrip(), this._newStrip()],
    };
  }

  /**
   * One wheel's strip state. `pts` is how many points are held: the latest
   * sample (l*) and, once there are two, the one before it (p*) with the
   * joint offset (pox, poz) already decided and the unit direction (ux, uz)
   * from p to l. A segment is emitted one sample LATE -- when the sample
   * after its far end arrives -- because that is when the far end's mitre
   * is known. At any driving frame rate the lag is under the tyre.
   */
  _newStrip() {
    return { pts: 0, lx: 0, lz: 0, la: 0, px: 0, pz: 0, pox: 0, poz: 0, pa: 0, ux: 0, uz: 0 };
  }

  /** Forget every mark: a new course, a fresh surface. */
  clearSkids() {
    this.skid.n = 0;
    this.skid.head = 0;
    for (const st of this.skid.strip) st.pts = 0;
  }

  /**
   * Lay rubber under any wheel that is sliding this frame.
   *
   * `intensity` is one number per wheel, 0..1, decided by the game from the
   * slip the tyre is at; here it only sets how dark the mark is. Each wheel
   * lays a continuous MITRED strip through its samples: consecutive segments
   * share the joint's bisector edge, so a curve neither gaps on the outside
   * nor overlaps (and double-darkens) on the inside the way independent
   * quads did. The intensity is per point, so it ramps along the strip
   * rather than stepping per frame, and a wheel that starts or stops
   * sliding gets a segment fading from or to nothing. A jump (a respawn, a
   * seek) breaks the strip rather than drawing a streak across the course.
   */
  addSkids(intensity) {
    const sk = this.skid;
    for (let i = 0; i < 4; i++) {
      const x = this._hubXZ[i * 2];
      const z = this._hubXZ[i * 2 + 1];
      const st = sk.strip[i];
      const a = intensity[i] ?? 0;
      const sliding = a > 0.15;
      // Faint at the onset, never black: rubber on asphalt is a shade
      // darker, not paint.
      const alpha = sliding ? Math.min(0.38, 0.04 + a * 0.34) : 0;
      if (st.pts === 0) { st.lx = x; st.lz = z; st.la = alpha; st.pts = 1; continue; }
      const dx = x - st.lx, dz = z - st.lz;
      const len = Math.hypot(dx, dz);
      if (len < 0.015) continue;                      // not moved: wait
      if (len > 2.5) {                                // a jump: break the strip
        this._capSkid(st);
        st.lx = x; st.lz = z; st.la = alpha; st.pts = 1;
        continue;
      }
      if (!sliding && st.la <= 0) {                   // rolling: just follow
        this._capSkid(st);
        st.lx = x; st.lz = z; st.pts = 1;
        continue;
      }
      // A new point. Decide the joint offset at the previous point: the
      // plain normal if it starts the strip, otherwise the mitre between the
      // incoming and outgoing directions, capped at twice the half-width so
      // a sharp kink cannot spike.
      const ux = dx / len, uz = dz / len;
      let lox = -uz * SKID_HALF_W, loz = ux * SKID_HALF_W;
      if (st.pts === 2) {
        const bx = st.ux + ux, bz = st.uz + uz;
        const bl = Math.hypot(bx, bz);
        if (bl > 1e-3) {
          const cosHalf = (bx * ux + bz * uz) / bl;
          const s = SKID_HALF_W / Math.max(cosHalf, 0.5);
          lox = (-bz / bl) * s; loz = (bx / bl) * s;
        }
        this._emitSkid(st.px, st.pz, st.pox, st.poz, st.pa, st.lx, st.lz, lox, loz, st.la);
      }
      st.px = st.lx; st.pz = st.lz; st.pox = lox; st.poz = loz; st.pa = st.la;
      st.lx = x; st.lz = z; st.la = alpha; st.ux = ux; st.uz = uz; st.pts = 2;
      // Rolled off: this point is the fade-out, so close the strip on it.
      if (!sliding) this._capSkid(st);
    }
  }

  /** Emit the pending segment with a plain end, and drop back to one point. */
  _capSkid(st) {
    if (st.pts === 2) {
      this._emitSkid(st.px, st.pz, st.pox, st.poz, st.pa,
                     st.lx, st.lz, -st.uz * SKID_HALF_W, st.ux * SKID_HALF_W, st.la);
      st.pts = 1;
    }
  }

  /** One quad of the strip into the ring buffer: from a (offset ao) to b (offset bo). */
  _emitSkid(ax, az, aox, aoz, aa, bx, bz, box, boz, ba) {
    const gl = this.gl;
    const sk = this.skid;
    const v = sk.seg;
    const put = (k, x, z, alpha, side) => {
      v[k] = x; v[k + 1] = SKID_Y; v[k + 2] = z; v[k + 3] = alpha; v[k + 4] = side;
    };
    put(0, ax + aox, az + aoz, aa, 1); put(5, ax - aox, az - aoz, aa, -1); put(10, bx - box, bz - boz, ba, -1);
    put(15, ax + aox, az + aoz, aa, 1); put(20, bx - box, bz - boz, ba, -1); put(25, bx + box, bz + boz, ba, 1);
    gl.bindBuffer(gl.ARRAY_BUFFER, sk.buf);
    gl.bufferSubData(gl.ARRAY_BUFFER, sk.head * SKID_FLOATS * 4, v);
    sk.head = (sk.head + 1) % SKID_MAX;
    sk.n = Math.min(sk.n + 1, SKID_MAX);
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
    // Multiplicative: dst *= (1 - alpha). The mark is a darkening of the
    // surface it is on, which has already been lit, shadowed and fogged, so
    // it inherits all three. The fragment's colour is irrelevant.
    gl.blendFunc(gl.ZERO, gl.ONE_MINUS_SRC_ALPHA);
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
    // The ghost has no wheel drawn, but its driver's arms still follow the
    // steering it logged, or they would sit at dead-ahead through every
    // corner of the lap.
    this.wheelFrame(this._ghostWheelLocal, g.steerRad || 0, s.wheels?.steerRatio);
    this.placeArms(this._ghostWheelLocal, this._ghostChassis, this._ghostArmMats);
    multiply(this._ghostGloveModel, this._ghostChassis, this._ghostWheelLocal);
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
    // Units 0 and 1 hold the cascades here: `drawDashScreen` puts the near
    // one back the moment it is done with the panel.
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
    // The ghost's suspension is drawn at its modelled (static) pose.
    for (const rp of this.rigParts ?? []) part(rp.mesh, this._ghostChassis, MAT.paint);
    if (this.car.driver) part(this.car.driver, this._ghostChassis, MAT.suit);
    if (this.car.helmet) part(this.car.helmet, this._ghostChassis, MAT.helmet);
    if (this.car.gloves) part(this.car.gloves, this._ghostGloveModel, MAT.suit);
    if (this.arms && this.car.upperArm && this.car.forearm) {
      for (let k = 0; k < 4; k++) {
        part(k & 1 ? this.car.forearm : this.car.upperArm, this._ghostArmMats[k], MAT.suit);
      }
    }
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
    // Shadows off: the instances above are all the main pass needs.
    if (this.quality.shadows === 0) return;

    // Light view with the eye at the origin: only the ortho window moves,
    // which is what makes the texel snap possible.
    lookAlong(this.lightView, [0, 0, 0], [-SUN[0], -SUN[1], -SUN[2]], [0, 1, 0]);
    const c = transformPoint(this.lightView, [cam.x, 0.3, -cam.y]);
    for (let i = 0; i < 2; i++) {
      const half = SHADOW_HALF[i];
      const texel = (2 * half) / this.quality.shadowSize;
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

      // Car: body and tyres in both cascades.
      gl.useProgram(this.progDepthCar);
      const ud = this.u.depthCar;
      gl.uniformMatrix4fv(ud.uViewProj, false, this.lightViewProj[i]);
      gl.uniformMatrix4fv(ud.uModel, false, this.chassis);
      gl.bindVertexArray(this.car.body.vao);
      this.drawArrays(gl.TRIANGLES, 0, this.car.body.count);
      for (const rp of this.rigParts ?? []) {
        gl.uniformMatrix4fv(ud.uModel, false, rp.model);
        gl.bindVertexArray(rp.mesh.vao);
        this.drawArrays(gl.TRIANGLES, 0, rp.mesh.count);
      }
      for (let k = 0; k < 4; k++) {
        gl.uniformMatrix4fv(ud.uModel, false, this._wheelMats[k]);
        gl.bindVertexArray(this.car.tire.vao);
        this.drawArrays(gl.TRIANGLES, 0, this.car.tire.count);
      }
      // The cockpit's own casters -- steering wheel, dash case, rims -- in
      // the NEAR cascade only. From the far cascade's 62 mm texels they are
      // inside the body's and tyres' silhouettes anyway; from the driver's
      // seat, the wheel's shadow on the tub and the spokes' on the inner
      // sidewall are what give the interior any depth at all. Without them
      // the cockpit was lit flat, every surface the same sunlit value.
      if (i === 0) {
        const cast = (mesh, model) => {
          if (!mesh) return;
          gl.uniformMatrix4fv(ud.uModel, false, model);
          gl.bindVertexArray(mesh.vao);
          this.drawArrays(gl.TRIANGLES, 0, mesh.count);
        };
        cast(this.car.steeringWheel, this.steerModel);
        cast(this.car.dashCase, this.dashModel);
        for (let k = 0; k < 4; k++) cast(this.car.rim, this._wheelMats[k]);
        // The arms and gloves, only while the driver is drawn: a shadow of
        // hands that are not there is worse than no shadow.
        if (this.arms && !s.hideDriver) {
          for (let k = 0; k < 4; k++) cast(k & 1 ? this.car.forearm : this.car.upperArm, this._armMats[k]);
          cast(this.car.gloves, this.steerModel);
        }
      }
      // The driver's helmet stands proud of the tub, so its shadow is on the
      // deck from any camera; cast it in both cascades, whether or not the
      // cockpit camera is hiding the mesh itself.
      for (const mesh of [this.car.driver, this.car.helmet]) {
        if (!mesh) continue;
        gl.uniformMatrix4fv(ud.uModel, false, this.chassis);
        gl.bindVertexArray(mesh.vao);
        this.drawArrays(gl.TRIANGLES, 0, mesh.count);
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
    for (const rp of this.rigParts ?? []) part(rp.mesh, rp.model, null, MAT.paint);
    // The driver rides on the chassis frame. Not from the cockpit camera:
    // that eye is inside the helmet, and a helmet lining is not a view.
    if (!s.hideDriver) {
      if (this.car.driver) part(this.car.driver, this.chassis, null, MAT.suit);
      if (this.car.helmet) part(this.car.helmet, this.chassis, null, MAT.helmet);
    }
    // The arms and gloves go with the rest of the driver: not from the
    // seat. The real driver's hands are on the real rim, and a second pair
    // drawn over the dash read as wrong, not as presence. The arms are
    // posed by `placeArms` so the elbows stay in the tub at any lock.
    if (!s.hideDriver && this.arms && this.car.upperArm && this.car.forearm) {
      for (let k = 0; k < 4; k++) {
        part(k & 1 ? this.car.forearm : this.car.upperArm, this._armMats[k], null, MAT.suit);
      }
      if (this.car.gloves) part(this.car.gloves, this.steerModel, null, MAT.suit);
    }

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

    // The dash FIRST, then the wheel, both on the frames `placeCockpit` set
    // up before the shadow pass. Drawn in that order so the depth buffer
    // does the occluding; nothing here needs to know where the grips are.
    if (this.car.dashCase) part(this.car.dashCase, this.dashModel, null, MAT.dash);
    part(this.car.steeringWheel, this.steerModel, null, MAT.wheel);

    // The screen last of the car's parts: it is the only thing drawn with a
    // different program, and switching back and forth per object costs more
    // than doing it once.
    this.drawDashScreen();
  }

  /**
   * The steering wheel's and the dash's model matrices for this frame.
   *
   * Once per frame, BEFORE the shadow pass, because both are drawn into the
   * near cascade and then again in the main pass, and the two must agree or
   * the wheel's shadow would lag its spin by a frame.
   *
   * The wheel's own frame has the rotation axis on +Z, so the column basis
   * tilts that frame onto the column before the spin is applied. The dash
   * is bolted to the column shroud, not to the wheel, so it shares the
   * column's frame and its tilt but NOT its spin -- which is the whole
   * reason a grip sweeps across it at 90 degrees of lock instead of the
   * dash swinging with the driver's hands.
   */
  placeCockpit(s) {
    const T = this._t;
    const w = s.wheels;
    const tilt = GEO.steerTiltRad;
    const basis = T[3];
    basis.set([
      0, 0, -1, 0,
      Math.sin(tilt), Math.cos(tilt), 0, 0,
      Math.cos(tilt), -Math.sin(tilt), 0, 0,
      0, 0, 0, 1,
    ]);
    const sc = this.carModel?.steerCentre ?? GEO.steerCentre;
    const dc = GEO.dashCentre;
    // The team's car: the dash where their assembly has it.
    if (this.carModel?.cockpit) multiply(this.dashModel, this.chassis, this.carModel.cockpit.dash);
    else this.chain(this.dashModel, [
      this.chassis,
      translation(T[0], sc[0], sc[1], sc[2]),
      basis,
      translation(T[1], dc[0], dc[1], dc[2]),
      rotX(T[2], GEO.dashTiltRad),
    ]);
    this.wheelFrame(this.wheelLocal, w.steerRad, w.steerRatio);
    multiply(this.steerModel, this.chassis, this.wheelLocal);
    this.placeArms(this.wheelLocal, this.chassis, this._armMats);
  }

  /**
   * The steering wheel's frame in chassis space: column position, column
   * tilt, then the spin. `steerModel` is the chassis times this; the arm IK
   * reads it directly, because the gloves are authored in this frame.
   */
  wheelFrame(out, steerRad, steerRatio) {
    const T = this._t;
    const tilt = GEO.steerTiltRad;
    const basis = T[3];
    basis.set([
      0, 0, -1, 0,
      Math.sin(tilt), Math.cos(tilt), 0, 0,
      Math.cos(tilt), -Math.sin(tilt), 0, 0,
      0, 0, 0, 1,
    ]);
    const sc = this.carModel?.steerCentre ?? GEO.steerCentre;
    // The team's car: the wheel where their assembly has it (column at 18 deg).
    if (this.carModel?.cockpit) {
      return multiply(out, this.carModel.cockpit.steer, rotZ(T[1], -steerRad * (steerRatio ?? GEO.steeringRatio)));
    }
    return this.chain(out, [
      translation(T[0], sc[0], sc[1], sc[2]),
      basis,
      rotZ(T[1], -steerRad * (steerRatio ?? GEO.steeringRatio)),
    ]);
  }

  /**
   * Pose the driver's arms: a two-bone IK per side from the shoulder (fixed
   * in the chassis) to the glove (fixed on the wheel, so it goes wherever
   * the steering takes it).
   *
   * The reach is clamped to what the two bones can span; with the lengths
   * in `armRig` the wheel never carries a hand out of reach, so the fist
   * always sits on the wrist. The elbow angle comes from the law of
   * cosines, and the elbow itself is dropped into the plane of the
   * shoulder-hand line and the hint direction -- down and a little
   * outboard, which is where an elbow goes in a tub 600 mm wide. The
   * frames are built from the bone directions with the hint as the up
   * reference; both bones are tubes, so their roll does not matter.
   *
   * `wheelLocal` is the wheel's frame in CHASSIS space and `chassis` the
   * frame the result is drawn in; the ghost passes its own of each. Writes
   * `out` as [upper L, fore L, upper R, fore R]. No allocation.
   */
  placeArms(wheelLocal, chassis, out) {
    const rig = this.arms;
    if (!rig || !this.car.upperArm || !this.car.forearm) return;
    const V = this._armV;
    const S = V[0], H = V[1], U = V[2], N = V[3], E = V[4], X = V[5], Y = V[6], Z = V[7];
    const a = rig.lUpper, b = rig.lFore;
    const EPS = 0.005;
    const setCross = (o, p, q) => {
      o[0] = p[1] * q[2] - p[2] * q[1];
      o[1] = p[2] * q[0] - p[0] * q[2];
      o[2] = p[0] * q[1] - p[1] * q[0];
    };
    const unit = (o) => {
      const l = Math.hypot(o[0], o[1], o[2]) || 1;
      o[0] /= l; o[1] /= l; o[2] /= l;
      return l;
    };
    for (let side = 0; side < 2; side++) {
      const sh = rig.shoulders[side], hd = rig.hands[side], hint = rig.elbowHints[side];
      S[0] = sh[0]; S[1] = sh[1]; S[2] = sh[2];
      // The glove, carried from the wheel's frame into the chassis's.
      H[0] = wheelLocal[0] * hd[0] + wheelLocal[4] * hd[1] + wheelLocal[8] * hd[2] + wheelLocal[12];
      H[1] = wheelLocal[1] * hd[0] + wheelLocal[5] * hd[1] + wheelLocal[9] * hd[2] + wheelLocal[13];
      H[2] = wheelLocal[2] * hd[0] + wheelLocal[6] * hd[1] + wheelLocal[10] * hd[2] + wheelLocal[14];
      U[0] = H[0] - S[0]; U[1] = H[1] - S[1]; U[2] = H[2] - S[2];
      const dist = unit(U);
      const d = Math.min(Math.max(dist, Math.abs(a - b) + EPS), a + b - EPS);
      // Law of cosines: the elbow's distance along the shoulder-hand line
      // and its drop off it.
      const xE = (d * d + a * a - b * b) / (2 * d);
      const h = Math.sqrt(Math.max(0, a * a - xE * xE));
      // The hint, made perpendicular to the line.
      let k = hint[0] * U[0] + hint[1] * U[1] + hint[2] * U[2];
      N[0] = hint[0] - k * U[0]; N[1] = hint[1] - k * U[1]; N[2] = hint[2] - k * U[2];
      if (unit(N) < 1e-4) {
        // Hint along the bone line: fall back to straight down.
        k = -U[1];
        N[0] = -k * U[0]; N[1] = -1 - k * U[1]; N[2] = -k * U[2];
        unit(N);
      }
      E[0] = S[0] + U[0] * xE + N[0] * h;
      E[1] = S[1] + U[1] * xE + N[1] * h;
      E[2] = S[2] + U[2] * xE + N[2] * h;
      // Upper arm: +X from the shoulder to the elbow.
      X[0] = E[0] - S[0]; X[1] = E[1] - S[1]; X[2] = E[2] - S[2];
      unit(X);
      setCross(Z, X, N); unit(Z);
      setCross(Y, Z, X);
      basisFromAxes(this._armT, X, Y, Z, S);
      multiply(out[side * 2], chassis, this._armT);
      // Forearm: +X from the elbow to the (reach-clamped) hand.
      X[0] = S[0] + U[0] * d - E[0]; X[1] = S[1] + U[1] * d - E[1]; X[2] = S[2] + U[2] * d - E[2];
      unit(X);
      setCross(Z, X, N); unit(Z);
      setCross(Y, Z, X);
      basisFromAxes(this._armT, X, Y, Z, E);
      multiply(out[side * 2 + 1], chassis, this._armT);
    }
  }

  /**
   * The lit face of the dash.
   *
   * Its own program and its own VAO, so it is one state change and six
   * vertices. The texture is only uploaded when the panel has actually been
   * redrawn -- `updateDashPanel` sets the flag -- because an upload from a
   * canvas is a GPU copy of a megabyte (plus its mip chain) and doing it on
   * a frame where nothing changed is pure waste.
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
      // Into the storage `makePanelTexture` allocated, not a texImage2D that
      // would reallocate it; then the mip chain, which the minified panel
      // is sampled from far more than level 0.
      gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, p.w, p.h, gl.RGBA, gl.UNSIGNED_BYTE, p.canvas);
      gl.generateMipmap(gl.TEXTURE_2D);
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

    // Unit 0 belongs to the near shadow cascade for every lit program; put
    // it back here, where the panel was borrowed, so no later pass (the sky,
    // the ghost, whatever is added next) has to remember to.
    gl.bindTexture(gl.TEXTURE_2D, this.shadow[0].tex);
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
      d[o + 4] = 1;   // a cone: gets the reflective collar
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
      d[o + 4] = 0;   // not a cone: no collar
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

/** Insert `#define` lines straight after the `#version` line, which must stay first. */
function withDefines(src, defs) {
  const nl = src.indexOf("\n");
  return src.slice(0, nl + 1) + defs + src.slice(nl + 1);
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
