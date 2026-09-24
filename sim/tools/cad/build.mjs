// Build the simulator's car.glb from the team's SolidWorks exports.
//
//   node build.mjs [errorFraction] [minPartM] [out]
//
// Suspension, wheels and mounts come from chassis_and_suspension.glb; the
// chassis and every aero part from aero_and_chassis.glb (its tubes carry the
// real steel finish). Everything is baked to world space (the sim's loader
// reads node translations only), hardware below `minPartM` across is dropped,
// the six parts SolidWorks mirrored through the ground plane are reflected
// back, colours are cleaned up (no textures: the loader reads a base colour
// per material), each part is simplified to a bounded shape error, and the
// wheel is emitted once, centred on its hub, under four wheel_* nodes.
import { NodeIO, Document } from "@gltf-transform/core";
import { getBounds } from "@gltf-transform/core";
import { weld, simplifyPrimitive, dedup, prune, join } from "@gltf-transform/functions";
import { MeshoptSimplifier } from "meshoptimizer";

const ERR = +(process.argv[2] ?? 0.0015);       // simplifier error, METRES (absolute, per part)
const MIN_PART = +(process.argv[3] ?? 0.03);     // drop parts smaller than this (m, bbox diagonal)
const OUT = process.argv[4] ?? "out/car.glb";
const io = new NodeIO();
await MeshoptSimplifier.ready;

const out = new Document();
const buffer = out.createBuffer();
const outScene = out.createScene("car");
const matCache = new Map();

// ---- colours ---------------------------------------------------------------
const CARBON = [0.045, 0.046, 0.05, 1];
function outColour(mat, fromAero) {
  if (!mat) return [0.55, 0.56, 0.6, 1];
  const name = (mat.getName() ?? "").toLowerCase();
  const c = mat.getBaseColorFactor();
  const textured = !!mat.getBaseColorTexture();
  if (name.includes("carbon") || textured) return CARBON;
  const [r, g, b] = c;
  const sat = Math.max(r, g, b) - Math.min(r, g, b);
  // SolidWorks' default blue-grey plastic and the loud engineering colour
  // codes (pure green, magenta, yellow...) are carbon on the real car.
  const swDefault = Math.abs(r - 0.6) < 0.03 && Math.abs(g - 0.65) < 0.03 && Math.abs(b - 0.86) < 0.03;
  if (fromAero && name.startsWith("defaultplastic")) return CARBON;
  if (swDefault || (name.startsWith("defaultplastic") && sat > 0.5)) return CARBON;
  return c;
}
const ALUMINIUM = [0.74, 0.76, 0.79, 1];
/** Machined parts the CAD left on SolidWorks' default plastic: aluminium,
 *  not carbon (the front rockers, the uprights, the hub spacers). */
const MACHINED = /Rocker|UPRT|WBS|Clevis|Shim|Spacer/i;
const isDefault = (mat) => !mat || /^defaultplastic/i.test(mat.getName() ?? "");
/** SolidWorks' untouched default appearance: the blue-grey plastic. */
const isSwDefault = (mat) => {
  if (!mat) return true;
  const [r, g, b] = mat.getBaseColorFactor();
  return isDefault(mat) && Math.abs(r - 0.6) < 0.03 && Math.abs(g - 0.65) < 0.03 && Math.abs(b - 0.86) < 0.03;
};
/** A part's name without the export's mirror/instance decoration, so a
 *  right-hand part can find its left-hand twin. */
const twinKey = (leaf) => leaf.split("/").pop().replace(/^(Mirror)+/, "").replace(/-\d+$/, "").replace(/(\D)\d$/, "$1");
const twinMats = new Map();
/** Left-hand parts with more than one finish: triangle centroids, for the
 *  mirror to take its finishes from triangle by triangle. */
const twinGeo = new Map();
const GRID = 0.006;
/** Left side of the car, every finished triangle, for right-side parts the
 *  export left unfinished (the right wishbones are separate parts from the
 *  left ones, not mirrors, and came out on the default appearance). */
const leftSide = { cents: [], mats: [], cells: new Map() };
function sidePush(x, y, z, mat) {
  const i = leftSide.mats.length;
  leftSide.cents.push(x, y, z); leftSide.mats.push(mat);
  const k = `${Math.floor(x / GRID)},${Math.floor(y / GRID)},${Math.floor(z / GRID)}`;
  (leftSide.cells.get(k) ?? leftSide.cells.set(k, []).get(k)).push(i);
}
function buildGrid(cents, mats) {
  const cells = new Map();
  for (let i = 0; i < mats.length; i++) {
    const k = `${Math.floor(cents[i * 3] / GRID)},${Math.floor(cents[i * 3 + 1] / GRID)},${Math.floor(cents[i * 3 + 2] / GRID)}`;
    (cells.get(k) ?? cells.set(k, []).get(k)).push(i);
  }
  return { cents, mats, cells };
}
function nearestMat(g, x, y, z) {
  const cx = Math.floor(x / GRID), cy = Math.floor(y / GRID), cz = Math.floor(z / GRID);
  for (let r = 1; r <= 4; r++) {
    let best = -1, bd = Infinity;
    for (let i = -r; i <= r; i++) for (let j = -r; j <= r; j++) for (let k = -r; k <= r; k++) {
      for (const t of g.cells.get(`${cx + i},${cy + j},${cz + k}`) ?? []) {
        const d = (g.cents[t * 3] - x) ** 2 + (g.cents[t * 3 + 1] - y) ** 2 + (g.cents[t * 3 + 2] - z) ** 2;
        if (d < bd) { bd = d; best = t; }
      }
    }
    if (best >= 0) return g.mats[best];
  }
  return null;
}
/** Finishes set by hand, over whatever the CAD says. */
const OVERRIDES = [
  // The frame is black tube on the car (the chassis and the engine-mount
  // tube sections), not the CAD's polished steel.
  { test: /Chassis FINAL|Chassis Engine Mounts/i, key: "chassis-black", colour: [0.03, 0.03, 0.033, 1], metal: 0.3, rough: 0.45, aero: true },
  // The springs are powdercoated black on the car.
  { test: /Spring/i, key: "spring-powder", colour: [0.035, 0.035, 0.04, 1], metal: 0.2, rough: 0.5 },
  // The firewall (upper, lower and access panels) is black on the car.
  { test: /^26-08-FW-(LFW|UFWL|LFWAP)-/i, key: "firewall-black", colour: [0.03, 0.03, 0.033, 1], metal: 0.2, rough: 0.5 },
  // Radiator cores and brackets: aluminium, at a grey the sim's base-colour
  // shading reads as metal rather than the CAD's near-white.
  // The exhaust is titanium: a grey with the blue it takes from the heat.
  { test: /Primar|Collector|Secondaries|Flange|^Clamp$|MUFFLER|Exh-Mount|Exhaust_port|T-Bolt Sleeve/i, file: /engine/,
    key: "exhaust-ti", colour: [0.13, 0.155, 0.22, 1], metal: 0.7, rough: 0.35 },
  { test: /BigMishiRad|Rad-Bracket/i, key: "radiator-al", colour: [0.2, 0.21, 0.225, 1], metal: 0.6, rough: 0.4, aero: true },
  // Wishbone tubes: the left ones are bare steel in the CAD, the right ones
  // black powdercoat; the car's are the left's.
  { test: /(^|-)(FS|RS)-(L|R)-(UCA|LCA)(-|$)|^(FS|RS)-(L|R)-(UCA|LCA)/i, key: "arm-steel", colour: [0.39, 0.35, 0.31, 1], metal: 0.6, rough: 0.4 },
];
/** SolidWorks gives metals near-white base colours and leaves the look to
 *  its reflections; the sim reads the base colour alone, so a sheet of
 *  aluminium (the firewall) or the titanium exhaust came out chalk white.
 *  For the engine bay and driver interface: metals down to a mid grey that
 *  keeps their hue, glass to a smoked grey, pure whites to an off-white. */
let tame = false;
let curFile = "";
function tamed(c, mat) {
  const name = (mat?.getName() ?? "").toLowerCase();
  const lum = 0.3 * c[0] + 0.59 * c[1] + 0.11 * c[2];
  if (/glass/.test(name)) return [0.22, 0.23, 0.25, 1];
  if (/steel|alumin|titanium|chrome|brass|gold/.test(name) && lum > 0.5) {
    // Keep a little of the hue (gold stays gold), lose the glare.
    // Linear values: the loader gamma-encodes, so 0.22 shows as ~0.5 grey.
    const k = (/gold|brass/.test(name) ? 0.32 : 0.22) / lum;
    const grey = [0.3, 0.59, 0.11].reduce((s, w, i) => s + w * c[i], 0);
    return [0, 1, 2].map((i) => Math.min(1, (grey + (c[i] - grey) * 0.35) * k)).concat(1);
  }
  if (lum > 0.8) return [0.45, 0.45, 0.45, 1];
  return c;
}
function material(mat, fromAero, leaf = "") {
  const seg = leaf.split("/").pop();
  for (const o of OVERRIDES) {
    if ((!fromAero || o.aero) && (!o.file || o.file.test(curFile)) && o.test.test(seg)) {
      if (!matCache.has(o.key)) matCache.set(o.key, out.createMaterial(o.key).setBaseColorFactor(o.colour).setMetallicFactor(o.metal).setRoughnessFactor(o.rough));
      return matCache.get(o.key);
    }
  }
  // The export drops the appearances of mirrored parts (the right-hand
  // damper came out plain): use the left-hand twin's.
  if (/^Mirror/.test(leaf.split("/").pop()) && isSwDefault(mat) && twinMats.has(twinKey(leaf))) mat = twinMats.get(twinKey(leaf));
  if (isDefault(mat) && MACHINED.test(leaf)) {
    const key = "machined";
    if (!matCache.has(key)) matCache.set(key, out.createMaterial("aluminium-metal").setBaseColorFactor(ALUMINIUM).setMetallicFactor(0.6).setRoughnessFactor(0.4));
    return matCache.get(key);
  }
  let c = outColour(mat, fromAero);
  const metal = /steel|alumin|gold|titanium|brass|chrome/.test((mat?.getName() ?? "").toLowerCase());
  if (tame) c = tamed(c, mat);
  c = c.map((v) => Math.round(v * 50) / 50);
  const key = c.join(",") + (metal ? "m" : "");
  if (!matCache.has(key)) {
    matCache.set(key, out.createMaterial(`c${matCache.size}${metal ? "-metal" : ""}`)
      .setBaseColorFactor(c).setMetallicFactor(metal ? 0.6 : 0).setRoughnessFactor(metal ? 0.35 : 0.55));
  }
  return matCache.get(key);
}

// ---- math ------------------------------------------------------------------
const xfPoint = (m, x, y, z) => [
  m[0] * x + m[4] * y + m[8] * z + m[12], m[1] * x + m[5] * y + m[9] * z + m[13], m[2] * x + m[6] * y + m[10] * z + m[14]];
const xfDir = (m, x, y, z) => [m[0] * x + m[4] * y + m[8] * z, m[1] * x + m[5] * y + m[9] * z, m[2] * x + m[6] * y + m[10] * z];
const det3 = (m) => m[0] * (m[5] * m[10] - m[9] * m[6]) - m[4] * (m[1] * m[10] - m[9] * m[2]) + m[8] * (m[1] * m[6] - m[5] * m[2]);

/** Copy one primitive into `out`, transformed to world space (optionally
 *  reflected through z=0), with a flat material and no UVs. */
function bakePrimitive(prim, m, reflectZ, mat, fromAero, leaf, reflectY = false) {
  const pos = prim.getAttribute("POSITION").getArray();
  const nrm = prim.getAttribute("NORMAL")?.getArray();
  const n = pos.length / 3;
  const P = new Float32Array(n * 3), N = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    let p = xfPoint(m, pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
    let d = nrm ? xfDir(m, nrm[i * 3], nrm[i * 3 + 1], nrm[i * 3 + 2]) : [0, 0, 1];
    if (reflectZ) { p[2] = -p[2]; d[2] = -d[2]; }
    if (reflectY) { p[1] = -p[1]; d[1] = -d[1]; }
    const l = Math.hypot(...d) || 1;
    P.set(p, i * 3); N.set([d[0] / l, d[1] / l, d[2] / l], i * 3);
  }
  let idx = prim.getIndices()?.getArray();
  idx = idx ? Uint32Array.from(idx) : Uint32Array.from({ length: n }, (_, i) => i);
  // A mirroring transform (negative determinant), or our own reflection, turns
  // the triangles inside out: flip the winding back.
  const flips = ((det3(m) < 0) !== reflectZ) !== reflectY;
  if (flips) for (let i = 0; i < idx.length; i += 3) { const t = idx[i + 1]; idx[i + 1] = idx[i + 2]; idx[i + 2] = t; }
  const p = out.createPrimitive()
    .setAttribute("POSITION", out.createAccessor().setType("VEC3").setArray(P).setBuffer(buffer))
    .setAttribute("NORMAL", out.createAccessor().setType("VEC3").setArray(N).setBuffer(buffer))
    .setIndices(out.createAccessor().setType("SCALAR").setArray(idx).setBuffer(buffer))
    .setMaterial(material(mat, fromAero, leaf));
  return p;
}

function worldBounds(prims, m) {
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const pr of prims) {
    const a = pr.getAttribute("POSITION");
    const mn = a.getMin([]), mx = a.getMax([]);
    for (const x of [mn[0], mx[0]]) for (const y of [mn[1], mx[1]]) for (const z of [mn[2], mx[2]]) {
      const w = xfPoint(m, x, y, z);
      for (let i = 0; i < 3; i++) { lo[i] = Math.min(lo[i], w[i]); hi[i] = Math.max(hi[i], w[i]); }
    }
  }
  return { lo, hi };
}

const report = { kept: 0, dropped: 0, droppedTris: 0, reflected: [], dupes: 0, rig: {} };

// ---- suspension rig ------------------------------------------------------------
// The team's OptimumK hardpoints (inches; x forward from the front axle, y
// left, z up -- the CAD's own frame). Left side; the right is mirrored in y.
const TEAM = JSON.parse((await import("node:fs")).readFileSync(
  (process.env.USERPROFILE ?? process.env.HOME) + "/sdm26-assetto-corsa/data/sdm26_team_data.json", "utf8")).hardpoints;
const HP_KEYS = { UF: "CHAS_UppFor", UA: "CHAS_UppAft", LF: "CHAS_LowFor", LA: "CHAS_LowAft", UB: "UPRI_UppPnt", LB: "UPRI_LowPnt",
  TC: "CHAS_TiePnt", TU: "UPRI_TiePnt", PP: "NSMA_PPAttPnt", ATT: "CHAS_AttPnt", PIV: "CHAS_RocPiv", ROD: "ROCK_RodPnt",
  COI: "ROCK_CoiPnt", WC: "wheel_centre" };
const hardpoints = {};
for (const corner of ["fl", "fr", "rl", "rr"]) {
  const ax = corner[0] === "f" ? "front" : "rear";
  const sy = corner[1] === "l" ? 1 : -1;
  const hp = {};
  for (const [k, name] of Object.entries(HP_KEYS)) {
    const v = TEAM[ax][name];
    hp[k] = [v[0] * 0.0254, sy * v[1] * 0.0254, v[2] * 0.0254];
  }
  hp.pushOn = /upper/i.test(TEAM[ax].pushrod_on ?? "") ? "uca" : "lca";
  hardpoints[corner] = hp;
}
/** Which moving part of which corner a suspension part is, or null (body). */
function rigRole(path, c) {
  if (!/Suspension Assembly|ARB-ASSY/.test(path)) return null;
  const corner = (c[0] > -0.75 ? "f" : "r") + (c[1] > 0 ? "l" : "r");
  let role = null;
  if (/TTX25|250Spring/i.test(path)) {
    role = /Spring/i.test(path) ? "spring" : /PistonArm|BottomBall/i.test(path) ? "damper_rod" : "damper_top";
  } else if (/FS-ARB-ASSY.*(DL-Tube|Rod End)/i.test(path) && c[0] > -0.75) role = "droplink";
  else if (/ARB/i.test(path)) return null;
  else if (/Rocker|Spacer-RockerShock/i.test(path)) role = "rocker";
  else if (/PRSA/i.test(path)) role = "pushrod";
  else if (/TLSA/i.test(path)) role = "tie";
  else if (/UCA/i.test(path)) role = "uca";
  else if (/LCA/i.test(path)) role = "lca";
  else if (/UPRT|WBS/i.test(path)) role = "upright";
  else {
    // Anything else hanging at the hub (bearings, the clevis and shims at the
    // upper ball joint) goes with the upright.
    const wc = hardpoints[corner].WC;
    if (Math.hypot(c[0] - wc[0], c[1] - wc[1], c[2] - wc[2]) < 0.13) role = "upright";
  }
  return role ? { corner, role } : null;
}
const rigMeshes = new Map(); // "rig:fl:uca" -> Mesh
// ---- driver controls that move ---------------------------------------------
// Each turns about one pivot axis (CAD +y, across the car), measured off the
// bearings in the driver-interface assembly; a positive angle moves the top
// forward, which is the foot (or hand) pushing. `curve` maps the driver's
// input (0..1) to degrees.
//  brake     pivot on the needle-roller/thrust bearings; the master cylinders
//            ride on the arm (their pushrods anchor at the base). No SDM26
//            stroke is on record: SDM25's pedal-effort run (Pedal Effort
//            Calculator, Run 1) went 1-10 deg for 2-173 lbf and flattened past
//            9, so the input -- the driver's load -- is mapped through that.
//  throttle  on its TPS shaft, 0 to 15 deg stop to stop (SDM26 Systems Design
//            Report 5.3.4.1: "limited to +0 degrees to -15 degrees").
//  clutch    the hand lever, on its two bearings at the base: 2.6 in of
//            travel at the grip (Systems Design Report 5.2.4), 366 mm above
//            the pivot -> 10.3 deg.
const SDM25_EFFORT = [0, 2.02, 3.96, 10.32, 22.40, 51.53, 90.44, 122.58, 149.68, 172.73];
const CONTROLS = {
  brake: { pivot: [0.419, 0.070, 0.213], axis: [0, 1, 0],
    curve: SDM25_EFFORT.map((f, deg) => [+(f / 172.73).toFixed(4), deg]) },
  throttle: { pivot: [0.424, -0.070, 0.210], axis: [0, 1, 0], curve: [[0, 0], [1, 15]] },
  clutch: { pivot: [-0.172, -0.204, 0.138], axis: [0, 1, 0], curve: [[0, 0], [1, 10.3]] },
};
/** Which control a driver-interface part moves with, or null. Above the
 *  pivot height it is on the arm; below, it is the base, mounts and bearings. */
function controlOf(path, c) {
  if (/26-08-BPAS-ASM/.test(path) && c[2] > 0.30) return "brake";
  if (/PB-TPM-ASM/.test(path) && c[2] > 0.30) return "throttle";
  if (/26-08-DACH-ASM/.test(path) && c[2] > 0.25) return "clutch";
  return null;
}
const ctlMeshes = new Map();
const dropBounds = {};       // corner -> {lo, hi} of the front ARB drop link
const seen = new Set();
const wheelParts = []; // meshes of the FL wheel (rim + tyre), world space
let bodyMeshes = [];

/** McMaster/COTS hardware and fasteners: none of it goes in the car. */
const HARDWARE = /\d{5}A\d{2,4}|Screw|Insert|Locknut|Nut|weld nuts|Washer|Heat-Set|Rivet|Dzus|DZUS|Bolt|HRDW|HWTB|6804ZZ|REDSPC/i;
/** Where the team's assembly puts the moving wheel and the dash (CAD frame),
 *  read off the driver-interface export. */
const cockpit = {};

/** Meshes simplified harder: the engine bay, behind the firewall and under
 *  the engine cover, is seen from a chase camera metres away at best. */
const coarse = new WeakSet();
// ---- livery -------------------------------------------------------------------
// The painted bodywork: nose, side panels, cowl, the small body wings, and
// both wings with their endplates. These carry TEXCOORD_0 (see `unwrap`);
// everything else is untextured.
const LIVERY = /STRU-Hood|STRU-Body-Panels|STRU-Cowl|Aero-Body-Wing|Endplate|Rear-Wing-E_1|Rear-Wing-018-E\d-Shell|FW-013-E\d/i;
const livery = new WeakSet();
async function ingest(file, { fromAero, skip, finishes = true, hardware = true, mounts = false, errScale = 1 }) {
  const src = await io.read(file);
  tame = !finishes;
  curFile = file;
  if (mounts) {
    src.getRoot().listScenes()[0].traverse((n) => {
      const nm = n.getName() ?? "";
      // The wheel assembly's own frame is the frame steering-wheel.glb is in
      // (+y up the wheel, +z away from the driver, origin on the column axis).
      if (/^26-08-SW-SWMA-ASM-U V4$/.test(nm)) cockpit.steer = Array.from(n.getWorldMatrix());
      // The Strada's part frame is the STEP's, which dash.glb is built from.
      if (/AIMXSStrada1\.2-1$/.test(nm)) cockpit.dash = Array.from(n.getWorldMatrix());
    });
  }
  // First pass: each left-hand part's dominant material, for its mirror.
  src.getRoot().listScenes()[0].traverse((node) => {
    const leaf = node.getName() ?? "";
    if (!node.getMesh() || /^Mirror/.test(leaf.split("/").pop())) return;
    let best = null, n = -1;
    for (const pr of node.getMesh().listPrimitives()) {
      // The part's signature finish: size, weighted hard toward colour (the
      // TTX body is gold AND satin; gold is what it looks like).
      const f = pr.getMaterial()?.getBaseColorFactor() ?? [0.5, 0.5, 0.5];
      const sat = Math.max(f[0], f[1], f[2]) - Math.min(f[0], f[1], f[2]);
      const c = (pr.getIndices()?.getCount() ?? 0) * (1 + 6 * sat);
      if (c > n && !isSwDefault(pr.getMaterial())) { n = c; best = pr.getMaterial(); }
    }
    if (best && !twinMats.has(twinKey(leaf))) twinMats.set(twinKey(leaf), best);
    if (finishes) {
      const m = node.getWorldMatrix();
      for (const pr of node.getMesh().listPrimitives()) {
        if (isSwDefault(pr.getMaterial())) continue;
        const P = pr.getAttribute("POSITION").getArray(), I = pr.getIndices()?.getArray();
        if (!I) continue;
        // Every 2nd triangle is plenty to find a finish; halves the memory.
        for (let t = 0; t < I.length; t += 6) {
          let cx = 0, cy = 0, cz = 0;
          for (let k = 0; k < 3; k++) { const w = xfPoint(m, P[I[t + k] * 3], P[I[t + k] * 3 + 1], P[I[t + k] * 3 + 2]); cx += w[0]; cy += w[1]; cz += w[2]; }
          if (cy / 3 > 0.02) sidePush(cx / 3, cy / 3, cz / 3, pr.getMaterial());
        }
      }
    }
    const mats = new Set(node.getMesh().listPrimitives().map((pr) => pr.getMaterial()));
    if (mats.size > 1) {
      const m = node.getWorldMatrix(), cents = [], tmats = [];
      for (const pr of node.getMesh().listPrimitives()) {
        const P = pr.getAttribute("POSITION").getArray(), I = pr.getIndices()?.getArray();
        if (!I) continue;
        for (let t = 0; t < I.length; t += 3) {
          let cx = 0, cy = 0, cz = 0;
          for (let k = 0; k < 3; k++) { const w = xfPoint(m, P[I[t + k] * 3], P[I[t + k] * 3 + 1], P[I[t + k] * 3 + 2]); cx += w[0]; cy += w[1]; cz += w[2]; }
          cents.push(cx / 3, cy / 3, cz / 3); tmats.push(pr.getMaterial());
        }
      }
      // Keyed by axle too: the front and rear dampers share part names.
      const ax = cents[0] > -0.75 ? "f" : "r";
      if (!twinGeo.has(twinKey(leaf) + ax)) twinGeo.set(twinKey(leaf) + ax, buildGrid(cents, tmats));
    }
  });
  src.getRoot().listScenes()[0].traverse((node) => {
    const mesh = node.getMesh();
    if (!mesh) return;
    const path = []; for (let q = node; q; q = q.getParentNode()) path.unshift(q.getName() ?? "");
    const name = path.join(" > ");
    const m = node.getWorldMatrix();
    const prims = mesh.listPrimitives();
    const tris = prims.reduce((s, p) => s + (p.getIndices()?.getCount() ?? p.getAttribute("POSITION").getCount()) / 3, 0);
    if (skip && skip.test(name)) { report.dropped++; report.droppedTris += tris; return; }
    if (hardware && HARDWARE.test(path[path.length - 1])) { report.hardware = (report.hardware ?? 0) + 1; report.droppedTris += tris; return; }
    const { lo, hi } = worldBounds(prims, m);
    const diag = Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]);
    if (diag < MIN_PART) { report.dropped++; report.droppedTris += tris; return; }
    // Duplicates: same part at the same place (the export repeats a few).
    const key = `${Math.round(tris)}:${lo.map((v) => v.toFixed(3))}:${hi.map((v) => v.toFixed(3))}`;
    if (seen.has(key)) { report.dupes++; return; }
    seen.add(key);
    // Mirrored through the ground by the export: its twin is the same height up.
    const reflectZ = hi[2] < 0.02 && lo[2] < -0.05;
    if (reflectZ) report.reflected.push(path[path.length - 1]);
    const leaf = path[path.length - 1];
    // The side body wings: the export put the left one 180 mm aft of where
    // its ribs are (x -0.090 vs the ribs' +0.094); the right-hand (mirrored)
    // one is where the ribs are. Build the left as the right's mirror image.
    if (fromAero && /^SDM26-Aero-Body-Wing/.test(leaf)) { report.dropped++; return; }
    if (fromAero && /^MirrorSDM26-Aero-Body-Wing/.test(leaf)) {
      const twin = out.createMesh(leaf + "-mirrored");
      for (const pr of prims) if (pr.getMode() === 4) twin.addPrimitive(bakePrimitive(pr, m, reflectZ, pr.getMaterial(), fromAero, leaf, true));
      bodyMeshes.push(twin);
      livery.add(twin);
      report.bodyWingMirrored = true;
    }
    // ...and the right wing is missing its outboard rib (the left has ribs
    // at y +0.239 and +0.398; the right only at -0.239): mirror it across.
    {
      const cx = (lo[0] + hi[0]) / 2, cy = (lo[1] + hi[1]) / 2, cz = (lo[2] + hi[2]) / 2;
      if (fromAero && /ExternalRib/.test(leaf) && Math.abs(cx - 0.094) < 0.01 && Math.abs(cz - 0.446) < 0.01 && cy > 0.35) {
        const twin = out.createMesh(leaf + "-mirrored");
        for (const pr of prims) if (pr.getMode() === 4) twin.addPrimitive(bakePrimitive(pr, m, reflectZ, pr.getMaterial(), fromAero, leaf, true));
        bodyMeshes.push(twin);
        report.bodyWingRibMirrored = true;
      }
    }
    const isWheel = /OZ_Formula|Hoosier/i.test(leaf);
    const cc = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2];
    if (reflectZ) cc[2] = -cc[2];
    const rig = !fromAero && !isWheel ? rigRole(name, cc) : null;
    const ctl = mounts ? controlOf(name, cc) : null;
    const outMesh = out.createMesh(leaf);
    if (errScale > 1) coarse.add(outMesh);
    if (fromAero && LIVERY.test(leaf)) livery.add(outMesh);
    const tg = (/^Mirror/.test(leaf.split("/").pop())
      ? twinGeo.get(twinKey(leaf) + ((lo[0] + hi[0]) / 2 > -0.75 ? "f" : "r")) : null)
      ?? (!fromAero && finishes && (lo[1] + hi[1]) / 2 < -0.02 ? leftSide : null);
    for (const pr of prims) {
      if (pr.getMode() !== 4) continue; // triangles only
      if (tg && isSwDefault(pr.getMaterial()) && pr.getIndices()) {
        const P = pr.getAttribute("POSITION").getArray(), I = pr.getIndices().getArray();
        const groups = new Map();
        for (let t = 0; t < I.length; t += 3) {
          let cx = 0, cy = 0, cz = 0;
          for (let k = 0; k < 3; k++) { const w = xfPoint(m, P[I[t + k] * 3], P[I[t + k] * 3 + 1], P[I[t + k] * 3 + 2]); cx += w[0]; cy += w[1]; cz += w[2]; }
          const mt = nearestMat(tg, cx / 3, -cy / 3, cz / 3) ?? pr.getMaterial();
          (groups.get(mt) ?? groups.set(mt, []).get(mt)).push(I[t], I[t + 1], I[t + 2]);
        }
        for (const [mt, idx] of groups) {
          const sub = src.createPrimitive().setAttribute("POSITION", pr.getAttribute("POSITION"))
            .setIndices(src.createAccessor().setType("SCALAR").setArray(new Uint32Array(idx)));
          if (pr.getAttribute("NORMAL")) sub.setAttribute("NORMAL", pr.getAttribute("NORMAL"));
          outMesh.addPrimitive(bakePrimitive(sub, m, reflectZ, mt, fromAero, leaf));
        }
        report.splitFinish = (report.splitFinish ?? 0) + 1;
        continue;
      }
      outMesh.addPrimitive(bakePrimitive(pr, m, reflectZ, pr.getMaterial(), fromAero, leaf));
    }
    report.kept++;
    if (ctl) {
      const key = `ctl:${ctl}`;
      if (!ctlMeshes.has(key)) ctlMeshes.set(key, out.createMesh(key));
      for (const pr of outMesh.listPrimitives()) ctlMeshes.get(key).addPrimitive(pr);
      outMesh.dispose();
      report.ctl = report.ctl ?? {}; report.ctl[key] = (report.ctl[key] ?? 0) + 1;
    } else if (isWheel) {
      const c = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2];
      wheelParts.push({ mesh: outMesh, centre: c, leaf });
    } else if (rig) {
      const key = `rig:${rig.corner}:${rig.role}`;
      if (!rigMeshes.has(key)) rigMeshes.set(key, out.createMesh(key));
      for (const pr of outMesh.listPrimitives()) rigMeshes.get(key).addPrimitive(pr);
      outMesh.dispose();
      report.rig[key] = (report.rig[key] ?? 0) + 1;
      if (rig.role === "droplink") {
        const b = (dropBounds[rig.corner] ??= { lo: [9, 9, 9], hi: [-9, -9, -9] });
        for (let i = 0; i < 3; i++) { b.lo[i] = Math.min(b.lo[i], lo[i]); b.hi[i] = Math.max(b.hi[i], hi[i]); }
      }
    } else {
      bodyMeshes.push(outMesh);
    }
  });
}

await ingest("raw/chassis_and_suspension.glb", { fromAero: false, skip: /SDM26 Chassis FINAL/ });
await ingest("raw/aero_and_chassis.glb", { fromAero: true, skip: /Knob-Grip|Rad-Reference/ });
// The engine bay. Not the chassis or its tab assembly (already in, and the
// tab assembly's copies sit under the ground), and not the two stand-ins for
// masses (OilMass, fortniteballs).
const REPEATS = /SDM26 Chassis FINAL|26-03-TAB-ASSEMBLY|Suspension Tabs as a Part|Lower Diff Mounts|Rear_Slug|Chassis Engine Mounts|^sdm26-master-assembly > IAP-1$|CoolantFIllNeck/;
await ingest("raw/chassis_and_engine.glb", { fromAero: false, finishes: false, errScale: 2,
  skip: new RegExp(REPEATS.source.replace("|CoolantFIllNeck", "") + "|OilMass|fortniteballs") });
// The driver interface: pedals, floor, seat, head restraint, firewall,
// column, rack and the dash panel. Not the wheel or the Strada: those are the
// sim's own moving wheel and live dash, placed where this file puts them. And
// not one mirrored rack mount the export threw 0.5 m ahead of the car.
await ingest("raw/chassis_and_driver_interface.glb", { fromAero: false, finishes: false, mounts: true,
  skip: new RegExp(REPEATS.source + "|SW-SWMA-ASM|AIMXSStrada|Mirror26-08-SS-SASRL-PT-R1-3") });
// dash.glb is the STEP moved so the display centre is its origin (build_dash.mjs:
// y -0.0482, z +0.0008): undo that here, so the matrix takes dash.glb straight in.
if (cockpit.dash) {
  const m = cockpit.dash, oy = 0.0482, oz = -0.0008;
  for (let i = 0; i < 3; i++) m[12 + i] += m[4 + i] * oy + m[8 + i] * oz;
}
if (!cockpit.steer || !cockpit.dash) throw new Error("no wheel/dash mount in the driver-interface export");

// ---- wheels: keep one corner's rim + tyre, centred on its hub ----------------
const hubsOf = (sx, sy) => wheelParts.filter((w) => Math.sign(w.centre[0] + 0.7) === sx && Math.sign(w.centre[1]) === sy);
const corners = { wheel_fl: [1, 1], wheel_fr: [1, -1], wheel_rl: [-1, 1], wheel_rr: [-1, -1] };
const hub = {};
for (const [n, [sx, sy]] of Object.entries(corners)) {
  const parts = hubsOf(sx, sy);
  const c = [0, 1, 2].map((i) => parts.reduce((s, w) => s + w.centre[i], 0) / parts.length);
  hub[n] = { c, parts };
}
// One wheel's geometry, re-centred on its hub.
const flParts = hub.wheel_fl.parts;
const wheelMesh = out.createMesh("wheel");
for (const w of flParts) {
  for (const pr of w.mesh.listPrimitives()) {
    const a = pr.getAttribute("POSITION").getArray();
    const c = hub.wheel_fl.c;
    for (let i = 0; i < a.length; i += 3) { a[i] -= c[0]; a[i + 1] -= c[1]; a[i + 2] -= c[2]; }
    wheelMesh.addPrimitive(pr);
  }
}
for (const [n, { c }] of Object.entries(hub)) outScene.addChild(out.createNode(n).setMesh(wheelMesh).setTranslation(c));
for (const w of wheelParts) w.mesh.dispose();

// ---- body -------------------------------------------------------------------
for (const m of bodyMeshes) outScene.addChild(out.createNode(m.getName()).setMesh(m));
// ---- the rig: moving parts, one node each, and the hardpoints they move on --
for (const [key, m] of rigMeshes) outScene.addChild(out.createNode(key).setMesh(m));
for (const [key, m] of ctlMeshes) outScene.addChild(out.createNode(key).setMesh(m));
for (const [corner, b] of Object.entries(dropBounds)) {
  const cx = (b.lo[0] + b.hi[0]) / 2, cy = (b.lo[1] + b.hi[1]) / 2;
  hardpoints[corner].DLB = [cx, cy, b.lo[2] + 0.008];
  hardpoints[corner].DLT = [cx, cy, b.hi[2] - 0.008];
}
outScene.setExtras({
  suspension: { frame: "cad: x forward from the front axle, y left, z up; metres", corners: hardpoints },
  // Column-major 4x4s, CAD frame: steering-wheel.glb's frame and dash.glb's
  // (less the display offset build_dash.mjs applies) in the car.
  cockpit,
  controls: CONTROLS,
});

// ---- reduce -------------------------------------------------------------------
const trisOf = () => out.getRoot().listMeshes().reduce((s, m) => s + m.listPrimitives().reduce((t, p) => t + p.getIndices().getCount() / 3, 0), 0);
const before = trisOf();
await out.transform(weld());
// An absolute error per part: meshopt's error is a fraction of the part's own
// size, so one relative figure crushed the 25 mm chassis tubes (the chassis is
// a 1.2 m part) while barely touching a 3 cm bracket.
for (const mesh of out.getRoot().listMeshes()) {
  const isWheel = mesh.getName() === "wheel";
  for (const prim of mesh.listPrimitives()) {
    const a = prim.getAttribute("POSITION");
    const mn = a.getMin([]), mx = a.getMax([]);
    const radius = Math.max(1e-6, Math.hypot(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]) / 2);
    const abs = isWheel ? ERR * 0.6 : coarse.has(mesh) ? ERR * 2 : ERR;
    simplifyPrimitive(prim, { simplifier: MeshoptSimplifier, ratio: 0, error: Math.min(0.05, abs / radius), lockBorder: false });
  }
}
await out.transform(dedup(), prune());
const afterSimplify = trisOf();

// ---- the livery UV map --------------------------------------------------------
// A real unwrap, one scale for everything, every visible surface in its own
// place (nothing stacked, nothing mirrored):
//
//  - BODY SKIN: nose, side panels and cowl as one piece. Across = along the
//    car, nose left; down = round it, measured along the surface, from the
//    right side (upside down) over the top centreline to the left side
//    (upright); cut along the underside. A stripe drawn straight down runs
//    over the car unbroken.
//  - WING ELEMENTS: every element of the front wing, the rear wing and the
//    small body wings is its own pair of pieces, UPPER and LOWER surface,
//    each flattened onto that element's own best-fit plane (a cambered flap
//    is unrolled, not seen from overhead where its neighbours would cover
//    it). Upper: nose left, the car's left side down. Lower: as seen from
//    underneath, nose left, the car's left side up.
//  - ENDPLATES: outer faces only, seen from the side they face.
//
// Faces that look INTO the car (inner skins, an endplate's inboard face)
// get no livery (-1, -1) so nothing shows mirrored. Image v runs down (row
// 0 is v = 0, as WebGL samples an unflipped image).
const liveryMeshes = out.getRoot().listMeshes().filter((m) => livery.has(m));
function liveryKind(name) {
  if (/STRU-Hood|STRU-Body-Panels|STRU-Cowl/i.test(name)) return "body";
  if (/Endplate/i.test(name)) return "endplate";
  return "element";   // a wing element: front or rear wing, or a small body wing
}
/** A readable name for a wing element, from its CAD part. */
function elementName(name, c) {
  const leaf = name.replace(/-mirrored$/, "");
  if (/Aero-Body-Wing/i.test(leaf)) return `body wing ${c[1] >= 0 ? "left" : "right"}`;
  const fw = /FW-013-(E\d)(?:-FINAL)?(?:-Radius)?(?:-thickness)?-?(Upper|Lower)?/i.exec(leaf);
  if (fw) return `front wing ${fw[1]}${fw[2] ? (fw[2].toLowerCase() === "upper" ? "a" : "b") : ""}`;
  const rw = /Rear-Wing-(?:018-)?(E_?\d)/i.exec(leaf);
  if (rw) return `rear wing ${rw[1].replace("_", "")}`;
  return leaf.slice(0, 24);
}
const triCentroid = (q) => [0, 1, 2].map((a) => (q[0][a] + q[1][a] + q[2][a]) / 3);
const cross3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const unit = (a) => { const l = Math.hypot(...a) || 1; return a.map((x) => x / l); };
/** The face's outward normal: its winding, turned to agree with the exported
 *  vertex normals where they disagree. */
function faceNormal(q, N, v) {
  const e1 = [q[1][0] - q[0][0], q[1][1] - q[0][1], q[1][2] - q[0][2]];
  const e2 = [q[2][0] - q[0][0], q[2][1] - q[0][1], q[2][2] - q[0][2]];
  let fn = cross3(e1, e2);
  if (N) {
    const mn = [0, 1, 2].map((ax) => (N[v[0] * 3 + ax] + N[v[1] * 3 + ax] + N[v[2] * 3 + ax]) / 3);
    if (dot3(mn, fn) < 0) fn = fn.map((x) => -x);
  }
  return fn;
}
const forEachTri = (fn) => {
  for (const mesh of liveryMeshes) {
    const kind = liveryKind(mesh.getName());
    for (const prim of mesh.listPrimitives()) {
      const P = prim.getAttribute("POSITION").getArray(), N = prim.getAttribute("NORMAL")?.getArray();
      const I = prim.getIndices().getArray();
      for (let t = 0; t < I.length; t += 3) {
        const v = [I[t], I[t + 1], I[t + 2]];
        const q = v.map((i) => [P[i * 3], P[i * 3 + 1], P[i * 3 + 2]]);
        fn(kind, mesh, prim, t / 3, q, faceNormal(q, N, v));
      }
    }
  }
};

// -- the body skin: its outline, station by station -----------------------------
const bodyPts = [];
for (const mesh of liveryMeshes) if (liveryKind(mesh.getName()) === "body") for (const prim of mesh.listPrimitives()) {
  const P = prim.getAttribute("POSITION").getArray();
  for (let i = 0; i < P.length; i += 3) bodyPts.push([P[i], P[i + 1], P[i + 2]]);
}
let BX0 = Infinity, BX1 = -Infinity;
for (const p of bodyPts) { BX0 = Math.min(BX0, p[0]); BX1 = Math.max(BX1, p[0]); }
const DX = 0.02, NS = Math.ceil((BX1 - BX0) / DX) + 1, NB = 180;
const slices = Array.from({ length: NS }, () => ({ zlo: Infinity, zhi: -Infinity, pts: [] }));
for (const p of bodyPts) {
  const i = Math.min(NS - 1, Math.max(0, Math.round((p[0] - BX0) / DX)));
  for (const j of [i - 1, i, i + 1]) if (j >= 0 && j < NS) { const sl = slices[j]; sl.pts.push(p); sl.zlo = Math.min(sl.zlo, p[2]); sl.zhi = Math.max(sl.zhi, p[2]); }
}
const phiOf = (y, dz) => Math.atan2(y, dz);
const binOf = (phi) => Math.min(NB - 1, Math.max(0, Math.floor(((phi + Math.PI) / (2 * Math.PI)) * NB)));
for (const sl of slices) {
  sl.zc = Number.isFinite(sl.zlo) ? (sl.zlo + sl.zhi) / 2 : NaN;
  if (!sl.pts.length) continue;
  const r = new Float64Array(NB).fill(NaN);
  for (const p of sl.pts) { const b = binOf(phiOf(p[1], p[2] - sl.zc)); const rr = Math.hypot(p[1], p[2] - sl.zc); if (!(r[b] >= rr)) r[b] = rr; }
  const known = []; for (let b = 0; b < NB; b++) if (Number.isFinite(r[b])) known.push(b);
  for (let b = 0; b < NB; b++) if (!Number.isFinite(r[b])) {
    let a = known[0], c = known[0], da = 1e9, dc = 1e9;
    for (const k of known) { const f = (b - k + NB) % NB, g = (k - b + NB) % NB; if (f < da) { da = f; a = k; } if (g < dc) { dc = g; c = k; } }
    r[b] = (r[a] * dc + r[c] * da) / (da + dc);
  }
  sl.r = Float64Array.from(r, (_, b) => (r[(b + NB - 1) % NB] + 2 * r[b] + r[(b + 1) % NB]) / 4);
}
const dphi = (2 * Math.PI) / NB;
for (const sl of slices) {
  if (!sl.r) continue;
  const S = new Float64Array(NB + 1), k0 = NB / 2;
  for (let k = k0; k < NB; k++) { const a = sl.r[k], b = sl.r[(k + 1) % NB]; S[k + 1] = S[k] + Math.hypot(((a + b) / 2) * dphi, b - a); }
  for (let k = k0; k > 0; k--) { const a = sl.r[k - 1], b = sl.r[k % NB]; S[k - 1] = S[k] - Math.hypot(((a + b) / 2) * dphi, b - a); }
  sl.S = S;
}
for (let i = 0; i < NS; i++) {
  if (!slices[i].S) continue;
  const acc = new Float64Array(NB + 1); let w = 0, zc = 0;
  for (let j = i - 2; j <= i + 2; j++) if (j >= 0 && j < NS && slices[j].S) { for (let k = 0; k <= NB; k++) acc[k] += slices[j].S[k]; zc += slices[j].zc; w++; }
  slices[i].Ss = acc.map((v) => v / w); slices[i].zcs = zc / w;
}
for (let i = 0; i < NS; i++) if (!slices[i].Ss) {
  let j = 1; while (!(slices[i - j]?.Ss || slices[i + j]?.Ss)) j++;
  const n = slices[i - j]?.Ss ? slices[i - j] : slices[i + j];
  slices[i].Ss = n.Ss; slices[i].zcs = n.zcs;
}
function arcAt(x, y, z, phiHint) {
  const f = Math.min(NS - 1, Math.max(0, (x - BX0) / DX)), i = Math.floor(f), t = f - i, j = Math.min(NS - 1, i + 1);
  const zc = slices[i].zcs * (1 - t) + slices[j].zcs * t;
  let phi = phiOf(y, z - zc);
  if (phiHint != null && Math.abs(phi - phiHint) > Math.PI) phi += phi < 0 ? 2 * Math.PI : -2 * Math.PI;
  const q = (phi + Math.PI) / dphi, k = Math.floor(q), a = q - k;
  const look = (S) => { if (k < 0) return S[0] + (S[1] - S[0]) * q; if (k >= NB) return S[NB] + (S[NB] - S[NB - 1]) * (q - NB); return S[k] * (1 - a) + S[k + 1] * a; };
  return { s: look(slices[i].Ss) * (1 - t) + look(slices[j].Ss) * t, phi, zc };
}
let sMin = Infinity, sMax = -Infinity;
for (const sl of slices) { sMin = Math.min(sMin, sl.Ss[0]); sMax = Math.max(sMax, sl.Ss[NB]); }

// -- classify every face into a piece -------------------------------------------
// Element planes: least-squares plane of each element's vertices (PCA), normal
// turned to point up; span axis = the car's y projected into the plane, chord
// axis = normal x span (pointing forward).
const elementPlanes = new Map();
for (const mesh of liveryMeshes) {
  if (liveryKind(mesh.getName()) !== "element") continue;
  const pts = [];
  for (const prim of mesh.listPrimitives()) { const P = prim.getAttribute("POSITION").getArray(); for (let i = 0; i < P.length; i += 3) pts.push([P[i], P[i + 1], P[i + 2]]); }
  const c = [0, 1, 2].map((a) => pts.reduce((s, p) => s + p[a], 0) / pts.length);
  const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (const p of pts) { const d = [p[0] - c[0], p[1] - c[1], p[2] - c[2]]; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[i][j] += d[i] * d[j]; }
  // Smallest eigenvector by inverse power iteration on C + eps I.
  const solve = (M, b) => { // 3x3 Cramer
    const det = (m) => m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    const D = det(M) || 1e-30;
    return [0, 1, 2].map((k) => det(M.map((row, i) => row.map((v, j) => (j === k ? b[i] : v)))) / D);
  };
  const tr = C[0][0] + C[1][1] + C[2][2];
  const M = C.map((row, i) => row.map((v, j) => v + (i === j ? 1e-9 * tr : 0)));
  let n = [0.1, 0.2, 1];
  for (let it = 0; it < 30; it++) n = unit(solve(M, n));
  if (n[2] < 0) n = n.map((x) => -x);
  const span = unit([0 - n[1] * n[0], 1 - n[1] * n[1], 0 - n[1] * n[2]]);   // y minus its normal component
  const chord = unit(cross3(span, n));                                     // forward-ish
  const fwd = chord[0] >= 0 ? chord : chord.map((x) => -x);
  elementPlanes.set(mesh, { c, n, span, fwd });
}
// A face is on the OUTSIDE only if a ray from it, along its normal, leaves
// its own part (for the body: all the body panels) without hitting it again.
// That drops the inner skin of a hollow wing element or a thick panel, and a
// panel tucked under its neighbour, which would otherwise stack on the same
// paint as the face that covers it.
const groupTris = new Map();
const groupOf = (kind, mesh) => (kind === "body" ? "body" : mesh);
forEachTri((kind, mesh, prim, t, q) => {
  const g = groupOf(kind, mesh);
  if (!groupTris.has(g)) groupTris.set(g, []);
  groupTris.get(g).push(q);
});
// Only near hits count: an inner skin faces the opposite skin a wall's
// thickness away; a bracket or a curl metres off doesn't hide anything.
const RAY_MAX = +(process.env.RAY_MAX ?? 0.06);
function rayHits(g, o, d) {
  let best = Infinity;
  for (const q of groupTris.get(g)) {
    const e1 = [q[1][0] - q[0][0], q[1][1] - q[0][1], q[1][2] - q[0][2]], e2 = [q[2][0] - q[0][0], q[2][1] - q[0][1], q[2][2] - q[0][2]];
    const h = cross3(d, e2), a = dot3(e1, h);
    if (Math.abs(a) < 1e-12) continue;
    const f = 1 / a, s = [o[0] - q[0][0], o[1] - q[0][1], o[2] - q[0][2]];
    const u = f * dot3(s, h); if (u < 0 || u > 1) continue;
    const qq = cross3(s, e1), v = f * dot3(d, qq); if (v < 0 || u + v > 1) continue;
    const tt = f * dot3(e2, qq); if (tt > 2e-4 && tt < best) best = tt;
  }
  return best;
}
let hiddenFaces = 0;
// (The face itself is skipped by the minimum hit distance.)
const isOutside = (kind, mesh, q, fn) => {
  const tHit = rayHits(groupOf(kind, mesh), triCentroid(q), unit(fn)), hit = tHit < (kind === "endplate" ? Infinity : RAY_MAX);
  if (hit) hiddenFaces++;
  return !hit;
};
// Wing elements are sorted by height instead: flattened on the element's
// plane, a face is on the UPPER surface where nothing of the element lies
// above it, on the LOWER where nothing lies below; anything in between is an
// inner skin. (Normals can't be trusted at a thin trailing edge, where the
// simplified mesh's vertex normals average top and bottom.)
const ELEM_CELL = 0.005, ELEM_TOL = 0.003;
const heightFields = new Map();
function heightField(mesh) {
  if (heightFields.has(mesh)) return heightFields.get(mesh);
  const pl = elementPlanes.get(mesh), tris = groupTris.get(mesh);
  const st = (p) => [dot3(p, pl.span), dot3(p, pl.fwd), dot3(p, pl.n)];
  const P = tris.map((q) => q.map(st));
  let s0 = Infinity, s1 = -Infinity, c0 = Infinity, c1 = -Infinity;
  for (const q of P) for (const [a, b] of q) { s0 = Math.min(s0, a); s1 = Math.max(s1, a); c0 = Math.min(c0, b); c1 = Math.max(c1, b); }
  const NSx = Math.ceil((s1 - s0) / ELEM_CELL) + 1, NC = Math.ceil((c1 - c0) / ELEM_CELL) + 1;
  const hi = new Float64Array(NSx * NC).fill(-Infinity), lo = new Float64Array(NSx * NC).fill(Infinity);
  const put = (i, j, h) => { const k = j * NSx + i; if (h > hi[k]) hi[k] = h; if (h < lo[k]) lo[k] = h; };
  for (const q of P) {
    const [[ax, ay, ah], [bx, by, bh], [cx, cy, ch]] = q;
    const d = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
    for (const v of q) put(Math.round((v[0] - s0) / ELEM_CELL), Math.round((v[1] - c0) / ELEM_CELL), v[2]);
    if (Math.abs(d) < 1e-12) continue;
    const i0 = Math.floor((Math.min(ax, bx, cx) - s0) / ELEM_CELL), i1 = Math.ceil((Math.max(ax, bx, cx) - s0) / ELEM_CELL);
    const j0 = Math.floor((Math.min(ay, by, cy) - c0) / ELEM_CELL), j1 = Math.ceil((Math.max(ay, by, cy) - c0) / ELEM_CELL);
    for (let j = Math.max(0, j0); j <= Math.min(NC - 1, j1); j++) for (let i = Math.max(0, i0); i <= Math.min(NSx - 1, i1); i++) {
      const px = s0 + i * ELEM_CELL, py = c0 + j * ELEM_CELL;
      const w1 = ((px - ax) * (cy - ay) - (cx - ax) * (py - ay)) / d, w2 = ((bx - ax) * (py - ay) - (px - ax) * (by - ay)) / d, w0 = 1 - w1 - w2;
      if (w0 < -1e-6 || w1 < -1e-6 || w2 < -1e-6) continue;
      put(i, j, w0 * ah + w1 * bh + w2 * ch);
    }
  }
  const hf = { st, at: (a, b) => { const k = Math.min(NC - 1, Math.max(0, Math.round((b - c0) / ELEM_CELL))) * NSx + Math.min(NSx - 1, Math.max(0, Math.round((a - s0) / ELEM_CELL))); return [lo[k], hi[k]]; } };
  heightFields.set(mesh, hf);
  return hf;
}
/** "upper", "lower" or null (an inner skin) for a wing element's face. */
function elementSide(mesh, q, fn) {
  const hf = heightField(mesh), pl = elementPlanes.get(mesh);
  const [a, b, h] = hf.st(triCentroid(q));
  const [lo, hi] = hf.at(a, b);
  const top = h >= hi - ELEM_TOL, bottom = h <= lo + ELEM_TOL;
  // The wall's inner skin sits within the tolerance of the outer one but
  // faces the other way; the normal settles it.
  const facesUp = dot3(fn, pl.n) >= 0;
  return top && facesUp ? "upper" : bottom && !facesUp ? "lower" : null;
}
// An "endplate" part can hold several parallel plates (the front wing's
// endplate and its fences). Each plate is its own piece: group the side-facing
// faces by distance from the centreline, a gap over 5 cm starts a new plate.
// The outermost is the endplate; inner ones are fences A, B.. from inboard.
const plateLayers = new Map();
{
  const ys = new Map();
  forEachTri((kind, mesh, prim, t, q, fn) => {
    if (kind !== "endplate") return;
    const c = triCentroid(q), k = `${/FW-013/i.test(mesh.getName()) ? "front" : "rear"} ${c[1] >= 0 ? "left" : "right"}`;
    if (!ys.has(k)) ys.set(k, []);
    ys.get(k).push(Math.abs(c[1]));
  });
  for (const [k, arr] of ys) {
    arr.sort((a, b) => a - b);
    const layers = [[arr[0], arr[0]]];
    for (const y of arr) { if (y - layers.at(-1)[1] > 0.05) layers.push([y, y]); else layers.at(-1)[1] = y; }
    plateLayers.set(k, layers);
  }
}
// -- relax the body skin: as-rigid-as-possible -----------------------------------
// The station-by-station unroll is exact only where the skin runs along the
// car. A face that looks forward or back (a sidepod's leading face, the step
// from the cockpit side down onto the sidepod) has almost no length along x,
// so the unroll squashes it to a sliver and shears its neighbours. Starting
// from the unroll (so the layout stays where it was), let every triangle
// take back its true shape: alternate fitting each face's best rotation
// (local) and a least-squares solve for the vertices (global), with a faint
// pull to the start so the piece doesn't drift. Vertices are shared across a
// face edge only where the unroll gave them the same place: the underside
// cut stays cut.
let bodySideTilt = null;
const ARAP_ITERS = +(process.env.ARAP_ITERS ?? 30);
function relaxBody(faceUV, faceQ, faceN) {
  const vid = new Map(), U = [], V = [], faces = [];
  const idOf = (p, uv) => {
    const k = p.map((x) => Math.round(x * 1e4)).join(",") + "|" + uv.map((x) => Math.round(x * 500)).join(",");
    if (!vid.has(k)) { vid.set(k, U.length); U.push(uv[0]); V.push(uv[1]); }
    return vid.get(k);
  };
  // The unroll's handedness: seen from outside, is a face's UV copy turned
  // the same way or mirrored? (It is one or the other for the whole skin;
  // take the majority, as squashed faces can come out either way.)
  const areaUV = (uv) => (uv[1][0] - uv[0][0]) * (uv[2][1] - uv[0][1]) - (uv[2][0] - uv[0][0]) * (uv[1][1] - uv[0][1]);
  const outwardCCW = (q, f) => Math.sign(dot3(cross3(q[1].map((x, a) => x - q[0][a]), q[2].map((x, a) => x - q[0][a])), faceN.get(f)));
  let vote = 0;
  for (const [f, uv] of faceUV) vote += Math.sign(areaUV(uv)) * outwardCCW(faceQ.get(f), f);
  const hand = vote >= 0 ? 1 : -1;
  for (const [f, uv] of faceUV) {
    const q = faceQ.get(f), ids = [0, 1, 2].map((k) => idOf(q[k], uv[k]));
    const mirror = hand * outwardCCW(q, f) < 0 ? -1 : 1;
    // The face in its own plane: p0 at the origin, p1 on the x axis.
    const e1 = q[1].map((x, a) => x - q[0][a]), e2 = q[2].map((x, a) => x - q[0][a]);
    const l1 = Math.hypot(...e1), cr = Math.hypot(...cross3(e1, e2));
    if (l1 < 1e-7 || cr < 1e-10) continue;
    const P = [[0, 0], [l1, 0], [dot3(e1, e2) / l1, mirror * cr / l1]];
    // Cotangent weights, one per edge (i, j), from the angle opposite it.
    const cot = (a, b, c) => { const u = [P[b][0] - P[a][0], P[b][1] - P[a][1]], v = [P[c][0] - P[a][0], P[c][1] - P[a][1]]; return (u[0] * v[0] + u[1] * v[1]) / Math.abs(u[0] * v[1] - u[1] * v[0]); };
    const E = [[0, 1, cot(2, 0, 1)], [1, 2, cot(0, 1, 2)], [2, 0, cot(1, 2, 0)]].map(([i, j, w]) => [i, j, Math.min(10, Math.max(0.01, w))]);
    faces.push({ f, ids, P, E });
  }
  const n = U.length;
  // The system matrix doesn't change between iterations: build it once.
  const rows = Array.from({ length: n }, () => new Map());
  let diagMean = 0;
  for (const { ids, E } of faces) for (const [i, j, w] of E) {
    const a = ids[i], b = ids[j];
    rows[a].set(a, (rows[a].get(a) ?? 0) + w); rows[b].set(b, (rows[b].get(b) ?? 0) + w);
    rows[a].set(b, (rows[a].get(b) ?? 0) - w); rows[b].set(a, (rows[b].get(a) ?? 0) - w);
  }
  for (let i = 0; i < n; i++) diagMean += rows[i].get(i) ?? 0;
  const PULL = +(process.env.ARAP_PULL ?? 1e-2) * (diagMean / n);
  const U0 = U.slice(), V0 = V.slice();
  for (let i = 0; i < n; i++) rows[i].set(i, (rows[i].get(i) ?? 0) + PULL);
  // Keep the sides level: on a face that looks sideways, the car's x axis
  // should run straight across the image (no change in v along it), so a
  // line of text painted level on the template sits level on the car. A
  // soft term, only in the v solve.
  const LEVEL = +(process.env.ARAP_LEVEL ?? 1) * (diagMean / n);
  const rowsV = rows.map((m) => new Map(m));
  let areaMean = 0; for (const { f } of faces) { const q = faceQ.get(f); areaMean += Math.hypot(...cross3(q[1].map((v, a) => v - q[0][a]), q[2].map((v, a) => v - q[0][a]))) / 2; }
  areaMean /= faces.length || 1;
  for (const { f, ids } of faces) {
    const q = faceQ.get(f), nn = unit(faceN.get(f)), side = nn[1] * nn[1];
    if (side < 0.25) continue;
    const e1 = q[1].map((v, a) => v - q[0][a]), e2 = q[2].map((v, a) => v - q[0][a]);
    const g11 = dot3(e1, e1), g12 = dot3(e1, e2), g22 = dot3(e2, e2), det = g11 * g22 - g12 * g12;
    if (det < 1e-14) continue;
    const a = (g22 * e1[0] - g12 * e2[0]) / det, b = (g11 * e2[0] - g12 * e1[0]) / det;
    const len = Math.hypot(a, b) || 1;
    const coef = [(-a - b) / len, a / len, b / len], w = LEVEL * side * (Math.sqrt(det) / 2) / areaMean;
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
      const m = rowsV[ids[r]]; m.set(ids[c], (m.get(ids[c]) ?? 0) + w * coef[r] * coef[c]);
    }
  }
  const pack = (rs) => rs.map((m) => [Int32Array.from(m.keys()), Float64Array.from(m.values())]);
  const RU = pack(rows), RV = pack(rowsV);
  let R = RU;
  const mul = (x, y) => { for (let i = 0; i < n; i++) { const [c, w] = R[i]; let s = 0; for (let k = 0; k < c.length; k++) s += w[k] * x[c[k]]; y[i] = s; } };
  const cg = (x, b) => {   // conjugate gradients, warm-started
    const r = new Float64Array(n), p = new Float64Array(n), Ap = new Float64Array(n);
    mul(x, Ap); let rr = 0;
    for (let i = 0; i < n; i++) { r[i] = b[i] - Ap[i]; p[i] = r[i]; rr += r[i] * r[i]; }
    const tol = 1e-20 * n;
    for (let it = 0; it < 400 && rr > tol; it++) {
      mul(p, Ap); let pAp = 0; for (let i = 0; i < n; i++) pAp += p[i] * Ap[i];
      const a = rr / pAp; let rr2 = 0;
      for (let i = 0; i < n; i++) { x[i] += a * p[i]; r[i] -= a * Ap[i]; rr2 += r[i] * r[i]; }
      const beta = rr2 / rr; rr = rr2;
      for (let i = 0; i < n; i++) p[i] = r[i] + beta * p[i];
    }
  };
  const x = Float64Array.from(U), y = Float64Array.from(V);
  for (let iter = 0; iter < ARAP_ITERS; iter++) {
    const bx = Float64Array.from(U0, (u) => u * PULL), by = Float64Array.from(V0, (v) => v * PULL);
    for (const { ids, P, E } of faces) {
      // Local: the rotation that best maps the face's true edges onto its
      // current UV edges (never a reflection).
      let s00 = 0, s01 = 0, s10 = 0, s11 = 0;
      for (const [i, j, w] of E) {
        const du = x[ids[i]] - x[ids[j]], dv = y[ids[i]] - y[ids[j]], px = P[i][0] - P[j][0], py = P[i][1] - P[j][1];
        s00 += w * du * px; s01 += w * du * py; s10 += w * dv * px; s11 += w * dv * py;
      }
      const th = Math.atan2(s10 - s01, s00 + s11), c = Math.cos(th), s = Math.sin(th);
      for (const [i, j, w] of E) {
        const px = P[i][0] - P[j][0], py = P[i][1] - P[j][1], rx = c * px - s * py, ry = s * px + c * py;
        bx[ids[i]] += w * rx; by[ids[i]] += w * ry; bx[ids[j]] -= w * rx; by[ids[j]] -= w * ry;
      }
    }
    R = RU; cg(x, bx); R = RV; cg(y, by);   // global
  }
  for (const { f, ids } of faces) faceUV.set(f, ids.map((i) => [x[i], y[i]]));
  // How level the sides stay: on faces that look sideways, the angle the
  // car's x axis makes with the image's across. Text painted level on the
  // template tilts on the car by this much.
  const tilts = [];
  for (const { f } of faces) {
    const q = faceQ.get(f), nn = unit(faceN.get(f)); if (Math.abs(nn[1]) < 0.8) continue;
    const uv = faceUV.get(f), e1 = q[1].map((v, a) => v - q[0][a]), e2 = q[2].map((v, a) => v - q[0][a]);
    // Solve [e1 e2] [a b]^T ~ x-hat in the face plane (least squares), carry to UV.
    const g11 = dot3(e1, e1), g12 = dot3(e1, e2), g22 = dot3(e2, e2), r1 = e1[0], r2 = e2[0], det = g11 * g22 - g12 * g12;
    if (det < 1e-14) continue;
    const a = (g22 * r1 - g12 * r2) / det, b = (g11 * r2 - g12 * r1) / det;
    const du = a * (uv[1][0] - uv[0][0]) + b * (uv[2][0] - uv[0][0]), dv = a * (uv[1][1] - uv[0][1]) + b * (uv[2][1] - uv[0][1]);
    tilts.push([Math.abs(Math.atan2(dv, -du)) * 180 / Math.PI, Math.sqrt(det) / 2]);
  }
  tilts.sort((p, q) => p[0] - q[0]);
  const tot = tilts.reduce((s, t) => s + t[1], 0), pct = (fr) => { let acc = 0; for (const [d, w] of tilts) { acc += w; if (acc >= fr * tot) return +d.toFixed(1); } return 0; };
  bodySideTilt = [pct(0.5), pct(0.9)];
  // The top centreline, where it lands: the relaxed spots of the skin's
  // vertices on y = 0, on top.
  const top = [];
  for (const [f, uv] of faceUV) faceQ.get(f).forEach((p, k) => {
    if (Math.abs(p[1]) < 0.002 && arcAt(p[0], p[1], p[2]).phi > -Math.PI / 2 && arcAt(p[0], p[1], p[2]).phi < Math.PI / 2) top.push(uv[k]);
  });
  top.sort((a, b) => a[0] - b[0]);
  return top;
}
const bodyFaceUV = new Map(), bodyFaceQ = new Map(), bodyFaceN = new Map();
const pieces = new Map();   // key -> { key, lo:[u,v], hi:[u,v], map(p) -> [u, v] in metres }
const faceInfo = [], faceVis = [];        // per livery face, in forEachTri order: { key | null }
function pieceFor(key, map) {
  if (!pieces.has(key)) pieces.set(key, { key, map, lo: [Infinity, Infinity], hi: [-Infinity, -Infinity], tris: 0, area3: 0 });
  return pieces.get(key);
}
forEachTri((kind, mesh, prim, t, q, fn) => {
  const c = triCentroid(q);
  let key = null, map = null;
  const side = kind === "element" ? elementSide(mesh, q, fn) : null;
  const outside = kind === "element" ? side !== null : isOutside(kind, mesh, q, fn);
  if (kind === "element" && !side) hiddenFaces++;
  if (!outside) { /* inner skin: no livery */ }
  else if (kind === "body") {
    const zc = arcAt(c[0], c[1], c[2]).zc;
    if (fn[1] * c[1] + fn[2] * (c[2] - zc) >= 0) {
      key = "body skin";
      map = null;   // done per vertex below (needs the triangle's cut side)
    }
  } else if (kind === "endplate") {
    const side = c[1] >= 0 ? 1 : -1;
    // Its outer face only; a flange or rim seen edge-on would smear.
    if (unit(fn)[1] * side >= 0.8) {
      const which = /FW-013/i.test(mesh.getName()) ? "front" : "rear";
      const sideName = side > 0 ? "left" : "right";
      const layers = plateLayers.get(`${which} ${sideName}`), ay = Math.abs(c[1]);
      const li = layers.findIndex(([a, b]) => ay >= a - 1e-6 && ay <= b + 1e-6);
      key = li === layers.length - 1 ? `${which} endplate ${sideName}` : `${which} wing fence ${sideName} ${"ABCDEF"[li]}`;
      map = side > 0 ? (p) => [-p[0], -p[2]] : (p) => [p[0], -p[2]];
    }
  } else {
    const pl = elementPlanes.get(mesh);
    const up = side === "upper";
    key = `${elementName(mesh.getName(), c)} ${up ? "upper" : "lower"}`;
    // Span across the image, leading edge up, as you would read a wing:
    // upper surface seen from behind and above (car's left on the left),
    // lower surface seen from underneath, leading edge up (car's left on the
    // right, as it is when you look up at it).
    map = up
      ? (p) => [-dot3(p, pl.span), -dot3(p, pl.fwd)]
      : (p) => [dot3(p, pl.span), -dot3(p, pl.fwd)];
  }
  faceInfo.push(key);
  // VISCHK=1: also write out/vis.bin, the UVs of the faces that see out, for
  // the overlap check (tools/cad/livery_overlap.py).
  if (process.env.VISCHK) faceVis.push(kind !== "element" || rayHits(mesh, triCentroid(q), unit(fn)) === Infinity);
  if (!key) return;
  const pc = pieceFor(key, map);
  pc.tris++;
  const e1 = [q[1][0] - q[0][0], q[1][1] - q[0][1], q[1][2] - q[0][2]], e2 = [q[2][0] - q[0][0], q[2][1] - q[0][1], q[2][2] - q[0][2]];
  pc.area3 += Math.hypot(...cross3(e1, e2)) / 2;
  let uvs;
  if (key === "body skin") {
    const hint = arcAt(c[0], c[1], c[2]).phi;
    bodyFaceUV.set(faceInfo.length - 1, q.map((p) => [BX1 - p[0], arcAt(p[0], p[1], p[2], hint).s - sMin]));
    bodyFaceQ.set(faceInfo.length - 1, q); bodyFaceN.set(faceInfo.length - 1, fn);
    return;   // bounds after the relaxation
  } else uvs = q.map(pc.map);
  for (const [u, v] of uvs) { pc.lo[0] = Math.min(pc.lo[0], u); pc.lo[1] = Math.min(pc.lo[1], v); pc.hi[0] = Math.max(pc.hi[0], u); pc.hi[1] = Math.max(pc.hi[1], v); }
});
const bodyTopLine = relaxBody(bodyFaceUV, bodyFaceQ, bodyFaceN);
{
  const pc = pieces.get("body skin");
  for (const uv of bodyFaceUV.values()) for (const [u, v] of uv) { pc.lo[0] = Math.min(pc.lo[0], u); pc.lo[1] = Math.min(pc.lo[1], v); pc.hi[0] = Math.max(pc.hi[0], u); pc.hi[1] = Math.max(pc.hi[1], v); }
}
// Drop slivers: a piece under 2 cm^2 is a CAD edge, not a paintable surface.
for (const [k, pc] of pieces) if (pc.area3 < 2e-4) { pieces.delete(k); }

// -- pack: skyline bottom-left, body first, one scale ---------------------------
const MARGIN = 0.04;   // metres between pieces (~35 px at 4096): room for mip bleed
const list = [...pieces.values()].map((pc) => ({ pc, w: pc.hi[0] - pc.lo[0], h: pc.hi[1] - pc.lo[1] }));
list.sort((a, b) => (a.pc.key === "body skin" ? -1 : b.pc.key === "body skin" ? 1 : b.h - a.h || b.w - a.w));
function packInto(W) {
  const sky = [{ x: 0, w: W, y: 0 }];
  const placed = [];
  for (const it of list) {
    const w = it.w + MARGIN, h = it.h + MARGIN;
    let best = null;
    for (let i = 0; i < sky.length; i++) {
      if (sky[i].x + w > W + 1e-9) break;
      let y = 0, rem = w, j = i;
      while (rem > 1e-9 && j < sky.length) { y = Math.max(y, sky[j].y); rem -= sky[j].w; j++; }
      if (rem > 1e-9) continue;
      if (!best || y + h < best.y + best.h - 1e-9 || (Math.abs(y + h - best.y - best.h) < 1e-9 && sky[i].x < best.x)) best = { i, x: sky[i].x, y, h };
    }
    if (!best) return null;
    placed.push({ it, x: best.x, y: best.y });
    // raise the skyline over [x, x + w)
    const x0 = best.x, x1 = best.x + w, ny = best.y + h, next = [];
    for (const s of sky) {
      const s0 = s.x, s1 = s.x + s.w;
      if (s1 <= x0 || s0 >= x1) { next.push(s); continue; }
      if (s0 < x0) next.push({ x: s0, w: x0 - s0, y: s.y });
      if (s1 > x1) next.push({ x: x1, w: s1 - x1, y: s.y });
    }
    next.push({ x: x0, w: x1 - x0, y: ny });
    next.sort((a, b) => a.x - b.x);
    sky.length = 0; sky.push(...next);
  }
  const H = Math.max(...placed.map((p) => p.y + p.it.h + MARGIN));
  return { placed, H };
}
let bestPack = null;
const minW = Math.max(...list.map((it) => it.w)) + MARGIN;
for (let W = minW; W < minW * 3; W += 0.02) {
  const r = packInto(W);
  if (!r) continue;
  const side = Math.max(W, r.H);
  if (!bestPack || side < bestPack.side) bestPack = { ...r, W, side };
}
const TEX_M = bestPack.side + MARGIN;
for (const { it, x, y } of bestPack.placed) { it.pc.x = x + MARGIN / 2; it.pc.y = y + MARGIN / 2; it.pc.w = it.w; it.pc.h = it.h; }

// -- write the UVs ----------------------------------------------------------------
let liveryTris = 0, inward = 0, fi = 0;
const visUV = [];
const stretch = new Map();   // key -> [uv area / 3d area] per face
for (const mesh of liveryMeshes) {
  const kind = liveryKind(mesh.getName());
  for (const prim of mesh.listPrimitives()) {
    const P = prim.getAttribute("POSITION").getArray(), N = prim.getAttribute("NORMAL")?.getArray();
    const I = prim.getIndices().getArray();
    const nt = I.length / 3;
    const pos = new Float32Array(nt * 9), nrm = new Float32Array(nt * 9), uv = new Float32Array(nt * 6);
    for (let t = 0; t < nt; t++) {
      const v = [I[t * 3], I[t * 3 + 1], I[t * 3 + 2]];
      const q = v.map((i) => [P[i * 3], P[i * 3 + 1], P[i * 3 + 2]]);
      const key = faceInfo[fi++];
      const pc = key ? pieces.get(key) : null;
      let uvs;
      if (!pc) { uvs = [[-TEX_M, -TEX_M], [-TEX_M, -TEX_M], [-TEX_M, -TEX_M]]; inward++; }
      else {
        let raw;
        if (key === "body skin") raw = bodyFaceUV.get(fi - 1);
        else raw = q.map(pc.map);
        uvs = raw.map(([a, b]) => [pc.x + (a - pc.lo[0]), pc.y + (b - pc.lo[1])]);
        const e1 = [q[1][0] - q[0][0], q[1][1] - q[0][1], q[1][2] - q[0][2]], e2 = [q[2][0] - q[0][0], q[2][1] - q[0][1], q[2][2] - q[0][2]];
        const a3 = Math.hypot(...cross3(e1, e2)) / 2;
        const a2 = Math.abs((uvs[1][0] - uvs[0][0]) * (uvs[2][1] - uvs[0][1]) - (uvs[2][0] - uvs[0][0]) * (uvs[1][1] - uvs[0][1])) / 2;
        if (a3 > 1e-7) (stretch.get(key) ?? stretch.set(key, []).get(key)).push([a2 / a3, a3]);
      }
      for (let k = 0; k < 3; k++) {
        pos.set(q[k], (t * 3 + k) * 3);
        if (N) nrm.set([N[v[k] * 3], N[v[k] * 3 + 1], N[v[k] * 3 + 2]], (t * 3 + k) * 3);
        uv[(t * 3 + k) * 2] = uvs[k][0] / TEX_M;
        uv[(t * 3 + k) * 2 + 1] = uvs[k][1] / TEX_M;
        if (process.env.VISCHK && pc && faceVis[fi - 1]) visUV.push(uvs[k][0] / TEX_M, uvs[k][1] / TEX_M);
      }
    }
    liveryTris += nt;
    prim.getAttribute("POSITION").setArray(pos);
    if (N) prim.getAttribute("NORMAL").setArray(nrm);
    prim.setAttribute("TEXCOORD_0", out.createAccessor().setType("VEC2").setArray(uv).setBuffer(buffer));
    const mat = prim.getMaterial();
    if (mat) {
      const mk = "livery:" + mat.getName();
      if (!matCache.has(mk)) matCache.set(mk, mat.clone().setName(mat.getName() + "-livery"));
      prim.setMaterial(matCache.get(mk));
    }
    prim.setIndices(out.createAccessor().setType("SCALAR").setArray(Uint32Array.from({ length: nt * 3 }, (_, i) => i)).setBuffer(buffer));
  }
}
// Stretch: area-weighted, 1.0 = true scale. Report the spread per piece.
const stretchReport = {};
for (const [k, arr] of stretch) {
  arr.sort((a, b) => a[0] - b[0]);
  const tot = arr.reduce((s, x) => s + x[1], 0);
  const q = (f) => { let acc = 0; for (const [r, a] of arr) { acc += a; if (acc >= f * tot) return +r.toFixed(2); } return +arr.at(-1)[0].toFixed(2); };
  stretchReport[k] = [q(0.05), q(0.5), q(0.95)];
}
const used = [...pieces.values()].reduce((s, pc) => s + pc.w * pc.h, 0) / (TEX_M * TEX_M);
if (process.env.VISCHK) (await import("node:fs")).writeFileSync("out/vis.bin", Buffer.from(new Float32Array(visUV).buffer));
report.livery = { bodySideTiltDeg_p50_p90: bodySideTilt, tris: liveryTris, inwardNoLivery: inward, innerSkin: hiddenFaces, pieces: pieces.size, textureMetres: +TEX_M.toFixed(3),
  mmPerPx4096: +(TEX_M * 1000 / 4096).toFixed(2), boxFill: +used.toFixed(2), stretch_p5_p50_p95: stretchReport };
const frac = (pc) => [pc.x / TEX_M, pc.y / TEX_M, pc.w / TEX_M, pc.h / TEX_M];
const bodyPc = pieces.get("body skin");
outScene.setExtras({ ...outScene.getExtras(), livery: {
  size: TEX_M,
  layout: "unwrap-v3",
  topLine: bodyTopLine.map(([u, v]) => [+((bodyPc.x + u - bodyPc.lo[0]) / TEX_M).toFixed(5), +((bodyPc.y + v - bodyPc.lo[1]) / TEX_M).toFixed(5)]),
  views: Object.fromEntries([...pieces.values()].map((pc) => [pc.key, frac(pc)])),
} });

if (process.env.TOP) {
  const rows = out.getRoot().listMeshes().map((m) => [m.listPrimitives().reduce((t, p) => t + p.getIndices().getCount() / 3, 0), m.getName()]);
  rows.sort((a, b) => b[0] - a[0]);
  for (const [t, n] of rows.slice(0, +process.env.TOP)) console.log(String(t).padStart(7), n);
}
// Merge the body into one mesh per material (few draw calls); the wheel nodes
// keep their shared mesh.
// keepAttributes: prune otherwise drops TEXCOORD_0, which no material
// texture uses -- the livery is applied by the renderer, not the file.
await out.transform(join({ keepNamed: false, filter: (n) => !/^(wheel_|rig:|ctl:)/.test(n.getName()) }), prune({ keepAttributes: true }));
// `join` may leave body nodes nested under one parent; the loader sums
// translations, which are all zero for baked body nodes, so that is fine.
// Parts the simplifier emptied: an empty accessor gets null bounds, which is
// invalid glTF (Blender and validators refuse the file; the sim shrugged).
for (const mesh of out.getRoot().listMeshes()) {
  for (const prim of mesh.listPrimitives()) {
    if (!prim.getAttribute("POSITION")?.getCount() || !prim.getIndices()?.getCount()) { mesh.removePrimitive(prim); prim.dispose(); }
  }
}
await out.transform(prune({ keepAttributes: true }));
// Smaller storage: normals as normalized bytes (the loader scales them back),
// indices as 16-bit wherever a primitive has fewer than 65,536 vertices.
for (const mesh of out.getRoot().listMeshes()) {
  for (const prim of mesh.listPrimitives()) {
    const n = prim.getAttribute("NORMAL");
    if (n && n.getComponentType() === 5126) {
      const f = n.getArray(), q = new Int8Array(f.length);
      for (let i = 0; i < f.length; i++) q[i] = Math.max(-127, Math.min(127, Math.round(f[i] * 127)));
      n.setArray(q).setNormalized(true);
    }
    const ix = prim.getIndices();
    if (prim.getAttribute("POSITION").getCount() < 65536 && !(ix.getArray() instanceof Uint16Array)) ix.setArray(Uint16Array.from(ix.getArray()));
  }
}
await io.write(OUT, out);
const size = (await import("node:fs")).statSync(OUT).size;
console.log(JSON.stringify({
  err: ERR, minPart: MIN_PART, out: OUT, mb: +(size / 1e6).toFixed(2),
  trisIn: Math.round(before + report.droppedTris), trisKept: Math.round(before), trisOut: Math.round(afterSimplify),
  wheelTris: wheelMesh.listPrimitives().reduce((t, p) => t + p.getIndices().getCount() / 3, 0),
  splitFinish: report.splitFinish, hardware: report.hardware, parts: report.kept, dropped: report.dropped, dupes: report.dupes, reflected: report.reflected, rig: report.rig, ctl: report.ctl, livery: report.livery,
  hubs: Object.fromEntries(Object.entries(hub).map(([k, v]) => [k, v.c.map((x) => +x.toFixed(3)).concat(v.parts.map((p) => p.leaf).join("|"))])),
  materials: out.getRoot().listMaterials().length, meshes: out.getRoot().listMeshes().length,
}, null, 1));
