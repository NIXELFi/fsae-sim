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
if (process.env.TOP) {
  const rows = out.getRoot().listMeshes().map((m) => [m.listPrimitives().reduce((t, p) => t + p.getIndices().getCount() / 3, 0), m.getName()]);
  rows.sort((a, b) => b[0] - a[0]);
  for (const [t, n] of rows.slice(0, +process.env.TOP)) console.log(String(t).padStart(7), n);
}
// Merge the body into one mesh per material (few draw calls); the wheel nodes
// keep their shared mesh.
await out.transform(join({ keepNamed: false, filter: (n) => !/^(wheel_|rig:|ctl:)/.test(n.getName()) }), prune());
// `join` may leave body nodes nested under one parent; the loader sums
// translations, which are all zero for baked body nodes, so that is fine.
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
  splitFinish: report.splitFinish, hardware: report.hardware, parts: report.kept, dropped: report.dropped, dupes: report.dupes, reflected: report.reflected, rig: report.rig, ctl: report.ctl,
  hubs: Object.fromEntries(Object.entries(hub).map(([k, v]) => [k, v.c.map((x) => +x.toFixed(3)).concat(v.parts.map((p) => p.leaf).join("|"))])),
  materials: out.getRoot().listMaterials().length, meshes: out.getRoot().listMeshes().length,
}, null, 1));
