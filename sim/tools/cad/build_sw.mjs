// Steering wheel: SWMA-ASM export -> data/steering-wheel.glb.
//   node build_sw.mjs [errM] [in] [out]
// Drops McMaster hardware (inserts, screws, nuts), bakes transforms, keeps the
// wheel's own frame (metres, +y up, the driver on -z -- the sim's steering
// wheel convention), flattens colours (carbon textures -> carbon black) and
// simplifies each part to an absolute error.
import { NodeIO, Document } from "@gltf-transform/core";
import { weld, simplifyPrimitive, dedup, prune, join } from "@gltf-transform/functions";
import { MeshoptSimplifier } from "meshoptimizer";
const ERR = +(process.argv[2] ?? 0.0003);
const IN = process.argv[3] ?? "raw/SWMA-ASM-U_V4.glb";
const OUT = process.argv[4] ?? "out/steering-wheel.glb";
await MeshoptSimplifier.ready;
const io = new NodeIO();
const src = await io.read(IN);
const out = new Document(); const buf = out.createBuffer(); const scene = out.createScene("sw");
const HARDWARE = /\b\d{5}A\d{2,4}\b|\d{5}A\d{2,4}_|Screw|Insert|Locknut|Nut\b|Washer|Heat-Set|SKF/i;
const CARBON = [0.045, 0.046, 0.05, 1];
const mats = new Map();
function mat(m) {
  const name = (m?.getName() ?? "").toLowerCase();
  let c = m ? m.getBaseColorFactor() : [0.5, 0.5, 0.5, 1];
  if (name.includes("carbon") || m?.getBaseColorTexture()) c = CARBON;
  c = c.map((v) => Math.round(v * 50) / 50);
  const k = c.join(",");
  if (!mats.has(k)) mats.set(k, out.createMaterial("m" + mats.size).setBaseColorFactor(c).setMetallicFactor(0).setRoughnessFactor(0.5));
  return mats.get(k);
}
const xp = (m, x, y, z) => [m[0] * x + m[4] * y + m[8] * z + m[12], m[1] * x + m[5] * y + m[9] * z + m[13], m[2] * x + m[6] * y + m[10] * z + m[14]];
const xd = (m, x, y, z) => [m[0] * x + m[4] * y + m[8] * z, m[1] * x + m[5] * y + m[9] * z, m[2] * x + m[6] * y + m[10] * z];
const det = (m) => m[0] * (m[5] * m[10] - m[9] * m[6]) - m[4] * (m[1] * m[10] - m[9] * m[2]) + m[8] * (m[1] * m[6] - m[5] * m[2]);
let kept = 0, dropped = 0, trisIn = 0;
const mesh = out.createMesh("steering_wheel");
src.getRoot().listScenes()[0].traverse((node) => {
  const me = node.getMesh(); if (!me) return;
  const path = []; for (let q = node; q; q = q.getParentNode()) path.unshift(q.getName() ?? "");
  const tris = me.listPrimitives().reduce((s, p) => s + (p.getIndices()?.getCount() ?? 0) / 3, 0);
  trisIn += tris;
  if (HARDWARE.test(path[path.length - 1])) { dropped++; return; }
  kept++;
  const m = node.getWorldMatrix();
  for (const pr of me.listPrimitives()) {
    if (pr.getMode() !== 4) continue;
    const pos = pr.getAttribute("POSITION").getArray(), nrm = pr.getAttribute("NORMAL")?.getArray();
    const n = pos.length / 3, P = new Float32Array(n * 3), N = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      P.set(xp(m, pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]), 3 * i);
      const d = nrm ? xd(m, nrm[3 * i], nrm[3 * i + 1], nrm[3 * i + 2]) : [0, 0, 1];
      const l = Math.hypot(...d) || 1; N.set([d[0] / l, d[1] / l, d[2] / l], 3 * i);
    }
    let idx = Uint32Array.from(pr.getIndices()?.getArray() ?? Array.from({ length: n }, (_, i) => i));
    if (det(m) < 0) for (let i = 0; i < idx.length; i += 3) { const t = idx[i + 1]; idx[i + 1] = idx[i + 2]; idx[i + 2] = t; }
    const p = out.createPrimitive()
      .setAttribute("POSITION", out.createAccessor().setType("VEC3").setArray(P).setBuffer(buf))
      .setAttribute("NORMAL", out.createAccessor().setType("VEC3").setArray(N).setBuffer(buf))
      .setIndices(out.createAccessor().setType("SCALAR").setArray(idx).setBuffer(buf))
      .setMaterial(mat(pr.getMaterial()));
    // Simplify each part on its own, to an absolute error.
    const a = p.getAttribute("POSITION"); const mn = a.getMin([]), mx = a.getMax([]);
    const r = Math.max(1e-6, Math.hypot(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]) / 2);
    mesh.addPrimitive(p);
    p.__err = Math.min(0.05, ERR / r);
  }
});
await out.transform(weld());
for (const p of mesh.listPrimitives()) simplifyPrimitive(p, { simplifier: MeshoptSimplifier, ratio: 0, error: p.__err ?? Math.min(0.05, ERR / 0.05), lockBorder: false });
scene.addChild(out.createNode("steering_wheel").setMesh(mesh));
await out.transform(dedup(), join({ keepNamed: false }), prune());
await io.write(OUT, out);
const trisOut = out.getRoot().listMeshes().reduce((s, me) => s + me.listPrimitives().reduce((t, p) => t + p.getIndices().getCount() / 3, 0), 0);
console.log(JSON.stringify({ err: ERR, in: IN, trisIn, parts: kept, droppedHardware: dropped, trisOut, mb: +((await import("node:fs")).statSync(OUT).size / 1e6).toFixed(2), materials: out.getRoot().listMaterials().length }));
