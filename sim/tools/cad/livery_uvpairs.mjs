// Triangle-by-triangle UV correspondence between two builds of the same car:
// writes Float32 records [newU0,newV0,newU1,newV1,newU2,newV2, oldU0..oldV2].
//   node uvpairs.mjs <old.glb> <new.glb> <out.bin>
import { NodeIO } from "@gltf-transform/core";
import fs from "node:fs";
const [, , oldF, newF, outF] = process.argv;
const tris = async (f) => {
  const d = await new NodeIO().read(f), res = [];
  for (const m of d.getRoot().listMeshes()) for (const p of m.listPrimitives()) {
    const uv = p.getAttribute("TEXCOORD_0"); if (!uv) continue;
    const P = p.getAttribute("POSITION").getArray(), U = uv.getArray(), I = p.getIndices()?.getArray();
    const n = I ? I.length : P.length / 3, at = (i) => (I ? I[i] : i);
    for (let t = 0; t < n; t += 3) res.push([0, 1, 2].map((k) => { const i = at(t + k); return { p: [P[i * 3], P[i * 3 + 1], P[i * 3 + 2]], uv: [U[i * 2], U[i * 2 + 1]] }; }));
  }
  return res;
};
const key = (t) => [0, 1, 2].map((a) => Math.round(((t[0].p[a] + t[1].p[a] + t[2].p[a]) / 3) * 1e4)).join(",");
const old = new Map(); for (const t of await tris(oldF)) old.set(key(t), t);
const out = []; let miss = 0, skip = 0;
for (const t of await tris(newF)) {
  if (t[0].uv[0] < 0) continue;
  const o = old.get(key(t)); if (!o) { miss++; continue; }
  if (o[0].uv[0] < 0) { skip++; continue; }
  const ov = t.map((v) => o.reduce((b, w) => (Math.hypot(...w.p.map((x, a) => x - v.p[a])) < Math.hypot(...b.p.map((x, a) => x - v.p[a])) ? w : b)).uv);
  out.push(...t.flatMap((v) => v.uv), ...ov.flat());
}
fs.writeFileSync(outF, Buffer.from(new Float32Array(out).buffer));
console.log("pairs", out.length / 12, "unmatched", miss, "old had no livery", skip);
