// Per livery triangle: its UVs and how much it faces sideways (|n.y|), for
// placing stickers where the panel is flat and faces out.
//   node livery_sidemask.mjs car.glb out.bin  -> Float32 [u0,v0,u1,v1,u2,v2,|ny|]
import { NodeIO } from "@gltf-transform/core";
import fs from "node:fs";
const d = await new NodeIO().read(process.argv[2]); const out = [];
for (const m of d.getRoot().listMeshes()) for (const p of m.listPrimitives()) {
  const uv = p.getAttribute("TEXCOORD_0"); if (!uv) continue;
  const U = uv.getArray(), P = p.getAttribute("POSITION").getArray(), I = p.getIndices()?.getArray();
  const n = I ? I.length : U.length / 2, at = (i) => (I ? I[i] : i);
  for (let t = 0; t < n; t += 3) {
    const q = [0, 1, 2].map((k) => at(t + k)); if (U[q[0] * 2] < 0) continue;
    const a = q.map((i) => [P[i * 3], P[i * 3 + 1], P[i * 3 + 2]]);
    const e1 = a[1].map((x, k) => x - a[0][k]), e2 = a[2].map((x, k) => x - a[0][k]);
    const c = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
    const l = Math.hypot(...c) || 1;
    for (const i of q) out.push(U[i * 2], U[i * 2 + 1]);
    out.push(Math.abs(c[1]) / l);
  }
}
fs.writeFileSync(process.argv[3], Buffer.from(new Float32Array(out).buffer));
