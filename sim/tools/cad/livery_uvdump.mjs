import { NodeIO } from "@gltf-transform/core";
import fs from "node:fs";
const d = await new NodeIO().read(process.argv[2]); const out = [];
for (const m of d.getRoot().listMeshes()) for (const p of m.listPrimitives()) {
  const uv = p.getAttribute("TEXCOORD_0"); if (!uv) continue;
  const U = uv.getArray(), I = p.getIndices()?.getArray(); const n = I ? I.length : U.length / 2;
  for (let t = 0; t < n; t += 3) { const q = [0,1,2].map(k => I ? I[t+k] : t+k); if (U[q[0]*2] < 0) continue; for (const i of q) out.push(U[i*2], U[i*2+1]); }
}
fs.writeFileSync(process.argv[3], Buffer.from(new Float32Array(out).buffer));
fs.writeFileSync(process.argv[3] + ".json", JSON.stringify(d.getRoot().listScenes()[0].getExtras().livery.views));
