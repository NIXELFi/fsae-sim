// Strada (from the STEP) -> data/dash.glb, in the sim's dash frame: centred on
// the DISPLAY, +x across, +y up, +z away from the driver, metres.
//   node build_dash.mjs [screenCentreY] [pushBackM]
import { NodeIO, Document } from "@gltf-transform/core";
import { weld, dedup, prune, join } from "@gltf-transform/functions";
const CY = +(process.argv[2] ?? 0.0482);   // display centre height in the CAD frame
const PUSH = +(process.argv[3] ?? 0.0008);  // behind the live screen quad, no z-fight
const io = new NodeIO();
const src = await io.read("out/strada.glb");
const out = new Document(); const buf = out.createBuffer(); const scene = out.createScene("dash");
// Body roles, by index in the STEP (see strada_dbg_*.png): 3 housing, 0 the
// connector mount, 1 the connector, 2 the button; 4 and 5 are two zero-depth
// sheets over the whole face (glass/decal) that would only z-fight.
const COLOUR = { 3: [0.055, 0.056, 0.06, 1], 0: [0.09, 0.09, 0.095, 1], 1: [0.12, 0.12, 0.13, 1], 2: [0.75, 0.1, 0.1, 1] };
const mats = new Map();
const mat = (c) => { const k = c.join(); if (!mats.has(k)) mats.set(k, out.createMaterial("d" + mats.size).setBaseColorFactor(c).setRoughnessFactor(0.45).setMetallicFactor(0)); return mats.get(k); };
const mesh = out.createMesh("dash");
src.getRoot().listMeshes().forEach((m, i) => {
  if (!(i in COLOUR)) return;
  for (const p of m.listPrimitives()) {
    const P = Float32Array.from(p.getAttribute("POSITION").getArray());
    for (let k = 0; k < P.length; k += 3) { P[k + 1] -= CY; P[k + 2] += PUSH; }
    const q = out.createPrimitive()
      .setAttribute("POSITION", out.createAccessor().setType("VEC3").setArray(P).setBuffer(buf))
      .setIndices(out.createAccessor().setType("SCALAR").setArray(Uint32Array.from(p.getIndices().getArray())).setBuffer(buf))
      .setMaterial(mat(COLOUR[i]));
    const n = p.getAttribute("NORMAL");
    if (n) q.setAttribute("NORMAL", out.createAccessor().setType("VEC3").setArray(Float32Array.from(n.getArray())).setBuffer(buf));
    mesh.addPrimitive(q);
  }
});
scene.addChild(out.createNode("dash").setMesh(mesh));
await out.transform(weld(), dedup(), join({ keepNamed: false }), prune());
await io.write("out/dash.glb", out);
const tris = out.getRoot().listMeshes().reduce((s, m) => s + m.listPrimitives().reduce((t, p) => t + p.getIndices().getCount() / 3, 0), 0);
console.log(JSON.stringify({ tris, kb: Math.round((await import("node:fs")).statSync("out/dash.glb").size / 1024) }));
