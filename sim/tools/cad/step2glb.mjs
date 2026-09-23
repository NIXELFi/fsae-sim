// Fetch the Strada STEP from the vault, tessellate it with OpenCascade (WASM),
// and write a GLB with one mesh per STEP face-group, keeping names and colours.
//   node step2glb.mjs <sha256> <out.glb> [linearDeflectionM]
import fs from "node:fs";
import zlib from "node:zlib";
import occtimportjs from "occt-import-js";
import { Document, NodeIO } from "@gltf-transform/core";
const [, , sha, OUT, defl = "0.0002"] = process.argv;
const res = await fetch(`${process.env.SUPABASE_URL}/storage/v1/object/vault-objects/${sha.slice(0, 2)}/${sha}`,
  { headers: { Authorization: `Bearer ${process.env.SUPABASE_SERVICE_KEY}`, apikey: process.env.SUPABASE_SERVICE_KEY } });
let buf = Buffer.from(await res.arrayBuffer());
if (buf[0] === 0x1f) buf = zlib.gunzipSync(buf);
fs.writeFileSync("raw/strada.step", buf);
const occt = await occtimportjs();
const r = occt.ReadStepFile(new Uint8Array(buf), {
  linearUnit: "meter", linearDeflectionType: "absolute_value", linearDeflection: +defl, angularDeflection: 0.2,
});
if (!r.success) throw new Error("STEP read failed");
const doc = new Document(); const b = doc.createBuffer(); const scene = doc.createScene();
const mats = new Map();
const mat = (c) => {
  const k = (c ?? [0.3, 0.3, 0.3]).map((v) => v.toFixed(3)).join(",");
  if (!mats.has(k)) mats.set(k, doc.createMaterial("m" + mats.size).setBaseColorFactor([...(c ?? [0.3, 0.3, 0.3]), 1]).setRoughnessFactor(0.5).setMetallicFactor(0));
  return mats.get(k);
};
let tris = 0;
const names = [];
r.meshes.forEach((m, i) => {
  const pos = new Float32Array(m.attributes.position.array);
  const nrm = m.attributes.normal ? new Float32Array(m.attributes.normal.array) : null;
  const idx = new Uint32Array(m.index.array);
  // Per-face colours when the STEP carries them; else the mesh colour.
  const groups = m.brep_faces?.length ? m.brep_faces : [{ first: 0, last: idx.length / 3 - 1, color: m.color }];
  const mesh = doc.createMesh(m.name || `part${i}`);
  const byColour = new Map();
  for (const g of groups) {
    const c = g.color ?? m.color;
    const k = JSON.stringify(c);
    if (!byColour.has(k)) byColour.set(k, { c, tri: [] });
    for (let t = g.first; t <= g.last; t++) byColour.get(k).tri.push(idx[3 * t], idx[3 * t + 1], idx[3 * t + 2]);
  }
  for (const { c, tri } of byColour.values()) {
    const p = doc.createPrimitive()
      .setAttribute("POSITION", doc.createAccessor().setType("VEC3").setArray(pos).setBuffer(b))
      .setIndices(doc.createAccessor().setType("SCALAR").setArray(new Uint32Array(tri)).setBuffer(b))
      .setMaterial(mat(c));
    if (nrm) p.setAttribute("NORMAL", doc.createAccessor().setType("VEC3").setArray(nrm).setBuffer(b));
    mesh.addPrimitive(p);
    tris += tri.length / 3;
  }
  names.push(`${m.name} [${idx.length / 3} tris, colours ${[...byColour.values()].map((v) => JSON.stringify(v.c?.map((x) => +x.toFixed(2)))).join(" ")}]`);
  scene.addChild(doc.createNode(m.name || `part${i}`).setMesh(mesh));
});
await new NodeIO().write(OUT, doc);
console.log(JSON.stringify({ meshes: r.meshes.length, tris, out: OUT, mb: +(fs.statSync(OUT).size / 1e6).toFixed(2) }));
console.log(names.join("\n"));
