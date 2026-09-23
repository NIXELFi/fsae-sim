// List every mesh node with its world bounds, flagging ones that sit below the
// ground or away from where the car is.
import { NodeIO } from "@gltf-transform/core";
import { getBounds } from "@gltf-transform/core";
const io = new NodeIO();
const doc = await io.read(process.argv[2]);
const root = doc.getRoot();
const scene = root.listScenes()[0];
const rows = [];
scene.traverse((n) => {
  if (!n.getMesh()) return;
  const b = getBounds(n);
  const c = b.min.map((v, i) => (v + b.max[i]) / 2);
  const flag = b.min[2] < -0.02 ? "BELOW-GROUND" : "";
  const tris = n.getMesh().listPrimitives().reduce((s, p) => s + (p.getIndices()?.getCount() ?? p.getAttribute("POSITION").getCount()) / 3, 0);
  const path = []; let q = n; while (q) { path.unshift(q.getName()); q = q.getParentNode(); }
  rows.push({ path: path.slice(-3).join(" > "), c: c.map((v) => v.toFixed(3)).join(","), zmin: b.min[2].toFixed(3), tris, flag });
});
const only = process.argv[3];
for (const r of rows) if (!only || (only === "flag" ? r.flag : r.path.match(new RegExp(only, "i")))) console.log(`${r.flag.padEnd(13)} ${String(Math.round(r.tris)).padStart(7)}  c=(${r.c}) zmin=${r.zmin}  ${r.path}`);
