// Dump a GLB's structure: node tree (names, depth), triangles per subtree,
// world-space bounds, materials and textures. Reads the JSON chunk and the
// accessor min/max only, so it is fast on huge files.
import fs from "node:fs";
const file = process.argv[2];
const maxDepth = +(process.argv[3] ?? 4);
const buf = fs.readFileSync(file);
const jsonLen = buf.readUInt32LE(12);
const doc = JSON.parse(buf.subarray(20, 20 + jsonLen).toString("utf8"));
const acc = doc.accessors ?? [];
const meshTris = (m) => (doc.meshes[m].primitives ?? []).reduce((s, p) => {
  if (p.indices != null) return s + acc[p.indices].count / 3;
  return s + acc[p.attributes.POSITION].count / 3;
}, 0);
// world matrices
const mul = (a, b) => { const o = new Array(16).fill(0); for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) for (let k = 0; k < 4; k++) o[j * 4 + i] += a[k * 4 + i] * b[j * 4 + k]; return o; };
const local = (n) => {
  if (n.matrix) return n.matrix;
  const [x, y, z, w] = n.rotation ?? [0, 0, 0, 1];
  const [sx, sy, sz] = n.scale ?? [1, 1, 1];
  const [tx, ty, tz] = n.translation ?? [0, 0, 0];
  return [
    (1 - 2 * (y * y + z * z)) * sx, 2 * (x * y + z * w) * sx, 2 * (x * z - y * w) * sx, 0,
    2 * (x * y - z * w) * sy, (1 - 2 * (x * x + z * z)) * sy, 2 * (y * z + x * w) * sy, 0,
    2 * (x * z + y * w) * sz, 2 * (y * z - x * w) * sz, (1 - 2 * (x * x + y * y)) * sz, 0,
    tx, ty, tz, 1];
};
const I = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
const xf = (m, p) => [0, 1, 2].map((i) => m[i] * p[0] + m[4 + i] * p[1] + m[8 + i] * p[2] + m[12 + i]);
const lines = [];
let totalTris = 0;
const g = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] };
function walk(ni, parentM, depth) {
  const n = doc.nodes[ni];
  const m = mul(parentM, local(n));
  let tris = 0; const bb = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] };
  if (n.mesh != null) {
    tris += meshTris(n.mesh);
    for (const p of doc.meshes[n.mesh].primitives) {
      const a = acc[p.attributes.POSITION];
      if (!a.min) continue;
      for (const cx of [a.min[0], a.max[0]]) for (const cy of [a.min[1], a.max[1]]) for (const cz of [a.min[2], a.max[2]]) {
        const w = xf(m, [cx, cy, cz]);
        for (let i = 0; i < 3; i++) { bb.min[i] = Math.min(bb.min[i], w[i]); bb.max[i] = Math.max(bb.max[i], w[i]); }
      }
    }
  }
  const idx = lines.length; lines.push(null);
  for (const c of n.children ?? []) {
    const r = walk(c, m, depth + 1);
    tris += r.tris;
    for (let i = 0; i < 3; i++) { bb.min[i] = Math.min(bb.min[i], r.bb.min[i]); bb.max[i] = Math.max(bb.max[i], r.bb.max[i]); }
  }
  const size = bb.min[0] < Infinity ? bb.max.map((v, i) => (v - bb.min[i]).toFixed(3)).join("x") : "-";
  const ctr = bb.min[0] < Infinity ? bb.max.map((v, i) => ((v + bb.min[i]) / 2).toFixed(3)).join(",") : "-";
  lines[idx] = depth <= maxDepth ? `${"  ".repeat(depth)}${n.name ?? "(unnamed)"}  [${Math.round(tris).toLocaleString()} tris, size ${size}, centre ${ctr}]` : null;
  return { tris, bb };
}
for (const root of doc.scenes[doc.scene ?? 0].nodes) {
  const r = walk(root, I, 0);
  totalTris += r.tris;
  for (let i = 0; i < 3; i++) { g.min[i] = Math.min(g.min[i], r.bb.min[i]); g.max[i] = Math.max(g.max[i], r.bb.max[i]); }
}
console.log(`== ${file.split(/[\\/]/).pop()}: ${(buf.length / 1e6).toFixed(1)} MB, ${doc.nodes?.length ?? 0} nodes, ${doc.meshes?.length ?? 0} meshes, ${Math.round(totalTris).toLocaleString()} tris`);
console.log(`   bounds ${g.min.map((v) => v.toFixed(3))} .. ${g.max.map((v) => v.toFixed(3))}`);
console.log(`   materials ${doc.materials?.length ?? 0}, textures ${doc.textures?.length ?? 0}, images ${doc.images?.length ?? 0}, extensions ${JSON.stringify(doc.extensionsUsed ?? [])}, generator ${doc.asset?.generator}`);
const mats = (doc.materials ?? []).map((m) => `${m.name ?? "?"}:${(m.pbrMetallicRoughness?.baseColorFactor ?? [1, 1, 1, 1]).map((x) => x.toFixed(2)).join("/")}${m.pbrMetallicRoughness?.baseColorTexture ? "+tex" : ""}`);
console.log("   " + mats.slice(0, 40).join("  ") + (mats.length > 40 ? ` ... (+${mats.length - 40})` : ""));
console.log(lines.filter(Boolean).join("\n"));
