// Livery templates from a built car.glb (its TEXCOORD_0 and scene extras
// `livery`): a paint template with the view frames, labels and the panels'
// wireframe; a blank (fully transparent) livery; and a test livery that
// shows the mapping on the car.
//   node livery_template.mjs <car.glb> <outDir> [size]
import { NodeIO } from "@gltf-transform/core";
import { chromium } from "file:///C:/Users/nick5/AppData/Local/npm-cache/_npx/e41f203b7505f1fb/node_modules/playwright/index.mjs";
import fs from "node:fs";
const [, , file = "out/car_uv.glb", outDir = "out/livery", sizeArg = "4096"] = process.argv;
const SIZE = +sizeArg;
fs.mkdirSync(outDir, { recursive: true });
const doc = await new NodeIO().read(file);
const extras = doc.getRoot().listScenes()[0].getExtras();
const views = extras.livery.views;
const segs = [];
for (const mesh of doc.getRoot().listMeshes()) for (const p of mesh.listPrimitives()) {
  const uv = p.getAttribute("TEXCOORD_0"); if (!uv) continue;
  const a = uv.getArray(), idx = p.getIndices()?.getArray();
  const n = idx ? idx.length : a.length / 2;
  const at = (i) => (idx ? idx[i] : i);
  for (let t = 0; t < n; t += 3) {
    const q = [0, 1, 2].map((k) => [a[at(t + k) * 2], a[at(t + k) * 2 + 1]]);
    segs.push(q);
  }
}
console.log("livery triangles", segs.length, "views", Object.keys(views).join(","));
const b = await chromium.launch();
const page = await b.newPage();
const draw = async (mode) => page.evaluate(({ SIZE, views, segs, mode, meters }) => {
  const c = document.createElement("canvas"); c.width = c.height = SIZE;
  const g = c.getContext("2d");
  const S = SIZE;
  if (mode === "template") { g.fillStyle = "#ffffff"; g.fillRect(0, 0, S, S); }
  const NAMES = { left: "LEFT SIDE  (nose \u2190)", right: "RIGHT SIDE  (nose \u2192)", top: "TOP  (nose \u2190)", bottom: "BOTTOM  (nose \u2192)", front: "FRONT", rear: "REAR" };
  const HUE = { left: 0, right: 30, top: 200, bottom: 280, front: 120, rear: 60 };
  for (const [k, [x, y, w, h]] of Object.entries(views)) {
    if (mode === "test") {
      // A numbered grid, one 10 cm cell at a time, tinted per view.
      const cell = 0.10 / meters;             // texture fraction per 10 cm
      let n = 0;
      for (let gy = y; gy < y + h; gy += cell) for (let gx = x; gx < x + w; gx += cell) {
        const i = Math.round((gx - x) / cell), j = Math.round((gy - y) / cell);
        g.fillStyle = `hsl(${HUE[k]}, 70%, ${(i + j) % 2 ? 45 : 70}%)`;
        g.fillRect(gx * S, gy * S, cell * S + 1, cell * S + 1);
        if ((i + j) % 4 === 0) { g.fillStyle = "#000"; g.font = `${Math.round(cell * S * 0.35)}px sans-serif`; g.fillText(String(n), gx * S + 4, gy * S + cell * S * 0.5); }
        n++;
      }
      g.fillStyle = "rgba(255,255,255,0.9)"; g.font = `bold ${Math.round(h * S * 0.22)}px sans-serif`;
      g.fillText(k.toUpperCase(), (x + w * 0.05) * S, (y + h * 0.6) * S);
    }
    if (mode === "template") {
      g.strokeStyle = "#1a73e8"; g.lineWidth = Math.max(2, S / 1024);
      g.strokeRect(x * S, y * S, w * S, h * S);
      // In the margin above the frame, clear of the panels.
      g.fillStyle = "#1a73e8"; g.font = `bold ${Math.round(S / 120)}px sans-serif`;
      g.fillText(NAMES[k], x * S + 4, y * S - S / 400);
    }
  }
  if (mode === "template" || mode === "test") {
    g.strokeStyle = mode === "template" ? "rgba(0,0,0,0.55)" : "rgba(0,0,0,0.25)";
    g.lineWidth = Math.max(1, S / 4096);
    g.beginPath();
    for (const q of segs) { g.moveTo(q[0][0] * S, q[0][1] * S); g.lineTo(q[1][0] * S, q[1][1] * S); g.lineTo(q[2][0] * S, q[2][1] * S); g.closePath(); }
    g.stroke();
  }
  if (mode === "template") {
    // In the empty corner beside TOP and BOTTOM.
    const fx = views.front[0], fy = views.top[1] + 0.01;
    g.fillStyle = "#333"; g.font = `bold ${Math.round(S / 80)}px sans-serif`;
    g.fillText("SDM26 livery template", fx * S, fy * S + S / 60);
    g.fillStyle = "#555"; g.font = `${Math.round(S / 125)}px sans-serif`;
    const lines = [
      `${SIZE} px = ${meters.toFixed(2)} m, ${(meters * 1000 / SIZE).toFixed(2)} mm per px,`,
      "one scale for every view.",
      "",
      "Paint on a layer above this one, then export",
      "THAT layer alone as data/livery.png:",
      "PNG with alpha, square, any size.",
      "",
      "Transparent = bare carbon.",
      "Each view is drawn as you would see it standing",
      "there, so text reads correctly on both sides.",
      "",
      "A panel belongs to the view it faces most; paint",
      "a stripe across a view edge in both views.",
    ];
    lines.forEach((t, i) => g.fillText(t, fx * S, fy * S + S / 30 + i * S / 95));
  }
  return c.toDataURL("image/png");
}, { SIZE, views, segs, mode, meters: extras.livery.size });
for (const [mode, name] of [["template", "livery_template.png"], ["blank", "livery_blank.png"], ["test", "livery_test.png"]]) {
  const url = await draw(mode);
  fs.writeFileSync(`${outDir}/${name}`, Buffer.from(url.split(",")[1], "base64"));
  console.log(name, (fs.statSync(`${outDir}/${name}`).size / 1024).toFixed(0), "KB");
}
await b.close();
