// Screenshot harness: home screen plus in-game cockpit and chase views.
//
//     node tools/shots.mjs <tag> [width] [height]
//
// Writes docs/screenshots/<tag>-{home,cockpit,chase}.png. Needs serve.py on
// :5273 and a playwright install (path below is the local npx cache).

import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const PW = "/Users/nmurray/.npm/_npx/e41f203b7505f1fb/node_modules/playwright/index.mjs";
const { chromium } = await import(PW);

const here = dirname(fileURLToPath(import.meta.url));
const out = join(here, "..", "docs", "screenshots");
const tag = process.argv[2] || "shot";
const W = Number(process.argv[3] || 1600);
const H = Number(process.argv[4] || 900);
const track = process.argv[5] || "autocross";

const browser = await chromium.launch({
  args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"],
});
const page = await browser.newPage({ viewport: { width: W, height: H }, deviceScaleFactor: 1 });
page.on("pageerror", (e) => console.error("pageerror:", e.message));
page.on("console", (m) => { if (m.type() === "error") console.error("console:", m.text()); });
await page.goto("http://localhost:5273/index.html");
await page.waitForFunction(() => !document.getElementById("start").disabled, null, { timeout: 60000 });
if (track !== "autocross") {
  await page.selectOption("#track", track);
  await page.waitForFunction(() => !document.getElementById("start").disabled, null, { timeout: 60000 });
}
await page.waitForTimeout(400);
await page.screenshot({ path: join(out, `${tag}-home.png`) });

await page.click("#start");
await page.waitForTimeout(300);

// Put the car mid-course at speed so the views show a lived-in frame.
const shot = async (cameraIndex, name) => {
  await page.evaluate(({ cameraIndex }) => {
    const g = window.__sim;
    g.cameraIndex = cameraIndex;
    const t = g.track;
    const i = Math.min(t.center.length - 1, Math.floor(t.center.length * 0.18));
    const p = t.poseAt(i);
    g.car.respawn(p.x, p.y, p.psi, 14);
    g.camRoll = 0.012; g.camPitch = -0.004;
  }, { cameraIndex });
  await page.waitForTimeout(250);
  await page.screenshot({ path: join(out, `${tag}-${name}.png`) });
};
await shot(0, "cockpit");
await shot(2, "chase");

const perf = await page.evaluate(async () => {
  const g = window.__sim;
  const n = 120;
  const t0 = performance.now();
  for (let i = 0; i < n; i++) g.renderer.draw(g.__lastDraw ?? g.renderer.__last ?? {
    car: { x: g.car.X, y: g.car.Y, psi: g.car.psi, rollRad: 0, pitchRad: 0 },
    view: { ahead: -4.6, height: 1.85, pitchOffset: -0.14, rigid: false },
    hubs: undefined, wheels: { steerRad: 0, spinFront: 0, spinRear: 0, rimFade: 0 }, heaveM: 0, fovBoost: 0,
  });
  const gl = g.renderer.gl; gl.finish();
  return { msPerFrame: (performance.now() - t0) / n, drawCalls: g.renderer.stats?.drawCalls ?? null,
           glError: gl.getError() };
});
console.log(`${tag}: ${JSON.stringify(perf)} (swiftshader, not representative of GPU time)`);
await browser.close();
