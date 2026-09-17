// Launch-screen screenshots: every tab, at a given viewport.
//     node tools/tabshots.mjs <tag> <width> <height>
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
// `npm i -D playwright` in sim/, or point $PLAYWRIGHT at an installed copy.
const { chromium } = await import(process.env.PLAYWRIGHT ?? "playwright");
const here = dirname(fileURLToPath(import.meta.url));
const out = join(here, "..", "docs", "screenshots");
const [tag = "tabs", W = 1600, H = 900] = process.argv.slice(2);
const browser = await chromium.launch({ args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"] });
const page = await browser.newPage({ viewport: { width: Number(W), height: Number(H) }, deviceScaleFactor: 1 });
page.on("pageerror", (e) => console.error("pageerror:", e.message));
await page.goto("http://localhost:5273/index.html");
await page.waitForFunction(() => !document.getElementById("start").disabled, null, { timeout: 60000 });
await page.waitForTimeout(300);
for (const pane of ["course", "car", "controls", "audio", "vehicle"]) {
  await page.click(`.tab[data-pane="${pane}"]`);
  await page.waitForTimeout(150);
  await page.screenshot({ path: join(out, `${tag}-${pane}.png`) });
}
await page.click(`.tab[data-pane="course"]`);
await browser.close();
