// The two car appearances (team CAD / classic) must differ ONLY in how the
// car looks: this drives the browser build with a hand-pumped, fixed 1/60 s
// frame clock and scripted inputs, once per appearance (and once switching
// mid-drive), and compares the car's state, the timing, the cone hits and the
// run log frame by frame. Any difference fails.
//
//   npm run serve (or python tools/serve.py 5391), then
//   node tools/test_visual_car.mjs [http://localhost:5391]
//
// Needs Playwright (npx playwright) and a data/car.glb to compare against.
import { chromium } from "playwright";

const BASE = process.argv[2] ?? "http://localhost:5391";
const FRAMES = 1500;

async function drive(label, appearance, switchAt = -1) {
  const b = await chromium.launch({ args: ["--use-angle=d3d11", "--enable-gpu", "--ignore-gpu-blocklist"] });
  const p = await b.newPage({ viewport: { width: 960, height: 540 } });
  const errors = [];
  p.on("pageerror", (e) => errors.push(e.message));
  await p.addInitScript((appearance) => {
    localStorage.setItem("fsae.visualCar", appearance);
    // No real devices: a wheel on the desk would otherwise put its own
    // steering into the log (the Gamepad API sees it), different every run.
    navigator.getGamepads = () => [];
    // Take the frame clock over once booted (`__manual`): until then frames
    // run as normal, because the boot itself waits on them. After, callbacks
    // queue and the test pumps them with a fixed step.
    window.__rafQ = [];
    const real = window.requestAnimationFrame.bind(window);
    window.requestAnimationFrame = (cb) => {
      if (!window.__manual) return real(cb);
      window.__rafQ.push(cb); return window.__rafQ.length;
    };
    // A fixed start, so every run's frame times -- and their float rounding --
    // are the same numbers.
    window.__takeOver = () => { window.__manual = true; window.__now = 1e6; };
    window.__pump = (n, dtMs) => {
      for (let i = 0; i < n; i++) {
        window.__now = (window.__now ?? 0) + dtMs;
        const q = window.__rafQ; window.__rafQ = [];
        for (const cb of q) cb(window.__now);
      }
    };
  }, appearance);
  await p.goto(`${BASE}/?track=autocross`, { waitUntil: "load" });
  await p.waitForFunction(() => window.__sim?.car && !document.querySelector("#start")?.disabled, null, { timeout: 60000 });
  const trace = await p.evaluate(async ({ FRAMES, switchAt }) => {
    const g = window.__sim, car = g.car;
    window.__takeOver();
    // Two pumped frames with the menu still up (no physics) so the loop's
    // own clock is on the fixed step before the first driven frame.
    await new Promise((r) => setTimeout(r, 100));
    window.__pump(2, 1000 / 60);
    document.querySelector("#start").click();
    // Let the start's own async steps land before the first pumped frame.
    await new Promise((r) => setTimeout(r, 300));
    let frame = 0;
    const orig = car.step.bind(car);
    // Inputs by frame number, never by time: pull away, weave through the
    // cones, brake, turn -- enough to hit cones and change laps state.
    car.step = (dt) => {
      const f = frame;
      const steer = f < 120 ? 0 : Math.sin(f / 35) * 0.9;
      const throttle = f < 900 ? 0.6 : f < 1100 ? 0 : 0.4;
      const brake = f >= 900 && f < 1100 ? 0.8 : 0;
      orig(dt, { steer, throttle, brake });
    };
    const out = [];
    for (; frame < FRAMES; frame++) {
      if (frame === switchAt) {
        g.visualCar = g.visualCar === "classic" ? "cad" : "classic";
        g.applyVisualCar();
      }
      window.__pump(1, 1000 / 60);
      await null; // let microtasks between frames run, as they would live
      if (frame % 25 === 0 || frame === FRAMES - 1) {
        const t = g.timing, rec = g.recorder;
        out.push([frame, car.X, car.Y, car.psi, car.u, car.v, car.r, car.delta, car.wF, car.wR,
          t?.state, t?.elapsed, t?.cones, t?.offCourse, t?.lap, rec?.samples ?? null,
          g.track.cones.filter((c) => c.down).length]);
      }
    }
    if (g.clock <= 0) throw new Error("the game never ran a frame -- nothing was compared");
    // The whole run log, every channel of every sample, as a hash.
    // -0 and 0 are the same number: the steering input's smoothing lands on
    // either from run to run, whatever the car looks like.
    const csv = (g.recorder?.toCsv?.() ?? "").replace(/(^|,)-0(?=,|$)/gm, "$10");
    let h = 2166136261;
    for (let i = 0; i < csv.length; i++) { h ^= csv.charCodeAt(i); h = Math.imul(h, 16777619) >>> 0; }
    out.push(["log", csv.length, h.toString(16)]);
    return { out, cad: !!g.renderer?.carModel, rig: g.renderer?.rigParts?.length ?? 0, status: g.cadStatus };
  }, { FRAMES, switchAt });
  await b.close();
  console.log(`  ${label}: drawn with ${trace.cad ? "the CAD car" : "the classic car"} (rig parts ${trace.rig}); ${errors.length} page errors`);
  if (errors.length) console.log("   ", errors.slice(0, 3).join(" | "));
  return trace;
}

const cad = await drive("team CAD", "cad");
const classic = await drive("classic", "classic");
const switched = await drive("CAD, switched to classic mid-drive", "cad", 700);

let fails = 0;
const compare = (name, a, b) => {
  let first = null;
  for (let i = 0; i < a.out.length; i++) {
    if (JSON.stringify(a.out[i]) !== JSON.stringify(b.out[i])) { first = i; break; }
  }
  if (first === null) console.log(`  [PASS] ${name}: identical over ${FRAMES} frames (${a.out.length} checkpoints)`);
  else { fails++; console.log(`  [FAIL] ${name}: differ at frame ${a.out[first][0]}\n    ${JSON.stringify(a.out[first])}\n    ${JSON.stringify(b.out[first])}`); }
};
const last = cad.out[cad.out.length - 2];
console.log(`  run log: ${cad.out[cad.out.length - 1][1].toLocaleString()} characters, hash ${cad.out[cad.out.length - 1][2]}`);
console.log(`  drive: ${last[16]} cones down (${cad.out[0][16]} at the start), timing ${last[10]}, ${last[15]} samples logged, ${Math.hypot(last[1], last[2]).toFixed(1)} m from the origin`);
if (!cad.cad) { fails++; console.log("  [FAIL] no CAD car was loaded, so there was nothing to compare"); }
compare("CAD vs classic", cad, classic);
compare("CAD vs switching appearance mid-drive", cad, switched);
console.log(fails ? `\n${fails} FAILED` : "\nALL CHECKS PASSED");
process.exit(fails ? 1 : 0);
