// Desktop smoke test: launch the built exe and confirm the GAME booted inside
// it, not merely that a window appeared.
//
// WebView2 accepts Chromium's --remote-debugging-port, so we attach over the
// DevTools protocol and ask the page directly. Without this the only evidence
// a build "works" is that a process stayed alive, which a blank window also
// does.
//
//     node tools/smoke_desktop.mjs

import { spawn, spawnSync } from "node:child_process";
import { dirname, join, relative } from "node:path";
import { existsSync, readdirSync, statSync } from "node:fs";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const exe = join(here, "..", "src-tauri", "target", "release", "fsae-sim.exe");
const PORT = 9222;

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/**
 * One quick look for the app's page target. WebView2 exposes an about:blank
 * target before it navigates, so that one is skipped. Deliberately does NOT
 * retry -- attachWhenReady owns the waiting, and nesting a retry loop inside a
 * retry loop turns a 20 second wait into a twelve minute one.
 */
async function findPage() {
  try {
    const res = await fetch(`http://127.0.0.1:${PORT}/json/list`);
    const targets = await res.json();
    return targets.find((t) =>
      t.type === "page" && t.webSocketDebuggerUrl && t.url && t.url !== "about:blank") ?? null;
  } catch {
    return null; // debug port not up yet
  }
}

/**
 * Attach to the app page and wait until it is genuinely ready.
 *
 * Two traps here, both of which produce confusing failures:
 *   - WebView2 exposes an about:blank target before it navigates.
 *   - A target can report the app URL while the document is still committing,
 *     so attaching then evaluating dies with "Execution context was destroyed".
 * So: find a real target, attach, and only proceed once an evaluate actually
 * succeeds and readyState is complete. Retry the whole thing if it does not.
 */
async function attachWhenReady(deadlineMs = 30000) {
  const until = Date.now() + deadlineMs;
  for (let attempt = 0; Date.now() < until; attempt++) {
    const page = await findPage();
    if (!page) { await sleep(300); continue; }

    let ws;
    try {
      ws = new WebSocket(page.webSocketDebuggerUrl);
      await new Promise((res, rej) => {
        ws.addEventListener("open", res, { once: true });
        ws.addEventListener("error", () => rej(new Error("socket")), { once: true });
        setTimeout(() => rej(new Error("socket timeout")), 4000);
      });
      const state = await evaluate(ws, "document.readyState", 1000 + attempt);
      if (state === "complete" || state === "interactive") return { ws, page };
    } catch { /* context died mid-navigation; try again */ }

    try { ws?.close(); } catch { /* ignore */ }
    await sleep(400);
  }
  return null;
}

function evaluate(ws, expression, id) {
  return new Promise((resolve, reject) => {
    const onMessage = (ev) => {
      const msg = JSON.parse(ev.data);
      if (msg.id !== id) return;
      ws.removeEventListener("message", onMessage);
      if (msg.error) return reject(new Error(JSON.stringify(msg.error)));
      const r = msg.result?.result;
      if (r?.subtype === "error") return reject(new Error(r.description));
      resolve(r?.value);
    };
    ws.addEventListener("message", onMessage);
    ws.send(JSON.stringify({
      id, method: "Runtime.evaluate",
      params: { expression, awaitPromise: true, returnByValue: true },
    }));
    setTimeout(() => reject(new Error("evaluate timed out")), 20000);
  });
}

// Pre-flight: a survivor from a previous run keeps the WebView2 user-data
// folder locked, and a new instance then silently fails to create its webview.
// The symptom is "no ready page", which points nowhere near the real cause.
if (process.platform === "win32") {
  spawnSync("taskkill", ["/IM", "fsae-sim.exe", "/T", "/F"], { stdio: "ignore" });
  await sleep(1200);
}

const child = spawn(exe, [], {
  env: { ...process.env, WEBVIEW2_ADDITIONAL_BROWSER_ARGUMENTS: `--remote-debugging-port=${PORT}` },
  stdio: "ignore",
  detached: false,
});

// The exe must be newer than everything it embeds.
//
// `generate_context!` bakes the frontend in at compile time, so a build that
// failed leaves a perfectly working *old* binary sitting there, and every check
// below will pass against yesterday's code. That is not hypothetical: a build
// that reported `Access is denied (os error 5)` -- a stray WebView2 child still
// holding the file -- was followed by a full-green smoke run reporting audio
// levels from the version before the fix. Silent success on stale artefacts is
// the worst failure mode this harness can have.
{
  const exeTime = statSync(exe).mtimeMs;
  const roots = ["src", "index.html", "data"];
  let newest = { path: null, time: 0 };
  const walk = (p) => {
    const st = statSync(p);
    if (st.isDirectory()) {
      for (const entry of readdirSync(p)) walk(join(p, entry));
    } else if (st.mtimeMs > newest.time) {
      newest = { path: p, time: st.mtimeMs };
    }
  };
  for (const r of roots) {
    const abs = join(here, "..", r);
    if (existsSync(abs)) walk(abs);
  }
  if (newest.time > exeTime) {
    console.error(
      `
STALE BINARY: ${newest.path} is newer than the exe.
` +
      `Rebuild first -- and check the build actually succeeded, because a
` +
      `failed build leaves the previous exe in place and every check below
` +
      `would pass against it.
`,
    );
    process.exit(1);
  }
  console.log(`exe is current (newest source: ${relative(join(here, ".."), newest.path)})`);
}

let code = 1;
try {
  const attached = await attachWhenReady();
  if (!attached) throw new Error("no ready page — WebView2 never loaded the frontend");
  const { ws, page } = attached;
  console.log(`page: ${page.title || "(loading)"}`);
  console.log(`url:  ${page.url}`);

  // Wait for boot, then interrogate the running game.
  const report = await evaluate(ws, `(async () => {
    for (let i = 0; i < 100 && !window.__sim; i++) await new Promise(r => setTimeout(r, 100));
    const g = window.__sim;
    if (!g) return JSON.stringify({ booted: false, error: 'window.__sim never appeared' });
    // Actually simulate and draw a frame inside the desktop webview.
    g.restart();
    for (let i = 0; i < 30; i++) g.update(1/60);
    g.render();
    const gl = g.renderer.gl;
    const veh = document.getElementById('vehicle');
    return JSON.stringify({
      booted: true,
      isDesktop: !!window.__TAURI__,
      bodyDesktopClass: document.body.classList.contains('desktop'),
      specRows: veh ? veh.querySelectorAll('.pgroup tr').length : 0,
      specDof: veh ? veh.querySelectorAll('.dof-group li').length : 0,
      estimateChips: veh ? veh.querySelectorAll('.prov-estimate').length : 0,
      track: g.track.name,
      cones: g.track.cones.length,
      engine: g.powertrain.sourceName,
      peakTorque: g.powertrain.peakTorque.torqueNm,
      etc: g.etc.name + '/' + g.etc.points.length + 'pts',
      webgl2: !!gl,
      renderer: gl.getParameter(gl.VERSION),
      drawingBuffer: [gl.drawingBufferWidth, gl.drawingBufferHeight],
      glError: gl.getError(),
      speedAfter30Frames: +g.car.speed.toFixed(2),
    });
  })()`, 1);

  // The MIS venue is a separate code path -- a different track class, a
  // different mesh builder, and the only barrier collision in the game. Boot
  // the default course and you exercise none of it, so load it explicitly and
  // drive the car at the wall.
  const venue = await evaluate(ws, `(async () => {
    const g = window.__sim;
    await g.load('mis');
    g.restart();
    const t = g.track;
    // Point the car at the foot of the banking and pin the throttle.
    const i = t.segment.indexOf('turn12') + 120;
    const [cx, cy] = t.center[i], h = t.heading[i];
    const nx = Math.sin(h), ny = -Math.cos(h);
    g.car.respawn(cx + nx * (t.barrierOffset - 45), cy + ny * (t.barrierOffset - 45),
                  Math.atan2(ny, nx), 12);
    // Pin the throttle by replacing the poll, which is what update() reads.
    const inp = g.input;
    inp.poll = () => {
      for (const key of Object.keys(inp.edges)) inp.edges[key] = false;
      Object.assign(inp.state, { steer: 0, throttle: 1, brake: 0, launch: false });
      return inp.state;
    };
    let worst = -Infinity;
    for (let k = 0; k < 240; k++) {
      g.update(1 / 60);
      worst = Math.max(worst, t.locate(g.car.X, g.car.Y, g.car.psi).lateral);
    }
    g.render();
    const gl = g.renderer.gl;
    return JSON.stringify({
      name: t.name,
      kind: t.kind,
      lengthM: +t.length.toFixed(1),
      venueTriangles: g.renderer.venue ? g.renderer.venue.count / 3 : 0,
      ribbonDisabled: g.renderer.ribbon === null,
      barrierOffsetM: t.barrierOffset,
      worstLateralM: +worst.toFixed(3),
      heldByBarrier: worst <= t.barrierOffset + 1e-6,
      glError: gl.getError(),
    });
  })()`, 3);
  console.log("venue:", venue);

  // The engine synthesiser runs in an AudioWorklet, and AudioWorklet in
  // WebView2 is exactly the kind of thing that can be missing or refuse to load
  // without saying so -- the game falls back to the oscillator bank and sounds
  // merely worse, which no automated check would otherwise notice. So: start
  // audio, wait for the worklet, and measure the spectrum coming out of it.
  const sound = await evaluate(ws, `(async () => {
    const g = window.__sim;
    g.audio.start();
    for (let i = 0; i < 80 && !g.audio.usingModel && !g.audio.modelError; i++) {
      await new Promise(r => setTimeout(r, 100));
    }
    if (!g.audio.usingModel) {
      return JSON.stringify({ usingModel: false, modelError: g.audio.modelError });
    }

    const ctx = g.audio.ctx;
    const an = ctx.createAnalyser();
    an.fftSize = 8192;
    an.smoothingTimeConstant = 0;   // or the reading carries history
    g.audio.modelGain.connect(an);

    const probe = async (rpm) => {
      g.audio.modelNode.port.postMessage({
        type: 'operating-point', rpm, throttle: 1, torqueNm: 60,
      });
      await new Promise(r => setTimeout(r, 700));
      const bins = new Float32Array(an.frequencyBinCount);
      an.getFloatFrequencyData(bins);
      const hzPer = ctx.sampleRate / an.fftSize;
      let best = { hz: 0, db: -Infinity };
      for (let i = 1; i < bins.length && i * hzPer < 3000; i++) {
        if (bins[i] > best.db) best = { hz: i * hzPer, db: bins[i] };
      }
      return { rpm, expectedHz: rpm / 30, peakHz: +best.hz.toFixed(0), peakDb: +best.db.toFixed(1) };
    };

    const a = await probe(9000);
    const b = await probe(13000);

    // And silence when it is not running.
    g.audio.modelNode.port.postMessage({ type: 'running', running: false });
    await new Promise(r => setTimeout(r, 900));
    const bins = new Float32Array(an.frequencyBinCount);
    an.getFloatFrequencyData(bins);
    let stopped = -Infinity;
    for (const v of bins) stopped = Math.max(stopped, v);

    return JSON.stringify({
      usingModel: true,
      contextRate: ctx.sampleRate,
      probes: [a, b],
      // The loudest component must be an integer multiple of the firing
      // frequency at BOTH operating points.
      //
      // Not "the loudest component IS the firing frequency" -- that is false,
      // and the model is right to make it false. A tuned exhaust can have a
      // harmonic louder than its fundamental depending on where the pipe
      // resonance falls, and at 9000 rpm this one does: the peak sits at
      // 598 Hz, the second harmonic of 300. What must hold is that the peak is
      // locked to the firing rate, and an integer ratio at two different rpms
      // is what a fixed resonance could not fake.
      tracksFiringFrequency: [a, b].every((p) => {
        const ratio = p.peakHz / p.expectedHz;
        const n = Math.round(ratio);
        return n >= 1 && Math.abs(ratio - n) < 0.08;
      }),
      stoppedPeakDb: +stopped.toFixed(1),
    });
  })()`, 5);
  console.log("audio:", sound);

  // The controls panel is generated from a description rather than hand-built,
  // so a mistake shows up as an empty panel rather than an exception.
  const controls = await evaluate(ws, `(() => {
    const g = window.__sim;
    const root = document.getElementById('controls');
    const sel = document.getElementById('controlProfile');
    if (!root || !sel) return JSON.stringify({ mounted: false });
    sel.value = 'wheel';
    sel.dispatchEvent(new Event('change'));
    const after = document.getElementById('controls');
    const rows = [...after.querySelectorAll('.ctl-row')];
    const rateRow = rows.find(r => r.querySelector('label')?.textContent === 'Max steering speed');
    let reached = null;
    if (rateRow) {
      const slider = rateRow.querySelector('input[type=range]');
      slider.value = '500';
      slider.dispatchEvent(new Event('input'));
      g.update(1 / 60);
      reached = g.car.steeringServo.maxRateDegPerS;
    }
    sel.value = 'keyboard';
    sel.dispatchEvent(new Event('change'));
    return JSON.stringify({
      mounted: true,
      profiles: [...sel.options].map(o => o.value),
      wheelRows: rows.length,
      editReachedTheCar: reached === 500,
    });
  })()`, 6);
  console.log("controls:", controls);

  console.log("game:", report);

  const errors = await evaluate(ws, `JSON.stringify(window.__errors || [])`, 7);
  console.log("page errors:", errors);

  const parsed = JSON.parse(report);
  const v = JSON.parse(venue);
  const a = JSON.parse(sound);
  const c = JSON.parse(controls);
  code = parsed.booted && parsed.webgl2 && parsed.glError === 0 &&
         parsed.specRows > 0 && parsed.specDof > 0 &&
         v.kind === 'venue' && v.venueTriangles > 1000 && v.ribbonDisabled &&
         v.heldByBarrier && v.glError === 0 &&
         a.usingModel && a.tracksFiringFrequency && a.stoppedPeakDb < -60 &&
         c.mounted && c.editReachedTheCar ? 0 : 1;
  ws.close();
} catch (err) {
  console.error("FAILED:", err.message);
} finally {
  // child.kill() terminates the Tauri host but leaves WebView2's child
  // processes running, and a survivor keeps holding the debug port -- which
  // makes the NEXT run fail with "no debuggable page" for no visible reason.
  // Kill the whole tree.
  if (process.platform === "win32") {
    spawnSync("taskkill", ["/PID", String(child.pid), "/T", "/F"], { stdio: "ignore" });
  } else {
    child.kill("SIGKILL");
  }
  await sleep(700);
}

console.log(code === 0 ? "\nDESKTOP SMOKE TEST PASSED\n" : "\nDESKTOP SMOKE TEST FAILED\n");
process.exit(code);
