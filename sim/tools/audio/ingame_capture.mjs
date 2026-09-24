#!/usr/bin/env node
// Record the GAME's full audio mix for a recorded run: engine worklet, tyres,
// wind, road, drivetrain, shift clunks -- everything src/game/audio.js makes --
// by playing the run through the game's own replay audio path in a headless
// browser and capturing the output of the master limiter in real time.
//
//     node ingame_capture.mjs --run <runId> [--camera Cockpit|Chase] [--out file.wav]
//                             [--url http://localhost:5391] [--master 0.5]
//
// Needs the browser build served (python tools/serve.py <port>) and
// Playwright: $PLAYWRIGHT = path to playwright's index.mjs (or `npm i -D
// playwright` in sim/), $CHROME = a Chromium executable if Playwright's own
// is not installed. Real time: a 40 s run takes 40 s.
//
// The state fed to the audio each tick is `game.replayAudioState()` over the
// real `Replay` of the run, i.e. exactly what a replay in the app plays. The
// limiter's gain reduction is reported so level changes can be checked.

import fs from "node:fs";
import path from "node:path";

const args = Object.fromEntries(process.argv.slice(2).reduce((a, v, i, all) => {
  if (v.startsWith("--")) a.push([v.slice(2), all[i + 1] && !all[i + 1].startsWith("--") ? all[i + 1] : true]);
  return a;
}, []));
const url = args.url ?? "http://localhost:5391";
const run = args.run;
if (!run) throw new Error("--run <runId> required");
const out = path.resolve(args.out ?? `out/ingame_${run}.wav`);
const camera = args.camera ?? "Cockpit";
const master = Number(args.master ?? 0.5);
// --solo engine|others: mute everything but the engine model, or the engine
// model alone, to measure what the rest of the mix contributes.
const solo = args.solo ?? null;
// --engine-config '{"intakeNoise":{...}}': rebuild the game's engine worklet
// with this EngineAudio config (prototypes that live in the model's config).
const engineConfig = args["engine-config"] ? JSON.parse(args["engine-config"]) : null;

const pw = await import(process.env.PLAYWRIGHT ?? "playwright");
const browser = await pw.chromium.launch({
  headless: true,
  ...(process.env.CHROME ? { executablePath: process.env.CHROME } : {}),
  args: ["--autoplay-policy=no-user-gesture-required"],
});
const page = await browser.newPage();
const errors = [];
page.on("pageerror", (e) => errors.push(String(e)));
page.on("console", (m) => { if (m.type() === "error" && !/404/.test(m.text())) errors.push(m.text()); });
await page.goto(`${url}/index.html`, { waitUntil: "load" });
await page.waitForFunction(() => window.__sim && window.__sim.audio && window.__sim.powertrain, null, { timeout: 60000 });

const res = await page.evaluate(async ({ run, camera, master, solo, engineConfig }) => {
  const game = window.__sim;
  const a = game.audio;
  const { Replay } = await import("/src/game/replay.js");
  const { parseTelemetry } = await import("/src/game/runStore.js");
  const [manifest, text] = await Promise.all([
    fetch(`/runs/${run}/run.json`).then((r) => (r.ok ? r.json() : {})),
    fetch(`/runs/${run}/telemetry.csv`).then((r) => r.text()),
  ]);
  const replay = new Replay(manifest, parseTelemetry(text));
  a.start();
  for (let i = 0; i < 100 && !a.usingModel; i++) await new Promise((r) => setTimeout(r, 100));
  const ctx = a.ctx;
  if (ctx.state !== "running") await ctx.resume();
  a.master.gain.value = master;
  if (engineConfig) {
    const node = new AudioWorkletNode(ctx, "engine-processor", {
      numberOfInputs: 0, numberOfOutputs: 1, outputChannelCount: [1],
      processorOptions: { engine: "cbr600rr-sdm26", config: engineConfig },
    });
    a.modelNode.disconnect();
    node.connect(a.helmet ?? a.modelGain);
    a.modelNode = node;
  }
  a.setCamera(camera);
  if (solo === "others") a.modelGain.gain.value = 0;
  if (solo === "engine") {
    for (const b of [a.scrubF, a.scrubR, a.squeal, a.spin, a.rolling, a.wind, a.wind2, a.windHiss, a.road, a.lockup, a.surface, a.induction])
      if (b) b.g.disconnect();
    if (a.driveBus) a.driveBus.disconnect();
    a.shiftClunk = () => {};
  }
  await new Promise((r) => setTimeout(r, 1200));   // start-up sounds pass

  // capture: limiter -> ScriptProcessor (stereo) -> silent sink
  const L = [], R = [];
  let recording = false;
  const sp = ctx.createScriptProcessor(4096, 2, 2);
  sp.onaudioprocess = (e) => {
    if (!recording) return;
    L.push(new Float32Array(e.inputBuffer.getChannelData(0)));
    R.push(new Float32Array(e.inputBuffer.getChannelData(1)));
  };
  const sink = ctx.createGain(); sink.gain.value = 0;
  a.limiter.connect(sp); sp.connect(sink); sink.connect(ctx.destination);

  // The game's own loop keeps calling audio.update() with the live car's
  // state (idling on the grid behind the menu); left alone that interleaves
  // with the replay's operating points and the engine hunts between the two.
  // Take update() away from the game for the capture.
  const update = a.update.bind(a);
  a.update = () => {};
  const view = { replay, powertrain: game.powertrain };
  let minRed = 0;
  const t0 = ctx.currentTime + 0.05;
  let last = t0;
  recording = true;
  await new Promise((resolve) => {
    const id = setInterval(() => {
      const now = ctx.currentTime;
      const t = now - t0;
      if (t >= replay.duration) { clearInterval(id); resolve(); return; }
      replay.seek(Math.max(t, 0));
      update(game.replayAudioState.call(view), Math.max(now - last, 1e-3));
      last = now;
      minRed = Math.min(minRed, a.limiter.reduction);
    }, 8);
  });
  recording = false;
  const n = L.reduce((s, b) => s + b.length, 0);
  const pcm = new Int16Array(n * 2);
  let k = 0, peak = 0;
  for (let b = 0; b < L.length; b++) {
    for (let i = 0; i < L[b].length; i++) {
      const l = L[b][i], r = R[b][i];
      peak = Math.max(peak, Math.abs(l), Math.abs(r));
      pcm[k++] = Math.max(-1, Math.min(1, l)) * 32767;
      pcm[k++] = Math.max(-1, Math.min(1, r)) * 32767;
    }
  }
  // base64 in chunks
  const u8 = new Uint8Array(pcm.buffer);
  let s = "";
  for (let i = 0; i < u8.length; i += 0x8000) s += String.fromCharCode(...u8.subarray(i, i + 0x8000));
  return { b64: btoa(s), rate: ctx.sampleRate, frames: n, peak, minLimiterDb: minRed, source: a.engineSource() };
}, { run, camera, master, solo, engineConfig });

const pcm = Buffer.from(res.b64, "base64");
const hdr = Buffer.alloc(44);
hdr.write("RIFF", 0); hdr.writeUInt32LE(36 + pcm.length, 4); hdr.write("WAVE", 8);
hdr.write("fmt ", 12); hdr.writeUInt32LE(16, 16); hdr.writeUInt16LE(1, 20); hdr.writeUInt16LE(2, 22);
hdr.writeUInt32LE(res.rate, 24); hdr.writeUInt32LE(res.rate * 4, 28); hdr.writeUInt16LE(4, 32); hdr.writeUInt16LE(16, 34);
hdr.write("data", 36); hdr.writeUInt32LE(pcm.length, 40);
fs.mkdirSync(path.dirname(out), { recursive: true });
fs.writeFileSync(out, Buffer.concat([hdr, pcm]));
console.log(JSON.stringify({ out, seconds: +(res.frames / res.rate).toFixed(2), peakDbfs: +(20 * Math.log10(res.peak || 1e-9)).toFixed(1),
  limiterMinDb: +res.minLimiterDb.toFixed(1), engineSource: res.source, pageErrors: errors }));
await browser.close();
