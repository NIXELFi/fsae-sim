#!/usr/bin/env node
// Offline render of the sim's engine-sound model to a WAV, plus a sidecar
// JSON of the operating points that drove it.
//
// It imports src/audio/engineAudio.js directly, so it always renders the LIVE
// model -- whatever is on disk right now -- and never a copy. The operating
// point mapping mirrors the game's replay path (main.js replayAudioState ->
// game/audio.js update): throttle = plate (engine.tps), torque = the indicated
// torque plate * (wot + motoring), cut on the limiter, overrun on a closed
// plate above 6000 rpm at speed, and zero throttle/torque while shifting.
//
// Only the physical engine model is rendered. The game's WebAudio layers --
// intake/induction noise, tyre squeal, wind, shift clunk, master mix -- are not
// in it and are NOT here.
//
// Usage:
//   node render_sim.mjs --csv <telemetry.csv> [--out out/run.wav] [--start s] [--dur s]
//   node render_sim.mjs --sweep standard|wot|idle|limiter|overrun|steps|accel [--wot-seconds 8] [--out ...]
//   node render_sim.mjs --rpm-track <t,rpm[,phase] csv> [--out ...]
// Options: --rate 48000  --op-rate 100 (operating-point updates per second)
//          --cabin cockpit|trackside|anechoic (default: the model's, cockpit)
//          --overrides tweak.json (spec / synth-parameter experiments, see below)

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = path.dirname(fileURLToPath(import.meta.url));
const simRoot = path.resolve(here, "..", "..");
const imp = (rel) => import(pathToFileURL(path.join(simRoot, rel)).href);

const { EngineAudio, cbr600rrSdm26, LIMITER_STUTTER_HZ } = await imp("src/audio/engineAudio.js");
const { SDM26 } = await imp("src/vehicle/params.js");
const { Powertrain } = await imp("src/vehicle/powertrain.js");

// ---------------------------------------------------------------------------
function parseArgs(argv) {
  const a = {};
  for (let i = 0; i < argv.length; i++) {
    const k = argv[i];
    if (!k.startsWith("--")) continue;
    const v = argv[i + 1] && !argv[i + 1].startsWith("--") ? argv[++i] : true;
    a[k.slice(2)] = v;
  }
  return a;
}
const args = parseArgs(process.argv.slice(2));
const fs_ = Number(args.rate ?? 48000);
const opRate = Number(args["op-rate"] ?? 100);

const curve = JSON.parse(fs.readFileSync(path.join(simRoot, "data", "sdm26-torque.json"), "utf8"));
const pt = new Powertrain(SDM26, curve);
const LIMIT = SDM26.revLimitRpm;
const HYST = SDM26.revLimitHystRpm ?? 150;
const IDLE = SDM26.idleRpm;
const IDLE_PLATE = SDM26.idleThrottleFrac ?? 0.22;

/** The indicated torque the audio model is fed (main.js replayAudioState). */
const torqueFor = (rpm, plate) => plate * (pt.wotTorque(rpm) + pt.motoringTorque(rpm));

// ---------------------------------------------------------------------------
// Operating-point sources. Each returns a function t -> op, and a duration.

function fromCsv(file, start = 0, dur = Infinity) {
  const text = fs.readFileSync(file, "utf8").trim().split(/\r?\n/);
  const head = text[0].split(",");
  const col = (name) => head.indexOf(name);
  const need = ["time_s", "engine.rpm", "engine.tps"];
  for (const n of need) if (col(n) < 0) throw new Error(`${file}: no column ${n}`);
  const cols = {
    t: col("time_s"), rpm: col("engine.rpm"), tps: col("engine.tps"), aps: col("engine.aps"),
    gear: col("engine.gear"), shifting: col("sim.shifting"),
    vkph: col("drivetrain.vehicle_speed"), gps: col("gps.speed"),
  };
  const rows = [];
  for (let i = 1; i < text.length; i++) {
    const f = text[i].split(",");
    const num = (c) => (c >= 0 ? Number(f[c]) : NaN);
    const t = num(cols.t);
    if (!(t >= start) || t > start + dur) continue;
    // Speed in m/s: the replay path uses drivetrain.vehicle_speed / 3.6.
    let speed = num(cols.vkph) / 3.6;
    if (!Number.isFinite(speed)) speed = num(cols.gps);
    rows.push({
      t: t - start, rpm: num(cols.rpm), plate: num(cols.tps) / 100,
      aps: Number.isFinite(num(cols.aps)) ? num(cols.aps) / 100 : num(cols.tps) / 100,
      gear: num(cols.gear), shifting: num(cols.shifting) > 0.5, speed,
    });
  }
  if (rows.length < 2) throw new Error("not enough telemetry rows in range");
  let j = 0;
  const at = (t) => {
    while (j < rows.length - 2 && rows[j + 1].t <= t) j++;
    while (j > 0 && rows[j].t > t) j--;
    const a = rows[j], b = rows[j + 1];
    const u = Math.min(Math.max((t - a.t) / Math.max(b.t - a.t, 1e-9), 0), 1);
    const lerp = (x, y) => x + (y - x) * u;
    const rpm = lerp(a.rpm, b.rpm);
    const plate = lerp(a.plate, b.plate);
    const shifting = (u < 0.5 ? a : b).shifting;
    const cut = rpm >= LIMIT - HYST && plate > 0.3;
    const speed = lerp(a.speed, b.speed);
    return {
      rpm,
      throttle: shifting ? 0 : plate,
      torqueNm: shifting ? 0 : torqueFor(rpm, plate),
      cut: cut && !shifting,
      overrun: !shifting && plate < 0.08 && rpm > 6000 && speed > 6,
      gear: (u < 0.5 ? a : b).gear,
      segment: "telemetry",
    };
  };
  return { at, duration: rows[rows.length - 1].t, source: { kind: "telemetry", file: path.resolve(file), start } };
}

/**
 * An rpm trace (CSV with a header: t, rpm or rpm_src, optional phase =
 * lc|pull|shift). Used to drive the model along a recording's own rpm, e.g.
 * the one analyze/accel_ab.py tracks from a video. Phases: `lc` is launch
 * control -- the engine bouncing on the LC limiter (launchRpm, launchHystRpm
 * 400 in params.js) with the ignition cut on the falling half, as the live
 * game toggles it; `shift` is the gearshift cut (zero throttle and torque, as
 * main.js passes while shifting); `pull` is full throttle.
 */
function fromTrack(file) {
  const lines = fs.readFileSync(file, "utf8").trim().split(/\r?\n/);
  const head = lines[0].split(",").map((h) => h.trim());
  const ci = (n) => head.indexOf(n);
  const cr = ci("rpm_src") >= 0 ? ci("rpm_src") : ci("rpm");
  if (ci("t") < 0 || cr < 0) throw new Error(`${file}: need columns t and rpm (or rpm_src)`);
  const rows = lines.slice(1).map((l) => {
    const f = l.split(",");
    return { t: Number(f[ci("t")]), rpm: Number(f[cr]), phase: ci("phase") >= 0 ? f[ci("phase")].trim() : "pull" };
  }).filter((r) => Number.isFinite(r.t) && Number.isFinite(r.rpm));
  const t0 = rows[0].t;
  for (const r of rows) r.t -= t0;
  const LC_HYST = SDM26.launchHystRpm ?? 400;
  let j = 0;
  const at = (t) => {
    while (j < rows.length - 2 && rows[j + 1].t <= t) j++;
    const a = rows[j], b = rows[j + 1];
    const u = Math.min(Math.max((t - a.t) / Math.max(b.t - a.t, 1e-9), 0), 1);
    let rpm = a.rpm + (b.rpm - a.rpm) * u;
    const phase = u < 0.5 ? a.phase : b.phase;
    if (phase === "lc") {
      // bounce between (lc - hyst) and lc at ~15 Hz; cut while falling
      const ph = (t * 15) % 1;
      const top = rpm + LC_HYST / 2;
      const tri = ph < 0.5 ? ph * 2 : 2 - ph * 2;
      rpm = top - LC_HYST + LC_HYST * tri;
      return { rpm, throttle: 1, torqueNm: torqueFor(rpm, 1), cut: ph >= 0.5, overrun: false, gear: 1, segment: "lc" };
    }
    if (phase === "shift") return { rpm, throttle: 0, torqueNm: 0, cut: false, overrun: false, gear: null, segment: "shift" };
    return { rpm, throttle: 1, torqueNm: torqueFor(rpm, 1), cut: false, overrun: false, gear: null, segment: "pull" };
  };
  return { at, duration: rows[rows.length - 1].t, source: { kind: "rpm-track", file: path.resolve(file) } };
}

/** A piecewise script of segments; each seg: {dur, fn(u, tSeg) -> partial op}. */
function script(segs, name) {
  const total = segs.reduce((s, x) => s + x.dur, 0);
  const at = (t) => {
    let t0 = 0;
    for (const s of segs) {
      if (t < t0 + s.dur || s === segs[segs.length - 1]) {
        const ts = Math.min(t - t0, s.dur);
        const o = s.fn(ts / s.dur, ts);
        const plate = o.plate;
        return {
          rpm: o.rpm,
          throttle: plate,
          torqueNm: o.torqueNm ?? torqueFor(o.rpm, plate),
          cut: !!o.cut,
          overrun: !!o.overrun,
          gear: o.gear ?? null,
          segment: s.label,
        };
      }
      t0 += s.dur;
    }
  };
  return { at, duration: total, source: { kind: "sweep", name } };
}

const seg = {
  idle: (dur = 3) => ({ label: "idle", dur, fn: () => ({ rpm: IDLE, plate: IDLE_PLATE }) }),
  wot: (dur = 8, r0 = 4000, r1 = LIMIT - HYST) => ({
    label: "wot", dur, fn: (u) => ({ rpm: r0 + (r1 - r0) * u, plate: 1 }),
  }),
  // The engine bouncing on the limiter: the game's hard cut with hysteresis
  // measures a ~150 rpm buzz at 30-40 Hz; a 35 Hz triangle between the
  // re-enable point and the limit.
  limiter: (dur = 2) => ({
    label: "limiter", dur, fn: (_u, ts) => {
      const ph = (ts * 35) % 1;
      const tri = ph < 0.5 ? ph * 2 : 2 - ph * 2;
      // The live game toggles the cut with the hysteresis (on at the limit,
      // off at limit - hyst): cut on the falling half of the bounce.
      return { rpm: LIMIT - HYST + HYST * tri, plate: 1, cut: ph >= 0.5 };
    },
  }),
  overrun: (dur = 4, r0 = 13000, r1 = 5000) => ({
    label: "overrun", dur, fn: (u) => {
      const rpm = r0 + (r1 - r0) * u;
      return { rpm, plate: 0, overrun: rpm > 6000 };
    },
  }),
};

function sweep(name, wotS) {
  switch (name) {
    case "idle": return script([seg.idle(4)], name);
    case "wot": return script([seg.wot(wotS)], name);
    case "limiter": return script([seg.limiter(3)], name);
    case "overrun": return script([seg.overrun(5)], name);
    case "steps": {
      // Steady WOT points, 1.5 s each: clean order profiles without ramp smear.
      const s = [];
      for (let r = 4000; r <= 14000; r += 1000) s.push({ label: `wot${r}`, dur: 1.5, fn: () => ({ rpm: r, plate: 1 }) });
      return script(s, name);
    }
    case "accel": {
      // Launch control then an accel pull through the box: LC at launchRpm
      // for 1.5 s, then each gear from the post-shift rpm to the shift rpm at
      // a rate that falls with gear, 90 ms shift cuts between.
      const lc = SDM26.launchRpm ?? 7000;
      const shiftAt = Number(args["shift-rpm"] ?? 12000);
      const lcHyst = SDM26.launchHystRpm ?? 400;
      const s = [{ label: "lc", dur: 1.5, fn: (_u, ts) => {
        const ph = (ts * 15) % 1; const tri = ph < 0.5 ? ph * 2 : 2 - ph * 2;
        return { rpm: lc - lcHyst + lcHyst * tri, plate: 1, cut: ph >= 0.5 };
      } }];
      const G = SDM26.gearRatios;
      let r0 = lc;
      const pullS = [0.9, 0.75, 0.85, 1.0, 1.15, 1.3];
      for (let g = 0; g < Math.min(G.length, Number(args.gears ?? 6)); g++) {
        const a = r0;
        s.push({ label: `pull${g + 1}`, dur: pullS[g], fn: (u) => ({ rpm: a + (shiftAt - a) * u, plate: 1 }) });
        if (g + 1 < G.length) {
          const next = shiftAt * G[g + 1] / G[g];
          s.push({ label: "shift", dur: 0.09, fn: (u) => ({ rpm: shiftAt + (next - shiftAt) * u, plate: 0, torqueNm: 0 }) });
          r0 = next;
        }
      }
      return script(s, name);
    }
    case "standard":
      return script([seg.idle(2), seg.wot(wotS), seg.limiter(2), seg.overrun(4), seg.idle(2)], name);
    default: throw new Error(`unknown sweep ${name}`);
  }
}

// ---------------------------------------------------------------------------
function writeWavFloat(file, samples, rate) {
  const n = samples.length;
  const buf = Buffer.alloc(44 + n * 4);
  buf.write("RIFF", 0); buf.writeUInt32LE(36 + n * 4, 4); buf.write("WAVE", 8);
  buf.write("fmt ", 12); buf.writeUInt32LE(16, 16); buf.writeUInt16LE(3, 20); buf.writeUInt16LE(1, 22);
  buf.writeUInt32LE(rate, 24); buf.writeUInt32LE(rate * 4, 28); buf.writeUInt16LE(4, 32); buf.writeUInt16LE(32, 34);
  buf.write("data", 36); buf.writeUInt32LE(n * 4, 40);
  for (let i = 0; i < n; i++) buf.writeFloatLE(samples[i], 44 + i * 4);
  fs.writeFileSync(file, buf);
}

// ---------------------------------------------------------------------------
let src, defaultName;
if (args["rpm-track"]) {
  src = fromTrack(args["rpm-track"]);
  defaultName = `sim_track_${path.basename(args["rpm-track"]).replace(/\.csv$/i, "")}`;
} else if (args.csv) {
  src = fromCsv(args.csv, Number(args.start ?? 0), Number(args.dur ?? Infinity));
  defaultName = `sim_${path.basename(path.dirname(path.resolve(args.csv)))}`;
} else {
  const name = args.sweep === true || !args.sweep ? "standard" : args.sweep;
  src = sweep(name, Number(args["wot-seconds"] ?? 8));
  defaultName = `sweep_${name}`;
}
const out = path.resolve(args.out ?? path.join(here, "out", `${defaultName}.wav`));
fs.mkdirSync(path.dirname(out), { recursive: true });

// --overrides <json>: experiment without editing the model.
//   { "spec": {...deep-merged into cbr600rrSdm26()...}, "params": {...synth parameters...},
//     "config": {...EngineAudio config, e.g. "cabin": "trackside"...} }
// Arrays in "spec" replace wholesale (e.g. a new "tailpipes" list); pipe
// entries may be given as {lengthM, diameterM, loss, dampingHz}.
const isObj = (v) => v && typeof v === "object" && !Array.isArray(v);
const deepMerge = (a, b) => {
  const o = { ...a };
  for (const [k, v] of Object.entries(b ?? {})) o[k] = isObj(v) && isObj(a?.[k]) ? deepMerge(a[k], v) : v;
  return o;
};
const toPipe = (p) => (p && p.diameterM != null
  ? { ...p, areaM2: (Math.PI * p.diameterM * p.diameterM) / 4 } : p);
const ov = args.overrides ? JSON.parse(fs.readFileSync(args.overrides, "utf8")) : {};
const spec = deepMerge(cbr600rrSdm26(), ov.spec);
for (const k of ["primaries", "collectors", "tailpipes"]) if (Array.isArray(spec[k])) spec[k] = spec[k].map(toPipe);
if (Array.isArray(spec.mufflers)) spec.mufflers = spec.mufflers.map((c) => (Array.isArray(c) ? c.map(toPipe) : [toPipe(c)]));
// --cabin cockpit|trackside|anechoic: the listener. The game uses "trackside"
// whenever the camera is outside the car, so compare exterior recordings with it.
const cabin = typeof args.cabin === "string" ? { cabin: args.cabin } : {};
const engine = new EngineAudio(spec, { sampleRate: fs_, ...cabin, ...(ov.config ?? {}) });
if (ov.params) engine.setParameters(ov.params);
const block = Math.max(1, Math.round(fs_ / opRate));
const nTotal = Math.round(src.duration * fs_);
const samples = new Float32Array(nTotal);
const ops = { t: [], rpm: [], throttle: [], torqueNm: [], cut: [], overrun: [], gear: [], segment: [] };

// Pre-roll at the first operating point so the slow resonance compressor and
// the waveguides settle -- otherwise the first second is a start-up transient.
{
  const o = src.at(0);
  engine.setOperatingPoint(o.rpm, o.throttle, o.torqueNm, o.cut, o.overrun);
  engine.render(new Float32Array(Math.round(fs_ * 1.0)));
}

const t0 = Date.now();
// The intake (plenum/restrictor resonator) is part of the model now -- see
// `intake` in cbr600rrSdm26(). Turn it off or change it with --overrides, e.g.
// {"spec": {"intake": {"level": 0}}}.

for (let s = 0; s < nTotal; s += block) {
  const t = s / fs_;
  const o = src.at(t);
  engine.setOperatingPoint(o.rpm, o.throttle, o.torqueNm, o.cut, o.overrun);
  engine.render(samples.subarray(s, Math.min(s + block, nTotal)));
  ops.t.push(+t.toFixed(5)); ops.rpm.push(+o.rpm.toFixed(1)); ops.throttle.push(+o.throttle.toFixed(4));
  ops.torqueNm.push(+o.torqueNm.toFixed(3)); ops.cut.push(o.cut ? 1 : 0); ops.overrun.push(o.overrun ? 1 : 0);
  ops.gear.push(o.gear); ops.segment.push(o.segment);
}
writeWavFloat(out, samples, fs_);
let peak = 0;
for (const v of samples) peak = Math.max(peak, Math.abs(v));
const side = out.replace(/\.wav$/i, "") + ".ops.json";
fs.writeFileSync(side, JSON.stringify({
  generator: "sim/tools/audio/render_sim.mjs",
  model: "sim/src/audio/engineAudio.js (live import)",
  spec: spec.name, overrides: ov,
  sampleRate: fs_, opRateHz: fs_ / block, limiterStutterHz: LIMITER_STUTTER_HZ,
  revLimitRpm: LIMIT, source: src.source, durationS: nTotal / fs_, peak,
  note: "Engine model only: no intake, tyre, wind or shift-clunk layers from game/audio.js.",
  ops,
}));
console.log(`wrote ${out} (${(nTotal / fs_).toFixed(2)} s, peak ${peak.toFixed(3)}, ${((Date.now() - t0) / 1000).toFixed(1)} s)`);
console.log(`wrote ${side}`);
