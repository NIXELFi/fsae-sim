// Replay -> MP4, rendered offline.
//
// Not a screen recording. The replay is a pure function of time (replay.js),
// so the export steps it frame by frame at exactly 1/fps, draws each frame
// with the game's own renderer at the file's resolution, and hands it to the
// browser's H.264 encoder (WebCodecs). Nothing is dropped however slow the
// machine; it just takes longer. The sound is the replay's own engine,
// tyre, wind and drivetrain synthesis (audio.js), run on an
// OfflineAudioContext over the same span and encoded to AAC. mp4-muxer
// (vendor/, MIT) puts the two in one file, which the desktop shell writes to
// Videos/FSAE Sim (src-tauri/src/video.rs); a browser downloads it.

import { Muxer, StreamTarget, ArrayBufferTarget } from "../vendor/mp4-muxer.mjs";
import { EngineAudio } from "./audio.js";
import { fmt } from "./timing.js";

export const RESOLUTIONS = {
  "1080p60": { w: 1920, h: 1080, fps: 60, bitrate: 16e6, codec: "avc1.640028" },
  "1080p30": { w: 1920, h: 1080, fps: 30, bitrate: 10e6, codec: "avc1.640028" },
  "1440p60": { w: 2560, h: 1440, fps: 60, bitrate: 28e6, codec: "avc1.640032" },
  "4k30": { w: 3840, h: 2160, fps: 30, bitrate: 40e6, codec: "avc1.640033" },
};

const SAMPLE_RATE = 48000;
/** Audio operating points per second: the live engine pushes at most every
 *  12 ms (audio.js), so this is the same rate the driver heard. */
const AUDIO_HZ = 100;
/** Lead-in and run-out around a lap, so it starts and ends on the move. */
const PAD_S = 1.0;
/** When the span runs to the end of the recording (a run that stopped at the
 *  flag), hold the last frame this long so the final time can be read, and
 *  fade the sound out over it. */
const END_HOLD_S = 2.0;

/** What this browser can do, before offering the button at all. */
export async function exportSupport() {
  if (typeof VideoEncoder === "undefined" || typeof AudioEncoder === "undefined" ||
      typeof OfflineAudioContext === "undefined") {
    return { ok: false, why: "this window cannot encode video (no WebCodecs)" };
  }
  try {
    const v = await VideoEncoder.isConfigSupported({ codec: "avc1.640028", width: 1920, height: 1080, bitrate: 16e6, framerate: 60 });
    const a = await AudioEncoder.isConfigSupported({ codec: "mp4a.40.2", sampleRate: SAMPLE_RATE, numberOfChannels: 2, bitrate: 192000 });
    if (!v.supported) return { ok: false, why: "no H.264 encoder" };
    return { ok: true, audio: a.supported ? "aac" : null };
  } catch (e) {
    return { ok: false, why: String(e?.message ?? e) };
  }
}

/** The spans worth exporting: each scored lap, padded, and the whole run. */
export function exportRanges(replay) {
  const out = replay.laps.map((l) => ({
    label: `Lap ${l.lap}  ${fmt(l.total)}${l.cones ? ` (${l.cones} cone${l.cones > 1 ? "s" : ""})` : ""}${l === replay.bestLap ? "  best" : ""}`,
    from: Math.max(0, (l.startedAtS ?? 0) - PAD_S),
    to: Math.min(replay.duration, l.endedAtS + PAD_S),
    lap: l.lap,
  }));
  out.push({ label: `Whole run  ${fmt(replay.duration)}`, from: 0, to: replay.duration, lap: null });
  return out;
}

function desktopInvoke() {
  return window.__TAURI__?.core?.invoke ?? null;
}

/**
 * Where the bytes go. Desktop: streamed to disk through the shell, positional
 * writes in order. Browser: kept in memory and downloaded at the end.
 */
export async function openSink(name) {
  const invoke = desktopInvoke();
  if (!invoke) {
    const target = new ArrayBufferTarget();
    return {
      target,
      async finish() {
        const url = URL.createObjectURL(new Blob([target.buffer], { type: "video/mp4" }));
        const a = document.createElement("a");
        a.href = url; a.download = `${name}.mp4`; a.click();
        setTimeout(() => URL.revokeObjectURL(url), 60_000);
        return { path: `${name}.mp4`, desktop: false };
      },
      async abort() {},
    };
  }
  const file = await invoke("video_begin", { name });
  let chain = Promise.resolve();
  let failed = null;
  const target = new StreamTarget({
    chunked: true,
    onData: (data, position) => {
      // The muxer may reuse its buffer; copy before it goes async.
      const bytes = data.slice();
      chain = chain.then(() => invoke("video_write", bytes, {
        headers: { "x-video-id": String(file.id), "x-offset": String(position) },
      })).catch((e) => { failed ??= e; });
    },
  });
  return {
    target,
    async finish() {
      await chain;
      if (failed) throw new Error(`writing the video failed: ${failed}`);
      const path = await invoke("video_end", { id: file.id, keep: true });
      return { path, desktop: true };
    },
    async abort() {
      await chain.catch(() => {});
      await invoke("video_end", { id: file.id, keep: false }).catch(() => {});
    },
  };
}

/**
 * The replay's sound over [from, to), rendered offline: the same EngineAudio
 * graph the replay plays live, fed the same `replayAudioState()` the live
 * replay feeds it, at the times the samples belong to.
 */
export async function renderAudio(game, from, to, cameraName, onProgress, fadeFrom = null) {
  const replay = game.replay;
  const length = Math.ceil((to - from) * SAMPLE_RATE);
  const ctx = new OfflineAudioContext({ numberOfChannels: 2, length, sampleRate: SAMPLE_RATE });
  const audio = new EngineAudio();
  audio.start(ctx);
  await audio.modelReady;
  audio.setCamera(cameraName);
  audio.reset();
  const dt = 1 / AUDIO_HZ;
  for (let t = from, k = 0; t < to; t = from + (++k) * dt) {
    replay.seek(t);
    audio.update(game.replayAudioState(), dt, t - from);
    if (k % 500 === 0) onProgress?.((t - from) / (to - from));
  }
  await audio.sendTimeline();
  const buffer = await ctx.startRendering();
  // The held end: the recording has stopped, so the sound goes, not the
  // engine held at whatever it was doing at the flag.
  if (fadeFrom != null) {
    const i0 = Math.max(0, Math.floor((fadeFrom - from) * SAMPLE_RATE));
    const n = Math.max(1, Math.floor(1.2 * SAMPLE_RATE));
    for (let c = 0; c < buffer.numberOfChannels; c++) {
      const ch = buffer.getChannelData(c);
      for (let i = i0; i < ch.length; i++) {
        const k = (i - i0) / n;
        ch[i] *= k >= 1 ? 0 : 0.5 * (1 + Math.cos(Math.PI * k));
      }
    }
  }
  return buffer;
}

/** Encode an AudioBuffer as AAC into the muxer. */
export async function encodeAudio(buffer, muxer) {
  let error = null;
  const enc = new AudioEncoder({
    output: (chunk, meta) => muxer.addAudioChunk(chunk, meta),
    error: (e) => { error = e; },
  });
  enc.configure({ codec: "mp4a.40.2", sampleRate: SAMPLE_RATE, numberOfChannels: 2, bitrate: 192000 });
  const L = buffer.getChannelData(0);
  const R = buffer.getChannelData(1);
  const block = 4800; // 100 ms
  for (let i = 0; i < buffer.length; i += block) {
    const n = Math.min(block, buffer.length - i);
    const planar = new Float32Array(n * 2);
    planar.set(L.subarray(i, i + n), 0);
    planar.set(R.subarray(i, i + n), n);
    const data = new AudioData({
      format: "f32-planar", sampleRate: SAMPLE_RATE, numberOfFrames: n, numberOfChannels: 2,
      timestamp: Math.round((i / SAMPLE_RATE) * 1e6), data: planar,
    });
    enc.encode(data);
    data.close();
    if (error) throw error;
  }
  await enc.flush();
  enc.close();
  if (error) throw error;
}

// ---- the overlay -------------------------------------------------------------

function roundRect(g, x, y, w, h, r) {
  g.beginPath();
  g.moveTo(x + r, y);
  g.arcTo(x + w, y, x + w, y + h, r);
  g.arcTo(x + w, y + h, x, y + h, r);
  g.arcTo(x, y + h, x, y, r);
  g.arcTo(x, y, x + w, y, r);
  g.closePath();
}

/** The course as this run drove it, for the map: positions every 0.1 s. */
function trackMap(replay) {
  if (replay._map) return replay._map;
  const pts = [];
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (let t = 0; t <= replay.duration; t += 0.1) {
    const x = replay.value("sim.pos_x", t);
    const y = replay.value("sim.pos_y", t);
    pts.push([x, y]);
    minX = Math.min(minX, x); maxX = Math.max(maxX, x);
    minY = Math.min(minY, y); maxY = Math.max(maxY, y);
  }
  replay._map = { pts, minX, maxX, minY, maxY };
  return replay._map;
}

/**
 * Telemetry burned into the frame. Laid out for the cockpit camera, whose
 * middle-bottom is the car's own dash and wheel, so everything sits at the
 * edges: who and the lap clock (top left) with sector splits, the course map
 * with the car on it (top right), a live readout column (left), speed / gear
 * / revs (bottom left), and the last six seconds of throttle, brake and steer
 * beside the g-g (bottom right). Scaled to the frame height, so 1080p and 4K
 * read the same. `info.gap()` (optional) is a live gap to another driver.
 */
export function drawOverlay(g, W, H, game, info) {
  const r = game.replay;
  const s = H / 1080;
  const panel = "rgba(8, 10, 14, 0.64)";
  const text = "#f3f4f6";
  const dim = "rgba(243, 244, 246, 0.62)";
  const green = "#22c55e", red = "#ef4444", blue = "#60a5fa", amber = "#f59e0b";
  const font = (px, w = 600) => `${w} ${Math.round(px * s)}px "Segoe UI", system-ui, sans-serif`;
  const mono = (px, w = 700) => `${w} ${Math.round(px * s)}px "Cascadia Mono", Consolas, ui-monospace, monospace`;
  const card = (x, y, w, h) => { g.fillStyle = panel; roundRect(g, x, y, w, h, 14 * s); g.fill(); };
  const v = (id) => r.value(id);

  // ---- who, and the lap clock ----
  const lap = r.lapAt();
  const inLap = lap && r.t >= (lap.startedAtS ?? 0);
  const lapT = inLap ? Math.min(r.t - lap.startedAtS, lap.raw ?? lap.spanS ?? Infinity) : 0;
  const done = inLap && r.t >= lap.endedAtS - 1e-3;
  const gap = info.gap?.();
  const topH = info.gap ? 196 : 150;
  card(32 * s, 28 * s, 470 * s, topH * s);
  g.fillStyle = text; g.font = font(26, 700);
  g.fillText(info.driver, 54 * s, 68 * s);
  g.fillStyle = dim; g.font = font(19, 500);
  g.fillText(info.subtitle, 54 * s, 98 * s);
  g.fillStyle = done ? "#a7f3d0" : text; g.font = mono(40);
  g.fillText(lap ? `L${lap.lap}  ${fmt(done ? lap.total : lapT)}` : "--", 54 * s, 152 * s);
  if (info.gap) {
    g.font = mono(26);
    if (gap == null) {
      g.fillStyle = dim;
      g.fillText(`--  vs ${info.compareName}`, 54 * s, 196 * s);
    } else {
      g.fillStyle = gap > 0 ? "#fca5a5" : "#a7f3d0";
      g.fillText(`${gap >= 0 ? "+" : "-"}${Math.abs(gap).toFixed(3)}  vs ${info.compareName}`, 54 * s, 196 * s);
    }
  }
  if (lap?.sectors?.length) {
    let acc = 0;
    let x = 520 * s;
    lap.sectors.forEach((sec, i) => {
      acc += sec;
      if (!inLap || lapT < acc - 1e-3) return;
      card(x, 28 * s, 150 * s, 64 * s);
      g.fillStyle = dim; g.font = font(16, 600);
      g.fillText(`S${i + 1}`, x + 16 * s, 54 * s);
      g.fillStyle = text; g.font = mono(24);
      g.fillText(sec.toFixed(3), x + 16 * s, 82 * s);
      x += 162 * s;
    });
  }

  // ---- the course map ----
  {
    const m = trackMap(r);
    const size = 300 * s, mx = W - size - 32 * s, my = 28 * s, pad = 22 * s;
    card(mx, my, size, size);
    const span = Math.max(m.maxX - m.minX, m.maxY - m.minY, 1);
    const k = (size - 2 * pad) / span;
    const ox = mx + pad + ((size - 2 * pad) - (m.maxX - m.minX) * k) / 2;
    const oy = my + pad + ((size - 2 * pad) - (m.maxY - m.minY) * k) / 2;
    const P = (x, y) => [ox + (x - m.minX) * k, oy + (m.maxY - y) * k];
    g.lineJoin = "round"; g.lineCap = "round";
    g.strokeStyle = "rgba(255,255,255,0.28)"; g.lineWidth = 5 * s;
    g.beginPath();
    m.pts.forEach(([x, y], i) => { const [px, py] = P(x, y); if (i) g.lineTo(px, py); else g.moveTo(px, py); });
    g.stroke();
    // Where this lap has been, then the car.
    if (inLap) {
      g.strokeStyle = blue; g.lineWidth = 3 * s;
      g.beginPath();
      let first = true;
      for (let t = lap.startedAtS; t <= r.t; t += 0.1) {
        const [px, py] = P(r.value("sim.pos_x", t), r.value("sim.pos_y", t));
        if (first) { g.moveTo(px, py); first = false; } else g.lineTo(px, py);
      }
      g.stroke();
    }
    const [cx, cy] = P(v("sim.pos_x"), v("sim.pos_y"));
    g.fillStyle = "#fff"; g.beginPath(); g.arc(cx, cy, 7 * s, 0, Math.PI * 2); g.fill();
    g.fillStyle = blue; g.beginPath(); g.arc(cx, cy, 4.5 * s, 0, Math.PI * 2); g.fill();
  }

  // ---- the readout column ----
  {
    const rows = [
      ["RPM", Math.round(v("engine.rpm")).toLocaleString("en-US"), ""],
      ["Steering", v("chassis.steering_angle").toFixed(0), "deg"],
      ["Lateral", Math.abs(v("imu.lat_g")).toFixed(2), "g"],
      ["Long", v("imu.long_g").toFixed(2), "g"],
      ["Yaw rate", Math.abs(v("imu.yaw_rate")).toFixed(0), "deg/s"],
      ["Body slip", v("sim.body_slip_deg").toFixed(1), "deg"],
      ["Slip F / R", `${Math.abs(v("sim.slip_front_deg")).toFixed(1)} / ${Math.abs(v("sim.slip_rear_deg")).toFixed(1)}`, "deg"],
      ["Grip F / R", `${v("sim.util_front").toFixed(2)} / ${v("sim.util_rear").toFixed(2)}`, ""],
      ["Throttle", v("engine.aps").toFixed(0), "%"],
      ["Brake", v("brake.driver_load").toFixed(0), "%"],
    ];
    const x = 32 * s, y = (topH + 44) * s, w = 380 * s, lh = 34 * s;
    card(x, y, w, (rows.length * 34 + 24) * s);
    rows.forEach(([label, val, unit], i) => {
      const yy = y + 36 * s + i * lh;
      g.fillStyle = dim; g.font = font(17, 600); g.textAlign = "left";
      g.fillText(label, x + 18 * s, yy);
      g.fillStyle = text; g.font = mono(21); g.textAlign = "right";
      g.fillText(val, x + w - 72 * s, yy);
      g.fillStyle = dim; g.font = font(15, 600); g.textAlign = "left";
      g.fillText(unit, x + w - 64 * s, yy);
    });
    g.textAlign = "left";
  }

  // ---- speed, gear, revs ----
  const kmh = v("drivetrain.vehicle_speed");
  const gear = Math.round(r.valueAt("engine.gear"));
  const rpm = v("engine.rpm");
  const bx = 32 * s, by = H - 190 * s;
  card(bx, by, 420 * s, 160 * s);
  g.fillStyle = text; g.font = mono(84); g.textAlign = "right";
  g.fillText(String(Math.round(kmh)), bx + 230 * s, by + 98 * s);
  g.textAlign = "left";
  g.fillStyle = dim; g.font = font(20, 600);
  g.fillText("km/h", bx + 240 * s, by + 96 * s);
  g.fillStyle = text; g.font = mono(64);
  g.fillText(gear > 0 ? String(gear) : "N", bx + 336 * s, by + 96 * s);
  const frac = Math.max(0, Math.min(1, rpm / 14500));
  const rx = bx + 22 * s, ry = by + 122 * s, rw = 376 * s, rh = 18 * s;
  g.fillStyle = "rgba(255,255,255,0.12)";
  roundRect(g, rx, ry, rw, rh, 6 * s); g.fill();
  g.fillStyle = frac > 0.9 ? red : frac > 0.78 ? amber : green;
  roundRect(g, rx, ry, Math.max(rh, rw * frac), rh, 6 * s); g.fill();

  // ---- the last six seconds: throttle, brake, steer ----
  {
    const cw = 470 * s, ch = 190 * s, cx0 = W - 260 * s - cw - 16 * s, cy0 = H - ch - 30 * s;
    card(cx0, cy0, cw, ch);
    const px = cx0 + 16 * s, pw = cw - 70 * s, py = cy0 + 30 * s, ph = ch - 48 * s;
    const key = (label, col, x) => {
      g.fillStyle = dim; g.font = font(14, 700);
      g.fillText(label, x, cy0 + 22 * s);
      const tw = g.measureText(label).width;
      g.fillStyle = col; g.fillRect(x + tw + 6 * s, cy0 + 13 * s, 14 * s, 4 * s);
    };
    key("THR", green, px);
    key("BRK", red, px + 64 * s);
    key("STEER", "#e5e7eb", px + 128 * s);
    g.fillStyle = dim; g.textAlign = "right"; g.fillText("last 6 s", px + pw, cy0 + 22 * s); g.textAlign = "left";
    g.strokeStyle = "rgba(255,255,255,0.12)"; g.lineWidth = 1 * s;
    g.beginPath(); g.moveTo(px, py + ph / 2); g.lineTo(px + pw, py + ph / 2); g.stroke();
    const WIN = 6, N = 120;
    const trace = (fn, col, width) => {
      g.strokeStyle = col; g.lineWidth = width * s; g.beginPath();
      for (let i = 0; i <= N; i++) {
        const tt = Math.max(0, r.t - WIN + (WIN * i) / N);
        const X = px + (pw * i) / N;
        const Y = py + ph * (1 - Math.max(0, Math.min(1, fn(tt))));
        if (i === 0) g.moveTo(X, Y); else g.lineTo(X, Y);
      }
      g.stroke();
    };
    trace((t) => r.value("brake.driver_load", t) / 100, red, 3);
    trace((t) => r.value("engine.aps", t) / 100, green, 3);
    // Steering: centre line is straight ahead, full height is 180 deg of rim.
    trace((t) => 0.5 - r.value("chassis.steering_angle", t) / 360, "rgba(229,231,235,0.9)", 2);
    const bxx = px + pw + 14 * s, bw = 14 * s;
    for (const [val, col, off] of [[v("engine.aps") / 100, green, 0], [v("brake.driver_load") / 100, red, bw + 6 * s]]) {
      g.fillStyle = "rgba(255,255,255,0.12)"; g.fillRect(bxx + off, py, bw, ph);
      const hh = ph * Math.max(0, Math.min(1, val));
      g.fillStyle = col; g.fillRect(bxx + off, py + ph - hh, bw, hh);
    }
  }

  // ---- the g-g, with the last second as a trail ----
  const cx = W - 130 * s, cy = H - 130 * s, R = 92 * s, gMax = 2.0;
  g.fillStyle = panel;
  g.beginPath(); g.arc(cx, cy, R + 14 * s, 0, Math.PI * 2); g.fill();
  g.strokeStyle = "rgba(255,255,255,0.18)"; g.lineWidth = 1.5 * s;
  for (const k of [0.5, 1, 1.5, 2]) { g.beginPath(); g.arc(cx, cy, (R * k) / gMax, 0, Math.PI * 2); g.stroke(); }
  g.beginPath(); g.moveTo(cx - R, cy); g.lineTo(cx + R, cy); g.moveTo(cx, cy - R); g.lineTo(cx, cy + R); g.stroke();
  const at = (tt) => [cx + (r.value("imu.lat_g", tt) / gMax) * R, cy - (r.value("imu.long_g", tt) / gMax) * R];
  g.strokeStyle = "rgba(96,165,250,0.7)"; g.lineWidth = 3 * s;
  g.beginPath();
  for (let k = 20; k >= 0; k--) {
    const [x, y] = at(Math.max(0, r.t - k * 0.05));
    if (k === 20) g.moveTo(x, y); else g.lineTo(x, y);
  }
  g.stroke();
  const [dx, dy] = at(r.t);
  g.fillStyle = blue;
  g.beginPath(); g.arc(dx, dy, 8 * s, 0, Math.PI * 2); g.fill();
  g.fillStyle = dim; g.font = font(15, 700); g.textAlign = "center";
  g.fillText(`${Math.hypot(v("imu.lat_g"), v("imu.long_g")).toFixed(2)} g`, cx, cy + R + 2 * s);
  g.textAlign = "left";
}

// ---- the export --------------------------------------------------------------

/**
 * Render [from, to] of the open replay to an MP4.
 *
 * @param game   the Game (main.js), with a replay open
 * @param opts   { from, to, quality ("1080p60"...), overlay, name,
 *                 onProgress(fraction, phase), signal (AbortSignal) }
 * @returns      { path, desktop, seconds }
 */
export async function exportReplayVideo(game, opts) {
  const replay = game.replay;
  if (!replay) throw new Error("no replay open");
  const q = RESOLUTIONS[opts.quality] ?? RESOLUTIONS["1080p60"];
  const { w: W, h: H, fps } = q;
  const from = Math.max(0, opts.from ?? 0);
  const end = Math.min(replay.duration, opts.to ?? replay.duration);
  if (!(end > from)) throw new Error("nothing to export");
  const held = end >= replay.duration - 0.05;
  const to = held ? end + END_HOLD_S : end;
  const signal = opts.signal;
  const progress = opts.onProgress ?? (() => {});
  const cameraName = opts.cameraName ?? "Chase";
  const m = replay.manifest ?? {};
  const info = {
    driver: m.driver || "Driver",
    subtitle: [m.trackName || m.track, (m.stats?.vehicleModel ?? m.laps?.[0]?.vehicleModel) === 3 ? "SDM26 4-wheel" : "SDM26 bicycle", m.simVersion ? `sim ${m.simVersion}` : null]
      .filter(Boolean).join("  |  "),
  };

  // Everything this changes, to put back however it ends.
  const saved = { t: replay.t, playing: replay.playing, lastDt: game.lastDt };
  replay.pause();
  game.exporting = true;
  const sink = await openSink(opts.name ?? "replay");
  let ok = false;
  const started = performance.now();
  try {
    const audioSupported = (await exportSupport()).audio === "aac";
    const muxer = new Muxer({
      target: sink.target,
      video: { codec: "avc", width: W, height: H, frameRate: fps },
      audio: audioSupported ? { codec: "aac", numberOfChannels: 2, sampleRate: SAMPLE_RATE } : undefined,
      fastStart: false,
      firstTimestampBehavior: "offset",
    });

    // ---- sound first: it is the quick half, and the muxer holds it until
    // the video catches up.
    if (audioSupported) {
      progress(0, "audio");
      const buffer = await renderAudio(game, from, to, cameraName, (f) => progress(f * 0.08, "audio"), held ? end : null);
      if (signal?.aborted) throw new DOMException("cancelled", "AbortError");
      await encodeAudio(buffer, muxer);
    }

    // ---- then the pictures, one exact frame at a time.
    let error = null;
    const venc = new VideoEncoder({
      output: (chunk, meta) => muxer.addVideoChunk(chunk, meta),
      error: (e) => { error = e; },
    });
    venc.configure({ codec: q.codec, width: W, height: H, bitrate: q.bitrate, framerate: fps, latencyMode: "quality" });
    const comp = new OffscreenCanvas(W, H);
    const g = comp.getContext("2d");
    const renderer = game.renderer;
    renderer.fixedSize = { w: W, h: H };
    game.chaseYaw = null; // the chase camera snaps onto the car at the start
    const frames = Math.round((to - from) * fps);
    for (let k = 0; k < frames; k++) {
      if (signal?.aborted) throw new DOMException("cancelled", "AbortError");
      if (error) throw error;
      replay.seek(from + k / fps);
      game.lastDt = 1 / fps;
      game.applyReplayFrame();
      game.render();
      // Straight after the draw, in the same task: the WebGL canvas still
      // holds this frame.
      g.drawImage(renderer.canvas, 0, 0, W, H);
      if (opts.overlay !== false) drawOverlay(g, W, H, game, info);
      const frame = new VideoFrame(comp, { timestamp: Math.round((k * 1e6) / fps), duration: Math.round(1e6 / fps) });
      venc.encode(frame, { keyFrame: k % (fps * 2) === 0 });
      frame.close();
      // Let the encoder drain and the page breathe (the progress bar, a
      // cancel click) without ever skipping a frame.
      while (venc.encodeQueueSize > 6) await new Promise((res) => setTimeout(res, 1));
      if (k % 10 === 0) {
        progress(0.08 + 0.9 * (k / frames), "video");
        await new Promise((res) => setTimeout(res, 0));
      }
    }
    await venc.flush();
    venc.close();
    if (error) throw error;
    progress(0.99, "saving");
    muxer.finalize();
    const out = await sink.finish();
    ok = true;
    progress(1, "done");
    return { ...out, seconds: (performance.now() - started) / 1000 };
  } finally {
    if (!ok) await sink.abort();
    game.renderer.fixedSize = null;
    game.exporting = false;
    game.lastDt = saved.lastDt;
    replay.seek(saved.t);
    replay.playing = saved.playing;
    game.chaseYaw = null;
    game.applyReplayFrame();
  }
}

/** Show the file in Explorer / Finder. Desktop only. */
export function revealVideo(path) {
  return desktopInvoke()?.("video_reveal", { path });
}

/** A file name from the run: driver, course, lap time. */
export function exportName(replay, range) {
  const m = replay.manifest ?? {};
  const lap = range?.lap != null ? replay.laps.find((l) => l.lap === range.lap) : null;
  const bits = [m.driver || "replay", m.track || "", lap ? `lap ${lap.lap} ${fmt(lap.total)}` : "full run", (m.runId || "").slice(0, 15)];
  return bits.filter(Boolean).join(" ").replace(/[:/\\]/g, ".");
}

// ---- the dialog --------------------------------------------------------------

/**
 * The export dialog, over the replay: which lap, what quality, overlay or
 * not; then a progress bar with a cancel, then the file.
 */
export async function openExportDialog(game) {
  const replay = game.replay;
  if (!replay) return;
  document.getElementById("video-export")?.remove();
  const support = await exportSupport();
  const ranges = exportRanges(replay);
  const best = ranges.findIndex((r) => r.lap != null && r.lap === replay.bestLap?.lap);
  const box = document.createElement("div");
  box.id = "video-export";
  box.style.cssText = "position:fixed;inset:0;display:flex;align-items:center;justify-content:center;background:rgba(0,0,0,0.55);z-index:1000;";
  box.innerHTML = `
    <div style="background:#111418;color:#f3f4f6;border:1px solid rgba(255,255,255,0.12);border-radius:12px;padding:22px 26px;min-width:380px;font:14px system-ui,sans-serif;box-shadow:0 20px 60px rgba(0,0,0,0.5)">
      <h3 style="margin:0 0 14px;font-size:18px">Export video</h3>
      <div data-part="form">
        <label style="display:block;margin:8px 0 4px;opacity:.75">What</label>
        <select data-k="range" style="width:100%">${ranges.map((r, i) => `<option value="${i}" ${i === (best >= 0 ? best : 0) ? "selected" : ""}>${r.label}</option>`).join("")}</select>
        <label style="display:block;margin:12px 0 4px;opacity:.75">Quality</label>
        <select data-k="quality" style="width:100%">
          <option value="1080p60" selected>1080p, 60 fps</option>
          <option value="1080p30">1080p, 30 fps</option>
          <option value="1440p60">1440p, 60 fps</option>
          <option value="4k30">4K, 30 fps</option>
        </select>
        <label style="display:flex;gap:8px;align-items:center;margin:14px 0 4px"><input type="checkbox" data-k="overlay" checked> Telemetry overlay (lap clock, speed, pedals, g-g)</label>
        <p style="margin:10px 0 0;opacity:.6;font-size:12px">Camera: ${game.cameraName?.() ?? "current"}. Sound is the replay's own engine and tyres.</p>
        ${support.ok ? "" : `<p style="color:#fca5a5">${support.why}</p>`}
        <div style="display:flex;gap:10px;justify-content:flex-end;margin-top:18px">
          <button class="secondary" data-act="close">Cancel</button>
          <button data-act="go" ${support.ok ? "" : "disabled"}>Export</button>
        </div>
      </div>
      <div data-part="run" hidden>
        <div data-k="phase" style="margin-bottom:8px;opacity:.8">Starting...</div>
        <div style="height:10px;background:rgba(255,255,255,0.1);border-radius:5px;overflow:hidden"><div data-k="bar" style="height:100%;width:0;background:#60a5fa"></div></div>
        <div style="display:flex;justify-content:flex-end;margin-top:16px"><button class="secondary" data-act="cancel">Cancel</button></div>
      </div>
      <div data-part="done" hidden>
        <p data-k="result" style="word-break:break-all"></p>
        <div style="display:flex;gap:10px;justify-content:flex-end;margin-top:16px">
          <button class="secondary" data-act="reveal">Show in folder</button>
          <button data-act="close">Done</button>
        </div>
      </div>
    </div>`;
  document.body.appendChild(box);
  const $ = (sel) => box.querySelector(sel);
  const part = (name) => { for (const p of box.querySelectorAll("[data-part]")) p.hidden = p.dataset.part !== name; };
  let abort = null;
  let path = null;
  const close = () => { abort?.abort(); box.remove(); };
  box.addEventListener("keydown", (e) => { e.stopPropagation(); if (e.key === "Escape") close(); });
  box.addEventListener("click", async (e) => {
    const act = e.target?.dataset?.act;
    if (act === "close") close();
    if (act === "cancel") abort?.abort();
    if (act === "reveal" && path) revealVideo(path);
    if (act !== "go") return;
    const range = ranges[Number($('[data-k="range"]').value)];
    abort = new AbortController();
    part("run");
    try {
      const res = await exportReplayVideo(game, {
        from: range.from, to: range.to,
        quality: $('[data-k="quality"]').value,
        overlay: $('[data-k="overlay"]').checked,
        name: exportName(replay, range),
        cameraName: game.cameraName?.(),
        signal: abort.signal,
        onProgress: (f, phase) => {
          $('[data-k="bar"]').style.width = `${Math.round(f * 100)}%`;
          $('[data-k="phase"]').textContent =
            phase === "audio" ? "Rendering the sound..." : phase === "video" ? `Rendering frames... ${Math.round(f * 100)}%` : phase === "saving" ? "Saving..." : "Done";
        },
      });
      path = res.path;
      $('[data-k="result"]').textContent = `Saved ${res.path} (${res.seconds.toFixed(0)} s to render)`;
      $('[data-act="reveal"]').hidden = !res.desktop;
      part("done");
    } catch (err) {
      if (err?.name === "AbortError") { close(); return; }
      $('[data-k="result"]').textContent = `Export failed: ${err?.message ?? err}`;
      $('[data-act="reveal"]').hidden = true;
      part("done");
    }
  });
}
