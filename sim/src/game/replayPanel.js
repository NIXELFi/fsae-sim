// The replay overlay: transport, stats, laps and events over the 3D view.
//
// This is DOM rather than a canvas HUD because every part of it is something
// you click or drag -- a scrub bar, lap chips, a speed control. The driving
// HUD stays a canvas because nothing on it is interactive and it has to be
// cheap at 144 Hz; here the 3D scene is the only thing redrawn every frame and
// the panels refresh at 20 Hz, which is faster than anyone reads.

import { fmt } from "./timing.js";

const REFRESH_HZ = 20;

/** Vertical gap between trace lanes, in CSS pixels. */
const TRACE_GAP = 3;
/** The pedal graph's window, in seconds, centred on the playhead. */
const PEDAL_WINDOW_S = 8;
/** Its height in CSS pixels. */
const PEDAL_HEIGHT = 74;
/** Width of the label gutter to the left of the traces. */
const TRACE_GUTTER = 108;

/** Channels shown in the live readout, in the order a driver reads them. */
const READOUTS = [
  { id: "drivetrain.vehicle_speed", label: "Speed", unit: "km/h", dp: 1 },
  { id: "engine.rpm", label: "RPM", unit: "", dp: 0 },
  { id: "engine.gear", label: "Gear", unit: "", dp: 0, step: true },
  { id: "chassis.steering_angle", label: "Rim", unit: "deg", dp: 1 },
  { id: "engine.aps", label: "Throttle", unit: "%", dp: 0 },
  { id: "engine.tps", label: "Plate", unit: "%", dp: 0 },
  { id: "brake.driver_load", label: "Brake", unit: "%", dp: 0 },
  { id: "brake.front_pressure", label: "Brake P/F", unit: "kPa", dp: 0 },
  { id: "brake.rear_pressure", label: "Brake P/R", unit: "kPa", dp: 0 },
  { id: "sim.ffb_command", label: "FFB", unit: "", dp: 2, signed: true },
  { id: "sim.rim_torque_nm", label: "Wheel torque", unit: "N.m", dp: 1 },
  { id: "imu.lat_g", label: "Lateral", unit: "g", dp: 2 },
  { id: "imu.long_g", label: "Long", unit: "g", dp: 2 },
  { id: "sim.body_slip_deg", label: "Body slip", unit: "deg", dp: 1 },
  { id: "sim.slip_front_deg", label: "Slip F", unit: "deg", dp: 1 },
  { id: "sim.slip_rear_deg", label: "Slip R", unit: "deg", dp: 1 },
  { id: "sim.util_front", label: "Grip used F", unit: "", dp: 2 },
  { id: "sim.util_rear", label: "Grip used R", unit: "", dp: 2 },
  { id: "sim.balance", label: "Balance", unit: "", dp: 2, signed: true },
  { id: "sim.track_s_m", label: "Distance", unit: "m", dp: 0 },
];

/**
 * The trace strip under the transport: what the car and the driver were doing
 * across the whole run, on the same x-axis as the scrub bar.
 *
 * A replay that only shows the car moving is a video. The reason to keep 100 Hz
 * telemetry is to see the INPUTS -- where the brake came off, how long the
 * trail was, whether the wheel was fighting back, where the time actually went
 * -- and to see them against each other at the same instant. So every lane
 * shares one time axis with the scrub bar and with the playhead, and clicking
 * anywhere on them seeks there.
 *
 * `scale` is fixed rather than auto-ranged wherever the number has a meaning a
 * driver already knows: a brake trace that rescales itself between runs cannot
 * be compared with the one beside it. Lanes that have no natural full scale say
 * so with `auto` and print the range they settled on.
 */
const LANES = [
  {
    key: "pedals",
    label: "Throttle / brake",
    height: 54,
    series: [
      { id: "engine.tps", colour: "#49c17a", fill: "rgba(73,193,122,.20)", lo: 0, hi: 100 },
      { id: "brake.driver_load", colour: "#e2564a", fill: "rgba(226,86,74,.22)", lo: 0, hi: 100 },
    ],
    unit: "%",
  },
  {
    key: "brakeline",
    label: "Brake pressure F / R",
    height: 44,
    series: [
      { id: "brake.front_pressure", colour: "#e2564a", lo: 0, auto: true },
      { id: "brake.rear_pressure", colour: "#e29a4a", lo: 0, auto: true },
    ],
    unit: "kPa",
  },
  {
    key: "speed",
    label: "Speed",
    height: 44,
    series: [{ id: "drivetrain.vehicle_speed", colour: "#d9dde3", lo: 0, auto: true }],
    unit: "km/h",
  },
  {
    key: "steer",
    label: "Steering",
    height: 44,
    series: [{ id: "chassis.steering_angle", colour: "#6fb2f0", auto: true, symmetric: true }],
    unit: "deg",
    zero: true,
  },
  {
    key: "ffb",
    label: "Force feedback",
    height: 44,
    // The command is what the wheel was ASKED for, normalised; `sim.ffb_clipped`
    // is when the rig could not deliver it. Clipping is the single most useful
    // thing on this lane -- a clipped wheel is a wheel telling the driver
    // nothing -- so it is drawn as a band along the top rather than a line
    // nobody would notice.
    series: [{ id: "sim.ffb_command", colour: "#c79bf0", auto: true, symmetric: true }],
    flag: { id: "sim.ffb_clipped", colour: "rgba(226,86,74,.85)", label: "clipped" },
    unit: "",
    zero: true,
  },
  {
    key: "grip",
    label: "Grip used F / R",
    height: 44,
    series: [
      { id: "sim.util_front", colour: "#f0c96f", lo: 0, hi: 1.1 },
      { id: "sim.util_rear", colour: "#6fd7c0", lo: 0, hi: 1.1 },
    ],
    unit: "",
  },
  {
    key: "delta",
    label: "Delta to reference",
    height: 50,
    // Logged live, against whatever the driver was actually chasing -- see
    // `manifest.reference`. Where the trace is blank there was no reference.
    series: [{ id: "sim.delta_s", colour: "#f0c96f", auto: true, symmetric: true,
               gate: "sim.delta_valid", fillSigned: true }],
    gateId: "sim.delta_valid",
    emptyNote: "no reference lap was loaded for this run",
    unit: "s",
    zero: true,
  },
];

const EVENT_LABEL = {
  cone: "Cone",
  "off-course": "Off course",
  lap: "Lap",
  shift: "Shift",
  end: "End",
};

export class ReplayPanel {
  /**
   * @param root     a container element that lives over the canvas
   * @param replay   the Replay being watched
   * @param actions  { onExit, onGhost, onCamera }
   */
  constructor(root, replay, actions = {}) {
    this.root = root;
    this.replay = replay;
    this.actions = actions;
    this.ghost = null;
    this.lastPaint = 0;
    this.scrubbing = false;
    this.build();
    this.paint(0, true);
  }

  setGhost(ghost) {
    this.ghost = ghost;
    this.el.ghostName.textContent = ghost
      ? `${ghost.manifest.driver ?? "Ghost"} ${ghost.bestLap ? fmt(ghost.bestLap.total) : ""}`.trim()
      : "none";
    this.el.root.classList.toggle("has-ghost", !!ghost);
    if (this.el.ghostPick) this.el.ghostPick.value = ghost?.runId ?? "";
  }

  /** Name the camera on its button, so cycling it is not a guess. */
  setCameraName(name) {
    const b = this.el.root.querySelector('[data-act="camera"]');
    if (b) b.textContent = `Camera: ${name}`;
  }

  /**
   * Offer the runs a ghost can be picked from -- same course, not this one.
   * Ghosts used to be reachable only from a launcher's `--ghost` flag, which
   * left the sim's own Runs tab with no way to put two drives side by side.
   *
   * @param runs  [{ runId, label }] newest first
   */
  setGhostChoices(runs) {
    const sel = this.el.ghostPick;
    if (!sel) return;
    const cur = this.ghost?.runId ?? "";
    sel.innerHTML = `<option value="">${runs.length ? "Pick a ghost..." : "No other run on this course"}</option>` +
      runs.map((r) => `<option value="${esc(r.runId)}"${r.runId === cur ? " selected" : ""}>${esc(r.label)}</option>`).join("");
    sel.hidden = false;
    sel.onchange = () => {
      const id = sel.value;
      this.actions.onGhost?.(id || null);
    };
  }

  build() {
    const m = this.replay.manifest;
    const st = m.stats ?? {};
    const when = m.startedAt ? new Date(m.startedAt) : null;
    const whenText = when && !isNaN(when)
      ? when.toLocaleString(undefined, { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" })
      : "";

    this.root.innerHTML = `
      <div class="rp-root">
        <header class="rp-head">
          <div class="rp-title">
            <b>${esc(m.driver || "Unknown")}</b>
            <span>${esc(m.trackName || m.track || "")}</span>
            <small>${esc(whenText)}${m.session ? ` &middot; ${esc(m.session)}` : ""}</small>
          </div>
          <div class="rp-headline">
            <div><span>Best</span><b>${st.bestLapS != null ? fmt(st.bestLapS) : "--.---"}</b></div>
            <div><span>Theoretical</span><b>${st.theoreticalBestS != null ? fmt(st.theoreticalBestS) : "--.---"}</b></div>
            <div><span>Cones</span><b>${st.totalCones ?? 0}</b></div>
            <div><span>Off course</span><b>${st.totalOffCourse ?? 0}</b></div>
            <div><span>Peak lat</span><b>${(st.peakLatG ?? 0).toFixed(2)} g</b></div>
            <div><span>Top speed</span><b>${(st.peakSpeedKph ?? 0).toFixed(0)} km/h</b></div>
          </div>
          <div class="rp-headbtns">
            <button data-act="camera" class="secondary" title="Change camera (C)">Camera</button>
            <button data-act="exit" class="secondary" title="Back to the launch screen (Esc)">Close</button>
          </div>
        </header>

        <aside class="rp-side rp-left" data-panel="left">
          <div class="rp-side-head">
            <b>Live</b>
            <button class="rp-roll" data-roll="left" title="Roll this panel up">&#9650;</button>
          </div>
          <div class="rp-bars">
            <div class="rp-bar"><i data-bar="throttle"></i><span>Throttle</span></div>
            <div class="rp-bar rp-brake"><i data-bar="brake"></i><span>Brake</span></div>
          </div>
          <div class="rp-pedalgraph">
            <canvas data-pedals></canvas>
            <span class="rp-pedalspan">pedals &middot; plate faint &middot; &plusmn;${(PEDAL_WINDOW_S / 2).toFixed(0)} s</span>
          </div>
          <dl class="rp-readouts">
            ${READOUTS.map((r) => `<dt>${esc(r.label)}</dt><dd data-ch="${esc(r.id)}">-</dd>`).join("")}
          </dl>
        </aside>

        <aside class="rp-side rp-right" data-panel="right">
          <div class="rp-side-head">
            <b>Session</b>
            <button class="rp-roll" data-roll="right" title="Roll this panel up">&#9650;</button>
          </div>
          <h4>Laps</h4>
          <div class="rp-laps" data-laps></div>
          <h4>Delta to best</h4>
          <div class="rp-delta"><b data-delta>--.--</b><small data-deltanote>needs a second lap</small></div>
          <h4>Ghost</h4>
          <div class="rp-ghost">
            <span data-ghostname>none</span>
            <b data-ghostgap></b>
          </div>
          <select data-ghostpick title="Put another run on this course in the scene beside this one" hidden>
            <option value="">Pick a ghost...</option>
          </select>
          <h4>Events</h4>
          <div class="rp-events" data-events></div>
        </aside>

        <div class="rp-hint">Tab brings the overlay back &middot; space plays &middot; arrows step</div>

        <footer class="rp-foot traces-off">
          <div class="rp-transport">
            <button data-act="back" title="Back 1 s (left arrow)">&#9664;&#9664;</button>
            <button data-act="play" class="rp-play" title="Play / pause (space)">&#9654;</button>
            <button data-act="fwd" title="Forward 1 s (right arrow)">&#9654;&#9654;</button>
            <div class="rp-rates">
              ${[0.1, 0.25, 0.5, 1, 2, 4].map((r) =>
                `<button data-rate="${r}" class="${r === 1 ? "on" : ""}">${r}x</button>`).join("")}
            </div>
            <span class="rp-clock"><b data-clock>0.000</b> / ${this.replay.duration.toFixed(2)} s</span>
            <button data-act="traces" class="secondary" title="Show or hide the traces (T)">Traces</button>
            <button data-act="bare" class="secondary" title="Hide every overlay (Tab)">Hide UI</button>
          </div>
          <div class="rp-traces" data-traces>
            <canvas data-tracecanvas></canvas>
            <div class="rp-trace-head" data-tracehead></div>
            <div class="rp-trace-keys" data-tracekeys></div>
          </div>
          <div class="rp-scrub" data-scrub>
            <div class="rp-track"></div>
            <div class="rp-marks" data-marks></div>
            <div class="rp-played" data-played></div>
            <div class="rp-head-mark" data-headmark></div>
          </div>
        </footer>
      </div>`;

    const q = (sel) => this.root.querySelector(sel);
    this.el = {
      root: q(".rp-root"),
      play: q(".rp-play"),
      clock: q("[data-clock]"),
      played: q("[data-played]"),
      headMark: q("[data-headmark]"),
      scrub: q("[data-scrub]"),
      marks: q("[data-marks]"),
      laps: q("[data-laps]"),
      delta: q("[data-delta]"),
      deltaNote: q("[data-deltanote]"),
      events: q("[data-events]"),
      ghostName: q("[data-ghostname]"),
      ghostGap: q("[data-ghostgap]"),
      ghostPick: q("[data-ghostpick]"),
      pedals: q("[data-pedals]"),
      throttle: q("[data-bar='throttle']"),
      brake: q("[data-bar='brake']"),
      foot: q(".rp-foot"),
      left: q('[data-panel="left"]'),
      right: q('[data-panel="right"]'),
      traces: q("[data-traces]"),
      traceCanvas: q("[data-tracecanvas]"),
      traceHead: q("[data-tracehead]"),
      traceKeys: q("[data-tracekeys]"),
      readouts: new Map(READOUTS.map((r) => [r.id, q(`[data-ch="${cssEsc(r.id)}"]`)])),
    };

    this.drawMarks();
    this.drawLaps();
    this.wire();
    this.restoreLayout();
  }

  /**
   * Throttle and brake either side of the playhead.
   *
   * The bars above say how much pedal there is right now; a number cannot show
   * a SHAPE. Whether the brake came off in one movement or three, how long the
   * trail was, whether the throttle was fed in or thrown at it -- that is what
   * a driver is looking for when they open a replay, and it only exists as a
   * curve.
   *
   * Centred on the playhead rather than trailing it, because the interesting
   * part of a release is what happens either side of the moment you paused on.
   * Redrawn every paint: at 20 Hz over an 8-second window this is a few
   * hundred samples, which is nothing, and it has to move with the playhead.
   */
  drawPedals() {
    const cv = this.el.pedals;
    const r = this.replay;
    if (!cv || !r) return;
    const cssW = Math.max(1, cv.clientWidth || cv.parentElement.clientWidth);
    const cssH = PEDAL_HEIGHT;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    if (cv.width !== Math.round(cssW * dpr) || cv.height !== Math.round(cssH * dpr)) {
      cv.width = Math.round(cssW * dpr);
      cv.height = Math.round(cssH * dpr);
      cv.style.width = cssW + "px";
      cv.style.height = cssH + "px";
    }
    const g = cv.getContext("2d");
    if (!g) return;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, cssW, cssH);
    g.fillStyle = "rgba(255,255,255,.035)";
    g.fillRect(0, 0, cssW, cssH);

    const half = PEDAL_WINDOW_S / 2;
    const from = r.t - half;
    const to = r.t + half;
    const pad = 2;
    const top = pad;
    const inner = cssH - pad * 2;
    const xOf = (t) => ((t - from) / PEDAL_WINDOW_S) * cssW;
    const yOf = (v) => top + inner - (Math.max(0, Math.min(100, v)) / 100) * inner;

    const line = (id, stroke, fill, width = 1.4) => {
      const col = r.tel.byId.get(id);
      if (!col) return;
      // Walk the rows inside the window. `indexAt` is a cursor search, so this
      // is a scan of the window, not of the run.
      const i0 = r.indexAt(Math.max(0, from));
      const i1 = r.indexAt(Math.min(r.duration, to));
      if (i1 <= i0) return;
      // At most one point per pixel: an 8 s window at 100 Hz is 800 samples
      // across ~164 px, and drawing all of them is five times the work for a
      // line that looks the same.
      const step = Math.max(1, Math.floor((i1 - i0) / Math.max(1, cssW)));
      g.beginPath();
      g.moveTo(xOf(r.time[i0]), yOf(col[i0]));
      for (let i = i0; i <= i1; i += step) g.lineTo(xOf(r.time[i]), yOf(col[i]));
      g.lineTo(xOf(r.time[i1]), yOf(col[i1]));
      if (fill) {
        g.save();
        g.lineTo(xOf(r.time[i1]), top + inner);
        g.lineTo(xOf(r.time[i0]), top + inner);
        g.closePath();
        g.fillStyle = fill;
        g.fill();
        g.restore();
        // The fill closed the path, so stroke the line again on its own.
        g.beginPath();
        g.moveTo(xOf(r.time[i0]), yOf(col[i0]));
        for (let i = i0; i <= i1; i += step) g.lineTo(xOf(r.time[i]), yOf(col[i]));
        g.lineTo(xOf(r.time[i1]), yOf(col[i1]));
      }
      g.strokeStyle = stroke;
      g.lineWidth = width;
      g.lineJoin = "round";
      g.stroke();
    };

    // The plate first and faint, so the pedals draw over it.
    //
    // `engine.aps` is the ACCELERATOR PEDAL -- what the driver's foot did.
    // `engine.tps` is the throttle PLATE, which is the ETC map's answer to
    // that pedal, and on a real run the two differ by up to 14 points. Drawing
    // the plate against the brake PEDAL put a driver input and an actuator
    // output on the same axes and invited exactly the comparison they cannot
    // support: it looked like the throttle and the brake disagreed, when what
    // actually disagreed was the pedal and the map.
    //
    // Both pedals are now the primary traces, and the plate stays as a dim
    // line because the gap between it and the pedal IS the ETC map, which is
    // worth being able to see while tuning it.
    line("engine.tps", "rgba(73,193,122,.40)", null, 1);
    line("engine.aps", "#49c17a", "rgba(73,193,122,.20)");
    line("brake.driver_load", "#e2564a", "rgba(226,86,74,.22)");

    // Now.
    const x = Math.round(xOf(r.t)) + 0.5;
    g.strokeStyle = "rgba(255,255,255,.75)";
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(x, 0);
    g.lineTo(x, cssH);
    g.stroke();
  }

  /**
   * Show or hide the trace strip.
   *
   * Off by default. Seven lanes of telemetry is what you want when you are
   * reading a lap and the last thing you want when you are watching one, and
   * at 380px tall it was most of the screen.
   */
  toggleTraces(on = null) {
    const show = on == null ? this.el.foot.classList.contains("traces-off") : on;
    this.el.foot.classList.toggle("traces-off", !show);
    if (show) this.drawTraces();       // sized to the width it has now
    remember("traces", show);
  }

  /** Roll one side panel up to its title bar. */
  togglePanel(which, on = null) {
    const el = this.el[which];
    if (!el) return;
    const collapsed = on == null ? !el.classList.contains("collapsed") : !on;
    el.classList.toggle("collapsed", collapsed);
    const btn = el.querySelector("[data-roll]");
    if (btn) {
      btn.innerHTML = collapsed ? "&#9660;" : "&#9650;";
      btn.title = collapsed ? "Roll this panel down" : "Roll this panel up";
    }
    remember(`panel:${which}`, !collapsed);
  }

  /** Everything off, for actually watching the car. */
  toggleBare(on = null) {
    const bare = on == null ? !this.el.root.classList.contains("rp-bare") : on;
    this.el.root.classList.toggle("rp-bare", bare);
    remember("bare", bare);
  }

  /** Put the panels back the way this user last left them. */
  restoreLayout() {
    this.toggleTraces(recall("traces", false));
    this.togglePanel("left", recall("panel:left", true));
    this.togglePanel("right", recall("panel:right", true));
    this.toggleBare(recall("bare", false));
  }

  /** Lap boundaries and incidents, drawn onto the scrub bar. Somebody
   *  watching a run wants to jump to the cone, not hunt for it. */
  drawMarks() {
    const d = this.replay.duration || 1;
    const bits = [];
    for (const l of this.replay.laps) {
      const pct = ((l.startedAtS ?? 0) / d) * 100;
      bits.push(`<i class="m-lap" style="left:${pct.toFixed(3)}%" title="Lap ${l.lap}"></i>`);
    }
    for (const e of this.replay.events) {
      const cls = e.kind === "cone" ? "m-cone" : e.kind === "off-course" ? "m-off" : null;
      if (!cls) continue;
      const pct = (e.t / d) * 100;
      bits.push(`<i class="${cls}" style="left:${pct.toFixed(3)}%" title="${esc(EVENT_LABEL[e.kind] ?? e.kind)} at ${e.t.toFixed(2)} s"></i>`);
    }
    this.el.marks.innerHTML = bits.join("");
  }

  drawLaps() {
    const laps = this.replay.laps;
    if (!laps.length) {
      this.el.laps.innerHTML = `<p class="rp-empty">No scored lap in this run.</p>`;
      return;
    }
    const best = this.replay.bestLap;
    this.el.laps.innerHTML = laps.map((l) => {
      const isBest = best && l.lap === best.lap;
      const gap = best && !isBest ? l.total - best.total : null;
      return `<button class="rp-lap${isBest ? " best" : ""}" data-lap="${l.lap}">
        <span class="n">L${l.lap}</span>
        <span class="t">${fmt(l.total)}</span>
        <span class="g">${gap == null ? (isBest ? "best" : "") : `+${gap.toFixed(3)}`}</span>
        <span class="s">${l.sectors.map((s) => s.toFixed(2)).join(" / ") || ""}</span>
        ${l.cones ? `<span class="p">${l.cones}c</span>` : ""}
        ${l.off ? `<span class="p">OFF - no time</span>` : ""}
      </button>`;
    }).join("");
  }

  /**
   * Draw every lane once.
   *
   * Once, not per frame: a run's traces never change, so the only thing that
   * moves is the playhead, and that is a 2px element the browser can shift
   * without touching the canvas. A 28,000-sample endurance log across seven
   * lanes is far too much to redraw at 20 Hz and completely free to redraw on
   * a resize.
   *
   * The x-axis is the run's whole duration, shared with the scrub bar directly
   * above it, so the cone marks, the lap boundaries and the traces all line up
   * and one click seeks all of them.
   */
  drawTraces() {
    const cv = this.el.traceCanvas;
    const r = this.replay;
    if (!cv || !r) return;
    const cssW = Math.max(1, Math.round(this.el.traces.clientWidth - TRACE_GUTTER));
    const total = LANES.reduce((a, l) => a + l.height, 0) + (LANES.length - 1) * TRACE_GAP;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    cv.width = Math.round(cssW * dpr);
    cv.height = Math.round(total * dpr);
    cv.style.width = cssW + "px";
    cv.style.height = total + "px";
    this.el.traces.style.height = total + "px";

    const g = cv.getContext("2d");
    if (!g) return;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, cssW, total);

    const n = r.rows;
    const keys = [];
    if (n < 2) { this.el.traceKeys.innerHTML = ""; return; }
    // One column of pixels per sample at most: past that the extra samples
    // cannot be seen and cost real time on a long run. Each column draws the
    // MIN and MAX of the samples it covers, so a spike one sample wide -- a
    // kerb strike, a moment of clipping -- is still visible rather than being
    // averaged away, which is the whole reason to keep 100 Hz.
    const cols = Math.max(2, Math.min(cssW, n));
    const perCol = n / cols;
    let y = 0;
    for (const lane of LANES) {
      this.drawLane(g, lane, 0, y, cssW, lane.height, cols, perCol, keys);
      y += lane.height + TRACE_GAP;
    }
    this.el.traceKeys.innerHTML = keys.join("");
  }

  /** One lane: its baseline, its series, its flag band and its label. */
  drawLane(g, lane, x, y, w, h, cols, perCol, keys) {
    const r = this.replay;
    const pad = 3;
    const top = y + pad;
    const inner = h - pad * 2;

    g.fillStyle = "rgba(255,255,255,.03)";
    g.fillRect(x, y, w, h);

    // Resolve the vertical scale first, because every series in a lane shares
    // it -- front and rear brake pressure on separate scales would look like a
    // bias that is not there.
    let lo = Infinity;
    let hi = -Infinity;
    const cached = [];
    for (const sp of lane.series) {
      const col = r.tel.byId.get(sp.id);
      cached.push(col || null);
      if (!col) continue;
      if (sp.lo != null && sp.hi != null) {
        lo = Math.min(lo, sp.lo);
        hi = Math.max(hi, sp.hi);
        continue;
      }
      const gate = sp.gate ? r.tel.byId.get(sp.gate) : null;
      for (let i = 0; i < r.rows; i++) {
        if (gate && !gate[i]) continue;
        const v = col[i];
        if (!Number.isFinite(v)) continue;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
      if (sp.lo != null) lo = Math.min(lo, sp.lo);
    }
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) { lo = 0; hi = 1; }
    if (lane.series.some((sp) => sp.symmetric)) {
      const m = Math.max(Math.abs(lo), Math.abs(hi), 1e-6);
      lo = -m;
      hi = m;
    }
    if (hi - lo < 1e-9) hi = lo + 1;
    const span = hi - lo;
    const toY = (v) => top + inner - ((v - lo) / span) * inner;
    const px = (c) => x + (c / Math.max(1, cols - 1)) * w;

    // The zero line, where zero is something you steer around rather than a
    // floor to sit on.
    if (lane.zero && lo < 0 && hi > 0) {
      g.strokeStyle = "rgba(255,255,255,.2)";
      g.lineWidth = 1;
      g.beginPath();
      g.moveTo(x, Math.round(toY(0)) + 0.5);
      g.lineTo(x + w, Math.round(toY(0)) + 0.5);
      g.stroke();
    }

    for (let si = 0; si < lane.series.length; si++) {
      const sp = lane.series[si];
      const col = cached[si];
      if (!col) continue;
      const gate = sp.gate ? r.tel.byId.get(sp.gate) : null;
      const mins = new Float64Array(cols).fill(NaN);
      const maxs = new Float64Array(cols).fill(NaN);
      for (let c = 0; c < cols; c++) {
        const from = Math.floor(c * perCol);
        const to = Math.min(r.rows, Math.max(from + 1, Math.floor((c + 1) * perCol)));
        let mn = Infinity;
        let mx = -Infinity;
        for (let i = from; i < to; i++) {
          if (gate && !gate[i]) continue;
          const v = col[i];
          if (!Number.isFinite(v)) continue;
          if (v < mn) mn = v;
          if (v > mx) mx = v;
        }
        if (mn <= mx) { mins[c] = mn; maxs[c] = mx; }
      }

      if (sp.fill || sp.fillSigned) {
        const base = sp.fillSigned ? toY(0) : toY(Math.max(lo, sp.lo != null ? sp.lo : lo));
        g.fillStyle = sp.fill || "rgba(240,201,111,.20)";
        g.beginPath();
        let open = false;
        for (let c = 0; c < cols; c++) {
          if (Number.isNaN(maxs[c])) {
            if (open) { g.lineTo(px(Math.max(0, c - 1)), base); g.closePath(); open = false; }
            continue;
          }
          if (!open) { g.moveTo(px(c), base); open = true; }
          g.lineTo(px(c), toY(maxs[c]));
        }
        if (open) { g.lineTo(px(cols - 1), base); g.closePath(); }
        g.fill();
      }

      g.strokeStyle = sp.colour;
      g.lineWidth = 1.25;
      g.lineJoin = "round";
      g.beginPath();
      let open = false;
      for (let c = 0; c < cols; c++) {
        if (Number.isNaN(maxs[c])) { open = false; continue; }
        const yTop = toY(maxs[c]);
        const yBot = toY(mins[c]);
        if (!open) { g.moveTo(px(c), yTop); open = true; } else g.lineTo(px(c), yTop);
        if (yBot !== yTop) g.lineTo(px(c), yBot);
      }
      g.stroke();
    }

    // A flag channel -- clipping -- as a band along the top, because a
    // one-sample event on a line is invisible and this one matters: a clipped
    // wheel is a wheel telling the driver nothing.
    if (lane.flag) {
      const col = r.tel.byId.get(lane.flag.id);
      if (col) {
        g.fillStyle = lane.flag.colour;
        const cw = Math.max(1, w / cols);
        for (let c = 0; c < cols; c++) {
          const from = Math.floor(c * perCol);
          const to = Math.min(r.rows, Math.max(from + 1, Math.floor((c + 1) * perCol)));
          let on = false;
          for (let i = from; i < to && !on; i++) if (col[i]) on = true;
          if (on) g.fillRect(px(c), y, cw, 3);
        }
      }
    }

    // A lane that is empty for a REASON should say so. A blank delta lane and
    // a delta that was flat zero all run look identical otherwise.
    if (lane.gateId) {
      const gc = r.tel.byId.get(lane.gateId);
      let any = false;
      if (gc) for (let i = 0; i < r.rows && !any; i++) if (gc[i]) any = true;
      if (!any) {
        g.fillStyle = "rgba(255,255,255,.28)";
        g.font = "11px system-ui, sans-serif";
        g.textBaseline = "middle";
        g.fillText(lane.emptyNote || "no data", x + 10, y + h / 2);
      }
    }

    const series = lane.series
      .map((sp) => '<u style="background:' + esc(sp.colour) + '"></u>')
      .join("");
    const flag = lane.flag
      ? '<u class="flag" style="background:' + esc(lane.flag.colour) + '"></u>'
      : "";
    keys.push(
      '<span class="rp-key" style="height:' + lane.height + 'px">' +
        "<b>" + esc(lane.label) + "</b>" +
        "<i>" + esc(fmtAxis(lo) + "–" + fmtAxis(hi) + (lane.unit ? " " + lane.unit : "")) + "</i>" +
        series + flag +
      "</span>",
    );
  }

  wire() {
    const r = this.replay;
    // Held so `destroy()` can detach it. `this.root` is the persistent overlay
    // element, not the markup this panel built, so `innerHTML = ""` does NOT
    // remove a listener attached here -- every replay opened would add another,
    // and each closure pins its Replay and the whole parsed telemetry behind
    // it. A seven-minute endurance log is 26 MB.
    this.onClick = (e) => {
      const act = e.target.closest("[data-act]")?.dataset.act;
      if (act === "play") { r.toggle(); this.paint(0, true); return; }
      if (act === "back") { r.nudge(-1); this.paint(0, true); return; }
      if (act === "fwd") { r.nudge(1); this.paint(0, true); return; }
      if (act === "exit") { this.actions.onExit?.(); return; }
      if (act === "camera") { this.actions.onCamera?.(); return; }
      if (act === "traces") { this.toggleTraces(); return; }
      if (act === "bare") { this.toggleBare(); return; }
      const roll = e.target.closest("[data-roll]")?.dataset.roll;
      if (roll) { this.togglePanel(roll); return; }
      const rate = e.target.closest("[data-rate]")?.dataset.rate;
      if (rate) {
        r.setRate(Number(rate));
        for (const b of this.root.querySelectorAll("[data-rate]")) {
          b.classList.toggle("on", b.dataset.rate === rate);
        }
        return;
      }
      const lap = e.target.closest("[data-lap]")?.dataset.lap;
      if (lap) { r.seekLap(Number(lap)); this.paint(0, true); }
    };
    this.root.addEventListener("click", this.onClick);

    // Scrubbing. Pointer capture so a drag that leaves the bar keeps working,
    // which is what anyone dragging a 40-second run at this scale will do.
    const seekFromEvent = (ev) => {
      const box = this.el.scrub.getBoundingClientRect();
      const a = Math.max(0, Math.min(1, (ev.clientX - box.left) / Math.max(1, box.width)));
      r.seek(a * r.duration);
      this.paint(0, true);
    };
    this.el.scrub.addEventListener("pointerdown", (ev) => {
      this.scrubbing = true;
      this.wasPlaying = r.playing;
      r.pause();
      this.el.scrub.setPointerCapture(ev.pointerId);
      seekFromEvent(ev);
    });
    this.el.scrub.addEventListener("pointermove", (ev) => {
      if (this.scrubbing) seekFromEvent(ev);
    });
    const endScrub = (ev) => {
      if (!this.scrubbing) return;
      this.scrubbing = false;
      try { this.el.scrub.releasePointerCapture(ev.pointerId); } catch { /* already gone */ }
      if (this.wasPlaying) r.play();
    };
    this.el.scrub.addEventListener("pointerup", endScrub);
    this.el.scrub.addEventListener("pointercancel", endScrub);

    // The traces share the scrub bar's x-axis, so they scrub too. Anyone
    // looking at a brake trace wants to click the corner they are looking at,
    // not find it again on a 6px bar underneath.
    const seekFromTrace = (ev) => {
      const box = this.el.traceCanvas.getBoundingClientRect();
      const a = Math.max(0, Math.min(1, (ev.clientX - box.left) / Math.max(1, box.width)));
      r.seek(a * r.duration);
      this.paint(0, true);
    };
    this.el.traces.addEventListener("pointerdown", (ev) => {
      this.scrubbing = true;
      this.wasPlaying = r.playing;
      r.pause();
      this.el.traces.setPointerCapture(ev.pointerId);
      seekFromTrace(ev);
    });
    this.el.traces.addEventListener("pointermove", (ev) => {
      if (this.scrubbing) seekFromTrace(ev);
    });
    const endTrace = (ev) => {
      if (!this.scrubbing) return;
      this.scrubbing = false;
      try { this.el.traces.releasePointerCapture(ev.pointerId); } catch { /* already gone */ }
      if (this.wasPlaying) r.play();
    };
    this.el.traces.addEventListener("pointerup", endTrace);
    this.el.traces.addEventListener("pointercancel", endTrace);

    // Redrawn on resize, not on a timer: the traces are static for the run, so
    // this is the only thing that can invalidate them. Held on `this` so
    // `destroy()` can detach it -- a window listener that pins a Replay is how
    // a 26 MB endurance log stays alive after the panel has gone.
    this.onResize = () => this.drawTraces();
    window.addEventListener("resize", this.onResize);
  }

  /**
   * Refresh the panels. Called every frame; does the work at `REFRESH_HZ`
   * unless forced, because reading 14 channels and rewriting the event list
   * 144 times a second is work nobody can see.
   */
  paint(dt, force = false) {
    const now = performance.now();
    if (!force && now - this.lastPaint < 1000 / REFRESH_HZ) return;
    this.lastPaint = now;
    const r = this.replay;

    this.el.play.innerHTML = r.playing ? "&#10074;&#10074;" : "&#9654;";
    this.el.clock.textContent = r.t.toFixed(3);
    const pct = r.duration > 0 ? (r.t / r.duration) * 100 : 0;
    this.el.played.style.width = `${pct.toFixed(3)}%`;
    this.el.headMark.style.left = `${pct.toFixed(3)}%`;
    // The only part of the trace strip that moves. Positioned against the
    // canvas rather than the strip, because the strip is padded by the label
    // gutter and a percentage of the whole would drift right by 108px.
    const gw = this.el.traceCanvas.clientWidth || 1;
    this.el.traceHead.style.left = `${(TRACE_GUTTER + (pct / 100) * gw).toFixed(1)}px`;

    // The PEDAL, to match the brake bar beside it. The plate is on the graph
    // below and in its own readout; a bar labelled "Throttle" sitting next to
    // one labelled "Brake" has to be the same kind of measurement.
    const thr = clamp01(r.value("engine.aps") / 100);
    const brk = clamp01(r.value("brake.driver_load") / 100);
    this.el.throttle.style.width = `${(thr * 100).toFixed(1)}%`;
    this.el.brake.style.width = `${(brk * 100).toFixed(1)}%`;
    this.drawPedals();

    for (const spec of READOUTS) {
      const el = this.el.readouts.get(spec.id);
      if (!el) continue;
      const v = spec.step ? r.valueAt(spec.id) : r.value(spec.id);
      const text = spec.dp === 0 ? String(Math.round(v)) : v.toFixed(spec.dp);
      el.textContent = spec.unit ? `${spec.signed && v > 0 ? "+" : ""}${text} ${spec.unit}`
                                 : `${spec.signed && v > 0 ? "+" : ""}${text}`;
    }

    const cur = r.lapAt();
    for (const b of this.el.laps.querySelectorAll("[data-lap]")) {
      b.classList.toggle("now", cur != null && Number(b.dataset.lap) === cur.lap);
    }

    // Two different deltas, and they answer different questions. The LOGGED
    // one is what the driver was looking at while they drove -- against
    // whatever reference was loaded at the time -- and it is the honest answer
    // to "what were they reacting to". `deltaToBest` is the analyst's one,
    // computed now, against this run's own best lap. Show the live one when
    // there is one, because the replay is of a drive, not of a spreadsheet.
    const liveValid = r.value("sim.delta_valid") > 0.5;
    const live = liveValid ? r.value("sim.delta_s") : null;
    const ref = r.manifest.reference;
    const d = live != null ? live : r.deltaToBest();
    if (d == null) {
      this.el.delta.textContent = "--.--";
      this.el.delta.className = "";
      this.el.deltaNote.textContent = r.laps.length > 1 ? "on the best lap" : "needs a second lap";
    } else if (live != null) {
      this.el.delta.textContent = `${d >= 0 ? "+" : ""}${d.toFixed(3)}`;
      this.el.delta.className = d <= 0 ? "up" : "down";
      this.el.deltaNote.textContent = ref?.label
        ? `live, vs ${ref.label}${ref.lapS != null ? ` ${fmt(ref.lapS)}` : ""}`
        : "live, as the driver saw it";
    } else {
      this.el.delta.textContent = `${d >= 0 ? "+" : ""}${d.toFixed(3)}`;
      this.el.delta.className = d <= 0 ? "up" : "down";
      this.el.deltaNote.textContent = `vs lap ${r.bestLap.lap}, same point on the course`;
    }

    if (this.ghost) {
      const gap = ghostGap(r, this.ghost);
      this.el.ghostGap.textContent = gap == null ? "" : `${gap >= 0 ? "+" : ""}${gap.toFixed(3)} s`;
      this.el.ghostGap.className = gap == null ? "" : gap <= 0 ? "up" : "down";
    }

    const evs = r.eventsNear(8, 0.5);
    this.el.events.innerHTML = evs.length
      ? evs.slice(0, 8).map((e) => `<div class="rp-ev ${esc(e.kind)}">
          <span class="t">${e.t.toFixed(2)}</span>
          <span class="k">${esc(EVENT_LABEL[e.kind] ?? e.kind)}</span>
          <span class="d">${esc(eventDetail(e))}</span></div>`).join("")
      : `<p class="rp-empty">nothing in the last few seconds</p>`;
  }

  destroy() {
    if (this.onClick) this.root.removeEventListener("click", this.onClick);
    this.onClick = null;
    if (this.onResize) window.removeEventListener("resize", this.onResize);
    this.onResize = null;
    // Drop the references this panel holds so the Replay (and its ~77
    // Float64Arrays) can be collected as soon as the game lets go of it.
    this.replay = null;
    this.ghost = null;
    this.el = null;
    this.root.innerHTML = "";
  }
}

/**
 * How far ahead or behind the ghost is, in seconds, at the same point on the
 * course. Positive means the run being watched is behind its ghost.
 */
export function ghostGap(replay, ghost) {
  const s = replay.value("sim.track_s_m");
  const cur = replay.lapAt();
  const gLap = ghost.bestLap ?? ghost.lapAt(ghost.t);
  if (!cur || !gLap) return null;
  const mine = replay.t - (cur.startedAtS ?? 0);
  const theirs = ghost.timeAtDistanceInLap(gLap, s);
  if (theirs == null) return null;
  return mine - theirs;
}

/** Axis numbers short enough to sit in the label gutter. */
function fmtAxis(v) {
  const a = Math.abs(v);
  if (a >= 1000) return (v / 1000).toFixed(1) + "k";
  if (a >= 100) return v.toFixed(0);
  if (a >= 10) return v.toFixed(1);
  if (a >= 1) return v.toFixed(2);
  return v.toFixed(3);
}

/* Panel layout is a per-machine preference, like a window size: it belongs in
 * localStorage, not in a run. Wrapped because a private window throws on the
 * accessor itself rather than just returning nothing. */
const LAYOUT_KEY = "fsae-sim:replay-layout";

function layout() {
  try {
    return JSON.parse(localStorage.getItem(LAYOUT_KEY) || "{}") || {};
  } catch {
    return {};
  }
}

function remember(key, value) {
  try {
    const all = layout();
    all[key] = value;
    localStorage.setItem(LAYOUT_KEY, JSON.stringify(all));
  } catch { /* not worth interrupting a replay over */ }
}

function recall(key, dflt) {
  const v = layout()[key];
  return typeof v === "boolean" ? v : dflt;
}

function clamp01(v) { return Math.max(0, Math.min(1, Number.isFinite(v) ? v : 0)); }

function eventDetail(e) {
  if (e.kind === "cone") return `${e.n} at ${Math.round(e.s ?? 0)} m`;
  if (e.kind === "off-course") return `${(e.lateral ?? 0).toFixed(1)} m wide at ${Math.round(e.s ?? 0)} m`;
  if (e.kind === "shift") return `${e.from} to ${e.to} at ${e.rpm} rpm`;
  if (e.kind === "lap") return fmt(e.total ?? 0);
  if (e.kind === "end") return String(e.reason ?? "");
  return "";
}

function esc(v) {
  return String(v).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

/** Attribute selectors here contain dots (`sim.pos_x`), which are legal
 *  inside a quoted attribute value but would break an unquoted one. */
function cssEsc(v) {
  return String(v).replace(/["\\]/g, "\\$&");
}
