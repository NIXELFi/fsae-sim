// Interactive editor for the ETC pedal map.
//
// Drag the breakpoints, click the plot to add one, right-click or double-click
// to remove it. The dashed diagonal is 1:1 so you can always see what the map
// is doing relative to a linear pedal, and the live marker shows where the
// driver's foot actually is -- keep a pad connected and you can hold a pedal
// position and watch what plate it asks for while you shape the curve.

import { ETC_PRESETS } from "../vehicle/etcMap.js";

const PLOT = { x0: 46, y0: 16, w: 268, h: 208 };
const VB = { w: 336, h: 262 };
const SVG_NS = "http://www.w3.org/2000/svg";

export class EtcEditor {
  /**
   * @param {HTMLElement} root   container element (hidden by default)
   * @param {object} opts  { getMap, onChange, getLive }
   */
  constructor(root, opts) {
    this.root = root;
    this.getMap = opts.getMap;
    this.onChange = opts.onChange ?? (() => {});
    this.getLive = opts.getLive ?? (() => null);
    this.onClose = opts.onClose ?? null;
    this.isOpen = false;
    this.selected = -1;
    this.dragging = -1;
    this._build();
  }

  // ------------------------------------------------------------- markup ---

  _build() {
    this.root.innerHTML = `
      <div class="etc-card">
        <div class="etc-head">
          <div>
            <h2>Electronic throttle map</h2>
            <p>Accelerator pedal position to throttle plate. Drag a point, click
               the plot to add one, right-click a point to remove it.</p>
          </div>
          <button class="etc-close" data-act="close">Done</button>
        </div>

        <div class="etc-body">
          <div class="etc-plotwrap">
            <svg class="etc-plot" viewBox="0 0 ${VB.w} ${VB.h}"></svg>
            <div class="etc-stats"></div>
          </div>

          <div class="etc-side">
            <label class="etc-label">Presets</label>
            <div class="etc-presets"></div>
            <p class="etc-note"></p>

            <label class="etc-label">Breakpoints</label>
            <div class="etc-table"></div>
            <div class="etc-actions">
              <button data-act="add">Add point</button>
              <button data-act="reset">Reset to linear</button>
            </div>
            <div class="etc-actions">
              <button data-act="copy">Copy JSON</button>
              <button data-act="paste">Paste JSON</button>
            </div>
            <p class="etc-msg"></p>
          </div>
        </div>
      </div>`;

    this.svg = this.root.querySelector(".etc-plot");
    this.statsEl = this.root.querySelector(".etc-stats");
    this.presetsEl = this.root.querySelector(".etc-presets");
    this.noteEl = this.root.querySelector(".etc-note");
    this.tableEl = this.root.querySelector(".etc-table");
    this.msgEl = this.root.querySelector(".etc-msg");

    for (const key of Object.keys(ETC_PRESETS)) {
      const b = document.createElement("button");
      b.textContent = ETC_PRESETS[key].label;
      b.dataset.preset = key;
      this.presetsEl.appendChild(b);
    }

    this.root.addEventListener("click", (e) => {
      const preset = e.target.closest("[data-preset]");
      if (preset) {
        this.getMap().loadPreset(preset.dataset.preset);
        this.selected = -1;
        this._changed();
        return;
      }
      const act = e.target.closest("[data-act]")?.dataset.act;
      if (act) this._action(act, e);
    });

    // Plot interaction.
    this.svg.addEventListener("pointerdown", (e) => this._onDown(e));
    this.svg.addEventListener("pointermove", (e) => this._onMove(e));
    this.svg.addEventListener("pointerup", (e) => this._onUp(e));
    this.svg.addEventListener("pointercancel", (e) => this._onUp(e));
    this.svg.addEventListener("contextmenu", (e) => {
      e.preventDefault();
      const i = this._hitTest(this._toPlot(e));
      if (i > 0) { this.getMap().removePoint(i); this.selected = -1; this._changed(); }
    });
    this.svg.addEventListener("dblclick", (e) => {
      const i = this._hitTest(this._toPlot(e));
      if (i > 0) { this.getMap().removePoint(i); this.selected = -1; this._changed(); }
    });

    this._keyHandler = (e) => {
      if (!this.isOpen) return;
      if (e.key === "Escape") { e.preventDefault(); this.close(); }
      if ((e.key === "Delete" || e.key === "Backspace") && this.selected > 0 &&
          !/^(INPUT|TEXTAREA)$/.test(e.target.tagName)) {
        e.preventDefault();
        this.getMap().removePoint(this.selected);
        this.selected = -1;
        this._changed();
      }
    };
    addEventListener("keydown", this._keyHandler);
  }

  _action(act, e) {
    const map = this.getMap();
    if (act === "close") this.close();
    if (act === "reset") { map.loadPreset("linear"); this.selected = -1; this._changed(); }
    if (act === "add") {
      // Drop the new point into the widest gap, on the current curve, so the
      // shape does not jump when you add somewhere to grab.
      let bestGap = -1, at = 50;
      for (let i = 0; i < map.points.length - 1; i++) {
        const gap = map.points[i + 1][0] - map.points[i][0];
        if (gap > bestGap) { bestGap = gap; at = (map.points[i][0] + map.points[i + 1][0]) / 2; }
      }
      const idx = map.addPoint(at, map.plateAt(at));
      if (idx >= 0) this.selected = idx;
      this._changed();
    }
    if (act === "copy") {
      const text = JSON.stringify(map.toJSON());
      navigator.clipboard?.writeText(text).then(
        () => this._say("Copied to clipboard."),
        () => this._say(text));
    }
    if (act === "paste") {
      navigator.clipboard?.readText().then((text) => {
        try {
          const o = JSON.parse(text);
          const pts = Array.isArray(o) ? o : o.points;
          if (!Array.isArray(pts)) throw new Error("no points array");
          map.name = (o && o.name) || "custom";
          map.setPoints(pts);
          this.selected = -1;
          this._changed();
          this._say(`Loaded ${map.points.length} breakpoints.`);
        } catch (err) {
          this._say(`Could not read that JSON: ${err.message}`);
        }
      }, () => this._say("Clipboard read was blocked by the browser."));
    }
  }

  _say(text) {
    this.msgEl.textContent = text;
    clearTimeout(this._msgTimer);
    this._msgTimer = setTimeout(() => { this.msgEl.textContent = ""; }, 4000);
  }

  // -------------------------------------------------------- coordinates ---

  /** Client event to plot coordinates in percent. */
  _toPlot(e) {
    const pt = this.svg.createSVGPoint();
    pt.x = e.clientX; pt.y = e.clientY;
    const m = this.svg.getScreenCTM();
    const p = m ? pt.matrixTransform(m.inverse()) : { x: 0, y: 0 };
    return {
      pedal: ((p.x - PLOT.x0) / PLOT.w) * 100,
      plate: ((PLOT.y0 + PLOT.h - p.y) / PLOT.h) * 100,
      vx: p.x, vy: p.y,
    };
  }

  _px(pedal) { return PLOT.x0 + (pedal / 100) * PLOT.w; }
  _py(plate) { return PLOT.y0 + PLOT.h - (plate / 100) * PLOT.h; }

  _hitTest(p) {
    const map = this.getMap();
    let best = -1, bestD = 11; // viewBox units
    map.points.forEach(([x, y], i) => {
      const d = Math.hypot(this._px(x) - p.vx, this._py(y) - p.vy);
      if (d < bestD) { bestD = d; best = i; }
    });
    return best;
  }

  _onDown(e) {
    const p = this._toPlot(e);
    const i = this._hitTest(p);
    if (i >= 0) {
      this.selected = i;
      if (i > 0) { this.dragging = i; this.svg.setPointerCapture(e.pointerId); }
      this._render();
      return;
    }
    // Empty plot area: add a breakpoint there.
    if (p.pedal > 0 && p.pedal < 100 && p.plate >= -4 && p.plate <= 104) {
      const idx = this.getMap().addPoint(p.pedal, p.plate);
      if (idx >= 0) {
        this.selected = idx;
        this.dragging = idx;
        this.svg.setPointerCapture(e.pointerId);
        this._changed();
      }
    }
  }

  _onMove(e) {
    if (this.dragging < 0) return;
    const p = this._toPlot(e);
    this.getMap().movePoint(this.dragging, p.pedal, p.plate);
    // The point may have been re-sorted; follow it.
    this._changed();
  }

  _onUp(e) {
    if (this.dragging >= 0) {
      try { this.svg.releasePointerCapture(e.pointerId); } catch { /* ignore */ }
      this.dragging = -1;
      this._changed();
    }
  }

  // ------------------------------------------------------------ drawing ---

  _changed() {
    const map = this.getMap();
    map.name = map.name === "custom" ? "custom" : this._matchesPreset(map) ?? "custom";
    this.onChange(map);
    this._render();
  }

  _matchesPreset(map) {
    for (const [key, preset] of Object.entries(ETC_PRESETS)) {
      if (preset.points.length !== map.points.length) continue;
      const same = preset.points.every(([x, y], i) =>
        Math.abs(map.points[i][0] - x) < 0.01 && Math.abs(map.points[i][1] - y) < 0.01);
      if (same) return key;
    }
    return null;
  }

  _render() {
    if (!this.isOpen) return;
    const map = this.getMap();
    const svg = this.svg;
    while (svg.firstChild) svg.removeChild(svg.firstChild);

    const add = (tag, attrs, parent = svg) => {
      const el = document.createElementNS(SVG_NS, tag);
      for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
      parent.appendChild(el);
      return el;
    };

    add("rect", { x: PLOT.x0, y: PLOT.y0, width: PLOT.w, height: PLOT.h, class: "etc-bg" });

    for (let v = 0; v <= 100; v += 20) {
      add("line", { x1: this._px(v), y1: PLOT.y0, x2: this._px(v), y2: PLOT.y0 + PLOT.h, class: "etc-grid" });
      add("line", { x1: PLOT.x0, y1: this._py(v), x2: PLOT.x0 + PLOT.w, y2: this._py(v), class: "etc-grid" });
      add("text", { x: this._px(v), y: PLOT.y0 + PLOT.h + 14, class: "etc-tick" }).textContent = v;
      add("text", { x: PLOT.x0 - 8, y: this._py(v) + 3.5, class: "etc-tick etc-tick-y" }).textContent = v;
    }
    add("text", { x: PLOT.x0 + PLOT.w / 2, y: VB.h - 4, class: "etc-axis" })
      .textContent = "Accelerator pedal  %";
    const yl = add("text", { x: 12, y: PLOT.y0 + PLOT.h / 2, class: "etc-axis" });
    yl.setAttribute("transform", `rotate(-90 12 ${PLOT.y0 + PLOT.h / 2})`);
    yl.textContent = "Throttle plate  %";

    // 1:1 reference.
    add("line", {
      x1: this._px(0), y1: this._py(0), x2: this._px(100), y2: this._py(100),
      class: "etc-linear",
    });

    // The curve.
    const pts = map.sample(121).map(([x, y]) => `${this._px(x).toFixed(2)},${this._py(y).toFixed(2)}`);
    add("polyline", { points: pts.join(" "), class: "etc-curve" });

    // Live pedal marker.
    const live = this.getLive();
    if (live && live.pedal > 0.004) {
      const lx = this._px(live.pedal * 100), ly = this._py(live.plate * 100);
      add("line", { x1: lx, y1: PLOT.y0 + PLOT.h, x2: lx, y2: ly, class: "etc-live-line" });
      add("line", { x1: PLOT.x0, y1: ly, x2: lx, y2: ly, class: "etc-live-line" });
      add("circle", { cx: lx, cy: ly, r: 4.5, class: "etc-live" });
    }

    // Breakpoint handles.
    map.points.forEach(([x, y], i) => {
      const cls = i === this.selected ? "etc-pt etc-pt-sel" : "etc-pt";
      const anchor = i === 0 || i === map.points.length - 1;
      add("circle", {
        cx: this._px(x), cy: this._py(y), r: anchor ? 4.5 : 5.5,
        class: anchor ? `${cls} etc-pt-anchor` : cls,
      });
    });

    this._renderStats();
    this._renderTable();
  }

  _renderStats() {
    const map = this.getMap();
    const d = map.describe();
    const presetKey = this._matchesPreset(map);
    this.noteEl.textContent = presetKey ? ETC_PRESETS[presetKey].note : "Custom map.";
    this.statsEl.innerHTML = `
      <span><b>${d.points}</b> points</span>
      <span>initial gain <b>${d.initialGain.toFixed(2)}</b> <i>${d.character}</i></span>
      <span>max vs linear <b>${d.maxDeviation > 0 ? "+" : ""}${d.maxDeviation}%</b>
        at <b>${d.maxDeviationAtPedal}%</b> pedal</span>
      <span>plate at full pedal <b>${d.plateAtFullPedal}%</b></span>`;
  }

  _renderTable() {
    const map = this.getMap();
    const last = map.points.length - 1;
    this.tableEl.innerHTML = "";
    map.points.forEach(([x, y], i) => {
      const row = document.createElement("div");
      row.className = "etc-row" + (i === this.selected ? " sel" : "");
      const anchor = i === 0 || i === last;
      row.innerHTML = `
        <input type="number" min="0" max="100" step="0.5" value="${round(x)}"
               ${i === 0 || i === last ? "disabled" : ""} data-i="${i}" data-f="x">
        <span class="etc-arrow">&rarr;</span>
        <input type="number" min="0" max="100" step="0.5" value="${round(y)}"
               ${i === 0 ? "disabled" : ""} data-i="${i}" data-f="y">
        <button class="etc-del" data-del="${i}" ${anchor ? "disabled" : ""}
                title="${anchor ? "Anchors cannot be removed" : "Remove"}">&times;</button>`;
      this.tableEl.appendChild(row);
    });

    this.tableEl.querySelectorAll("input").forEach((inp) => {
      inp.addEventListener("change", () => {
        const i = +inp.dataset.i;
        const p = map.points[i];
        if (!p) return;
        const x = inp.dataset.f === "x" ? +inp.value : p[0];
        const y = inp.dataset.f === "y" ? +inp.value : p[1];
        map.movePoint(i, x, y);
        this._changed();
      });
    });
    this.tableEl.querySelectorAll("[data-del]").forEach((b) => {
      b.addEventListener("click", () => {
        map.removePoint(+b.dataset.del);
        this.selected = -1;
        this._changed();
      });
    });
  }

  // --------------------------------------------------------------- open ---

  open() {
    this.isOpen = true;
    this.root.hidden = false;
    this._render();
    if (!this._raf) {
      const tick = () => {
        if (!this.isOpen) { this._raf = null; return; }
        // Only the live marker needs redrawing at frame rate.
        const live = this.getLive();
        // Redraw for a real pedal move, never for analog jitter, and never
        // while the driver is typing into the table (a rebuild eats the edit).
        const typing = this.root.contains(document.activeElement) &&
          /^(INPUT|TEXTAREA)$/.test(document.activeElement.tagName);
        if (live && !typing && Math.abs(live.pedal - (this._lastPedal ?? -1)) > 0.004) {
          this._lastPedal = live.pedal;
          this._render();
        }
        this._raf = requestAnimationFrame(tick);
      };
      this._raf = requestAnimationFrame(tick);
    }
  }

  close() {
    if (!this.isOpen) return;
    this.isOpen = false;
    this.root.hidden = true;
    this.onClose?.();
  }
}

function round(v) { return Math.round(v * 10) / 10; }
