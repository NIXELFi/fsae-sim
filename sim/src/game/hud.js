// 2D overlay HUD, drawn on a canvas above the WebGL view.
//
// Laid out like a steering-wheel dash rather than a video-game UI: shift lights
// across the top of the field of view, a linear tach where a Motec would put
// it, and the timing block where the driver can find it on a straight. Colours
// are Sun Devil maroon and gold.

const GOLD = "#FFC627";
const MAROON = "#8C1D40";

export class Hud {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.minimapCache = null;
  }

  resize() {
    const dpr = Math.min(devicePixelRatio || 1, 2);
    const cw = this.canvas.clientWidth > 0 ? this.canvas.clientWidth : 1280;
    const ch = this.canvas.clientHeight > 0 ? this.canvas.clientHeight : 720;
    const w = Math.max(1, Math.floor(cw * dpr));
    const h = Math.max(1, Math.floor(ch * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
      this.minimapCache = null;
    }
    this.dpr = dpr;
    return { w, h };
  }

  draw(s) {
    const { w, h } = this.resize();
    const ctx = this.ctx;
    ctx.clearRect(0, 0, w, h);
    ctx.save();
    // Work in CSS pixels regardless of DPR.
    ctx.scale(this.dpr, this.dpr);
    const W = w / this.dpr, H = h / this.dpr;

    this.shiftLights(ctx, W, s);
    this.tach(ctx, W, H, s);
    this.timing(ctx, s);
    this.penalties(ctx, W, s);
    this.minimap(ctx, H, s);
    this.gg(ctx, W, H, s);
    this.balance(ctx, W, H, s);
    this.setupPanel(ctx, W, H, s);
    this.message(ctx, W, H, s);
    if (s.paused) this.paused(ctx, W, H);

    ctx.restore();
  }

  // ------------------------------------------------------------ components ---

  shiftLights(ctx, W, s) {
    const n = 10;
    const total = Math.min(W * 0.42, 460);
    const gap = 5;
    const bw = (total - gap * (n - 1)) / n;
    const x0 = W / 2 - total / 2;
    const y = 16;
    // Lights start well before the shift point so the last three are the cue.
    const from = s.shiftRpm * 0.72;
    const frac = clamp((s.rpm - from) / Math.max(1, s.revLimit - from), 0, 1);
    const lit = Math.floor(frac * n + 1e-6);
    const flash = s.rpm >= s.revLimit - 120 && Math.floor(performance.now() / 70) % 2 === 0;

    for (let i = 0; i < n; i++) {
      const on = flash || i < lit;
      ctx.fillStyle = on
        ? (i < 4 ? "#3ddc84" : i < 7 ? GOLD : "#ff3b30")
        : "rgba(255,255,255,0.10)";
      roundRect(ctx, x0 + i * (bw + gap), y, bw, 9, 3);
      ctx.fill();
    }
  }

  tach(ctx, W, H, s) {
    const bw = Math.min(W * 0.46, 520);
    const bh = 20;
    const x = W / 2 - bw / 2;
    const y = H - 128;

    panel(ctx, x - 14, y - 30, bw + 28, 140, 10);

    // rpm bar
    ctx.fillStyle = "rgba(255,255,255,0.08)";
    roundRect(ctx, x, y, bw, bh, 5); ctx.fill();
    const frac = clamp(s.rpm / s.revLimit, 0, 1);
    const grad = ctx.createLinearGradient(x, 0, x + bw, 0);
    grad.addColorStop(0, "#2f9e6e");
    grad.addColorStop(0.55, GOLD);
    grad.addColorStop(0.86, "#ff9f0a");
    grad.addColorStop(1, "#ff3b30");
    ctx.fillStyle = grad;
    roundRect(ctx, x, y, Math.max(3, bw * frac), bh, 5); ctx.fill();

    // The CFD sweep's peak-power rpm, marked so the driver can use the curve.
    for (const [rpm, label] of [[s.peakTorqueRpm, "Mt"], [s.peakPowerRpm, "Pk"]]) {
      const px = x + bw * clamp(rpm / s.revLimit, 0, 1);
      ctx.strokeStyle = "rgba(255,255,255,0.55)";
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(px, y - 4); ctx.lineTo(px, y + bh + 4); ctx.stroke();
      ctx.fillStyle = "rgba(255,255,255,0.55)";
      ctx.font = "9px ui-monospace, monospace";
      ctx.textAlign = "center";
      ctx.fillText(label, px, y - 7);
    }

    ctx.textAlign = "left";
    ctx.fillStyle = "rgba(255,255,255,0.6)";
    ctx.font = "11px ui-monospace, monospace";
    ctx.fillText(`${Math.round(s.rpm)} rpm`, x, y + bh + 18);

    // gear
    ctx.textAlign = "center";
    ctx.fillStyle = s.shifting ? "rgba(255,255,255,0.35)" : GOLD;
    ctx.font = "600 44px ui-monospace, monospace";
    ctx.fillText(String(s.gear), W / 2, y + bh + 40);

    // speed
    ctx.textAlign = "right";
    ctx.fillStyle = "#fff";
    ctx.font = "600 30px ui-monospace, monospace";
    ctx.fillText(String(Math.round(s.speedKph)), x + bw, y + bh + 34);
    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.font = "11px ui-monospace, monospace";
    ctx.fillText("km/h", x + bw, y + bh + 48);

    if (s.tractionControl) {
      ctx.textAlign = "left";
      ctx.fillStyle = "#3ddc84";
      ctx.font = "600 10px ui-monospace, monospace";
      ctx.fillText("TC", x, y - 12);
    }

    this.pedalTrace(ctx, x, y + bh + 30, bw * 0.42, s);
  }

  /**
   * Driver inputs: accelerator pedal, the plate the ETC map asked for, and
   * brake. APS against TPS matters because on anything but a linear map the
   * gap between those two bars IS the map, live — the only way to feel what a
   * curve actually did without stopping to look at it.
   */
  pedalTrace(ctx, x, y, w, s) {
    const h = 7, gap = 3;
    const rows = [
      { label: "APS", v: s.pedal, fill: "#e8ecf3" },
      { label: "TPS", v: s.plate, fill: GOLD },
      { label: "BRK", v: s.brake, fill: "#ff4d43" },
    ];
    ctx.font = "8px ui-monospace, monospace";
    rows.forEach((r, i) => {
      const ry = y + i * (h + gap);
      ctx.textAlign = "left";
      ctx.fillStyle = "rgba(255,255,255,0.45)";
      ctx.fillText(r.label, x, ry + h - 0.5);
      const bx = x + 25;
      ctx.fillStyle = "rgba(255,255,255,0.10)";
      roundRect(ctx, bx, ry, w, h, 3); ctx.fill();
      const frac = clamp(r.v, 0, 1);
      if (frac > 0.002) {
        ctx.fillStyle = r.fill;
        roundRect(ctx, bx, ry, Math.max(2, w * frac), h, 3); ctx.fill();
      }
      ctx.textAlign = "left";
      ctx.fillStyle = "rgba(255,255,255,0.6)";
      ctx.fillText(`${Math.round(frac * 100)}`.padStart(3), bx + w + 6, ry + h - 0.5);
    });

    // Name the map so a driver knows which one they are on without pausing.
    ctx.textAlign = "left";
    ctx.fillStyle = "rgba(255,255,255,0.32)";
    ctx.fillText(s.etcName.toUpperCase(), x, y + rows.length * (h + gap) + 7);
  }

  /**
   * Live setup: what the d-pad is pointing at and what it is set to. Shows the
   * delta from where the car started so you always know how far you have
   * wandered from the baseline mid-session.
   */
  setupPanel(ctx, W, H, s) {
    const items = s.setup.items;
    const w = 176, rowH = 22;
    const h = 26 + items.length * rowH;
    const x = W - w - 18, y = H - 132 - 18 - 34 - h - 10;

    panel(ctx, x, y, w, h, 8);
    ctx.textAlign = "left";
    ctx.fillStyle = s.setup.active ? GOLD : "rgba(255,255,255,0.45)";
    ctx.font = "9px ui-monospace, monospace";
    ctx.fillText("SETUP  ◄ ► select   ▲ ▼ adjust", x + 10, y + 15);

    items.forEach((it, i) => {
      const ry = y + 24 + i * rowH;
      if (it.selected) {
        ctx.fillStyle = "rgba(255,198,39,0.12)";
        roundRect(ctx, x + 5, ry - 2, w - 10, rowH - 2, 5); ctx.fill();
        ctx.fillStyle = GOLD;
        ctx.font = "600 10px ui-monospace, monospace";
        ctx.fillText("▸", x + 8, ry + 11);
      }
      ctx.fillStyle = it.selected ? "#fff" : "rgba(255,255,255,0.62)";
      ctx.font = "10px ui-monospace, monospace";
      ctx.fillText(it.short, x + 19, ry + 11);

      ctx.textAlign = "right";
      ctx.font = "600 12px ui-monospace, monospace";
      ctx.fillStyle = it.selected ? GOLD : "#e8ecf3";
      ctx.fillText(`${it.value.toFixed(1)}${it.unit}`, x + w - 40, ry + 11);

      const d = it.value - it.baseline;
      ctx.font = "9px ui-monospace, monospace";
      ctx.fillStyle = Math.abs(d) < 0.05 ? "rgba(255,255,255,0.28)"
        : d > 0 ? "#3ddc84" : "#4a9eff";
      ctx.fillText(Math.abs(d) < 0.05 ? "base" : `${d > 0 ? "+" : ""}${d.toFixed(1)}`,
        x + w - 8, ry + 11);
      ctx.textAlign = "left";
    });
  }

  timing(ctx, s) {
    const x = 18, y = 18, w = 216;
    panel(ctx, x, y, w, 108, 10);
    ctx.textAlign = "left";

    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.font = "10px ui-monospace, monospace";
    ctx.fillText(s.trackName.toUpperCase(), x + 12, y + 18);

    ctx.fillStyle = "#fff";
    ctx.font = "600 30px ui-monospace, monospace";
    ctx.fillText(s.lapTimeText, x + 12, y + 50);

    ctx.font = "11px ui-monospace, monospace";
    ctx.fillStyle = "rgba(255,255,255,0.6)";
    ctx.fillText(`LAST  ${s.lastLapText}`, x + 12, y + 70);
    ctx.fillStyle = GOLD;
    ctx.fillText(`BEST  ${s.bestLapText}`, x + 12, y + 86);

    ctx.fillStyle = "rgba(255,255,255,0.45)";
    ctx.fillText(s.closed ? `LAP ${s.lap}` : `${s.progressPct}%`, x + 12, y + 101);
  }

  penalties(ctx, W, s) {
    const w = 150, x = W - w - 18, y = 18;
    panel(ctx, x, y, w, 76, 10);
    ctx.textAlign = "left";

    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.font = "10px ui-monospace, monospace";
    ctx.fillText("PENALTIES", x + 12, y + 18);

    ctx.font = "600 15px ui-monospace, monospace";
    ctx.fillStyle = s.cones > 0 ? "#ff9f0a" : "rgba(255,255,255,0.75)";
    ctx.fillText(`CONES  ${s.cones}`, x + 12, y + 40);
    ctx.fillStyle = s.offCourse > 0 ? "#ff3b30" : "rgba(255,255,255,0.75)";
    ctx.fillText(`OFF    ${s.offCourse}`, x + 12, y + 58);

    ctx.textAlign = "right";
    ctx.fillStyle = s.penaltyS > 0 ? "#ff9f0a" : "rgba(255,255,255,0.4)";
    ctx.font = "11px ui-monospace, monospace";
    ctx.fillText(`+${s.penaltyS.toFixed(1)}s`, x + w - 12, y + 18);
  }

  minimap(ctx, H, s) {
    const size = 178, x = 18, y = H - size - 18;
    panel(ctx, x, y, size, size, 10);

    const t = s.track;
    if (!this.minimapCache || this.minimapCache.track !== t) {
      // Fit the course into the box once; only the car dot moves per frame.
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      for (const [px, py] of t.center) {
        minX = Math.min(minX, px); maxX = Math.max(maxX, px);
        minY = Math.min(minY, py); maxY = Math.max(maxY, py);
      }
      const pad = 16;
      const scale = Math.min((size - pad * 2) / (maxX - minX), (size - pad * 2) / (maxY - minY));
      this.minimapCache = {
        track: t, scale,
        cx: (minX + maxX) / 2, cy: (minY + maxY) / 2,
      };
    }
    const m = this.minimapCache;
    const toPx = (px, py) => [
      x + size / 2 + (px - m.cx) * m.scale,
      y + size / 2 - (py - m.cy) * m.scale,
    ];

    ctx.beginPath();
    for (let i = 0; i < t.center.length; i += 2) {
      const [px, py] = toPx(t.center[i][0], t.center[i][1]);
      i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
    }
    if (t.closed) ctx.closePath();
    ctx.strokeStyle = "rgba(255,255,255,0.34)";
    ctx.lineWidth = 3;
    ctx.stroke();

    // start/finish tick
    const [sx, sy] = toPx(t.center[0][0], t.center[0][1]);
    ctx.fillStyle = GOLD;
    ctx.fillRect(sx - 3, sy - 3, 6, 6);

    // car
    const [cx, cy] = toPx(s.carX, s.carY);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-s.carPsi + Math.PI / 2);
    ctx.fillStyle = MAROON;
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(0, -6); ctx.lineTo(4, 5); ctx.lineTo(-4, 5);
    ctx.closePath();
    ctx.fill(); ctx.stroke();
    ctx.restore();
  }

  gg(ctx, W, H, s) {
    const size = 132, x = W - size - 18, y = H - size - 18;
    panel(ctx, x, y, size, size, 10);
    const cx = x + size / 2, cy = y + size / 2;
    const r = size / 2 - 16;

    ctx.strokeStyle = "rgba(255,255,255,0.18)";
    ctx.lineWidth = 1;
    for (const g of [1, 2]) {
      ctx.beginPath();
      ctx.arc(cx, cy, (r * g) / 2, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.beginPath();
    ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy);
    ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r);
    ctx.stroke();

    ctx.fillStyle = "rgba(255,255,255,0.4)";
    ctx.font = "9px ui-monospace, monospace";
    ctx.textAlign = "center";
    ctx.fillText("2g", cx, cy - r - 4);

    // Trail of recent g, then the live dot.
    for (let i = 0; i < s.ggTrail.length; i++) {
      const p = s.ggTrail[i];
      const a = (i / s.ggTrail.length) * 0.5;
      ctx.fillStyle = `rgba(255,198,39,${a.toFixed(3)})`;
      ctx.beginPath();
      ctx.arc(cx + (p.ay / 2) * r, cy - (p.ax / 2) * r, 1.6, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.fillStyle = "#fff";
    ctx.beginPath();
    ctx.arc(cx + (s.ayG / 2) * r, cy - (s.axG / 2) * r, 4, 0, Math.PI * 2);
    ctx.fill();
  }

  balance(ctx, W, H, s) {
    // Which axle is closer to its limit. This is the bicycle model's own
    // utilisation, not a guess, so it reads like a real balance trace.
    const w = 132, x = W - w - 18, y = H - 132 - 18 - 34;
    panel(ctx, x, y, w, 26, 8);
    const cx = x + w / 2;
    ctx.fillStyle = "rgba(255,255,255,0.18)";
    ctx.fillRect(cx - 0.5, y + 5, 1, 16);

    const b = clamp(s.balance, -0.5, 0.5) / 0.5;
    const bw = (w / 2 - 8) * Math.abs(b);
    ctx.fillStyle = b > 0 ? "#ff3b30" : "#4a9eff";
    if (b > 0) ctx.fillRect(cx, y + 8, bw, 10);
    else ctx.fillRect(cx - bw, y + 8, bw, 10);

    ctx.font = "8px ui-monospace, monospace";
    ctx.fillStyle = "rgba(255,255,255,0.45)";
    ctx.textAlign = "left";
    ctx.fillText("UNDER", x + 6, y + 22);
    ctx.textAlign = "right";
    ctx.fillText("OVER", x + w - 6, y + 22);
  }

  message(ctx, W, H, s) {
    if (!s.message) return;
    ctx.textAlign = "center";
    ctx.font = "600 26px ui-monospace, monospace";
    const tw = ctx.measureText(s.message).width;
    panel(ctx, W / 2 - tw / 2 - 20, H * 0.24, tw + 40, 46, 10);
    ctx.fillStyle = GOLD;
    ctx.fillText(s.message, W / 2, H * 0.24 + 31);
  }

  paused(ctx, W, H) {
    ctx.fillStyle = "rgba(6,8,12,0.62)";
    ctx.fillRect(0, 0, W, H);
    ctx.textAlign = "center";
    ctx.fillStyle = "#fff";
    ctx.font = "600 40px ui-monospace, monospace";
    ctx.fillText("PAUSED", W / 2, H / 2 - 6);
    ctx.font = "13px ui-monospace, monospace";
    ctx.fillStyle = "rgba(255,255,255,0.6)";
    ctx.fillText("Menu / Esc  resume", W / 2, H / 2 + 22);
    ctx.fillText("L3 / H  home screen", W / 2, H / 2 + 42);
    ctx.fillText("View / Backspace  restart run", W / 2, H / 2 + 62);
  }
}

// ---------------------------------------------------------------- drawing ---

function panel(ctx, x, y, w, h, r) {
  ctx.fillStyle = "rgba(10,13,20,0.62)";
  roundRect(ctx, x, y, w, h, r);
  ctx.fill();
  ctx.strokeStyle = "rgba(255,255,255,0.10)";
  ctx.lineWidth = 1;
  ctx.stroke();
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

function clamp(x, lo, hi) { return x < lo ? lo : x > hi ? hi : x; }
