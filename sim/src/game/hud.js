// 2D overlay HUD, drawn on a canvas above the WebGL view.
//
// Laid out like a steering-wheel dash rather than a video-game UI: shift lights
// across the top of the field of view, a linear tach where a Motec would put
// it, and the timing block where the driver can find it on a straight. Colours
// are Sun Devil maroon and gold.

/** The CASE, 137 x 84 mm -- what the screen overlay draws. */
const DASH_UNIT_ASPECT = 137 / 84;
/** The DISPLAY, a 5-inch 800x480 panel, 108 x 65 mm -- what the quad on the
 *  car shows. The two are different rectangles and drawing one at the
 *  other's proportions is how the layout ends up stretched. */
const DASH_SCREEN_ASPECT = 108 / 65;

const GOLD = "#FFC627";
const MAROON = "#8C1D40";

/**
 * How much of the dash to draw.
 *
 * Everything here is useful to somebody, and all of it at once is a windscreen
 * you cannot see out of -- nine blocks around the edge of the frame, two of
 * them (the g-g trace and the balance bar) being engineering instruments
 * rather than driving ones. So the driver picks, `H` cycles, and the choice is
 * remembered per machine.
 *
 * `clean` is the default and is what a real dash gives you: shift lights, a
 * tach, the clock, the delta and the penalties. The analysis instruments are
 * one keypress away and are on the replay in far more detail anyway.
 */
export const HUD_DENSITY = ["clean", "full", "minimal"];

const SHOWS = {
  // `dash` is the Strada unit: it carries the shift LEDs, the rpm bar, the
  // gear, the lap and lap time, the penalties and the course. So the separate
  // `shiftLights` / `tach` / `timing` / `penalties` blocks are the SAME
  // information a second time, and they are off wherever the dash is on.
  //
  // `full` keeps the engineering instruments -- the g-g trace, the minimap,
  // the balance bar -- around the edges of the dash, which is where a test
  // engineer's overlay would go and where nothing on a real car goes.
  full: {
    dash: true, shiftLights: false, tach: false, timing: false,
    delta: true, penalties: false,
    minimap: true, gg: true, balance: true, setup: "always", message: true,
  },
  clean: {
    dash: true, shiftLights: false, tach: false, timing: false,
    delta: true, penalties: false,
    minimap: false, gg: false, balance: true, setup: "active", message: true,
  },
  minimal: {
    dash: true, shiftLights: false, tach: false, timing: false,
    delta: false, penalties: false,
    minimap: false, gg: false, balance: false, setup: "active", message: true,
  },
};

const DENSITY_KEY = "fsae-sim:hud-density";

/**
 * Which dash the driver wants.
 *
 *   auto    the panel on the car, and the overlay only from cameras where the
 *           real one is a postage stamp. What a real cockpit gives you.
 *   overlay the old screen dash, always. For anyone who does not get on with
 *           reading a panel at its real angular size -- which is small, and on
 *           a monitor at arm's length it is smaller than it is in the car.
 *   car     the panel on the car and nothing else, ever.
 *
 * The car's own panel is always drawn: it is a part of the car, and switching
 * it off would leave a hole in the cockpit.
 */
export const DASH_MODES = ["auto", "overlay", "car"];
const DASH_MODE_KEY = "fsae-sim:dash-mode";

function savedDashMode() {
  try {
    const v = localStorage.getItem(DASH_MODE_KEY);
    return DASH_MODES.includes(v) ? v : "auto";
  } catch {
    return "auto";
  }
}

/** Seconds the setup adjuster stays on screen after the last nudge. */
const SETUP_HOLD_S = 4;

function savedDensity() {
  try {
    const v = localStorage.getItem(DENSITY_KEY);
    return HUD_DENSITY.includes(v) ? v : "clean";
  } catch {
    return "clean";
  }
}

export class Hud {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.minimapCache = null;
    this.density = savedDensity();
    /** Set by the game each frame from the camera; see `draw`. */
    this.overlayDash = true;
    this.lastState = null;
    this.dashMode = savedDashMode();
    /** When the setup adjuster was last touched, so it can fade out of the
     *  way rather than sitting there for the whole run. */
    this._setupSeen = 0;
  }

  /** Next dash mode, wrapping. Returns the new one so the caller can say so. */
  cycleDashMode() {
    const i = DASH_MODES.indexOf(this.dashMode);
    this.dashMode = DASH_MODES[(i + 1) % DASH_MODES.length];
    try { localStorage.setItem(DASH_MODE_KEY, this.dashMode); } catch { /* fine */ }
    return this.dashMode;
  }

  setDashMode(mode) {
    if (!DASH_MODES.includes(mode)) return this.dashMode;
    this.dashMode = mode;
    try { localStorage.setItem(DASH_MODE_KEY, mode); } catch { /* fine */ }
    return mode;
  }

  /** Next density, wrapping. Returns the new one so the caller can say so. */
  cycleDensity() {
    const i = HUD_DENSITY.indexOf(this.density);
    this.density = HUD_DENSITY[(i + 1) % HUD_DENSITY.length];
    try { localStorage.setItem(DENSITY_KEY, this.density); } catch { /* fine */ }
    return this.density;
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

  /** Blank the overlay (the launch screen shows the scene without a HUD). */
  clear() {
    const { w, h } = this.resize();
    this.ctx.clearRect(0, 0, w, h);
  }

  draw(s) {
    // Held so the car's dash panel can be repainted from the same state on a
    // frame where it is due, without the game having to build it twice.
    this.lastState = s;
    const { w, h } = this.resize();
    const ctx = this.ctx;
    ctx.clearRect(0, 0, w, h);
    ctx.save();
    // Work in CSS pixels regardless of DPR.
    ctx.scale(this.dpr, this.dpr);
    const W = w / this.dpr, H = h / this.dpr;

    const show = SHOWS[this.density] ?? SHOWS.clean;
    // The setup adjuster appears when it is being used and fades a few seconds
    // after the driver stops. It is the one panel that is genuinely transient:
    // two values you nudge on the straight, not something to watch.
    if (s.setup?.active) this._setupSeen = performance.now();
    const setupAge = (performance.now() - this._setupSeen) / 1000;
    const showSetup = show.setup === "always" ||
      (show.setup === "active" && this._setupSeen > 0 && setupAge < SETUP_HOLD_S);

    if (show.shiftLights) this.shiftLights(ctx, W, s);
    // `overlayDash` is what the CAMERA wants; `dashMode` is what the DRIVER
    // wants, and the driver wins.
    const wantOverlay = this.dashMode === "overlay" ? true
      : this.dashMode === "car" ? false
      : this.overlayDash !== false;
    if (show.dash && wantOverlay) this.dash(ctx, W, H, s);
    if (show.tach) this.tach(ctx, W, H, s);
    if (show.timing) this.timing(ctx, s);
    if (show.delta) this.delta(ctx, W, H, s);
    if (show.penalties) this.penalties(ctx, W, s);
    if (show.minimap) this.minimap(ctx, H, s);
    if (show.gg) this.gg(ctx, W, H, s);
    if (show.balance) this.balance(ctx, W, H, s);
    if (showSetup) {
      // Fade the last half second so it leaves rather than blinks out.
      const fade = show.setup === "always"
        ? 1
        : Math.min(1, Math.max(0, (SETUP_HOLD_S - setupAge) / 0.5));
      ctx.save();
      ctx.globalAlpha = fade;
      this.setupPanel(ctx, W, H, s);
      ctx.restore();
    }
    if (show.message) this.message(ctx, W, H, s);
    if (s.paused) this.paused(ctx, W, H);

    ctx.restore();
  }

  // ------------------------------------------------------------ components ---

  // ------------------------------------------------------------ the dash ---
  //
  // Laid out as an AiM Strada MX Lite, because that is the dash in the car.
  // Driving a simulator whose instruments sit somewhere else and read in a
  // different order teaches the wrong glance: the whole point of a
  // driver-in-loop is that the look you practise is the look you make.
  //
  // `dash()` only decides WHERE the unit goes. `drawDash()` renders it into a
  // plain rectangle and touches nothing outside it, so the same function draws
  // the screen overlay AND the texture on the car's own dash panel. Nothing in
  // it may read `W`/`H` or the canvas size.
  //
  // Every dimension below is a fraction of the SCREEN's height, measured off
  // the real unit, so the proportions hold at any size.

  dash(ctx, W, H, s) {
    const w = Math.min(W * 0.30, 400);
    const h = w / DASH_UNIT_ASPECT;
    this.drawDash(ctx, W / 2 - w / 2, H - h - 14, w, h, s);
  }

  /**
   * The unit into `(x, y, w, h)`.
   *
   * `chrome` decides whether the CASE is drawn. On the screen overlay it is:
   * there is no geometry, and a floating rectangle of numbers does not read as
   * a dash. On the car it is NOT: the case, its bezel and its buttons are real
   * geometry there, and painting a second case into the texture would put a
   * picture of a dash inside a dash.
   *
   * With `chrome: false` the whole rect IS the display, which is why the two
   * callers pass different aspect ratios -- `DASH_UNIT_ASPECT` for the case,
   * `DASH_SCREEN_ASPECT` for the display.
   */
  drawDash(ctx, x, y, w, h, s, { chrome = true } = {}) {
    ctx.save();

    let sx = x;
    let sy = y;
    let sw = w;
    let sh = h;

    if (chrome) {
      ctx.fillStyle = "#0a0a0c";
      roundRect(ctx, x, y, w, h, h * 0.10);
      ctx.fill();
      ctx.strokeStyle = "rgba(255,255,255,0.12)";
      ctx.lineWidth = Math.max(1, h * 0.006);
      roundRect(ctx, x + 0.5, y + 0.5, w - 1, h - 1, h * 0.10);
      ctx.stroke();

      // The button columns either side -- 14.5 mm of case on the real unit,
      // which is most of why it is recognisable as this unit.
      const bez = w * 0.106;
      this.dashBezel(ctx, x, y, bez, h, "left");
      this.dashBezel(ctx, x + w - bez, y, bez, h, "right");
      sx = x + bez;
      sw = w - bez * 2;
      sy = y + h * 0.075;
      sh = h * 0.85;
    }

    ctx.fillStyle = "#000";
    roundRect(ctx, sx, sy, sw, sh, sh * 0.03);
    ctx.fill();

    // Vertical budget, as fractions of the screen height.
    const pad = sw * 0.022;
    const ix = sx + pad;
    const iw = sw - pad * 2;
    let cy = sy + sh * 0.035;

    cy += this.dashLeds(ctx, ix, cy, iw, sh * 0.085, s) + sh * 0.035;
    cy += this.dashRpmBar(ctx, ix, cy, iw, sh * 0.125, s) + sh * 0.012;
    cy += this.dashRpmScale(ctx, ix, cy, iw, sh * 0.075, s) + sh * 0.020;

    const footH = sh * 0.10;
    this.dashMiddle(ctx, ix, cy, iw, sy + sh - footH - cy - sh * 0.02, s);
    this.dashFooter(ctx, ix, sy + sh - footH, iw, footH, s);
    ctx.restore();
  }

  /** One side bezel: three button marks and the legends beside them. */
  dashBezel(ctx, x, y, w, h, side) {
    const marks = side === "left"
      ? [GOLD, "#e8ecf3", "#e8ecf3"]
      : ["#4aa8ff", "#e8ecf3", "#e8ecf3"];
    const bw = w * 0.46;
    const bh = h * 0.055;
    const bx = side === "left" ? x + w - bw - w * 0.10 : x + w * 0.10;
    for (let i = 0; i < marks.length; i++) {
      ctx.fillStyle = marks[i];
      roundRect(ctx, bx, y + h * (0.20 + i * 0.13), bw, bh, bh * 0.4);
      ctx.fill();
    }
    ctx.fillStyle = "rgba(255,255,255,0.42)";
    ctx.font = `${(h * 0.055).toFixed(1)}px ui-monospace, monospace`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    const cx = x + w / 2;
    ctx.fillText(side === "left" ? "MENU" : "MEM", cx, y + h * 0.68);
    ctx.fillText(side === "left" ? "\u00ab" : "OK", cx, y + h * 0.745);
    ctx.fillText(side === "left" ? "\u00bb" : "VIEW", cx, y + h * 0.87);
  }

  /**
   * The LED row.
   *
   * Ten of them, four green, three amber, three red, exactly as the unit is
   * shipped. Flashes whole at the limiter.
   */
  dashLeds(ctx, x, y, w, h, s) {
    const n = 10;
    const gap = w * 0.010;
    const bw = (w - gap * (n - 1)) / n;
    const from = s.shiftRpm * 0.72;
    const frac = clamp((s.rpm - from) / Math.max(1, s.revLimit - from), 0, 1);
    const lit = Math.floor(frac * n + 1e-6);
    const flash = s.rpm >= s.revLimit - 120 && Math.floor(performance.now() / 70) % 2 === 0;
    for (let i = 0; i < n; i++) {
      const on = flash || i < lit;
      const colour = i < 4 ? "#31d158" : i < 7 ? "#ffd60a" : "#ff453a";
      ctx.fillStyle = on ? colour : "rgba(255,255,255,0.08)";
      roundRect(ctx, x + i * (bw + gap), y, bw, h, h * 0.32);
      ctx.fill();
      if (on) {
        ctx.save();
        ctx.globalAlpha = 0.45;
        ctx.shadowColor = colour;
        ctx.shadowBlur = h;
        ctx.fill();
        ctx.restore();
      }
    }
    return h;
  }

  /**
   * The rpm bar.
   *
   * A FIXED rainbow whose lit fraction moves, not a bar whose colour slides
   * with the needle: the same rpm is then always the same colour in the same
   * place, which is the only reason a bar is quicker to read than a number.
   * The unlit remainder stays faintly visible, the way the real panel's
   * backlight leaves it, so you can see where the next colour begins.
   */
  dashRpmBar(ctx, x, y, w, h, s) {
    const grad = ctx.createLinearGradient(x, 0, x + w, 0);
    grad.addColorStop(0.00, "#1f6fff");
    grad.addColorStop(0.20, "#00c8ff");
    grad.addColorStop(0.42, "#31d158");
    grad.addColorStop(0.64, "#ffd60a");
    grad.addColorStop(0.84, "#ff7a1a");
    grad.addColorStop(1.00, "#ff2fd0");
    ctx.save();
    roundRect(ctx, x, y, w, h, h * 0.16);
    ctx.clip();
    ctx.globalAlpha = 0.22;
    ctx.fillStyle = grad;
    ctx.fillRect(x, y, w, h);
    ctx.globalAlpha = 1;
    ctx.fillRect(x, y, w * clamp(s.rpm / Math.max(1, s.revLimit), 0, 1), h);
    ctx.restore();
    return h;
  }

  /** Thousands under the bar, and the two marks the engine actually has --
   *  peak torque and peak power, which is what the bar is FOR. */
  dashRpmScale(ctx, x, y, w, h, s) {
    const limit = Math.max(1, s.revLimit);
    ctx.textBaseline = "top";
    ctx.textAlign = "center";
    ctx.font = `${(h * 0.80).toFixed(1)}px ui-monospace, monospace`;
    const step = limit > 16000 ? 4000 : 2000;
    for (let rpm = 0; rpm <= limit + 1; rpm += step) {
      ctx.fillStyle = "rgba(255,255,255,0.45)";
      ctx.fillText(String(Math.round(rpm / 1000)), x + w * (rpm / limit), y);
    }
    for (const [rpm, mark] of [[s.peakTorqueRpm, "#ffd60a"], [s.peakPowerRpm, "#ff2fd0"]]) {
      if (!rpm) continue;
      ctx.fillStyle = mark;
      ctx.fillRect(x + w * clamp(rpm / limit, 0, 1) - 0.5, y - h * 0.30, 1, h * 0.28);
    }
    return h;
  }

  /**
   * The centre block: six fields round a big gear, with the lap number and the
   * running lap time tucked either side of the gear's shoulders.
   *
   * The real unit fills this area -- the numbers are nearly as tall as the
   * rows that hold them, and only the gear is bigger. Anything smaller reads
   * as a web page rather than a dash.
   */
  dashMiddle(ctx, x, y, w, h, s) {
    const colW = w * 0.255;
    const rowH = h / 3;
    const d = s.delta;
    const live = d && d.hasReference && d.delta != null;

    const left = [
      ["SPEED", "KM/H", Math.round(s.speedKph).toString(), "#e8ecf3"],
      ["LAT", "G", fixed(Math.abs(s.ayG), 2), "#e8ecf3"],
      ["BRAKE", "%", Math.round(clamp(s.brake, 0, 1) * 100).toString(), "#e8ecf3"],
    ];
    const right = [
      ["DELTA", "S", live ? signed(d.delta, 2) : "--.--",
        live ? (d.delta <= 0 ? "#31d158" : "#ff453a") : "rgba(255,255,255,0.30)"],
      ["LAST", "", s.lastLapText, "#e8ecf3"],
      ["BEST", "", s.bestLapText, "#ffd60a"],
    ];
    for (let i = 0; i < 3; i++) {
      this.dashField(ctx, x, y + i * rowH, colW, rowH, left[i], "left");
      this.dashField(ctx, x + w - colW, y + i * rowH, colW, rowH, right[i], "right");
    }

    const cx = x + w / 2;
    ctx.textAlign = "center";
    ctx.textBaseline = "alphabetic";

    // Lap number, green, and the running lap time, magenta -- the two coloured
    // readouts on the real screen, sitting either side of the gear's shoulder.
    const shoulder = w * 0.155;
    ctx.fillStyle = "#31d158";
    ctx.font = `600 ${(h * 0.20).toFixed(1)}px ui-monospace, monospace`;
    ctx.fillText(String(s.lap ?? 0), cx - shoulder, y + h * 0.20);
    ctx.fillStyle = "#ff2fd0";
    ctx.font = `600 ${(h * 0.175).toFixed(1)}px ui-monospace, monospace`;
    ctx.fillText(s.lapTimeText, cx + shoulder, y + h * 0.20);

    ctx.fillStyle = "rgba(255,255,255,0.34)";
    ctx.font = `${(h * 0.070).toFixed(1)}px ui-monospace, monospace`;
    ctx.fillText("LAP", cx - shoulder, y + h * 0.285);
    ctx.fillText("LAP TIME", cx + shoulder, y + h * 0.285);

    // The gear, big enough to be read without looking straight at it.
    ctx.fillStyle = s.shifting ? "rgba(255,255,255,0.30)" : "#ffffff";
    ctx.font = `600 ${(h * 0.60).toFixed(1)}px ui-monospace, monospace`;
    ctx.fillText(gearText(s), cx, y + h * 0.83);

    // rpm under it, small: the bar is the primary read.
    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.font = `${(h * 0.095).toFixed(1)}px ui-monospace, monospace`;
    ctx.fillText(String(Math.round(s.rpm)), cx, y + h * 0.975);

    if (s.tractionControl) {
      ctx.fillStyle = "#31d158";
      ctx.font = `600 ${(h * 0.075).toFixed(1)}px ui-monospace, monospace`;
      ctx.fillText("TC", cx + w * 0.30, y + h * 0.975);
    }
  }

  /** One numeric field: a big value with its name and unit beneath. */
  dashField(ctx, x, y, w, h, [label, unit, value, colour], align) {
    const edge = align === "left" ? x : x + w;
    ctx.textAlign = align === "left" ? "left" : "right";
    ctx.textBaseline = "alphabetic";
    ctx.fillStyle = colour;
    ctx.font = `600 ${(h * 0.52).toFixed(1)}px ui-monospace, monospace`;
    ctx.fillText(value, edge, y + h * 0.54);
    ctx.fillStyle = "rgba(255,255,255,0.34)";
    ctx.font = `${(h * 0.185).toFixed(1)}px ui-monospace, monospace`;
    ctx.fillText(unit ? `${label}  ${unit}` : label, edge, y + h * 0.80);
  }

  /** Penalties, the course, and the clock -- the real unit's status strip. */
  dashFooter(ctx, x, y, w, h, s) {
    ctx.font = `${(h * 0.52).toFixed(1)}px ui-monospace, monospace`;
    ctx.textBaseline = "middle";
    const cy = y + h * 0.55;
    const pen = (s.cones ?? 0) + (s.offCourse ?? 0);

    ctx.textAlign = "left";
    ctx.fillStyle = pen > 0 ? "#ff453a" : "#31d158";
    ctx.fillText(
      s.offCourse > 0
        ? `OFF COURSE - NO TIME  (${s.cones}C ${s.offCourse}OFF)`
        : pen > 0 ? `${s.cones}C +${(s.penaltyS ?? 0).toFixed(0)}s` : "CLEAN",
      x, cy,
    );
    ctx.textAlign = "center";
    ctx.fillStyle = "rgba(255,255,255,0.42)";
    ctx.fillText(String(s.trackName ?? "").toUpperCase(), x + w / 2, cy);
    ctx.textAlign = "right";
    ctx.fillText(clockText(), x + w, cy);
  }

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
   * gap between those two bars IS the map, live -- the only way to feel what a
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
    ctx.fillText("SETUP   < > select   ^ v adjust", x + 10, y + 15);

    items.forEach((it, i) => {
      const ry = y + 24 + i * rowH;
      if (it.selected) {
        ctx.fillStyle = "rgba(255,198,39,0.12)";
        roundRect(ctx, x + 5, ry - 2, w - 10, rowH - 2, 5); ctx.fill();
        ctx.fillStyle = GOLD;
        ctx.font = "600 10px ui-monospace, monospace";
        ctx.fillText(">", x + 8, ry + 11);
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

  /**
   * Live delta to a reference lap, and where this lap has been losing.
   *
   * Centred under the shift lights, because it is read the same way they are:
   * at a glance, on the way into a corner, without moving your eyes off the
   * road. The number is the gap at this point on the course; the bar under it
   * is the same thing at a scale you can read peripherally; and the trace
   * below that is the whole lap so far, which is the part that answers "where
   * am I losing it" rather than "am I losing it".
   */
  delta(ctx, W, H, s) {
    const d = s.delta;
    if (!d || !d.hasReference) return;

    const w = 190, h = 54;
    const x = W / 2 - w / 2;
    const y = 34;
    panel(ctx, x, y, w, h, 8);

    const value = d.delta;
    const known = value != null;
    // Green when up on the reference, red when down. A tenth either side of
    // level is drawn as level: a delta that flickers colour every corner is
    // noise a driver learns to ignore.
    const col = !known ? "rgba(255,255,255,0.35)"
      : value < -0.05 ? "#3ddc84"
      : value > 0.05 ? "#ff453a"
      : "rgba(255,255,255,0.8)";

    ctx.textAlign = "center";
    ctx.fillStyle = col;
    ctx.font = "600 26px ui-monospace, monospace";
    ctx.fillText(
      known ? `${value >= 0 ? "+" : "−"}${Math.abs(value).toFixed(2)}` : "--.--",
      W / 2, y + 26,
    );

    // The bar: full width is half a second either way, which is the range a
    // driver is actually working in. Past that it pins and the number carries
    // the rest.
    const bw = w - 24, bx = x + 12, by = y + 34, bh = 5;
    ctx.fillStyle = "rgba(255,255,255,0.10)";
    roundRect(ctx, bx, by, bw, bh, 2.5);
    ctx.fill();
    if (known) {
      const frac = clamp(value / 0.5, -1, 1);
      const half = bw / 2;
      const len = Math.abs(frac) * half;
      ctx.fillStyle = col;
      roundRect(ctx, bx + half + Math.min(0, frac) * half, by, Math.max(1.5, len), bh, 2.5);
      ctx.fill();
    }
    // Centre tick, so "level" has a mark to sit against.
    ctx.fillStyle = "rgba(255,255,255,0.35)";
    ctx.fillRect(bx + bw / 2 - 0.5, by - 2, 1, bh + 4);

    ctx.font = "9px ui-monospace, monospace";
    ctx.fillStyle = "rgba(255,255,255,0.45)";
    const label = d.referenceLabel ?? "reference";
    ctx.fillText(
      d.referenceLapS != null ? `vs ${label} ${fmtShort(d.referenceLapS)}` : `vs ${label}`,
      W / 2, y + h - 4,
    );
    ctx.textAlign = "left";

    this.deltaTrace(ctx, W, x, y + h + 6, w, 34, d);
  }

  /**
   * The delta all the way round the lap so far.
   *
   * This is the answer to "where am I losing time": the line rises through
   * the corners that cost and is flat through the ones that did not. It is
   * drawn small and low-contrast on purpose -- it is for the straights and
   * for the replay, not for the braking zone.
   */
  deltaTrace(ctx, W, x, y, w, h, d) {
    const pts = d.trace;
    if (!pts || pts.length < 2) return;
    panel(ctx, x, y, w, h, 6);

    // Scale to whatever this lap has actually done, floored so a tidy lap
    // does not magnify a hundredth into a mountain.
    const span = Math.max(0.25, Math.abs(d.bestDelta ?? 0), Math.abs(d.worstDelta ?? 0));
    const mid = y + h / 2;
    const sx = (v) => x + 4 + (v / d.lengthM) * (w - 8);
    const sy = (v) => mid - clamp(v / span, -1, 1) * (h / 2 - 4);

    ctx.strokeStyle = "rgba(255,255,255,0.18)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x + 4, mid);
    ctx.lineTo(x + w - 4, mid);
    ctx.stroke();

    // Two passes so the colour tells you which side of the reference each
    // part of the lap was on, rather than one line in one colour.
    for (const [sign, colour] of [[-1, "#3ddc84"], [1, "#ff453a"]]) {
      ctx.beginPath();
      let drawing = false;
      for (const p of pts) {
        const on = sign < 0 ? p.delta < 0 : p.delta >= 0;
        if (on) {
          const px = sx(p.s), py = sy(p.delta);
          if (drawing) ctx.lineTo(px, py);
          else { ctx.moveTo(px, py); drawing = true; }
        } else {
          drawing = false;
        }
      }
      ctx.strokeStyle = colour;
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // Where the car is now.
    const last = pts[pts.length - 1];
    ctx.fillStyle = "rgba(255,255,255,0.85)";
    ctx.beginPath();
    ctx.arc(sx(last.s), sy(last.delta), 2, 0, Math.PI * 2);
    ctx.fill();
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
    // The pause menu (index.html) carries the title and the actions; the
    // HUD only dims the frame behind it.
  }
}

// ---------------------------------------------------------------- drawing ---

function panel(ctx, x, y, w, h, r) {
  ctx.fillStyle = "rgba(12,15,20,0.66)";
  roundRect(ctx, x, y, w, h, r);
  ctx.fill();
  ctx.strokeStyle = "rgba(255,255,255,0.09)";
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

/** A lap time short enough to sit under the delta without crowding it. */
function fmtShort(seconds) {
  if (seconds == null || !isFinite(seconds)) return "--.--";
  const m = Math.floor(seconds / 60);
  const r = seconds - m * 60;
  return m > 0 ? `${m}:${r.toFixed(2).padStart(5, "0")}` : r.toFixed(2);
}

/** Gear as a driver reads it: N for neutral, R for reverse, else the number. */
function gearText(s) {
  const g = s.gear;
  if (g === 0) return "N";
  if (g < 0) return "R";
  return String(g);
}

function fixed(v, dp) {
  return Number.isFinite(v) ? v.toFixed(dp) : (0).toFixed(dp);
}

function signed(v, dp) {
  if (!Number.isFinite(v)) return (0).toFixed(dp);
  return (v > 0 ? "+" : v < 0 ? "-" : " ") + Math.abs(v).toFixed(dp);
}

/** Wall clock, where the real dash shows the time of day. */
function clockText() {
  const d = new Date();
  return `${String(d.getHours()).padStart(2, "0")}:${String(d.getMinutes()).padStart(2, "0")}`;
}
