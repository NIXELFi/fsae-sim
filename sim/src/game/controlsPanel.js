// The controls settings panel on the home screen.
//
// Generated from `editableSettings()` rather than hand-built, for the same
// reason the vehicle spec sheet is: a hand-built form drifts from what the code
// actually reads, and the drift is invisible until a slider silently does
// nothing.
//
// The live axis monitor is not a debugging leftover. Wheels and pedal sets do
// not use the Gamepad API's standard mapping and every vendor assigns axes
// differently, so there is no table that is correct for all of them. Watching
// the raw numbers move while you press a pedal is the only reliable way to find
// out which axis it is on, and calibrating against the values that axis really
// produces is the only way to get full travel out of it.

import { editableSettings } from "./controlProfiles.js";

const PROFILE_NOTES = {
  keyboard:
    "A key is a step input, so the steering is ramped in software. These numbers " +
    "are the software standing in for a driver's hands.",
  "gamepad-xbox":
    "Full lock is 28 degrees and the tyre peaks at 8.5 degrees of slip, so a " +
    "linear stick puts the useful travel in the first third. The response curve " +
    "moves the resolution to where the grip is.",
  "gamepad-ps":
    "DualShock and DualSense report the same standard mapping as an Xbox pad; " +
    "the button names differ and the triggers are longer.",
  wheel:
    "Your hands are the input, so the software should get out of the way. " +
    "Deadzone 0 and response curve 1.00 are correct here — anything else is " +
    "smoothing away real information.",
};

export class ControlsPanel {
  /**
   * @param root  container element
   * @param input the Input instance, so the panel can read live axis values
   * @param onChange called after any edit, so the game can re-read the profile
   */
  constructor(root, input, onChange, game = null) {
    this.root = root;
    this.input = input;
    this.onChange = onChange;
    /** The game, for the force feedback status and live torque. Optional. */
    this.game = game;
    this.calibrating = null;
    this.render();
  }

  settings() {
    return this.input.settings;
  }

  render() {
    const s = this.settings();
    const id = s.activeId;
    const profile = s.active();

    this.root.innerHTML = "";

    const head = el("div", "ctl-head");
    head.append(el("h3", null, "Controls"));

    const select = document.createElement("select");
    select.id = "controlProfile";
    for (const pid of s.ids()) {
      const opt = document.createElement("option");
      opt.value = pid;
      opt.textContent = s.get(pid).label;
      if (pid === id) opt.selected = true;
      select.append(opt);
    }
    select.addEventListener("change", () => {
      this.input.setProfile(select.value);
      this.onChange?.();
      this.render();
    });
    head.append(select);
    this.root.append(head);

    const note = el("p", "ctl-note", PROFILE_NOTES[id] || "");
    this.root.append(note);

    if (profile.kind === "wheel") this.root.append(this.wheelMapping(id, profile));
    if (profile.kind === "wheel") this.root.append(this.ffbBlock(id, profile));
    if (profile.kind === "keyboard") this.root.append(this.mouseToggle(id, profile));

    for (const group of editableSettings(profile)) {
      const box = el("div", "ctl-group");
      box.append(el("h4", null, group.group));
      for (const item of group.items) box.append(this.row(id, item));
      this.root.append(box);
    }

    if (profile.kind === "wheel" || profile.kind === "gamepad") {
      this.root.append(this.calibrationBlock(id, profile));
    }

    const reset = document.createElement("button");
    reset.className = "secondary ctl-reset";
    reset.textContent = "Reset this profile";
    reset.addEventListener("click", () => {
      s.resetProfile(id);
      this.input.refreshProfile();
      this.onChange?.();
      this.render();
    });
    this.root.append(reset);
  }

  /** One labelled slider plus number box, with a per-row reset. */
  row(id, item) {
    const s = this.settings();
    const wrap = el("div", "ctl-row");
    const label = el("label", null, item.label);
    const value = el("span", "ctl-value");

    const slider = document.createElement("input");
    slider.type = "range";
    slider.min = item.min;
    slider.max = item.max;
    slider.step = item.step;

    const number = document.createElement("input");
    number.type = "number";
    number.min = item.min;
    number.max = item.max;
    number.step = item.step;

    const show = () => {
      const v = s.read(id, item.path);
      slider.value = v;
      number.value = v;
      value.textContent = `${v}${item.unit || ""}`;
      wrap.classList.toggle("changed", s.isOverridden(id, item.path));
    };

    const commit = (raw) => {
      let v = Number(raw);
      if (!Number.isFinite(v)) return;
      v = Math.min(Math.max(v, item.min), item.max);
      s.set(id, item.path, v);
      this.input.refreshProfile();
      this.onChange?.();
      show();
    };

    slider.addEventListener("input", () => commit(slider.value));
    number.addEventListener("change", () => commit(number.value));

    wrap.addEventListener("dblclick", () => {
      s.resetPath(id, item.path);
      this.input.refreshProfile();
      this.onChange?.();
      show();
    });

    wrap.append(label, slider, number, value);
    if (item.note) wrap.append(el("small", "ctl-hint", item.note));
    show();
    return wrap;
  }

  /**
   * The single most consequential wheel setting, so it gets its own control
   * rather than being buried in a slider list.
   */
  wheelMapping(id, profile) {
    const s = this.settings();
    const box = el("div", "ctl-group");
    box.append(el("h4", null, "Rim to road wheel"));

    const lock = this.input.carLockDeg ?? 28;
    const ratio = this.input.carSteeringRatio ?? 4;
    const carRim = (lock * ratio).toFixed(0);

    const sel = document.createElement("select");
    for (const [v, text] of [
      ["match-car", `Match the car (${carRim}° rim, lock to lock)`],
      ["scale-to-lock", "Scale my wheel's rotation to full lock"],
    ]) {
      const o = document.createElement("option");
      o.value = v;
      o.textContent = text;
      if (profile.wheel.mapping === v) o.selected = true;
      sel.append(o);
    }
    sel.addEventListener("change", () => {
      s.set(id, "wheel.mapping", sel.value);
      this.input.refreshProfile();
      this.onChange?.();
      this.render();
    });
    box.append(sel);

    const explain =
      profile.wheel.mapping === "match-car"
        ? `Set your wheel's driver software to ${carRim}° rotation. Then a given ` +
          `hand position is a given front wheel angle, and what you feel is what ` +
          `the tyres are doing. Leaving a 900° wheel at 900 makes this mapping ` +
          `use only the first ${((100 * carRim) / 900).toFixed(0)}% of its travel — ` +
          `which is correct, and feels wrong, because the wheel is set up wrong.`
        : `Your wheel's full ${profile.wheel.rotationDeg}° becomes the car's ` +
          `${lock}° of lock. Nothing to reconfigure, but the steering is then ` +
          `${(profile.wheel.rotationDeg / carRim).toFixed(1)}x slower than the real ` +
          `car's and the ratio is a fiction.`;
    box.append(el("small", "ctl-hint", explain));

    const soft = checkbox("Soft lock beyond full steering", profile.wheel.softLock, (on) => {
      s.set(id, "wheel.softLock", on);
      this.input.refreshProfile();
      this.onChange?.();
    });
    box.append(soft);
    return box;
  }

  /**
   * Force feedback: the switch, whether the shell can actually drive a wheel,
   * and a live torque bar so a driver can see the model's signal before
   * trusting their hands to it.
   */
  ffbBlock(id, profile) {
    const s = this.settings();
    const box = el("div", "ctl-group");
    box.append(el("h4", null, "Force feedback"));
    box.append(
      checkbox("Drive the wheel from the tyre model", profile.forceFeedback.enabled, (on) => {
        s.set(id, "forceFeedback.enabled", on);
        this.input.refreshProfile();
        this.onChange?.();
      }),
    );
    box.append(
      checkbox("Invert direction", profile.forceFeedback.invert, (on) => {
        s.set(id, "forceFeedback.invert", on);
        this.input.refreshProfile();
        this.onChange?.();
      }),
    );

    const st = this.game?.rigState;
    let line;
    if (!st || !st.ffbSupported) {
      line = "Not available here: force feedback needs the Windows desktop build (DirectInput). " +
        "In a browser the torque is still computed and shown below.";
    } else if (st.wheelPresent) {
      line = `Driving ${st.wheelName || "the wheel"} natively at 1 kHz.`;
    } else {
      line = `No force feedback wheel: ${st.wheelError || "none found"}. Restart the game with the base on.`;
    }
    box.append(el("small", "ctl-hint", line));
    const stats = this.game?.car?.stats;
    if (stats && stats.ticks > 0) {
      box.append(el("small", "ctl-hint",
        `Rig: ${stats.rateHz.toFixed(0)} Hz, ${stats.tickUsAvg.toFixed(0)} us/tick avg, ` +
        `${stats.tickUsMax.toFixed(0)} us max, ${stats.overruns} overruns.`));
    }

    // Live torque, as a centred bar. Right of centre pulls the rim clockwise.
    const meter = el("div", "ctl-ffb-meter");
    const fill = el("div", "ctl-ffb-fill");
    meter.append(fill);
    const read = el("pre", "ctl-axes", "");
    box.append(meter, read);

    const tick = () => {
      if (!this.root.isConnected) return;
      const last = this.game?.ffb?.last;
      if (last) {
        const c = last.command;
        fill.style.left = `${50 + Math.min(0, c) * 50}%`;
        fill.style.width = `${Math.abs(c) * 50}%`;
        fill.style.background = last.clipped ? "#e0552f" : "";
        read.textContent =
          `rim torque ${last.torqueNm >= 0 ? " " : ""}${last.torqueNm.toFixed(2)} N.m` +
          `   tyre ${last.align.toFixed(2)}   damping ${last.damping.toFixed(2)}` +
          `   stop ${last.softLock.toFixed(2)}   texture ${last.textureNm.toFixed(2)}` +
          (last.clipped ? "   CLIPPING" : "");
      }
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
    return box;
  }

  mouseToggle(id, profile) {
    const s = this.settings();
    const box = el("div", "ctl-group");
    box.append(el("h4", null, "Mouse steering"));
    box.append(
      checkbox("Steer with the mouse", profile.mouse.enabled, (on) => {
        s.set(id, "mouse.enabled", on);
        this.input.refreshProfile();
        this.onChange?.();
        this.render();
      }),
    );
    box.append(
      el(
        "small",
        "ctl-hint",
        "Off by default. Mouse steering is a different skill, not a better one.",
      ),
    );
    return box;
  }

  /**
   * Live axis readout plus a per-pedal calibration capture.
   *
   * Calibration is the difference between a pedal that reaches 100% and one
   * that stops at 60%: a G29 brake rests near -1 and tops out near +1, a load
   * cell may never reach +1 at any force a person can apply, and a gamepad
   * trigger reports 0..1 as a button.
   */
  calibrationBlock(id, profile) {
    const box = el("div", "ctl-group");
    box.append(el("h4", null, "Axes and calibration"));
    box.append(
      el(
        "small",
        "ctl-hint",
        "Press a pedal and watch which number moves — that is its axis. " +
          "Wheels do not use a standard mapping, so there is no table that is " +
          "right for every device.",
      ),
    );

    const monitor = el("pre", "ctl-axes", "(no device)");
    box.append(monitor);

    const tick = () => {
      if (!this.root.isConnected) return; // panel replaced; stop the loop
      const axes = this.input.rawAxes || [];
      monitor.textContent = axes.length
        ? axes.map((v, i) => `axis ${i}: ${v >= 0 ? " " : ""}${v.toFixed(3)}`).join("\n")
        : "(no device — connect one and press a button)";

      if (this.calibrating) {
        const { which, axis } = this.calibrating;
        const v = axes[axis] ?? 0;
        this.calibrating.min = Math.min(this.calibrating.min, v);
        this.calibrating.max = Math.max(this.calibrating.max, v);
        this.calibrating.el.textContent =
          `${which}: seen ${this.calibrating.min.toFixed(3)} … ` +
          `${this.calibrating.max.toFixed(3)}`;
      }
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);

    for (const which of ["throttle", "brake", "clutch"]) {
      const cal = profile.pedals?.[which];
      if (!cal || cal.source === null || !cal.isAxis) continue;

      const rowEl = el("div", "ctl-row ctl-cal");
      const status = el("span", "ctl-value", `axis ${cal.source}`);
      const btn = document.createElement("button");
      btn.className = "secondary";
      btn.textContent = `Calibrate ${which}`;

      btn.addEventListener("click", () => {
        if (this.calibrating && this.calibrating.which === which) {
          // Second click: store what we saw.
          const { min, max } = this.calibrating;
          if (Math.abs(max - min) < 0.2) {
            status.textContent = "not enough travel seen — try again";
          } else {
            // Rest is whichever end it sat at longest; assume the pedal starts
            // released, so the first value read is the rest position.
            const rest = this.calibrating.first;
            const restIsMin = Math.abs(rest - min) < Math.abs(rest - max);
            this.settings().calibratePedal(
              id,
              which,
              restIsMin ? min : max,
              restIsMin ? max : min,
            );
            this.input.refreshProfile();
            this.onChange?.();
            status.textContent = `calibrated: ${min.toFixed(3)} … ${max.toFixed(3)}`;
          }
          this.calibrating = null;
          btn.textContent = `Calibrate ${which}`;
        } else {
          const v = (this.input.rawAxes || [])[cal.source] ?? 0;
          this.calibrating = {
            which,
            axis: cal.source,
            min: v,
            max: v,
            first: v,
            el: status,
          };
          btn.textContent = "Press it fully, then click again";
        }
      });

      rowEl.append(el("label", null, which), btn, status);
      box.append(rowEl);
    }

    return box;
  }
}

function el(tag, cls, text) {
  const n = document.createElement(tag);
  if (cls) n.className = cls;
  if (text != null) n.textContent = text;
  return n;
}

function checkbox(label, checked, onChange) {
  const wrap = el("label", "ctl-check");
  const box = document.createElement("input");
  box.type = "checkbox";
  box.checked = !!checked;
  box.addEventListener("change", () => onChange(box.checked));
  wrap.append(box, document.createTextNode(" " + label));
  return wrap;
}
