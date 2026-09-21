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

import { applyPedal, editableSettings } from "./controlProfiles.js";
import { presetFor } from "./wheelPresets.js";
import {
  ACTIONS, ACTION_GROUPS, BindingCapture, buttonLabel, buttonSlot,
  defaultKeys, ensureEscapeHatch, keyLabel, UNBOUND,
} from "./controlBindings.js";

const PROFILE_NOTES = {
  keyboard:
    "A key is a step input, so the steering is ramped in software. These numbers " +
    "are the software standing in for a driver's hands.",
  "gamepad-xbox":
    "Full lock is 46 degrees and the tyre peaks at 7.3 degrees of slip, so a " +
    "linear stick puts the useful travel in the first sixth. The response curve " +
    "moves the resolution to where the grip is.",
  "gamepad-ps":
    "DualShock and DualSense report the same standard mapping as an Xbox pad; " +
    "the button names differ and the triggers are longer.",
  wheel:
    "Your hands are the input, so the software should get out of the way. " +
    "Deadzone 0 and response curve 1.00 are correct here -- anything else is " +
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
    /** "Press what you want it to be" -- one at a time, for every row. */
    this.capture = new BindingCapture(input);
    /** The live axis readout, when the active profile has one. */
    this.monitorEl = null;
    /**
     * A line to show after the next re-render.
     *
     * Binding something re-renders the panel, which throws away the element
     * the message would have gone in. Parking it here and reading it back at
     * the end of `render` is what makes "Upshift: E; it was Downshift" survive
     * the redraw that the binding itself caused.
     */
    this.pendingMsg = "";
    /**
     * Whether the panel is actually on screen.
     *
     * The live readouts below (the torque meter, the axis monitor) run on
     * requestAnimationFrame, and `#controls` lives inside the home menu, so
     * `root.isConnected` is true for the life of the page: both loops used
     * to run while DRIVING, writing style and text into a hidden tab every
     * frame and forcing a style-and-layout pass each time the renderer next
     * read `clientWidth` (~1100 forced layouts a second in a trace). An
     * IntersectionObserver reports the panel hidden (display:none through
     * the menu or the tab) or shown without touching layout ourselves; the
     * loops park while it is hidden and restart when it comes back.
     */
    this._visible = typeof IntersectionObserver === "undefined";
    this._loops = [];
    if (!this._visible) {
      this._io = new IntersectionObserver((entries) => {
        this._visible = entries[entries.length - 1].isIntersecting;
        if (this._visible) for (const loop of this._loops.slice()) this._kick(loop);
      });
      this._io.observe(root);
    }
    this.render();
  }

  /**
   * Run `fn` once per animation frame while the panel is visible and `gen`
   * is still the current render. Parks, rather than exits, when the panel
   * is hidden; the observer above wakes it.
   */
  startLoop(gen, fn) {
    const loop = { gen, fn, scheduled: false };
    this._loops.push(loop);
    this._kick(loop);
  }

  _kick(loop) {
    if (loop.scheduled) return;
    if (!this.root.isConnected || loop.gen !== this._gen) {
      const i = this._loops.indexOf(loop);
      if (i >= 0) this._loops.splice(i, 1);
      return;
    }
    if (!this._visible) return;
    loop.scheduled = true;
    requestAnimationFrame(() => {
      loop.scheduled = false;
      if (!this.root.isConnected || loop.gen !== this._gen) {
        const i = this._loops.indexOf(loop);
        if (i >= 0) this._loops.splice(i, 1);
        return;
      }
      loop.fn();
      this._kick(loop);
    });
  }

  settings() {
    return this.input.settings;
  }

  render() {
    const s = this.settings();
    const id = s.activeId;
    const profile = s.active();

    // Any capture in flight belongs to rows that are about to be thrown away.
    this.capture?.cancel();
    this.monitorEl = null;
    this.axisMsg = null;
    this.bindMsg = null;
    this.root.innerHTML = "";
    // Every live-readout loop below captures this; a re-render retires the
    // old ones instead of leaving them writing to detached nodes forever.
    this._gen = (this._gen || 0) + 1;
    const gen = this._gen;

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
      this.root.append(this.axisBlock(id, profile));
    }
    this.root.append(this.bindingBlock(id, profile));

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

    // One animation frame for the whole panel; see `startPump`.
    this.startPump(gen);
    if (this.pendingMsg && this.bindMsg) this.bindMsg.textContent = this.pendingMsg;
    this.pendingMsg = "";
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

    // The fallback is the car's real lock, so a path that somehow skips the
    // boot step reads like this car rather than like one two revisions ago.
    const lock = this.input.carLockDeg ?? 46;
    // The rack's measured stop at the rim, not lock x nominal ratio: the real
    // rack is progressive, so the nominal ratio puts the stop nowhere near the
    // right place (203 deg against a measured 179).
    const carRim = (this.input.carRimHalfDeg ?? 179).toFixed(0);

    const sel = document.createElement("select");
    for (const [v, text] of [
      ["match-car", `Match the car (${carRim} deg rim, lock to lock)`],
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
        ? `The rim turns the road wheels through the car's MEASURED rack, which ` +
          `is progressive: about 5.3 deg of rim per road degree on centre and ` +
          `3.4 past 90 deg. Set BOTH your wheel's driver software AND the ` +
          `rotation slider below to ${2 * carRim} deg (lock to lock) and your ` +
          `hand position is the front wheel angle, with the base's own stop at ` +
          `the car's. They must always agree: if the driver software says 900 ` +
          `and this says ${2 * carRim}, everything downstream is scaled wrong.`
        : `Your wheel's full ${profile.wheel.rotationDeg} deg becomes the car's ` +
          `${lock} deg of lock. Nothing to reconfigure, but the steering is then ` +
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
    // The re-render generation, captured here rather than read from
    // `render()`'s scope -- it is not in scope here, and the live loop below
    // threw a ReferenceError on its very first tick, so the torque meter has
    // never actually updated.
    const gen = this._gen;
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
    this.statusEl = el("small", "ctl-hint", this.rigStatusLine());
    box.append(this.statusEl);
    if (st?.wheelPresent && st.deviceNames?.length > 1) {
      box.append(el("small", "ctl-hint", `Also reading: ${st.deviceNames.slice(1).join(", ")} (axes 8 and up, buttons 32 and up).`));
    }

    // Which base, when there is a choice.
    if (st?.available?.length) {
      const pick = document.createElement("select");
      const auto = document.createElement("option");
      auto.value = "";
      auto.textContent = "Steer with: first force feedback wheel found";
      pick.append(auto);
      for (const d of st.available) {
        const o = document.createElement("option");
        o.value = d.name;
        o.textContent = `Steer with: ${d.name}${d.forceFeedback ? "" : " (no force feedback)"}`;
        if (profile.wheel.deviceName === d.name) o.selected = true;
        pick.append(o);
      }
      pick.addEventListener("change", () => {
        s.set(id, "wheel.deviceName", pick.value);
        this.input.refreshProfile();
        this.onChange?.();
      });
      box.append(pick);
    }

    // What the preset did, and what it could not know.
    const base = st?.wheelName || this.input.padName;
    if (profile.kind === "wheel" && base) {
      const preset = presetFor(base);
      box.append(el("small", "ctl-hint",
        `${preset.label}: rated ${preset.ratedNm} N.m, ${preset.rotationDeg} deg. ${preset.note}` +
        (preset.verify ? " Pedal axes are a starting guess: press each pedal and watch the monitor." : "")));
    }
    box.append(el("small", "ctl-hint",
      "Direction check: at speed, in a corner, ease your grip. The rim should pull back toward centre. " +
      "If it pulls further into the corner, tick Invert."));
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

    this.startLoop(gen, () => {
      const last = this.game?.ffb?.last;
      if (last) {
        const c = last.command;
        fill.style.left = `${50 + Math.min(0, c) * 50}%`;
        fill.style.width = `${Math.abs(c) * 50}%`;
        fill.style.background = last.clipped ? "#e0552f" : "";
        read.textContent =
          `rim torque ${last.torqueNm >= 0 ? " " : ""}${last.torqueNm.toFixed(2)} N.m` +
          `   tyre ${last.align.toFixed(2)}   damping ${last.damping.toFixed(2)}` +
          ((last.oversteer ?? 0) !== 0 ? `   oversteer ${last.oversteer.toFixed(2)}` : "") +
          `   stop ${last.softLock.toFixed(2)}   texture ${last.textureNm.toFixed(2)}` +
          (last.clipped ? "   CLIPPING" : "");
      }
    });
    return box;
  }

  /** The one line that says what the rig is doing with the wheel. */
  rigStatusLine() {
    const st = this.game?.rigState;
    if (!st || !st.ffbSupported) {
      return "Not available here: force feedback needs the Windows desktop build (DirectInput). " +
        "In a browser the torque is still computed and shown below.";
    }
    if (st.wheelPresent && st.ffbActive) return `Driving ${st.wheelName || "the wheel"} natively at 1 kHz.`;
    if (st.wheelPresent) return `Reading ${st.wheelName} for steering and pedals, but not driving it: ${st.wheelError}`;
    return `No wheel: ${st.wheelError || "none found"}. Plug the base in, put it in PC mode, and pick it below.`;
  }

  /**
   * Refresh the rig status without rebuilding the panel. A full render() in
   * the middle of a slider drag replaces the slider under the pointer and the
   * drag dies, so the per-change path must only touch this line.
   */
  updateStatus() {
    if (this.statusEl?.isConnected) this.statusEl.textContent = this.rigStatusLine();
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
  /**
   * Axes: which one is the steering, which one each pedal is, and how far
   * each pedal travels.
   *
   * One gesture does both jobs. Sweeping a pedal end to end says WHICH axis it
   * is -- the one that moved -- and at the same time measures where it rests
   * and where it bottoms out, which is the calibration. Wheels and pedal sets
   * do not use a standard mapping, so there is no table that is right for
   * every device and no way to know but to look.
   */
  axisBlock(id, profile) {
    const s = this.settings();
    const box = el("div", "ctl-group");
    box.append(el("h4", null, "Axes and calibration"));
    box.append(
      el(
        "small",
        "ctl-hint",
        "Click a row, then move the thing it is for: the axis that moves is " +
          "the one it binds to. Sweeping a pedal all the way and letting it go " +
          "also calibrates its travel -- so take your feet off the others " +
          "first, or the one you are resting on is read as its own rest " +
          "position. Esc stops listening.",
      ),
    );

    // Which device is which, when there is more than one.
    //
    // The rig reads a wheel base plus anything else plugged in, and gives each
    // device its own block of eight axes -- the base is 0-7, the next thing is
    // 8-15, and so on. Fanatec, Simucube, Heusinkveld and most load-cell sets
    // put the pedals on their OWN USB, so on those rigs the pedals are not on
    // the base's axes at all. A preset that guessed base axes then reads a
    // pedal that never moves, which looks exactly like "the pedals do not
    // work" and gives the driver nothing to go on.
    const devices = this.input.nativeDeviceNames ?? [];
    if (devices.length > 1) {
      const lines = devices.map((n, i) => `  axes ${i * 8}-${i * 8 + 7}   ${n}`);
      box.append(el("small", "ctl-hint",
        `The rig is reading ${devices.length} devices, eight axes each:`));
      box.append(el("pre", "ctl-axes", lines.join("\n")));

      const onBase = ["throttle", "brake", "clutch"].filter((w) => {
        const cal = profile.pedals?.[w];
        return cal && cal.isAxis && typeof cal.source === "number" && cal.source < 8;
      });
      if (onBase.length) {
        box.append(el("small", "ctl-hint ctl-warn",
          `${onBase.join(", ")} ${onBase.length === 1 ? "is" : "are"} set to an axis on ` +
          `the base (0-7). If your pedals are the separate device above, that is why ` +
          `they do nothing -- click Re-detect and sweep each one.`));
      }
    }

    const monitor = el("pre", "ctl-axes", "(no device)");
    this.monitorEl = monitor;
    box.append(monitor);

    const msg = el("p", "bind-msg axis-msg", "");
    this.axisMsg = msg;

    // ---- steering ----
    {
      const row = el("div", "ctl-row ctl-cal");
      const status = el("span", "ctl-value", `axis ${profile.axes?.steer ?? 0}`);
      const btn = document.createElement("button");
      btn.className = "secondary";
      btn.textContent = "Detect";
      btn.addEventListener("click", () => {
        this.listen(btn, "axis", "Turn the wheel lock to lock", (r) => {
          if (!r) return;
          if (r.kind !== "axis") {
            msg.textContent = "That is a button, not an axis. Steering needs an axis.";
            return;
          }
          s.set(id, "axes.steer", r.index);
          this.input.refreshProfile();
          this.onChange?.();
          status.textContent = `axis ${r.index}`;
          msg.textContent = `Steering is axis ${r.index}.`;
        });
      });
      row.append(el("label", null, "Steering"), btn, status);
      box.append(row);
    }

    // ---- pedals ----
    for (const which of ["throttle", "brake", "clutch"]) {
      const cal = profile.pedals?.[which];
      if (!cal) continue;

      const row = el("div", "ctl-row ctl-cal");
      const status = el("span", "ctl-value", pedalSummary(cal));
      const btn = document.createElement("button");
      btn.className = "secondary";
      btn.textContent = cal.source == null ? "Assign" : "Re-detect";

      btn.addEventListener("click", () => {
        this.listen(btn, "axis", `Press ${which} all the way, then let go`, (r) => {
          if (!r) return;
          // Rest is where it was sitting when the sweep started; full travel
          // is whichever end it went to. A G29 brake rests at +1 and goes to
          // -1, an SR-P rests at -1 -- assuming either is how a pedal ends up
          // with a third of its travel.
          const restIsMin = Math.abs(r.first - r.min) < Math.abs(r.first - r.max);
          const rawMin = restIsMin ? r.min : r.max;
          const rawMax = restIsMin ? r.max : r.min;
          s.set(id, `pedals.${which}.source`, r.index);
          s.set(id, `pedals.${which}.isAxis`, r.kind === "axis");
          s.set(id, `pedals.${which}.rawMin`, rawMin);
          s.set(id, `pedals.${which}.rawMax`, rawMax);
          this.input.refreshProfile();
          this.onChange?.();
          const now = this.settings().get(id).pedals[which];
          status.textContent = pedalSummary(now);
          btn.textContent = "Re-detect";
          msg.textContent =
            `${which}: ${r.kind === "axis" ? "axis" : "trigger"} ${r.index}, ` +
            `${rawMin.toFixed(3)} at rest to ${rawMax.toFixed(3)} flat out.`;
        });
      });

      const clear = document.createElement("button");
      clear.className = "secondary";
      clear.textContent = "Default";
      clear.addEventListener("click", () => {
        for (const f of ["source", "isAxis", "rawMin", "rawMax"]) {
          s.resetPath(id, `pedals.${which}.${f}`);
        }
        this.input.refreshProfile();
        this.onChange?.();
        this.render();
      });

      row.append(el("label", null, which), btn, status, clear);
      box.append(row);
    }

    box.append(msg);
    return box;
  }

  /**
   * Every control, and what it is bound to.
   *
   * Two columns because there are two devices in the room at once: the
   * keyboard is always live, whatever is plugged in, so a wheel driver still
   * needs to know what Escape does. Click a cell and press the thing you want
   * it to be -- a key, a button, a paddle. The same gesture every racing game
   * uses, because a list of key codes is not something anyone can check
   * against the wheel in front of them.
   */
  bindingBlock(id, profile) {
    const s = this.settings();
    const hasDevice = profile.kind === "wheel" || profile.kind === "gamepad";
    const box = el("div", "ctl-group");
    box.append(el("h4", null, "Bindings"));

    const msg = el("p", "bind-msg", "");
    this.bindMsg = msg;

    box.append(el("small", "ctl-hint",
      "Click a cell, then press what you want it to be. Right-click clears it. " +
      "Binding something already taken moves it, and says so. Escape stops the " +
      "listening rather than binding, so it stays the pause key."));

    const legend = el("div", "bind-legend");
    legend.append(el("span", null, "Control"), el("span", "c", "Key"),
      el("span", "c", hasDevice ? "Device" : ""));
    box.append(legend);

    for (const groupName of ACTION_GROUPS) {
      const actions = ACTIONS.filter((a) => a.group === groupName);
      if (!actions.length) continue;
      box.append(el("small", "ctl-hint", groupName));

      for (const action of actions) {
        box.append(this.bindingRow(id, profile, action, hasDevice));
      }
    }

    // ---- what to do when it has all gone wrong ----
    const actions = el("div", "bind-actions");
    const resetKeys = document.createElement("button");
    resetKeys.className = "secondary";
    resetKeys.textContent = "Reset every key";
    resetKeys.addEventListener("click", () => {
      s.set(id, "keys", defaultKeys());
      this.commit(id, "Every key is back to the shipped binding.");
    });
    actions.append(resetKeys);
    if (hasDevice) {
      const resetButtons = document.createElement("button");
      resetButtons.className = "secondary";
      resetButtons.textContent = "Reset every button";
      resetButtons.addEventListener("click", () => {
        s.resetPath(id, "buttons");
        this.commit(id, "Every button is back to the shipped binding.");
      });
      actions.append(resetButtons);
    }
    box.append(actions, msg);
    return box;
  }

  /** One control: its name, its key, and its button. */
  bindingRow(id, profile, action, hasDevice) {
    const s = this.settings();
    const row = el("div", "bind-row");

    const what = el("span", "bind-what", action.label);
    if (action.hold) what.append(el("span", "bind-hold", "hold"));
    row.append(what);

    // ---- the key ----
    const codes = profile.keys?.[action.id] ?? [];
    const keyBtn = document.createElement("button");
    keyBtn.type = "button";
    keyBtn.className = "bind-key" + (codes.length ? "" : " unset") +
      (s.isOverridden(id, `keys.${action.id}`) ? " changed" : "");
    keyBtn.textContent = codes.length ? codes.map(keyLabel).join(" / ") : "--";
    keyBtn.title = codes.length
      ? `${codes.join(", ")}  --  click to rebind, right-click to clear`
      : "Click to bind a key";
    keyBtn.addEventListener("click", () => {
      this.listen(keyBtn, "key", null, (r) => {
        if (!r) return;
        const taken = this.claimKey(id, profile, action.id, r.code);
        this.commit(id, taken
          ? `${keyLabel(r.code)} is now ${action.label}; it was ${taken}.`
          : `${action.label}: ${keyLabel(r.code)}.`);
      });
    });
    const clearKey = () => {
      s.set(id, `keys.${action.id}`, []);
      this.commit(id, `${action.label} has no key.`);
    };
    keyBtn.addEventListener("contextmenu", (e) => { e.preventDefault(); clearKey(); });
    // Right-click is not reachable from a keyboard, and neither is a mouse on
    // a rig where the only thing within reach is the wheel.
    keyBtn.addEventListener("keydown", (e) => {
      if (e.code !== "Delete" && e.code !== "Backspace") return;
      e.preventDefault();
      clearKey();
    });
    row.append(keyBtn);

    // ---- the device button ----
    if (!hasDevice) return row;
    const devBtn = document.createElement("button");
    devBtn.type = "button";
    devBtn.className = "bind-key";
    if (action.keysOnly) {
      // Either a keyboard-only action, or an axis -- see `axisBlock`.
      devBtn.disabled = true;
      devBtn.textContent = action.id === "mapEditor" ? "--" : "axis";
      devBtn.title = action.id === "mapEditor"
        ? "Keyboard only"
        : "On a device this is an axis; set it under Axes and calibration.";
      row.append(devBtn);
      return row;
    }
    const slot = buttonSlot(action);
    const index = profile.buttons?.[slot];
    const bound = typeof index === "number" && index >= 0;
    devBtn.textContent = bound ? buttonLabel(index, profile.labels, slot) : "--";
    devBtn.className += bound ? "" : " unset";
    if (s.isOverridden(id, `buttons.${slot}`)) devBtn.className += " changed";
    devBtn.title = bound
      ? `Button ${index}  --  click to rebind, right-click to clear`
      : "Click, then press the button you want";
    devBtn.addEventListener("click", () => {
      this.listen(devBtn, "button", null, (r) => {
        if (!r) return;
        const taken = this.claimButton(id, profile, slot, r.index);
        this.commit(id, taken
          ? `${buttonLabel(r.index)} is now ${action.label}; it was ${taken}.`
          : `${action.label}: ${buttonLabel(r.index)}.`);
      });
    });
    const clearButton = () => {
      s.set(id, `buttons.${slot}`, UNBOUND);
      this.commit(id, `${action.label} has no button.`);
    };
    devBtn.addEventListener("contextmenu", (e) => { e.preventDefault(); clearButton(); });
    devBtn.addEventListener("keydown", (e) => {
      if (e.code !== "Delete" && e.code !== "Backspace") return;
      e.preventDefault();
      clearButton();
    });
    row.append(devBtn);
    return row;
  }

  /**
   * Apply a binding change: write it, re-read it, say what happened, redraw.
   *
   * One funnel for all four of them (bind a key, clear a key, bind a button,
   * clear a button) so the escape-hatch guard cannot be forgotten at one call
   * site, and so the message always survives the redraw the change causes.
   */
  commit(id, note) {
    const putBack = ensureEscapeHatch(this.settings(), id);
    this.input.refreshProfile();
    this.onChange?.();
    this.pendingMsg = putBack
      ? `${note}  Pause had nothing left, so Esc is back -- something has to stop the game.`
      : note;
    this.render();
  }

  /**
   * Give a key to an action, taking it off whatever had it.
   *
   * Two actions on one key is not a preference, it is a bug you find at speed
   * -- so the new binding wins and the old one is told. Returns the label of
   * the action that lost it, or null.
   */
  claimKey(id, profile, actionId, code) {
    const s = this.settings();
    const taken = [];
    for (const a of ACTIONS) {
      if (a.id === actionId) continue;
      const codes = profile.keys?.[a.id] ?? [];
      if (!codes.includes(code)) continue;
      taken.push(a.label);
      s.set(id, `keys.${a.id}`, codes.filter((c) => c !== code));
    }
    s.set(id, `keys.${actionId}`, [code]);
    // All of them: a key held by two actions is a mess the driver made and
    // naming only the last one leaves them looking for the other.
    return taken.length ? taken.join(" and ") : null;
  }

  /** The same, for a button on the device. */
  claimButton(id, profile, slot, index) {
    const s = this.settings();
    const taken = [];
    for (const a of ACTIONS) {
      const other = buttonSlot(a);
      if (other === slot || a.keysOnly) continue;
      if (profile.buttons?.[other] !== index) continue;
      taken.push(a.label);
      s.set(id, `buttons.${other}`, UNBOUND);
    }
    s.set(id, `buttons.${slot}`, index);
    return taken.length ? taken.join(" and ") : null;
  }

  /**
   * Put one control into "press what you want it to be".
   *
   * The button says so itself rather than a modal saying it somewhere else:
   * the thing you clicked is the thing that is listening, which is the only
   * arrangement that survives twenty rows.
   */
  listen(btnEl, mode, prompt, onResult) {
    // Stop whatever was listening FIRST, and before reading this cell's text.
    // `capture.start` cancels the previous capture, whose callback restores
    // the cell it belonged to -- so clicking the same cell twice (the natural
    // "did that register?" gesture) read "press a key" as the text to put
    // back, then had the first callback overwrite it with the real binding.
    // The row was left listening with nothing on screen saying so, and the
    // next key pressed went into it.
    this.capture.cancel();
    const was = btnEl.textContent;
    const wasClass = btnEl.className;
    btnEl.className = wasClass.replace(" unset", "") + " listening";
    btnEl.textContent = prompt ?? "";
    if (this.axisMsg && mode === "axis") this.axisMsg.textContent = "";
    this.capture.start(mode, (result) => {
      // The row may already be gone: a result triggers a re-render.
      if (btnEl.isConnected) {
        btnEl.className = wasClass;
        btnEl.textContent = was;
      }
      onResult(result);
    });
    // After `start`, so the capture knows which mode to describe. A caller's
    // own prompt wins; otherwise the cell says what to do and how to stop.
    btnEl.textContent = prompt ?? this.capture.prompt();
  }

  /** Each assigned pedal as a bar, 0 to 100%, through its own calibration. */
  calibratedPedals(axes) {
    const prof = this.input.profile;
    const out = [];
    for (const which of ["throttle", "brake", "clutch"]) {
      const cal = prof.pedals?.[which];
      if (!cal || cal.source == null || !cal.isAxis) continue;
      const v = applyPedal(cal, axes[cal.source] ?? 0);
      const filled = Math.round(Math.max(0, Math.min(1, v)) * 20);
      const bar = "#".repeat(filled) + ".".repeat(20 - filled);
      out.push(`${which.padEnd(9)}${bar} ${(v * 100).toFixed(0).padStart(3)}%`);
    }
    return out;
  }

  /**
   * The per-frame pump: the live axis readout, any calibration in flight, and
   * the binding capture.
   *
   * One loop for all of those rather than one each, and it retires itself
   * when the panel is re-rendered -- the old arrangement left detached loops
   * running forever, writing into nodes nobody could see. (The force-feedback
   * torque meter keeps its own loop: it only exists on a wheel profile and it
   * reads from the rig rather than from this panel.)
   */
  startPump(gen) {
    this.startLoop(gen, () => {
      this.capture?.tick();

      const monitor = this.monitorEl;
      if (monitor && monitor.isConnected) {
        const axes = this.input.rawAxes || [];
        if (!axes.length) {
          monitor.textContent = "(no device -- connect one and press a button)";
        } else {
          const rows = axes.map((v, i) => `axis ${i}: ${v >= 0 ? " " : ""}${v.toFixed(3)}`);
          // The pedals as the CAR sees them, under the raw numbers.
          //
          // The raw floats say which axis moved; they do not say whether the
          // calibration reaches 100%, which is the thing a driver actually
          // needs to know and cannot otherwise find out without going and
          // driving. Same maths the game uses.
          const pedals = this.calibratedPedals(axes);
          if (pedals.length) rows.push("", ...pedals);
          monitor.textContent = rows.join("\n");
        }
      }
    });
  }
}

/** A pedal's binding and travel, in one line. */
function pedalSummary(cal) {
  if (!cal || cal.source == null) return "not assigned";
  const where = cal.isAxis ? `axis ${cal.source}` : `trigger ${cal.source}`;
  return `${where}  ${Number(cal.rawMin).toFixed(2)} to ${Number(cal.rawMax).toFixed(2)}`;
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
