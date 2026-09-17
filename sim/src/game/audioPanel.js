// Audio levels on the home screen.
//
// Separate sliders per source rather than one master, because the sources are
// not interchangeable. The engine is continuous and sets the mood; tyre scrub
// is information about how much grip is left; a cone strike is a discrete event
// telling you that you have just taken a two-second penalty. Which of those you
// want louder depends on whether you are learning the car or chasing a time,
// and that is a decision for the driver rather than for a fixed mix.
//
// Levels persist, so a driver who turns the wind down keeps it down.

import { DEFAULT_MIX, MIX_LABELS } from "./audio.js";

const NOTES = {
  master: "Everything, including the fallback synth.",
  engine:
    "The physical exhaust model. Level already tracks how hard the engine is " +
    "working -- idle sits about 20 dB below the limiter on its own.",
  tyres: "Scrub and wheelspin. Turn it up to hear the limit approaching.",
  wind: "Rises with the square of speed.",
  cones: "The clatter when you knock one over. Worth +2 s, so worth hearing.",
};

export class AudioPanel {
  /**
   * @param root   container element
   * @param audio  the EngineAudio instance (the game's audio manager)
   */
  constructor(root, audio) {
    this.root = root;
    this.audio = audio;
    this.render();
  }

  render() {
    this.root.innerHTML = "";

    const head = document.createElement("div");
    head.className = "ctl-head";
    const h = document.createElement("h3");
    h.textContent = "Audio levels";
    head.append(h);
    this.root.append(head);

    const box = document.createElement("div");
    box.className = "ctl-group";

    for (const name of Object.keys(DEFAULT_MIX)) {
      box.append(this.row(name));
    }
    this.root.append(box);

    const reset = document.createElement("button");
    reset.className = "secondary ctl-reset";
    reset.textContent = "Reset levels";
    reset.addEventListener("click", () => {
      this.audio.resetMix();
      this.render();
    });
    this.root.append(reset);
  }

  row(name) {
    const wrap = document.createElement("div");
    wrap.className = "ctl-row";

    const label = document.createElement("label");
    label.textContent = MIX_LABELS[name] || name;

    const slider = document.createElement("input");
    slider.type = "range";
    slider.min = 0;
    slider.max = 1;
    slider.step = 0.01;

    const number = document.createElement("input");
    number.type = "number";
    number.min = 0;
    number.max = 100;
    number.step = 1;

    const value = document.createElement("span");
    value.className = "ctl-value";

    const show = () => {
      const v = this.audio.mix[name];
      slider.value = v;
      number.value = Math.round(v * 100);
      value.textContent = `${Math.round(v * 100)}%`;
      wrap.classList.toggle("changed", Math.abs(v - DEFAULT_MIX[name]) > 1e-6);
    };

    slider.addEventListener("input", () => {
      this.audio.setVolume(name, Number(slider.value));
      show();
    });
    number.addEventListener("change", () => {
      this.audio.setVolume(name, Number(number.value) / 100);
      show();
    });
    // Double-click a row to put it back to the shipped level, matching the
    // vehicle and control panels.
    wrap.addEventListener("dblclick", () => {
      this.audio.setVolume(name, DEFAULT_MIX[name]);
      show();
    });

    wrap.append(label, slider, number, value);
    if (NOTES[name]) {
      const note = document.createElement("small");
      note.className = "ctl-hint";
      note.textContent = NOTES[name];
      wrap.append(note);
    }
    show();
    return wrap;
  }
}
