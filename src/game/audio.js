// Engine, tyre and wind audio.
//
// The engine note comes from `src/audio/engineAudio.js`: a physical model of
// combustion and exhaust gas dynamics, adapted from ange-yaghi/engine-sim, run
// on the audio thread in an AudioWorklet. Everything else here -- tyre scrub,
// wind, cone strikes -- stays as plain WebAudio nodes, because none of it is
// an engine and none of it benefits from being modelled.
//
// The oscillator bank below is kept as a fallback, and it is not dead code:
// AudioWorklet needs a secure context and a browser that supports it, and if
// `addModule` fails there is no sound at all otherwise. It builds the note from
// the firing frequency (a four-stroke four fires twice per crank revolution, so
// f_fire = rpm / 30) with harmonics stacked on top -- recognisably the same
// engine, obviously synthetic next to the real thing.

const MIX_KEY = "fsae-sim.audio.v1";

/**
 * Per-source levels, 0..1.
 *
 * Separate rather than one master because the sources are not interchangeable:
 * the engine is continuous and sets the mood, tyre scrub is information about
 * grip, and a cone strike is a discrete event telling you that you have just
 * been given a two-second penalty. Which of those you want louder depends on
 * whether you are learning the car or chasing a time.
 */
export const DEFAULT_MIX = {
  master: 0.5,
  engine: 0.9,
  tyres: 1.0,
  wind: 1.0,
  cones: 1.0,
};

export const MIX_LABELS = {
  master: "Master",
  engine: "Engine",
  tyres: "Tyres",
  wind: "Wind",
  cones: "Cone strikes",
};

export class EngineAudio {
  constructor() {
    this.ctx = null;
    this.ready = false;
    this.enabled = true;
    this.mix = { ...DEFAULT_MIX };
    this.loadMix();
    /** True once the physical model is running on the audio thread. */
    this.usingModel = false;
    this.modelNode = null;
    this.modelError = null;
  }

  /** Must be called from a user gesture -- browsers block audio otherwise. */
  start() {
    if (this.ctx) {
      if (this.ctx.state === "suspended") this.ctx.resume();
      return;
    }
    const Ctx = window.AudioContext || window.webkitAudioContext;
    if (!Ctx) return;
    const ctx = new Ctx();
    this.ctx = ctx;

    this.master = ctx.createGain();
    this.master.gain.value = this.mix.master;
    this.master.connect(ctx.destination);

    // ---- engine: three harmonics through a throttle-controlled lowpass ----
    this.engineGain = ctx.createGain();
    this.engineGain.gain.value = 0;
    this.engineFilter = ctx.createBiquadFilter();
    this.engineFilter.type = "lowpass";
    this.engineFilter.frequency.value = 900;
    this.engineFilter.Q.value = 1.1;
    this.engineGain.connect(this.engineFilter);
    this.engineFilter.connect(this.master);

    this.oscs = [];
    const harmonics = [
      { mult: 0.5, type: "sawtooth", gain: 0.30 },  // half order, gives it weight
      { mult: 1.0, type: "sawtooth", gain: 0.55 },  // firing frequency
      { mult: 2.0, type: "square", gain: 0.22 },
      { mult: 3.0, type: "sawtooth", gain: 0.12 },
    ];
    for (const h of harmonics) {
      const osc = ctx.createOscillator();
      osc.type = h.type;
      const g = ctx.createGain();
      g.gain.value = h.gain;
      osc.connect(g);
      g.connect(this.engineGain);
      osc.start();
      this.oscs.push({ osc, mult: h.mult });
    }

    // ---- shared noise source for induction, tyres and wind ----
    const noiseBuf = ctx.createBuffer(1, ctx.sampleRate * 2, ctx.sampleRate);
    const data = noiseBuf.getChannelData(0);
    for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1;
    this.noise = ctx.createBufferSource();
    this.noise.buffer = noiseBuf;
    this.noise.loop = true;
    this.noise.start();

    const branch = (type, freq, q, gain) => {
      const f = ctx.createBiquadFilter();
      f.type = type; f.frequency.value = freq; f.Q.value = q;
      const g = ctx.createGain();
      g.gain.value = gain;
      this.noise.connect(f); f.connect(g); g.connect(this.master);
      return { f, g };
    };
    this.induction = branch("bandpass", 700, 0.8, 0);
    this.squeal = branch("bandpass", 1350, 7.0, 0);
    this.wind = branch("lowpass", 520, 0.7, 0);

    this.ready = true;

    // Bring up the physical model asynchronously. `start()` is called from a
    // user gesture and must stay synchronous, so the fallback plays until the
    // worklet is live -- typically a frame or two, and inaudible.
    this.startModel();
  }

  async startModel() {
    const ctx = this.ctx;
    if (!ctx || !ctx.audioWorklet) {
      this.modelError = "AudioWorklet unavailable";
      return;
    }
    try {
      await ctx.audioWorklet.addModule("./src/audio/engineWorklet.js");
      const node = new AudioWorkletNode(ctx, "engine-processor", {
        numberOfInputs: 0,
        numberOfOutputs: 1,
        outputChannelCount: [1],
        processorOptions: { engine: "cbr600rr-sdm26" },
      });
      this.modelGain = ctx.createGain();
      this.modelGain.gain.value = this.mix.engine;
      node.connect(this.modelGain);
      this.modelGain.connect(this.master);
      this.modelNode = node;
      this.usingModel = true;

      // Silence the fallback rather than tearing it down: if the worklet ever
      // stops, the oscillators are still wired up and can be faded back in.
      this.engineGain.gain.setTargetAtTime(0, ctx.currentTime, 0.02);
    } catch (err) {
      this.modelError = String(err && err.message ? err.message : err);
      console.warn("engine model unavailable, using the oscillator fallback:", this.modelError);
    }
  }

  /** What the engine note is currently being produced by. For the HUD. */
  engineSource() {
    if (!this.ready) return "off";
    return this.usingModel ? "physical model" : "oscillator fallback";
  }

  setEnabled(on) {
    this.enabled = on;
    if (this.master) this.master.gain.value = on ? this.mix.master : 0;
  }

  /** Set one source level, 0..1, and persist it. */
  setVolume(name, value) {
    if (!(name in this.mix)) return;
    this.mix[name] = Math.min(Math.max(Number(value) || 0, 0), 1);
    this.applyMix();
    this.saveMix();
  }

  resetMix() {
    this.mix = { ...DEFAULT_MIX };
    this.applyMix();
    this.saveMix();
  }

  /**
   * Push the mix onto the audio graph.
   *
   * Only master and engine have a node that holds a steady value; the tyre,
   * wind and cone levels are applied where those sounds are generated, because
   * their gains are already being driven per frame by the physics and a static
   * node would just be overwritten.
   */
  applyMix() {
    if (!this.ready) return;
    this.master.gain.value = this.enabled ? this.mix.master : 0;
    if (this.modelGain) this.modelGain.gain.value = this.mix.engine;
  }

  saveMix() {
    try {
      localStorage.setItem(MIX_KEY, JSON.stringify(this.mix));
    } catch {
      // Storage unavailable; the mix just does not persist.
    }
  }

  loadMix() {
    try {
      const raw = localStorage.getItem(MIX_KEY);
      if (!raw) return;
      const saved = JSON.parse(raw);
      for (const k of Object.keys(DEFAULT_MIX)) {
        if (typeof saved?.[k] === "number") {
          this.mix[k] = Math.min(Math.max(saved[k], 0), 1);
        }
      }
    } catch {
      // Corrupt storage should not stop audio starting.
    }
  }

  /**
   * @param {object} s
   *   rpm, throttle 0..1, torqueNm, speed m/s, slip 0..~2 (worst axle
   *   utilisation), wheelspin 0..1, shifting bool
   */
  update(s, dt) {
    if (!this.ready || !this.enabled) return;
    const ctx = this.ctx;
    const now = ctx.currentTime;
    const glide = Math.max(0.012, Math.min(0.05, dt * 2));

    const firing = Math.max(20, (s.rpm / 30));
    const rev = Math.min(1, s.rpm / 9000);

    if (this.usingModel) {
      // The model takes the torque the engine is actually making and solves
      // its heat release to match, so the note is generated from a cylinder
      // pressure trace that does the validated amount of work. During a shift
      // the ignition really is cut, and passing zero torque is what makes that
      // audible rather than a scripted effect.
      this.modelNode.port.postMessage({
        type: "operating-point",
        rpm: s.rpm,
        throttle: s.shifting ? 0 : s.throttle,
        torqueNm: s.shifting ? 0 : (s.torqueNm ?? 0),
      });
    } else {
      for (const { osc, mult } of this.oscs) {
        osc.frequency.setTargetAtTime(firing * mult, now, glide);
      }

      // Louder and brighter on throttle; the ignition cut during a shift is
      // audible because the physics really does cut it.
      const load = s.shifting ? 0.12 : 0.30 + 0.70 * s.throttle;
      this.engineGain.gain.setTargetAtTime(0.16 + 0.30 * load * (0.45 + 0.55 * rev), now, glide);
      this.engineFilter.frequency.setTargetAtTime(
        600 + 5200 * load * (0.3 + 0.7 * rev), now, glide);
    }

    this.induction.g.gain.setTargetAtTime(0.05 * s.throttle * rev * this.mix.engine, now, glide);
    this.induction.f.frequency.setTargetAtTime(400 + firing * 1.6, now, glide);

    // Tyres only talk once they are near the limit; the pitch climbs with
    // how far past peak slip they are.
    const scrub = Math.max(0, Math.min(1, (s.slip - 0.82) / 0.5));
    const spin = Math.max(0, Math.min(1, s.wheelspin));
    const squealAmt = Math.max(scrub, spin * 0.8) * Math.min(1, s.speed / 3);
    this.squeal.g.gain.setTargetAtTime(0.20 * squealAmt * this.mix.tyres, now, glide);
    this.squeal.f.frequency.setTargetAtTime(1100 + 900 * squealAmt, now, glide);

    this.wind.g.gain.setTargetAtTime(
      Math.min(0.10, (s.speed / 32) ** 2 * 0.10) * this.mix.wind, now, glide);
    this.wind.f.frequency.setTargetAtTime(320 + s.speed * 22, now, glide);
  }

  /** One-shot clatter for a cone strike. */
  coneHit() {
    if (!this.ready || !this.enabled) return;
    const ctx = this.ctx;
    const now = ctx.currentTime;
    const osc = ctx.createOscillator();
    osc.type = "square";
    osc.frequency.setValueAtTime(220, now);
    osc.frequency.exponentialRampToValueAtTime(90, now + 0.14);
    const g = ctx.createGain();
    // exponentialRampToValueAtTime cannot start from or reach zero, so a muted
    // cone level has to skip the sound entirely rather than ramp to silence.
    const peak = 0.28 * this.mix.cones;
    if (peak < 1e-4) return;
    g.gain.setValueAtTime(peak, now);
    g.gain.exponentialRampToValueAtTime(0.001, now + 0.18);
    osc.connect(g); g.connect(this.master);
    osc.start(now);
    osc.stop(now + 0.2);
  }
}
