// Engine, tyre, wind and drivetrain audio.
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

import { SDM26 } from "../vehicle/params.js";

const MIX_KEY = "fsae-sim.audio.v1";

/**
 * The drivetrain's teeth, for the mesh frequencies.
 *
 * `primary` is the crank gear to the clutch basket, 36 -> 76, which is the
 * 2.111 in params.js. `gears` are [driven, drive] per gear for a CBR600RR
 * box consistent with params.js's ratios exactly (33/12 = 2.750, 32/16,
 * 30/18, 26/18, 30/23, 29/24); Honda's own tooth counts are not in the repo,
 * so these are the plausible integer pairs, not a parts list. The sprockets
 * are not in params either: 14 front / 42 rear is the plausible pair for its
 * finalDrive of 3.0.
 *
 * `level` is each source's peak amplitude before the Engine slider (the
 * engine model runs at ~0.19 RMS by comparison): easy to trim, and
 * deliberately quiet. A sportbike cassette box is not a race car's
 * straight-cut box -- its whine is texture, not a feature -- and the chain,
 * exposed behind the driver, is the only part meant to be noticed.
 */
export const DRIVETRAIN = {
  primary: [76, 36],
  gears: [[33, 12], [32, 16], [30, 18], [26, 18], [30, 23], [29, 24]],
  frontSprocket: 14,
  rearSprocket: 42,
  level: { chain: 0.016, chainBuzz: 0.016, gear: 0.0025, primary: 0.0012, rattle: 0.010 },
};

/**
 * Tyre, wind and road levels at full effect, before the Tyres / Wind sliders.
 * No recording of these exists yet; they are set by ear to sit under the
 * engine (the limiter stays at 0-1 dB), and live here so a recording can
 * calibrate them.
 */
export const TYRE = {
  scrub: 0.11, squeal: 0.035, spin: 0.10, lock: 0.20, rolling: 0.030,
  wind: 0.18, windHiss: 0.03, road: 0.10,
};

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
  cues: 0.8,
};

/**
 * Fixed trim on the physical model's output, on top of the Engine slider.
 * At the default mix the model sat at -22 to -26 dBFS RMS after the master,
 * quiet against everything else, while the output limiter was doing nothing
 * to it (0-0.9 dB of gain reduction with the engine alone). +3 dB here, and
 * the cockpit gains another ~5 dB from the intake now being heard from the
 * driver's seat (engineAudio.js, intake.cockpitGain): together that is the
 * ~6 dB asked for in the cockpit, with the limiter still at 0-0.2 dB at
 * master 0.5 and about 1 dB at master 1.0 (measured, headless browser build).
 * +6 dB here put the cockpit into 3-7 dB of limiting at master 1.0.
 * It is 2 x 1.4: the model's output is now halved for headroom
 * (ENGINE_OUTPUT_HEADROOM in engineAudio.js) and this puts the level back.
 * A trim rather than a new default because the slider tops out at 1 and a
 * saved mix keeps its own slider value -- the trim applies to everyone.
 */
export const ENGINE_MODEL_TRIM = 2.8;

/** The helmet's high-frequency loss for the cockpit listener: see startModel. */
export const HELMET = { shelfHz: 1200, gainDb: -8 };

export const MIX_LABELS = {
  master: "Master",
  engine: "Engine",
  tyres: "Tyres",
  wind: "Wind",
  cones: "Cone strikes",
  cues: "Timing cues",
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
    // Honour the sound toggle: it is applied before the context exists.
    this.master.gain.value = this.enabled ? this.mix.master : 0;
    // A brick-wall on the way out. The engine model soft-clips its own
    // output, but squeal, wind, a cone and a shift clunk all sum on top of
    // it before the master, and at master 1.0 that hard-clipped.
    this.limiter = ctx.createDynamicsCompressor();
    this.limiter.threshold.value = -6;
    this.limiter.knee.value = 0;
    this.limiter.ratio.value = 12;
    this.limiter.attack.value = 0.002;
    this.limiter.release.value = 0.10;
    this.master.connect(this.limiter);
    this.limiter.connect(ctx.destination);

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
    this.noiseBuf = noiseBuf;   // one-shots take their own source off it
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
    // A second noise source, started a second into the same buffer, so the
    // two are decorrelated: wind from one side of the car does not sound
    // like a copy of the other.
    this.noise2 = ctx.createBufferSource();
    this.noise2.buffer = noiseBuf;
    this.noise2.loop = true;
    this.noise2.loopStart = 0;
    this.noise2.start(0, 1.0);
    const branchFrom = (src, type, freq, q, gain, pan) => {
      const f = ctx.createBiquadFilter();
      f.type = type; f.frequency.value = freq; f.Q.value = q;
      const g = ctx.createGain();
      g.gain.value = gain;
      const p = ctx.createStereoPanner();
      p.pan.value = pan;
      src.connect(f); f.connect(g); g.connect(p); p.connect(this.master);
      return { f, g, p };
    };

    this.induction = branch("bandpass", 700, 0.8, 0);
    // Tyres, per axle: scrub (broadband) front and rear from decorrelated
    // noise, a quiet tonal squeal past the limit, wheelspin, rolling noise.
    this.scrubF = branch("bandpass", 700, 0.9, 0);
    this.scrubR = branchFrom(this.noise2, "bandpass", 650, 0.9, 0, 0);
    this.squeal = branch("bandpass", 1100, 6.0, 0);
    this.spin = branchFrom(this.noise2, "bandpass", 420, 0.8, 0, 0);
    this.rolling = branch("bandpass", 800, 0.6, 0);
    // Wind on both sides of the helmet, decorrelated, so speed has width.
    this.wind = branchFrom(this.noise, "lowpass", 520, 0.7, 0, -0.65);
    this.wind2 = branchFrom(this.noise2, "lowpass", 560, 0.7, 0, 0.65);
    // The road through the seat: a low rumble that rises with speed. It is
    // what makes 60 km/h in a cockpit feel like motion rather than a video.
    this.road = branch("lowpass", 90, 0.8, 0);
    // A locked wheel is not a scrubbing one: it is a flat spot being dragged,
    // lower and rougher than squeal. Was never voiced, so a braking lock-up
    // sounded exactly like a power slide and ABS was inaudible.
    this.lockup = branch("lowpass", 520, 1.2, 0);
    // Flow over the helmet shell: the hiss that only arrives at speed.
    this.windHiss = branchFrom(this.noise2, "bandpass", 2600, 0.8, 0, 0);

    // ---- drivetrain: chain, gearbox mesh, primary drive, overrun rattle ----
    // Oscillators whose frequencies follow the gearing (see DRIVETRAIN), on
    // one bus scaled by the Engine slider and by the camera.
    this.driveBus = ctx.createGain();
    this.driveBus.gain.value = 0;
    this.driveBus.connect(this.master);
    const tone = (type, lowpassHz) => {
      const o = ctx.createOscillator();
      o.type = type;
      o.frequency.value = 100;
      const f = ctx.createBiquadFilter();
      f.type = "lowpass"; f.frequency.value = lowpassHz;
      const gn = ctx.createGain();
      gn.gain.value = 0;
      o.connect(f); f.connect(gn); gn.connect(this.driveBus);
      o.start();
      return { o, f, g: gn };
    };
    this.chain = tone("sawtooth", 2500);
    this.gearMesh = tone("triangle", 6000);
    this.primaryMesh = tone("sine", 16000);
    // The chain's polygon buzz: noise narrowed around twice the mesh rate.
    {
      const f = ctx.createBiquadFilter();
      f.type = "bandpass"; f.frequency.value = 400; f.Q.value = 3;
      const gn = ctx.createGain();
      gn.gain.value = 0;
      this.noise2.connect(f); f.connect(gn); gn.connect(this.driveBus);
      this.chainBuzz = { f, g: gn };
    }
    // Backlash rattle: band noise, amplitude-modulated at the crank rate.
    {
      const f = ctx.createBiquadFilter();
      f.type = "bandpass"; f.frequency.value = 1800; f.Q.value = 1.2;
      const gn = ctx.createGain();
      gn.gain.value = 0;
      const am = ctx.createGain();
      am.gain.value = 0.5;
      const lfo = ctx.createOscillator();
      lfo.type = "square"; lfo.frequency.value = 100;
      const depth = ctx.createGain();
      depth.gain.value = 0.5;
      lfo.connect(depth); depth.connect(am.gain);
      lfo.start();
      this.noise.connect(f); f.connect(gn); gn.connect(am); am.connect(this.driveBus);
      this.rattle = { f, g: gn };
      this.rattleLfo = lfo;
    }
    // Off the course: gravel and grass under the floor. The only cue that
    // the lap was void used to be a toast.
    this.surface = branch("lowpass", 240, 0.9, 0);

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
      this.modelGain.gain.value = this.mix.engine * ENGINE_MODEL_TRIM;
      // The helmet. The engine model was matched to a microphone outside the
      // car; the driver hears it through a helmet, which takes the top off:
      // roughly 5-10 dB above 1-2 kHz. A high shelf on the engine, cockpit
      // cameras only (setCamera), and it answered the team driver's "it
      // sounds very high pitched".
      this.helmet = ctx.createBiquadFilter();
      this.helmet.type = "highshelf";
      this.helmet.frequency.value = HELMET.shelfHz;
      this.helmet.gain.value = (this.camera?.inside ?? true) ? HELMET.gainDb : 0;
      node.connect(this.helmet);
      this.helmet.connect(this.modelGain);
      this.modelGain.connect(this.master);
      // Width. The model is mono, and a mono engine in headphones sits in
      // the middle of the skull. Two short, unequal delays panned either
      // way (a Haas pair) spread it without moving its centre; the direct
      // path stays dominant so nothing smears.
      //
      // The copies are high-passed at 800 Hz first. Full-band, a delayed
      // copy at 0.32 comb-filters the note by +-7 dB per ear (13 dB summed to
      // mono on laptop speakers), with notches that sweep through the firing
      // fundamental as the revs change: at 8000 rpm it was cut 7 dB in one
      // ear. That is the low end, and it belongs in the middle anyway; the
      // width only needs the upper harmonics and the rasp.
      this.widthFilter = ctx.createBiquadFilter();
      this.widthFilter.type = "highpass";
      this.widthFilter.frequency.value = 800;
      this.widthFilter.Q.value = Math.SQRT1_2;
      this.modelGain.connect(this.widthFilter);
      this.width = [];
      for (const [delayS, pan] of [[0.0058, -0.6], [0.0091, 0.6]]) {
        const d = ctx.createDelay(0.05);
        d.delayTime.value = delayS;
        const g = ctx.createGain();
        g.gain.value = 0.32;
        const p = ctx.createStereoPanner();
        p.pan.value = pan;
        this.widthFilter.connect(d); d.connect(g); g.connect(p); p.connect(this.master);
        this.width.push({ d, g, p });
      }
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

  /**
   * Pause or resume the whole context. Paused, hidden or on the menu the
   * engine must go quiet; left running it holds the last operating point
   * (12,000 rpm wide open, say) for as long as the window is in the
   * background, and keeps rendering on the audio thread.
   */
  setRunning(on) {
    const ctx = this.ctx;
    if (!ctx) return;
    if (on) {
      if (ctx.state === "suspended") ctx.resume().catch(() => {});
    } else if (ctx.state === "running") {
      ctx.suspend().catch(() => {});
    }
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
    if (this.modelGain) this.modelGain.gain.value = this.mix.engine * ENGINE_MODEL_TRIM;
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
   *   rpm, throttle 0..1, torqueNm, throttlePlate 0..1, limiter bool (the
   *   ignition is being cut; torqueNm is then what the engine would make),
   *   gear (0-based; a change is the shift clunk), speed m/s, slip 0..~2
   *   (worst axle utilisation), wheelspin 0..1, lock 0..1, offTrack bool,
   *   shifting bool; and per axle, when available: utilF/utilR, kappaF/
   *   kappaR, slipFDeg/slipRDeg, and driveForceN (the chain's load)
   */
  update(s, dt) {
    if (!this.ready || !this.enabled) return;
    const ctx = this.ctx;
    const now = ctx.currentTime;

    // The dogs engaging. Voiced when the gear actually changes -- the end of
    // the 100 ms cut -- which is also the only moment both the JS model, the
    // rig and a replay agree on. A downshift gets the exhaust chuff of the
    // blip on top. Above the rate cap below: a gear change is an edge and
    // must never fall in a skipped frame.
    if (s.gear != null) {
      if (this._lastGear != null && s.gear !== this._lastGear) this.shiftClunk(s.gear < this._lastGear);
      this._lastGear = s.gear;
    }

    // Rate cap. Called every rendered frame, this pushed one message to the
    // worklet and 18 `setTargetAtTime`s into the graph -- ~2600 automation
    // events a second at 144 Hz -- for glides of 12-50 ms that cannot follow
    // faster than ~60 Hz anyway; and every message made the worklet re-solve
    // its heat release on the audio thread. So the continuous part runs at
    // most every 12 ms: every frame on a 60 Hz display, every other at 144.
    // The two EDGES the worklet must not learn about late -- the limiter cut
    // and the shift cut -- force a push the frame they change, either way.
    const shifting = !!s.shifting;
    const cut = !!s.limiter && !shifting;
    const edge = cut !== this._lastCut || shifting !== this._lastShifting;
    this._lastCut = cut;
    this._lastShifting = shifting;
    const since = now - (this._lastPushAt ?? -1);
    if (!edge && since < 0.012) return;
    // The glide spans the real interval between pushes, not the frame.
    const glide = Math.max(0.012, Math.min(0.05, Math.max(dt, since) * 2));
    this._lastPushAt = now;

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
        // The plate position, not the pedal: at idle the pedal is at rest but
        // the plate is held at 14%, and that is what the engine is breathing
        // through.
        throttle: shifting ? 0 : (s.throttlePlate ?? s.throttle),
        torqueNm: shifting ? 0 : (s.torqueNm ?? 0),
        cut,
        // Closed throttle at speed: the overrun, where a CBR pops.
        overrun: !shifting && (s.throttlePlate ?? s.throttle) < 0.08 && s.rpm > 6000 && s.speed > 6,
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

    // Induction hiss, for the oscillator fallback only. The physical model
    // carries the intake itself now -- the plenum and restrictor as a tuned
    // source, heard louder from the cockpit -- and filtered noise on top of it
    // was hiss, not intake.
    const induction = this.usingModel ? 0 : 0.05 * s.throttle * rev * this.mix.engine;
    this.induction.g.gain.setTargetAtTime(induction, now, glide);
    this.induction.f.frequency.setTargetAtTime(400 + firing * 1.6, now, glide);

    const cam = this.camera ?? { inside: true, wind: 1 };
    const v = Math.max(0, s.speed || 0);
    const speedGate = Math.min(1, v / 3);

    // ---- tyres ------------------------------------------------------------
    // Racing slicks on asphalt mostly SCRUB -- a broadband, gritty roar that
    // rises through the last few percent before the limit -- and squeal only a
    // little, and only past it. A road tyre's tonal squeal is what makes a
    // sim sound cartoonish. Each axle is voiced from its own utilisation and
    // slip angle, so an understeering push (front) and a sliding rear sound
    // like different ends of the car. Longitudinal slip has its own voices:
    // wheelspin (rear, driven) and lock-up.
    const utilF = s.utilF ?? s.slip ?? 0;
    const utilR = s.utilR ?? s.slip ?? 0;
    const slipF = Math.abs(s.slipFDeg ?? 0);
    const slipR = Math.abs(s.slipRDeg ?? 0);
    const scrubOf = (util, slipDeg) =>
      Math.max(0, Math.min(1, Math.max((util - 0.78) / 0.32, (slipDeg - 4) / 8)));
    const scrubF = scrubOf(utilF, slipF) * speedGate;
    const scrubR = scrubOf(utilR, slipR) * speedGate;
    const tl = this.mix.tyres;
    this.scrubF.g.gain.setTargetAtTime(TYRE.scrub * scrubF * tl, now, glide);
    this.scrubF.f.frequency.setTargetAtTime(520 + 9 * v + 300 * scrubF, now, glide);
    this.scrubR.g.gain.setTargetAtTime(TYRE.scrub * scrubR * tl, now, glide);
    this.scrubR.f.frequency.setTargetAtTime(480 + 8 * v + 280 * scrubR, now, glide);
    // The tonal part, past the peak of the curve only, and quiet.
    const over = Math.max(0, Math.min(1, (Math.max(utilF, utilR) - 1.0) / 0.25))
      * speedGate;
    this.squeal.g.gain.setTargetAtTime(TYRE.squeal * over * tl, now, glide);
    this.squeal.f.frequency.setTargetAtTime(880 + 420 * over + 6 * v, now, glide);
    // Wheelspin: the driven rears turning faster than the road -- a coarse
    // roar whose pitch follows the slip speed.
    const kR = s.kappaR ?? ((s.wheelspin ?? 0) + 0.15);
    const spin = Math.max(0, Math.min(1, (kR - 0.12) / 0.35)) * speedGate;
    this.spin.g.gain.setTargetAtTime(TYRE.spin * spin * tl, now, glide);
    this.spin.f.frequency.setTargetAtTime(300 + 900 * spin + 4 * v, now, glide);
    // Lock-up: a flat spot being dragged, lower and rougher still.
    const lock = Math.max(0, Math.min(1, s.lock ?? 0)) * Math.min(1, v / 4);
    this.lockup.g.gain.setTargetAtTime(TYRE.lock * lock * tl, now, glide);
    this.lockup.f.frequency.setTargetAtTime(380 + 320 * lock, now, glide);
    // Rolling: tread on the surface texture, ~ speed^1.3 (road-tyre noise
    // scales 30-40 log(v); a slick on smooth asphalt is at the quiet end).
    const roll = Math.min(1.2, (v / 30) ** 1.3);
    this.rolling.g.gain.setTargetAtTime(TYRE.rolling * roll * tl * (cam.inside ? 1 : 0.6), now, glide);
    this.rolling.f.frequency.setTargetAtTime(500 + 14 * v, now, glide);

    // ---- wind and road -----------------------------------------------------
    // Aerodynamic noise at the helmet goes as dynamic pressure, speed^2: a
    // low buffeting roar each side (decorrelated) plus the hiss of flow over
    // the helmet shell that only comes in at speed.
    const q = Math.min(1.1, (v / 32) ** 2);
    const windLevel = TYRE.wind * q * this.mix.wind * cam.wind;
    this.wind.g.gain.setTargetAtTime(windLevel, now, glide);
    this.wind2.g.gain.setTargetAtTime(windLevel * 0.9, now, glide);
    this.wind.f.frequency.setTargetAtTime(320 + v * 22, now, glide);
    this.wind2.f.frequency.setTargetAtTime(360 + v * 24, now, glide);
    this.windHiss.g.gain.setTargetAtTime(TYRE.windHiss * q * this.mix.wind * cam.wind, now, glide);
    // The road through the seat, felt more than heard: on the car only.
    const road = Math.min(1, v / 25) * (cam.inside ? 1 : 0.25);
    this.road.g.gain.setTargetAtTime(TYRE.road * road * this.mix.wind, now, glide);
    this.road.f.frequency.setTargetAtTime(70 + v * 1.5, now, glide);

    // Gravel is not steady: a slow flutter is most of what makes it read as
    // a surface rather than a hiss.
    const off = s.offTrack ? Math.min(1, v / 12) : 0;
    const flutter = 0.7 + 0.3 * Math.sin(now * 41) * Math.sin(now * 7.3);
    this.surface.g.gain.setTargetAtTime(0.16 * off * flutter * tl, now, glide);
    this.surface.f.frequency.setTargetAtTime(200 + v * 9, now, glide);

    // ---- drivetrain ---------------------------------------------------------
    // Frequencies from the car's own gearing (see DRIVETRAIN). The chain is
    // exposed right behind the driver and is the one you hear: its mesh
    // (rear sprocket teeth x wheel rev/s) and the polygon buzz around it. The
    // gearbox is a sportbike cassette box -- spur gears, but small, oiled and
    // inside the cases -- so its mesh and the primary drive's are barely-there
    // texture. All of it scales with the load through the chain.
    const wheelRps = v / (2 * Math.PI * SDM26.tireRadiusM);
    const mainRps = (s.rpm / 60) * DRIVETRAIN.primary[1] / DRIVETRAIN.primary[0];
    const g = Math.max(0, Math.min(DRIVETRAIN.gears.length - 1, s.gear ?? 0));
    const fChain = Math.max(20, DRIVETRAIN.rearSprocket * wheelRps);
    const fGear = Math.max(20, DRIVETRAIN.gears[g][1] * mainRps);
    const fPrimary = Math.max(20, DRIVETRAIN.primary[1] * (s.rpm / 60));
    const load = Math.max(0, Math.min(1, Math.abs(s.driveForceN ?? 0) / 3000));
    const moving = Math.min(1, v / 2);
    const busLevel = this.mix.engine * (cam.inside ? 1 : 0.35);
    this.driveBus.gain.setTargetAtTime(busLevel, now, glide);
    this.chain.o.frequency.setTargetAtTime(fChain, now, glide);
    this.chain.g.gain.setTargetAtTime(DRIVETRAIN.level.chain * moving * (0.35 + 0.65 * load), now, glide);
    this.chainBuzz.f.frequency.setTargetAtTime(fChain * 2, now, glide);
    this.chainBuzz.g.gain.setTargetAtTime(DRIVETRAIN.level.chainBuzz * moving * (0.3 + 0.7 * load), now, glide);
    this.gearMesh.o.frequency.setTargetAtTime(fGear, now, glide);
    this.gearMesh.g.gain.setTargetAtTime(shifting ? 0 : DRIVETRAIN.level.gear * moving * load, now, glide);
    this.primaryMesh.o.frequency.setTargetAtTime(Math.min(fPrimary, 16000), now, glide);
    this.primaryMesh.g.gain.setTargetAtTime(DRIVETRAIN.level.primary * (0.3 + 0.7 * load) * Math.min(1, s.rpm / 4000), now, glide);
    // Backlash rattle on the overrun: the gears unloaded, chattering at the
    // crank rate. Closed throttle, at speed, not in a shift.
    const overrun = !shifting && (s.throttlePlate ?? s.throttle ?? 0) < 0.08 && v > 5 && s.rpm > 4000 ? 1 : 0;
    this.rattle.g.gain.setTargetAtTime(DRIVETRAIN.level.rattle * overrun, now, glide);
    this.rattleLfo.frequency.setTargetAtTime(Math.max(10, s.rpm / 60), now, glide);
    this._lastLoad = load;
  }

  /**
   * Which camera the sound is heard from.
   *
   * The cockpit and nose are on the car: the boxy close reflections of the
   * roll hoop and the sidepod, the wind on the helmet, the road through the
   * seat. Chase and walkaround are outside it: a wider, more diffuse space,
   * little wind, no seat. The engine model already carries both impulse
   * responses; nothing ever switched them.
   */
  setCamera(name) {
    const inside = name === "Cockpit" || name === "Nose";
    this.camera = { inside, wind: name === "Nose" ? 1.25 : inside ? 1.0 : name === "Chase" ? 0.5 : 0.25 };
    this.modelNode?.port.postMessage({ type: "cabin", cabin: inside ? "cockpit" : "trackside" });
    if (this.helmet) this.helmet.gain.value = inside ? HELMET.gainDb : 0;
  }

  /** Forget the last gear and clear the engine model: the car was respawned. */
  reset() {
    this._lastGear = null;
    this.modelNode?.port.postMessage({ type: "reset" });
  }

  // ---- one-shots ----------------------------------------------------------

  /** A short burst of the shared noise through one filter, decaying. */
  burst({ type, freq, q, gain, decay, at }) {
    if (gain < 1e-4) return;
    const ctx = this.ctx;
    const src = ctx.createBufferSource();
    src.buffer = this.noiseBuf;
    const f = ctx.createBiquadFilter();
    f.type = type; f.frequency.value = freq; f.Q.value = q;
    const g = ctx.createGain();
    g.gain.setValueAtTime(gain, at);
    g.gain.exponentialRampToValueAtTime(0.001, at + decay);
    src.connect(f); f.connect(g); g.connect(this.master);
    src.start(at);
    src.stop(at + decay + 0.02);
  }

  /** A pitched thud: a sine dropping in pitch as it decays. */
  thump(freq, dur, gain, at) {
    if (gain < 1e-4) return;
    const ctx = this.ctx;
    const osc = ctx.createOscillator();
    osc.type = "sine";
    osc.frequency.setValueAtTime(freq, at);
    osc.frequency.exponentialRampToValueAtTime(freq * 0.55, at + dur);
    const g = ctx.createGain();
    g.gain.setValueAtTime(gain, at);
    g.gain.exponentialRampToValueAtTime(0.001, at + dur);
    osc.connect(g); g.connect(this.master);
    osc.start(at);
    osc.stop(at + dur + 0.02);
  }

  /** A clean tone for a timing cue. */
  beep(freq, dur, gain, at) {
    if (gain < 1e-4) return;
    const ctx = this.ctx;
    const osc = ctx.createOscillator();
    osc.type = "sine";
    osc.frequency.value = freq;
    const g = ctx.createGain();
    g.gain.setValueAtTime(0.0001, at);
    g.gain.linearRampToValueAtTime(gain, at + 0.008);
    g.gain.setValueAtTime(gain, at + dur - 0.02);
    g.gain.exponentialRampToValueAtTime(0.0001, at + dur);
    osc.connect(g); g.connect(this.master);
    osc.start(at);
    osc.stop(at + dur + 0.01);
  }

  /** The gearbox: dog ring clunk, and the blip's exhaust chuff on the way down. */
  shiftClunk(down) {
    if (!this.ready || !this.enabled) return;
    const now = this.ctx.currentTime;
    // Louder the harder the dogs go in: the load through the chain just
    // before the shift. Inside the car it comes through the seat.
    const cam = this.camera ?? { inside: true };
    const lvl = this.mix.engine * (0.7 + 0.3 * (this._lastLoad ?? 0.5)) * (cam.inside ? 1 : 0.5);
    this.burst({ type: "bandpass", freq: 2400, q: 1.2, gain: 0.14 * lvl, decay: 0.05, at: now });
    this.burst({ type: "bandpass", freq: 3900, q: 3.0, gain: 0.06 * lvl, decay: 0.025, at: now });
    this.thump(75, 0.08, 0.16 * lvl, now);
    if (down) this.burst({ type: "lowpass", freq: 420, q: 0.8, gain: 0.10 * lvl, decay: 0.14, at: now });
  }

  /**
   * A timing cue: green, a sector, a lap, the flag, an excursion. Short
   * tones, so they never fight the engine for attention -- just enough that
   * the driver does not have to read the toast.
   */
  cue(kind) {
    if (!this.ready || !this.enabled) return;
    const seq = CUES[kind];
    if (!seq) return;
    const level = 0.16 * this.mix.cues;
    let t = this.ctx.currentTime;
    for (const [freq, dur] of seq) {
      this.beep(freq, dur, level, t);
      t += dur + 0.03;
    }
  }

  /**
   * A cone strike: hollow plastic against the nose. Was a square-wave sweep,
   * which read as a game "boop" rather than a thing being hit.
   */
  coneHit() {
    if (!this.ready || !this.enabled) return;
    const now = this.ctx.currentTime;
    const peak = 0.5 * this.mix.cones;
    if (peak < 1e-4) return;
    this.burst({ type: "bandpass", freq: 900, q: 0.7, gain: peak, decay: 0.12, at: now });
    this.burst({ type: "bandpass", freq: 2600, q: 1.5, gain: peak * 0.5, decay: 0.05, at: now });
    this.thump(58, 0.07, peak * 0.9, now);
  }
}

/** Tone sequences for `cue`: [frequency Hz, duration s]. */
const CUES = {
  green: [[880, 0.14]],
  sector: [[990, 0.08]],
  sectorUp: [[1320, 0.07], [1760, 0.10]],
  sectorDown: [[660, 0.12]],
  lap: [[990, 0.08], [1320, 0.10]],
  invalid: [[440, 0.16], [330, 0.18]],
  off: [[330, 0.20]],
  finish: [[1100, 0.08], [1100, 0.08], [1470, 0.18]],
};
