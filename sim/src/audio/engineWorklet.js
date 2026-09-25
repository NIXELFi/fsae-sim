// AudioWorklet host for the physically-modelled engine.
//
// The synthesiser must run on the audio thread. Rendering it on the main
// thread and pushing buffers would put every garbage collection, layout and
// WebGL draw between the engine and the speaker, and the result is a note that
// crackles whenever the frame time moves -- which, in a driving game, is
// exactly when the engine is doing something interesting.
//
// The processor owns the model. The main thread only ever posts an operating
// point: rpm, throttle, torque. That is a deliberately narrow interface, and it
// is the same one the Rust build uses.

import {
  EngineAudio,
  cbr600rrSdm26,
  singleCylinder450,
  DEFAULT_AUDIO_CONFIG,
} from "./engineAudio.js";

const ENGINES = {
  "cbr600rr-sdm26": cbr600rrSdm26,
  "single-450": singleCylinder450,
};

class EngineProcessor extends AudioWorkletProcessor {
  constructor(options) {
    super();
    const opts = (options && options.processorOptions) || {};
    const make = ENGINES[opts.engine] || cbr600rrSdm26;

    this.engine = new EngineAudio(make(), {
      ...DEFAULT_AUDIO_CONFIG,
      // `sampleRate` is a global inside an AudioWorkletGlobalScope. Using the
      // context's actual rate matters: the delay lines are sized in samples, so
      // a model built for 48 kHz running in a 44.1 kHz context would put every
      // pipe resonance about 9% sharp.
      sampleRate,
      ...(opts.config || {}),
    });

    this.silent = false;
    this.gain = 1;

    this.port.onmessage = (e) => {
      const m = e.data || {};
      switch (m.type) {
        case "operating-point":
          this.engine.setOperatingPoint(m.rpm, m.throttle, m.torqueNm, !!m.cut, !!m.overrun);
          break;
        case "running":
          this.engine.setRunning(!!m.running);
          break;
        case "parameters":
          this.engine.setParameters(m.parameters || {});
          break;
        case "cabin":
          this.engine.setCabin(m.cabin);
          break;
        case "gain":
          this.gain = Math.min(Math.max(m.gain, 0), 2);
          break;
        case "reset":
          this.engine.reset();
          break;
        case "timeline":
          // Offline rendering (a replay exported to video): every operating
          // point up front, each applied once the render clock reaches its
          // `at`. Live messages would race the renderer, which runs as fast
          // as it can.
          this.timeline = m.points || [];
          this.timelineAt = 0;
          this.port.postMessage({ type: "timeline-ready" });
          break;
        default:
          break;
      }
    };
  }

  process(_inputs, outputs) {
    const out = outputs[0];
    if (!out || out.length === 0) return true;

    const channel = out[0];
    const tl = this.timeline;
    if (tl) {
      // `currentTime` is the start of this render quantum (128 samples).
      while (this.timelineAt < tl.length && tl[this.timelineAt].at <= currentTime) {
        const m = tl[this.timelineAt++];
        this.engine.setOperatingPoint(m.rpm, m.throttle, m.torqueNm, !!m.cut, !!m.overrun);
      }
    }
    this.engine.render(channel);

    if (this.gain !== 1) {
      for (let i = 0; i < channel.length; i++) channel[i] *= this.gain;
    }
    // Mono model, so copy rather than render twice.
    for (let c = 1; c < out.length; c++) out[c].set(channel);

    // Never return false: that permanently ends the processor, and there is no
    // way to restart it short of rebuilding the whole node.
    return true;
  }
}

registerProcessor("engine-processor", EngineProcessor);
