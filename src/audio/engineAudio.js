// Physically-modelled internal-combustion engine audio.
//
// A port of the `engine-audio` Rust crate in ../../fsae-sim-rs, which is itself
// adapted from ange-yaghi/engine-sim (MIT). The repository the community
// edition lives in ships the built application and no source; the algorithms
// follow the original open codebase.
//
// There are no recordings here. The chain is:
//
//   crank angle
//     -> per-cylinder pressure (single-zone, Wiebe heat release)
//     -> flow through the exhaust valve
//     -> exhaust waveguide (primaries -> collector -> tailpipe -> open end)
//     -> synthesiser (jitter, DC removal, derivative, noise, convolution)
//     -> samples
//
// Two implementations of the same model is a cost worth naming. The
// alternative was compiling the Rust to WebAssembly, which would have ended
// this frontend's zero-build-step property -- the reason a `.js` edit is
// instantly live in the browser. Keeping them in lockstep is handled the same
// way sim-core is: the Rust side emits golden vectors and `tools/validate.js`
// checks this file against them. Change one, regenerate, re-check.
//
// The port is deliberately literal. Where a line here looks unidiomatic for
// JavaScript, it is matching the Rust so the two can be diffed by eye.

// ---------------------------------------------------------------------------
// DSP primitives
// ---------------------------------------------------------------------------

/**
 * xorshift32.
 *
 * The 32-bit generator, rather than a better 64-bit one, exists precisely so
 * this file and the Rust crate draw identical noise: JavaScript has no native
 * 64-bit integer arithmetic outside BigInt. Since jitter and air noise both
 * reach the output, different noise would make the two waveforms diverge from
 * the first sample and the golden-vector comparison would be worthless.
 */
export class Rng {
  constructor(seed) {
    // Rust folds a u64 seed as `(seed as u32) ^ ((seed >> 32) as u32)`. Doing
    // that in JavaScript needs the high half taken by division, not by `>>>`:
    // `seed >>> 0` on a value that already fits in 32 bits returns the value
    // itself, so `seed ^ (seed >>> 0)` is identically zero and every generator
    // silently collapses to the same fallback state.
    const lo = seed >>> 0;
    const hi = Math.floor(seed / 4294967296) >>> 0;
    const s = (lo ^ hi) >>> 0;
    this.state = s === 0 ? 0x9e3779b9 : s;
  }

  nextU32() {
    let x = this.state;
    x = (x ^ (x << 13)) >>> 0;
    x = (x ^ (x >>> 17)) >>> 0;
    x = (x ^ (x << 5)) >>> 0;
    this.state = x;
    return x;
  }

  /** Uniform in [-1, 1). */
  uniform() {
    return (this.nextU32() >>> 8) / 8388608 - 1;
  }
}

/** One-pole low pass. Isolates DC, and band-limits noise. */
export class LowPassFilter {
  constructor(cutoffHz, sampleRate) {
    this.y = 0;
    this.setCutoff(cutoffHz, sampleRate);
  }

  setCutoff(cutoffHz, sampleRate) {
    const rc = 1 / (2 * Math.PI * Math.max(cutoffHz, 1e-3));
    const dt = 1 / sampleRate;
    this.alpha = dt / (rc + dt);
  }

  f(x) {
    this.y += this.alpha * (x - this.y);
    return this.y;
  }
}

/** Second-order Butterworth low pass, transposed direct form II. */
export class ButterworthLowPass {
  constructor(cutoffHz, sampleRate) {
    this.z1 = 0;
    this.z2 = 0;
    this.setCutoff(cutoffHz, sampleRate);
  }

  setCutoff(cutoffHz, sampleRate) {
    // Below Nyquist, or the prewarp blows up.
    const fc = Math.min(Math.max(cutoffHz, 1), sampleRate * 0.49);
    const k = Math.tan((Math.PI * fc) / sampleRate);
    const k2 = k * k;
    const norm = 1 / (1 + Math.SQRT2 * k + k2);
    this.b0 = k2 * norm;
    this.b1 = 2 * this.b0;
    this.b2 = this.b0;
    this.a1 = 2 * (k2 - 1) * norm;
    this.a2 = (1 - Math.SQRT2 * k + k2) * norm;
  }

  f(x) {
    const y = this.b0 * x + this.z1;
    this.z1 = this.b1 * x - this.a1 * y + this.z2;
    this.z2 = this.b2 * x - this.a2 * y;
    return y;
  }
}

/**
 * Discrete derivative, normalised to unit gain at a reference frequency.
 *
 * Sound radiated from an open pipe goes as the rate of change of the flow
 * leaving it, not as the pressure inside it -- feed raw manifold pressure to a
 * speaker and you get a muffled thump.
 *
 * The normalisation is the whole point, and getting it wrong is what made the
 * engine shriek. A plain first-order difference divided by `dt` multiplies by
 * the sample rate: at 48 kHz the derivative term arrived roughly 2500x larger
 * than the direct signal it was supposed to be blended into at 1%. Since a
 * derivative's gain rises linearly with frequency, that put 84% of the output
 * energy between 1.5 and 4 kHz at 3000 rpm, against 2.8% below 500 Hz.
 *
 * A difference `1 - z^-1` has magnitude `2 sin(pi f / fs)`. Dividing by that
 * value at `refHz` makes the filter unity-gain there, so `dfFMix` behaves like
 * the mix fraction it is named after, and the high-frequency lift it adds is
 * proportional rather than overwhelming.
 */
export class DerivativeFilter {
  constructor(sampleRate, refHz = 300) {
    this.prev = 0;
    this.scale = 1 / (2 * Math.sin((Math.PI * refHz) / sampleRate));
  }

  f(x) {
    const d = (x - this.prev) * this.scale;
    this.prev = x;
    return d;
  }
}

/** Cycle-to-cycle variation. Without it the output is periodic to the sample. */
export class JitterFilter {
  constructor(amount, sampleRate, seed) {
    this.amount = amount;
    this.lp = new LowPassFilter(400, sampleRate);
    this.rng = new Rng(seed);
  }

  f(x) {
    if (this.amount <= 0) return x;
    const n = this.lp.f(this.rng.uniform());
    return x * (1 + this.amount * n);
  }
}

/**
 * Automatic gain toward a target RMS.
 *
 * This tracks mean square over a long window rather than chasing peaks, and
 * that choice is the fix for a specific failure. The previous version was a
 * peak follower with a 1 ms attack. An exhaust blowdown transient is about ten
 * samples wide at 48 kHz -- far faster than a 1 ms attack can respond -- so the
 * gain was always set by the quiet stretch between pulses, and every pulse then
 * arrived into a gain far too high and slammed into the output clamp. Measured
 * output RMS was 0.99 against a ±1 clamp at every operating point: not levelled
 * audio, a square wave.
 *
 * Averaging over 150 ms sets the *average* level and leaves transients alone.
 * The peaks that result are handled by soft clipping downstream, which rounds
 * them instead of shearing them flat.
 */
export class LevelingFilter {
  constructor(target, sampleRate) {
    this.target = target;
    this.minGain = 1e-5;
    this.maxGain = 4;
    this.meanSquare = target * target;
    // 150 ms: long compared with a firing period even at idle (19 ms at
    // 1600 rpm), so the gain does not pump once per combustion event.
    this.alpha = 1 - Math.exp(-1 / (0.15 * sampleRate));
  }

  gain() {
    return Math.min(
      Math.max(this.target / Math.sqrt(Math.max(this.meanSquare, 1e-14)), this.minGain),
      this.maxGain,
    );
  }

  f(x) {
    this.meanSquare += this.alpha * (x * x - this.meanSquare);
    return x * this.gain();
  }
}

/**
 * Soft clip.
 *
 * `tanh` is linear for small signals and compresses smoothly beyond, so a
 * transient that would have been sheared flat by a hard clamp is rounded
 * instead. Hard clipping generates high-order harmonics across the whole
 * spectrum, which is a large part of what "unbearable" sounded like.
 */
export function softClip(x) {
  return Math.tanh(x);
}

/** Direct-form FIR against a stored impulse response. */
export class ConvolutionFilter {
  constructor(ir) {
    this.setImpulseResponse(ir);
  }

  setImpulseResponse(ir) {
    this.ir = ir instanceof Float32Array ? ir : Float32Array.from(ir);
    this.history = new Float32Array(Math.max(this.ir.length, 1));
    this.cursor = 0;
  }

  f(x) {
    const n = this.history.length;
    const ir = this.ir;
    if (n === 0 || ir.length === 0) return x;
    this.history[this.cursor] = x;
    let acc = 0;
    let idx = this.cursor;
    for (let t = 0; t < ir.length; t++) {
      acc += ir[t] * this.history[idx];
      idx = idx === 0 ? n - 1 : idx - 1;
    }
    this.cursor = (this.cursor + 1) % n;
    return acc;
  }
}

// ---------------------------------------------------------------------------
// Impulse responses
// ---------------------------------------------------------------------------

const CABIN_SHAPES = {
  // Close and boxy: strong early reflections off the roll hoop, floor and
  // sidepod, and a short tail. This is what a driver actually hears.
  cockpit: {
    reflections: [
      [0.6, -0.62],
      [1.1, 0.44],
      [1.9, -0.31],
      [3.2, 0.22],
      [4.8, -0.14],
    ],
    decayS: 0.02,
    cutoffHz: 5500,
  },
  // Further away, more diffuse, with a ground bounce.
  trackside: {
    reflections: [
      [1.8, 0.52],
      [4.5, -0.34],
      [7.9, 0.24],
      [12.0, -0.16],
      [17.5, 0.1],
    ],
    decayS: 0.055,
    cutoffHz: 7500,
  },
  anechoic: null,
};

/**
 * Build an impulse response procedurally.
 *
 * engine-sim ships recorded responses in its own sound library; those are its
 * assets, not ours to redistribute. Discrete early reflections over a decaying
 * band-limited noise tail is the structure of a real small-space response and
 * does the same job: it gives the dry waveguide output a body.
 */
export function buildImpulseResponse(cabin, taps, sampleRate, seed) {
  const shape = CABIN_SHAPES[cabin];
  if (!shape || taps === 0) return Float32Array.of(1);

  const ir = new Float32Array(taps);
  const rng = new Rng(seed);
  ir[0] = 1;

  for (const [ms, amp] of shape.reflections) {
    const idx = Math.round(ms * 1e-3 * sampleRate);
    if (idx < taps) ir[idx] += amp;
  }

  const start = Math.floor(0.7e-3 * sampleRate);
  const tau = shape.decayS * sampleRate;
  for (let i = start; i < taps; i++) {
    ir[i] += rng.uniform() * Math.exp(-i / tau) * 0.35;
  }

  const lp = new ButterworthLowPass(shape.cutoffHz, sampleRate);
  for (let i = 0; i < taps; i++) ir[i] = lp.f(ir[i]);

  // Normalise by energy, not peak, so switching cabin does not change loudness.
  let energy = 0;
  for (let i = 0; i < taps; i++) energy += ir[i] * ir[i];
  if (energy > 1e-12) {
    const g = 1 / Math.sqrt(energy);
    for (let i = 0; i < taps; i++) ir[i] *= g;
  }
  return ir;
}

// ---------------------------------------------------------------------------
// Engine definition
// ---------------------------------------------------------------------------

export const DEFAULT_GAS = {
  ambientPa: 101325,
  ambientK: 293,
  exhaustKMax: 1150,
  exhaustKMin: 650,
  gamma: 1.33,
  rSpecific: 287,
};

export function speedOfSound(gas, temperatureK) {
  return Math.sqrt(gas.gamma * gas.rSpecific * Math.max(temperatureK, 1));
}

export function gasDensity(gas, pressurePa, temperatureK) {
  return pressurePa / (gas.rSpecific * Math.max(temperatureK, 1));
}

/**
 * @param dampingHz  Cutoff of the per-traverse loss filter.
 *
 * Real pipe losses are frequency-dependent -- viscous and thermal losses in the
 * boundary layer grow roughly as the square root of frequency, so a wave loses
 * its high frequencies fastest. Modelling the loss as a flat multiplier instead
 * leaves every mode with the same Q, and the consequence is audible: at low rpm
 * the combustion pulses are far apart, and the lightly damped 1-2 kHz pipe
 * modes ring on between them until they are all you can hear. A one-pole
 * low-pass in the delay path is the standard waveguide treatment and is closer
 * to the physics than the flat multiplier it replaces.
 */
export function pipeFromDiameter(lengthM, diameterM, loss, dampingHz = 3000) {
  return { lengthM, areaM2: (Math.PI * diameterM * diameterM) / 4, loss, dampingHz };
}

/**
 * The SDM26's engine: a Honda CBR600RR (PC40) inline four, FSAE-restricted.
 *
 * Geometry is the published Honda specification. The rev limit matches the one
 * the vehicle model already uses. Header lengths are estimates -- a tape
 * measure on the real car would improve the resonance behaviour and nothing
 * else about the model.
 */
export function cbr600rrSdm26() {
  return {
    name: "Honda CBR600RR PC40 (SDM26, FSAE-restricted)",
    boreM: 0.067,
    strokeM: 0.0425,
    conrodM: 0.0905,
    compressionRatio: 12.2,
    // 180-degree crank inline four, firing order 1-2-4-3. The even spacing is
    // what gives it the flat scream instead of a beat.
    firingAnglesDeg: [0, 180, 540, 360],
    timing: {
      evoDeg: 132,
      evcDeg: 372,
      ivoDeg: 348,
      ivcDeg: 576,
      exhaustValveDiameterM: 0.0235,
      exhaustCd: 0.72,
    },
    combustion: { startDeg: -20, durationDeg: 52, wiebeA: 5, wiebeM: 2 },
    // 4-2-1 in reality; modelled 4-1, which keeps the primary resonance and
    // loses the secondary.
    primaries: [0, 1, 2, 3].map(() => pipeFromDiameter(0.42, 0.032, 0.01, 3200)),
    primaryToCollector: [0, 0, 0, 0],
    collectors: [pipeFromDiameter(0.28, 0.048, 0.012, 2400)],
    // The tailpipe carries the muffler. FSAE caps noise at 110 dBA, so the car
    // has one, and a muffler is exactly a device that absorbs the mid and high
    // frequencies while passing the low-frequency pulse. Modelling it as heavy
    // damping on this pipe is cruder than modelling its chambers, and it is the
    // difference between a burble and a whine at idle.
    tailpipes: [pipeFromDiameter(0.55, 0.045, 0.03, 1100)],
    idleRpm: 1600,
    redlineRpm: 14500,
    gas: { ...DEFAULT_GAS },
  };
}

/** A single-cylinder thumper, to prove nothing is hard-wired to four. */
export function singleCylinder450() {
  return {
    name: "450 single",
    boreM: 0.096,
    strokeM: 0.0622,
    conrodM: 0.104,
    compressionRatio: 12,
    firingAnglesDeg: [0],
    timing: {
      evoDeg: 125,
      evcDeg: 375,
      ivoDeg: 345,
      ivcDeg: 570,
      exhaustValveDiameterM: 0.031,
      exhaustCd: 0.7,
    },
    combustion: { startDeg: -18, durationDeg: 55, wiebeA: 5, wiebeM: 2 },
    primaries: [pipeFromDiameter(0.55, 0.038, 0.01, 3200)],
    primaryToCollector: [0],
    collectors: [pipeFromDiameter(0.3, 0.042, 0.012, 2400)],
    tailpipes: [pipeFromDiameter(0.4, 0.04, 0.03, 1100)],
    idleRpm: 1500,
    redlineRpm: 11000,
    gas: { ...DEFAULT_GAS },
  };
}

export function cylinderCount(spec) {
  return spec.firingAnglesDeg.length;
}

export function pistonAreaM2(spec) {
  return (Math.PI * spec.boreM * spec.boreM) / 4;
}

export function displacementPerCylinderM3(spec) {
  return pistonAreaM2(spec) * spec.strokeM;
}

/** Total swept volume, m^3. */
export function displacementM3(spec) {
  return displacementPerCylinderM3(spec) * cylinderCount(spec);
}

export function clearanceVolumeM3(spec) {
  return displacementPerCylinderM3(spec) / Math.max(spec.compressionRatio - 1, 1e-6);
}

/** Cylinder volume at a crank angle, from the slider-crank relation. */
export function volumeAt(spec, thetaDeg) {
  const theta = (thetaDeg * Math.PI) / 180;
  const r = spec.strokeM / 2;
  const l = spec.conrodM;
  const s = r * Math.cos(theta) + Math.sqrt(Math.max(l * l - (r * Math.sin(theta)) ** 2, 0));
  const x = l + r - s;
  return clearanceVolumeM3(spec) + pistonAreaM2(spec) * x;
}

export function validateSpec(spec) {
  const n = cylinderCount(spec);
  if (n === 0) return "engine has no cylinders";
  if (spec.primaries.length !== n) return `${spec.primaries.length} primaries for ${n} cylinders`;
  if (spec.primaryToCollector.length !== n) return "primaryToCollector needs one entry per cylinder";
  for (let i = 0; i < n; i++) {
    if (spec.primaryToCollector[i] >= spec.collectors.length) {
      return `primary ${i} points at a collector that does not exist`;
    }
  }
  if (spec.tailpipes.length !== spec.collectors.length) return "one tailpipe per collector";
  if (spec.compressionRatio <= 1) return "compression ratio must exceed 1";
  if (spec.conrodM <= spec.strokeM / 2) return "conrod is shorter than the crank throw";
  return null;
}

// ---------------------------------------------------------------------------
// Cylinder
// ---------------------------------------------------------------------------

/** Wiebe mass-fraction burned. */
export function wiebe(thetaDeg, startDeg, durationDeg, a, m) {
  if (thetaDeg <= startDeg) return 0;
  const x = Math.min((thetaDeg - startDeg) / Math.max(durationDeg, 1e-3), 1);
  return 1 - Math.exp(-a * Math.pow(x, m + 1));
}

/**
 * Effective exhaust-valve flow area at a crank angle.
 *
 * A raised-cosine lift profile rather than a cam file. The curtain area is
 * capped by the port area, which is the real behaviour and stops the model
 * dumping the whole cylinder in one sample at high lift.
 */
export function exhaustFlowArea(spec, thetaDeg) {
  const t = spec.timing;
  const span = t.evcDeg > t.evoDeg ? t.evcDeg - t.evoDeg : t.evcDeg + 720 - t.evoDeg;
  let rel = thetaDeg - t.evoDeg;
  while (rel < 0) rel += 720;
  if (rel > span) return 0;
  const liftFrac = 0.5 * (1 - Math.cos((2 * Math.PI * rel) / span));
  const maxLift = t.exhaustValveDiameterM * 0.25;
  const curtain = Math.PI * t.exhaustValveDiameterM * liftFrac * maxLift;
  const port = (Math.PI * t.exhaustValveDiameterM * t.exhaustValveDiameterM) / 4;
  return t.exhaustCd * Math.min(curtain, port);
}

const TABLE_BINS = 2880; // 0.25 crank degrees

/**
 * Everything in the cylinder inner loop that is a pure function of crank
 * angle, evaluated once. `Math.pow` and `Math.exp` in `wiebe` are relatively
 * more expensive here than in Rust, so this matters more on this side.
 */
export class CycleTables {
  constructor(spec) {
    this.volume = new Float32Array(TABLE_BINS);
    this.valveArea = new Float32Array(TABLE_BINS);
    this.burned = new Float32Array(TABLE_BINS);
    const c = spec.combustion;
    for (let i = 0; i < TABLE_BINS; i++) {
      const theta = (i * 720) / TABLE_BINS;
      this.volume[i] = volumeAt(spec, theta);
      this.valveArea[i] = exhaustFlowArea(spec, theta);
      // The burn straddles TDC; sample it on the signed axis so the tail
      // before TDC is not truncated.
      const signed = theta > 360 ? theta - 720 : theta;
      this.burned[i] = wiebe(signed, c.startDeg, c.durationDeg, c.wiebeA, c.wiebeM);
    }
  }

  static lookup(table, thetaDeg) {
    const n = table.length;
    const pos = thetaDeg * (n / 720);
    const i = Math.floor(pos);
    const frac = pos - i;
    const a = ((i % n) + n) % n;
    const b = (a + 1) % n;
    return table[a] * (1 - frac) + table[b] * frac;
  }

  volumeAt(theta) {
    return CycleTables.lookup(this.volume, theta);
  }

  valveAreaAt(theta) {
    return CycleTables.lookup(this.valveArea, theta);
  }

  burnedAt(theta) {
    return CycleTables.lookup(this.burned, theta);
  }
}

/** Is theta inside the arc start -> end, going forwards, on a 720 circle? */
function isBetween(theta, start, end) {
  const span = end >= start ? end - start : end + 720 - start;
  let rel = theta - start;
  while (rel < 0) rel += 720;
  return rel <= span;
}

/** Integrate p dV over the closed part of the cycle for a given heat release. */
function indicatedWork(spec, pIvc, heatJ) {
  const c = spec.combustion;
  const gamma = spec.gas.gamma;
  const evo = spec.timing.evoDeg;
  const step = 0.5;
  let theta = spec.timing.ivcDeg - 720;
  let p = pIvc;
  let v = volumeAt(spec, theta);
  let work = 0;

  while (theta < evo) {
    const next = theta + step;
    const vNext = volumeAt(spec, next);
    const dv = vNext - v;
    const dq =
      heatJ *
      (wiebe(next, c.startDeg, c.durationDeg, c.wiebeA, c.wiebeM) -
        wiebe(theta, c.startDeg, c.durationDeg, c.wiebeA, c.wiebeM));
    const dp = ((gamma - 1) / v) * dq - ((gamma * p) / v) * dv;
    work += p * dv;
    p = Math.max(p + dp, 1);
    v = vNext;
    theta = next;
  }
  return work;
}

/**
 * Solve for the heat release that makes one cycle do the requested work.
 *
 * This is where the Helios CFD torque sweep enters the audio. Indicated work
 * relates to torque by T = W * nCyl / (4 pi) for a four-stroke, and work is
 * very nearly affine in heat release, so a secant step plus one correction
 * lands it.
 */
export function calibrate(spec, op) {
  const gas = spec.gas;
  const mapFrac = 0.14 + 0.72 * op.throttle;
  const pIvc = gas.ambientPa * mapFrac;
  const tIvc = gas.ambientK + 40 + 90 * op.throttle;

  const perCylinderTarget = (op.targetTorqueNm * 4 * Math.PI) / cylinderCount(spec);
  const w0 = indicatedWork(spec, pIvc, 0);
  const guess = 600;
  const w1 = indicatedWork(spec, pIvc, guess);
  const slope = (w1 - w0) / guess;

  let q = Math.abs(slope) < 1e-9 ? guess : (perCylinderTarget - w0) / slope;
  q = Math.min(Math.max(q, 0), 5000);
  const w2 = indicatedWork(spec, pIvc, q);
  if (Math.abs(slope) > 1e-9) {
    q = Math.min(Math.max(q + (perCylinderTarget - w2) / slope, 0), 5000);
  }
  return { pIvc, tIvc, heatReleaseJ: q };
}

/**
 * Advance one cylinder. Returns the volumetric flow leaving the exhaust valve,
 * m^3/s, which is the source term for the waveguide.
 *
 * `backPressurePa` is what the runner presents at the port. Feeding that back
 * is the mechanism header tuning works by: a wave returning while the valve is
 * still open changes how much the cylinder can dump.
 */
export function stepCylinder(spec, tables, cyl, crankDeg, dt, cal, op, backPressurePa) {
  let theta = crankDeg - cyl.phaseDeg;
  while (theta < 0) theta += 720;
  while (theta >= 720) theta -= 720;

  const gas = spec.gas;
  const t = spec.timing;
  const degPerS = op.rpm * 6;
  const dtheta = degPerS * dt;

  const v = tables.volumeAt(theta);
  const vNext = tables.volumeAt((theta + dtheta) % 720);
  const dv = vNext - v;

  const area = tables.valveAreaAt(theta);
  const inClosed = isBetween(theta, t.ivcDeg, t.evoDeg);
  const exhaustOpen = area > 0;
  const intakeOpen = isBetween(theta, t.ivoDeg, t.ivcDeg);

  let flowOut = 0;

  if (inClosed) {
    const dq = cal.heatReleaseJ * (tables.burnedAt((theta + dtheta) % 720) - tables.burnedAt(theta));
    const dp = ((gas.gamma - 1) / v) * dq - ((gas.gamma * cyl.pressurePa) / v) * dv;
    cyl.pressurePa = Math.max(cyl.pressurePa + dp, 1000);
  } else if (exhaustOpen) {
    const dpAcross = cyl.pressurePa - backPressurePa;
    const rho = gasDensity(gas, Math.max(cyl.pressurePa, backPressurePa), cyl.temperatureK);

    // Orifice flow, regularised near zero pressure difference.
    //
    // The plain law u = sign(dp) * sqrt(2|dp|/rho) has INFINITE slope at
    // dp = 0. That matters because after blowdown the cylinder sits close to
    // the runner pressure for the whole exhaust stroke, so dp hovers near
    // zero -- and there the square root turns every small returning wave into
    // a large swing in flow, which injects back into the runner and changes
    // the pressure again. The result was a limit cycle: cylinder pressure a
    // clean 76 Hz at 3000 rpm, while the valve flow oscillated at 3 kHz and
    // dominated the entire output.
    //
    // Below `DP_LAMINAR` the law is linear in dp, matched in value at the
    // crossover so the curve is continuous and its slope is finite
    // everywhere. This is also the more physical choice: flow through a
    // restriction at a small pressure difference is viscosity-dominated and
    // proportional to dp, not to its square root. The square-root law is the
    // fully turbulent limit.
    const DP_LAMINAR = 1500; // Pa
    const mag = Math.abs(dpAcross);
    const turbulent = Math.sqrt((2 * DP_LAMINAR) / Math.max(rho, 1e-6));
    let uTarget =
      mag >= DP_LAMINAR
        ? Math.sign(dpAcross) * Math.sqrt((2 * mag) / Math.max(rho, 1e-6))
        : (dpAcross / DP_LAMINAR) * turbulent;

    // An exhaust port flows worse backwards than forwards. The valve seat and
    // the port are shaped for gas leaving the cylinder; reversed, the jet
    // separates and the effective discharge coefficient drops. 0.7 is the usual
    // ballpark. This only matters on a closed throttle, which is exactly where
    // reverse flow dominates -- idle and the overrun.
    if (uTarget < 0) uTarget *= 0.7;

    // Choke it. A port cannot pass gas faster than the local speed of sound
    // however large the pressure ratio, and at blowdown the ratio is enormous.
    // Unchoked, the incompressible orifice equation returns about 930 m/s,
    // which empties the cylinder in a few degrees and slams the waveguide hard
    // enough that the next pulse comes back inverted.
    const cCyl = Math.sqrt(gas.gamma * gas.rSpecific * Math.max(cyl.temperatureK, 1));
    uTarget = Math.min(Math.max(uTarget, -cCyl), cCyl);

    // Port inertance: the slug of gas in the port has mass, so its velocity
    // cannot change instantaneously. A short first-order lag is what that
    // amounts to, and it removes what the regularisation above leaves behind.
    // The time constant is kept well below the blowdown edge -- about three
    // samples at 48 kHz against roughly eight even at the rev limiter -- so it
    // damps the chatter without softening the pulse that makes the sound.
    const tau = 6e-5;
    const k = Math.min(dt / tau, 1);
    cyl.portVelocity += k * (uTarget - cyl.portVelocity);
    const u = cyl.portVelocity;

    const volumetric = area * u;
    flowOut = volumetric;
    const dpFlow = -gas.gamma * cyl.pressurePa * (volumetric / v) * dt;
    const dpVol = ((-gas.gamma * cyl.pressurePa) / v) * dv;
    cyl.pressurePa = Math.max(cyl.pressurePa + dpFlow + dpVol, 1000);
    cyl.portVelocity = u;
  } else if (intakeOpen) {
    const tau = (0.35 / Math.max(degPerS, 1)) * 180;
    const k = Math.min(dt / Math.max(tau, 1e-6), 1);
    cyl.pressurePa += k * (cal.pIvc - cyl.pressurePa);
    cyl.temperatureK += k * (cal.tIvc - cyl.temperatureK);
  } else {
    cyl.pressurePa += 0.05 * (gas.ambientPa - cyl.pressurePa);
  }

  if (inClosed) {
    const scale = Math.max(1 + (dv / v) * (gas.gamma - 1), 0.1);
    cyl.temperatureK = Math.min(Math.max(cyl.temperatureK * scale, 250), 3200);
  } else if (exhaustOpen) {
    cyl.temperatureK = op.exhaustK;
  }

  cyl.exhaustFlow = flowOut;
  return flowOut;
}

// ---------------------------------------------------------------------------
// Exhaust waveguide
// ---------------------------------------------------------------------------

class DelayLine {
  constructor(capacity) {
    this.buf = new Float32Array(Math.max(capacity, 4));
    this.cursor = 0;
    this.delay = 1;
  }

  setDelay(samples) {
    this.delay = Math.min(Math.max(samples, 1), this.buf.length - 2);
  }

  read() {
    const n = this.buf.length;
    const i = Math.floor(this.delay);
    const frac = this.delay - i;
    const a = (this.cursor + n - i) % n;
    const b = (a + n - 1) % n;
    return this.buf[a] * (1 - frac) + this.buf[b] * frac;
  }

  write(x) {
    this.buf[this.cursor] = x;
    this.cursor = (this.cursor + 1) % this.buf.length;
  }

  clear() {
    this.buf.fill(0);
  }
}

class Pipe {
  constructor(spec, sampleRate) {
    // Size for the slowest sound speed we will ever see, so the buffer always
    // holds the delay.
    const capacity = Math.ceil((spec.lengthM / 300) * sampleRate) + 8;
    this.fwd = new DelayLine(capacity);
    this.bwd = new DelayLine(capacity);
    this.areaM2 = spec.areaM2;
    this.lengthM = spec.lengthM;
    this.gain = Math.min(Math.max(1 - spec.loss, 0), 1);
    this.admittance = 1;
    // Frequency-dependent loss, one filter per direction of travel.
    const hz = spec.dampingHz ?? 3000;
    this.dampFwd = new LowPassFilter(hz, sampleRate);
    this.dampBwd = new LowPassFilter(hz, sampleRate);
  }

  /** Delay output with this traverse's losses applied. */
  readFwd() {
    return this.dampFwd.f(this.fwd.read()) * this.gain;
  }

  readBwd() {
    return this.dampBwd.f(this.bwd.read()) * this.gain;
  }

  retune(c, rho, sampleRate) {
    const samples = (this.lengthM / Math.max(c, 1)) * sampleRate;
    this.fwd.setDelay(samples);
    this.bwd.setDelay(samples);
    this.admittance = this.areaM2 / (Math.max(rho, 1e-6) * Math.max(c, 1));
  }
}

/**
 * Reflection at the cylinder end of a primary.
 *
 * r = (A_pipe - A_valve) / (A_pipe + A_valve). A shut valve gives +1, a rigid
 * wall; a valve open to the full pipe area gives 0, perfectly absorbing.
 * Treating this end as rigid regardless of valve position leaves the primary
 * with a Q high enough that pulses interfere with their own echoes and the
 * note stops tracking the firing rate.
 */
function portReflection(pipeAreaM2, valveAreaM2) {
  const a = Math.max(pipeAreaM2, 1e-12);
  const v = Math.max(valveAreaM2, 0);
  return Math.min(Math.max((a - v) / (a + v), -1), 1);
}

export class ExhaustNetwork {
  constructor(spec, sampleRate) {
    this.sampleRate = sampleRate;
    this.primaries = spec.primaries.map((p) => new Pipe(p, sampleRate));
    this.collectors = spec.collectors.map((p) => new Pipe(p, sampleRate));
    this.tailpipes = spec.tailpipes.map((p) => new Pipe(p, sampleRate));
    this.primaryToCollector = spec.primaryToCollector.slice();
    this.rhoC = 1;
    // A real open pipe reflects most of a low-frequency wave back inverted and
    // radiates the rest.
    this.openEndReflection = 0.85;
    this.outputs = new Float32Array(this.tailpipes.length);

    const np = this.primaries.length;
    const nc = this.collectors.length;
    const nt = this.tailpipes.length;
    // Preallocated: step() runs 48,000 times a second.
    this.w = {
      primFwdOut: new Float32Array(np),
      primBwdOut: new Float32Array(np),
      collFwdOut: new Float32Array(nc),
      collBwdOut: new Float32Array(nc),
      tailFwdOut: new Float32Array(nt),
      tailBwdOut: new Float32Array(nt),
      primFwdIn: new Float32Array(np),
      primBwdIn: new Float32Array(np),
      collFwdIn: new Float32Array(nc),
      collBwdIn: new Float32Array(nc),
      tailFwdIn: new Float32Array(nt),
      tailBwdIn: new Float32Array(nt),
    };
    this.setGasState(spec.gas, spec.gas.exhaustKMax);
  }

  /**
   * Retune every delay line for a new gas temperature. Call when the operating
   * point moves, not per sample.
   *
   * At 1150 K the speed of sound is about 660 m/s against 343 in ambient air,
   * so the tuned length of the header genuinely shifts with load. That is why
   * the note changes on the overrun rather than only with rpm.
   */
  setGasState(gas, exhaustK) {
    const c = speedOfSound(gas, exhaustK);
    const rho = gasDensity(gas, gas.ambientPa, exhaustK);
    this.rhoC = rho * c;
    for (const p of [...this.primaries, ...this.collectors, ...this.tailpipes]) {
      p.retune(c, rho, this.sampleRate);
    }
  }

  reset() {
    for (const p of [...this.primaries, ...this.collectors, ...this.tailpipes]) {
      p.fwd.clear();
      p.bwd.clear();
    }
    this.outputs.fill(0);
  }

  /**
   * Pressure the runner presents at cylinder `i`'s exhaust port.
   *
   * `portVelocityMs` is the gas velocity currently going through the valve,
   * positive out of the cylinder. It matters, and leaving it out was the single
   * biggest error in the model.
   *
   * A pipe is not an infinite reservoir. Push gas into it and the pressure at
   * the inlet rises immediately by `rho c u`; pull gas out and it falls. That
   * is the pipe's characteristic impedance, and it acts within the sample --
   * not after the round trip it takes a wave to come back. Without it the
   * exhaust valve saw a constant one atmosphere and could draw from it at will.
   *
   * The consequence was severe at small throttle openings. At idle the cylinder
   * is at about 0.2 atm when the valve opens, so gas rushed backwards into it
   * at sonic velocity through the full valve area and injected a 52 kPa wave --
   * against 58 kPa for a full-power blowdown at the limiter. Idle was as loud
   * as the limiter, and the model's entire dynamic range was 9 dB.
   *
   * With the impedance term the flow throttles itself: the harder it pulls, the
   * lower the pressure it is pulling against.
   */
  portPressure(i, ambientPa, valveAreaM2, portVelocityMs = 0) {
    const p = this.primaries[i];
    const r = portReflection(p.areaM2, valveAreaM2);
    // Deliberately reads the raw delay line rather than `readBwd()`. The
    // damping filters are stateful and are advanced exactly once per sample by
    // `step`; running one here as well would double-filter the primary and
    // clock its state twice per sample.
    const wave = (1 + r) * p.bwd.read() * p.gain;
    // Volume flow through the valve becomes a particle velocity in the pipe.
    const uPipe = (portVelocityMs * valveAreaM2) / Math.max(p.areaM2, 1e-9);
    return ambientPa + wave + this.rhoC * uPipe;
  }

  step(sourceFlow, valveArea) {
    const w = this.w;

    // Read every pipe end before writing anything. Doing it in one pass lets a
    // junction see its own output this sample, which is an algebraic loop and
    // turns into a howl.
    for (let i = 0; i < this.primaries.length; i++) {
      const p = this.primaries[i];
      w.primFwdOut[i] = p.readFwd();
      w.primBwdOut[i] = p.readBwd();
    }
    for (let i = 0; i < this.collectors.length; i++) {
      const p = this.collectors[i];
      w.collFwdOut[i] = p.readFwd();
      w.collBwdOut[i] = p.readBwd();
    }
    for (let i = 0; i < this.tailpipes.length; i++) {
      const p = this.tailpipes[i];
      w.tailFwdOut[i] = p.readFwd();
      w.tailBwdOut[i] = p.readBwd();
    }

    // Cylinder end: a velocity source at a partially reflecting junction.
    for (let i = 0; i < this.primaries.length; i++) {
      const aPipe = Math.max(this.primaries[i].areaM2, 1e-9);
      const u = (sourceFlow[i] || 0) / aPipe;
      const r = portReflection(aPipe, valveArea[i] || 0);
      w.primFwdIn[i] = r * w.primBwdOut[i] + this.rhoC * u;
    }

    // Primaries into collectors: Kelly-Lochbaum scattering.
    for (let c = 0; c < this.collectors.length; c++) {
      let num = 0;
      let den = 0;
      for (let i = 0; i < this.primaryToCollector.length; i++) {
        if (this.primaryToCollector[i] === c) {
          num += this.primaries[i].admittance * w.primFwdOut[i];
          den += this.primaries[i].admittance;
        }
      }
      num += this.collectors[c].admittance * w.collBwdOut[c];
      den += this.collectors[c].admittance;
      const pj = den > 1e-12 ? (2 * num) / den : 0;
      for (let i = 0; i < this.primaryToCollector.length; i++) {
        if (this.primaryToCollector[i] === c) w.primBwdIn[i] = pj - w.primFwdOut[i];
      }
      w.collFwdIn[c] = pj - w.collBwdOut[c];
    }

    // Collector into tailpipe: the same scattering across an area step.
    for (let c = 0; c < this.collectors.length; c++) {
      const yc = this.collectors[c].admittance;
      const yt = this.tailpipes[c].admittance;
      const den = yc + yt;
      const pj = den > 1e-12 ? (2 * (yc * w.collFwdOut[c] + yt * w.tailBwdOut[c])) / den : 0;
      w.collBwdIn[c] = pj - w.collFwdOut[c];
      w.tailFwdIn[c] = pj - w.tailBwdOut[c];
    }

    // Open end: reflect inverted, radiate the rest.
    for (let t = 0; t < this.tailpipes.length; t++) {
      w.tailBwdIn[t] = -this.openEndReflection * w.tailFwdOut[t];
      this.outputs[t] = (1 + this.openEndReflection) * w.tailFwdOut[t];
    }

    for (let i = 0; i < this.primaries.length; i++) {
      this.primaries[i].fwd.write(w.primFwdIn[i]);
      this.primaries[i].bwd.write(w.primBwdIn[i]);
    }
    for (let c = 0; c < this.collectors.length; c++) {
      this.collectors[c].fwd.write(w.collFwdIn[c]);
      this.collectors[c].bwd.write(w.collBwdIn[c]);
    }
    for (let t = 0; t < this.tailpipes.length; t++) {
      this.tailpipes[t].fwd.write(w.tailFwdIn[t]);
      this.tailpipes[t].bwd.write(w.tailBwdIn[t]);
    }
  }
}

// ---------------------------------------------------------------------------
// Synthesiser
// ---------------------------------------------------------------------------

export const DEFAULT_AUDIO_PARAMETERS = {
  volume: 1,
  convolution: 1,
  // How much differentiated signal to blend in. Now that the derivative is
  // normalised to unity gain at its reference frequency, this behaves like the
  // mix fraction it is named after: it adds edge and bite without deciding the
  // whole spectral balance.
  dfFMix: 0.10,
  airNoise: 0.5,
  airNoiseCutoffHz: 2000,
  jitter: 0.06,

  /**
   * Output tone control: a gentle roll-off above this.
   *
   * Physically justified, not a sticking plaster. Radiation from an open pipe
   * falls away at high frequency, bodywork and a helmet absorb it, and air
   * absorption removes more over distance. Without it the sharp edge of each
   * blowdown pulse survives all the way to the speaker with nothing between.
   */
  toneCutoffHz: 3200,

  /**
   * Pascals that map to full scale.
   *
   * A FIXED reference, deliberately, and this replaced an automatic gain
   * control. The AGC was doing exactly what it was asked -- driving every
   * operating point to the same output RMS -- and that is the wrong goal for an
   * engine. It erased the difference between idle and the limiter, leaving a
   * measured 3.2 dB of A-weighted range where a real engine spans 25-35 dB, and
   * it did it while boosting the quiet, ring-dominated idle signal by four
   * times, which brought the high-frequency noise floor up with it.
   *
   * With a fixed reference, loudness is whatever the physics produced, times
   * the level trim below. Roughly one atmosphere is a natural scale for
   * exhaust pressure and keeps a healthy peak around 0.2-0.3.
   */
  pressureRefPa: 90000,

  /**
   * Range of the resonance compressor, dB.
   *
   * The waveguide's output swings about 12 dB across the rev range purely from
   * which pipe modes the firing harmonics happen to land on. Tuned-length
   * resonance is real and worth hearing, but 12 dB of it is far more than a
   * real exhaust shows and it swamped the intended loudness curve -- 10000 rpm
   * came out louder than the limiter.
   *
   * This is a slow, hard-limited automatic gain that removes that variation and
   * nothing else. Limiting the range is what distinguishes it from the AGC it
   * replaced: at +/-8 dB it cannot flatten a 25 dB loudness curve, so the level
   * trim still decides how loud each operating point is. Set to 0 to hear the
   * pipe resonance raw.
   */
  resonanceCompressDb: 8,

  /** Target RMS for the resonance compressor, before the level trim. */
  compressorTarget: 0.09,

  /**
   * Loudness floor, as a fraction of full scale, for an engine making no power.
   *
   * Not zero: an engine on the overrun still pumps air and is audible. It is
   * just much quieter than one pulling.
   */
  levelFloor: 0.025,

  /**
   * Exponent mapping combustion power to loudness.
   *
   * Level tracks the chemical power the engine is actually releasing -- heat
   * release per cycle times firing rate -- because that is what sets how hard
   * an exhaust is driven, and it falls to nothing on a closed throttle without
   * any special case. Sound pressure goes roughly as the square root of
   * acoustic power, and only a fraction of combustion power becomes sound, so
   * the exponent is well below 1. 0.55 measures out at 20 dB from idle to the
   * limiter and 26 dB from a closed-throttle overrun, which is the right
   * ballpark for a car you can stand beside at idle and cannot at full
   * throttle.
   */
  levelExponent: 0.55,
};

export class Synthesizer {
  constructor(channelCount, sampleRate, ir, params) {
    this.sampleRate = sampleRate;
    this.params = { ...params };
    this.channels = [];
    for (let i = 0; i < channelCount; i++) {
      this.channels.push({
        jitter: new JitterFilter(this.params.jitter, sampleRate, 0xc0ffee + i),
        // 10 Hz, matching engine-sim's DC filter cutoff.
        dc: new LowPassFilter(10, sampleRate),
        derivative: new DerivativeFilter(sampleRate),
        airNoiseLp: new ButterworthLowPass(this.params.airNoiseCutoffHz, sampleRate),
        convolution: new ConvolutionFilter(ir),
        rng: new Rng(0xbeef + i * 7919),
      });
    }
    // engine-sim antialiases at 45% of the sample rate.
    this.antialias = new ButterworthLowPass(sampleRate * 0.45, sampleRate);
    // Two poles of tone control, cascaded, for a 24 dB/octave roll-off. One
    // pole was not enough to stop the pipe ring dominating at low rpm.
    this.tone1 = new ButterworthLowPass(this.params.toneCutoffHz, sampleRate);
    this.tone2 = new ButterworthLowPass(this.params.toneCutoffHz, sampleRate);
    this.compressor = new LevelingFilter(this.params.compressorTarget, sampleRate);
    this.applyCompressorRange();
  }

  applyCompressorRange() {
    const db = Math.max(this.params.resonanceCompressDb, 0);
    const span = Math.pow(10, db / 20);
    this.compressor.target = this.params.compressorTarget;
    this.compressor.maxGain = span;
    this.compressor.minGain = 1 / span;
  }

  setParameters(p) {
    this.params = { ...this.params, ...p };
    for (const c of this.channels) {
      c.jitter.amount = this.params.jitter;
      c.airNoiseLp.setCutoff(this.params.airNoiseCutoffHz, this.sampleRate);
    }
    this.tone1.setCutoff(this.params.toneCutoffHz, this.sampleRate);
    this.tone2.setCutoff(this.params.toneCutoffHz, this.sampleRate);
    this.applyCompressorRange();
  }

  setImpulseResponse(ir) {
    for (const c of this.channels) c.convolution.setImpulseResponse(ir);
  }

  /**
   * One output sample from one pressure sample per channel.
   *
   * `load` scales the turbulent-air contribution -- an engine on the overrun
   * does not hiss like one at wide-open throttle. Note that the noise
   * *modulates* rather than adds, which is what keeps a stopped engine silent
   * however much noise is dialled in.
   */
  render(inputs, load, level = 1) {
    const p = this.params;
    let sum = 0;
    const l = Math.min(Math.max(load, 0), 1);

    for (let i = 0; i < this.channels.length; i++) {
      const ch = this.channels[i];
      const raw = inputs[i] || 0;

      const fIn = ch.jitter.f(raw);
      const fDc = ch.dc.f(fIn);
      const f = fIn - fDc;
      const fP = ch.derivative.f(fIn);

      const noise = ch.airNoiseLp.f(ch.rng.uniform());
      const rMixed = 1 + noise * p.airNoise * l;

      const vIn = fP * p.dfFMix + f * rMixed * (1 - p.dfFMix);
      const conv = Math.min(Math.max(p.convolution, 0), 1);
      sum += conv > 0 ? conv * ch.convolution.f(vIn) + (1 - conv) * vIn : vIn;
    }

    let signal = this.tone2.f(this.tone1.f(sum));
    signal = this.antialias.f(signal);

    // Pascals to full scale by a fixed reference, then the resonance
    // compressor takes out the pipe-mode swing, and only then does `level`
    // decide how loud this operating point should be. The order matters:
    // compressing after the level trim would undo it.
    let out = signal / p.pressureRefPa;
    if (p.resonanceCompressDb > 0) out = this.compressor.f(out);
    return softClip(out * level * p.volume);
  }
}

// ---------------------------------------------------------------------------
// The whole thing
// ---------------------------------------------------------------------------

export const DEFAULT_AUDIO_CONFIG = {
  sampleRate: 48000,
  // 256 taps at 48 kHz covers the whole early-reflection structure the impulse
  // response exists to provide (the cockpit response's last discrete
  // reflection lands at 4.8 ms, or 230 samples). engine-sim allows far longer
  // responses; the audio worklet is the binding constraint here.
  irTaps: 256,
  cabin: "cockpit",
  seed: 0x5dae2026,
};

export class EngineAudio {
  constructor(spec, config = {}) {
    const err = validateSpec(spec);
    if (err) throw new Error(`engine spec: ${err}`);

    this.spec = spec;
    this.config = { ...DEFAULT_AUDIO_CONFIG, ...config };
    const cfg = this.config;

    const ir = buildImpulseResponse(cfg.cabin, cfg.irTaps, cfg.sampleRate, cfg.seed);
    this.synth = new Synthesizer(
      spec.tailpipes.length,
      cfg.sampleRate,
      ir,
      DEFAULT_AUDIO_PARAMETERS,
    );
    this.exhaust = new ExhaustNetwork(spec, cfg.sampleRate);
    this.tables = new CycleTables(spec);
    this.cylinders = spec.firingAnglesDeg.map((phaseDeg) => ({
      phaseDeg,
      pressurePa: spec.gas.ambientPa,
      temperatureK: spec.gas.ambientK,
      exhaustFlow: 0,
      // Gas velocity in the exhaust port. State, because the port has
      // inertance -- see stepCylinder.
      portVelocity: 0,
    }));

    this.crankDeg = 0;
    this.running = true;
    /** Output trim for the current operating point; see setOperatingPoint. */
    this.level = 0.3;
    this.op = {
      rpm: spec.idleRpm,
      throttle: 0,
      targetTorqueNm: 0,
      exhaustK: spec.gas.exhaustKMin,
    };
    this.cal = calibrate(spec, this.op);

    this.flow = new Float32Array(this.cylinders.length);
    this.valveArea = new Float32Array(this.cylinders.length);
    this.referencePowerW = this.computeReferencePower();
  }

  setParameters(p) {
    this.synth.setParameters(p);
  }

  setCabin(cabin) {
    this.config.cabin = cabin;
    this.synth.setImpulseResponse(
      buildImpulseResponse(cabin, this.config.irTaps, this.config.sampleRate, this.config.seed),
    );
  }

  setRunning(running) {
    this.running = running;
  }

  /**
   * Move to a new operating point. Call at the physics rate, not per sample:
   * it re-solves the heat release and may retune every delay line.
   */
  setOperatingPoint(rpm, throttle, torqueNm) {
    const th = Math.min(Math.max(throttle, 0), 1);
    const load = Math.min(Math.max(torqueNm, 0) / 70, 1) * 0.6 + th * 0.4;
    const gas = this.spec.gas;
    const exhaustK = gas.exhaustKMin + (gas.exhaustKMax - gas.exhaustKMin) * load;
    const retune = Math.abs(exhaustK - this.op.exhaustK) > 5;

    this.op = { rpm: Math.max(rpm, 0), throttle: th, targetTorqueNm: torqueNm, exhaustK };
    this.cal = calibrate(this.spec, this.op);
    if (retune) this.exhaust.setGasState(gas, exhaustK);

    // How loud this operating point should be, from the chemical power the
    // engine is actually releasing.
    //
    // Heat release per cylinder per cycle times the firing rate is the fuel
    // power going in, and that is what drives the exhaust. Using it rather than
    // a hand-blended mix of throttle and rpm means the overrun falls away on
    // its own -- no combustion, no power, no noise beyond the floor -- and
    // every intermediate point lands where the physics puts it.
    const p = this.synth.params;
    const firingPerSecond = (this.op.rpm / 120) * this.cylinders.length;
    const chemicalPowerW = this.cal.heatReleaseJ * firingPerSecond;
    const rel = Math.min(chemicalPowerW / this.referencePowerW, 1);
    this.level = p.levelFloor + (1 - p.levelFloor) * Math.pow(rel, p.levelExponent);
  }

  /**
   * Combustion power at the limiter on full throttle, W.
   *
   * The reference the level curve is measured against. Computed once from the
   * engine's own spec rather than hard-coded, so a different engine scales
   * itself instead of coming out silent or clipped.
   */
  computeReferencePower() {
    const op = {
      rpm: this.spec.redlineRpm,
      throttle: 1,
      // Peak torque is not in the spec, so use an indicated mean effective
      // pressure of 10 bar -- a normal figure for a naturally aspirated engine
      // at full load, and only a scale factor here.
      targetTorqueNm: (10e5 * displacementM3(this.spec)) / (4 * Math.PI),
      exhaustK: this.spec.gas.exhaustKMax,
    };
    const cal = calibrate(this.spec, op);
    return cal.heatReleaseJ * ((this.spec.redlineRpm / 120) * this.cylinders.length);
  }

  /** Fill `out` (a Float32Array) with mono samples in [-1, 1]. */
  render(out) {
    const dt = 1 / this.config.sampleRate;
    const degPerSample = this.op.rpm * 6 * dt;
    const ambient = this.spec.gas.ambientPa;
    const load = this.op.throttle;

    for (let s = 0; s < out.length; s++) {
      for (let i = 0; i < this.cylinders.length; i++) {
        const cyl = this.cylinders[i];
        let theta = this.crankDeg - cyl.phaseDeg;
        while (theta < 0) theta += 720;
        while (theta >= 720) theta -= 720;

        const area = this.tables.valveAreaAt(theta);
        this.valveArea[i] = area;
        // The port velocity from the previous sample closes the impedance loop.
        // Strictly this is implicit -- the pressure depends on the flow and the
        // flow on the pressure -- and using the previous value is one Gauss-
        // Seidel step. The port inertance already limits how fast the velocity
        // can move, so the lag is smaller than the physical time constant and
        // the loop is stable.
        const back = this.exhaust.portPressure(i, ambient, area, cyl.portVelocity);
        this.flow[i] = this.running
          ? stepCylinder(this.spec, this.tables, cyl, this.crankDeg, dt, this.cal, this.op, back)
          : 0;
      }

      this.exhaust.step(this.flow, this.valveArea);
      out[s] = this.synth.render(this.exhaust.outputs, load, this.level);

      this.crankDeg += degPerSample;
      if (this.crankDeg >= 720) this.crankDeg -= 720;
    }
  }

  reset() {
    this.exhaust.reset();
    this.crankDeg = 0;
    for (const c of this.cylinders) {
      c.pressurePa = this.spec.gas.ambientPa;
      c.temperatureK = this.spec.gas.ambientK;
      c.exhaustFlow = 0;
      c.portVelocity = 0;
    }
  }
}
