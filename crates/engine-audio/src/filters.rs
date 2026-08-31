//! DSP primitives for the engine synthesiser.
//!
//! These are adapted from `ange-yaghi/engine-sim` (MIT), specifically the
//! filter set that `Synthesizer::renderAudio` runs each input sample through.
//! The names are kept deliberately close to the originals so the two can be
//! read side by side.

use core::f32::consts::PI;

/// One-pole low pass. Used two ways in the chain: to isolate the DC component
/// of the manifold pressure so it can be subtracted, and to band-limit noise.
#[derive(Clone, Debug)]
pub struct LowPassFilter {
    alpha: f32,
    y: f32,
}

impl LowPassFilter {
    pub fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        let mut f = Self { alpha: 0.0, y: 0.0 };
        f.set_cutoff(cutoff_hz, sample_rate);
        f
    }

    pub fn set_cutoff(&mut self, cutoff_hz: f32, sample_rate: f32) {
        let rc = 1.0 / (2.0 * PI * cutoff_hz.max(1e-3));
        let dt = 1.0 / sample_rate;
        self.alpha = dt / (rc + dt);
    }

    #[inline]
    pub fn f(&mut self, x: f32) -> f32 {
        self.y += self.alpha * (x - self.y);
        self.y
    }

    pub fn reset(&mut self) {
        self.y = 0.0;
    }
}

/// Second-order Butterworth low pass, transposed direct form II.
///
/// The chain uses this twice: to shape the air-noise channel, and as the
/// anti-aliasing stage on the summed signal before it leaves the synthesiser.
#[derive(Clone, Debug)]
pub struct ButterworthLowPass {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

impl ButterworthLowPass {
    pub fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        let mut f = Self {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            z1: 0.0,
            z2: 0.0,
        };
        f.set_cutoff(cutoff_hz, sample_rate);
        f
    }

    pub fn set_cutoff(&mut self, cutoff_hz: f32, sample_rate: f32) {
        // Clamp below Nyquist: a cutoff at or above it makes the prewarp blow up.
        let fc = cutoff_hz.clamp(1.0, sample_rate * 0.49);
        let k = (PI * fc / sample_rate).tan();
        let k2 = k * k;
        let norm = 1.0 / (1.0 + core::f32::consts::SQRT_2 * k + k2);
        self.b0 = k2 * norm;
        self.b1 = 2.0 * self.b0;
        self.b2 = self.b0;
        self.a1 = 2.0 * (k2 - 1.0) * norm;
        self.a2 = (1.0 - core::f32::consts::SQRT_2 * k + k2) * norm;
    }

    #[inline]
    pub fn f(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }

    pub fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }
}

/// Discrete derivative, scaled by the sample rate.
///
/// This is the single most important filter in the chain and the reason the
/// output sounds like an exhaust rather than like a pressure gauge. Sound
/// radiated from an open pipe is proportional to the *rate of change* of the
/// volume velocity leaving it, not to the pressure inside it. Feed the raw
/// manifold pressure to a speaker and you get a muffled thump; differentiate
/// it first and the sharp edge of each blowdown pulse comes back.
#[derive(Clone, Debug)]
pub struct DerivativeFilter {
    prev: f32,
    dt: f32,
}

impl DerivativeFilter {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            prev: 0.0,
            dt: 1.0 / sample_rate,
        }
    }

    #[inline]
    pub fn f(&mut self, x: f32) -> f32 {
        let d = (x - self.prev) / self.dt;
        self.prev = x;
        d
    }

    pub fn reset(&mut self) {
        self.prev = 0.0;
    }
}

/// Cycle-to-cycle jitter.
///
/// A real engine never fires twice identically -- charge motion, residual gas
/// and injector scatter all vary. Without this the synthesiser produces a tone
/// so periodic it sounds synthetic, especially at steady rpm. The original
/// perturbs the read offset into the input buffer; this perturbs the amplitude
/// of each sample by a slow random walk, which is cheaper and does the same
/// perceptual job.
#[derive(Clone, Debug)]
pub struct JitterFilter {
    amount: f32,
    lp: LowPassFilter,
    rng: Rng,
}

impl JitterFilter {
    pub fn new(amount: f32, sample_rate: f32, seed: u64) -> Self {
        Self {
            amount,
            lp: LowPassFilter::new(400.0, sample_rate),
            rng: Rng::new(seed),
        }
    }

    pub fn set_amount(&mut self, amount: f32) {
        self.amount = amount;
    }

    #[inline]
    pub fn f(&mut self, x: f32) -> f32 {
        if self.amount <= 0.0 {
            return x;
        }
        let n = self.lp.f(self.rng.uniform());
        x * (1.0 + self.amount * n)
    }
}

/// Automatic gain, targeting a peak level.
///
/// Directly adapted from engine-sim's `LevelingFilter`. Output level otherwise
/// swings by orders of magnitude between idle and the limiter, because the
/// blowdown pulse amplitude scales with cylinder pressure and firing rate. The
/// attack is fast and the release slow, so a single loud transient does not
/// duck the whole engine note.
#[derive(Clone, Debug)]
pub struct LevelingFilter {
    pub target: f32,
    pub min_gain: f32,
    pub max_gain: f32,
    peak: f32,
    attack: f32,
    release: f32,
}

impl LevelingFilter {
    pub fn new(target: f32, sample_rate: f32) -> Self {
        Self {
            target,
            min_gain: 1e-5,
            max_gain: 1.9,
            peak: target,
            // ~1 ms attack, ~250 ms release.
            attack: 1.0 - (-1.0 / (0.001 * sample_rate)).exp(),
            release: 1.0 - (-1.0 / (0.250 * sample_rate)).exp(),
        }
    }

    #[inline]
    pub fn f(&mut self, x: f32) -> f32 {
        let a = x.abs();
        if a > self.peak {
            self.peak += self.attack * (a - self.peak);
        } else {
            self.peak += self.release * (a - self.peak);
        }
        let gain = (self.target / self.peak.max(1e-9)).clamp(self.min_gain, self.max_gain);
        x * gain
    }

    pub fn gain(&self) -> f32 {
        (self.target / self.peak.max(1e-9)).clamp(self.min_gain, self.max_gain)
    }
}

/// Direct-form FIR convolution against a stored impulse response.
///
/// engine-sim allows impulse responses up to 10,000 samples and leans on C++
/// and SIMD to afford it. This runs in a browser audio worklet as well as
/// natively, so the default is 512 taps (about 11 ms at 48 kHz) -- long enough
/// for the body and cabin colouration that the IR is actually there to provide,
/// short enough that the cost is 25 MMAC/s rather than 480.
#[derive(Clone, Debug)]
pub struct ConvolutionFilter {
    ir: Vec<f32>,
    history: Vec<f32>,
    cursor: usize,
}

impl ConvolutionFilter {
    pub fn new(ir: Vec<f32>) -> Self {
        let n = ir.len().max(1);
        Self {
            ir,
            history: vec![0.0; n],
            cursor: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.ir.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ir.is_empty()
    }

    pub fn set_impulse_response(&mut self, ir: Vec<f32>) {
        let n = ir.len().max(1);
        self.ir = ir;
        self.history = vec![0.0; n];
        self.cursor = 0;
    }

    #[inline]
    pub fn f(&mut self, x: f32) -> f32 {
        let n = self.history.len();
        if n == 0 || self.ir.is_empty() {
            return x;
        }
        self.history[self.cursor] = x;
        let mut acc = 0.0f32;
        let mut idx = self.cursor;
        for tap in &self.ir {
            acc += tap * self.history[idx];
            idx = if idx == 0 { n - 1 } else { idx - 1 };
        }
        self.cursor = (self.cursor + 1) % n;
        acc
    }

    pub fn reset(&mut self) {
        for h in &mut self.history {
            *h = 0.0;
        }
        self.cursor = 0;
    }
}

/// xorshift32, so the audio is reproducible across runs, platforms *and
/// languages*.
///
/// The 32-bit generator is chosen over a better 64-bit one for one reason:
/// JavaScript has no native 64-bit integer arithmetic outside `BigInt`, so a
/// `xorshift64*` port would either be slow or would have to draw different
/// noise. Different noise means the JS port cannot be compared against golden
/// samples at all -- the jitter and air-noise stages both feed the output, so
/// the waveforms would diverge from the first sample for a reason that tells
/// you nothing. 32 bits of state is ample for audio dither and it maps exactly
/// onto JavaScript's `|0` and `>>>` operators.
#[derive(Clone, Debug)]
pub struct Rng {
    state: u32,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        let s = (seed as u32) ^ ((seed >> 32) as u32);
        Self {
            state: if s == 0 { 0x9E37_79B9 } else { s },
        }
    }

    #[inline]
    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.state = x;
        x
    }

    /// Uniform in [-1, 1).
    #[inline]
    pub fn uniform(&mut self) -> f32 {
        // Top 24 bits: exactly representable in an f32 mantissa, and the same
        // arithmetic JavaScript does with `(x >>> 8) / 8388608 - 1`.
        let bits = self.next_u32() >> 8; // 24 bits
        (bits as f32 / 8_388_608.0) - 1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lowpass_settles_to_dc() {
        let mut f = LowPassFilter::new(10.0, 48_000.0);
        for _ in 0..48_000 {
            f.f(2.0);
        }
        assert!((f.f(2.0) - 2.0).abs() < 1e-3);
    }

    #[test]
    fn butterworth_passes_dc_and_kills_nyquist() {
        let mut f = ButterworthLowPass::new(1_000.0, 48_000.0);
        for _ in 0..5_000 {
            f.f(1.0);
        }
        assert!((f.f(1.0) - 1.0).abs() < 1e-3);

        let mut g = ButterworthLowPass::new(1_000.0, 48_000.0);
        let mut peak = 0.0f32;
        for i in 0..5_000 {
            let y = g.f(if i % 2 == 0 { 1.0 } else { -1.0 });
            if i > 1_000 {
                peak = peak.max(y.abs());
            }
        }
        assert!(peak < 0.01, "nyquist leaked through: {peak}");
    }

    #[test]
    fn derivative_of_a_ramp_is_its_slope() {
        let fs = 48_000.0;
        let mut d = DerivativeFilter::new(fs);
        d.f(0.0);
        let y = d.f(1.0 / fs); // ramp of 1.0 per second
        assert!((y - 1.0).abs() < 1e-3, "got {y}");
    }

    #[test]
    fn convolution_with_a_unit_impulse_is_identity() {
        let mut c = ConvolutionFilter::new(vec![1.0, 0.0, 0.0, 0.0]);
        assert_eq!(c.f(0.5), 0.5);
        assert_eq!(c.f(-0.25), -0.25);
    }

    #[test]
    fn convolution_delays_by_the_tap_index() {
        let mut c = ConvolutionFilter::new(vec![0.0, 0.0, 1.0]);
        assert_eq!(c.f(1.0), 0.0);
        assert_eq!(c.f(0.0), 0.0);
        assert_eq!(c.f(0.0), 1.0);
    }

    #[test]
    fn leveler_pulls_a_loud_signal_down_to_target() {
        let mut l = LevelingFilter::new(0.5, 48_000.0);
        let mut last = 0.0;
        for i in 0..48_000 {
            let x = 10.0 * (i as f32 * 0.05).sin();
            last = l.f(x);
        }
        // After a second of a 10x-too-loud sine the output must be bounded by
        // the target times the peak of the sine, not by the raw amplitude.
        assert!(last.abs() < 1.0, "leveler did not pull down: {last}");
    }

    #[test]
    fn rng_is_deterministic_and_in_range() {
        let mut a = Rng::new(12345);
        let mut b = Rng::new(12345);
        for _ in 0..1_000 {
            let x = a.uniform();
            assert_eq!(x, b.uniform());
            assert!((-1.0..1.0).contains(&x), "out of range: {x}");
        }
    }

    #[test]
    fn rng_matches_the_javascript_formulation() {
        // The exact sequence the JS port must reproduce. If this changes, the
        // golden vectors have to be regenerated and the JS port re-checked.
        let mut r = Rng::new(1);
        let got: Vec<u32> = (0..5).map(|_| r.next_u32()).collect();
        assert_eq!(got, vec![270369, 67634689, 2647435461, 307599695, 2398689233]);
    }

    #[test]
    fn rng_does_not_get_stuck_at_zero() {
        let mut r = Rng::new(0);
        let mut seen_nonzero = false;
        for _ in 0..100 {
            if r.next_u32() != 0 {
                seen_nonzero = true;
            }
        }
        assert!(seen_nonzero, "generator collapsed to zero");
    }
}
