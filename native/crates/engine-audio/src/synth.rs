//! The synthesiser stage.
//!
//! This is the closest adaptation of engine-sim in the crate: it is the filter
//! chain from `Synthesizer::renderAudio`, in the same order, with the same
//! parameter names. The physical justification for each stage:
//!
//! 1. **Jitter** -- no two combustion cycles are identical. Without this the
//!    output is periodic to the sample and sounds synthetic at steady rpm.
//! 2. **DC removal** -- the manifold sits at a mean pressure that carries no
//!    sound. Subtracting it is what stops the leveller chasing a bias.
//! 3. **Derivative** -- radiated sound goes as the *rate of change* of the
//!    flow leaving the pipe, not as the pressure in it.
//! 4. **Air noise** -- induction and turbulent flow noise, band-limited and
//!    used to modulate rather than to add, so it tracks engine load.
//! 5. **Convolution** -- puts the dry pipe output into a space.
//! 6. **Anti-aliasing** -- the waveguide and the derivative both produce
//!    energy near Nyquist that will fold back audibly.
//! 7. **Levelling** -- output level otherwise swings by orders of magnitude
//!    between idle and the limiter.

use crate::filters::{
    soft_clip, ButterworthLowPass, ConvolutionFilter, DerivativeFilter, JitterFilter,
    LevelingFilter, LowPassFilter, Rng,
};

/// Mixing and shaping controls, named after engine-sim's `AudioParameters`.
#[derive(Clone, Copy, Debug)]
pub struct AudioParameters {
    pub volume: f32,
    /// How much of the convolved signal to use, 0..1.
    pub convolution: f32,
    /// Blend between the differentiated signal and the raw one. Now that the
    /// derivative is normalised to unity gain at its reference frequency, this
    /// behaves like the mix fraction it is named after: it adds edge and bite
    /// without deciding the whole spectral balance.
    pub df_f_mix: f32,
    /// Depth of the turbulent-air modulation, 0..1.
    pub air_noise: f32,
    pub air_noise_cutoff_hz: f32,
    /// Cycle-to-cycle variation, 0..1.
    pub jitter: f32,
    /// Output tone control: a gentle roll-off above this.
    ///
    /// Physically justified rather than a sticking plaster. Radiation from an
    /// open pipe falls away at high frequency, bodywork and a helmet absorb it,
    /// and air absorption removes more over distance.
    pub tone_cutoff_hz: f32,
    /// Pascals that map to full scale.
    ///
    /// A FIXED reference, deliberately, and this replaced an automatic gain
    /// control. The AGC was doing exactly what it was asked -- driving every
    /// operating point to the same output RMS -- and that is the wrong goal for
    /// an engine. It erased the difference between idle and the limiter,
    /// leaving 3.2 dB of A-weighted range where a real engine spans 25-35 dB,
    /// while boosting the quiet, ring-dominated idle signal fourfold and
    /// bringing its high-frequency noise floor up with it.
    pub pressure_ref_pa: f32,

    /// Range of the resonance compressor, dB.
    ///
    /// The waveguide's output swings about 12 dB across the rev range purely
    /// from which pipe modes the firing harmonics land on. Tuned-length
    /// resonance is real and worth hearing, but 12 dB of it is far more than a
    /// real exhaust shows, and it swamped the loudness curve -- 10000 rpm came
    /// out louder than the limiter. Limiting the range is what distinguishes
    /// this from the AGC it replaced: at +/-8 dB it cannot flatten a 20 dB
    /// loudness curve. Set to 0 to hear the pipe resonance raw.
    pub resonance_compress_db: f32,

    /// Target RMS for the resonance compressor, before the level trim.
    pub compressor_target: f32,

    /// Loudness floor, as a fraction of full scale, for an engine making no
    /// power. Not zero: an engine on the overrun still pumps air.
    pub level_floor: f32,

    /// Exponent mapping combustion power to loudness.
    ///
    /// Level tracks the chemical power the engine is releasing -- heat release
    /// per cycle times firing rate -- because that is what drives an exhaust,
    /// and it falls away on a closed throttle without any special case. Sound
    /// pressure goes roughly as the square root of acoustic power and only a
    /// fraction of combustion power becomes sound, so the exponent is well
    /// below 1. 0.55 measures out at 20 dB idle to limiter and 26 dB from a
    /// closed-throttle overrun.
    pub level_exponent: f32,
}

impl Default for AudioParameters {
    fn default() -> Self {
        Self {
            volume: 1.0,
            convolution: 1.0,
            df_f_mix: 0.10,
            air_noise: 0.5,
            air_noise_cutoff_hz: 2_000.0,
            jitter: 0.06,
            tone_cutoff_hz: 3_200.0,
            pressure_ref_pa: 90_000.0,
            resonance_compress_db: 8.0,
            compressor_target: 0.09,
            level_floor: 0.025,
            level_exponent: 0.55,
        }
    }
}

/// Per-input-channel filter state. One channel per tailpipe.
struct ChannelFilters {
    jitter: JitterFilter,
    dc: LowPassFilter,
    derivative: DerivativeFilter,
    air_noise_lp: ButterworthLowPass,
    convolution: ConvolutionFilter,
    rng: Rng,
}

pub struct Synthesizer {
    channels: Vec<ChannelFilters>,
    // Two poles of tone control, cascaded, for a 24 dB/octave roll-off. One
    // pole was not enough to stop the pipe ring dominating at low rpm.
    tone1: ButterworthLowPass,
    tone2: ButterworthLowPass,
    antialias: ButterworthLowPass,
    compressor: LevelingFilter,
    params: AudioParameters,
    sample_rate: f32,
}

impl Synthesizer {
    pub fn new(channel_count: usize, sample_rate: f32, ir: Vec<f32>, params: AudioParameters) -> Self {
        let channels = (0..channel_count)
            .map(|i| ChannelFilters {
                jitter: JitterFilter::new(params.jitter, sample_rate, 0xC0FFEE + i as u64),
                // 10 Hz, matching engine-sim's DC filter cutoff.
                dc: LowPassFilter::new(10.0, sample_rate),
                derivative: DerivativeFilter::new(sample_rate),
                air_noise_lp: ButterworthLowPass::new(params.air_noise_cutoff_hz, sample_rate),
                convolution: ConvolutionFilter::new(ir.clone()),
                rng: Rng::new(0xBEEF + i as u64 * 7919),
            })
            .collect();

        let mut compressor = LevelingFilter::new(params.compressor_target, sample_rate);
        let span = 10f32.powf(params.resonance_compress_db.max(0.0) / 20.0);
        compressor.max_gain = span;
        compressor.min_gain = 1.0 / span;

        Self {
            channels,
            tone1: ButterworthLowPass::new(params.tone_cutoff_hz, sample_rate),
            tone2: ButterworthLowPass::new(params.tone_cutoff_hz, sample_rate),
            // engine-sim antialiases at 45% of the sample rate.
            antialias: ButterworthLowPass::new(sample_rate * 0.45, sample_rate),
            compressor,
            params,
            sample_rate,
        }
    }

    pub fn parameters(&self) -> &AudioParameters {
        &self.params
    }

    pub fn set_parameters(&mut self, p: AudioParameters) {
        for c in &mut self.channels {
            c.jitter.set_amount(p.jitter);
            c.air_noise_lp.set_cutoff(p.air_noise_cutoff_hz, self.sample_rate);
        }
        self.tone1.set_cutoff(p.tone_cutoff_hz, self.sample_rate);
        self.tone2.set_cutoff(p.tone_cutoff_hz, self.sample_rate);
        let span = 10f32.powf(p.resonance_compress_db.max(0.0) / 20.0);
        self.compressor.target = p.compressor_target;
        self.compressor.max_gain = span;
        self.compressor.min_gain = 1.0 / span;
        self.params = p;
    }

    pub fn set_impulse_response(&mut self, ir: Vec<f32>) {
        for c in &mut self.channels {
            c.convolution.set_impulse_response(ir.clone());
        }
    }

    pub fn compressor_gain(&self) -> f32 {
        self.compressor.gain()
    }

    /// Render one output sample from one pressure sample per channel.
    ///
    /// `load` (0..1) scales the turbulent-air contribution: an engine on the
    /// overrun does not hiss like one at wide-open throttle. `level` scales the
    /// output with how hard the engine is working, so idle stays quiet.
    pub fn render(&mut self, inputs: &[f32], load: f32, level: f32) -> f32 {
        let p = self.params;
        let mut sum = 0.0f32;

        for (i, ch) in self.channels.iter_mut().enumerate() {
            let raw = inputs.get(i).copied().unwrap_or(0.0);

            let f_in = ch.jitter.f(raw);
            let f_dc = ch.dc.f(f_in);
            let f = f_in - f_dc;
            let f_p = ch.derivative.f(f_in);

            let noise = ch.air_noise_lp.f(ch.rng.uniform());
            let r_mixed = 1.0 + noise * p.air_noise * load.clamp(0.0, 1.0);

            let v_in = f_p * p.df_f_mix + f * r_mixed * (1.0 - p.df_f_mix);

            let conv = p.convolution.clamp(0.0, 1.0);
            let v = if conv > 0.0 {
                conv * ch.convolution.f(v_in) + (1.0 - conv) * v_in
            } else {
                v_in
            };
            sum += v;
        }

        // Tone, then antialias, then a fixed pressure reference, then the
        // resonance compressor, and only then `level`. The order matters:
        // compressing after the level trim would undo it.
        let mut signal = self.tone2.f(self.tone1.f(sum));
        signal = self.antialias.f(signal) / p.pressure_ref_pa;
        if p.resonance_compress_db > 0.0 {
            signal = self.compressor.f(signal);
        }
        soft_clip(signal * level * p.volume)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synth(channels: usize) -> Synthesizer {
        Synthesizer::new(channels, 48_000.0, vec![1.0], AudioParameters::default())
    }

    #[test]
    fn silence_in_silence_out() {
        let mut s = synth(1);
        // Air noise modulates rather than adds, so a zero input stays zero
        // however much noise is dialled in. That is the property that keeps a
        // stopped engine actually silent.
        for _ in 0..10_000 {
            assert_eq!(s.render(&[0.0], 1.0, 1.0), 0.0);
        }
    }

    #[test]
    fn output_is_bounded() {
        let mut s = synth(1);
        let mut rng = Rng::new(11);
        for _ in 0..200_000 {
            let y = s.render(&[rng.uniform() * 5_000.0], 1.0, 1.0);
            assert!((-1.0..=1.0).contains(&y), "escaped the clamp: {y}");
            assert!(y.is_finite());
        }
    }

    #[test]
    fn dc_is_removed() {
        let mut s = synth(1);
        let mut p = AudioParameters::default();
        // Isolate the DC path: no noise, no convolution, no derivative blend.
        p.air_noise = 0.0;
        p.jitter = 0.0;
        p.convolution = 0.0;
        p.df_f_mix = 0.0;
        p.resonance_compress_db = 0.0;
        p.pressure_ref_pa = 1.0;
        p.tone_cutoff_hz = 20_000.0;
        s.set_parameters(p);
        let mut last = 1.0;
        for _ in 0..48_000 * 3 {
            last = s.render(&[1.0], 0.0, 1.0);
        }
        assert!(last.abs() < 0.01, "constant input left {last} at the output");
    }

    #[test]
    fn the_compressor_cannot_flatten_the_loudness_curve() {
        // The property that distinguishes this from the automatic gain control
        // it replaced. The compressor exists to take out the ~12 dB of swing
        // the waveguide shows across the rev range purely from which pipe modes
        // the firing harmonics land on. It must NOT be able to erase the
        // difference between idle and the limiter -- that was the old AGC's
        // failure, and it made idle as loud as full throttle.
        //
        // Two sources 20 dB apart, each compressed independently: with a range
        // of +/-8 dB the most it can close is 16 dB, so at least 4 dB has to
        // survive.
        let rms_of = |amplitude: f32| {
            let mut s = synth(1);
            let mut p = AudioParameters::default();
            p.pressure_ref_pa = 1.0;
            p.air_noise = 0.0;
            p.jitter = 0.0;
            p.convolution = 0.0;
            s.set_parameters(p);
            let mut sum = 0.0f32;
            let mut n = 0u32;
            for i in 0..48_000 * 2 {
                let y = s.render(&[amplitude * (i as f32 * 0.05).sin()], 0.0, 1.0);
                if i > 48_000 {
                    sum += y * y;
                    n += 1;
                }
            }
            (sum / n as f32).sqrt()
        };

        let quiet = rms_of(0.01);
        let loud = rms_of(0.1); // 20 dB louder
        let survived = 20.0 * (loud / quiet.max(1e-12)).log10();
        assert!(
            survived > 3.0,
            "a 20 dB input difference collapsed to {survived:.1} dB"
        );
    }

    #[test]
    fn the_compressor_does_reduce_a_resonance_swing() {
        // The other half: it has to actually do its job. The same 20 dB
        // difference must come out smaller than it went in.
        let rms_of = |amplitude: f32| {
            let mut s = synth(1);
            let mut p = AudioParameters::default();
            p.pressure_ref_pa = 1.0;
            p.air_noise = 0.0;
            p.jitter = 0.0;
            p.convolution = 0.0;
            s.set_parameters(p);
            let mut sum = 0.0f32;
            let mut n = 0u32;
            for i in 0..48_000 * 2 {
                let y = s.render(&[amplitude * (i as f32 * 0.05).sin()], 0.0, 1.0);
                if i > 48_000 {
                    sum += y * y;
                    n += 1;
                }
            }
            (sum / n as f32).sqrt()
        };
        let survived = 20.0 * (rms_of(0.1) / rms_of(0.01).max(1e-12)).log10();
        assert!(survived < 20.0, "nothing was compressed: {survived:.1} dB");
    }

    #[test]
    fn channels_sum() {
        let mut one = synth(1);
        let mut two = synth(2);
        // Two identical channels should be louder than one before levelling
        // settles. Check the first sample, before the leveller has moved.
        let a = one.render(&[1.0], 0.0, 1.0);
        let b = two.render(&[1.0, 1.0], 0.0, 1.0);
        assert!(b.abs() >= a.abs());
    }
}
