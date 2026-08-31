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
    /// Target output RMS. The leveller aims the average level here.
    pub leveler_target: f32,
    pub leveler_max_gain: f32,
    pub leveler_min_gain: f32,
    /// How much quieter the engine gets when it is doing no work.
    ///
    /// A real engine at idle is far quieter than one at wide-open throttle, and
    /// levelling everything to the same loudness is both wrong and unpleasant:
    /// it takes the weak, ring-dominated output of an overrun and amplifies it
    /// until the ringing is all you can hear. 0 normalises everything; 1 makes
    /// idle silent.
    pub load_level_depth: f32,
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
            leveler_target: 0.14,
            leveler_max_gain: 4.0,
            leveler_min_gain: 1e-5,
            load_level_depth: 0.72,
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
    leveler: LevelingFilter,
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

        let mut leveler = LevelingFilter::new(params.leveler_target, sample_rate);
        leveler.max_gain = params.leveler_max_gain;
        leveler.min_gain = params.leveler_min_gain;

        Self {
            channels,
            tone1: ButterworthLowPass::new(params.tone_cutoff_hz, sample_rate),
            tone2: ButterworthLowPass::new(params.tone_cutoff_hz, sample_rate),
            // engine-sim antialiases at 45% of the sample rate.
            antialias: ButterworthLowPass::new(sample_rate * 0.45, sample_rate),
            leveler,
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
        self.leveler.target = p.leveler_target;
        self.leveler.max_gain = p.leveler_max_gain;
        self.leveler.min_gain = p.leveler_min_gain;
        self.params = p;
    }

    pub fn set_impulse_response(&mut self, ir: Vec<f32>) {
        for c in &mut self.channels {
            c.convolution.set_impulse_response(ir.clone());
        }
    }

    pub fn leveler_gain(&self) -> f32 {
        self.leveler.gain()
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

        // Tone, then antialias, then level. Rolling off before the leveller
        // means the gain is set by what will actually be heard rather than by
        // ringing that is about to be filtered away.
        let mut signal = self.tone2.f(self.tone1.f(sum));
        signal = self.antialias.f(signal);
        soft_clip(self.leveler.f(signal) * level * p.volume)
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
        p.leveler_target = 1.0;
        p.leveler_max_gain = 1.0;
        p.leveler_min_gain = 1.0;
        p.tone_cutoff_hz = 20_000.0;
        s.set_parameters(p);
        let mut last = 1.0;
        for _ in 0..48_000 * 3 {
            last = s.render(&[1.0], 0.0, 1.0);
        }
        assert!(last.abs() < 0.01, "constant input left {last} at the output");
    }

    #[test]
    fn levelling_pulls_a_loud_source_back() {
        let mut quiet = synth(1);
        let mut loud = synth(1);
        let mut peak_q = 0.0f32;
        let mut peak_l = 0.0f32;
        for i in 0..48_000 {
            let phase = i as f32 * 0.02;
            peak_q = peak_q.max(quiet.render(&[phase.sin() * 1.0], 1.0, 1.0).abs());
            peak_l = peak_l.max(loud.render(&[phase.sin() * 1_000.0], 1.0, 1.0).abs());
        }
        // A thousand times the input must not give a thousand times the output.
        assert!(peak_l < peak_q * 20.0, "{peak_q} vs {peak_l}");
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
