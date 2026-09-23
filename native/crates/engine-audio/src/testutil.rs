//! Measurement helpers for the tests.
//!
//! Pitch detection here is not a nicety. Engine audio is deliberately rich:
//! the firing frequency at 6000 rpm is 200 Hz, but the exhaust rings hardest
//! around its own resonance several hundred hertz higher. Counting zero
//! crossings therefore measures the pipe, not the engine, and reports roughly
//! the same answer whatever the rpm -- which is exactly the wrong tool for
//! asserting that pitch tracks rpm.

/// Fundamental frequency by normalised autocorrelation.
///
/// Autocorrelation asks "at what lag does this waveform repeat", which is the
/// question we actually mean. It is immune to the spectral content sitting on
/// top of the repetition, so a 200 Hz pulse train exciting a 700 Hz resonance
/// still reads as 200 Hz.
pub fn fundamental_hz(buf: &[f32], fs: f32, min_hz: f32, max_hz: f32) -> f32 {
    let min_lag = (fs / max_hz).floor().max(2.0) as usize;
    let max_lag = (fs / min_hz).ceil() as usize;
    if buf.len() < max_lag * 2 + 2 || min_lag >= max_lag {
        return 0.0;
    }

    // Remove the mean; a DC offset biases every lag equally and flattens the
    // peak we are looking for.
    let mean = buf.iter().sum::<f32>() / buf.len() as f32;
    let x: Vec<f32> = buf.iter().map(|v| v - mean).collect();

    let n = x.len() - max_lag;
    let energy0: f32 = x[..n].iter().map(|v| v * v).sum();
    if energy0 < 1e-12 {
        return 0.0;
    }

    let mut corr = vec![0.0f32; max_lag + 2];
    let mut best_r = f32::NEG_INFINITY;
    for lag in min_lag..=max_lag {
        let mut acc = 0.0f32;
        let mut norm = 0.0f32;
        for i in 0..n {
            acc += x[i] * x[i + lag];
            norm += x[i + lag] * x[i + lag];
        }
        let r = acc / (energy0.sqrt() * norm.max(1e-12).sqrt());
        corr[lag] = r;
        if r > best_r {
            best_r = r;
        }
    }

    // Take the SHORTEST lag that is a local maximum and within 90% of the best
    // correlation.
    //
    // Both halves of that matter. Restricting to local maxima is what makes it
    // correct at all: autocorrelation of anything smooth rises gradually toward
    // its peak, so "the first lag above a threshold" lands part-way up the
    // slope and reports a frequency that is simply too high. Preferring the
    // shortest such peak is the octave guard -- a periodic signal correlates
    // almost as well at twice its period, and without this the answer halves at
    // random.
    let threshold = best_r * 0.90;
    let mut chosen = None;
    for lag in (min_lag + 1)..max_lag {
        if corr[lag] >= threshold && corr[lag] > corr[lag - 1] && corr[lag] >= corr[lag + 1] {
            chosen = Some(lag);
            break;
        }
    }
    let lag = match chosen {
        Some(l) => l,
        None => return 0.0,
    };

    // Parabolic interpolation through the three samples around the peak, so the
    // answer is not quantised to whole samples. At 48 kHz a lag of 240 steps in
    // increments of nearly 1 Hz, which is coarse enough to fail a tight
    // assertion for no real reason.
    let (y0, y1, y2) = (corr[lag - 1], corr[lag], corr[lag + 1]);
    let denom = y0 - 2.0 * y1 + y2;
    let offset = if denom.abs() > 1e-12 {
        (0.5 * (y0 - y2) / denom).clamp(-0.5, 0.5)
    } else {
        0.0
    };

    fs / (lag as f32 + offset)
}

/// Peak of the magnitude spectrum, by direct evaluation at candidate bins.
///
/// Used where the resonance itself is the thing under test, rather than the
/// repetition rate.
pub fn spectral_peak_hz(buf: &[f32], fs: f32, min_hz: f32, max_hz: f32, bins: usize) -> f32 {
    let mean = buf.iter().sum::<f32>() / buf.len() as f32;
    let mut best = (min_hz, 0.0f32);
    for b in 0..bins {
        let f = min_hz + (max_hz - min_hz) * b as f32 / (bins.max(2) - 1) as f32;
        let w = 2.0 * core::f32::consts::PI * f / fs;
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for (i, &v) in buf.iter().enumerate() {
            let x = v - mean;
            let p = w * i as f32;
            re += x * p.cos();
            im += x * p.sin();
        }
        let mag = (re * re + im * im).sqrt();
        if mag > best.1 {
            best = (f, mag);
        }
    }
    best.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn autocorrelation_finds_a_sine() {
        let fs = 48_000.0;
        let f = 200.0;
        let buf: Vec<f32> = (0..24_000)
            .map(|i| (2.0 * core::f32::consts::PI * f * i as f32 / fs).sin())
            .collect();
        let got = fundamental_hz(&buf, fs, 80.0, 600.0);
        assert!((got - f).abs() / f < 0.02, "got {got}");
    }

    #[test]
    fn autocorrelation_finds_the_pulse_rate_not_the_ringing() {
        // The exact situation the engine produces: a 200 Hz pulse train where
        // each pulse is a burst of 900 Hz.
        let fs = 48_000.0;
        let period = (fs / 200.0) as usize;
        let buf: Vec<f32> = (0..24_000)
            .map(|i| {
                let phase = i % period;
                let env = (-(phase as f32) / 40.0).exp();
                // Phase resets each pulse. A continuous carrier at 900 Hz spans
                // 4.5 cycles in 240 samples, so it inverts every pulse and the
                // signal genuinely repeats at 100 Hz, not 200 -- a property of
                // the test signal, not of the detector.
                env * (2.0 * core::f32::consts::PI * 900.0 * phase as f32 / fs).sin()
            })
            .collect();
        let got = fundamental_hz(&buf, fs, 80.0, 600.0);
        assert!((got - 200.0).abs() / 200.0 < 0.05, "got {got}, expected 200");
    }

    #[test]
    fn spectral_peak_finds_a_sine() {
        let fs = 48_000.0;
        let buf: Vec<f32> = (0..8_192)
            .map(|i| (2.0 * core::f32::consts::PI * 640.0 * i as f32 / fs).sin())
            .collect();
        let got = spectral_peak_hz(&buf, fs, 100.0, 2_000.0, 200);
        assert!((got - 640.0).abs() < 20.0, "got {got}");
    }
}

#[cfg(test)]
mod perf {
    use crate::*;
    use std::time::Instant;

    /// The synthesiser has to keep ahead of the sound card, and in the web
    /// build it has to do it inside an audio worklet sharing a core with the
    /// renderer. This measures how much faster than real time it runs.
    #[test]
    fn renders_comfortably_faster_than_real_time() {
        let cfg = AudioConfig::default();
        let mut e = EngineAudio::new(cbr600rr_sdm26(), cfg).unwrap();
        e.set_operating_point(12_000.0, 1.0, 60.0);

        let seconds = 5.0;
        let n = (cfg.sample_rate * seconds) as usize;
        let mut buf = vec![0.0f32; n];

        // Best of five, not one run.
        //
        // One timed run measures the MACHINE, not the code: a laptop that is
        // also compiling, or a CI box sharing a core, renders the same audio
        // several times slower and the test fails for a reason that has
        // nothing to do with the synthesiser. This one did exactly that --
        // 13.5x on a busy machine, 35x on the same machine a second later --
        // and a perf guard that cries wolf gets muted, which is worse than not
        // having it. The fastest run is the one the scheduler stayed out of,
        // so it is the one that says how fast the code is.
        let mut best = 0.0f32;
        for _ in 0..5 {
            let t0 = Instant::now();
            e.render(&mut buf);
            let factor = seconds / t0.elapsed().as_secs_f32();
            if factor > best {
                best = factor;
            }
        }
        println!("rendered {seconds} s of audio at {best:.0}x real time (best of 5)");
        // The bar is deliberately well above 1x. An audio callback that only
        // just keeps up drops out the moment anything else contends for the
        // core, and in the web build this shares a machine with the renderer.
        // The JS port runs several times slower than this, so headroom here is
        // headroom there.
        assert!(
            best > 15.0,
            "only {best:.1}x real time at best; too close to the edge for an audio thread"
        );
    }
}

#[cfg(test)]
mod spectrum_probe {
    use crate::*;

    fn mag_at(buf: &[f32], fs: f32, f: f32) -> f32 {
        let mean = buf.iter().sum::<f32>() / buf.len() as f32;
        let w = 2.0 * core::f32::consts::PI * f / fs;
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for (i, &v) in buf.iter().enumerate() {
            let x = v - mean;
            re += x * (w * i as f32).cos();
            im += x * (w * i as f32).sin();
        }
        (re * re + im * im).sqrt() / buf.len() as f32
    }

    /// The strongest statement available about whether the engine is modelled
    /// correctly: an evenly-firing four must put its energy on multiples of the
    /// firing frequency and essentially nothing anywhere else.
    ///
    /// This replaced an autocorrelation check that was passing for a weaker
    /// reason. Correlation at the firing lag is actually *negative* here,
    /// because a loud several-kilohertz resonance shifted by 240 samples is not
    /// a whole number of its own periods and drags the total down. The spectrum
    /// is not fooled by that.
    #[test]
    fn energy_lands_on_firing_harmonics_and_nowhere_else() {
        let fs = 48_000.0;
        for &rpm in &[6_000.0f32, 12_000.0] {
            let mut e = EngineAudio::new(cbr600rr_sdm26(), AudioConfig::default()).unwrap();
            e.set_operating_point(rpm, 1.0, 55.0);
            let mut warm = vec![0.0f32; 48_000];
            e.render(&mut warm);
            let mut buf = vec![0.0f32; 24_000];
            e.render(&mut buf);

            let f_fire = rpm / 30.0;
            let fundamental = mag_at(&buf, fs, f_fire);
            assert!(
                fundamental > 1e-3,
                "{rpm} rpm: no energy at the firing frequency {f_fire:.0} Hz"
            );

            // Little at half order. A perfectly even four has no once-per-cycle
            // component, and this used to demand 20x below the fundamental. The
            // spec now carries the real engine's cylinder-to-cylinder
            // differences, and the onboard recording of the car puts its half
            // and odd orders at -13 to -16 dB (4.5-6.5x). Wrong firing angles
            // or a broken 720-degree wrap still fail this: they put these
            // level with the fundamental.
            for (label, f) in [
                ("half order", f_fire * 0.5),
                ("0.75x", f_fire * 0.75),
                ("1.5x", f_fire * 1.5),
            ] {
                let m = mag_at(&buf, fs, f);
                assert!(
                    m < fundamental / 4.0,
                    "{rpm} rpm: {label} at {f:.0} Hz is {m:.5}, too close to the                      firing fundamental {fundamental:.5}"
                );
            }

            // The harmonics have to actually be there -- a pure tone at the
            // firing frequency would pass the test above and sound nothing like
            // an engine.
            let h2 = mag_at(&buf, fs, f_fire * 2.0);
            assert!(
                h2 > fundamental / 20.0,
                "{rpm} rpm: second harmonic is missing, output is a bare tone"
            );
        }
    }
}
