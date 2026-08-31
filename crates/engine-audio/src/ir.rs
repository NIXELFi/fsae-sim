//! Impulse responses for the convolution stage.
//!
//! engine-sim ships recorded impulse responses in `es/sound-library`. Those are
//! its own assets, not ours to redistribute, so this generates equivalents
//! procedurally: a set of discrete early reflections over an exponentially
//! decaying, band-limited noise tail. That is the structure of a real
//! small-space impulse response, and it is what gives the dry waveguide output
//! a body instead of sounding like it was recorded inside the pipe.
//!
//! Everything here is deterministic from a seed, so the JS port convolves
//! against exactly the same numbers.

use crate::filters::{ButterworthLowPass, Rng};

/// Where the listener is. A driver's ears and a trackside microphone hear very
/// different things from the same exhaust, and most of the difference is this.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cabin {
    /// In the car. Close, boxy, strong early reflections off the bodywork and
    /// the driver's own helmet, short tail.
    Cockpit,
    /// Alongside the car. Longer, more diffuse, a ground reflection.
    Trackside,
    /// No space at all -- the raw waveguide. Useful for debugging and for
    /// anyone who wants to run their own convolution downstream.
    Anechoic,
}

/// Build an impulse response.
///
/// `taps` is the length in samples. 512 at 48 kHz is about 11 ms, which is
/// enough for the early-reflection structure that carries the impression of a
/// space; a full reverb tail is not what this stage is for.
pub fn build(cabin: Cabin, taps: usize, sample_rate: f32, seed: u64) -> Vec<f32> {
    if cabin == Cabin::Anechoic || taps == 0 {
        return vec![1.0];
    }

    let mut ir = vec![0.0f32; taps];
    let mut rng = Rng::new(seed);

    // Direct sound.
    ir[0] = 1.0;

    // Early reflections: delay in milliseconds, and amplitude.
    let (reflections, decay_s, cutoff_hz): (&[(f32, f32)], f32, f32) = match cabin {
        Cabin::Cockpit => (
            &[
                (0.6, -0.62), // roll hoop, very close and inverted
                (1.1, 0.44),  // floor
                (1.9, -0.31), // sidepod
                (3.2, 0.22),  // firewall
                (4.8, -0.14),
            ],
            0.020,
            5_500.0,
        ),
        Cabin::Trackside => (
            &[
                (1.8, 0.52), // ground bounce
                (4.5, -0.34),
                (7.9, 0.24),
                (12.0, -0.16),
                (17.5, 0.10),
            ],
            0.055,
            7_500.0,
        ),
        Cabin::Anechoic => (&[], 0.0, 20_000.0),
    };

    for &(ms, amp) in reflections {
        let idx = (ms * 1e-3 * sample_rate).round() as usize;
        if idx < taps {
            ir[idx] += amp;
        }
    }

    // Diffuse tail: noise under an exponential envelope, starting after the
    // first reflection so it does not smear the direct sound.
    let start = (0.7e-3 * sample_rate) as usize;
    let tau = decay_s * sample_rate;
    for (i, v) in ir.iter_mut().enumerate().skip(start) {
        let env = (-(i as f32) / tau).exp();
        *v += rng.uniform() * env * 0.35;
    }

    // Band-limit it. An IR with energy up at Nyquist makes the convolution
    // output hissy in a way no real space is.
    let mut lp = ButterworthLowPass::new(cutoff_hz, sample_rate);
    for v in ir.iter_mut() {
        *v = lp.f(*v);
    }

    normalise(&mut ir);
    ir
}

/// Scale so the response has unit energy.
///
/// Normalising by energy rather than by peak keeps the perceived loudness
/// steady when switching between cabins -- a longer, more diffuse response has
/// a lower peak but the same total power.
fn normalise(ir: &mut [f32]) {
    let energy: f32 = ir.iter().map(|v| v * v).sum();
    if energy > 1e-12 {
        let g = 1.0 / energy.sqrt();
        for v in ir.iter_mut() {
            *v *= g;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn anechoic_is_a_unit_impulse() {
        assert_eq!(build(Cabin::Anechoic, 512, 48_000.0, 1), vec![1.0]);
    }

    #[test]
    fn responses_have_unit_energy() {
        for cabin in [Cabin::Cockpit, Cabin::Trackside] {
            let ir = build(cabin, 512, 48_000.0, 9);
            let e: f32 = ir.iter().map(|v| v * v).sum();
            assert!((e - 1.0).abs() < 1e-4, "{cabin:?} energy {e}");
        }
    }

    #[test]
    fn responses_are_deterministic() {
        let a = build(Cabin::Cockpit, 256, 48_000.0, 42);
        let b = build(Cabin::Cockpit, 256, 48_000.0, 42);
        assert_eq!(a, b);
    }

    #[test]
    fn cockpit_decays_faster_than_trackside() {
        let tail = |c| {
            let ir = build(c, 1024, 48_000.0, 3);
            let n = ir.len();
            let late: f32 = ir[n / 2..].iter().map(|v| v * v).sum();
            let early: f32 = ir[..n / 2].iter().map(|v| v * v).sum();
            late / early.max(1e-12)
        };
        assert!(
            tail(Cabin::Cockpit) < tail(Cabin::Trackside),
            "cockpit should be the drier space"
        );
    }

    #[test]
    fn every_response_is_finite() {
        for cabin in [Cabin::Cockpit, Cabin::Trackside, Cabin::Anechoic] {
            assert!(build(cabin, 512, 48_000.0, 5).iter().all(|v| v.is_finite()));
        }
    }
}
