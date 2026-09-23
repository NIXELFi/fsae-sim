//! The timed skidpad (FSAE D.10), for both models. See tests/common/skidpad.rs
//! for the course, the driver and what "clean" means.
//!
//! The steady-state skidpad in tests/validation.rs stays the bicycle's pinned
//! number; this is the run the real 5.01 s (2026-03-14) was actually timed
//! on: a figure of eight, the tightest line the car holds, timed at the
//! start/stop line on the second lap of each circle.

mod common;

use common::skidpad::*;
use sim_core::prelude::*;

fn car(f: Fidelity) -> Box<dyn Solver> {
    build(f, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())))
}

/// Best clean run over a few constant-radius lines near the cones.
fn best_tight(c: &mut dyn Solver) -> (Line, Result) {
    let tight = tightest_line(c.params(), 0.0);
    let mut best: Option<(Line, Result)> = None;
    for m in [0.25, 0.3, 0.4, 0.5] {
        let r = tight + m;
        if let Some((l, res)) = fastest(c, r, r, 0.0) {
            if best.as_ref().map_or(true, |(_, b)| res.time() < b.time()) {
                best = Some((l, res));
            }
        }
    }
    best.expect("no clean timed skidpad on any line")
}

fn check(f: Fidelity, band: (f64, f64), symmetry_s: f64) {
    let mut c = car(f);
    let centre = fastest(c.as_mut(), D, D, 0.0).expect("no clean run on the lane centre").1;
    let (line, tight) = best_tight(c.as_mut());
    println!(
        "{:?}: lane centre {:.3} s | tight {:.3} s on {:.3} m at {:.2} m/s ({:.3} g, R {:.3} L {:.3})",
        f, centre.time(), tight.time(), line.r_line, line.v_line, tight.mean_ay_g, tight.right_s, tight.left_s
    );
    // Laps are timed the way the rules time them, so both circles are there
    // and the left and right agree on a symmetric car.
    assert!((tight.right_s - tight.left_s).abs() < symmetry_s, "R {:.3} vs L {:.3}", tight.right_s, tight.left_s);
    // Hugging the cones is what a driver does, and it has to pay.
    assert!(tight.time() < centre.time() - 0.05, "tight {:.3} vs centre {:.3}", tight.time(), centre.time());
    assert!(
        (band.0..=band.1).contains(&tight.time()),
        "timed skidpad {:.3} s, real SDM26 5.01 s",
        tight.time()
    );
}

#[test]
fn bicycle_timed_skidpad() {
    // Real best 5.01 s; the 4/23 ARB day ran 5.21-5.40.
    check(Fidelity::Bicycle, (4.85, 5.35), 0.08);
}

#[test]
fn double_track_timed_skidpad() {
    // Looser symmetry: this robot does not drive the 4-wheel model at its
    // limit (2026-09-23: a person ran it 0.2 s quicker than the bicycle while
    // the robot had them equal), so its right and left laps land further
    // apart. The model's grip is calibrated on the steady circle instead
    // (examples/peak_compare.rs), where it equals the bicycle.
    check(Fidelity::DoubleTrack, (4.85, 5.50), 0.15);
}
