//! The 4-wheel model's golden drive: `examples/golden_double_track.rs`
//! replayed and compared, every sampled row, with
//! `tests/golden/double_track.json`.
//!
//! Its physics fingerprint in tests/physics_rev.rs is five limit numbers
//! within a driver's tolerance, which lets a small change through by design.
//! This does not: any change to the double track's physics -- a load path, a
//! camber term, the diff -- moves some state here. Compared within 1e-6
//! (relative above 1) rather than to the printed digit, so a last-ulp libm
//! difference between platforms cannot fail it but a physics change will.
//! Regenerate (with the same sign-off a physics change needs) by running the
//! example into the file.

#[path = "../examples/golden_double_track.rs"]
mod golden;

use sim_core::prelude::*;

/// Every number in a row, in order.
fn numbers(row: &str) -> Vec<f64> {
    row.split(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-' || c == 'e' || c == 'E' || c == '+'))
        .filter(|t| t.chars().any(|c| c.is_ascii_digit()))
        .map(|t| t.parse::<f64>().unwrap_or_else(|_| panic!("bad number {t:?} in {row}")))
        .collect()
}

#[test]
fn the_double_track_reproduces_its_golden_drive() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/golden/double_track.json");
    let file = std::fs::read_to_string(path).expect("golden file");
    let want: Vec<&str> = file.lines().filter(|l| l.starts_with("{\"f\":")).map(|l| l.trim_end_matches(',')).collect();
    assert!(want.len() > 50, "golden has {} rows", want.len());

    let mut car = build(
        Fidelity::DoubleTrack,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    );
    car.reset(0.0, 0.0, 0.0, 15.0);
    car.powertrain_mut().set_gear(2);
    car.powertrain_mut().sync_to_wheel(15.0 / 0.2);
    let lock_scale = 28.0 / car.params().steering.max_steer_rad.to_degrees();
    let dt = 1.0 / 60.0;
    let mut got = Vec::new();
    for f in 0..12 * 60 {
        let (steer, throttle, brake) = golden::script(f as f64 * dt);
        if f == 150 || f == 300 {
            car.powertrain_mut().shift_up();
        }
        car.step(dt, Controls { steer: steer * lock_scale, throttle, brake });
        if f % 10 == 0 {
            got.push(golden::row(f, car.as_ref()));
        }
    }
    assert_eq!(got.len(), want.len());
    for (g, w) in got.iter().zip(want.iter()) {
        let (a, b) = (numbers(g), numbers(w));
        assert_eq!(a.len(), b.len(), "row shape changed:\n got {g}\nwant {w}");
        for (x, y) in a.iter().zip(&b) {
            assert!((x - y).abs() <= 1e-6 * y.abs().max(1.0), "the 4-wheel model moved:\n got {g}\nwant {w}");
        }
    }
}
