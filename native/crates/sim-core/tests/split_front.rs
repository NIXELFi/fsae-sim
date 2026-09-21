//! `split_front_wheels` (FFB/steering model v2.1): each front wheel its own
//! speed state. With it off the solver is bit-identical to before (guarded by
//! the golden vectors); these check what it adds when on.

use sim_core::prelude::*;

fn car(split: bool) -> Box<dyn Solver> {
    let mut car = build(
        Fidelity::Bicycle,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    );
    car.params_mut().split_front_wheels = split;
    car.reset(0.0, 0.0, 0.0, 15.0);
    car.powertrain_mut().set_gear(2);
    car.powertrain_mut().sync_to_wheel(15.0 / 0.2);
    car
}

fn corner_then_brake(split: bool, steer: f64, brake: f64, brake_s: f64) -> Box<dyn Solver> {
    let mut c = car(split);
    let dt = 0.01;
    for _ in 0..300 {
        let th = (0.12 + 0.6 * (15.0 - c.telemetry().speed)).clamp(0.0, 1.0);
        c.step(dt, Controls { steer, throttle: th, brake: 0.0 });
    }
    for _ in 0..(brake_s / dt) as usize {
        c.step(dt, Controls { steer, throttle: 0.0, brake });
    }
    c
}

#[test]
fn straight_line_braking_is_symmetric_and_matches_the_single_rotor() {
    let one = corner_then_brake(false, 0.0, 0.5, 0.8);
    let two = corner_then_brake(true, 0.0, 0.5, 0.8);
    let t = two.telemetry();
    assert!((t.kappa[FL] - t.kappa[FR]).abs() < 1e-9, "left/right differ in a straight line");
    assert!(two.state().r.abs() < 1e-9, "straight braking should not yaw");
    let du = (one.state().u - two.state().u).abs();
    assert!(du < 0.02, "split front changed straight-line braking by {du} m/s");
}

#[test]
fn inside_front_locks_first_braking_in_a_left_turn() {
    // Left turn: the left front is the unloaded inside wheel.
    let c = corner_then_brake(true, 0.10, 0.7, 0.3);
    let t = c.telemetry();
    // Heavy braking bleeds lateral g fast (the car runs wide); still turning.
    assert!(t.ay_g > 0.2, "should still be cornering, ay = {}", t.ay_g);
    assert!(
        t.kappa[FL] < t.kappa[FR] - 0.02,
        "inside front should be further into lockup: FL {} FR {}",
        t.kappa[FL], t.kappa[FR]
    );
}

#[test]
fn right_turn_mirrors() {
    let l = corner_then_brake(true, 0.10, 0.7, 0.3).telemetry();
    let r = corner_then_brake(true, -0.10, 0.7, 0.3).telemetry();
    assert!((l.kappa[FL] - r.kappa[FR]).abs() < 1e-6 && (l.kappa[FR] - r.kappa[FL]).abs() < 1e-6);
    assert!((l.scrub_moment_nm + r.scrub_moment_nm).abs() < 1e-6);
}

#[test]
fn stays_finite_and_stops_under_full_brake() {
    let mut c = corner_then_brake(true, 0.12, 1.0, 1.0);
    for _ in 0..600 {
        c.step(0.01, Controls { steer: 0.0, throttle: 0.0, brake: 1.0 });
    }
    let s = c.state();
    assert!(s.u.is_finite() && s.r.is_finite());
    assert!(s.speed() < 0.5, "should have stopped, speed {}", s.speed());
}
