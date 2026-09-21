//! The v2 steering-torque term: front longitudinal forces through the scrub
//! radius. It must have the sign of the aligning torque (toward centre) when
//! braking in a corner, mirror with the turn, and vanish without brake. That
//! it never touches the motion is guarded by the golden vectors
//! (`sim/tools/validate.js`), which are unchanged by this term.

use sim_core::prelude::*;

fn car() -> Box<dyn Solver> {
    let mut car = build(
        Fidelity::Bicycle,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    );
    car.reset(0.0, 0.0, 0.0, 15.0);
    car.powertrain_mut().set_gear(2);
    car.powertrain_mut().sync_to_wheel(15.0 / 0.2);
    car
}

/// 3 s in a steady corner (steer as a fraction of lock, + = left), then
/// `brake` for 0.4 s. Returns the telemetry at the end.
fn corner_then_brake(steer: f64, brake: f64) -> Telemetry {
    let mut c = car();
    let dt = 0.01;
    for _ in 0..300 {
        let th = (0.12 + 0.6 * (15.0 - c.telemetry().speed)).clamp(0.0, 1.0);
        c.step(dt, Controls { steer, throttle: th, brake: 0.0 });
    }
    for _ in 0..40 {
        c.step(dt, Controls { steer, throttle: 0.0, brake });
    }
    c.telemetry()
}

#[test]
fn braking_in_a_left_turn_adds_centring_torque() {
    let t = corner_then_brake(0.10, 0.45);
    assert!(t.ay_g > 0.5, "should be cornering left, ay = {}", t.ay_g);
    // Left turn: the aligning torque is negative (toward the right, centre).
    assert!(t.kingpin_torque_nm < 0.0);
    assert!(
        t.scrub_moment_nm < -1.0,
        "scrub moment should add centring torque when braking, got {}",
        t.scrub_moment_nm
    );
    // Order of magnitude: a fraction of the tyres' own aligning moment.
    assert!(t.scrub_moment_nm.abs() < t.kingpin_torque_nm.abs());
}

#[test]
fn mirrors_with_the_turn() {
    let l = corner_then_brake(0.10, 0.45);
    let r = corner_then_brake(-0.10, 0.45);
    assert!(r.scrub_moment_nm > 1.0, "right turn should flip the sign, got {}", r.scrub_moment_nm);
    let rel = (l.scrub_moment_nm + r.scrub_moment_nm).abs() / l.scrub_moment_nm.abs();
    assert!(rel < 0.05, "left {} vs right {}", l.scrub_moment_nm, r.scrub_moment_nm);
}

#[test]
fn negligible_without_brakes() {
    let t = corner_then_brake(0.10, 0.0);
    assert!(t.scrub_moment_nm.abs() < 0.5, "coasting front Fx is tiny, got {}", t.scrub_moment_nm);
}
