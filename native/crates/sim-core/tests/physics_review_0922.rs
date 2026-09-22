//! Regression tests for the 2026-09-22 physics review. Each one reproduces a
//! defect that was measured on the model before the fix.

use sim_core::prelude::*;
use sim_core::modular::{ElasticGeometricSuspension, SuspensionModel};

fn car() -> Box<dyn Solver> {
    sim_core::sdm26_default()
}

/// Held on the brakes with the throttle open -- the staging line on launch
/// control -- the car must not move. It crept BACKWARDS at 2.5 cm/s (7 cm in
/// 3 s) because `0.0.signum()` is +1 and the brake drove a stopped front
/// wheel into reverse.
#[test]
fn a_braked_car_does_not_creep_at_the_line() {
    for (throttle, lc) in [(1.0, true), (0.3, false), (0.06, false)] {
        let mut c = car();
        c.reset(0.0, 0.0, 0.0, 0.0);
        c.powertrain_mut().set_launch(lc);
        for _ in 0..3000 {
            c.step(0.001, Controls { steer: 0.0, throttle, brake: 1.0 });
        }
        let (s, t) = (c.state(), c.telemetry());
        assert!(s.x.abs() < 1e-6, "throttle {throttle} LC {lc}: crept {:.4} m", s.x);
        assert!(t.wheel_omega_front.abs() < 1e-9, "front wheel turning at {} rad/s", t.wheel_omega_front);
    }
}

/// A locked stop: the braked front wheel must sit at zero, not flicker
/// between zero and reverse every other substep.
#[test]
fn a_locked_wheel_stays_at_zero() {
    let mut c = car();
    c.reset(0.0, 0.0, 0.0, 20.0);
    let mut worst: f64 = 0.0;
    while c.state().u > 1.0 {
        c.step(0.001, Controls { steer: 0.0, throttle: 0.0, brake: 1.0 });
        worst = worst.min(c.telemetry().wheel_omega_front);
    }
    // Rounding may leave -1e-17; before the fix it reached -2.8 rad/s.
    assert!(worst > -1e-9, "front wheel ran backwards to {worst} rad/s in a forward stop");
}

/// Sliding sideways at full lock the front slip angle must stay under 90 deg
/// and the front tyre must push AGAINST the slide. It read 110 deg and pushed
/// with it.
#[test]
fn front_slip_angle_never_passes_ninety_degrees() {
    let mut c = car();
    c.reset(0.0, 0.0, 0.0, 0.0);
    {
        let s = c.state_mut();
        s.u = 0.3;
        s.v = -8.0;
    }
    // Put the rack at full left lock first (the servo needs a moment).
    for _ in 0..200 {
        c.step(0.001, Controls { steer: 1.0, throttle: 0.0, brake: 0.0 });
        let s = c.state_mut();
        s.u = 0.3;
        s.v = -8.0;
        s.r = 0.0;
    }
    let t = c.telemetry();
    assert!(t.slip_deg[FL].abs() < 90.0, "front slip {} deg", t.slip_deg[FL]);
    // Sliding right (v < 0): every tyre should be pushing left.
    assert!(t.ay_g > 0.0, "ay {} g", t.ay_g);
}

/// Drag acts against the velocity: a car sliding purely sideways loses
/// lateral speed to it and gains no forward deceleration from it.
#[test]
fn drag_opposes_the_velocity() {
    let mut p = sdm26();
    p.crr = 0.0;
    let mut c = build(
        Fidelity::Bicycle,
        Chassis::new(p, Box::new(LinearTyre { mu_x: 0.0, mu_y: 0.0, ..LinearTyre::sdm26() }), Box::new(IdealDrive::sdm26())),
    );
    c.reset(0.0, 0.0, 0.0, 0.0);
    {
        let s = c.state_mut();
        s.u = 0.0;
        s.v = 20.0;
    }
    c.step(0.001, Controls::default());
    let s = c.state();
    assert!(s.u.abs() < 1e-9, "sideways slide picked up forward speed {}", s.u);
    assert!(s.v < 20.0, "drag did not slow the slide");
}

/// Lateral load transfer: elastic + geometric + unsprung is the free-body
/// m.ay.h/t, and it FOLLOWS the CG height and roll centres when they are
/// edited. The stored roll arm made it 3.4 % low and blind to both.
#[test]
fn lateral_transfer_follows_cg_and_roll_centre_edits() {
    let s = ElasticGeometricSuspension;
    let total = |v: &VehicleParams| {
        let t = s.lateral_transfer(v, 9.81);
        // Per-axle transfer times its own track is a moment; sum and compare.
        t.front_n * v.track_front_m + t.rear_n * v.track_rear_m
    };
    let mut v = sdm26();
    let rigid = |v: &VehicleParams| v.mass_kg * 9.81 * v.cg_height_m;
    assert!((total(&v) / rigid(&v) - 1.0).abs() < 1e-9);
    v.cg_height_m += 0.1;
    assert!((total(&v) / rigid(&v) - 1.0).abs() < 1e-9, "CG edit");
    let before = total(&v);
    v.roll.rc_front_m += 0.05;
    assert!((total(&v) - before).abs() / before < 1e-9, "a roll-centre edit changed TOTAL transfer");
}
