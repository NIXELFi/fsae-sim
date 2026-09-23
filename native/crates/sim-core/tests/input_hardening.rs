//! Garbage in must not become NaN out. A NaN pedal or steer (a device that
//! reports nothing, a divide by zero upstream) used to reach the integrator,
//! and one NaN substep leaves the car NaN for the rest of the session.

use sim_core::prelude::*;

fn cars() -> Vec<Box<dyn Solver>> {
    [Fidelity::PointMass, Fidelity::Bicycle, Fidelity::DoubleTrack]
        .into_iter()
        .map(|f| build(f, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26()))))
        .collect()
}

fn finite(c: &dyn Solver) -> bool {
    let s = c.state();
    let t = c.telemetry();
    [s.u, s.v, s.r, s.x, s.y, s.psi, t.engine_rpm, t.wheel_omega_front, t.wheel_omega_rear, t.rim_torque_nm, t.steer_rad]
        .iter()
        .all(|x| x.is_finite())
}

#[test]
fn non_finite_controls_leave_the_state_finite() {
    let bad = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 1e300, -1e300];
    for mut c in cars() {
        c.reset(0.0, 0.0, 0.0, 12.0);
        for (i, &x) in bad.iter().cycle().take(40).enumerate() {
            let ctl = match i % 4 {
                0 => Controls { steer: x, throttle: 0.5, brake: 0.0 },
                1 => Controls { steer: 0.1, throttle: x, brake: 0.0 },
                2 => Controls { steer: 0.1, throttle: 0.0, brake: x },
                _ => Controls { steer: x, throttle: x, brake: x },
            };
            c.step(0.01, ctl);
            assert!(finite(c.as_ref()), "{}: state went non-finite on {ctl:?}", c.name());
        }
        // A NaN or infinite step is no step, not 100 ms.
        let before = c.state();
        c.step(f64::NAN, Controls::default());
        c.step(f64::INFINITY, Controls::default());
        c.step(-1.0, Controls::default());
        let after = c.state();
        assert_eq!((before.x, before.u), (after.x, after.u), "{}: a non-finite dt moved the car", c.name());
        // And it still drives afterwards.
        for _ in 0..100 {
            c.step(0.01, Controls { steer: 0.0, throttle: 0.4, brake: 0.0 });
        }
        assert!(finite(c.as_ref()) && c.state().u > 1.0, "{}: {:?}", c.name(), c.state());
    }
}

#[test]
fn sanitized_controls_are_in_range() {
    let c = Controls { steer: f64::NAN, throttle: 3.0, brake: -2.0 }.sanitized();
    assert_eq!((c.steer, c.throttle, c.brake), (0.0, 1.0, 0.0));
    let c = Controls { steer: -7.0, throttle: f64::INFINITY, brake: 0.4 }.sanitized();
    assert_eq!((c.steer, c.throttle, c.brake), (-1.0, 0.0, 0.4));
}
