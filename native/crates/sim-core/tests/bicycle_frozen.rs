//! The validated bicycle is FROZEN. Every new model (the double track, its
//! suspension, camber, load-dependent peak slip) lives beside it and must
//! not move it in any way.
//!
//! This re-runs the golden drive of `examples/golden_vehicle.rs` and requires
//! every sampled row to match `sim/data/vehicle-golden.json` to the last
//! printed digit. `validate.js` checks JS against the same file, so the file
//! is the bicycle's definition: a change that moves it fails here, and
//! regenerating the file to make it pass is changing the validated car.
//!
//! Regenerated ONCE since the freeze, with Nick's approval (2026-09-22,
//! commit e9933c0): the launch clutch / clutch-lock fixes in the shared
//! powertrain. And again the same night (Nick: "go ahead and add whatever"):
//! the tyre's longitudinal fall-off past the peak (FX_FALLOFF_KEEP). And on
//! 2026-09-23 by owner decision (physics review fix list, no era bump): the
//! wheels' implicit slip term gets its chassis half (no phantom inertia) and
//! the diff's clutch is integrated implicitly (step-independent stick band);
//! then the steering-feel calibration (rim torque). Any other regeneration
//! needs the same explicit sign-off.

use sim_core::prelude::*;

fn script(t: f64) -> (f64, f64, f64) {
    if t < 3.0 {
        (0.0, 0.35, 0.0)
    } else if t < 5.0 {
        (0.10, 0.30, 0.0)
    } else if t < 5.5 {
        (0.04, 0.0, 0.15)
    } else if t < 9.0 {
        (-0.08, 0.35, 0.0)
    } else {
        (0.06, 0.45, 0.0)
    }
}

#[test]
fn the_bicycle_reproduces_its_golden_drive_exactly() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../sim/data/vehicle-golden.json");
    let golden = std::fs::read_to_string(path).expect("golden file");
    let want: Vec<&str> = golden
        .lines()
        .filter(|l| l.starts_with("{\"f\":"))
        .map(|l| l.trim_end_matches(','))
        .collect();
    assert!(want.len() > 50, "golden has {} rows", want.len());

    let mut car = build(
        Fidelity::Bicycle,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    );
    car.reset(0.0, 0.0, 0.0, 15.0);
    car.powertrain_mut().set_gear(2);
    car.powertrain_mut().sync_to_wheel(15.0 / 0.2);
    let lock_scale = 28.0 / car.params().steering.max_steer_rad.to_degrees();
    let dt = 1.0 / 60.0;
    let mut got = Vec::new();
    for f in 0..12 * 60 {
        let (steer, throttle, brake) = script(f as f64 * dt);
        if f == 150 || f == 300 {
            car.powertrain_mut().shift_up();
        }
        car.step(dt, Controls { steer: steer * lock_scale, throttle, brake });
        if f % 10 == 0 {
            let s = car.state();
            let tel = car.telemetry();
            got.push(format!(
                "{{\"f\":{f},\"x\":{:.9},\"y\":{:.9},\"psi\":{:.9},\"u\":{:.9},\"v\":{:.9},\"r\":{:.9},\"rpm\":{:.9},\"gear\":{},\"wF\":{:.9},\"wR\":{:.9},\"ayG\":{:.9},\"slipF\":{:.9},\"kappaR\":{:.9},\"rim\":{:.9},\"trail\":{:.9},\"scrub\":{:.9}}}",
                s.x, s.y, s.psi, s.u, s.v, s.r, tel.engine_rpm, tel.gear,
                tel.wheel_omega_front, tel.wheel_omega_rear, tel.ay_g, tel.slip_deg[0],
                tel.kappa[2], tel.rim_torque_nm, tel.trail_front_m, tel.scrub_moment_nm
            ));
        }
    }
    assert_eq!(got.len(), want.len());
    for (g, w) in got.iter().zip(want.iter()) {
        assert_eq!(g, w, "the validated bicycle moved");
    }
}

/// The tyre the bicycle is built with carries none of the double track's
/// additions: its peak slip does not move with load.
#[test]
fn the_bicycles_tyre_is_the_validated_one() {
    let t = MagicFormulaTyre::sdm26();
    assert!(t.peak_alpha_scale_at.is_none());
    // And building a double track does not reach back into a bicycle's tyre:
    // each solver owns its own.
    let _dt = build(
        Fidelity::DoubleTrack,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    );
    assert!(MagicFormulaTyre::sdm26().peak_alpha_scale_at.is_none());
}
