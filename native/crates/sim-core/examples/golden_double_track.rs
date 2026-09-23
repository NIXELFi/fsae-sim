//! Emit the 4-wheel (double track) golden drive.
//!
//! The bicycle's golden drive (examples/golden_vehicle.rs) pins it to the
//! JS port and to the last printed digit; nothing did the same for the
//! double track, whose fingerprint was only five limit numbers. This is the
//! same scripted 12 s drive -- a rolling start, two upshifts, a corner each
//! way, a brake, a sweeper -- through the double track, with its own states
//! (roll, pitch, four loads, four cambers, four slip ratios, the diff) in
//! every sampled row. `tests/double_track_frozen.rs` replays it and compares
//! within 1e-6.
//!
//!   cargo run --release -p sim-core --example golden_double_track > crates/sim-core/tests/golden/double_track.json

use sim_core::prelude::*;

#[allow(dead_code)]
fn main() {
    let mut car = build(
        Fidelity::DoubleTrack,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    );
    car.reset(0.0, 0.0, 0.0, 15.0);
    car.powertrain_mut().set_gear(2);
    car.powertrain_mut().sync_to_wheel(15.0 / 0.2);
    let lock_scale = 28.0 / car.params().steering.max_steer_rad.to_degrees();
    let dt = 1.0 / 60.0;
    let mut rows = Vec::new();
    for f in 0..12 * 60 {
        let (steer, throttle, brake) = script(f as f64 * dt);
        if f == 150 || f == 300 {
            car.powertrain_mut().shift_up();
        }
        car.step(dt, Controls { steer: steer * lock_scale, throttle, brake });
        if f % 10 == 0 {
            rows.push(row(f, car.as_ref()));
        }
    }
    println!(
        "{{\"note\":\"Rust sim-core double track, scripted 12 s drive at 60 Hz frames; see examples/golden_double_track.rs\",\"dt\":{dt},\"shiftFrames\":[150,300],\"rows\":[\n{}\n]}}",
        rows.join(",\n")
    );
}

/// One sampled row: the pose and velocities, the powertrain, and the double
/// track's own states.
pub fn row(f: usize, car: &dyn Solver) -> String {
    let s = car.state();
    let t = car.telemetry();
    format!(
        "{{\"f\":{f},\"x\":{:.9},\"y\":{:.9},\"psi\":{:.9},\"u\":{:.9},\"v\":{:.9},\"r\":{:.9},\"rpm\":{:.9},\"gear\":{},\"wF\":{:.9},\"wR\":{:.9},\"ayG\":{:.9},\"rim\":{:.9},\"roll\":{:.9},\"pitch\":{:.9},\"fz\":[{:.9},{:.9},{:.9},{:.9}],\"camber\":[{:.9},{:.9},{:.9},{:.9}],\"kappa\":[{:.9},{:.9},{:.9},{:.9}],\"diff\":{:.9}}}",
        s.x, s.y, s.psi, s.u, s.v, s.r, t.engine_rpm, t.gear, t.wheel_omega_front, t.wheel_omega_rear, t.ay_g,
        t.rim_torque_nm, t.roll_deg, t.pitch_deg, t.fz[0], t.fz[1], t.fz[2], t.fz[3],
        t.camber_deg[0], t.camber_deg[1], t.camber_deg[2], t.camber_deg[3],
        t.kappa[0], t.kappa[1], t.kappa[2], t.kappa[3], t.diff_nm
    )
}

/// The bicycle golden's script, unchanged (see examples/golden_vehicle.rs).
pub fn script(t: f64) -> (f64, f64, f64) {
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
