//! Emit the vehicle-model golden vectors the JS build is checked against.
//!
//! A scripted 12 s drive -- launch, an upshift, a corner, a brake, a harder
//! corner the other way, and a flat-out sweeper -- sampled every tenth frame.
//! `sim/tools/validate.js` runs the identical script through the JS model and
//! compares. The two are ports of each other; a divergence is a bug in one of
//! them, not a modelling choice.
//!
//!   cargo run --release -p sim-core --example golden_vehicle > ../sim/data/vehicle-golden.json

use sim_core::prelude::*;

fn main() {
    let mut car = build(
        Fidelity::Bicycle,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    );
    car.reset(0.0, 0.0, 0.0, 0.0);
    let dt = 1.0 / 60.0;
    let frames = 12 * 60;
    let mut rows = Vec::new();
    for f in 0..frames {
        let t = f as f64 * dt;
        let (steer, throttle, brake) = script(t);
        if f == 150 || f == 300 {
            car.powertrain_mut().shift_up();
        }
        car.step(dt, Controls { steer, throttle, brake });
        if f % 10 == 0 {
            let s = car.state();
            let tel = car.telemetry();
            rows.push(format!(
                "{{\"f\":{f},\"x\":{:.9},\"y\":{:.9},\"psi\":{:.9},\"u\":{:.9},\"v\":{:.9},\"r\":{:.9},\"rpm\":{:.9},\"gear\":{},\"wF\":{:.9},\"wR\":{:.9},\"ayG\":{:.9},\"slipF\":{:.9},\"kappaR\":{:.9},\"rim\":{:.9},\"trail\":{:.9}}}",
                s.x, s.y, s.psi, s.u, s.v, s.r, tel.engine_rpm, tel.gear,
                tel.wheel_omega_front, tel.wheel_omega_rear, tel.ay_g, tel.slip_deg[0],
                tel.kappa[2], tel.rim_torque_nm, tel.trail_front_m
            ));
        }
    }
    println!(
        "{{\"note\":\"Rust sim-core bicycle solver, scripted 12 s drive at 60 Hz frames; see examples/golden_vehicle.rs\",\"dt\":{dt},\"shiftFrames\":[150,300],\"rows\":[\n{}\n]}}",
        rows.join(",\n")
    );
}

/// The drive. Kept dead simple so it is trivially reproduced in JS.
///
/// Deliberately NOT at the limit, and never near a stall. A launch on the
/// edge of wheelspin, a spinning car, or a clutch chattering at a crawl are
/// all discontinuous regimes where one-ulp differences between JS Math and
/// Rust libm flip a branch and grow into metres -- which says nothing about
/// whether the models agree. Real driving, inside the tyre and above walking
/// pace, is what is checked.
pub fn script(t: f64) -> (f64, f64, f64) {
    if t < 3.0 {
        (0.0, 0.55, 0.0)
    } else if t < 5.0 {
        (0.15, 0.4, 0.0)
    } else if t < 5.5 {
        (0.05, 0.0, 0.25)
    } else if t < 9.0 {
        (-0.12, 0.5, 0.0)
    } else {
        (0.08, 0.8, 0.0)
    }
}
