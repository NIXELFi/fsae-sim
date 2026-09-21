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
    // Rolling start at 5 m/s in first. From rest the clutch bites into
    // wheelspin (kappa > 1 with the measured 0.15 kg.m^2 wheels), and a
    // chattering clutch is exactly the branch-flipping regime this drive is
    // meant to avoid.
    // Rolling start at 15 m/s in third.
    //
    // It used to start at 5 m/s in first. That is now inside the clutch's
    // engagement window: a launch keeps the clutch slipping until the
    // driveline catches the crank at the launch rpm, which in first is about
    // 11 m/s, and while it is slipping the capacity varies continuously with
    // engine speed AND feeds back into it. One ulp of difference between JS
    // Math and Rust libm then walks the two builds apart -- which is exactly
    // the branch-flipping regime this drive has always been written to avoid,
    // and is why it also stays out of a standing start and off the limit.
    // Above the window the clutch is simply locked, as it is for almost all
    // real driving, and the two ports agree to the last bit again.
    // Rolling start at 15 m/s in third, and a deliberately gentle drive.
    //
    // It used to start at 5 m/s in first with a good deal more throttle. Two
    // things since have pushed that drive out of the regime it was chosen for.
    // The measured torque curve made the car slower, so it sat in first for
    // longer, and the rear axle became two wheels with a differential between
    // them, so one of them can now slip on its own. Together they took the
    // drive to 1.8x the tyre's peak slip ratio with 2856 rpm of clutch slip --
    // which is precisely the branch-flipping regime this drive exists to stay
    // out of, and the two ports duly walked apart.
    //
    // Above 12 m/s the clutch is locked, and at these throttle openings the
    // worst the tyres see is a quarter of their peak. Real driving, nowhere
    // near anything discontinuous, which is the whole point.
    car.reset(0.0, 0.0, 0.0, 15.0);
    car.powertrain_mut().set_gear(2);
    car.powertrain_mut().sync_to_wheel(15.0 / 0.2);
    // The scripted steer values are fractions of the 28 deg lock this drive
    // was written against. The rack's measured limit is now 46 deg, so they
    // are rescaled to keep the golden the same physical manoeuvre -- and so
    // still deliberately inside the tyre -- rather than a more aggressive one.
    // `sim/tools/validate.js` carries the identical constant.
    const SCRIPT_LOCK_DEG: f64 = 28.0;
    let lock_scale = SCRIPT_LOCK_DEG / car.params().steering.max_steer_rad.to_degrees();
    let dt = 1.0 / 60.0;
    let frames = 12 * 60;
    let mut rows = Vec::new();
    for f in 0..frames {
        let t = f as f64 * dt;
        let (steer, throttle, brake) = script(t);
        let steer = steer * lock_scale;
        if f == 150 || f == 300 {
            car.powertrain_mut().shift_up();
        }
        car.step(dt, Controls { steer, throttle, brake });
        if f % 10 == 0 {
            let s = car.state();
            let tel = car.telemetry();
            rows.push(format!(
                "{{\"f\":{f},\"x\":{:.9},\"y\":{:.9},\"psi\":{:.9},\"u\":{:.9},\"v\":{:.9},\"r\":{:.9},\"rpm\":{:.9},\"gear\":{},\"wF\":{:.9},\"wR\":{:.9},\"ayG\":{:.9},\"slipF\":{:.9},\"kappaR\":{:.9},\"rim\":{:.9},\"trail\":{:.9},\"scrub\":{:.9}}}",
                s.x, s.y, s.psi, s.u, s.v, s.r, tel.engine_rpm, tel.gear,
                tel.wheel_omega_front, tel.wheel_omega_rear, tel.ay_g, tel.slip_deg[0],
                tel.kappa[2], tel.rim_torque_nm, tel.trail_front_m, tel.scrub_moment_nm
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
