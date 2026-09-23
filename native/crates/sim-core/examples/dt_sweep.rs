//! Step-size sweep: how much the answer depends on the substep.
//!
//! The solver integrates at `SUBSTEP` (500 Hz) in the tests and the rig, but
//! the physics should not care: every number here should converge as the
//! step shrinks, and 500 Hz should already be on the converged value.
//! `tests/dt_convergence.rs` asserts it.
//!
//!   cargo run --release -p sim-core --example dt_sweep
//!   EV=1 cargo run ...   (a clutch-free drive: chassis, tyres, wheels, diff only)
use sim_core::prelude::*;

const RATES: [f64; 5] = [500.0, 1000.0, 2000.0, 4000.0, 8000.0];

fn car(f: Fidelity) -> Box<dyn Solver> {
    let pt: Box<dyn PowertrainModel> = if std::env::var_os("EV").is_some() {
        Box::new(ElectricDrive::concept())
    } else {
        Box::new(GearedEngine::sdm26())
    };
    build(f, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), pt))
}

/// Standing start with the slip-managed launch of validation.rs; returns the
/// times the car's reference point passes `marks` (interpolated, so not
/// quantised to the step). The DRIVER -- slip loop and shift call -- runs at
/// a fixed 500 Hz whatever the physics step, holding its output between: a
/// proportional loop on slip ratio is itself a discrete controller, and
/// running it at the physics rate would measure its step dependence, not
/// the car's.
fn launch(f: Fidelity, hz: f64, marks: &[f64]) -> Vec<f64> {
    let n = (hz / 500.0).round() as usize;
    let dt = 1.0 / hz;
    let mut c = car(f);
    c.reset(0.0, 0.0, 0.0, 0.0);
    let (mut t, mut throttle, mut out) = (0.0, 1.0, Vec::new());
    for i in 0.. {
        if i % n == 0 {
            let tel = c.telemetry();
            let pt = c.powertrain_mut();
            if pt.can_shift() && tel.engine_rpm > 12_000.0 && tel.gear < 5 {
                pt.shift_up();
            }
            throttle = (1.0 - (tel.kappa[RL].max(tel.kappa[RR]) - 0.13) * 8.0).clamp(0.15, 1.0);
        }
        let x0 = c.state().x;
        c.step(dt, Controls { steer: 0.0, throttle, brake: 0.0 });
        let x1 = c.state().x;
        while out.len() < marks.len() && x1 >= marks[out.len()] {
            out.push(t + dt * (marks[out.len()] - x0) / (x1 - x0));
        }
        t += dt;
        if out.len() == marks.len() || t > 12.0 {
            break;
        }
    }
    out
}

/// Steady cornering on the throttle at 20 m/s, 0.05 steer: the diff sits in
/// its stick band. Mean yaw rate over the last two seconds (deg/s).
fn steady_yaw(f: Fidelity, hz: f64) -> f64 {
    let dt = 1.0 / hz;
    let mut c = car(f);
    c.reset(0.0, 0.0, 0.0, 20.0);
    c.powertrain_mut().set_gear(2);
    c.powertrain_mut().sync_to_wheel(20.0 / 0.2);
    let n = (8.0 * hz) as usize;
    let (mut sum, mut k) = (0.0, 0);
    for i in 0..n {
        let th = (0.25 + (20.0 - c.state().speed()) * 0.5).clamp(0.0, 1.0);
        c.step(dt, Controls { steer: 0.05, throttle: th, brake: 0.0 });
        if i as f64 * dt > 6.0 {
            sum += c.state().r.to_degrees();
            k += 1;
        }
    }
    sum / k as f64
}

fn main() {
    for (name, f) in [("bicycle", Fidelity::Bicycle), ("double track", Fidelity::DoubleTrack)] {
        let acc: Vec<String> = RATES.iter().map(|&hz| format!("{:.4}", launch(f, hz, &[75.0])[0])).collect();
        let yaw: Vec<String> = RATES[..4].iter().map(|&hz| format!("{:.3}", steady_yaw(f, hz))).collect();
        println!("{name:<13} standing 75 m @500/1k/2k/4k/8k Hz: {}", acc.join(" "));
        println!("{name:<13} steady yaw    @500/1k/2k/4k Hz:    {} deg/s", yaw.join(" "));
        // D.9: staged 0.30 m behind the line, the clock runs from the line.
        let m = launch(f, 500.0, &[0.30, 75.30]);
        println!("{name:<13} 75 m from the line (staged 0.30 m back) @500 Hz: {:.3} s", m[1] - m[0]);
    }
}
