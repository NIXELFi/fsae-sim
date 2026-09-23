//! Step-size sweep: how much the answer depends on the substep.
//!
//! The solver integrates at `SUBSTEP` (500 Hz) in the tests and the rig, but
//! the physics should not care: every number here should converge as the
//! step shrinks, and 500 Hz should already be on the converged value.
//!
//!   cargo run --release -p sim-core --example dt_sweep
use sim_core::prelude::*;

fn car(f: Fidelity) -> Box<dyn Solver> {
    if std::env::var_os("EV").is_some() {
        return build(f, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(ElectricDrive::concept())));
    }
    build(f, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())))
}

/// Standing 75 m, slip-managed launch (validation.rs), crossing time
/// interpolated so it is not quantised to the step. The DRIVER -- the slip
/// loop and the shift decision -- runs at a fixed 500 Hz whatever the
/// physics step, holding its output in between: a proportional loop on
/// slip ratio is itself a discrete controller, and letting it run at the
/// physics rate measures the controller's step dependence, not the car's.
pub fn seventy_five(f: Fidelity, hz: f64) -> f64 {
    run_to(f, hz, 75.0, true)
}

pub fn run_to(f: Fidelity, hz: f64, goal: f64, shifts: bool) -> f64 {
    let n = (hz / 500.0).round() as usize;
    let dt = 1.0 / hz;
    let mut c = car(f);
    c.reset(0.0, 0.0, 0.0, 0.0);
    let mut t = 0.0;
    let mut throttle = 1.0;
    for i in 0.. {
        if i % n == 0 {
            let tel = c.telemetry();
            let pt = c.powertrain_mut();
            if shifts && pt.can_shift() && tel.engine_rpm > 12_000.0 && tel.gear < 5 {
                pt.shift_up();
            }
            let over = tel.kappa[RL].max(tel.kappa[RR]) - 0.13;
            throttle = (1.0 - over * 8.0).clamp(0.15, 1.0);
        }
        let x0 = c.state().x;
        c.step(dt, Controls { steer: 0.0, throttle, brake: 0.0 });
        let x1 = c.state().x;
        if x1 >= goal {
            return t + dt * (goal - x0) / (x1 - x0);
        }
        t += dt;
        if t > 12.0 {
            break;
        }
    }
    f64::NAN
}

/// Steady cornering on the throttle: fixed steer, speed held by a throttle
/// loop, mean yaw rate over the last two seconds (deg/s).
fn steady_yaw(f: Fidelity, hz: f64) -> f64 { steady_yaw_at(f, hz, 12.0, 0.12, 1) }
fn steady_yaw_at(f: Fidelity, hz: f64, v0: f64, steer: f64, gear: usize) -> f64 {
    let dt = 1.0 / hz;
    let mut c = car(f);
    c.reset(0.0, 0.0, 0.0, v0);
    c.powertrain_mut().set_gear(gear);
    c.powertrain_mut().sync_to_wheel(v0 / 0.2);
    let n = (8.0 * hz) as usize;
    let (mut sum, mut k) = (0.0, 0);
    for i in 0..n {
        let s = c.state();
        let th = (0.25 + (v0 - s.speed()) * 0.5).clamp(0.0, 1.0);
        c.step(dt, Controls { steer, throttle: th, brake: 0.0 });
        if i as f64 * dt > 6.0 {
            sum += c.state().r.to_degrees();
            k += 1;
        }
    }
    sum / k as f64
}

fn main() {
    if std::env::var_os("ISO3").is_some() {
        // Open-loop throttle, no controller at all; and ideal drive with the 500 Hz loop.
        for th in [0.3, 0.5, 1.0] {
            for f in [Fidelity::Bicycle, Fidelity::DoubleTrack] {
                let r: Vec<String> = [500.0, 1000.0, 2000.0, 4000.0, 8000.0].iter().map(|&hz| {
                    let dt = 1.0 / hz; let mut c = car(f); c.reset(0.0,0.0,0.0,0.0); let mut t = 0.0;
                    loop { let x0 = c.state().x; c.step(dt, Controls { steer: 0.0, throttle: th, brake: 0.0 }); let x1 = c.state().x;
                        if x1 >= 20.0 { break format!("{:.4}", t + dt * (20.0 - x0) / (x1 - x0)); } t += dt; if t > 20.0 { break "nan".into(); } }
                }).collect();
                println!("{f:?} open-loop throttle {th} 20 m: {}", r.join(" "));
            }
        }
        return;
    }
    if std::env::var_os("ISO2").is_some() {
        for (goal, sh) in [(5.0, false), (10.0, false), (20.0, false), (30.0, false), (30.0, true), (50.0, true), (75.0, true)] {
            for f in [Fidelity::Bicycle, Fidelity::DoubleTrack] {
                let r: Vec<String> = [500.0, 1000.0, 2000.0, 4000.0, 8000.0].iter().map(|&hz| format!("{:.4}", run_to(f, hz, goal, sh))).collect();
                println!("{f:?} {goal} m shifts {sh}: {}", r.join(" "));
            }
        }
        return;
    }
    if std::env::var_os("ISO").is_some() {
        // Isolate: which part of the car depends on the step?
        for label in ["ideal", "geared-noshift", "geared-noshift-40m"] {
            let mut row = String::new();
            for &hz in &[500.0, 1000.0, 2000.0, 4000.0, 8000.0] {
                let dt = 1.0 / hz;
                let mut c: Box<dyn Solver> = if label == "ideal" {
                    build(Fidelity::Bicycle, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(IdealDrive::sdm26())))
                } else { car(Fidelity::Bicycle) };
                c.reset(0.0, 0.0, 0.0, 0.0);
                let goal = if label.ends_with("40m") { 40.0 } else if label == "ideal" { 75.0 } else { 20.0 };
                let mut t = 0.0;
                loop {
                    let tel = c.telemetry();
                    let over = tel.kappa[RL].max(tel.kappa[RR]) - 0.13;
                    let x0 = c.state().x;
                    c.step(dt, Controls { steer: 0.0, throttle: (1.0 - over * 8.0).clamp(0.15, 1.0), brake: 0.0 });
                    let x1 = c.state().x;
                    if x1 >= goal { t += dt * (goal - x0) / (x1 - x0); break; }
                    t += dt;
                    if t > 20.0 { break; }
                }
                row += &format!(" {t:.4}");
            }
            println!("{label:<20}{row}");
        }
        return;
    }
    if std::env::var_os("TRACE").is_some() {
        for &hz in &[500.0, 1000.0, 2000.0, 8000.0] {
            let dt = 1.0 / hz;
            let mut c = car(Fidelity::Bicycle);
            c.reset(0.0, 0.0, 0.0, 0.0);
            let mut t = 0.0;
            let marks = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0];
            let mut mi = 0;
            let mut line = String::new();
            while mi < marks.len() {
                let tel = c.telemetry();
                {
                    let pt = c.powertrain_mut();
                    if pt.can_shift() && tel.engine_rpm > 12_000.0 && tel.gear < 5 {
                        pt.shift_up();
                    }
                }
                let over = tel.kappa[RL].max(tel.kappa[RR]) - 0.13;
                c.step(dt, Controls { steer: 0.0, throttle: (1.0 - over * 8.0).clamp(0.15, 1.0), brake: 0.0 });
                t += dt;
                if t >= marks[mi] - 1e-9 {
                    let tel = c.telemetry();
                    line += &format!(" {:.3}/{:.3}/{:.0}", c.state().x, tel.kappa[RL], tel.engine_rpm);
                    mi += 1;
                }
            }
            println!("{hz:>5}:{line}");
        }
        return;
    }
    if std::env::var_os("SCAN").is_some() {
        for (v0, st, g) in [(8.0,0.1,0),(8.0,0.2,1),(10.0,0.1,1),(15.0,0.05,2),(15.0,0.1,2),(20.0,0.05,2),(12.0,0.05,1),(6.0,0.3,0)] {
            let y: Vec<String> = [500.0,1000.0,2000.0,4000.0].iter().map(|&hz| format!("{:.3}", steady_yaw_at(Fidelity::Bicycle, hz, v0, st, g))).collect();
            println!("v {v0} steer {st} gear {g}: {}", y.join(" "));
        }
        return;
    }
    let rates = [500.0, 1000.0, 2000.0, 4000.0, 8000.0];
    for (name, f) in [("bicycle", Fidelity::Bicycle), ("double track", Fidelity::DoubleTrack)] {
        let acc: Vec<String> = rates.iter().map(|&hz| format!("{:.4}", seventy_five(f, hz))).collect();
        let yaw: Vec<String> = rates.iter().map(|&hz| format!("{:.3}", steady_yaw(f, hz))).collect();
        println!("{name:<13} 75 m  @500/1k/2k/4k/8k Hz: {}", acc.join(" "));
        println!("{name:<13} yaw   @500/1k/2k/4k/8k Hz: {}", yaw.join(" "));
    }
}
