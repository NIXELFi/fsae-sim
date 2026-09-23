//! Steady-state steering torque at the rim against steering-wheel angle,
//! next to the SDM26 design report's "Steer Force Targets" (p.20).
//!
//! For each steering-wheel angle the car is held at a constant speed on
//! that steer until it settles, and the rim torque is read the way the rig
//! computes it: the front tyres' moment about the kingpins through the
//! measured rack's LOCAL slope and its efficiency, times the feel scale
//! (`rim_torque_ratio` carries it), plus the caster/KPI jacking the mixer
//! adds (shown separately).
//!
//!   cargo run --release -p sim-core --example steer_torque [speeds...]
use sim_core::prelude::*;
use sim_core::vehicle::{road_from_rim_deg, road_per_rim};

const SWA: [f64; 9] = [5.0, 10.0, 20.0, 40.0, 60.0, 80.0, 120.0, 150.0, 170.0];

/// Report curves (lbf at the rim; autocross, endurance), read off the p.20 plot.
fn report_ax_lbf(swa: f64) -> Option<f64> {
    match swa as i64 {
        10 => Some(12.5),
        40 => Some(11.7),
        170 => Some(16.2),
        _ => None,
    }
}
fn report_en_lbf(swa: f64) -> Option<f64> {
    match swa as i64 {
        10 => Some(11.2),
        40 => Some(10.0),
        170 => Some(14.6),
        _ => None,
    }
}

/// Hold `swa` at speed `v` until it settles; (rim tyre torque N.m, jacking N.m, ay g).
fn steady(swa: f64, v: f64) -> (f64, f64, f64) {
    let mut c = build(
        Fidelity::Bicycle,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    );
    c.reset(0.0, 0.0, 0.0, v);
    c.powertrain_mut().set_gear(if v > 17.0 { 2 } else if v > 9.0 { 1 } else { 0 });
    c.powertrain_mut().sync_to_wheel(v / 0.2);
    let road = road_from_rim_deg(swa);
    let steer = road / c.params().steering.max_steer_rad.to_degrees();
    let (mut tq, mut n, mut ay) = (0.0, 0, 0.0);
    for i in 0..4000 {
        let th = (0.25 + (v - c.state().speed()) * 0.6).clamp(0.0, 1.0);
        c.step(0.002, Controls { steer, throttle: th, brake: 0.0 });
        if i >= 3000 {
            let t = c.telemetry();
            let p = c.params();
            let ratio = road_per_rim(swa) * p.steering.rack_efficiency * p.steering.feel_scale;
            tq += -t.kingpin_torque_nm * ratio;
            ay += t.ay_g;
            n += 1;
        }
    }
    let p = c.params();
    let t = c.telemetry();
    let (s, tr) = (p.steering.scrub_m, p.mechanical_trail());
    let (sin_l, sin_n) = (p.steering.kpi_rad.sin(), p.steering.caster_rad.sin());
    let d = t.steer_rad;
    let lift = -(t.fz[FL] + t.fz[FR]) * (s * sin_l - tr * sin_n) * d.sin();
    let tilt = -(t.fz[FL] - t.fz[FR]) * (s * sin_n + tr * sin_l) * d.cos();
    let jack = -(lift + tilt) * road_per_rim(swa) * p.steering.rack_efficiency;
    (tq / n as f64, jack, ay / n as f64)
}

/// The lowest speed at which `swa` holds `ay_g` in steady state: scan up,
/// then bisect the bracket (past the limit ay is not monotone in speed).
fn speed_for(swa: f64, ay_g: f64) -> Option<f64> {
    let mut v = 1.5;
    while steady(swa, v).2 < ay_g {
        v += 0.5;
        if v > 35.0 {
            return None;
        }
    }
    let (mut lo, mut hi) = (v - 0.5, v);
    for _ in 0..10 {
        let mid = 0.5 * (lo + hi);
        if steady(swa, mid).2 < ay_g { lo = mid } else { hi = mid }
    }
    Some(0.5 * (lo + hi))
}

fn main() {
    if let Ok(targets) = std::env::var("AY") {
        let targets: Vec<f64> = targets.split(',').filter_map(|a| a.parse().ok()).collect();
        let r_grip = sim_core::vehicle::STEER_RIM_GRIP_RADIUS_M;
        println!("steady cornering at a held lateral g, rim torque (N.m) vs steering-wheel angle; report at r = {r_grip} m");
        print!("{:>6} {:>8} {:>8}", "SWA", "rep AX", "rep EN");
        for a in &targets { print!(" | {a:.2} g:  v m/s  tyre   jack"); }
        println!();
        for &swa in &SWA {
            let rep = report_ax_lbf(swa).map(|l| format!("{:.2}", l * 4.448_221_6 * r_grip)).unwrap_or_default();
            let en = report_en_lbf(swa).map(|l| format!("{:.2}", l * 4.448_221_6 * r_grip)).unwrap_or_default();
            print!("{swa:>6.0} {rep:>8} {en:>8}");
            for &a in &targets {
                match speed_for(swa, a) {
                    Some(v) => { let (tq, j, _) = steady(swa, v); print!(" | {:>12.2} {:>6.2} {:>6.2}", v, tq, j); }
                    None => print!(" | {:>26}", "-"),
                }
            }
            println!();
        }
        return;
    }
    let speeds: Vec<f64> = std::env::args().skip(1).filter_map(|a| a.parse().ok()).collect();
    let speeds = if speeds.is_empty() { vec![8.0, 12.0, 16.0] } else { speeds };
    let r_grip = sim_core::vehicle::STEER_RIM_GRIP_RADIUS_M;
    println!("rim torque (N.m) vs steering-wheel angle; report at r = {r_grip} m (assumed grip radius)");
    print!("{:>6} {:>8}", "SWA", "report");
    for v in &speeds {
        print!(" | {:>5} m/s: tyre  jack   ay g", v);
    }
    println!();
    for &swa in &SWA {
        let rep = report_ax_lbf(swa).map(|l| format!("{:.2}", l * 4.448_221_6 * r_grip)).unwrap_or_default();
        print!("{swa:>6.0} {rep:>8}");
        for &v in &speeds {
            let mut c = build(
                Fidelity::Bicycle,
                Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
            );
            c.reset(0.0, 0.0, 0.0, v);
            c.powertrain_mut().set_gear(if v > 13.0 { 2 } else { 1 });
            c.powertrain_mut().sync_to_wheel(v / 0.2);
            let road = road_from_rim_deg(swa);
            let steer = road / c.params().steering.max_steer_rad.to_degrees();
            let (mut tq, mut n, mut ay) = (0.0, 0, 0.0);
            for i in 0..4000 {
                let th = (0.25 + (v - c.state().speed()) * 0.6).clamp(0.0, 1.0);
                c.step(0.002, Controls { steer, throttle: th, brake: 0.0 });
                if i >= 3000 {
                    let t = c.telemetry();
                    // The rig's local ratio, with the feel scale folded in the
                    // way `rim_torque_ratio` folds it into the nominal one.
                    let p = c.params();
                    let ratio = road_per_rim(swa) * p.steering.rack_efficiency * p.steering.feel_scale;
                    tq += -t.kingpin_torque_nm * ratio;
                    ay += t.ay_g;
                    n += 1;
                }
            }
            let p = c.params();
            let t = c.telemetry();
            let (s, tr) = (p.steering.scrub_m, p.mechanical_trail());
            let (sin_l, sin_n) = (p.steering.kpi_rad.sin(), p.steering.caster_rad.sin());
            let d = t.steer_rad;
            let lift = -(t.fz[FL] + t.fz[FR]) * (s * sin_l - tr * sin_n) * d.sin();
            let tilt = -(t.fz[FL] - t.fz[FR]) * (s * sin_n + tr * sin_l) * d.cos();
            let jack = -(lift + tilt) * road_per_rim(swa) * p.steering.rack_efficiency;
            print!(" | {:>10.2} {:>5.2} {:>6.2}", tq / n as f64, jack, ay / n as f64);
        }
        println!();
    }
}
