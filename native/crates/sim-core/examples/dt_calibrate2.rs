//! Calibrate the double track's front grip scale against the TIMED skidpad
//! the bicycle runs (same harness, same real anchor), and check it still
//! pushes and does not spin on the 10/15/20 m/s steer ramps.
#[path = "../tests/common/skidpad.rs"]
#[allow(dead_code)]
mod skidpad;
use sim_core::prelude::*;
fn car(scale: f64) -> Box<dyn Solver> {
    let mut p = sdm26();
    p.suspension.front_grip_scale = scale;
    p.suspension.rear_grip_scale = std::env::var("R").ok().and_then(|v| v.parse().ok()).unwrap_or(1.0);
    build(Fidelity::DoubleTrack, Chassis::new(p, Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())))
}
fn ramp(scale: f64, speed: f64) -> (f64, f64, f64) {
    const DT: f64 = 1.0 / 60.0;
    let mut c = car(scale); c.reset(0.0, 0.0, 0.0, speed);
    let ratios = [2.75, 2.0, 1.667, 1.444, 1.304, 1.208]; let (mut best, mut bd) = (0usize, f64::MAX);
    for (g, r) in ratios.iter().enumerate() { let rpm = speed / 0.2 * 2.111 * r * 3.0 * 60.0 / (2.0 * std::f64::consts::PI); if rpm < 14000.0 && (rpm - 9500.0).abs() < bd { bd = (rpm - 9500.0).abs(); best = g; } }
    c.powertrain_mut().set_gear(best); c.powertrain_mut().sync_to_wheel(speed / 0.2);
    let (mut t, mut pk, mut bal, mut beta) = (0.0, 0.0f64, 0.0, 0.0f64);
    while t < 12.0 { let th = (0.2 + (speed - c.state().speed()) * 0.8).clamp(0.0, 0.55);
        c.step(DT, Controls { steer: t / 12.0 * 0.6 * 28.0 / 46.0, throttle: th, brake: 0.0 }); t += DT;
        let tel = c.telemetry(); beta = beta.max(tel.body_slip_deg.abs()); if tel.ay_g > pk { pk = tel.ay_g; bal = -tel.balance; } }
    (pk, bal, beta)
}
fn main() {
    let scales: Vec<f64> = std::env::args().skip(1).map(|a| a.parse().unwrap()).collect();
    for s in scales {
        let mut c = car(s);
        let tight = skidpad::tightest_line(c.params(), 0.0);
        let mut best = f64::MAX;
        for m in [0.25, 0.3, 0.4, 0.5] { if let Some((_, r)) = skidpad::fastest(c.as_mut(), tight + m, tight + m, 0.0) { best = best.min(r.time()); } }
        let mut line = format!("scale {s:.3} (front {:.3}): timed skidpad {best:.3} s", 0.80 * s);
        for sp in [10.0, 15.0, 20.0] { let (pk, bal, beta) = ramp(s, sp); line += &format!(" | {sp}: {pk:.2}g bal {bal:+.2} b {beta:.1}"); }
        println!("{line}");
    }
}
