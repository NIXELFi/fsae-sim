//! Peak lateral capability of both models on the same steer ramp, and the
//! steady-state limit on the skidpad circle.
use sim_core::prelude::*;
fn car(f: Fidelity, fs: f64, rs: f64) -> Box<dyn Solver> {
    let mut p = sdm26(); p.suspension.front_grip_scale = fs; p.suspension.rear_grip_scale = rs;
    build(f, Chassis::new(p, Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())))
}
fn ramp(f: Fidelity, fs: f64, rs: f64, speed: f64) -> (f64, f64) {
    const DT: f64 = 1.0 / 200.0;
    let mut c = car(f, fs, rs); c.reset(0.0, 0.0, 0.0, speed);
    c.powertrain_mut().set_gear(if speed > 17.0 { 2 } else { 1 }); c.powertrain_mut().sync_to_wheel(speed / 0.2);
    let (mut t, mut pk, mut bal) = (0.0, 0.0f64, 0.0);
    while t < 16.0 { let th = (0.2 + (speed - c.state().speed()) * 0.8).clamp(0.0, 0.55);
        c.step(DT, Controls { steer: t / 16.0 * 0.7 * 28.0 / 46.0, throttle: th, brake: 0.0 }); t += DT;
        let tel = c.telemetry(); if tel.body_slip_deg.abs() > 20.0 { break; } if tel.ay_g > pk { pk = tel.ay_g; bal = -tel.balance; } }
    (pk, bal)
}
/// Steady circle at radius r: fastest speed holding the radius within 2 %.
fn circle(f: Fidelity, fs: f64, rs: f64, r: f64) -> (f64, f64) {
    const DT: f64 = 1.0 / 500.0;
    let mut best = 0.0; let mut v = 8.0;
    while v < 14.0 {
        let mut c = car(f, fs, rs); c.reset(0.0, 0.0, 0.0, v);
        let ls = 28.0 / 46.0; let ff = c.params().wheelbase_m / r / c.params().steering.max_steer_rad;
        let (mut integ, mut sr, mut n, mut blew) = (0.0, 0.0, 0, false);
        for i in 0..6000 { let s = c.state(); let err = s.speed() / r - s.r; integ = (integ + err * DT).clamp(-0.5, 0.5);
            let steer = (ff + (6.0 * err + 4.0 * integ) * ls).clamp(-ls, ls); let th = (0.3 + (v - s.speed()) * 0.6).clamp(0.0, 1.0);
            c.step(DT, Controls { steer, throttle: th, brake: 0.0 });
            if c.telemetry().body_slip_deg.abs() > 30.0 { blew = true; break; }
            if i > 4000 { let s = c.state(); sr += s.speed() / s.r.abs().max(1e-4); n += 1; } }
        let mr = if n > 0 { sr / n as f64 } else { 1e9 };
        if !blew && (mr - r).abs() / r < 0.02 && (c.state().speed() - v).abs() < 0.3 { best = v; }
        v += 0.05;
    }
    (best, best * best / r / 9.81)
}
fn main() {
    let (b11, _) = ramp(Fidelity::Bicycle, 1.0, 1.0, 11.0);
    let (b20, _) = ramp(Fidelity::Bicycle, 1.0, 1.0, 20.0);
    println!("bicycle            ramp 11: {b11:.3} g   ramp 20: {b20:.3} g");
    for (fs, rs) in [(1.08, 1.00), (1.06, 1.00), (1.05, 1.00), (1.04, 1.00), (1.03, 1.00), (1.05, 0.99), (1.04, 0.99), (1.03, 0.99)] {
        let (d11, bal11) = ramp(Fidelity::DoubleTrack, fs, rs, 11.0);
        let (d20, bal20) = ramp(Fidelity::DoubleTrack, fs, rs, 20.0);
        println!("4-wheel {fs:.3}/{rs:.3} ramp 11: {d11:.3} g ({:+.1}%, bal {bal11:+.2})   ramp 20: {d20:.3} g ({:+.1}%, bal {bal20:+.2})",
            (d11 / b11 - 1.0) * 100.0, (d20 / b20 - 1.0) * 100.0);
    }
}
