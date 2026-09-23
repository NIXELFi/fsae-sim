//! Steady skidpad-circle limit in BOTH directions, both models, the shipped
//! grip scales: the fastest speed that holds radius r within 2 %.
use sim_core::prelude::*;
fn car(f: Fidelity) -> Box<dyn Solver> {
    let p = sdm26();
    build(f, Chassis::new(p, Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())))
}
fn circle(f: Fidelity, r: f64, dir: f64) -> (f64, f64, f64, f64) {
    const DT: f64 = 1.0 / 500.0;
    let (mut best, mut beta, mut af, mut ar) = (0.0, 0.0, 0.0, 0.0);
    let mut v = 9.0;
    while v < 13.5 {
        let mut c = car(f); c.reset(0.0, 0.0, 0.0, v);
        let ls = 28.0 / 46.0; let ff = c.params().wheelbase_m / r / c.params().steering.max_steer_rad;
        let (mut integ, mut sr, mut n, mut blew) = (0.0, 0.0, 0, false);
        let (mut sb, mut sf, mut srr) = (0.0, 0.0, 0.0);
        for i in 0..6000 { let s = c.state(); let err = dir * s.speed() / r - s.r; integ = (integ + err * DT).clamp(-0.5, 0.5);
            let steer = (dir * ff + (6.0 * err + 4.0 * integ) * ls).clamp(-ls, ls); let th = (0.3 + (v - s.speed()) * 0.6).clamp(0.0, 1.0);
            c.step(DT, Controls { steer, throttle: th, brake: 0.0 });
            let t = c.telemetry();
            if t.body_slip_deg.abs() > 30.0 { blew = true; break; }
            if i > 4000 { let s = c.state(); sr += s.speed() / s.r.abs().max(1e-4); n += 1; sb += t.body_slip_deg; sf += (t.slip_deg[0] + t.slip_deg[1]) / 2.0; srr += (t.slip_deg[2] + t.slip_deg[3]) / 2.0; } }
        let mr = if n > 0 { sr / n as f64 } else { 1e9 };
        if !blew && (mr - r).abs() / r < 0.02 && (c.state().speed() - v).abs() < 0.3 {
            best = v; let k = n as f64; beta = sb / k; af = sf / k; ar = srr / k; }
        v += 0.05;
    }
    (best, beta, af, ar)
}
fn main() {
    let r = 8.7;
    for (name, f) in [("bicycle", Fidelity::Bicycle), ("4-wheel", Fidelity::DoubleTrack)] {
        for (side, dir) in [("left ", 1.0), ("right", -1.0)] {
            let (v, b, af, ar) = circle(f, r, dir);
            println!("{name:<8} {side} R {r}: {v:.2} m/s  {:.3} g  lap {:.3} s   beta {b:+.2}  aF {af:+.2}  aR {ar:+.2}",
                v * v / r / 9.81, 2.0 * std::f64::consts::PI * r / v.max(1e-6));
        }
    }
}
