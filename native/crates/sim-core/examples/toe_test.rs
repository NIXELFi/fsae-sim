//! Replicate the SDM26 toe test (Suspension Design Report 6.7) on the timed
//! skidpad: "1.1/-0.5 F/R toe setup was 0.11 s quicker than the baseline
//! 0.5/-0.5", "0.5/-1.4 ... 0.05 s quicker". Skidpad setup (OVDR 5.3): RSD
//! 47 %, diff preload 0. Run under each reading of the notation.
#[path = "../tests/common/skidpad.rs"]
#[allow(dead_code)]
mod skidpad;
use sim_core::prelude::*;
fn time_off(f: f64, r: f64, off: f64) -> f64 {
    let mut p = sdm26();
    p.roll.rsd_front = 0.47;
    p.diff.preload_nm = 0.0;
    p.suspension.toe_in_front_deg = f;
    p.suspension.toe_in_rear_deg = r;
    let mut c = build(Fidelity::DoubleTrack, Chassis::new(p, Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())));
    let tight = skidpad::tightest_line(c.params(), 0.0);
    let mut best = f64::MAX;
    for i in 0..8 {
        let r_line = tight + 0.2 + off + 0.05 * i as f64;
        if let Some((_, res)) = skidpad::fastest(c.as_mut(), r_line, r_line, 0.0) { best = best.min(res.time()); }
    }
    best
}
fn time(f: f64, r: f64) -> f64 { time_off(f, r, 0.0) }
fn main() {
    if std::env::var("NOISE").is_ok() {
        for (f, r) in [(0.5, -0.5), (1.1, -0.5)] { let v: Vec<String> = [0.0, 0.0125, 0.025, 0.0375].iter().map(|&o| format!("{:.3}", time_off(f, r, o))).collect(); println!("{f}/{r}: {}", v.join(" ")); }
        return;
    }
    let sets = [(0.5, -0.5), (1.1, -0.5), (0.5, -1.4)];
    for (name, scale, rsign) in [("per wheel, '-' = toe-out", 1.0, 1.0), ("per wheel, '-' = toe-in", 1.0, -1.0), ("total (half per wheel), '-' = out", 0.5, 1.0)] {
        let t: Vec<f64> = sets.iter().map(|&(f, r)| time(f * scale, r * scale * rsign)).collect();
        println!("{name:<36} base {:.3} | 1.1/-0.5 {:+.3} s (real -0.11) | 0.5/-1.4 {:+.3} s (real -0.05)", t[0], t[1] - t[0], t[2] - t[0]);
    }
}
