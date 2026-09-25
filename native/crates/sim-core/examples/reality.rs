//! Reality scorecard: both vehicle models against every number the real
//! SDM26 has given us, one table.
//!
//!   cargo run --release -p sim-core --example reality [-- skidpad]
//!
//! The real numbers and where they come from (2026-09-24 fidelity audit,
//! sim/docs/analysis-2026-09-24/):
//!
//! * The logged test days ran WITHOUT anti-roll bars: the shock pots give
//!   ~1110 N.m/deg of suspension roll stiffness, the coil springs alone
//!   (Chassis Design Binder 1122.3), and the 3/21 suspension slides say the
//!   5.01 s skidpad was "no arbs". So every comparison against a log is made
//!   on the SPRINGS-ONLY car (`springs_only`), and the default (bars on) car
//!   is printed beside it.
//! * Skidpad: 5.01 s best on the 3/14 test (no bars), 5.088 s at competition.
//! * Understeer gradient from the logs ~0.65 deg/g road wheel (95 %
//!   -0.2..1.6); low-frequency yaw gain 4.6-5.0 deg/s per road degree at the
//!   tryout speeds (~10 m/s).
//! * Roll to ground 0.82-0.98 deg/g (springs + tyres); pitch in braking
//!   0.38 deg/g at the pots (suspension only).
//! * Steer -> roll delay, analog channel to analog channel: 61-69 ms median.
//! * 75 m accel (5/3, ECU): 0-20 m 1.70-1.85 s; 20-75 m 2.7-2.8 s with a
//!   0.3-0.4 s mis-shift in it; competition 4.693 s.
//! * Upshift (ECU, 11 clean shifts): 50 ms press -> cut, 175-200 ms cut,
//!   crank falls 10,000-14,300 rpm/s, lands -1..+4 % of sync.
//! * Braking: drivers' best 1.39 g (not a limit stop).
use sim_core::prelude::*;
use std::f64::consts::PI;

const DT: f64 = 1.0 / 500.0;
/// The yaw-rate loop's gain on the steer command, as tests/validation.rs.
const PI_GAIN: f64 = 28.0 / 46.0;

#[derive(Clone, Copy, PartialEq)]
enum Cfg {
    /// The shipped car, bars on.
    Default,
    /// Coil springs only, as every logged test day ran.
    SpringsOnly,
}

fn params(cfg: Cfg) -> VehicleParams {
    let mut p = sdm26();
    // The wheel profile drives with no servo lag; the tests' 60 ms default
    // is a keyboard filter, not the car.
    p.steering.lag_s = 0.0;
    // Calibration sweeps: FS / RS override the 4-wheel grip scales.
    if let Ok(v) = std::env::var("FS") {
        p.suspension.front_grip_scale = v.parse().unwrap();
    }
    if let Ok(v) = std::env::var("RS") {
        p.suspension.rear_grip_scale = v.parse().unwrap();
    }
    if cfg == Cfg::SpringsOnly {
        springs_only(&mut p);
    }
    p
}

/// The car with the bars off: roll stiffness from the wheel rates alone,
/// in series with the tyres for the gradient.
fn springs_only(p: &mut VehicleParams) {
    let sp = &p.suspension;
    let per_axle = |k_wheel: f64, t: f64| k_wheel * t * t * 0.5; // N.m/rad
    let kf = per_axle(sp.wheel_rate_front_n_m, p.track_front_m);
    let kr = per_axle(sp.wheel_rate_rear_n_m, p.track_rear_m);
    let kt_f = per_axle(sp.tyre_rate_n_m, p.track_front_m);
    let kt_r = per_axle(sp.tyre_rate_n_m, p.track_rear_m);
    let series = |a: f64, b: f64| 1.0 / (1.0 / a + 1.0 / b);
    let k_total = series(kf, kt_f) + series(kr, kt_r);
    p.roll.rsd_front = kf / (kf + kr);
    let ms = p.sprung_mass();
    p.suspension.roll_gradient_deg_g = (ms * G * p.roll_arm() / k_total).to_degrees();
}

fn car(f: Fidelity, cfg: Cfg) -> Box<dyn Solver> {
    build(f, Chassis::new(params(cfg), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())))
}

fn lock_scale(c: &dyn Solver) -> f64 {
    // Controls.steer is a fraction of max_steer; road radians -> command.
    1.0 / c.params().steering.max_steer_rad
}

fn gear_for(v: f64) -> usize {
    let ratios = [2.75, 2.0, 1.667, 1.444, 1.304, 1.208];
    let (mut best, mut bd) = (0usize, f64::MAX);
    for (g, r) in ratios.iter().enumerate() {
        let rpm = v / 0.2 * 2.111 * r * 3.0 * 60.0 / (2.0 * PI);
        if rpm < 13000.0 && (rpm - 9500.0).abs() < bd {
            bd = (rpm - 9500.0).abs();
            best = g;
        }
    }
    best
}

fn start(f: Fidelity, cfg: Cfg, v: f64) -> Box<dyn Solver> {
    start_lag(f, cfg, v, 0.0)
}

/// A closed-loop robot needs the tests' 60 ms of hands (tests/validation.rs):
/// with none its yaw-rate loop rings against the tyre lag.
fn start_robot(f: Fidelity, cfg: Cfg, v: f64) -> Box<dyn Solver> {
    start_lag(f, cfg, v, 0.06)
}

fn start_lag(f: Fidelity, cfg: Cfg, v: f64, lag: f64) -> Box<dyn Solver> {
    let mut c = car(f, cfg);
    c.params_mut().steering.lag_s = lag;
    c.reset(0.0, 0.0, 0.0, v);
    c.powertrain_mut().set_gear(gear_for(v));
    c.powertrain_mut().sync_to_wheel(v / 0.2);
    c
}

/// Throttle that holds a speed (P + feed-forward), for constant-speed tests.
fn hold(c: &dyn Solver, v: f64) -> f64 {
    (0.25 + (v - c.state().speed()) * 0.8).clamp(0.0, 1.0)
}

/// Steady circle at radius r: fastest speed holding it within 2 %.
fn circle(f: Fidelity, cfg: Cfg, r: f64) -> f64 {
    let mut best = 0.0;
    let mut v = 8.0;
    while v < 14.0 {
        let mut c = start_robot(f, cfg, v);
        let ls = lock_scale(c.as_ref());
        let ff = (c.params().wheelbase_m / r) * ls;
        let (mut integ, mut sr, mut n, mut blew) = (0.0, 0.0, 0, false);
        for i in 0..6000 {
            let s = c.state();
            let err = s.speed() / r - s.r;
            integ = (integ + err * DT).clamp(-0.5, 0.5);
            let steer = (ff + (6.0 * err + 4.0 * integ) * PI_GAIN).clamp(-PI_GAIN, PI_GAIN);
            let th = (0.3 + (v - s.speed()) * 0.6).clamp(0.0, 1.0);
            c.step(DT, Controls { steer, throttle: th, brake: 0.0 });
            if c.telemetry().body_slip_deg.abs() > 30.0 {
                blew = true;
                break;
            }
            if i > 4000 {
                let s = c.state();
                sr += s.speed() / s.r.abs().max(1e-4);
                n += 1;
            }
        }
        let mr = if n > 0 { sr / n as f64 } else { 1e9 };
        if !blew && (mr - r).abs() / r < 0.02 && (c.state().speed() - v).abs() < 0.3 {
            best = v;
        }
        v += 0.05;
    }
    best * best / r / G
}

/// Slow steer ramp at constant speed: (understeer gradient deg/g over
/// 0.2-0.8 g, peak ay g).
fn ramp(f: Fidelity, cfg: Cfg, v: f64) -> (f64, f64) {
    let mut c = start(f, cfg, v);
    let ls = lock_scale(c.as_ref());
    let (mut t, mut pk) = (0.0, 0.0f64);
    let (mut sx, mut sy, mut sxx, mut sxy, mut n) = (0.0, 0.0, 0.0, 0.0, 0.0);
    while t < 20.0 {
        let road = (t / 20.0) * 28f64.to_radians();
        let th = hold(c.as_ref(), v);
        c.step(DT, Controls { steer: road * ls, throttle: th, brake: 0.0 });
        t += DT;
        let tel = c.telemetry();
        if tel.body_slip_deg.abs() > 20.0 {
            break;
        }
        pk = pk.max(tel.ay_g);
        if (0.2..0.8).contains(&tel.ay_g) {
            let x = tel.ay_g;
            let y = tel.steer_rad.to_degrees();
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
            n += 1.0;
        }
    }
    let slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    let l = c.params().wheelbase_m;
    let v2 = c.state().speed().powi(2).max(1.0);
    let k = slope - (l * G / v2).to_degrees();
    (k, pk)
}

/// Road-wheel sine at `hz`, `amp` deg: (yaw gain deg/s/deg, steer->yaw ms,
/// steer->ay ms, steer->roll ms).
fn sine(f: Fidelity, cfg: Cfg, v: f64, hz: f64, amp: f64) -> (f64, f64, f64, f64) {
    let mut c = start(f, cfg, v);
    let ls = lock_scale(c.as_ref());
    let w = 2.0 * PI * hz;
    let cycles = 6.0;
    let settle = 2.0 / hz;
    let total = settle + cycles / hz;
    let mut t = 0.0;
    // Fourier projections of steer, yaw, ay, roll on sin/cos at w.
    let mut acc = [[0.0f64; 2]; 4];
    while t < total {
        let road = amp.to_radians() * (w * t).sin();
        let th = hold(c.as_ref(), v);
        c.step(DT, Controls { steer: road * ls, throttle: th, brake: 0.0 });
        t += DT;
        if t > settle {
            let tel = c.telemetry();
            let vals = [tel.steer_rad.to_degrees(), tel.yaw_rate_deg_s, tel.ay_g, tel.roll_deg];
            for (k, x) in vals.iter().enumerate() {
                acc[k][0] += x * (w * t).sin() * DT;
                acc[k][1] += x * (w * t).cos() * DT;
            }
        }
    }
    let mag = |a: [f64; 2]| a[0].hypot(a[1]);
    let ph = |a: [f64; 2]| a[1].atan2(a[0]);
    let lag_ms = |k: usize| {
        let mut d = ph(acc[0]) - ph(acc[k]);
        while d < -PI {
            d += 2.0 * PI;
        }
        while d > PI {
            d -= 2.0 * PI;
        }
        d / w * 1000.0
    };
    let roll_ms = if mag(acc[3]) > 1e-6 { lag_ms(3) } else { f64::NAN };
    (mag(acc[1]) / mag(acc[0]), lag_ms(1), lag_ms(2), roll_ms)
}

/// Steady cornering near `ay` g at 15 m/s: roll deg/g. Pitch in a straight
/// 1 g stop: deg/g.
fn roll_pitch(f: Fidelity, cfg: Cfg) -> (f64, f64) {
    if f != Fidelity::DoubleTrack {
        return (f64::NAN, f64::NAN);
    }
    let v = 15.0;
    let r = v * v / (1.0 * G);
    let mut c = start_robot(f, cfg, v);
    let ls = lock_scale(c.as_ref());
    let ff = (c.params().wheelbase_m / r) * ls;
    let mut integ = 0.0;
    let (mut roll, mut ay, mut n) = (0.0, 0.0, 0.0);
    for i in 0..3000 {
        let s = c.state();
        let err = s.speed() / r - s.r;
        integ = (integ + err * DT).clamp(-0.5, 0.5);
        let steer = (ff + (6.0 * err + 4.0 * integ) * PI_GAIN).clamp(-PI_GAIN, PI_GAIN);
        c.step(DT, Controls { steer, throttle: hold(c.as_ref(), v), brake: 0.0 });
        if i > 2000 {
            let tel = c.telemetry();
            roll += tel.roll_deg;
            ay += tel.ay_g;
            n += 1.0;
        }
    }
    let roll_g = (roll / n) / (ay / n);
    // Pitch: brake at a pedal that gives about 1 g, read the settled pitch.
    let mut b = start(f, cfg, 22.0);
    let (mut p, mut ax, mut m) = (0.0, 0.0, 0.0);
    for i in 0..400 {
        b.step(DT, Controls { steer: 0.0, throttle: 0.0, brake: 0.55 });
        if i > 200 {
            let tel = b.telemetry();
            p += tel.pitch_deg;
            ax += tel.ax_g;
            m += 1.0;
        }
    }
    (roll_g.abs(), (p / m / (ax / m)).abs())
}

/// Standing 75 m the way the car does it: launch control held at the real
/// ECU's ~8000 rpm, dumped, flat out, upshifts at the best rpm.
/// (t20, t75, trap km/h).
fn accel(f: Fidelity, cfg: Cfg) -> (f64, f64, f64) {
    let mut c = car(f, cfg);
    c.reset(0.0, 0.0, 0.0, 0.0);
    if let Some(e) = c.powertrain_mut().as_any_mut().and_then(|a| a.downcast_mut::<GearedEngine>()) {
        e.launch_rpm = 8000.0;
    }
    c.powertrain_mut().set_launch(true);
    for _ in 0..1000 {
        c.step(DT, Controls { steer: 0.0, throttle: 1.0, brake: 1.0 });
    }
    c.powertrain_mut().set_launch(false);
    // Staged 0.3 m behind the line (D.9.2.3); the clock starts at the line.
    let x0 = c.state().x + 0.3;
    let (mut t, mut t0, mut t20) = (0.0, f64::NAN, f64::NAN);
    while t < 10.0 && c.state().x - x0 < 75.0 {
        let tel = c.telemetry();
        let up = c.powertrain_mut().optimal_upshift_rpm();
        {
            let pt = c.powertrain_mut();
            if pt.can_shift() && tel.engine_rpm > up && tel.gear < 5 {
                pt.shift_up();
            }
        }
        c.step(DT, Controls { steer: 0.0, throttle: 1.0, brake: 0.0 });
        t += DT;
        if t0.is_nan() && c.state().x >= x0 {
            t0 = t;
        }
        if t20.is_nan() && c.state().x - x0 >= 20.0 {
            t20 = t;
        }
    }
    (t20 - t0, t - t0, c.state().speed() * 3.6)
}

/// A 2->3 upshift at WOT: (crank fall rpm/s over the cut, rpm at the
/// engagement vs sync %, cut length ms, road speed 0.6 s after the press m/s).
fn upshift(f: Fidelity) -> (f64, f64, f64, f64) {
    let mut c = car(f, Cfg::Default);
    let v = 11_000.0 / 60.0 * 2.0 * PI / (2.111 * 2.0 * 3.0) * 0.2;
    c.reset(0.0, 0.0, 0.0, v);
    c.powertrain_mut().set_gear(1);
    c.powertrain_mut().sync_to_wheel(v / 0.2);
    if let Ok(x) = std::env::var("CUT") {
        if let Some(e) = c.powertrain_mut().as_any_mut().and_then(|a| a.downcast_mut::<GearedEngine>()) {
            e.shift_time_s = x.parse().unwrap();
        }
    }
    for _ in 0..50 {
        c.step(DT, Controls { steer: 0.0, throttle: 1.0, brake: 0.0 });
    }
    c.powertrain_mut().shift_up();
    let (mut t, mut t_cut0, mut rpm_cut0, mut t_cut1, mut rpm_cut1) = (0.0, f64::NAN, 0.0, f64::NAN, 0.0);
    let mut prev_gear = 1usize;
    let mut sync_pct = f64::NAN;
    let mut prev_shifting = false;
    let mut v06 = 0.0;
    while t < 0.8 {
        c.step(DT, Controls { steer: 0.0, throttle: 1.0, brake: 0.0 });
        t += DT;
        let tel = c.telemetry();
        if tel.shifting && !prev_shifting {
            t_cut0 = t;
            rpm_cut0 = tel.engine_rpm;
        }
        if tel.shifting {
            // The last reading still inside the cut, before the new gear
            // engages and the lock moves the crank.
            t_cut1 = t;
            rpm_cut1 = tel.engine_rpm;
            let n = 2.111 * 1.667 * 3.0;
            let sync = tel.wheel_omega_rear * n * 60.0 / (2.0 * PI);
            sync_pct = (tel.engine_rpm / sync - 1.0) * 100.0;
        }
        let _ = &mut prev_gear;
        if std::env::var("TRACE").is_ok() && t < 0.35 {
            println!("  t {:.3} rpm {:.0} gear {} shifting {} wR {:.2} u {:.3}", t, tel.engine_rpm, tel.gear, tel.shifting, tel.wheel_omega_rear, c.state().u);
        }
        prev_shifting = tel.shifting;
        if (t - 0.6).abs() < DT * 0.5 {
            v06 = c.state().speed();
        }
    }
    let fall = (rpm_cut0 - rpm_cut1) / (t_cut1 - t_cut0);
    (fall, sync_pct, (t_cut1 - t_cut0) * 1000.0, v06)
}

/// Full-pedal stop from 25 m/s: (distance m, mean g).
fn stop(f: Fidelity, cfg: Cfg, pedal: f64) -> (f64, f64) {
    let mut c = start(f, cfg, 25.0);
    let mut t = 0.0;
    while t < 6.0 && c.state().speed() > 0.5 {
        c.step(DT, Controls { steer: 0.0, throttle: 0.0, brake: pedal });
        t += DT;
    }
    (c.state().x, (25.0 - 0.5) / t / G)
}

/// Brake in a limit corner: steady at ~1.5 g on 15 m/s, hold the steer, a
/// brake step. Most negative body slip over 2 s (deg).
fn trail(f: Fidelity, cfg: Cfg, brake: f64) -> f64 {
    let v = 13.5;
    let r = v * v / (1.45 * G);
    let mut c = start_robot(f, cfg, v);
    let ls = lock_scale(c.as_ref());
    let ff = (c.params().wheelbase_m / r) * ls;
    let mut integ = 0.0;
    let mut steer = 0.0;
    for _ in 0..2500 {
        let s = c.state();
        let err = s.speed() / r - s.r;
        integ = (integ + err * DT).clamp(-0.5, 0.5);
        steer = (ff + (6.0 * err + 4.0 * integ) * PI_GAIN).clamp(-PI_GAIN, PI_GAIN);
        c.step(DT, Controls { steer, throttle: hold(c.as_ref(), v), brake: 0.0 });
    }
    let mut worst = 0.0f64;
    for _ in 0..1000 {
        c.step(DT, Controls { steer, throttle: 0.0, brake });
        worst = worst.min(c.telemetry().body_slip_deg);
        if c.state().speed() < 1.0 {
            break;
        }
    }
    worst
}

/// Full lock at walking pace: path radius over the kinematic L / tan(delta).
fn low_speed(f: Fidelity) -> f64 {
    let mut c = start(f, Cfg::Default, 1.0);
    let steer = 25f64.to_radians() * lock_scale(c.as_ref());
    let (mut sr, mut n) = (0.0, 0.0);
    for i in 0..5000 {
        let th = (0.1 + (1.0 - c.state().speed()) * 0.5).clamp(0.0, 1.0);
        c.step(DT, Controls { steer, throttle: th, brake: 0.0 });
        if i > 3000 {
            let s = c.state();
            sr += s.speed() / s.r.abs().max(1e-4);
            n += 1.0;
        }
    }
    (sr / n) / (c.params().wheelbase_m / 25f64.to_radians().tan())
}

fn main() {
    let skid = std::env::args().any(|a| a == "skidpad");
    if std::env::args().any(|a| a == "grip") {
        // Quick calibration view: the 4-wheel, bars on.
        let f = Fidelity::DoubleTrack;
        let c = Cfg::Default;
        let (k10, p10) = ramp(f, c, 10.0);
        let (k15, p15) = ramp(f, c, 15.0);
        let (k20, p20) = ramp(f, c, 20.0);
        println!(
            "circle {:.3} g | ramp 10 {p10:.3} (K {k10:.2}) 15 {p15:.3} (K {k15:.2}) 20 {p20:.3} (K {k20:.2})",
            circle(f, c, 8.6)
        );
        return;
    }
    let models = [("bicycle", Fidelity::Bicycle), ("4-wheel", Fidelity::DoubleTrack)];
    println!("{:<34} {:>11} {:>11} {:>11} {:>11}   real", "", "bike/bars", "bike/sprng", "4w/bars", "4w/sprng");
    let cols: Vec<(Fidelity, Cfg)> = models
        .iter()
        .flat_map(|(_, f)| [(*f, Cfg::Default), (*f, Cfg::SpringsOnly)])
        .collect();
    let row = |name: &str, real: &str, g: &dyn Fn(Fidelity, Cfg) -> f64, prec: usize| {
        let mut s = format!("{name:<34}");
        for (f, cfg) in &cols {
            s += &format!(" {:>11.*}", prec, g(*f, *cfg));
        }
        println!("{s}   {real}");
    };
    row("steady circle 8.6 m (g)", "~1.45 (5.01 s run)", &|f, c| circle(f, c, 8.6), 3);
    row("  -> lap time 8.6 m (s)", "", &|f, c| 2.0 * PI * 8.6 / (circle(f, c, 8.6) * G * 8.6).sqrt(), 3);
    for v in [10.0, 15.0, 20.0] {
        row(&format!("understeer grad @{v} (deg/g)"), "0.65 (-0.2..1.6)", &|f, c| ramp(f, c, v).0, 2);
        row(&format!("ramp peak ay @{v} (g)"), "", &|f, c| ramp(f, c, v).1, 3);
    }
    row("yaw gain 0.2 Hz @10 (deg/s/deg)", "4.6-5.0", &|f, c| sine(f, c, 10.0, 0.2, 1.0).0, 2);
    row("steer->yaw 1 Hz @10 (ms)", "", &|f, c| sine(f, c, 10.0, 1.0, 1.0).1, 1);
    row("steer->ay 1 Hz @10 (ms)", "", &|f, c| sine(f, c, 10.0, 1.0, 1.0).2, 1);
    row("steer->roll 1 Hz @10 (ms)", "61-69 (driver input)", &|f, c| sine(f, c, 10.0, 1.0, 1.0).3, 1);
    row("roll to ground (deg/g)", "0.82-0.98 (springs)", &|f, c| roll_pitch(f, c).0, 3);
    row("pitch braking 1 g (deg/g)", "0.38 pots / ~0.47 w tyres", &|f, c| roll_pitch(f, c).1, 3);
    row("accel 0-20 m (s)", "1.70-1.85", &|f, c| accel(f, c).0, 3);
    row("accel 20-75 m (s)", "2.3-2.5 (clean shift)", &|f, c| {
        let (a, b, _) = accel(f, c);
        b - a
    }, 3);
    row("accel 75 m (s)", "4.69 comp (spin)", &|f, c| accel(f, c).1, 3);
    row("accel trap (km/h)", "83-90", &|f, c| accel(f, c).2, 1);
    row("stop 25 m/s full pedal (m)", "", &|f, c| stop(f, c, 1.0).0, 2);
    row("stop 25 m/s 0.8 pedal (g mean)", "best driver 1.39", &|f, c| stop(f, c, 0.8).1, 3);
    for b in [0.15, 0.2, 0.3] {
        row(&format!("trail brake {b} min beta (deg)"), "stable", &|f, c| trail(f, c, b), 1);
    }
    for (name, f) in models {
        let (fall, sync, cut, v06) = upshift(f);
        println!(
            "{name:<8} 2->3 upshift: crank fall {fall:.0} rpm/s (real 10-14k), lands {sync:+.1} % of sync (real -1..+4), cut {cut:.0} ms (real 175-200 + 50 delay), v@0.6s {v06:.3} m/s"
        );
        println!("{name:<8} full-lock (25 deg) radius / kinematic at 1 m/s: {:.3}", low_speed(f));
    }
    if skid {
        #[path = "../tests/common/skidpad.rs"]
        #[allow(dead_code)]
        mod skidpad;
        for (name, f) in models {
            for cfg in [Cfg::Default, Cfg::SpringsOnly] {
                let mut c = car(f, cfg);
                c.params_mut().steering.lag_s = 0.06;
                let tight = skidpad::tightest_line(c.params(), 0.0);
                let mut best = f64::MAX;
                for m in [0.2, 0.3, 0.4] {
                    let r = tight + m;
                    for (rx, span) in [(r, 0.0), (r + 0.3, 1.0)] {
                        if let Some((_, res)) = skidpad::fastest(c.as_mut(), r, rx, span) {
                            best = best.min(res.time());
                        }
                    }
                }
                println!("{name:<8} {} timed skidpad (robot): {best:.3} s  (real 5.01 test / 5.088 comp)", if cfg == Cfg::Default { "bars   " } else { "springs" });
            }
        }
    }
}
