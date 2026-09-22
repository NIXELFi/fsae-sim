//! The double-track + suspension model (beta): the same validation the
//! bicycle carries, plus what only this model has -- a body that rolls and
//! pitches, and camber.

use sim_core::prelude::*;

const DT: f64 = 1.0 / 60.0;

fn car() -> Box<dyn Solver> {
    build(
        Fidelity::DoubleTrack,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    )
}

fn lock_scale() -> f64 {
    28.0 / sdm26().steering.max_steer_rad.to_degrees()
}

/// The validation skidpad driver (tests/validation.rs), unchanged.
fn skidpad_speed(car: &mut dyn Solver) -> f64 {
    const R: f64 = 9.125;
    let mut best = 0.0;
    let mut target = 8.0;
    while target <= 16.0 {
        car.params_mut().roll.rsd_front = 0.46;
        car.reset(0.0, 0.0, 0.0, target);
        let ff = car.params().wheelbase_m / R / car.params().steering.max_steer_rad;
        let ls = lock_scale();
        let (mut integral, mut sum_r, mut n) = (0.0, 0.0, 0);
        let mut blew = false;
        for i in 0..6000 {
            let s = car.state();
            let err = s.speed() / R - s.r;
            integral = (integral + err * DT).clamp(-0.5, 0.5);
            let steer = (ff + (6.0 * err + 4.0 * integral) * ls).clamp(-ls, ls);
            let throttle = (0.3 + (target - s.speed()) * 0.6).clamp(0.0, 1.0);
            car.step(DT, Controls { steer, throttle, brake: 0.0 });
            if car.telemetry().body_slip_deg.abs() > 45.0 {
                blew = true;
                break;
            }
            if i > 4000 {
                let s = car.state();
                sum_r += s.speed() / s.r.abs().max(1e-4);
                n += 1;
            }
        }
        let mean_r = if n > 0 { sum_r / n as f64 } else { 1e9 };
        if !blew && (mean_r - R).abs() / R < 0.04 && (car.state().speed() - target).abs() < 0.5 {
            best = target;
        }
        target += 0.05;
    }
    best
}

#[test]
fn skidpad_is_in_the_real_cars_band() {
    let mut c = car();
    let v = skidpad_speed(c.as_mut());
    let lap = 2.0 * std::f64::consts::PI * 9.125 / v;
    println!("double-track skidpad {v:.2} m/s -> {lap:.3} s");
    assert!((4.85..=5.35).contains(&lap), "skidpad {lap:.3} s, real run 5.02 s");
}

#[test]
fn front_limits_first_and_the_car_pushes() {
    for speed in [10.0, 15.0, 20.0] {
        let mut c = car();
        c.reset(0.0, 0.0, 0.0, speed);
        let ratios = [2.75, 2.0, 1.667, 1.444, 1.304, 1.208];
        let (mut best, mut bd) = (0usize, f64::MAX);
        for (g, r) in ratios.iter().enumerate() {
            let rpm = speed / 0.2 * 2.111 * r * 3.0 * 60.0 / (2.0 * std::f64::consts::PI);
            if rpm < 14000.0 && (rpm - 9500.0).abs() < bd {
                bd = (rpm - 9500.0).abs();
                best = g;
            }
        }
        c.powertrain_mut().set_gear(best);
        c.powertrain_mut().sync_to_wheel(speed / 0.2);
        let ls = lock_scale();
        let (mut t, mut peak_ay, mut bal, mut beta) = (0.0, 0.0f64, 0.0, 0.0f64);
        while t < 12.0 {
            let throttle = (0.2 + (speed - c.state().speed()) * 0.8).clamp(0.0, 0.55);
            c.step(DT, Controls { steer: t / 12.0 * 0.6 * ls, throttle, brake: 0.0 });
            t += DT;
            let tel = c.telemetry();
            beta = beta.max(tel.body_slip_deg.abs());
            if tel.ay_g > peak_ay {
                peak_ay = tel.ay_g;
                bal = -tel.balance;
            }
        }
        println!("{speed} m/s: {peak_ay:.2} g, front-rear {bal:.2}, body slip {beta:.1}");
        assert!((1.35..=2.0).contains(&peak_ay), "{speed} m/s peak {peak_ay:.2} g");
        assert!(bal > 0.08, "{speed} m/s: rear-limited ({bal:.2})");
        assert!(beta < 12.0, "{speed} m/s: spun ({beta:.1} deg)");
    }
}

/// The team's setup knob has to work the textbook way here too -- near the
/// limit, which is where it works. At the bicycle test's 9 m/s (1.06 g) this
/// model's radius moves by under 0.1 % across 40-70 %: in the linear range
/// load transfer costs almost nothing, and the bicycle only showed it because
/// its 0.80 front factor keeps the front nearer saturation. At 11 m/s it moves
/// 9 %. (At 40 % front this model spins at 11 m/s; the bicycle does not.)
#[test]
fn front_roll_stiffness_adds_understeer() {
    let mut radii = Vec::new();
    for rsd in [0.46, 0.50, 0.60, 0.70] {
        let mut c = car();
        c.params_mut().roll.rsd_front = rsd;
        c.reset(0.0, 0.0, 0.0, 11.0);
        let (mut sum, mut n) = (0.0, 0);
        for i in 0..4000 {
            let throttle = (0.3 + (11.0 - c.state().speed()) * 0.6).clamp(0.0, 1.0);
            c.step(DT, Controls { steer: 0.42 * lock_scale(), throttle, brake: 0.0 });
            if i > 3000 {
                let s = c.state();
                sum += s.speed() / s.r.abs().max(1e-4);
                n += 1;
            }
        }
        radii.push(sum / n as f64);
    }
    println!("radii by front roll stiffness: {radii:?}");
    for w in radii.windows(2) {
        assert!(w[1] > w[0], "stiffening the front did not add understeer: {radii:?}");
    }
}

#[test]
fn accelerates_and_brakes_like_the_car() {
    let mut c = car();
    c.reset(0.0, 0.0, 0.0, 0.0);
    let mut t = 0.0;
    while t < 12.0 && c.state().x < 75.0 {
        let tel = c.telemetry();
        {
            let pt = c.powertrain_mut();
            if pt.can_shift() && tel.engine_rpm > 12_000.0 && tel.gear < 5 {
                pt.shift_up();
            }
        }
        let over = c.telemetry().kappa[RL].max(c.telemetry().kappa[RR]) - 0.13;
        let throttle = (1.0 - over * 8.0).clamp(0.15, 1.0);
        c.step(DT, Controls { steer: 0.0, throttle, brake: 0.0 });
        t += DT;
    }
    println!("75 m in {t:.3} s");
    assert!((4.0..=5.2).contains(&t), "75 m in {t:.2} s");

    let mut c = car();
    c.powertrain_mut().set_gear(3);
    c.reset(0.0, 0.0, 0.0, 25.0);
    let mut peak = 0.0f64;
    let mut t = 0.0;
    while t < 6.0 && c.state().speed() > 0.5 {
        c.step(DT, Controls { steer: 0.0, throttle: 0.0, brake: 1.0 });
        peak = peak.min(c.telemetry().ax_g);
        t += DT;
    }
    let dist = c.state().x;
    println!("stopped in {dist:.2} m, peak {:.2} g", -peak);
    assert!((18.0..=40.0).contains(&dist), "stopping distance {dist:.1} m");
}

/// The body reproduces the gradients its stiffness is derived from
/// (`SuspensionParams`, with tyres). A touch more roll, because this model
/// also has gravity's moment as the body leans.
#[test]
fn body_rolls_and_pitches_by_the_team_gradients() {
    let mut c = car();
    c.powertrain_mut().set_gear(2);
    c.reset(0.0, 0.0, 0.0, 15.0);
    c.powertrain_mut().sync_to_wheel(75.0);
    for _ in 0..2500 {
        let th = (0.2 + (15.0 - c.state().u) * 0.5).clamp(0.0, 1.0);
        c.step(0.001, Controls { steer: 3.0 / 46.0, throttle: th, brake: 0.0 });
    }
    let t = c.telemetry();
    let roll_grad = t.roll_deg / t.ay_g;
    println!("roll {:.3} deg at {:.3} g -> {roll_grad:.3} deg/g", t.roll_deg, t.ay_g);
    let want = sdm26().suspension.roll_gradient_deg_g;
    assert!((want..=want * 1.03).contains(&roll_grad), "roll gradient {roll_grad:.3} deg/g, want {want}");

    let mut c = car();
    c.powertrain_mut().set_gear(3);
    c.reset(0.0, 0.0, 0.0, 20.0);
    c.powertrain_mut().sync_to_wheel(100.0);
    for _ in 0..700 {
        c.step(0.001, Controls { steer: 0.0, throttle: 0.0, brake: 0.6 });
    }
    let t = c.telemetry();
    let pitch_grad = t.pitch_deg / -t.ax_g;
    println!("pitch {:.3} deg at {:.3} g -> {pitch_grad:.3} deg/g", t.pitch_deg, t.ax_g);
    let want = sdm26().suspension.pitch_gradient_deg_g;
    assert!((want * 0.97..=want * 1.03).contains(&pitch_grad), "pitch gradient {pitch_grad:.3} deg/g, want {want}");
}

/// Roll is a damped mode, not an instant: after a step steer it lags and
/// settles, and it does not ring.
#[test]
fn roll_lags_and_settles() {
    let mut c = car();
    c.powertrain_mut().set_gear(2);
    c.reset(0.0, 0.0, 0.0, 15.0);
    c.powertrain_mut().sync_to_wheel(75.0);
    let mut hist = Vec::new();
    for _ in 0..1500 {
        let th = (0.2 + (15.0 - c.state().u) * 0.5).clamp(0.0, 1.0);
        c.step(0.001, Controls { steer: 3.0 / 46.0, throttle: th, brake: 0.0 });
        hist.push(c.telemetry().roll_deg);
    }
    let fin = *hist.last().unwrap();
    let peak = hist.iter().cloned().fold(0.0f64, f64::max);
    let at_20ms = hist[19];
    println!("roll at 20 ms {at_20ms:.3}, peak {peak:.3}, final {fin:.3} deg");
    assert!(fin > 0.2, "no roll in a left turn ({fin:.3})");
    assert!(at_20ms < 0.5 * fin, "roll is not lagging ({at_20ms:.3} of {fin:.3})");
    assert!(peak < fin * 1.10, "roll rings ({peak:.3} peak vs {fin:.3})");
}

/// Camber signs, in a left turn: the body leans right, so both rears lean
/// toward the outside; static negative camber leaves the loaded outer rear
/// near upright and the inner rear leaning out.
#[test]
fn camber_follows_the_body_the_right_way() {
    let mut c = car();
    c.powertrain_mut().set_gear(2);
    c.reset(0.0, 0.0, 0.0, 15.0);
    c.powertrain_mut().sync_to_wheel(75.0);
    for _ in 0..2500 {
        let th = (0.2 + (15.0 - c.state().u) * 0.5).clamp(0.0, 1.0);
        c.step(0.001, Controls { steer: 3.0 / 46.0, throttle: th, brake: 0.0 });
    }
    let t = c.telemetry();
    println!("camber {:?}, roll {:.3}", t.camber_deg, t.roll_deg);
    // Lean-left convention: static -0.7 deg is -0.7 on the left, +0.7 on the right.
    assert!(t.camber_deg[RL] < -0.7, "inner rear should lean further out: {:?}", t.camber_deg);
    assert!(t.camber_deg[RR] < 0.7 && t.camber_deg[RR] > -0.3, "outer rear: {:?}", t.camber_deg);
}

/// The friction-brake fix holds in this model too.
#[test]
fn a_braked_car_does_not_creep() {
    let mut c = car();
    c.reset(0.0, 0.0, 0.0, 0.0);
    c.powertrain_mut().set_launch(true);
    for _ in 0..3000 {
        c.step(0.001, Controls { steer: 0.0, throttle: 1.0, brake: 1.0 });
    }
    assert!(c.state().x.abs() < 1e-6, "crept {:.4} m", c.state().x);
}

/// Steering feel exists here now: a left turn makes a centring (negative)
/// kingpin torque.
#[test]
fn steering_torque_centres() {
    let mut c = car();
    c.powertrain_mut().set_gear(2);
    c.reset(0.0, 0.0, 0.0, 12.0);
    c.powertrain_mut().sync_to_wheel(60.0);
    for _ in 0..2000 {
        let th = (0.2 + (12.0 - c.state().u) * 0.5).clamp(0.0, 1.0);
        c.step(0.001, Controls { steer: 5.0 / 46.0, throttle: th, brake: 0.0 });
    }
    let t = c.telemetry();
    println!("kingpin {:.1} N.m, rim {:.2} N.m at {:.2} g", t.kingpin_torque_nm, t.rim_torque_nm, t.ay_g);
    assert!(t.kingpin_torque_nm < -5.0);
}
