//! The three events SDM has real stopwatch numbers for, plus the cross-checks
//! that keep the fidelity ladder honest.
//!
//! This is the JS `tools/validate.js` harness turned into actual tests. If a
//! change to the tyre or powertrain moves these, the change is wrong until
//! proven otherwise.

use sim_core::prelude::*;

const DT: f64 = 1.0 / 500.0;

fn bicycle() -> Box<dyn Solver> {
    build(
        Fidelity::Bicycle,
        Chassis::new(
            sdm26(),
            Box::new(MagicFormulaTyre::sdm26()),
            Box::new(GearedEngine::sdm26()),
        ),
    )
}

/// Constant-radius skidpad, driven rather than solved, so the transient model
/// has to actually settle. FSAE skidpad path radius for SDM26 is 9.125 m and
/// the car ran a real 5.02 s.
fn skidpad_speed(car: &mut dyn Solver) -> f64 {
    const R: f64 = 9.125;
    let mut best = 0.0;
    let mut target = 8.0;
    while target <= 16.0 {
        car.reset(0.0, 0.0, 0.0, target);
        // PI on yaw rate around an Ackermann feed-forward. The output is the
        // steer angle itself, not an increment — accumulating on top of an
        // integral term makes a double integrator that spins the car and
        // reports a grip limit it never reached.
        let ff = car.params().wheelbase_m / R / car.params().steering.max_steer_rad;
        let (mut integral, mut sum_r, mut n) = (0.0, 0.0, 0);
        let mut blew = false;
        for i in 0..6000 {
            let s = car.state();
            let err = s.speed() / R - s.r;
            integral = (integral + err * DT).clamp(-0.5, 0.5);
            let steer = (ff + 6.0 * err + 4.0 * integral).clamp(-1.0, 1.0);
            let v_err = target - s.speed();
            let throttle = (0.3 + v_err * 0.6).clamp(0.0, 1.0);
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
        let held = !blew
            && (mean_r - R).abs() / R < 0.04
            && (car.state().speed() - target).abs() < 0.5;
        if held {
            best = target;
        }
        target += 0.05;
    }
    best
}

#[test]
fn skidpad_matches_the_real_run() {
    let mut car = bicycle();
    let v = skidpad_speed(car.as_mut());
    let lap = 2.0 * std::f64::consts::PI * 9.125 / v;
    let g = v * v / 9.125 / 9.81;
    println!("skidpad {v:.2} m/s -> {lap:.3} s, {g:.3} g");
    assert!(
        (4.85..=5.35).contains(&lap),
        "skidpad {lap:.3} s, expected ~5.02 s (SDM26's real run)"
    );
    // Above the bare tyre mu because 11.5 m/s is already worth ~250 N of
    // downforce on this aero package: grip scales with it, mass does not.
    assert!((1.30..=1.60).contains(&g), "skidpad {g:.3} g");
}

/// 75 m acceleration with a managed launch. Deliberately banded ABOVE the
/// quasi-steady lap sim's 4.2 s: this model carries the driveline rotational
/// inertia (~+94 kg apparent in first) that a QSS sim ignores entirely, so it
/// should be a few tenths slower. If this ever comes in at 4.2 s, something has
/// stopped modelling the inertia.
#[test]
fn seventy_five_metre_accel() {
    let mut car = bicycle();
    car.reset(0.0, 0.0, 0.0, 0.0);
    let mut t = 0.0;
    while t < 12.0 && car.state().x < 75.0 {
        let tel = car.telemetry();
        {
            let pt = car.powertrain_mut();
            if pt.can_shift() && tel.engine_rpm > 12_000.0 && tel.gear < 5 {
                pt.shift_up();
            }
        }
        // Hold slip ratio just past peak, the way a good launch does.
        let over = car.telemetry().kappa[RL] - 0.13;
        let throttle = (1.0 - over * 8.0).clamp(0.15, 1.0);
        car.step(DT, Controls { steer: 0.0, throttle, brake: 0.0 });
        t += DT;
    }
    println!("75 m in {t:.3} s at {:.1} km/h", car.state().speed() * 3.6);
    assert!((4.0..=5.2).contains(&t), "75 m in {t:.2} s");
}

#[test]
fn braking_from_25_ms() {
    let mut car = bicycle();
    car.powertrain_mut().shift_up();
    car.powertrain_mut().shift_up();
    car.powertrain_mut().shift_up();
    car.reset(0.0, 0.0, 0.0, 25.0);
    let x0 = car.state().x;
    let mut peak = 0.0f64;
    let mut t = 0.0;
    while t < 6.0 && car.state().speed() > 0.5 {
        car.step(DT, Controls { steer: 0.0, throttle: 0.0, brake: 1.0 });
        peak = peak.min(car.telemetry().ax_g);
        t += DT;
    }
    let dist = car.state().x - x0;
    println!("stopped in {dist:.2} m, peak {:.2} g", -peak);
    assert!((18.0..=40.0).contains(&dist), "stopping distance {dist:.1} m");
    assert!((1.2..=2.6).contains(&-peak), "peak decel {:.2} g", -peak);
}

/// Roll stiffness must move the balance in the textbook direction. Constant
/// steer, constant speed: more front roll stiffness means more front lateral
/// load transfer, less front grip, and a wider radius.
#[test]
fn front_roll_stiffness_adds_understeer() {
    let mut radii = Vec::new();
    for rsd in [0.40, 0.50, 0.60, 0.70] {
        let mut car = bicycle();
        car.params_mut().roll.rsd_front = rsd;
        car.reset(0.0, 0.0, 0.0, 9.0);
        let (mut sum, mut n) = (0.0, 0);
        for i in 0..4000 {
            let v_err = 9.0 - car.state().speed();
            let throttle = (0.3 + v_err * 0.6).clamp(0.0, 1.0);
            car.step(DT, Controls { steer: 0.42, throttle, brake: 0.0 });
            if i > 3000 {
                let s = car.state();
                sum += s.speed() / s.r.abs().max(1e-4);
                n += 1;
            }
        }
        radii.push(sum / n as f64);
    }
    println!("radii by front roll stiffness: {radii:?}");
    for w in radii.windows(2) {
        assert!(
            w[1] > w[0],
            "stiffening the front did not add understeer: {radii:?}"
        );
    }
}

/// Brake bias must move which axle locks first.
#[test]
fn brake_bias_moves_the_lockup() {
    let lock_order = |bias: f64| -> (Option<f64>, Option<f64>) {
        let mut car = bicycle();
        car.params_mut().brakes.bias_front = bias;
        for _ in 0..4 {
            car.powertrain_mut().shift_up();
        }
        car.reset(0.0, 0.0, 0.0, 26.0);
        let (mut lf, mut lr) = (None, None);
        for i in 0..3000 {
            if car.state().speed() <= 6.0 {
                break;
            }
            let brake = (i as f64 * DT * 0.55).min(1.0);
            car.step(DT, Controls { steer: 0.0, throttle: 0.0, brake });
            let tel = car.telemetry();
            if lf.is_none() && tel.kappa[FL] < -0.30 {
                lf = Some(brake);
            }
            if lr.is_none() && tel.kappa[RL] < -0.30 {
                lr = Some(brake);
            }
            if lf.is_some() && lr.is_some() {
                break;
            }
        }
        (lf, lr)
    };

    let (lf_low, lr_low) = lock_order(0.48);
    let (lf_high, lr_high) = lock_order(0.74);
    println!("48% front: front {lf_low:?} rear {lr_low:?}");
    println!("74% front: front {lf_high:?} rear {lr_high:?}");

    // Low front bias: the rear should go first.
    if let (Some(f), Some(r)) = (lf_low, lr_low) {
        assert!(r < f, "48% front bias should lock the rear first");
    }
    // High front bias: the front should go first, or the rear never locks.
    match (lf_high, lr_high) {
        (Some(f), Some(r)) => assert!(f < r, "74% front bias should lock the front first"),
        (Some(_), None) => {}
        other => panic!("74% front bias never locked the front: {other:?}"),
    }
}

/// The whole point of the fidelity ladder: the same car through all three
/// solvers should agree on the big picture even though only two of them can
/// spin.
#[test]
fn all_three_fidelities_run_the_same_car() {
    for fidelity in [Fidelity::PointMass, Fidelity::Bicycle, Fidelity::DoubleTrack] {
        let mut car = build(
            fidelity,
            Chassis::new(
                sdm26(),
                Box::new(MagicFormulaTyre::sdm26()),
                Box::new(GearedEngine::sdm26()),
            ),
        );
        car.reset(0.0, 0.0, 0.0, 0.0);
        for _ in 0..1000 {
            car.step(DT, Controls { steer: 0.0, throttle: 1.0, brake: 0.0 });
        }
        let s = car.state();
        println!(
            "{:>28} level {}  {:.1} m, {:.1} km/h",
            car.name(),
            car.fidelity().level(),
            s.x,
            s.speed() * 3.6
        );
        assert!(s.x > 3.0, "{} barely moved", car.name());
        assert!(s.speed() > 3.0, "{} barely accelerated", car.name());
        assert!(s.speed() * 3.6 < 200.0, "{} ran away", car.name());
    }
}

/// Swapping the powertrain must not need a different solver.
#[test]
fn powertrains_are_interchangeable() {
    for pt in [
        Box::new(GearedEngine::sdm26()) as Box<dyn PowertrainModel>,
        Box::new(IdealDrive::sdm26()),
        Box::new(ElectricDrive::concept()),
    ] {
        let name = pt.name();
        let mut car = build(
            Fidelity::Bicycle,
            Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), pt),
        );
        car.reset(0.0, 0.0, 0.0, 0.0);
        for _ in 0..1500 {
            car.step(DT, Controls { steer: 0.0, throttle: 1.0, brake: 0.0 });
        }
        println!("{name:>34}: {:.1} km/h after 3 s", car.state().speed() * 3.6);
        assert!(car.state().speed() > 5.0, "{name} produced no drive");
    }
}

/// A different vehicle is a different parameter set, not different code.
#[test]
fn a_heavier_car_accelerates_slower() {
    let run = |v: VehicleParams| {
        let mut car = build(
            Fidelity::Bicycle,
            Chassis::new(
                v,
                Box::new(MagicFormulaTyre::sdm26()),
                Box::new(GearedEngine::sdm26()),
            ),
        );
        car.reset(0.0, 0.0, 0.0, 0.0);
        for _ in 0..1500 {
            car.step(DT, Controls { steer: 0.0, throttle: 0.6, brake: 0.0 });
        }
        car.state().x
    };
    let light = run(sdm26());
    let heavy = run(sdm25());
    println!("SDM26 {light:.2} m vs SDM25 {heavy:.2} m");
    assert!(heavy < light, "the heavier car should not get further");
}
