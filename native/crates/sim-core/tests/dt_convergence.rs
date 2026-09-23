//! The car must not depend on the step it is integrated with.
//!
//! Every test, fingerprint and the rig run the solver at `SUBSTEP` (500 Hz),
//! so a step dependence is not noise, it is a different car: before
//! 2026-09-23 the standing 75 m took 4.859 s at 500 Hz and 4.748 s at 8 kHz
//! (the wheels' implicit slip term was a ~40 kg-per-axle phantom inertia off
//! the line), and the diff's stick band was looser at 500 Hz, so a steady
//! corner's yaw rate moved with the step. See `chassis_coupling` and
//! `clutch_torque` in src/solver/mod.rs. `examples/dt_sweep.rs` prints the
//! full table.

use sim_core::prelude::*;

const RATES: [f64; 5] = [500.0, 1000.0, 2000.0, 4000.0, 8000.0];

fn car(f: Fidelity, geared: bool) -> Box<dyn Solver> {
    let pt: Box<dyn PowertrainModel> =
        if geared { Box::new(GearedEngine::sdm26()) } else { Box::new(ElectricDrive::concept()) };
    build(f, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), pt))
}

/// Standing 75 m with the slip-managed launch of validation.rs. The driver
/// (slip loop, shift call) runs at a fixed 500 Hz and holds its output
/// between, whatever the physics step: a proportional loop on slip ratio is
/// a discrete controller, and running it at the physics rate would measure
/// ITS step dependence, not the car's. Crossing time interpolated.
fn standing_75(f: Fidelity, geared: bool, hz: f64) -> f64 {
    let n = (hz / 500.0).round() as usize;
    let dt = 1.0 / hz;
    let mut c = car(f, geared);
    c.reset(0.0, 0.0, 0.0, 0.0);
    let (mut t, mut throttle) = (0.0, 1.0);
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
        if x1 >= 75.0 {
            return t + dt * (75.0 - x0) / (x1 - x0);
        }
        t += dt;
        assert!(t < 12.0, "{f:?} never reached 75 m at {hz} Hz");
    }
    unreachable!()
}

fn spread(xs: &[f64]) -> f64 {
    xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min)
}

/// Chassis, tyres, wheels and the diff: with a drive that has no clutch the
/// standing 75 m is the same to a millisecond from 500 Hz to 8 kHz (it was
/// ~110 ms apart with the phantom wheel inertia).
#[test]
fn the_launch_does_not_depend_on_the_step() {
    for f in [Fidelity::Bicycle, Fidelity::DoubleTrack] {
        let t: Vec<f64> = RATES.iter().map(|&hz| standing_75(f, false, hz)).collect();
        println!("{f:?} clutch-free 75 m @500..8k Hz: {t:.4?}");
        assert!(spread(&t) < 0.001, "{f:?}: 75 m moves {:.1} ms with the step: {t:.4?}", spread(&t) * 1e3);
    }
}

/// The SDM26 as driven, geared engine and all. What is left is the launch
/// clutch's lock test and the shift timer, both on the step's grid in the
/// powertrain and left as they are by decision (2026-09-23; the clutch-lock
/// behaviour is not to change): ~6 ms bicycle, ~10 ms double track between
/// 500 Hz and 8 kHz, against 111 ms before.
#[test]
fn the_geared_launch_barely_depends_on_the_step() {
    for f in [Fidelity::Bicycle, Fidelity::DoubleTrack] {
        let t: Vec<f64> = RATES.iter().map(|&hz| standing_75(f, true, hz)).collect();
        println!("{f:?} geared 75 m @500..8k Hz: {t:.4?}");
        assert!((t[0] - t[4]).abs() < 0.012, "{f:?}: 500 Hz vs 8 kHz {:.1} ms: {t:.4?}", (t[0] - t[4]).abs() * 1e3);
        assert!(spread(&t) < 0.015, "{f:?}: 75 m spread {:.1} ms: {t:.4?}", spread(&t) * 1e3);
    }
}

/// Steady corner on the throttle at 20 m/s with the diff in its stick band:
/// the yaw rate was 22.85 / 22.55 / 22.30 deg/s at 500 / 1k / 2k Hz.
fn steady_yaw(f: Fidelity, hz: f64) -> f64 {
    let dt = 1.0 / hz;
    let mut c = car(f, true);
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

#[test]
fn the_diffs_stick_band_does_not_depend_on_the_step() {
    for f in [Fidelity::Bicycle, Fidelity::DoubleTrack] {
        let y: Vec<f64> = RATES[..4].iter().map(|&hz| steady_yaw(f, hz)).collect();
        println!("{f:?} steady yaw @500..4k Hz: {y:.3?} deg/s");
        assert!(spread(&y) < 0.02, "{f:?}: steady yaw moves with the step: {y:.3?}");
    }
}
