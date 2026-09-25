//! The 2026-09-24 fidelity pass on the 4-wheel model, checked against the
//! car's own logs. The bicycle is frozen and must not pick any of it up.

use sim_core::prelude::*;
use std::f64::consts::PI;

fn car(f: Fidelity) -> Box<dyn Solver> {
    build(f, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())))
}

fn engine(c: &mut dyn Solver) -> &mut GearedEngine {
    c.powertrain_mut().as_any_mut().and_then(|a| a.downcast_mut::<GearedEngine>()).expect("geared engine")
}

#[test]
fn only_the_four_wheel_gets_the_logged_shift() {
    let mut dt = car(Fidelity::DoubleTrack);
    let e = engine(dt.as_mut());
    assert!(e.full_cut_drag && e.conserve_engagement);
    let mut bike = car(Fidelity::Bicycle);
    let e = engine(bike.as_mut());
    assert!(!e.full_cut_drag && !e.conserve_engagement, "the frozen bicycle's engine changed");
}

/// A 2->3 upshift at WOT from 11,000 rpm with the ECU-logged 185 ms cut:
/// (crank fall rpm/s through the cut, engine vs the new gear's sync at the
/// end of the cut, %).
fn upshift(cut_s: f64) -> (f64, f64) {
    const DT: f64 = 1.0 / 500.0;
    let mut c = car(Fidelity::DoubleTrack);
    engine(c.as_mut()).shift_time_s = cut_s;
    let v = 11_000.0 / 60.0 * 2.0 * PI / (2.111 * 2.0 * 3.0) * 0.2;
    c.reset(0.0, 0.0, 0.0, v);
    c.powertrain_mut().set_gear(1);
    c.powertrain_mut().sync_to_wheel(v / 0.2);
    for _ in 0..50 {
        c.step(DT, Controls { steer: 0.0, throttle: 1.0, brake: 0.0 });
    }
    c.powertrain_mut().shift_up();
    let (mut first, mut last, mut sync_pct) = (None, (0.0, 0.0), 0.0);
    let mut t = 0.0;
    while t < 0.5 {
        c.step(DT, Controls { steer: 0.0, throttle: 1.0, brake: 0.0 });
        t += DT;
        let tel = c.telemetry();
        if tel.shifting {
            first.get_or_insert((t, tel.engine_rpm));
            last = (t, tel.engine_rpm);
            let sync = tel.wheel_omega_rear * 2.111 * 1.667 * 3.0 * 60.0 / (2.0 * PI);
            sync_pct = (tel.engine_rpm / sync - 1.0) * 100.0;
        }
    }
    let (t0, r0) = first.unwrap();
    ((r0 - last.1) / (last.0 - t0), sync_pct)
}

#[test]
fn the_crank_falls_through_the_cut_as_the_ecu_logged_it() {
    // 5/3 accel logs, 11 clean shifts: the crank free-falls at 10,000-14,300
    // rpm/s and meets the new gear within -1..+4 % of sync after a 175-200
    // ms cut. Half the friction (the old cut) fell at ~6000 and met it 14 %
    // high even with the logged cut length.
    let (fall, sync) = upshift(0.185);
    assert!((10_000.0..14_300.0).contains(&fall), "crank fell at {fall:.0} rpm/s");
    assert!((-1.0..5.0).contains(&sync), "met the new gear {sync:+.1} % from sync");
}

#[test]
fn engaging_the_new_gear_keeps_the_launch_step_independent() {
    // The crank's excess momentum is handed over once, at engagement; handed
    // over every locked step it double-counted the crank inertia and cost
    // the standing 75 m 0.17 s at 500 Hz.
    let run = |dt: f64| {
        let mut c = car(Fidelity::DoubleTrack);
        c.reset(0.0, 0.0, 0.0, 0.0);
        let mut t = 0.0;
        while t < 12.0 && c.state().x < 75.0 {
            let tel = c.telemetry();
            let pt = c.powertrain_mut();
            if pt.can_shift() && tel.engine_rpm > 12_000.0 && tel.gear < 5 {
                pt.shift_up();
            }
            let over = tel.kappa[RL].max(tel.kappa[RR]) - 0.13;
            c.step(dt, Controls { steer: 0.0, throttle: (1.0 - over * 8.0).clamp(0.15, 1.0), brake: 0.0 });
            t += dt;
        }
        t
    };
    let (coarse, fine) = (run(1.0 / 500.0), run(1.0 / 2000.0));
    assert!((coarse - fine).abs() < 0.02, "75 m {coarse:.3} s at 500 Hz vs {fine:.3} s at 2 kHz");
}
