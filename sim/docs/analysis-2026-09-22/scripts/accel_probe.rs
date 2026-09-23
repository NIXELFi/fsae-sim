//! 75 m acceleration probe: where the time goes.
use sim_core::prelude::*;
fn main() {
    let args: Vec<String> = std::env::args().collect();
    let fid = if args.get(1).map(|s| s == "dt").unwrap_or(false) { Fidelity::DoubleTrack } else { Fidelity::Bicycle };
    let kcap: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(0.13);
    let mut c = build(fid, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())));
    c.reset(0.0, 0.0, 0.0, 0.0);
    let dt = 1.0 / 500.0;
    let lc = args.get(3).map(|s| s == "lc").unwrap_or(false);
    if lc {
        // Hold launch control with the throttle pinned until the engine
        // sits on the LC limiter, then release.
        c.powertrain_mut().set_launch(true);
        for _ in 0..1000 { c.step(dt, Controls { steer: 0.0, throttle: 1.0, brake: 1.0 }); }
        println!("LC held: rpm {:.0}, x {:.3}", c.telemetry().engine_rpm, c.state().x);
        c.powertrain_mut().set_launch(false);
    }
    let mut t = 0.0;
    let mut next_print = 0.0;
    let mut s52: Option<f64> = None;
    while t < 12.0 && c.state().x < 75.0 {
        let tel = c.telemetry();
        {
            let pt = c.powertrain_mut();
            if pt.can_shift() && tel.engine_rpm > 12_000.0 && tel.gear < 5 { pt.shift_up(); }
        }
        let over = tel.kappa[RL].max(tel.kappa[RR]) - kcap;
        let floor: f64 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(0.15);
        let throttle = (1.0 - over * 8.0).clamp(floor, 1.0);
        c.step(dt, Controls { steer: 0.0, throttle, brake: 0.0 });
        t += dt;
        if s52.is_none() && c.state().speed() >= 52.0 / 3.6 { s52 = Some(t); }
        if t >= next_print && args.len() < 6 {
            let s = c.state(); let tel = c.telemetry();
            println!("t {t:.2} x {:6.2} v {:5.2} ax {:.2}g gear {} rpm {:5.0} th {throttle:.2} kR {:.3} fzR {:.0} Fx {:.0} lock {}", s.x, s.speed(), tel.ax_g, tel.gear, tel.engine_rpm, tel.kappa[RL].max(tel.kappa[RR]), tel.fz[RL]+tel.fz[RR], tel.drive_force_n, tel.locked);
            next_print += 0.1;
        }
    }
    println!("75 m in {t:.3} s at {:.1} km/h, 52 km/h at {:.2} s", c.state().speed() * 3.6, s52.unwrap_or(f64::NAN));
}
