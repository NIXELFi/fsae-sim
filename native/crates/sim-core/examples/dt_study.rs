//! Bicycle vs double-track + suspension: the numbers behind the beta model.
use sim_core::prelude::*;
const DT: f64 = 1.0 / 60.0;
fn mk(f: Fidelity, tweak: &dyn Fn(&mut VehicleParams, &mut MagicFormulaTyre)) -> Box<dyn Solver> {
    let mut p = sdm26(); let mut t = MagicFormulaTyre::sdm26(); tweak(&mut p, &mut t);
    build(f, Chassis::new(p, Box::new(t), Box::new(GearedEngine::sdm26())))
}
fn skidpad(car: &mut dyn Solver) -> f64 {
    const R: f64 = 9.125; let mut best = 0.0; let mut target = 8.0;
    let ls = 28.0 / car.params().steering.max_steer_rad.to_degrees();
    while target <= 16.0 {
        car.params_mut().roll.rsd_front = 0.46; car.reset(0.0, 0.0, 0.0, target);
        let ff = car.params().wheelbase_m / R / car.params().steering.max_steer_rad;
        let (mut integral, mut sum_r, mut n, mut blew) = (0.0, 0.0, 0, false);
        for i in 0..6000 {
            let s = car.state(); let err = s.speed() / R - s.r;
            integral = (integral + err * DT).clamp(-0.5, 0.5);
            let steer = (ff + (6.0 * err + 4.0 * integral) * ls).clamp(-ls, ls);
            let throttle = (0.3 + (target - s.speed()) * 0.6).clamp(0.0, 1.0);
            car.step(DT, Controls { steer, throttle, brake: 0.0 });
            if car.telemetry().body_slip_deg.abs() > 45.0 { blew = true; break; }
            if i > 4000 { let s = car.state(); sum_r += s.speed() / s.r.abs().max(1e-4); n += 1; }
        }
        let mean_r = if n > 0 { sum_r / n as f64 } else { 1e9 };
        if !blew && (mean_r - R).abs() / R < 0.04 && (car.state().speed() - target).abs() < 0.5 { best = target; }
        target += 0.05;
    }
    best
}
fn report(name: &str, car: &mut dyn Solver) {
    let v = skidpad(car);
    let lap = 2.0 * std::f64::consts::PI * 9.125 / v;
    // hold at the found speed to read attitude
    let t = car.telemetry();
    println!("{name:<44} skidpad {lap:.3} s  {:.3} g   roll {:+.2} deg  camber FL/FR/RL/RR {:+.2} {:+.2} {:+.2} {:+.2}  fz {:.0}/{:.0}/{:.0}/{:.0}",
        v * v / 9.125 / 9.81, t.roll_deg, t.camber_deg[0], t.camber_deg[1], t.camber_deg[2], t.camber_deg[3], t.fz[0], t.fz[1], t.fz[2], t.fz[3]);
}
fn main() {
    let none = |_: &mut VehicleParams, _: &mut MagicFormulaTyre| {};
    report("bicycle (validated)", mk(Fidelity::Bicycle, &none).as_mut());
    report("double track + suspension + camber", mk(Fidelity::DoubleTrack, &none).as_mut());
    report("double track, camber off", mk(Fidelity::DoubleTrack, &|p, t| { t.camber_ratio_at = [(300.0,0.0),(655.0,0.0),(1000.0,0.0)]; t.camber_mu_quad = 0.0; let _ = p; }).as_mut());
    report("double track, 0 static camber", mk(Fidelity::DoubleTrack, &|p, _| { p.suspension.static_camber_front_deg = 0.0; p.suspension.static_camber_rear_deg = 0.0; }).as_mut());
    report("double track, -2.0 deg static camber", mk(Fidelity::DoubleTrack, &|p, _| { p.suspension.static_camber_front_deg = -2.0; p.suspension.static_camber_rear_deg = -2.0; }).as_mut());
    for fgf in [0.86, 0.87, 0.88, 0.89] {
        report(&format!("double track, front_grip_factor {fgf}"), mk(Fidelity::DoubleTrack, &|p, _| p.front_grip_factor = fgf).as_mut());
    }
    // steady gradients: 1 g lateral / braking
    for (label, f) in [("bicycle", Fidelity::Bicycle), ("double", Fidelity::DoubleTrack)] {
        // step steer at 15 m/s, 3rd gear
        let mut c = mk(f, &none); c.reset(0.0,0.0,0.0,15.0); c.powertrain_mut().set_gear(2); c.powertrain_mut().sync_to_wheel(75.0);
        for _ in 0..300 { let s=c.state(); c.step(0.002, Controls{steer:0.0, throttle:(0.2+(15.0-s.u)*0.5).clamp(0.0,1.0), brake:0.0}); }
        let (mut rpk, mut tpk, mut rollpk, mut t63) = (0.0f64, 0.0, 0.0f64, -1.0);
        let st = 3.0/46.0;
        let mut hist = Vec::new();
        for i in 0..2000 { let s=c.state(); c.step(0.001, Controls{steer:st, throttle:(0.2+(15.0-s.u)*0.5).clamp(0.0,1.0), brake:0.0});
            let r=c.state().r; hist.push(r); if r>rpk {rpk=r; tpk=i as f64*0.001;} rollpk=rollpk.max(c.telemetry().roll_deg); }
        let rss = *hist.last().unwrap();
        for (i,r) in hist.iter().enumerate() { if *r > 0.632*rss { t63 = i as f64*0.001; break; } }
        let t = c.telemetry();
        println!("{label:<8} step 3 deg @15 m/s: yaw 63% {:.3}s, peak {:.3}s overshoot {:.1}%, ay {:.2} g, roll ss {:+.2} deg (pk {:+.2}) -> {:.3} deg/g",
            t63, tpk, (rpk/rss-1.0)*100.0, t.ay_g, t.roll_deg, rollpk, t.roll_deg / t.ay_g.max(1e-6));
        // braking 20 m/s at 0.6 pedal
        let mut c = mk(f, &none); c.reset(0.0,0.0,0.0,20.0); c.powertrain_mut().set_gear(3); c.powertrain_mut().sync_to_wheel(100.0);
        let mut pit=0.0f64; let mut axg=0.0;
        for i in 0..800 { c.step(0.001, Controls{steer:0.0, throttle:0.0, brake:0.6}); if i==700 { let t=c.telemetry(); pit=t.pitch_deg; axg=t.ax_g; } }
        let mut dist=0.0; let x0=c.state().x; let _=x0;
        let mut c2 = mk(f, &none); c2.reset(0.0,0.0,0.0,20.0); c2.powertrain_mut().set_gear(3); c2.powertrain_mut().sync_to_wheel(100.0);
        while c2.state().u > 1.0 && dist < 100.0 { c2.step(0.001, Controls{steer:0.0, throttle:0.0, brake:0.7}); dist = c2.state().x; }
        println!("{label:<8} braking 0.6 pedal: ax {:.2} g pitch {:+.2} deg -> {:.3} deg/g ; 20->1 m/s at 0.7 pedal {:.2} m", axg, pit, pit/(-axg).max(1e-6), dist);
    }
}
