use sim_core::prelude::*;
const DT: f64 = 1.0 / 60.0;
fn car(f: Fidelity, fgf: f64) -> Box<dyn Solver> { let mut p = sdm26(); p.front_grip_factor = fgf;
    build(f, Chassis::new(p, Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26()))) }
fn ramp(f: Fidelity, fgf: f64, speed: f64) -> (f64, f64, f64) {
    let mut car = car(f, fgf); car.reset(0.0,0.0,0.0,speed);
    let ratios = [2.75, 2.0, 1.667, 1.444, 1.304, 1.208]; let (mut best, mut bd) = (0usize, f64::MAX);
    for (g, r) in ratios.iter().enumerate() { let rpm = speed/0.2*2.111*r*3.0*60.0/(2.0*std::f64::consts::PI); if rpm<14000.0 && (rpm-9500.0).abs()<bd {bd=(rpm-9500.0).abs(); best=g;} }
    car.powertrain_mut().set_gear(best); car.powertrain_mut().sync_to_wheel(speed/0.2);
    let ls = 28.0/46.0; let (mut t, mut pk, mut bal, mut beta) = (0.0, 0.0f64, 0.0, 0.0f64);
    while t < 12.0 { let v_err = speed - car.state().speed(); let th=(0.2+v_err*0.8).clamp(0.0,0.55);
        car.step(DT, Controls{steer: t/12.0*0.6*ls, throttle: th, brake: 0.0}); t+=DT; let tel=car.telemetry();
        beta=beta.max(tel.body_slip_deg.abs()); if tel.ay_g>pk {pk=tel.ay_g; bal=-tel.balance;} }
    (pk, bal, beta)
}
fn main() {
    for (lbl, f, fgfs) in [("bicycle", Fidelity::Bicycle, vec![0.80]), ("double", Fidelity::DoubleTrack, vec![0.80])] {
        for fgf in fgfs { let mut s = format!("{lbl:<8} fgf {fgf:.2}:"); for sp in [10.0,15.0,20.0] { let (pk,bal,beta)=ramp(f,fgf,sp); s+=&format!("  {sp} m/s {pk:.2} g bal {bal:+.2} beta {beta:.1}"); } println!("{s}"); }
    }
}
