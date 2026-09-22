//! Calibrate the double track's front grip against skidpad on the car's
//! SKIDPAD setup, and check it still pushes on the AUTOCROSS setup.
use sim_core::prelude::*;
const DT: f64 = 1.0 / 60.0;
fn car(skid: bool) -> Box<dyn Solver> {
    let mut p = sdm26();
    if skid { p.suspension.toe_in_rear_deg = -0.7; p.diff.preload_nm = 0.0; }
    build(Fidelity::DoubleTrack, Chassis::new(p, Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())))
}
fn skidpad(c: &mut dyn Solver, rsd: f64) -> f64 {
    const R: f64 = 9.125; let mut best = 0.0; let mut target = 8.0; let ls = 28.0/46.0;
    while target <= 16.0 {
        c.params_mut().roll.rsd_front = rsd; c.reset(0.0,0.0,0.0,target);
        let ff = c.params().wheelbase_m / R / c.params().steering.max_steer_rad;
        let (mut integ, mut sum_r, mut n, mut blew) = (0.0,0.0,0,false);
        for i in 0..6000 { let s=c.state(); let err=s.speed()/R - s.r; integ=(integ+err*DT).clamp(-0.5,0.5);
            let steer=(ff+(6.0*err+4.0*integ)*ls).clamp(-ls,ls); let th=(0.3+(target-s.speed())*0.6).clamp(0.0,1.0);
            c.step(DT, Controls{steer, throttle: th, brake: 0.0});
            if c.telemetry().body_slip_deg.abs()>45.0 {blew=true;break;}
            if i>4000 {let s=c.state(); sum_r+=s.speed()/s.r.abs().max(1e-4); n+=1;} }
        let mr = if n>0 {sum_r/n as f64} else {1e9};
        if !blew && (mr-R).abs()/R<0.04 && (c.state().speed()-target).abs()<0.5 { best=target; }
        target += 0.05;
    }
    2.0*std::f64::consts::PI*R/best
}
fn ramp(speed: f64) -> (f64,f64,f64) {
    let mut c = car(false); c.reset(0.0,0.0,0.0,speed);
    let ratios=[2.75,2.0,1.667,1.444,1.304,1.208]; let (mut best,mut bd)=(0usize,f64::MAX);
    for (g,r) in ratios.iter().enumerate() { let rpm=speed/0.2*2.111*r*3.0*60.0/(2.0*std::f64::consts::PI); if rpm<14000.0&&(rpm-9500.0).abs()<bd {bd=(rpm-9500.0).abs();best=g;} }
    c.powertrain_mut().set_gear(best); c.powertrain_mut().sync_to_wheel(speed/0.2);
    let (mut t,mut pk,mut bal,mut beta)=(0.0,0.0f64,0.0,0.0f64);
    while t<12.0 { let th=(0.2+(speed-c.state().speed())*0.8).clamp(0.0,0.55); c.step(DT,Controls{steer:t/12.0*0.6*28.0/46.0,throttle:th,brake:0.0}); t+=DT;
        let tel=c.telemetry(); beta=beta.max(tel.body_slip_deg.abs()); if tel.ay_g>pk {pk=tel.ay_g; bal=-tel.balance;} }
    (pk,bal,beta)
}
fn main() {
    let lap = skidpad(car(true).as_mut(), 0.47);
    let mut s = format!("skidpad(skid setup, 47%) {lap:.3} s |");
    for sp in [10.0,15.0,20.0] { let (pk,bal,beta)=ramp(sp); s+=&format!(" {sp}: {pk:.2}g bal {bal:+.2} beta {beta:.1} |"); }
    println!("{s}");
}
