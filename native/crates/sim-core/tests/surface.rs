//! Grass off the course at a real venue (sim-core surface.rs): slippery and
//! draggy on the double track, and nothing changes on asphalt or without a map.

use std::sync::Arc;
use sim_core::prelude::*;
use sim_core::surface::{SurfaceMap, GRASS_MU};

const DT: f64 = 1.0 / 60.0;

fn car() -> Box<dyn Solver> {
    build(
        Fidelity::DoubleTrack,
        Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())),
    )
}

/// A 4 km square site that is all one class (80 = pavement, 0 = grass),
/// the course frame the site frame shifted into its middle, no course.
fn site(class: u8) -> Arc<SurfaceMap> {
    let n = 400;
    Arc::new(SurfaceMap::new(n, n, 10.0, vec![class; n * n], 1.0, 0.0, [2000.0, -2000.0], vec![], vec![]))
}

/// Steady lateral acceleration at 15 m/s on a fixed steer, and the stopping
/// distance from 20 m/s on full brakes.
fn run(surface: Option<Arc<SurfaceMap>>) -> (f64, f64, Vec<f64>) {
    let mut c = car();
    c.set_surface(surface.clone());
    c.powertrain_mut().set_gear(2);
    c.reset(0.0, 0.0, 0.0, 15.0);
    let mut ay = 0.0;
    let mut trace = vec![];
    for i in 0..(4.0 / DT) as usize {
        // Hold the speed on the throttle so the lateral figure is steady.
        let throttle = if c.state().speed() < 15.0 { 0.35 } else { 0.0 };
        c.step(DT, Controls { steer: 10.0 / sdm26().steering.max_steer_rad.to_degrees(), throttle, brake: 0.0 });
        if i as f64 * DT > 3.0 { ay = c.telemetry().ay_g.abs(); }
        trace.push(c.state().x);
    }
    let mut c = car();
    c.set_surface(surface);
    c.powertrain_mut().set_gear(3);
    c.reset(0.0, 0.0, 0.0, 20.0);
    let mut t = 0.0;
    while t < 10.0 && c.state().speed() > 0.3 {
        c.step(DT, Controls { steer: 0.0, throttle: 0.0, brake: 1.0 });
        t += DT;
    }
    (ay, c.state().x, trace)
}

#[test]
fn asphalt_everywhere_is_bit_identical_to_no_map() {
    let (ay0, d0, tr0) = run(None);
    let (ay1, d1, tr1) = run(Some(site(80)));
    assert_eq!(ay0.to_bits(), ay1.to_bits());
    assert_eq!(d0.to_bits(), d1.to_bits());
    assert_eq!(tr0, tr1);
}

#[test]
fn grass_is_slippery_and_draggy() {
    let (ay_a, stop_a, _) = run(None);
    let (ay_g, stop_g, _) = run(Some(site(0)));
    println!("asphalt: {ay_a:.2} g, stops in {stop_a:.1} m; grass: {ay_g:.2} g, stops in {stop_g:.1} m");
    // Braking to a stop is friction-limited on grass: it takes about 1/mu as far.
    assert!(stop_g > stop_a * 1.4, "grass stop {stop_g:.1} m vs asphalt {stop_a:.1} m");
    // A turn the car holds on asphalt is beyond grass grip: less lateral g.
    assert!(ay_g < ay_a * 0.8 && ay_g < GRASS_MU * 1.6 + 0.2, "grass {ay_g:.2} g vs asphalt {ay_a:.2} g");
}
