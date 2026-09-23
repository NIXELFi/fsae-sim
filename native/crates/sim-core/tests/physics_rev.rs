//! The physics-revision guard (see src/physics_rev.rs).
//!
//! Each vehicle model's physics is fingerprinted. If a change moves a
//! fingerprint this fails, and someone has to decide: do lap times move? Then
//! bump that model's revision (src/physics_rev.rs AND sim/src/vehicle/
//! physicsRev.js) so its laps start a new leaderboard era, and re-record the
//! fingerprint below. If they do not, re-record the fingerprint only and say
//! why in the commit. Either way the leaderboard is never silently fed times
//! from a different car.

use sim_core::physics_rev::{fnv1a, PHYSICS_REV_BICYCLE, PHYSICS_REV_DOUBLE_TRACK};
use sim_core::prelude::*;

/// Recorded fingerprints, and the revision each was recorded at. A revision
/// bump without re-recording, or a re-record without a bump decision, both
/// show up here in review.
const BICYCLE: (u32, u64) = (1, 0x17c4be4f51b2e680);
/// ramp10/15/20 peak g, standing 75 m s, stop from 25 m/s m. Compared within
/// `DT_TOLERANCE`, not rounded: a value near its rounding edge trips on
/// noise, and one mid-step lets a real change through.
const DOUBLE_TRACK: (u32, [f64; 5]) = (2, [1.4744, 1.6461, 1.8712, 4.8720, 19.4129]);
/// What a driver would feel: 0.01 g, 0.01 s, 0.1 m.
const DT_TOLERANCE: [f64; 5] = [0.01, 0.01, 0.01, 0.01, 0.1];

fn bicycle_fingerprint() -> u64 {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../sim/data/vehicle-golden.json");
    let golden = std::fs::read_to_string(path).expect("golden file");
    // The golden drive IS the bicycle's physics (bicycle_frozen.rs replays it
    // to the last digit); its rows, line endings aside, are the fingerprint.
    let rows: Vec<&str> = golden.lines().filter(|l| l.starts_with("{\"f\":")).map(|l| l.trim_end_matches(',')).collect();
    fnv1a(&rows.join("\n"))
}

fn dt() -> Box<dyn Solver> {
    build(Fidelity::DoubleTrack, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())))
}

/// Peak lateral g on a slow steer ramp at constant speed.
fn ramp_peak(speed: f64) -> f64 {
    const DT: f64 = 1.0 / 200.0;
    let mut c = dt();
    c.reset(0.0, 0.0, 0.0, speed);
    c.powertrain_mut().set_gear(if speed > 17.0 { 2 } else { 1 });
    c.powertrain_mut().sync_to_wheel(speed / 0.2);
    let (mut t, mut pk) = (0.0, 0.0f64);
    while t < 16.0 {
        let th = (0.2 + (speed - c.state().speed()) * 0.8).clamp(0.0, 0.55);
        c.step(DT, Controls { steer: t / 16.0 * 0.7 * 28.0 / 46.0, throttle: th, brake: 0.0 });
        t += DT;
        let tel = c.telemetry();
        if tel.body_slip_deg.abs() > 20.0 {
            break;
        }
        pk = pk.max(tel.ay_g);
    }
    pk
}

/// Standing 75 m with a slip-managed launch, and a full-pedal stop from 25 m/s.
fn accel_and_stop() -> (f64, f64) {
    const DT: f64 = 1.0 / 500.0;
    let mut c = dt();
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
        let over = tel.kappa[RL].max(tel.kappa[RR]) - 0.13;
        c.step(DT, Controls { steer: 0.0, throttle: (1.0 - over * 8.0).clamp(0.15, 1.0), brake: 0.0 });
        t += DT;
    }
    let mut s = dt();
    s.powertrain_mut().set_gear(3);
    s.reset(0.0, 0.0, 0.0, 25.0);
    let mut u = 0.0;
    while u < 6.0 && s.state().speed() > 0.5 {
        s.step(DT, Controls { steer: 0.0, throttle: 0.0, brake: 1.0 });
        u += DT;
    }
    (t, s.state().x)
}

fn double_track_fingerprint() -> [f64; 5] {
    let (acc, stop) = accel_and_stop();
    [ramp_peak(10.0), ramp_peak(15.0), ramp_peak(20.0), acc, stop]
}

/// The JS mirror (what the recorder stamps on runs) must say the same.
fn js_revs() -> (u32, u32) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../sim/src/vehicle/physicsRev.js");
    let js = std::fs::read_to_string(path).expect("physicsRev.js");
    let rev = |key: &str| -> u32 {
        let line = js.lines().find(|l| l.trim_start().starts_with(key)).unwrap_or_else(|| panic!("no `{key}` in physicsRev.js"));
        line.split(':').nth(1).unwrap().trim().trim_end_matches(|c: char| !c.is_ascii_digit()).split(|c: char| !c.is_ascii_digit()).next().unwrap().parse().unwrap()
    };
    (rev("2:"), rev("3:"))
}

#[test]
fn physics_revisions_match_their_fingerprints() {
    let (js_bike, js_dt) = js_revs();
    assert_eq!((js_bike, js_dt), (PHYSICS_REV_BICYCLE, PHYSICS_REV_DOUBLE_TRACK), "physicsRev.js disagrees with physics_rev.rs");

    let bike = bicycle_fingerprint();
    let dtf = double_track_fingerprint();
    println!("bicycle rev {PHYSICS_REV_BICYCLE}: {bike:#x}");
    println!("double track rev {PHYSICS_REV_DOUBLE_TRACK}: {dtf:.4?}");

    assert_eq!(
        BICYCLE,
        (PHYSICS_REV_BICYCLE, bike),
        "\nThe BICYCLE's physics changed (its golden drive moved).\n\
         If lap times move: bump PHYSICS_REV_BICYCLE (physics_rev.rs + physicsRev.js) -- a new leaderboard era.\n\
         Either way, re-record BICYCLE in tests/physics_rev.rs as ({PHYSICS_REV_BICYCLE}, {bike:#x}).\n"
    );
    let moved = DOUBLE_TRACK.1.iter().zip(&dtf).zip(&DT_TOLERANCE).any(|((a, b), tol)| (a - b).abs() > *tol);
    assert!(
        DOUBLE_TRACK.0 == PHYSICS_REV_DOUBLE_TRACK && !moved,
        "\nThe 4-WHEEL model's physics changed.\n\
         If lap times move: bump PHYSICS_REV_DOUBLE_TRACK (physics_rev.rs + physicsRev.js) -- a new leaderboard era.\n\
         Either way, re-record DOUBLE_TRACK in tests/physics_rev.rs as ({PHYSICS_REV_DOUBLE_TRACK}, {dtf:.4?}).\n"
    );
}
