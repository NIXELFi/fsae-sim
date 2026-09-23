//! Timed skidpad (FSAE D.10) for both models: the tightest line each can
//! hold, with and without opening it up at the exit of each timed lap, next
//! to the old steady-state 9.125 m number.
//!
//!   cargo run --release -p sim-core --example timed_skidpad
#[path = "../tests/common/skidpad.rs"]
#[allow(dead_code)]
mod skidpad;

use sim_core::prelude::*;
use skidpad::*;

fn car(f: Fidelity) -> Box<dyn Solver> {
    build(f, Chassis::new(sdm26(), Box::new(MagicFormulaTyre::sdm26()), Box::new(GearedEngine::sdm26())))
}

fn main() {
    for (name, f) in [("bicycle", Fidelity::Bicycle), ("4-wheel beta", Fidelity::DoubleTrack)] {
        let mut c = car(f);
        let tight = tightest_line(c.params(), 0.0);
        println!("\n{name}: tyres touch the inner cones at a {tight:.3} m line");
        let mut overall: Option<(Line, skidpad::Result)> = None;
        let mut lines = vec![(D, D, 0.0)];
        for m in [0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5, 0.6] {
            let r = tight + m;
            lines.push((r, r, 0.0));
            for (dr, span) in [(0.3, 1.0), (0.6, 1.5)] {
                lines.push((r, r + dr, span));
            }
        }
        for (r, rx, span) in lines {
            match fastest(c.as_mut(), r, rx, span) {
                Some((l, res)) => {
                    println!(
                        "  line {r:.3} m, exit {rx:.2} m over {span:.1} rad: {:.3} s (R {:.3} / L {:.3}) at {:.2} m/s, {:.3} g, max body slip {:.1} deg",
                        res.time(), res.right_s, res.left_s, l.v_line, res.mean_ay_g, res.max_body_slip_deg
                    );
                    if overall.as_ref().map_or(true, |(_, b)| res.time() < b.time()) {
                        overall = Some((l, res));
                    }
                }
                None => println!("  line {r:.3} m, exit {rx:.2} m over {span:.1} rad: no clean run"),
            }
        }
        if let Some((l, r)) = overall {
            println!("  BEST {:.3} s on a {:.3} m line opening to {:.2} m", r.time(), l.r_line, l.r_exit);
        }
    }
}
