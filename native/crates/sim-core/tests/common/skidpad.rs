//! A TIMED skidpad: the FSAE figure of eight driven the way the rules time it.
//!
//! The validation skidpad (tests/validation.rs) is a steady-state number:
//! the fastest speed the car holds on a 9.125 m circle, turned into 2 pi R /
//! v. That is not what a real run is. FSAE Rules 2021 D.10:
//!
//! * two pairs of concentric circles, centres 18.25 m apart, inner circles
//!   15.25 m and outer 21.25 m in diameter, a 3.0 m driving path between;
//! * the line between the centres is the start/stop line, and a lap is once
//!   round one circle from that line back to it;
//! * enter perpendicular, one lap on the right circle, a TIMED lap on the
//!   right, then a lap on the left and a TIMED lap on the left, then exit
//!   "at the intersection moving in the same direction as entered";
//! * corrected time = (right + left) / 2 + 0.125 s per cone down. An off
//!   course is a DNF.
//!
//! Nothing says the car has to run the lane's centre. A driver hugs the
//! inner cones, which shortens the lap (a 8.4 m radius is 8 % shorter than
//! 9.125 m) at the price of a lower speed (grip goes as sqrt(R)), so the
//! lap time goes as sqrt(R) and the tightest line wins. So this driver runs
//! a line of radius `r_line` around each circle's centre, blended in from the
//! crossover, and may open it again over the last part of each timed lap and
//! accelerate out ("throttle up at the exit"). The search finds the tightest
//! line and the fastest speed that keep every tyre inside the lane.
//!
//! "Inside the lane" means no tyre's outer edge inside an inner circle -- the
//! 16 inner cones sit on its inside edge, so that is a cone -- and no tyre
//! outside both outer circles away from the 3 m entry/exit path. It is judged
//! at the four contact patches, padded by half a tyre width. That is stricter
//! than the rule (a cone is only down if it is hit; an OC is all four wheels
//! out), which is the right way round for a number that claims "clean".
//!
//! Harness only. It commands the same Controls a driver does and reads the
//! same state; it changes nothing in any model.

use sim_core::prelude::*;
use std::f64::consts::{FRAC_PI_2, PI};

/// Half the distance between the circle centres = the lane's centre radius.
pub const D: f64 = 9.125;
pub const R_INNER: f64 = 15.25 / 2.0;
pub const R_OUTER: f64 = 21.25 / 2.0;
/// Half the 3.0 m entry/exit path.
pub const GATE_HALF: f64 = 1.5;
/// Half the tyre's width. Hoosier 16x7.5-10: 7.5 in section.
pub const TYRE_HALF_W: f64 = 0.0953;
pub const DOO_PENALTY_S: f64 = 0.125;

/// Control rate. The solvers substep internally.
pub const DT: f64 = 1.0 / 200.0;

/// One way to drive the figure of eight.
#[derive(Debug, Clone, Copy)]
pub struct Line {
    /// Radius the car holds around each circle's centre (m).
    pub r_line: f64,
    /// Radius the line has opened to by the start/stop line at the end of
    /// each timed lap (m). `r_line` = no opening, a constant-radius lap.
    pub r_exit: f64,
    /// How much of the lap, before the line, the opening takes (rad).
    pub exit_span: f64,
    /// Target speed on `r_line` (m/s). Elsewhere the target is the same
    /// lateral acceleration on the local radius, so the car speeds up as the
    /// line opens.
    pub v_line: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Result {
    pub right_s: f64,
    pub left_s: f64,
    pub clean: bool,
    /// Why a run was not clean, for the search's diagnostics.
    pub why: &'static str,
    pub max_body_slip_deg: f64,
    /// Lateral g, mean over the two timed laps.
    pub mean_ay_g: f64,
    /// Where the car was when a run stopped being clean.
    pub fail_at: (f64, f64, usize),
}

impl Result {
    pub fn time(&self) -> f64 {
        0.5 * (self.right_s + self.left_s)
    }
}

/// A circle's centre, the direction the car goes round it (+1
/// anticlockwise) and the radius the car arrives on at the crossover.
struct Circle {
    cx: f64,
    sgn: f64,
    r_in: f64,
}

fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
    if e1 <= e0 {
        return if x >= e1 { 1.0 } else { 0.0 };
    }
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Blend from the crossover radius to the line takes this much of lap 1 / 3.
const ENTRY_SPAN: f64 = PI / 2.0;

fn radius_at(c: &Circle, line: &Line, tau: f64) -> f64 {
    let e = smoothstep(0.0, ENTRY_SPAN, tau);
    let x = smoothstep(4.0 * PI - line.exit_span, 4.0 * PI, tau);
    let r = c.r_in + (line.r_line - c.r_in) * e;
    r + (line.r_exit - line.r_line) * x
}

/// Path radius, its first and second derivative in tau.
fn radius_d(c: &Circle, line: &Line, tau: f64) -> (f64, f64, f64) {
    let h = 1e-3;
    let r0 = radius_at(c, line, tau);
    let rp = radius_at(c, line, tau + h);
    let rm = radius_at(c, line, tau - h);
    (r0, (rp - rm) / (2.0 * h), (rp - 2.0 * r0 + rm) / (h * h))
}

/// Signed path curvature (left positive) at `tau` on circle `c`.
fn curvature_at(c: &Circle, line: &Line, tau: f64) -> f64 {
    let (r, rp, rpp) = radius_d(c, line, tau);
    c.sgn * (r * r + 2.0 * rp * rp - r * rpp) / (r * r + rp * rp).powf(1.5)
}

/// How far ahead the steering feed-forward reads the path (s). A driver
/// turns in before the curvature changes, not at it: without this the car
/// runs wide at the entry and through the crossover by more than the lane's
/// 0.8 m of slack each side, and the search reports a grip limit it never
/// reached.
const PREVIEW_S: f64 = 0.12;

/// Speed on the UNTIMED laps, relative to the timed ones, until the car has
/// settled. Nothing is timed there, so a driver comes in under the limit and
/// builds up to it; the timed lap starts at full speed on the line.
fn untimed_factor(tau: f64) -> f64 {
    0.85 + 0.15 * smoothstep(0.5 * PI, 1.75 * PI, tau)
}

fn wrap(a: f64) -> f64 {
    let mut a = a % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    }
    if a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Is a point, padded by `h`, somewhere a tyre may be?
fn in_lane(x: f64, y: f64, h: f64) -> bool {
    let dr = (x - D).hypot(y);
    let dl = (x + D).hypot(y);
    if dr - h < R_INNER || dl - h < R_INNER {
        return false;
    }
    let in_ring = dr + h <= R_OUTER || dl + h <= R_OUTER;
    let in_gate_path = x.abs() + h <= GATE_HALF;
    in_ring || in_gate_path
}

/// Pick the gear closest to 9500 rpm at `v`, as the ramp checks do.
fn gear_for(v: f64) -> usize {
    let ratios = [2.75, 2.0, 1.667, 1.444, 1.304, 1.208];
    let (mut best, mut bd) = (0usize, f64::MAX);
    for (g, r) in ratios.iter().enumerate() {
        let rpm = v / 0.2 * 2.111 * r * 3.0 * 60.0 / (2.0 * PI);
        if rpm < 13000.0 && (rpm - 9500.0).abs() < bd {
            bd = (rpm - 9500.0).abs();
            best = g;
        }
    }
    best
}

/// Drive one run: entry, R, R (timed), L, L (timed), exit.
pub fn run(car: &mut dyn Solver, line: &Line) -> Result {
    let p = car.params().clone();
    let (a, b) = (p.a(), p.b());
    let (tf, tr) = (p.track_front_m / 2.0, p.track_rear_m / 2.0);
    let max_steer = p.steering.max_steer_rad;
    // The yaw-rate loop from tests/validation.rs, held to 28 deg of road
    // wheel for the same reason it is there.
    let ls = 28.0 / max_steer.to_degrees();

    let v_at = |r: f64| line.v_line * (r / line.r_line).sqrt();

    let entry_y = -12.0;
    let v0 = v_at(D) * untimed_factor(0.0);
    car.reset(0.0, entry_y, FRAC_PI_2, v0);
    car.powertrain_mut().set_gear(gear_for(line.v_line));
    car.powertrain_mut().sync_to_wheel(v0 / p.tyre_radius_m);

    let right = Circle { cx: D, sgn: -1.0, r_in: D };
    let left_r_in = 2.0 * D - line.r_exit;
    let left = Circle { cx: -D, sgn: 1.0, r_in: left_r_in };

    // phase 0 = entry straight, 1 = right circle, 2 = left, 3 = exit.
    let mut phase = 0;
    let mut tau = 0.0;
    let mut phi_last = 0.0;
    let exit_x = -(D - line.r_exit);
    let (mut integral, mut th_i) = (0.0, 0.0);
    let mut crossings: Vec<f64> = Vec::new();
    let mut last_y = entry_y;
    let mut t = 0.0;
    let mut max_beta: f64 = 0.0;
    let (mut ay_sum, mut ay_n) = (0.0, 0usize);

    let res = |crossings: &Vec<f64>, clean: bool, why: &'static str, mb: f64, ay: f64| {
        let right_s = if crossings.len() >= 3 { crossings[2] - crossings[1] } else { f64::NAN };
        let left_s = if crossings.len() >= 5 { crossings[4] - crossings[3] } else { f64::NAN };
        Result { right_s, left_s, clean, why, max_body_slip_deg: mb, mean_ay_g: ay, fail_at: (0.0, 0.0, 0) }
    };

    while t < 60.0 {
        let s = car.state();
        let speed = s.speed();

        // Path: curvature (left positive), heading, and the car's offset
        // to the left of it.
        let (kappa, psi_path, e_left, r_here) = match phase {
            0 => (0.0, FRAC_PI_2, -s.x, D),
            3 => (0.0, FRAC_PI_2, -(s.x - exit_x), line.r_exit),
            _ => {
                let c = if phase == 1 { &right } else { &left };
                let phi = s.y.atan2(s.x - c.cx);
                tau += c.sgn * wrap(phi - phi_last);
                phi_last = phi;
                let (r, rp, rpp) = radius_d(c, line, tau);
                let rho = (s.x - c.cx).hypot(s.y);
                let kmag = (r * r + 2.0 * rp * rp - r * rpp) / (r * r + rp * rp).powf(1.5);
                let beta = (rp / r).atan();
                let psi_circ = phi + c.sgn * FRAC_PI_2;
                (c.sgn * kmag, psi_circ - c.sgn * beta, -c.sgn * (rho - r), r)
            }
        };

        // Phase changes.
        match phase {
            0 if s.y >= 0.0 => {
                phase = 1;
                tau = 0.0;
                phi_last = s.y.atan2(s.x - right.cx);
            }
            1 if tau >= 4.0 * PI => {
                phase = 2;
                tau = 0.0;
                phi_last = s.y.atan2(s.x - left.cx);
            }
            2 if tau >= 4.0 * PI => phase = 3,
            3 if s.y > 8.0 => break,
            _ => {}
        }

        // Steering: a pure-pursuit-style correction on the path curvature,
        // fed to the validation skidpad's yaw-rate PI.
        let ld = (0.45 * speed).max(3.0);
        let e_h = wrap(s.psi - psi_path);
        let look = speed * PREVIEW_S;
        let k_prev = match phase {
            0 if -s.y < look => curvature_at(&right, line, (look + s.y) / D),
            1 => {
                let ta = tau + look / r_here;
                if ta < 4.0 * PI {
                    curvature_at(&right, line, ta)
                } else {
                    curvature_at(&left, line, (ta - 4.0 * PI) * r_here / left_r_in)
                }
            }
            2 => {
                let ta = tau + look / r_here;
                if ta < 4.0 * PI { curvature_at(&left, line, ta) } else { 0.0 }
            }
            _ => kappa,
        };
        let k_cmd = k_prev - 2.0 * e_left / (ld * ld) - 2.0 * e_h.sin() / ld;
        let r_target = speed * k_cmd;
        let err = r_target - s.r;
        integral = (integral + err * DT).clamp(-0.5, 0.5);
        let ff = p.wheelbase_m * k_cmd / max_steer;
        let steer = (ff + (6.0 * err + 4.0 * integral) * ls).clamp(-ls, ls);

        // Throttle: speed PI. Where the target runs away (the line is
        // opening, or the exit straight) this is simply full throttle.
        let v_target = match phase {
            0 => v0,
            3 => 30.0,
            _ if tau < 2.0 * PI => v_at(r_here) * untimed_factor(tau),
            _ => v_at(r_here),
        };
        let v_err = v_target - speed;
        th_i = (th_i + v_err * DT * 0.8).clamp(-0.3, 0.6);
        let throttle = (0.25 + 0.8 * v_err + th_i).clamp(0.0, 1.0);

        car.step(DT, Controls { steer, throttle, brake: 0.0 });
        t += DT;

        let s2 = car.state();
        // Start/stop line: the segment between the centres, crossed going
        // +y (every crossing of the figure of eight is, see the geometry).
        if last_y < 0.0 && s2.y >= 0.0 && s2.x.abs() < D {
            let frac = -last_y / (s2.y - last_y);
            crossings.push(t - DT + frac * DT);
        }
        last_y = s2.y;

        let tel = car.telemetry();
        max_beta = max_beta.max(tel.body_slip_deg.abs());
        if max_beta > 20.0 {
            return res(&crossings, false, "spun", max_beta, 0.0);
        }
        if crossings.len() == 2 || crossings.len() == 4 {
            ay_sum += tel.ay_g.abs();
            ay_n += 1;
        }
        if phase >= 1 || s2.y > -1.0 {
            let (c, sn) = (s2.psi.cos(), s2.psi.sin());
            for (lx, ly) in [(a, tf), (a, -tf), (-b, tr), (-b, -tr)] {
                let wx = s2.x + lx * c - ly * sn;
                let wy = s2.y + lx * sn + ly * c;
                if !in_lane(wx, wy, TYRE_HALF_W) {
                    let mut r = res(&crossings, false, "off line", max_beta, 0.0);
                    r.fail_at = (wx, wy, crossings.len());
                    return r;
                }
            }
        }
    }
    let ay = if ay_n > 0 { ay_sum / ay_n as f64 } else { 0.0 };
    if crossings.len() < 5 {
        return res(&crossings, false, "incomplete", max_beta, ay);
    }
    res(&crossings, true, "", max_beta, ay)
}

/// The fastest clean run for a given line shape: scan the target speed up
/// until the car cannot hold the line, then refine.
pub fn fastest(car: &mut dyn Solver, r_line: f64, r_exit: f64, exit_span: f64) -> Option<(Line, Result)> {
    let mut best: Option<(Line, Result)> = None;
    let consider = |line: Line, r: Result, best: &mut Option<(Line, Result)>| {
        if r.clean && best.as_ref().map_or(true, |(_, b)| r.time() < b.time()) {
            *best = Some((line, r));
        }
    };
    let mut v = 8.0;
    let mut last_clean = None;
    let mut fails = 0;
    while v < 16.0 {
        let line = Line { r_line, r_exit, exit_span, v_line: v };
        let r = run(car, &line);
        if r.clean {
            last_clean = Some(v);
            fails = 0;
        } else if last_clean.is_some() {
            fails += 1;
            if fails >= 3 {
                break;
            }
        }
        consider(line, r, &mut best);
        v += 0.2;
    }
    let v0 = last_clean?;
    let mut v = v0 - 0.2;
    while v <= v0 + 0.4 {
        let line = Line { r_line, r_exit, exit_span, v_line: v };
        let r = run(car, &line);
        consider(line, r, &mut best);
        v += 0.02;
    }
    best
}

/// The tightest line a car of this track can hold with its tyres inside the
/// lane, plus a small margin for the path-following error.
pub fn tightest_line(p: &VehicleParams, margin: f64) -> f64 {
    R_INNER + p.track_front_m.max(p.track_rear_m) / 2.0 + TYRE_HALF_W + margin
}
