//! Level 3 -- double-track (four-corner) model with a suspension.
//!
//! Everything the bicycle model does, per wheel, plus the body on its springs:
//!
//!   * Four contact patches, each with its own load, slip angle (in its own
//!     wheel frame), slip ratio against its own wheel-plane speed, wheel-speed
//!     state and friction brake.
//!   * The Salisbury differential between the rear wheels, with the
//!     driveline's inertia on the carrier -- the same clutch model as the
//!     bicycle solver.
//!   * A sprung body that ROLLS and PITCHES as states. Load transfer is no
//!     longer instantaneous: the geometric (roll-centre, anti) and unsprung
//!     paths are, as they are on the car, but the elastic path goes through
//!     the body's roll and pitch, so it lags, overshoots or settles as the
//!     springs and dampers decide. Stiffness comes from the team's validated
//!     roll and pitch gradients, split by the `rsd_front` setup knob.
//!   * Camber at each wheel, to the road: static, plus what body roll does
//!     through the linkage (and, fully, through the tyres' own deflection),
//!     plus bump from pitch, plus the steer through caster and KPI. The tyre
//!     turns it into thrust and a small peak-grip loss.
//!   * Steering feel from both front tyres' aligning moments, their brake
//!     forces through the scrub radius, and everything the rig's mixer reads.
//!
//! It is the BETA model. The bicycle is still the validated one, and nothing
//! here changes it.

use super::{
    advance_steer_capped, brake_torque, Chassis, ChassisState, Controls, Fidelity, Solver,
    Telemetry, SUBSTEP, FL, FR, RL, RR,
};
use crate::powertrain::PowertrainModel;
use crate::tyre::{Slip, TyreModel};
use crate::vehicle::{VehicleParams, G};

/// Finite-difference step in slip ratio for the wheel update's implicit term.
const KAPPA_H: f64 = 1e-4;

/// How much of `front_grip_factor`'s deficit this model resolves physically.
///
/// The 0.80 was pinned to the skidpad through the BICYCLE, which puts both
/// front tyres at one slip angle and one camber. With four patches, parallel
/// steer on a 9 m radius runs the inner front ~1.3 deg below the outer, the
/// rear gets its toe-in, and camber is real -- and at the same 0.80 this model
/// ran the skidpad in 5.73 s against the bicycle's 5.19 and the car's 5.02.
/// Scaling the factor by 1.10 (0.80 -> 0.88) puts it at 5.26 s while it still
/// pushes at the limit at 10, 15 and 20 m/s; 0.90 spins at 20 m/s on a steer
/// ramp. A scale rather than a second number so an edit to the factor still
/// moves both models. The 12 % it leaves is what nothing here explains yet
/// (compliance steer, the tyre's load-dependent peak slip, the fit itself).
const DT_FRONT_GRIP_SCALE: f64 = 1.10;

pub struct DoubleTrackSolver {
    c: Chassis,
    s: ChassisState,
    w: [f64; 4],
    alpha_lag: [f64; 4],
    delta: f64,
    steer_rate: f64,
    ax: f64,
    ay: f64,
    /// Body roll (rad, positive = right side down, as in a left turn) and
    /// pitch (rad, positive = nose down, as under braking), with their rates.
    roll: f64,
    roll_rate: f64,
    pitch: f64,
    pitch_rate: f64,
    t_lock_last: f64,
    tel: Telemetry,
}

/// Everything derived from the parameters for one substep of the body.
struct Body {
    ms: f64,
    hs: f64,
    arm: f64,
    k_roll: f64,
    c_roll: f64,
    i_roll: f64,
    k_pitch: f64,
    c_pitch: f64,
    i_pitch: f64,
    /// Fraction of each axle's roll that is the springs (the rest is the
    /// tyres squashing).
    susp_share_f: f64,
    susp_share_r: f64,
}

impl DoubleTrackSolver {
    pub fn new(c: Chassis) -> Self {
        Self {
            c,
            s: ChassisState::default(),
            w: [0.0; 4],
            alpha_lag: [0.0; 4],
            delta: 0.0,
            steer_rate: 0.0,
            ax: 0.0,
            ay: 0.0,
            roll: 0.0,
            roll_rate: 0.0,
            pitch: 0.0,
            pitch_rate: 0.0,
            t_lock_last: 0.0,
            tel: Telemetry::default(),
        }
    }

    /// Per-wheel steer angles, blended between parallel and true Ackermann.
    fn steer_angles(&self) -> (f64, f64) {
        let p = &self.c.params;
        let d = self.delta;
        if d.abs() < 1e-6 || p.steering.ackermann <= 0.0 {
            return (d, d);
        }
        let radius = p.wheelbase_m / d.tan();
        let inner = (p.wheelbase_m / (radius.abs() - p.track_front_m * 0.5)).atan();
        let outer = (p.wheelbase_m / (radius.abs() + p.track_front_m * 0.5)).atan();
        let k = p.steering.ackermann.clamp(0.0, 1.0);
        let (i, o) = (d.abs() + k * (inner - d.abs()), d.abs() + k * (outer - d.abs()));
        // Positive steer is left, so the left wheel is the inner one.
        if d > 0.0 {
            (i * d.signum(), o * d.signum())
        } else {
            (o * d.signum(), i * d.signum())
        }
    }

    fn body(&self) -> Body {
        let p = &self.c.params;
        let sp = &p.suspension;
        let ms = p.sprung_mass();
        let hs = p.sprung_cg_height();
        let arm = p.roll_arm();
        // Stiffness FROM the gradients: roll = ms g arm / K per g, pitch =
        // ms g hs (1 - anti) / K per g of braking. So the car does what the
        // team's validated numbers say it does.
        let anti_brake = 0.5 * (sp.anti_dive_front + sp.anti_lift_rear);
        let k_roll = ms * G * arm / sp.roll_gradient_deg_g.to_radians().max(1e-6);
        let k_pitch = ms * G * hs * (1.0 - anti_brake) / sp.pitch_gradient_deg_g.to_radians().max(1e-6);
        // About the roll axis and about the ground, not the CG.
        let i_roll = sp.ixx_kg_m2 + ms * arm * arm;
        let i_pitch = sp.iyy_kg_m2 + ms * hs * hs;
        let c_roll = 2.0 * sp.roll_damping_ratio * (k_roll * i_roll).sqrt();
        let c_pitch = 2.0 * sp.pitch_damping_ratio * (k_pitch * i_pitch).sqrt();
        // Each axle's roll stiffness is springs and tyres in series; the
        // springs' share of the angle is K_axle / K_springs.
        let share = |k_axle: f64, track: f64| {
            let k_tyre = sp.tyre_rate_n_m * track * track * 0.5;
            if k_axle >= k_tyre * 0.999 {
                0.0
            } else {
                let k_spring = 1.0 / (1.0 / k_axle - 1.0 / k_tyre);
                (k_axle / k_spring).clamp(0.0, 1.0)
            }
        };
        let rsd = p.roll.rsd_front;
        Body {
            ms,
            hs,
            arm,
            k_roll,
            c_roll,
            i_roll,
            k_pitch,
            c_pitch,
            i_pitch,
            susp_share_f: share(rsd * k_roll, p.track_front_m),
            susp_share_r: share((1.0 - rsd) * k_roll, p.track_rear_m),
        }
    }
}

impl Solver for DoubleTrackSolver {
    fn name(&self) -> &'static str {
        "Double track + suspension (beta)"
    }

    fn fidelity(&self) -> Fidelity {
        Fidelity::DoubleTrack
    }

    fn step(&mut self, dt: f64, controls: Controls) {
        let mut remaining = dt.min(0.1);
        while remaining > 1e-9 {
            let h = SUBSTEP.min(remaining);
            self.substep(h, controls);
            remaining -= h;
        }
    }

    fn state(&self) -> ChassisState {
        self.s
    }

    fn state_mut(&mut self) -> &mut ChassisState {
        &mut self.s
    }
    fn telemetry(&self) -> Telemetry {
        self.tel
    }

    fn reset(&mut self, x: f64, y: f64, psi: f64, speed: f64) {
        let gear = self.c.powertrain.telemetry().gear;
        self.s = ChassisState { u: speed, v: 0.0, r: 0.0, x, y, psi };
        let w = speed / self.c.params.tyre_radius_m;
        self.w = [w; 4];
        self.alpha_lag = [0.0; 4];
        self.delta = 0.0;
        self.steer_rate = 0.0;
        self.ax = 0.0;
        self.ay = 0.0;
        self.roll = 0.0;
        self.roll_rate = 0.0;
        self.pitch = 0.0;
        self.pitch_rate = 0.0;
        self.t_lock_last = 0.0;
        self.tel = Telemetry::default();
        // Keep the caller's gear through a rolling reset (see bicycle.rs).
        self.c.powertrain.reset();
        if speed > 0.0 {
            self.c.powertrain.set_gear(gear);
            self.c.powertrain.sync_to_wheel(w);
        }
    }

    fn params(&self) -> &VehicleParams {
        &self.c.params
    }
    fn params_mut(&mut self) -> &mut VehicleParams {
        &mut self.c.params
    }
    fn powertrain_mut(&mut self) -> &mut dyn PowertrainModel {
        self.c.powertrain.as_mut()
    }
    fn tyre(&self) -> &dyn TyreModel {
        self.c.tyre.as_ref()
    }

    fn tyre_mut(&mut self) -> &mut dyn TyreModel {
        self.c.tyre.as_mut()
    }
}

impl DoubleTrackSolver {
    fn substep(&mut self, dt: f64, controls: Controls) {
        let p_a = self.c.params.a();
        let p_b = self.c.params.b();
        let kin = (self.s.v + p_a * self.s.r).atan2(self.s.u.abs().max(0.6));
        self.delta = advance_steer_capped(
            self.delta, &mut self.steer_rate, controls.steer, &self.c.params, dt, Some(kin), self.s.u,
        );
        let (dl, dr) = self.steer_angles();
        // Static toe: toe-in turns the left wheel right (negative) and the
        // right wheel left.
        let (toe_f, toe_r) = (
            self.c.params.suspension.toe_in_front_deg.to_radians(),
            self.c.params.suspension.toe_in_rear_deg.to_radians(),
        );
        let steer = [dl - toe_f, dr + toe_f, -toe_r, toe_r];
        let body = self.body();

        let (u, v, r) = (self.s.u, self.s.v, self.s.r);
        let speed = u.hypot(v);
        let low_speed = (speed / 3.0).min(1.0);
        let (downforce, drag) = self.c.params.aero_forces(speed);

        let p = &self.c.params;
        let sp = &p.suspension;
        let m = p.mass_kg;
        let l = p.wheelbase_m;
        let radius = p.tyre_radius_m;
        let (tf, tr) = (p.track_front_m, p.track_rear_m);
        // Lateral offset of each patch (left positive) and its distance ahead
        // of the CG.
        let half_t = [tf * 0.5, -tf * 0.5, tr * 0.5, -tr * 0.5];
        let arm = [p_a, p_a, -p_b, -p_b];

        // ---- vertical loads -------------------------------------------------
        // Static and aero, per corner.
        let w = p.weight();
        let aero_f = downforce * p.aero.front_frac * 0.5;
        let aero_r = downforce * (1.0 - p.aero.front_frac) * 0.5;
        let stat_f = w * p_b / l * 0.5 + aero_f;
        let stat_r = w * p_a / l * 0.5 + aero_r;

        // What reaches the ground WITHOUT going through the springs: the
        // unsprung masses' own transfer, the roll-centre (geometric) share of
        // the sprung lateral transfer, and the anti-geometry share of the
        // sprung longitudinal transfer. Instant, as it is on the car.
        let (ax, ay) = (self.ax, self.ay);
        let (mu_f, mu_r) = (2.0 * p.unsprung_front_kg, 2.0 * p.unsprung_rear_kg);
        let ms_f = body.ms * p.weight_dist_front;
        let ms_r = body.ms * (1.0 - p.weight_dist_front);
        let geo_lat_f = (ms_f * p.roll.rc_front_m + mu_f * radius) * ay / tf;
        let geo_lat_r = (ms_r * p.roll.rc_rear_m + mu_r * radius) * ay / tr;
        let anti = if ax < 0.0 {
            0.5 * (sp.anti_dive_front + sp.anti_lift_rear)
        } else {
            0.5 * sp.anti_squat_rear
        };
        // Positive = onto the front axle.
        let geo_long = -((mu_f + mu_r) * radius + anti * body.ms * body.hs) * ax / l;

        // The elastic path: whatever the body's roll and pitch are loading
        // the springs and dampers with right now.
        let rsd = p.roll.rsd_front;
        let roll_moment = body.k_roll * self.roll + body.c_roll * self.roll_rate;
        let el_lat_f = rsd * roll_moment / tf;
        let el_lat_r = (1.0 - rsd) * roll_moment / tr;
        let el_long = (body.k_pitch * self.pitch + body.c_pitch * self.pitch_rate) / l;

        let long_f = 0.5 * (geo_long + el_long);
        let long_r = -long_f;
        // Positive ay (a left turn) and positive roll load the RIGHT tyres.
        let lat_f = geo_lat_f + el_lat_f;
        let lat_r = geo_lat_r + el_lat_r;
        let fz = [
            (stat_f + long_f - lat_f).max(0.0),
            (stat_f + long_f + lat_f).max(0.0),
            (stat_r + long_r - lat_r).max(0.0),
            (stat_r + long_r + lat_r).max(0.0),
        ];

        // ---- camber, each wheel to the road (rad, + = top leaning left) -----
        // SAE camber is negative with the top inboard; inboard is -y for a
        // left wheel and +y for a right one, so the lean is +camber on the
        // left and -camber on the right. Body roll (right side down) drags
        // both wheels' tops toward -y: through the linkage by the camber
        // gain, and through the tyres' own squash one-for-one.
        let (sf, sr) = (sp.static_camber_front_deg.to_radians(), sp.static_camber_rear_deg.to_radians());
        let roll_lean_f = self.roll * (sp.camber_gain_roll_front * body.susp_share_f + (1.0 - body.susp_share_f));
        let roll_lean_r = self.roll * (sp.camber_gain_roll_rear * body.susp_share_r + (1.0 - body.susp_share_r));
        // Bump from pitch: nose down compresses the front by a.theta and
        // extends the rear by b.theta.
        let bump_f = p_a * self.pitch;
        let bump_r = -p_b * self.pitch;
        let cam_bump_f = (sp.camber_gain_bump_front_deg_m * bump_f).to_radians();
        let cam_bump_r = (sp.camber_gain_bump_rear_deg_m * bump_r).to_radians();
        // Steer: caster leans both wheels toward the turn (the outer one
        // negative, the inner positive); KPI adds positive camber to both.
        let (sin_n, sin_l) = (p.steering.caster_rad.sin(), p.steering.kpi_rad.sin());
        let steer_cam = |d: f64| (sin_n * d.sin(), sin_l * (1.0 - d.cos()));
        let (nl, kl) = steer_cam(dl);
        let (nr, kr) = steer_cam(dr);
        let gamma = [
            (sf + cam_bump_f + kl) + nl - roll_lean_f,
            -(sf + cam_bump_f + kr) + nr - roll_lean_f,
            (sr + cam_bump_r) - roll_lean_r,
            -(sr + cam_bump_r) - roll_lean_r,
        ];

        // ---- per-corner slips and tyre forces -------------------------------
        let relax_len = self.c.tyre.relaxation_length();
        let blend = if relax_len > 0.0 { ((speed / relax_len) * dt).min(1.0) } else { 1.0 };

        let mut kappa = [0.0f64; 4];
        let mut stiff = [0.0f64; 4];
        let mut fx = [0.0f64; 4];
        let mut fy = [0.0f64; 4];
        let mut util = [0.0f64; 4];
        let mut trail = [0.0f64; 4];
        for i in 0..4 {
            // Patch velocity in the body frame, then in the wheel's frame.
            let vx = u - r * half_t[i];
            let vy = v + r * arm[i];
            let (cs, ss) = (steer[i].cos(), steer[i].sin());
            let vx_w = vx * cs + vy * ss;
            let vy_w = vy * cs - vx * ss;
            let alpha_raw = -vy_w.atan2(vx_w.abs().max(0.5));
            self.alpha_lag[i] += (alpha_raw - self.alpha_lag[i]) * blend;

            let k_den = vx_w.abs().max(2.0);
            kappa[i] = (self.w[i] * radius - vx_w) / k_den;

            let slip = Slip { alpha: self.alpha_lag[i], kappa: kappa[i] };
            let f = self.c.tyre.forces_cambered(slip, fz[i], gamma[i]);
            let f2 = self.c.tyre.forces_cambered(Slip { kappa: kappa[i] + KAPPA_H, ..slip }, fz[i], gamma[i]);
            stiff[i] = dt * radius * radius * ((f2.fx - f.fx) / KAPPA_H).max(0.0) / k_den;
            let grip = if i == FL || i == FR { (p.front_grip_factor * DT_FRONT_GRIP_SCALE).min(1.0) } else { 1.0 };
            fx[i] = f.fx;
            fy[i] = f.fy * low_speed * grip;
            util[i] = f.utilisation;
            trail[i] = f.trail;
        }

        // ---- resolve into the body frame and sum ----------------------------
        let (mut fx_body, mut fy_body, mut mz) = (0.0, 0.0, 0.0);
        let mut fxb = [0.0f64; 4];
        for i in 0..4 {
            let (c, s) = (steer[i].cos(), steer[i].sin());
            fxb[i] = fx[i] * c - fy[i] * s;
            let fyb = fx[i] * s + fy[i] * c;
            fx_body += fxb[i];
            fy_body += fyb;
            // Lateral force about the CG, longitudinal through its offset.
            mz += arm[i] * fyb - half_t[i] * fxb[i];
        }

        let fz_sum: f64 = fz.iter().sum();
        let roll_res = p.crr * fz_sum * if u >= 0.0 { 1.0 } else { -1.0 };
        // Drag against the velocity, not the nose (see bicycle.rs).
        let (drag_x, drag_y) = if speed > 1e-9 { (drag * u / speed, drag * v / speed) } else { (0.0, 0.0) };

        let du = (fx_body - drag_x - roll_res) / m + v * r;
        let dv = (fy_body - drag_y) / m - u * r;
        let dr = mz / p.izz_kg_m2;

        // ---- steering feel ---------------------------------------------------
        // Each front tyre's lateral force through its own pneumatic trail and
        // the mechanical trail, and their longitudinal forces through the
        // scrub radius -- the pair's moment about the kingpins.
        let mech = p.mechanical_trail();
        let align = -(fy[FL] * (trail[FL] + mech) + fy[FR] * (trail[FR] + mech));
        let fz_front = fz[FL] + fz[FR];
        let trail_front = if fz_front > 1.0 { (trail[FL] * fz[FL] + trail[FR] * fz[FR]) / fz_front } else { 0.0 };
        let scrub_nm = p.steering.scrub_m * (fx[FR] - fx[FL]);

        // ---- driveline and wheels -------------------------------------------
        let w_r_mean = 0.5 * (self.w[RL] + self.w[RR]);
        let drive = self.c.powertrain.step(dt, controls.throttle, w_r_mean, speed);

        let brake_total = controls.brake.clamp(0.0, 1.0) * p.brakes.max_torque_nm;
        let tb_f = brake_total * p.brakes.bias_front * 0.5;
        let tb_r = brake_total * (1.0 - p.brakes.bias_front) * 0.5;

        // Fronts: free wheels with friction brakes.
        let mut dw = [0.0f64; 4];
        for i in [FL, FR] {
            let inertia = p.wheel_inertia_front_kg_m2 + stiff[i];
            let t_free = -fx[i] * radius;
            dw[i] = (t_free - brake_torque(self.w[i], t_free, inertia, tb_f, dt)) / inertia;
        }

        // Rears: the Salisbury pair, exactly as the bicycle solver has it --
        // driveline inertia on the carrier, the clutch torque limited so it
        // cannot overshoot, brakes as friction through the coupled pair.
        let dfp = &p.diff;
        let t_in = drive.wheel_torque_nm;
        let lock_frac = if t_in >= 0.0 { dfp.power_lock } else { dfp.coast_lock };
        let t_cap = lock_frac * t_in.abs() + dfp.preload_nm;
        let d_w_rear = self.w[RR] - self.w[RL];
        let q = 0.25 * drive.added_wheel_inertia;
        let i_l = p.wheel_inertia_rear_kg_m2 + stiff[RL];
        let i_r = p.wheel_inertia_rear_kg_m2 + stiff[RR];
        let det = (i_l * i_r + q * (i_l + i_r)).max(1e-9);
        let anti_j = det / (i_l + i_r + 4.0 * q).max(1e-9);
        let t_spring = 0.5 * t_cap * (d_w_rear / dfp.stick_rad_s.max(1e-4)).tanh();
        let t_stop = anti_j * d_w_rear.abs() / dt;
        let t_lock = t_spring.signum() * t_spring.abs().min(t_stop);
        self.t_lock_last = t_lock;
        let t_rl = 0.5 * t_in + t_lock;
        let t_rr = 0.5 * t_in - t_lock;
        let p_l_free = t_rl - fx[RL] * radius;
        let p_r_free = t_rr - fx[RR] * radius;
        let (stop_l, stop_r) = (-self.w[RL] / dt, -self.w[RR] / dt);
        let tb_rl = brake_torque(0.0, p_l_free - (i_l + q) * stop_l - q * stop_r, 1.0, tb_r, dt);
        let tb_rr = brake_torque(0.0, p_r_free - q * stop_l - (i_r + q) * stop_r, 1.0, tb_r, dt);
        let (p_l, p_r) = (p_l_free - tb_rl, p_r_free - tb_rr);
        dw[RL] = (p_l * (i_r + q) - q * p_r) / det;
        dw[RR] = (p_r * (i_l + q) - q * p_l) / det;

        // ---- the body on its springs ----------------------------------------
        // Roll about the roll axis: the sprung mass's inertial moment through
        // its arm, gravity's (the CG swinging out as it leans), the springs
        // and dampers. Pitch about the ground: the elastic share of the
        // sprung longitudinal transfer against the pitch springs.
        let roll_drive = body.ms * ay * body.arm + body.ms * G * body.arm * self.roll;
        let roll_acc = (roll_drive - body.k_roll * self.roll - body.c_roll * self.roll_rate) / body.i_roll;
        let pitch_drive = -(1.0 - anti) * body.ms * ax * body.hs;
        let pitch_acc = (pitch_drive - body.k_pitch * self.pitch - body.c_pitch * self.pitch_rate) / body.i_pitch;
        self.roll_rate += roll_acc * dt;
        self.roll += self.roll_rate * dt;
        self.pitch_rate += pitch_acc * dt;
        self.pitch += self.pitch_rate * dt;

        // ---- integrate the chassis ------------------------------------------
        self.s.u += du * dt;
        self.s.v += dv * dt;
        self.s.r += dr * dt;
        for i in 0..4 {
            self.w[i] += dw[i] * dt;
        }
        // The rears have a driveline behind them and no reverse gear.
        self.w[RL] = self.w[RL].max(0.0);
        self.w[RR] = self.w[RR].max(0.0);
        self.ax = du - v * r;
        self.ay = dv + u * r;

        // Come to a genuine stop rather than creeping on numerical noise.
        if self.s.u.abs() < 0.25 && controls.throttle < 0.05 && self.s.speed() < 0.4 {
            self.s.u = 0.0;
            self.s.v = 0.0;
            self.s.r = 0.0;
            self.w = [0.0; 4];
            self.ax = 0.0;
            self.ay = 0.0;
            self.alpha_lag = [0.0; 4];
        }
        // u may go negative in a spin, as in the bicycle solver.

        self.s.x += (self.s.u * self.s.psi.cos() - self.s.v * self.s.psi.sin()) * dt;
        self.s.y += (self.s.u * self.s.psi.sin() + self.s.v * self.s.psi.cos()) * dt;
        self.s.psi += self.s.r * dt;

        let pt = self.c.powertrain.telemetry();
        let fz_r = fz[RL] + fz[RR];
        let util_r_axle = if fz_r > 1.0 { (util[RL] * fz[RL] + util[RR] * fz[RR]) / fz_r } else { 0.0 };
        let util_f_axle = if fz_front > 1.0 { (util[FL] * fz[FL] + util[FR] * fz[FR]) / fz_front } else { 0.0 };
        self.tel = Telemetry {
            speed: self.s.speed(),
            ax_g: self.ax / G,
            ay_g: self.ay / G,
            body_slip_deg: self.s.v.atan2(self.s.u.abs().max(0.1)).to_degrees(),
            yaw_rate_deg_s: self.s.r.to_degrees(),
            steer_rad: self.delta,
            fz,
            slip_deg: [
                self.alpha_lag[0].to_degrees(),
                self.alpha_lag[1].to_degrees(),
                self.alpha_lag[2].to_degrees(),
                self.alpha_lag[3].to_degrees(),
            ],
            kappa,
            utilisation: util,
            // Load-weighted per axle, as the bicycle reports it.
            balance: util_r_axle - util_f_axle,
            downforce_n: downforce,
            drag_n: drag,
            engine_rpm: pt.engine_rpm,
            gear: pt.gear,
            shifting: pt.shifting,
            wheel_omega_front: 0.5 * (self.w[FL] + self.w[FR]),
            wheel_omega_rear: 0.5 * (self.w[RL] + self.w[RR]),
            drive_force_n: fxb[RL] + fxb[RR],
            locked: drive.locked,
            diff_nm: self.t_lock_last,
            kingpin_torque_nm: align,
            rim_torque_nm: align * p.rim_torque_ratio(),
            trail_front_m: trail_front,
            mech_trail_m: mech,
            scrub_moment_nm: scrub_nm,
            roll_deg: self.roll.to_degrees(),
            pitch_deg: self.pitch.to_degrees(),
            camber_deg: [gamma[0].to_degrees(), gamma[1].to_degrees(), gamma[2].to_degrees(), gamma[3].to_degrees()],
        };
    }
}
