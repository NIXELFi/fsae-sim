//! Level 3 -- double-track (four-corner) model.
//!
//! Same three chassis degrees of freedom as the bicycle, but every contact
//! patch is its own: four vertical loads, four slip angles, four slip ratios,
//! four wheel-speed states. What that buys over the bicycle model:
//!
//!   * Ackermann is real, because there are two front wheels to steer by
//!     different amounts.
//!   * Longitudinal forces generate yaw moment through their lateral offset,
//!     so braking or driving asymmetry steers the car.
//!   * Load transfer is applied per corner rather than reconstructed per axle,
//!     so combined longitudinal + lateral transfer diagonalises properly.
//!
//! The rear differential is open -- equal torque to both wheels. A locked or
//! limited-slip diff is the obvious next step and would go here.

use super::{advance_steer, Chassis, ChassisState, Controls, Fidelity, Solver, Telemetry, SUBSTEP,
            FL, FR, RL, RR};
use crate::powertrain::PowertrainModel;
use crate::tyre::{Slip, TyreModel};
use crate::vehicle::{VehicleParams, G};

/// Finite-difference step in slip ratio for the wheel update's implicit term.
const KAPPA_H: f64 = 1e-4;

pub struct DoubleTrackSolver {
    c: Chassis,
    s: ChassisState,
    w: [f64; 4],
    alpha_lag: [f64; 4],
    delta: f64,
    steer_rate: f64,
    ax: f64,
    ay: f64,
    tel: Telemetry,
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
}

impl Solver for DoubleTrackSolver {
    fn name(&self) -> &'static str {
        "Double track (four corner)"
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
        self.s = ChassisState { u: speed, v: 0.0, r: 0.0, x, y, psi };
        let w = speed / self.c.params.tyre_radius_m;
        self.w = [w; 4];
        self.alpha_lag = [0.0; 4];
        self.delta = 0.0;
        self.ax = 0.0;
        self.ay = 0.0;
        self.tel = Telemetry::default();
        // Keep the caller's gear through a rolling reset (see bicycle.rs).
        let gear = self.c.powertrain.telemetry().gear;
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
        self.delta = advance_steer(self.delta, &mut self.steer_rate, controls.steer, &self.c.params, dt);
        let (dl, dr) = self.steer_angles();
        let steer = [dl, dr, 0.0, 0.0];

        let p_a = self.c.params.a();
        let p_b = self.c.params.b();
        let (u, v, r) = (self.s.u, self.s.v, self.s.r);
        let speed = u.hypot(v);
        let (downforce, drag) = self.c.params.aero_forces(speed);

        let p = &self.c.params;
        let m = p.mass_kg;
        let l = p.wheelbase_m;
        let half_t = [
            p.track_front_m * 0.5,
            -p.track_front_m * 0.5,
            p.track_rear_m * 0.5,
            -p.track_rear_m * 0.5,
        ];
        let arm = [p_a, p_a, -p_b, -p_b];

        // ---- per-corner vertical loads ----
        // Static + longitudinal transfer split by axle, then lateral transfer
        // split by axle roll stiffness, then aero.
        let fz_f_axle = (p.weight() * p_b) / l - (m * self.ax * p.cg_height_m) / l
            + downforce * p.aero.front_frac;
        let fz_r_axle = (p.weight() * p_a) / l + (m * self.ax * p.cg_height_m) / l
            + downforce * (1.0 - p.aero.front_frac);

        let ms = p.sprung_mass();
        let ms_f = ms * p.weight_dist_front;
        let ms_r = ms * (1.0 - p.weight_dist_front);
        let unsprung_f = 2.0 * p.unsprung_front_kg;
        let unsprung_r = 2.0 * p.unsprung_rear_kg;
        let d_fz_f = (ms * self.ay * p.roll.roll_arm_m * p.roll.rsd_front) / p.track_front_m
            + (ms_f * self.ay * p.roll.rc_front_m) / p.track_front_m
            + (unsprung_f * self.ay * p.tyre_radius_m) / p.track_front_m;
        let d_fz_r = (ms * self.ay * p.roll.roll_arm_m * (1.0 - p.roll.rsd_front)) / p.track_rear_m
            + (ms_r * self.ay * p.roll.rc_rear_m) / p.track_rear_m
            + (unsprung_r * self.ay * p.tyre_radius_m) / p.track_rear_m;

        // Positive ay is a left turn, which loads the right-hand tyres.
        let mut fz = [0.0f64; 4];
        fz[FL] = (fz_f_axle * 0.5 - d_fz_f).max(0.0);
        fz[FR] = (fz_f_axle * 0.5 + d_fz_f).max(0.0);
        fz[RL] = (fz_r_axle * 0.5 - d_fz_r).max(0.0);
        fz[RR] = (fz_r_axle * 0.5 + d_fz_r).max(0.0);

        // ---- per-corner slips ----
        let relax_len = self.c.tyre.relaxation_length();
        let blend = if relax_len > 0.0 {
            ((u.abs() / relax_len) * dt).min(1.0)
        } else {
            1.0
        };
        let radius = p.tyre_radius_m;

        let mut kappa = [0.0f64; 4];
        let mut stiff = [0.0f64; 4];
        let mut forces = [(0.0f64, 0.0f64, 0.0f64); 4];
        for i in 0..4 {
            // Velocity of this contact patch in the body frame.
            let vx = u - r * half_t[i];
            let vy = v + r * arm[i];
            let vx_safe = vx.abs().max(0.6);
            let alpha_raw = steer[i] - vy.atan2(vx_safe);
            self.alpha_lag[i] += (alpha_raw - self.alpha_lag[i]) * blend;

            let k_den = vx.abs().max(2.0);
            kappa[i] = (self.w[i] * radius - vx) / k_den;

            let f = self
                .c
                .tyre
                .forces(Slip { alpha: self.alpha_lag[i], kappa: kappa[i] }, fz[i]);
            // Linearised reaction for the implicit wheel update below.
            let f2 = self.c.tyre.forces(Slip { alpha: self.alpha_lag[i], kappa: kappa[i] + KAPPA_H }, fz[i]);
            stiff[i] = dt * radius * radius * ((f2.fx - f.fx) / KAPPA_H).max(0.0) / k_den;
            // Front lateral peak relative to the rear, as in the bicycle solver.
            let fy = if i == FL || i == FR { f.fy * p.front_grip_factor } else { f.fy };
            forces[i] = (f.fx, fy, f.utilisation);
        }

        // ---- resolve into the body frame and sum ----
        let (mut fx_body, mut fy_body, mut mz) = (0.0, 0.0, 0.0);
        for i in 0..4 {
            let (c, s) = (steer[i].cos(), steer[i].sin());
            let fxb = forces[i].0 * c - forces[i].1 * s;
            let fyb = forces[i].0 * s + forces[i].1 * c;
            fx_body += fxb;
            fy_body += fyb;
            // Lateral force about the CG, plus longitudinal force through its
            // lateral offset -- the term a bicycle model cannot have.
            mz += arm[i] * fyb - half_t[i] * fxb;
        }

        let fz_sum: f64 = fz.iter().sum();
        let roll_res = p.crr * fz_sum * if u >= 0.0 { 1.0 } else { -1.0 };

        let du = (fx_body - drag - roll_res) / m + v * r;
        let dv = fy_body / m - u * r;
        let dr = mz / p.izz_kg_m2;

        // ---- driveline: open diff, equal torque to both rears ----
        let rear_omega = 0.5 * (self.w[RL] + self.w[RR]);
        let drive = self.c.powertrain.step(dt, controls.throttle, rear_omega, speed);
        let per_rear_torque = drive.wheel_torque_nm * 0.5;
        let added = drive.added_wheel_inertia * 0.5;

        let brake_total = controls.brake.clamp(0.0, 1.0) * p.brakes.max_torque_nm;
        let tb = [
            brake_total * p.brakes.bias_front * 0.5,
            brake_total * p.brakes.bias_front * 0.5,
            brake_total * (1.0 - p.brakes.bias_front) * 0.5,
            brake_total * (1.0 - p.brakes.bias_front) * 0.5,
        ];

        for i in 0..4 {
            let inertia = if i == RL || i == RR {
                p.wheel_inertia_rear_kg_m2 + added
            } else {
                p.wheel_inertia_front_kg_m2
            };
            let drive_t = if i == RL || i == RR { per_rear_torque } else { 0.0 };
            let mut dw = (drive_t - forces[i].0 * radius - self.w[i].signum() * tb[i]) / (inertia + stiff[i]);
            if self.w[i] > 0.0 && self.w[i] + dw * dt < 0.0 && tb[i] > 0.0 && drive_t <= 0.0 {
                dw = -self.w[i] / dt;
            }
            self.w[i] = (self.w[i] + dw * dt).max(0.0);
        }

        self.s.u += du * dt;
        self.s.v += dv * dt;
        self.s.r += dr * dt;
        self.ax = du - v * r;
        self.ay = dv + u * r;

        if self.s.u.abs() < 0.25 && controls.throttle < 0.05 && self.s.speed() < 0.4 {
            self.s.u = 0.0;
            self.s.v = 0.0;
            self.s.r = 0.0;
            self.w = [0.0; 4];
            self.ax = 0.0;
            self.ay = 0.0;
        }
        if self.s.u < 0.0 {
            self.s.u = 0.0;
        }

        self.s.x += (self.s.u * self.s.psi.cos() - self.s.v * self.s.psi.sin()) * dt;
        self.s.y += (self.s.u * self.s.psi.sin() + self.s.v * self.s.psi.cos()) * dt;
        self.s.psi += self.s.r * dt;

        let pt = self.c.powertrain.telemetry();
        let util_f = forces[FL].2.max(forces[FR].2);
        let util_r = forces[RL].2.max(forces[RR].2);
        self.tel = Telemetry {
            speed: self.s.speed(),
            ax_g: self.ax / G,
            ay_g: self.ay / G,
            body_slip_deg: self.s.v.atan2(self.s.u.max(0.1)).to_degrees(),
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
            utilisation: [forces[0].2, forces[1].2, forces[2].2, forces[3].2],
            balance: util_r - util_f,
            downforce_n: downforce,
            drag_n: drag,
            engine_rpm: pt.engine_rpm,
            gear: pt.gear,
            shifting: pt.shifting,
            wheel_omega_front: 0.5 * (self.w[FL] + self.w[FR]),
            wheel_omega_rear: rear_omega,
            ..Default::default()
        };
    }
}
