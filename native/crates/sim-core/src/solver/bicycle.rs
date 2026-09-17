//! Level 2 — transient bicycle model.
//!
//! Each axle is one steering/driving unit, but grip is still evaluated on two
//! contact patches: the axle's load is split by the lateral transfer it is
//! carrying and the tyre is called for the inner and outer patch separately.
//! Because μ falls with load, those two never sum to what an evenly loaded
//! pair would make — which is the mechanism that turns roll-stiffness
//! distribution into understeer balance, and why this model responds to the
//! ARB setting at all.
//!
//! "Transient" is meant literally, in four places: the lateral equation keeps
//! m·u·r so yaw response overshoots; wheel speeds are states so slip ratio is
//! dynamic; slip angles pass through a relaxation-length lag; and load transfer
//! is driven by the previous substep's measured accelerations so it settles
//! rather than teleporting.

use super::{advance_steer_capped, Chassis, ChassisState, Controls, Fidelity, Solver, Telemetry, SUBSTEP};
use crate::powertrain::PowertrainModel;
use crate::tyre::{Slip, TyreModel};
use crate::vehicle::{VehicleParams, G};

/// Finite-difference step in slip ratio for the wheel update's implicit term.
const KAPPA_H: f64 = 1e-4;

pub struct BicycleSolver {
    c: Chassis,
    s: ChassisState,
    /// Front / rear axle speeds (rad/s).
    w_f: f64,
    w_r: f64,
    /// Relaxation-lagged slip angles (rad).
    a_f: f64,
    a_r: f64,
    delta: f64,
    /// Steering servo velocity state (rad/s).
    steer_rate: f64,
    ax: f64,
    ay: f64,
    tel: Telemetry,
}

impl BicycleSolver {
    pub fn new(c: Chassis) -> Self {
        Self {
            c,
            s: ChassisState::default(),
            w_f: 0.0,
            w_r: 0.0,
            a_f: 0.0,
            a_r: 0.0,
            delta: 0.0,
            steer_rate: 0.0,
            ax: 0.0,
            ay: 0.0,
            tel: Telemetry::default(),
        }
    }

    /// Lateral load transfer carried by each axle (N), from the elastic
    /// (roll-stiffness), geometric (roll-centre) and unsprung paths.
    fn lateral_transfer(&self, ay: f64) -> (f64, f64) {
        let p = &self.c.params;
        let ms = p.sprung_mass();
        let ms_f = ms * p.weight_dist_front;
        let ms_r = ms * (1.0 - p.weight_dist_front);
        let unsprung_f = 2.0 * p.unsprung_front_kg;
        let unsprung_r = 2.0 * p.unsprung_rear_kg;

        let d_f = (ms * ay * p.roll.roll_arm_m * p.roll.rsd_front) / p.track_front_m
            + (ms_f * ay * p.roll.rc_front_m) / p.track_front_m
            + (unsprung_f * ay * p.tyre_radius_m) / p.track_front_m;
        let d_r = (ms * ay * p.roll.roll_arm_m * (1.0 - p.roll.rsd_front)) / p.track_rear_m
            + (ms_r * ay * p.roll.rc_rear_m) / p.track_rear_m
            + (unsprung_r * ay * p.tyre_radius_m) / p.track_rear_m;
        (d_f, d_r)
    }

    /// Sum the two contact patches of an axle at their own loads.
    fn axle_forces(&self, slip: Slip, fz_axle: f64, d_fz: f64) -> AxleForces {
        if fz_axle <= 1.0 {
            return AxleForces::default();
        }
        let half = fz_axle * 0.5;
        let shift = d_fz.abs().min(half); // the inner tyre lifts, it does not go negative
        let outer = half + shift;
        let inner = half - shift;
        let fo = self.c.tyre.forces(slip, outer);
        let fi = self.c.tyre.forces(slip, inner);
        AxleForces {
            fx: fo.fx + fi.fx,
            fy: fo.fy + fi.fy,
            utilisation: fo.utilisation.max(fi.utilisation),
            inner_fz: inner,
            outer_fz: outer,
            // Self-aligning moment about the steering axis, both tyres, with
            // the mechanical trail added to each tyre's pneumatic trail. It
            // opposes the slip, which is what makes a wheel return to centre
            // and go light before the front lets go.
            align_nm: -(fo.fy * (fo.trail + self.c.params.mechanical_trail())
                + fi.fy * (fi.trail + self.c.params.mechanical_trail())),
            trail_m: (fo.trail * outer + fi.trail * inner) / fz_axle,
        }
    }
}

/// One axle's contact patches summed, plus what force feedback needs.
#[derive(Debug, Clone, Copy, Default)]
struct AxleForces {
    fx: f64,
    fy: f64,
    utilisation: f64,
    inner_fz: f64,
    outer_fz: f64,
    align_nm: f64,
    trail_m: f64,
}

impl Solver for BicycleSolver {
    fn name(&self) -> &'static str {
        "Transient bicycle"
    }

    fn fidelity(&self) -> Fidelity {
        Fidelity::Bicycle
    }

    fn step(&mut self, dt: f64, controls: Controls) {
        let mut remaining = dt.min(0.1); // never simulate more than 100 ms of catch-up
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
        // The powertrain's reset puts the box in first for a standing start;
        // placed at speed the caller's gear has to survive, or a rolling
        // respawn at 20 m/s lands on the limiter in first.
        let gear = self.c.powertrain.telemetry().gear;
        self.s = ChassisState { u: speed, v: 0.0, r: 0.0, x, y, psi };
        self.w_f = speed / self.c.params.tyre_radius_m;
        self.w_r = self.w_f;
        self.a_f = 0.0;
        self.a_r = 0.0;
        self.delta = 0.0;
        self.steer_rate = 0.0;
        self.ax = 0.0;
        self.ay = 0.0;
        self.tel = Telemetry::default();
        self.c.powertrain.reset();
        if speed > 0.0 {
            self.c.powertrain.set_gear(gear);
            self.c.powertrain.sync_to_wheel(self.w_r);
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

impl BicycleSolver {
    fn substep(&mut self, dt: f64, controls: Controls) {
        let kin = (self.s.v + self.c.params.a() * self.s.r).atan2(self.s.u.abs().max(0.6));
        self.delta = advance_steer_capped(
            self.delta, &mut self.steer_rate, controls.steer, &self.c.params, dt, Some(kin), self.s.u,
        );
        let d = self.delta;

        let (u, v, r) = (self.s.u, self.s.v, self.s.r);
        let speed = u.hypot(v);
        let u_safe = u.abs().max(0.6);

        let p_a = self.c.params.a();
        let p_b = self.c.params.b();
        let (downforce, drag) = self.c.params.aero_forces(speed);

        // Axle loads: static + longitudinal transfer + aero.
        let (l, w, h) = (
            self.c.params.wheelbase_m,
            self.c.params.weight(),
            self.c.params.cg_height_m,
        );
        let m = self.c.params.mass_kg;
        let front_frac = self.c.params.aero.front_frac;
        let fz_f = ((w * p_b) / l - (m * self.ax * h) / l + downforce * front_frac).max(0.0);
        let fz_r = ((w * p_a) / l + (m * self.ax * h) / l + downforce * (1.0 - front_frac)).max(0.0);

        let (d_fz_f, d_fz_r) = self.lateral_transfer(self.ay);

        // Slip angles with relaxation lag.
        let a_f_raw = d - (v + p_a * r).atan2(u_safe);
        let a_r_raw = -(v - p_b * r).atan2(u_safe);
        let relax_len = self.c.tyre.relaxation_length();
        let blend = if relax_len > 0.0 {
            ((u.abs() / relax_len) * dt).min(1.0)
        } else {
            1.0
        };
        self.a_f += (a_f_raw - self.a_f) * blend;
        self.a_r += (a_r_raw - self.a_r) * blend;

        // Slip ratios from the wheel-speed states.
        let k_den = u.abs().max(2.0);
        let radius = self.c.params.tyre_radius_m;
        let k_f = (self.w_f * radius - u) / k_den;
        let k_r = (self.w_r * radius - u) / k_den;

        let mut af = self.axle_forces(Slip { alpha: self.a_f, kappa: k_f }, fz_f, d_fz_f);
        let ar = self.axle_forces(Slip { alpha: self.a_r, kappa: k_r }, fz_r, d_fz_r);
        // Front lateral peak relative to the rear (`front_grip_factor`). The
        // fitted curve is linear in mu at a given slip, so scaling the force is
        // exactly a mu scaling; the aligning moment is Fy through the trail and
        // scales with it, utilisation does not. Same operation, same order as
        // the JS build, so the two stay bit-identical.
        af.fy *= self.c.params.front_grip_factor;
        af.align_nm *= self.c.params.front_grip_factor;
        let (fx_f, fy_f, util_f, fzi_f, fzo_f) = (af.fx, af.fy, af.utilisation, af.inner_fz, af.outer_fz);
        let (fx_r, fy_r, util_r, fzi_r, fzo_r) = (ar.fx, ar.fy, ar.utilisation, ar.inner_fz, ar.outer_fz);

        // Resolve the front through the steer angle.
        let (cd, sd) = (d.cos(), d.sin());
        let fx_fb = fx_f * cd - fy_f * sd;
        let fy_fb = fx_f * sd + fy_f * cd;

        let roll_res = self.c.params.crr * (fz_f + fz_r) * if u >= 0.0 { 1.0 } else { -1.0 };

        let du = (fx_fb + fx_r - drag - roll_res) / m + v * r;
        let dv = (fy_fb + fy_r) / m - u * r;
        let dr = (p_a * fy_fb - p_b * fy_r) / self.c.params.izz_kg_m2;

        // Driveline.
        let drive = self.c.powertrain.step(dt, controls.throttle, self.w_r, speed);

        // Wheel dynamics; brake torque must not drive a wheel backwards.
        let brake_total = controls.brake.clamp(0.0, 1.0) * self.c.params.brakes.max_torque_nm;
        let tb_f = brake_total * self.c.params.brakes.bias_front;
        let tb_r = brake_total * (1.0 - self.c.params.brakes.bias_front);
        let iw_f = 2.0 * self.c.params.wheel_inertia_front_kg_m2;
        let iw_r = 2.0 * self.c.params.wheel_inertia_rear_kg_m2 + drive.added_wheel_inertia;

        // Implicit in the tyre's longitudinal stiffness; see bicycle.js for
        // why (explicit Euler is unstable under ~4.5 m/s at 500 Hz). Same
        // finite difference, same order of operations, so the two stay
        // bit-identical.
        let dfx_f = ((self.axle_forces(Slip { alpha: self.a_f, kappa: k_f + KAPPA_H }, fz_f, d_fz_f).fx - fx_f) / KAPPA_H).max(0.0);
        let dfx_r = ((self.axle_forces(Slip { alpha: self.a_r, kappa: k_r + KAPPA_H }, fz_r, d_fz_r).fx - fx_r) / KAPPA_H).max(0.0);
        let stiff_f = dt * radius * radius * dfx_f / k_den;
        let stiff_r = dt * radius * radius * dfx_r / k_den;
        let mut dw_f = (-fx_f * radius - self.w_f.signum() * tb_f) / (iw_f + stiff_f);
        let mut dw_r =
            (drive.wheel_torque_nm - fx_r * radius - self.w_r.signum() * tb_r) / (iw_r + stiff_r);
        if self.w_f > 0.0 && self.w_f + dw_f * dt < 0.0 && tb_f > 0.0 {
            dw_f = -self.w_f / dt;
        }
        if self.w_r > 0.0
            && self.w_r + dw_r * dt < 0.0
            && tb_r > 0.0
            && drive.wheel_torque_nm <= 0.0
        {
            dw_r = -self.w_r / dt;
        }

        self.s.u += du * dt;
        self.s.v += dv * dt;
        self.s.r += dr * dt;
        self.w_f = (self.w_f + dw_f * dt).max(0.0);
        self.w_r = (self.w_r + dw_r * dt).max(0.0);

        // The measured accelerations that feed the next substep's transfer.
        self.ax = du - v * r;
        self.ay = dv + u * r;

        // Come to a genuine stop rather than creeping on numerical noise.
        if self.s.u.abs() < 0.25 && controls.throttle < 0.05 && self.s.speed() < 0.4 {
            self.s.u = 0.0;
            self.s.v = 0.0;
            self.s.r = 0.0;
            self.w_f = 0.0;
            self.w_r = 0.0;
            self.ax = 0.0;
            self.ay = 0.0;
            // The lagged slip angles too: relaxation is speed-proportional,
            // so at rest they would hold the last corner's slip forever and
            // leave a static aligning torque on the steering wheel.
            self.a_f = 0.0;
            self.a_r = 0.0;
        }
        if self.s.u < 0.0 {
            self.s.u = 0.0; // no reverse gear, and the tyre model is not valid backwards
        }

        self.s.x += (self.s.u * self.s.psi.cos() - self.s.v * self.s.psi.sin()) * dt;
        self.s.y += (self.s.u * self.s.psi.sin() + self.s.v * self.s.psi.cos()) * dt;
        self.s.psi += self.s.r * dt;

        let pt = self.c.powertrain.telemetry();
        self.tel = Telemetry {
            speed: self.s.speed(),
            ax_g: self.ax / G,
            ay_g: self.ay / G,
            body_slip_deg: self.s.v.atan2(self.s.u.max(0.1)).to_degrees(),
            yaw_rate_deg_s: self.s.r.to_degrees(),
            steer_rad: d,
            // Left/right by the sign of the transfer: positive ay is a left
            // turn, which loads the right-hand tyres.
            fz: if self.ay >= 0.0 {
                [fzi_f, fzo_f, fzi_r, fzo_r]
            } else {
                [fzo_f, fzi_f, fzo_r, fzi_r]
            },
            slip_deg: [
                self.a_f.to_degrees(),
                self.a_f.to_degrees(),
                self.a_r.to_degrees(),
                self.a_r.to_degrees(),
            ],
            kappa: [k_f, k_f, k_r, k_r],
            utilisation: [util_f, util_f, util_r, util_r],
            balance: util_r - util_f,
            downforce_n: downforce,
            drag_n: drag,
            engine_rpm: pt.engine_rpm,
            gear: pt.gear,
            shifting: pt.shifting,
            wheel_omega_front: self.w_f,
            wheel_omega_rear: self.w_r,
            drive_force_n: fx_r,
            locked: drive.locked,
            kingpin_torque_nm: af.align_nm,
            rim_torque_nm: af.align_nm * self.c.params.rim_torque_ratio(),
            trail_front_m: af.trail_m,
            mech_trail_m: self.c.params.mechanical_trail(),
        };
    }
}
