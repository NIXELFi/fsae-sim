//! Level 1 — grip-limited point mass.
//!
//! No yaw dynamics and no slip angles: steering commands a path curvature and
//! the car follows it up to whatever the friction ellipse allows, then washes
//! out. It cannot spin, it has no balance, and it never gets loose.
//!
//! That is the point. When the question is about grip budget, gearing or lap
//! time rather than car control, this answers it for a fraction of the work
//! and with nothing to tune. It is the same class of model as a quasi-steady
//! lap sim.

use super::{advance_steer, Chassis, ChassisState, Controls, Fidelity, Solver, Telemetry, SUBSTEP};
use crate::powertrain::PowertrainModel;
use crate::tyre::TyreModel;
use crate::vehicle::{VehicleParams, G};

pub struct PointMassSolver {
    c: Chassis,
    s: ChassisState,
    delta: f64,
    steer_rate: f64,
    ax: f64,
    ay: f64,
    tel: Telemetry,
}

impl PointMassSolver {
    pub fn new(c: Chassis) -> Self {
        Self {
            c,
            s: ChassisState::default(),
            delta: 0.0,
            steer_rate: 0.0,
            ax: 0.0,
            ay: 0.0,
            tel: Telemetry::default(),
        }
    }
}

impl Solver for PointMassSolver {
    fn name(&self) -> &'static str {
        "Grip-limited point mass"
    }

    fn fidelity(&self) -> Fidelity {
        Fidelity::PointMass
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
        self.delta = 0.0;
        self.ax = 0.0;
        self.ay = 0.0;
        self.tel = Telemetry::default();
        let gear = self.c.powertrain.telemetry().gear;
        self.c.powertrain.reset();
        if speed > 0.0 {
            self.c.powertrain.set_gear(gear);
            self.c.powertrain.sync_to_wheel(speed / self.c.params.tyre_radius_m);
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

impl PointMassSolver {
    fn substep(&mut self, dt: f64, controls: Controls) {
        let p = &self.c.params;
        self.delta = advance_steer(self.delta, &mut self.steer_rate, controls.steer, p, dt);

        let u = self.s.u.max(0.0);
        let (downforce, drag) = p.aero_forces(u);
        let fz_total = p.weight() + downforce;
        let corner = fz_total / 4.0;
        let (mu_x, mu_y) = self.c.tyre.peak_mu(corner);

        // Lateral: kinematic curvature demand, clipped by grip.
        let curvature_demand = self.delta.tan() / p.wheelbase_m;
        let ay_max = mu_y * fz_total / p.mass_kg;
        let curvature_max = if u > 0.5 { ay_max / (u * u) } else { f64::INFINITY };
        let curvature = curvature_demand.signum() * curvature_demand.abs().min(curvature_max);
        let ay = u * u * curvature;

        // Longitudinal: whatever the ellipse has left after the corner.
        let lat_use = if ay_max > 0.0 { (ay / ay_max).abs().min(1.0) } else { 0.0 };
        let fx_max = mu_x * fz_total * (1.0 - lat_use * lat_use).max(0.0).sqrt();

        let wheel_omega = u / p.tyre_radius_m;
        let drive = self.c.powertrain.step(dt, controls.throttle, wheel_omega, u);
        let drive_force = drive.wheel_torque_nm / p.tyre_radius_m;
        let brake_force =
            controls.brake.clamp(0.0, 1.0) * p.brakes.max_torque_nm / p.tyre_radius_m;

        let net = (drive_force - brake_force).clamp(-fx_max, fx_max);
        let roll_res = p.crr * fz_total;
        let du = (net - drag - if u > 0.1 { roll_res } else { 0.0 }) / p.mass_kg;

        self.s.u = (u + du * dt).max(0.0);
        self.s.r = self.s.u * curvature;
        self.ax = du;
        self.ay = ay;

        self.s.x += self.s.u * self.s.psi.cos() * dt;
        self.s.y += self.s.u * self.s.psi.sin() * dt;
        self.s.psi += self.s.r * dt;

        let pt = self.c.powertrain.telemetry();
        let util = (lat_use * lat_use + (net / fx_max.max(1.0)).powi(2)).sqrt();
        self.tel = Telemetry {
            speed: self.s.u,
            ax_g: self.ax / G,
            ay_g: self.ay / G,
            body_slip_deg: 0.0,
            yaw_rate_deg_s: self.s.r.to_degrees(),
            steer_rad: self.delta,
            fz: [corner; 4],
            slip_deg: [0.0; 4],
            kappa: [0.0; 4],
            utilisation: [util; 4],
            balance: 0.0,
            downforce_n: downforce,
            drag_n: drag,
            engine_rpm: pt.engine_rpm,
            gear: pt.gear,
            shifting: pt.shifting,
            wheel_omega_front: wheel_omega,
            wheel_omega_rear: wheel_omega,
            ..Default::default()
        };
    }
}
