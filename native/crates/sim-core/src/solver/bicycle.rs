//! Level 2 -- transient bicycle model.
//!
//! Each axle is one steering/driving unit, but grip is still evaluated on two
//! contact patches: the axle's load is split by the lateral transfer it is
//! carrying and the tyre is called for the inner and outer patch separately.
//! Because mu falls with load, those two never sum to what an evenly loaded
//! pair would make -- which is the mechanism that turns roll-stiffness
//! distribution into understeer balance, and why this model responds to the
//! ARB setting at all.
//!
//! "Transient" is meant literally, in four places: the lateral equation keeps
//! m.u.r so yaw response overshoots; wheel speeds are states so slip ratio is
//! dynamic; slip angles pass through a relaxation-length lag; and load transfer
//! is driven by the previous substep's measured accelerations so it settles
//! rather than teleporting.

use super::{advance_steer_capped, brake_torque, Chassis, ChassisState, Controls, Fidelity, Solver, Telemetry, SUBSTEP};
use crate::powertrain::PowertrainModel;
use crate::tyre::{Slip, TyreModel};
use crate::vehicle::{VehicleParams, G};

/// Finite-difference step in slip ratio for the wheel update's implicit term.
const KAPPA_H: f64 = 1e-4;

pub struct BicycleSolver {
    c: Chassis,
    s: ChassisState,
    /// Front axle speed (rad/s).
    w_f: f64,
    /// Front wheel speeds (rad/s), used only with `split_front_wheels`; kept
    /// equal to `w_f` otherwise.
    w_fl: f64,
    w_fr: f64,
    /// Rear wheel speeds (rad/s). Two of them, because a differential is the
    /// only thing between them and it is what decides the car's balance on
    /// the throttle. The front axle stays a single unit: there is nothing
    /// between the front wheels but the road.
    w_rl: f64,
    w_rr: f64,
    /// The clutch pack's transfer torque from the last substep, for telemetry.
    t_lock_last: f64,
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
            w_fl: 0.0,
            w_fr: 0.0,
            w_rl: 0.0,
            w_rr: 0.0,
            t_lock_last: 0.0,
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

        let d_f = (ms * ay * p.roll_arm() * p.roll.rsd_front) / p.track_front_m
            + (ms_f * ay * p.roll.rc_front_m) / p.track_front_m
            + (unsprung_f * ay * p.tyre_radius_m) / p.track_front_m;
        let d_r = (ms * ay * p.roll_arm() * (1.0 - p.roll.rsd_front)) / p.track_rear_m
            + (ms_r * ay * p.roll.rc_rear_m) / p.track_rear_m
            + (unsprung_r * ay * p.tyre_radius_m) / p.track_rear_m;
        (d_f, d_r)
    }

    /// Sum the two contact patches of an axle at their own loads.
    ///
    /// Each wheel gets its own slip ratio. With the front axle as one rotor
    /// (the default) both are the same number and this is operation for
    /// operation what it always was; `split_front_wheels` gives each front
    /// its own wheel speed, so the unloaded inside tyre can lock first.
    fn axle_forces(&self, alpha: f64, kappa_l: f64, kappa_r: f64, fz_axle: f64, d_fz: f64) -> AxleForces {
        if fz_axle <= 1.0 {
            return AxleForces::default();
        }
        let half = fz_axle * 0.5;
        let shift = d_fz.abs().min(half); // the inner tyre lifts, it does not go negative
        let outer = half + shift;
        let inner = half - shift;
        // Positive transfer (positive ay, a left turn) loads the RIGHT tyre.
        let right_outer = d_fz >= 0.0;
        let (kappa_o, kappa_i) = if right_outer { (kappa_r, kappa_l) } else { (kappa_l, kappa_r) };
        let fo = self.c.tyre.forces(Slip { alpha, kappa: kappa_o }, outer);
        let fi = self.c.tyre.forces(Slip { alpha, kappa: kappa_i }, inner);
        let (l, r) = if right_outer { (&fi, &fo) } else { (&fo, &fi) };
        let (fz_l, fz_r) = if right_outer { (inner, outer) } else { (outer, inner) };
        AxleForces {
            fx_left: l.fx,
            fx_right: r.fx,
            fz_left: fz_l,
            fz_right: fz_r,
            util_left: l.utilisation,
            util_right: r.utilisation,
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
            // Longitudinal forces through the scrub radius (static scrub, the
            // usual scope assumption): a force Fx at the contact patch, a
            // scrub radius outboard of its kingpin, turns that wheel by
            // -y*Fx, so the pair sums to scrub * (Fx_right - Fx_left). The
            // transfer is signed with ay, and positive ay (a left turn) loads
            // the right-hand tyre, so the outer patch is the right one then.
            // With one front rotor the split is by load alone; with
            // `split_front_wheels` a locking inside wheel changes it too.
            scrub_nm: self.c.params.steering.scrub_m * (r.fx - l.fx),
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
    /// Fx through the scrub radius, for the v2 steering-torque model only.
    scrub_nm: f64,
    /// Per-side values (left is +y), for the split front axle.
    fx_left: f64,
    fx_right: f64,
    fz_left: f64,
    fz_right: f64,
    util_left: f64,
    util_right: f64,
}

impl Solver for BicycleSolver {
    fn name(&self) -> &'static str {
        "Transient bicycle"
    }

    fn fidelity(&self) -> Fidelity {
        Fidelity::Bicycle
    }

    fn step(&mut self, dt: f64, controls: Controls) {
        let mut remaining = super::step_span(dt); // never more than 100 ms of catch-up
        let controls = controls.sanitized();
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
        self.w_fl = self.w_f;
        self.w_fr = self.w_f;
        self.w_rl = self.w_f;
        self.w_rr = self.w_f;
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
            self.c.powertrain.sync_to_wheel(self.w_rl);
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
        // The slip-angle kinematics use the real forward speed with only a
        // singularity guard, and the lateral force is faded in with speed
        // below 3 m/s instead. The old 3 m/s floor in the denominator bounded
        // the lateral loop gain but lied about the angle: a car creeping at
        // full lock reported forty-odd degrees of front slip, made a
        // kilonewton sideways and snapped into a slide at walking pace. See
        // bicycle.js, which this mirrors operation for operation.
        let u_kin = u.abs().max(0.5);
        let low_speed = (speed / 3.0).min(1.0);

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

        // Slip angles with relaxation lag, each from the velocity in its own
        // wheel's frame. The front used to be `d - atan2(v + a r, |u|)`, the
        // small-angle form: right while the car rolls forwards, wrong once it
        // does not. Sliding sideways at lock it passed 90 deg (110 measured,
        // against a true 46), where tan() changes sign and the front tyre
        // pushed WITH the slide; rolling backwards it kept the steer's
        // forward sign, so steering acted the wrong way. Resolved through the
        // steer angle and measured against |vx|, it is the angle between the
        // wheel plane and the patch velocity, in (-90, 90) deg, always.
        let (cd, sd) = (d.cos(), d.sin());
        let vy_f = v + p_a * r;
        let vx_fw = u * cd + vy_f * sd;
        let vy_fw = vy_f * cd - u * sd;
        let a_f_raw = -vy_fw.atan2(vx_fw.abs().max(0.5));
        let a_r_raw = -(v - p_b * r).atan2(u_kin);
        let relax_len = self.c.tyre.relaxation_length();
        // On the distance rolled in ANY direction, so a slide relaxes too.
        let blend = if relax_len > 0.0 {
            ((speed / relax_len) * dt).min(1.0)
        } else {
            1.0
        };
        self.a_f += (a_f_raw - self.a_f) * blend;
        self.a_r += (a_r_raw - self.a_r) * blend;

        // Slip ratios from the wheel-speed states. The front against its
        // speed ALONG THE STEERED WHEEL, not the body's u: at full lock the two
        // differ by cos(46 deg), and the front wheel speed read 30 % low.
        let k_den = u.abs().max(2.0);
        let radius = self.c.params.tyre_radius_m;
        let k_den_f = vx_fw.abs().max(2.0);
        let k_f = (self.w_f * radius - vx_fw) / k_den_f;
        // Split front axle (FFB model v2.1): each front wheel its own speed
        // state and its own forward speed, u - r*y with left positive, exactly
        // as the rear. Otherwise both fronts share the one rotor's slip ratio.
        let split = self.c.params.split_front_wheels;
        let half_track_f = self.c.params.track_front_m * 0.5;
        let (k_fl, k_fr) = if split {
            (
                (self.w_fl * radius - ((u - r * half_track_f) * cd + vy_f * sd)) / k_den_f,
                (self.w_fr * radius - ((u + r * half_track_f) * cd + vy_f * sd)) / k_den_f,
            )
        } else {
            (k_f, k_f)
        };

        // ---- the rear axle, one wheel at a time -------------------------
        // Each rear wheel carries its own load, its own forward speed and so
        // its own slip ratio; they share a slip angle, which is the one thing
        // a single-track front still gets to assume. Splitting them is what
        // makes a differential mean anything: with one rotor there is no
        // torque difference across the track and therefore no yaw moment from
        // the driven wheels at all.
        let half_r = fz_r * 0.5;
        let shift_r = d_fz_r.abs().min(half_r); // the inner tyre lifts, it does not go negative
        let (outer_r, inner_r) = (half_r + shift_r, half_r - shift_r);
        // Positive ay is a LEFT turn, which loads the right-hand tyres.
        let (fz_rl, fz_rr) = if self.ay >= 0.0 { (inner_r, outer_r) } else { (outer_r, inner_r) };
        // Forward speed at each rear contact patch: the wheel at lateral
        // offset y sees u - r*y, and left is positive y.
        let half_track_r = self.c.params.track_rear_m * 0.5;
        let u_rl = u - r * half_track_r;
        let u_rr = u + r * half_track_r;
        let k_rl = (self.w_rl * radius - u_rl) / k_den;
        let k_rr = (self.w_rr * radius - u_rr) / k_den;
        let mut f_rl = self.c.tyre.forces(Slip { alpha: self.a_r, kappa: k_rl }, fz_rl);
        let mut f_rr = self.c.tyre.forces(Slip { alpha: self.a_r, kappa: k_rr }, fz_rr);

        let mut af = self.axle_forces(self.a_f, k_fl, k_fr, fz_f, d_fz_f);
        // The low-speed fade (see `u_kin`), lateral only, before the grip
        // factor -- the same order as the JS build.
        af.fy *= low_speed;
        af.align_nm *= low_speed;
        f_rl.fy *= low_speed;
        f_rr.fy *= low_speed;
        let ar = AxleForces {
            fx: f_rl.fx + f_rr.fx,
            fy: f_rl.fy + f_rr.fy,
            utilisation: f_rl.utilisation.max(f_rr.utilisation),
            inner_fz: inner_r,
            outer_fz: outer_r,
            align_nm: 0.0,
            trail_m: 0.0,
            ..AxleForces::default()
        };
        // Front lateral peak relative to the rear (`front_grip_factor`). The
        // fitted curve is linear in mu at a given slip, so scaling the force is
        // exactly a mu scaling; the aligning moment is Fy through the trail and
        // scales with it, utilisation does not. Same operation, same order as
        // the JS build, so the two stay bit-identical.
        af.fy *= self.c.params.front_grip_factor;
        af.align_nm *= self.c.params.front_grip_factor;
        let (fx_f, fy_f, util_f, fzi_f, fzo_f) = (af.fx, af.fy, af.utilisation, af.inner_fz, af.outer_fz);
        // The rear no longer has a single utilisation worth reporting: the
        // two wheels can be doing quite different things. Per-wheel values go
        // to the telemetry array and the load-weighted axle figure to
        // `balance`, which is the one that describes the car.
        let (fx_r, fy_r, fzi_r, fzo_r) = (ar.fx, ar.fy, ar.inner_fz, ar.outer_fz);

        // Resolve the front through the steer angle.
        let fx_fb = fx_f * cd - fy_f * sd;
        let fy_fb = fx_f * sd + fy_f * cd;

        let roll_res = self.c.params.crr * (fz_f + fz_r) * if u >= 0.0 { 1.0 } else { -1.0 };

        // The yaw moment the driven wheels make across the track. This is the
        // whole point of modelling the differential: a force at lateral offset
        // y contributes -y*Fx, and left is positive y, so the outer wheel
        // pushing harder than the inner turns the car into the corner and the
        // inner pushing harder pushes the nose wide. Under power a Salisbury
        // LSD sends torque to the SLOWER, inner wheel, which is why a locked
        // car understeers on throttle; on a lift it drags the faster, outer
        // wheel, which is what steadies the rear instead of letting it come
        // round. With one rear rotor both of those are exactly zero.
        let n_diff = half_track_r * (f_rr.fx - f_rl.fx);
        // The same across the FRONT track, once the fronts are two wheels:
        // braking in a corner the loaded outside front drags harder, which
        // yaws the car out of the turn. With one front rotor the solver has
        // always left this out, so it stays out there (bit-identical).
        let n_front = if split { half_track_f * (af.fx_right - af.fx_left) * cd } else { 0.0 };

        // Drag acts against the velocity, not against the nose. Taken off u
        // alone, a car sliding sideways at 20 m/s lost 1.1 m/s^2 of FORWARD
        // speed and none sideways, and one travelling backwards in a spin
        // was pushed further backwards by it.
        let (drag_x, drag_y) = if speed > 1e-9 { (drag * u / speed, drag * v / speed) } else { (0.0, 0.0) };
        let du = (fx_fb + fx_r - drag_x - roll_res) / m + v * r;
        let dv = (fy_fb + fy_r - drag_y) / m - u * r;
        let dr = (p_a * fy_fb - p_b * fy_r + n_diff + n_front) / self.c.params.izz_kg_m2;

        // Driveline. The carrier turns at the mean of the two side gears, so
        // that is the speed the gearbox sees.
        let w_r_mean = 0.5 * (self.w_rl + self.w_rr);
        let drive = self.c.powertrain.step(dt, controls.throttle, w_r_mean, speed);

        // Wheel dynamics; brake torque must not drive a wheel backwards.
        let brake_total = controls.brake.clamp(0.0, 1.0) * self.c.params.brakes.max_torque_nm;
        let tb_f = brake_total * self.c.params.brakes.bias_front;
        let tb_r = brake_total * (1.0 - self.c.params.brakes.bias_front);
        let iw_f = 2.0 * self.c.params.wheel_inertia_front_kg_m2;
        let iw_r_side = self.c.params.wheel_inertia_rear_kg_m2;

        // ---- the differential -------------------------------------------
        // Salisbury clutch pack: `t_cap` is the largest torque DIFFERENCE the
        // ramps and the preload can hold across the two outputs, and the
        // transfer is half of it. See `DiffParams` for where C and B come
        // from. A 1.5-way has a shallower drive ramp than coast ramp, so it
        // locks harder under power than on the overrun.
        let dfp = &self.c.params.diff;
        let t_in = drive.wheel_torque_nm;
        let lock_frac = if t_in >= 0.0 { dfp.power_lock } else { dfp.coast_lock };
        let t_cap = lock_frac * t_in.abs() + dfp.preload_nm;
        // Coulomb friction with a soft sign, as everywhere else in this code
        // base: the clutch opposes the speed difference, saturating at half
        // the capacity, and inside the stick band it behaves as a spring
        // rather than switching between two branches. The spring is integrated
        // with a limit -- below, once the axle's inertia is known -- so it
        // cannot overshoot.
        let d_w_rear = self.w_rr - self.w_rl;
        let tb_r_side = 0.5 * tb_r;

        // Implicit in the tyre's longitudinal stiffness; see bicycle.js for
        // why (explicit Euler is unstable under ~4.5 m/s at 500 Hz). Same
        // finite difference, same order of operations, so the two stay
        // bit-identical.
        let dfx_f = ((self.axle_forces(self.a_f, k_f + KAPPA_H, k_f + KAPPA_H, fz_f, d_fz_f).fx - fx_f) / KAPPA_H).max(0.0);
        let dfx_rl = ((self.c.tyre.forces(Slip { alpha: self.a_r, kappa: k_rl + KAPPA_H }, fz_rl).fx - f_rl.fx) / KAPPA_H).max(0.0);
        let dfx_rr = ((self.c.tyre.forces(Slip { alpha: self.a_r, kappa: k_rr + KAPPA_H }, fz_rr).fx - f_rr.fx) / KAPPA_H).max(0.0);
        let stiff_f = dt * radius * radius * dfx_f / k_den_f;
        let stiff_rl = dt * radius * radius * dfx_rl / k_den;
        let stiff_rr = dt * radius * radius * dfx_rr / k_den;
        let t_free_f = -fx_f * radius;
        let tb_f_now = brake_torque(self.w_f, t_free_f, iw_f + stiff_f, tb_f, dt);
        let dw_f = (t_free_f - tb_f_now) / (iw_f + stiff_f);

        // Split front axle: each front wheel with its own inertia, its own
        // tyre Fx and HALF the front brake torque -- equal line pressure, so
        // equal torque, and the unloaded inside tyre reaches its limit first.
        // Same implicit stiffness term as the rear wheels.
        let (mut dw_fl, mut dw_fr, tb_f_side) = (0.0, 0.0, 0.5 * tb_f);
        if split {
            let iw = self.c.params.wheel_inertia_front_kg_m2;
            let s_of = |k: f64, fz: f64, fx: f64| {
                let d = ((self.c.tyre.forces(Slip { alpha: self.a_f, kappa: k + KAPPA_H }, fz).fx - fx) / KAPPA_H).max(0.0);
                dt * radius * radius * d / k_den_f
            };
            let stiff_fl = s_of(k_fl, af.fz_left, af.fx_left);
            let stiff_fr = s_of(k_fr, af.fz_right, af.fx_right);
            let (tfl, tfr) = (-af.fx_left * radius, -af.fx_right * radius);
            dw_fl = (tfl - brake_torque(self.w_fl, tfl, iw + stiff_fl, tb_f_side, dt)) / (iw + stiff_fl);
            dw_fr = (tfr - brake_torque(self.w_fr, tfr, iw + stiff_fr, tb_f_side, dt)) / (iw + stiff_fr);
        }

        // The two rear wheels, solved together. The driveline's reflected
        // inertia hangs on the CARRIER, which turns at the mean of the two
        // side gears, so it resists the wheels speeding up together and does
        // nothing at all to resist one speeding up while the other slows.
        // Hanging half of it on each wheel -- the obvious shortcut -- would
        // make the axle behave far more locked than the clutch pack actually
        // makes it, which is precisely the effect being modelled here.
        //
        //   (i_L + q) dw_L +       q dw_R = T_in/2 + t_lock - A_L
        //         q dw_L + (i_R + q) dw_R = T_in/2 - t_lock - A_R
        //
        // with q = I_driveline / 4 and A the tyre and brake torques.
        let q = 0.25 * drive.added_wheel_inertia;
        let i_l = iw_r_side + stiff_rl;
        let i_r = iw_r_side + stiff_rr;
        let det = (i_l * i_r + q * (i_l + i_r)).max(1e-9);

        // ---- the clutch pack's torque, limited so it cannot overshoot ----
        //
        // Feeding `t_lock` into the pair above, the ANTISYMMETRIC mode obeys
        //
        //   d(w_rr - w_rl)/dt = -t_lock * (i_l + i_r + 4q) / det = -t_lock / J
        //
        // so `anti_j` is the inertia the clutch actually works against -- a
        // fraction of a kg m^2. An explicit spring of gain
        // `0.5 t_cap / stick_rad_s` on that is unstable whenever
        // `dt * 0.5 t_cap / (stick_rad_s * J) > 2`, which the shipped preload
        // alone exceeds by an order of magnitude: once a corner had set the
        // rear wheels apart they sat in a permanent period-2 oscillation on
        // straight road, flipping sign every substep for the rest of the run
        // (measured: 2923 flips in six seconds, +-0.267 rad/s, never decaying)
        // and aliasing into the 100 Hz log as unexplainable noise on
        // `sim.kappa_r*` and `imu.yaw_rate`.
        //
        // The limit is what a stick constraint does: never apply more torque
        // than would bring the relative speed to zero in this step. Inside the
        // band that makes the spring a proper stick and is unconditionally
        // stable at any dt; outside it, where `tanh` has saturated and the
        // pack is genuinely slipping, the limit is far larger than
        // `0.5 t_cap` and nothing changes. Same order of operations as
        // bicycle.js so the two ports stay comparable.
        let anti_j = det / (i_l + i_r + 4.0 * q).max(1e-9);
        let t_spring = 0.5 * t_cap * (d_w_rear / dfp.stick_rad_s.max(1e-4)).tanh();
        let t_stop = anti_j * d_w_rear.abs() / dt;
        let t_lock = t_spring.signum() * t_spring.abs().min(t_stop);
        self.t_lock_last = t_lock;
        // Torque leaves the faster wheel and arrives at the slower one. These
        // are the torques the diff delivers BEFORE the driveline's own inertia
        // is taken out of them, which the coupled solve does.
        let t_rl = 0.5 * t_in + t_lock;
        let t_rr = 0.5 * t_in - t_lock;
        // Rear brakes as friction elements too (see `brake_torque`), through
        // the coupled pair: the torque each side needs to stop in this step
        // is read off the pair's equations with both wheels' target
        // accelerations set to -w/dt, then capped at that side's pedal torque.
        let p_l_free = t_rl - f_rl.fx * radius;
        let p_r_free = t_rr - f_rr.fx * radius;
        let (stop_l, stop_r) = (-self.w_rl / dt, -self.w_rr / dt);
        let tb_rl = brake_torque(0.0, p_l_free - (i_l + q) * stop_l - q * stop_r, 1.0, tb_r_side, dt);
        let tb_rr = brake_torque(0.0, p_r_free - q * stop_l - (i_r + q) * stop_r, 1.0, tb_r_side, dt);
        let p_l = p_l_free - tb_rl;
        let p_r = p_r_free - tb_rr;
        let dw_rl = (p_l * (i_r + q) - q * p_r) / det;
        let dw_rr = (p_r * (i_l + q) - q * p_l) / det;

        self.s.u += du * dt;
        self.s.v += dv * dt;
        self.s.r += dr * dt;
        // Fronts may roll backwards (see bicycle.js); the rear stays
        // non-negative for the driveline behind it.
        if split {
            self.w_fl += dw_fl * dt;
            self.w_fr += dw_fr * dt;
            // The axle figure the rest of the model reads is the mean.
            self.w_f = 0.5 * (self.w_fl + self.w_fr);
        } else {
            self.w_f += dw_f * dt;
            // Kept in step so switching the split on mid-run starts clean.
            self.w_fl = self.w_f;
            self.w_fr = self.w_f;
        }
        self.w_rl = (self.w_rl + dw_rl * dt).max(0.0);
        self.w_rr = (self.w_rr + dw_rr * dt).max(0.0);

        // The measured accelerations that feed the next substep's transfer.
        self.ax = du - v * r;
        self.ay = dv + u * r;

        // Come to a genuine stop rather than creeping on numerical noise.
        if self.s.u.abs() < 0.25 && controls.throttle < 0.05 && self.s.speed() < 0.4 {
            self.s.u = 0.0;
            self.s.v = 0.0;
            self.s.r = 0.0;
            self.w_f = 0.0;
            self.w_fl = 0.0;
            self.w_fr = 0.0;
            self.w_rl = 0.0;
            self.w_rr = 0.0;
            self.ax = 0.0;
            self.ay = 0.0;
            // The lagged slip angles too: relaxation is speed-proportional,
            // so at rest they would hold the last corner's slip forever and
            // leave a static aligning torque on the steering wheel.
            self.a_f = 0.0;
            self.a_r = 0.0;
        }
        // u may go negative: a spinning car travels backwards for a moment,
        // and pinning u at zero with v left alone made it orbit (see
        // bicycle.js). Slip angles use |u|, so the tyre model holds.

        self.s.x += (self.s.u * self.s.psi.cos() - self.s.v * self.s.psi.sin()) * dt;
        self.s.y += (self.s.u * self.s.psi.sin() + self.s.v * self.s.psi.cos()) * dt;
        self.s.psi += self.s.r * dt;

        let util_r_axle = if fz_r > 1.0 {
            (f_rl.utilisation * fz_rl + f_rr.utilisation * fz_rr) / fz_r
        } else {
            0.0
        };
        let pt = self.c.powertrain.telemetry();
        self.tel = Telemetry {
            speed: self.s.speed(),
            ax_g: self.ax / G,
            ay_g: self.ay / G,
            body_slip_deg: self.s.v.atan2(self.s.u.abs().max(0.1)).to_degrees(),
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
            kappa: [k_fl, k_fr, k_rl, k_rr],
            utilisation: if split {
                [af.util_left, af.util_right, f_rl.utilisation, f_rr.utilisation]
            } else {
                [util_f, util_f, f_rl.utilisation, f_rr.utilisation]
            },
            // Load-weighted across the rear, NOT the worse of the two
            // wheels. With a differential the lightly loaded inner wheel is
            // allowed to spin in a tight corner -- that is the diff doing its
            // job -- and reading that one wheel as "the rear axle is out of
            // grip" is simply wrong: it is carrying almost no load and almost
            // none of the axle's lateral force.
            balance: util_r_axle - util_f,
            downforce_n: downforce,
            drag_n: drag,
            engine_rpm: pt.engine_rpm,
            gear: pt.gear,
            shifting: pt.shifting,
            wheel_omega_front: self.w_f,
            wheel_omega_rear: 0.5 * (self.w_rl + self.w_rr),
            drive_force_n: fx_r,
            locked: drive.locked,
            diff_nm: self.t_lock_last,
            kingpin_torque_nm: af.align_nm,
            rim_torque_nm: af.align_nm * self.c.params.rim_torque_ratio(),
            trail_front_m: af.trail_m,
            mech_trail_m: self.c.params.mechanical_trail(),
            scrub_moment_nm: af.scrub_nm,
            ..Default::default()
        };
    }
}
