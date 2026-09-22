//! Solvers -- three fidelity levels behind one interface.
//!
//! The ladder exists so a study can pick the cheapest model that still answers
//! the question, and so the same vehicle, tyre and powertrain can be run
//! through all three to see what the extra fidelity is actually buying.

use crate::powertrain::PowertrainModel;
use crate::tyre::TyreModel;
use crate::vehicle::VehicleParams;

pub mod bicycle;
pub mod double_track;
pub mod point_mass;

pub use bicycle::BicycleSolver;
pub use double_track::DoubleTrackSolver;
pub use point_mass::PointMassSolver;

/// Fixed physics substep. Every solver integrates at this rate regardless of
/// how often `step` is called, so results never depend on frame rate.
pub const SUBSTEP: f64 = 1.0 / 500.0;

/// Speed above which the steering slip cap is at its full (narrow) width.
const SLIP_CAP_FULL_MPS: f64 = 8.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fidelity {
    /// Point mass: speed and heading only, grip-limited. No yaw dynamics.
    PointMass,
    /// Transient bicycle: 3 chassis DOF, one contact patch per axle, but grip
    /// still responds to left/right load transfer.
    Bicycle,
    /// Double track: four contact patches with individual loads and slips.
    DoubleTrack,
}

impl Fidelity {
    pub fn level(self) -> u8 {
        match self {
            Fidelity::PointMass => 1,
            Fidelity::Bicycle => 2,
            Fidelity::DoubleTrack => 3,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Controls {
    /// -1..1, positive is left.
    pub steer: f64,
    pub throttle: f64,
    pub brake: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ChassisState {
    /// Body-frame longitudinal velocity (m/s).
    pub u: f64,
    /// Body-frame lateral velocity (m/s), positive left.
    pub v: f64,
    /// Yaw rate (rad/s).
    pub r: f64,
    pub x: f64,
    pub y: f64,
    pub psi: f64,
}

impl ChassisState {
    pub fn speed(&self) -> f64 {
        self.u.hypot(self.v)
    }
}

/// Corner identifiers, front-left through rear-right.
pub const FL: usize = 0;
pub const FR: usize = 1;
pub const RL: usize = 2;
pub const RR: usize = 3;

#[derive(Debug, Clone, Copy, Default)]
pub struct Telemetry {
    pub speed: f64,
    pub ax_g: f64,
    pub ay_g: f64,
    pub body_slip_deg: f64,
    pub yaw_rate_deg_s: f64,
    pub steer_rad: f64,
    /// Per-corner vertical load (N). The bicycle solver fills all four by
    /// splitting each axle, so downstream code never has to branch on solver.
    pub fz: [f64; 4],
    pub slip_deg: [f64; 4],
    pub kappa: [f64; 4],
    pub utilisation: [f64; 4],
    /// Rear utilisation minus front: positive is rear-limited (oversteer).
    pub balance: f64,
    pub downforce_n: f64,
    pub drag_n: f64,
    pub engine_rpm: f64,
    pub gear: usize,
    pub shifting: bool,
    pub wheel_omega_front: f64,
    pub wheel_omega_rear: f64,
    /// Longitudinal force at the driven axle in the body frame (N).
    pub drive_force_n: f64,
    /// Clutch locked this substep.
    pub locked: bool,
    /// Torque the differential moved from the faster rear wheel to the
    /// slower one this substep (N.m, signed toward the left wheel). Zero for
    /// an open axle and for solvers without a differential.
    pub diff_nm: f64,
    /// Steering feel. The moment both front tyres put on the steering axis,
    /// and what reaches the driver's hands. Left-positive like `steer_rad`:
    /// positive tries to steer further left.
    pub kingpin_torque_nm: f64,
    pub rim_torque_nm: f64,
    /// Load-weighted front pneumatic trail and the mechanical trail (m).
    pub trail_front_m: f64,
    pub mech_trail_m: f64,
    /// The moment the front tyres' LONGITUDINAL forces put on the steering
    /// axis through the scrub radius, both wheels summed (N.m, kingpin level,
    /// left-positive like `kingpin_torque_nm`): scrub x (Fx_right - Fx_left).
    /// With one front rotor brake force splits by load, so braking in a corner
    /// the loaded outside front pulls harder and the pair is steered toward
    /// centre, adding weight to the rim. With `split_front_wheels` each wheel
    /// carries equal brake torque, the two forces nearly cancel, and the term
    /// only grows once the inside front locks. NOT part of `kingpin_torque_nm` and never fed
    /// back into the motion: the rig adds it to the rim torque when the
    /// profile selects the v2 steering-torque model. Zero for solvers that do
    /// not compute steering torque.
    pub scrub_moment_nm: f64,
    /// Body attitude (deg): roll positive right side down (a left turn),
    /// pitch positive nose down (braking). Zero from solvers with no
    /// suspension states.
    pub roll_deg: f64,
    pub pitch_deg: f64,
    /// Each wheel's inclination to the road (deg), positive with the top
    /// leaning left. Zero from solvers without camber.
    pub camber_deg: [f64; 4],
}

/// `Send + Sync` so a solver can live in an ECS resource or be shared across
/// threads. The tyre and powertrain behind it are already both.
pub trait Solver: Send + Sync {
    fn name(&self) -> &'static str;
    fn fidelity(&self) -> Fidelity;

    /// Advance by `dt` seconds using fixed internal substeps.
    fn step(&mut self, dt: f64, controls: Controls);

    fn state(&self) -> ChassisState;
    /// Direct access to the pose and velocities, for an external constraint
    /// (a barrier) that has to move the car. Not for driving it.
    fn state_mut(&mut self) -> &mut ChassisState;
    fn telemetry(&self) -> Telemetry;

    /// Place the car. If `speed` is non-zero the powertrain is synced to it.
    fn reset(&mut self, x: f64, y: f64, psi: f64, speed: f64);

    fn params(&self) -> &VehicleParams;
    fn params_mut(&mut self) -> &mut VehicleParams;
    fn powertrain_mut(&mut self) -> &mut dyn PowertrainModel;
    fn tyre(&self) -> &dyn TyreModel;
    fn tyre_mut(&mut self) -> &mut dyn TyreModel;
}

/// Everything a solver is assembled from. Swapping the tyre or the powertrain
/// is constructing this differently, not touching the solver.
pub struct Chassis {
    pub params: VehicleParams,
    pub tyre: Box<dyn TyreModel>,
    pub powertrain: Box<dyn PowertrainModel>,
}

impl Chassis {
    pub fn new(
        params: VehicleParams,
        tyre: Box<dyn TyreModel>,
        powertrain: Box<dyn PowertrainModel>,
    ) -> Self {
        Self { params, tyre, powertrain }
    }
}

/// Build a solver of the requested fidelity from the same parts.
pub fn build(fidelity: Fidelity, chassis: Chassis) -> Box<dyn Solver> {
    match fidelity {
        Fidelity::PointMass => Box::new(PointMassSolver::new(chassis)),
        Fidelity::Bicycle => Box::new(BicycleSolver::new(chassis)),
        Fidelity::DoubleTrack => Box::new(DoubleTrackSolver::new(chassis)),
    }
}

/// Torque a friction brake applies to a wheel this substep (N.m, positive
/// opposes positive wheel speed).
///
/// A brake is a friction element, not a torque source. Applying `sign(w) * tb`
/// is right while the wheel turns and wrong at rest: Rust's `0.0.signum()` is
/// +1, so a stopped, braked wheel was driven BACKWARDS by the full pedal
/// torque every substep. With the car held on the brakes and any throttle
/// (which keeps the stop-snap from firing) the fronts turned at -0.8 rad/s
/// and the car crept backwards off the line at 2.5 cm/s -- and JS, whose
/// `Math.sign(0)` is 0, dropped the brake entirely at rest and crept forward.
///
/// So the brake supplies what it takes to bring the wheel to rest in this
/// step -- `t_free` is every other torque on it, `inertia` the effective
/// inertia the step is taken against -- and never more than the pedal's
/// torque. A turning wheel sees exactly `sign(w) * tb` as before; a stopped
/// one is held; nothing overshoots through zero.
pub(crate) fn brake_torque(w: f64, t_free: f64, inertia: f64, tb: f64, dt: f64) -> f64 {
    if tb <= 0.0 {
        return 0.0;
    }
    (t_free + inertia * w / dt).clamp(-tb, tb)
}

/// Steering actuator shared by every solver: a rate- and acceleration-limited
/// servo behind a first-order lag, standing in for the driver's hands and rack
/// compliance. `rate` is the servo's own state (rad/s) and is updated in
/// place. With an effectively infinite acceleration limit this is exactly the
/// old rate-limit-then-lag, so the validated numbers do not move.
pub(crate) fn advance_steer(
    current: f64,
    rate: &mut f64,
    demand: f64,
    p: &VehicleParams,
    dt: f64,
) -> f64 {
    advance_steer_capped(current, rate, demand, p, dt, None, 0.0)
}

/// `advance_steer` with the slip cap: `kin` is the front's kinematic slip
/// term atan2(v + a r, u) (rad, left positive) when the caller can supply it.
/// With a cap the target is held inside kin +- cap, so a key or a stick
/// cannot command the front past the tyre's peak -- at 25 m/s the car's
/// peak lateral comes at under 5 deg of steer, and a 13 deg "lock" threw the
/// front three times past it and hooked the car round.
pub(crate) fn advance_steer_capped(
    current: f64,
    rate: &mut f64,
    demand: f64,
    p: &VehicleParams,
    dt: f64,
    kin: Option<f64>,
    u: f64,
) -> f64 {
    let st = &p.steering;
    let mut target = demand.clamp(-1.0, 1.0) * st.max_steer_rad;
    // Speed-sensitive rate: nobody flicks a wheel at 90 km/h the way they do
    // at 30, and a key press is the same step at both.
    let rate_scale = if st.rate_speed_ref_mps > 0.0 {
        (st.rate_speed_ref_mps / u.abs().max(0.1)).powf(st.rate_speed_exp).min(1.0)
    } else {
        1.0
    };
    if let Some(k) = kin {
        if st.slip_cap_rad > 0.0 {
            // The band is centred on the car's motion, not on zero, so when
            // the rear steps out the front follows the slide: the assist
            // counter-steers for a driver who has no seat to feel it in.
            // Opens up below SLIP_CAP_FULL_MPS (see bicycle.js): at walking
            // pace a band centred on the velocity commanded steer by itself.
            let cap = st.slip_cap_rad + (SLIP_CAP_FULL_MPS - u.abs()).max(0.0) * 10f64.to_radians();
            target = target.clamp(k - cap, k + cap);
            // The band follows the car's velocity, which in a spin is 70 to
            // 90 degrees off the nose; the rack still stops at lock.
            target = target.clamp(-st.max_steer_rad, st.max_steer_rad);
        }
    }
    let want = ((target - current) / st.lag_s.max(1e-4))
        .clamp(-st.rate_rad_s * rate_scale, st.rate_rad_s * rate_scale);
    let max_delta = st.accel_rad_s2 * rate_scale * dt;
    *rate += (want - *rate).clamp(-max_delta, max_delta);
    let mut next = current + *rate * dt;
    // Do not coast past the target: overshoot here is a discretisation
    // artefact, not modelled inertia.
    let err = target - current;
    if (err > 0.0 && next > target) || (err < 0.0 && next < target) {
        next = target;
        *rate = 0.0;
    }
    next
}
