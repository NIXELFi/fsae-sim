//! Solvers — three fidelity levels behind one interface.
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
}

/// `Send + Sync` so a solver can live in an ECS resource or be shared across
/// threads. The tyre and powertrain behind it are already both.
pub trait Solver: Send + Sync {
    fn name(&self) -> &'static str;
    fn fidelity(&self) -> Fidelity;

    /// Advance by `dt` seconds using fixed internal substeps.
    fn step(&mut self, dt: f64, controls: Controls);

    fn state(&self) -> ChassisState;
    fn telemetry(&self) -> Telemetry;

    /// Place the car. If `speed` is non-zero the powertrain is synced to it.
    fn reset(&mut self, x: f64, y: f64, psi: f64, speed: f64);

    fn params(&self) -> &VehicleParams;
    fn params_mut(&mut self) -> &mut VehicleParams;
    fn powertrain_mut(&mut self) -> &mut dyn PowertrainModel;
    fn tyre(&self) -> &dyn TyreModel;
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

/// Steering actuator shared by every solver: rate limit then first-order lag,
/// standing in for the driver's hands and rack compliance.
pub(crate) fn advance_steer(current: f64, demand: f64, p: &VehicleParams, dt: f64) -> f64 {
    let target = demand.clamp(-1.0, 1.0) * p.steering.max_steer_rad;
    let rate = ((target - current) / p.steering.lag_s)
        .clamp(-p.steering.rate_rad_s, p.steering.rate_rad_s);
    current + rate * dt
}
