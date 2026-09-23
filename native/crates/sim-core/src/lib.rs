//! Vehicle dynamics solver core for the SDM26 driving simulator.
//!
//! Dependency-free on purpose: this crate is the physics and nothing else, so
//! it compiles to WASM for a browser front end, links into Bevy, or sits
//! behind a Tauri command without dragging anything along, and its tests run
//! in milliseconds.
//!
//! Three things are swappable independently:
//!
//! * **Tyre** -- [`tyre::TyreModel`]. Linear-with-ceiling for grip-budget work,
//!   fitted Magic Formula with combined slip for driving. The tyre owns its own
//!   load sensitivity, so solvers never have to reason about how mu varies.
//! * **Powertrain** -- [`powertrain::PowertrainModel`]. An idealised drive, the
//!   real geared engine on the CFD sweep with a slipping clutch, or an electric
//!   motor. An EV conversion is a parameter set, not a rewrite.
//! * **Solver** -- [`solver::Solver`], at three fidelity levels: grip-limited
//!   point mass, transient bicycle, four-corner double track.
//!
//! A vehicle is pure data ([`vehicle::VehicleParams`]), so a different car is a
//! different value rather than different code.
//!
//! ```
//! use sim_core::prelude::*;
//!
//! let mut car = build(
//!     Fidelity::Bicycle,
//!     Chassis::new(
//!         sdm26(),
//!         Box::new(MagicFormulaTyre::sdm26()),
//!         Box::new(GearedEngine::sdm26()),
//!     ),
//! );
//! car.reset(0.0, 0.0, 0.0, 0.0);
//! car.step(0.02, Controls { steer: 0.0, throttle: 1.0, brake: 0.0 });
//! assert!(car.state().speed() > 0.0);
//! ```

pub mod assists;
pub mod boundary;
pub mod etc_map;
pub mod modular;
pub mod physics_rev;
pub mod powertrain;
pub mod solver;
pub mod tyre;
pub mod vehicle;

pub mod prelude {
    pub use crate::assists::Assists;
    pub use crate::boundary::Boundary;
    pub use crate::etc_map::EtcMap;
    pub use crate::powertrain::{
        ElectricDrive, GearedEngine, IdealDrive, PowertrainModel, PowertrainTelemetry,
    };
    pub use crate::solver::{
        build, BicycleSolver, Chassis, ChassisState, Controls, DoubleTrackSolver, Fidelity,
        PointMassSolver, Solver, Telemetry, FL, FR, RL, RR,
    };
    pub use crate::tyre::{LinearTyre, MagicFormulaTyre, Slip, TyreForces, TyreModel};
    pub use crate::vehicle::{sdm25, sdm26, VehicleParams, G};
}

/// Convenience: the SDM26 as it is actually run -- bicycle solver, fitted Magic
/// Formula, geared engine on the CFD sweep.
pub fn sdm26_default() -> Box<dyn solver::Solver> {
    solver::build(
        solver::Fidelity::Bicycle,
        solver::Chassis::new(
            vehicle::sdm26(),
            Box::new(tyre::MagicFormulaTyre::sdm26()),
            Box::new(powertrain::GearedEngine::sdm26()),
        ),
    )
}
