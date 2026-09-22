//! Swappable subsystems, and vehicles described as data.
//!
//! `sim-core` already had the two interfaces that mattered most for the solver
//! ladder -- [`TyreModel`] and [`PowertrainModel`]. This adds the two that were
//! still baked into the solver, suspension and aero, and a [`VehicleDefinition`]
//! that names which implementation of each a given car uses.
//!
//! The shape mirrors `sim/src/vehicle/modules.js` and
//! `sim/src/vehicle/library.js` deliberately: the same definition, with
//! the same field names, should be readable by both builds. That is why the
//! module identifiers are strings rather than an enum -- an enum would be
//! tidier in Rust and would immediately stop matching a JSON file written by
//! the web build.
//!
//! No second vehicle is defined here, on purpose. The framework is the thing
//! being built; inventing a plausible-looking car would put numbers in the
//! repository that nobody has measured.

use crate::vehicle::{self, VehicleParams};

/// Load transfer and body attitude.
///
/// Splitting this out of the solver is what lets a car have, say, a live axle
/// or a monoshock without the bicycle model knowing. The current implementation
/// is the one the solver has always used, lifted rather than rewritten.
pub trait SuspensionModel: Send + Sync {
    /// Lateral load transfer at each axle, N. Positive loads the outside tyre.
    fn lateral_transfer(&self, v: &VehicleParams, ay: f64) -> AxleTransfer;

    /// Longitudinal transfer front to rear, N. Positive loads the rear.
    fn longitudinal_transfer(&self, v: &VehicleParams, ax: f64) -> f64;

    /// Body attitude, degrees, given the car's roll and pitch gradients.
    ///
    /// The gradients are arguments rather than fields on `VehicleParams`
    /// because they are a driving-sim concern -- they exist to move a camera,
    /// and a lap sim has no use for them. Putting them in the shared parameter
    /// struct would push a rendering detail into the solver's data model.
    fn attitude(&self, v: &VehicleParams, ay_g: f64, ax_g: f64,
                roll_grad_deg_g: f64, pitch_grad_deg_g: f64) -> Attitude;

    fn name(&self) -> &'static str;
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AxleTransfer {
    pub front_n: f64,
    pub rear_n: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Attitude {
    pub roll_deg: f64,
    pub pitch_deg: f64,
}

/// Drag and downforce.
pub trait AeroModel: Send + Sync {
    /// Forces at a given speed, N. Downforce is positive downward.
    fn forces(&self, v: &VehicleParams, speed_ms: f64) -> AeroForces;
    fn name(&self) -> &'static str;
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AeroForces {
    pub drag_n: f64,
    pub downforce_front_n: f64,
    pub downforce_rear_n: f64,
}

impl AeroForces {
    pub fn total_downforce_n(&self) -> f64 {
        self.downforce_front_n + self.downforce_rear_n
    }
}

// ---------------------------------------------------------------------------
// Shipped implementations
// ---------------------------------------------------------------------------

/// Elastic plus geometric transfer, which is what the solver does today.
///
/// The elastic part comes from roll-stiffness distribution acting on the sprung
/// mass through the roll-couple arm; the geometric part acts instantly through
/// the roll centres; the unsprung mass transfers through its own centre of
/// gravity. Keeping the three separate is not pedantry -- they have different
/// time constants, and lumping them is what makes a simple model feel numb.
#[derive(Clone, Copy, Debug, Default)]
pub struct ElasticGeometricSuspension;

impl SuspensionModel for ElasticGeometricSuspension {
    fn lateral_transfer(&self, v: &VehicleParams, ay: f64) -> AxleTransfer {
        let m_sprung = v.sprung_mass();
        let front_share = v.roll.rsd_front;

        // Elastic: the sprung mass rolling about the roll axis.
        let elastic = m_sprung * ay * v.roll_arm();
        let elastic_f = elastic * front_share / v.track_front_m.max(1e-6);
        let elastic_r = elastic * (1.0 - front_share) / v.track_rear_m.max(1e-6);

        // Geometric: reacted straight through the roll centres, no roll needed.
        let w_f = m_sprung * v.weight_dist_front;
        let w_r = m_sprung * (1.0 - v.weight_dist_front);
        let geom_f = w_f * ay * v.roll.rc_front_m / v.track_front_m.max(1e-6);
        let geom_r = w_r * ay * v.roll.rc_rear_m / v.track_rear_m.max(1e-6);

        // Unsprung, through its own centre of gravity (about wheel centre).
        let uns_f = 2.0 * v.unsprung_front_kg * ay * v.tyre_radius_m / v.track_front_m.max(1e-6);
        let uns_r = 2.0 * v.unsprung_rear_kg * ay * v.tyre_radius_m / v.track_rear_m.max(1e-6);

        AxleTransfer {
            front_n: elastic_f + geom_f + uns_f,
            rear_n: elastic_r + geom_r + uns_r,
        }
    }

    fn longitudinal_transfer(&self, v: &VehicleParams, ax: f64) -> f64 {
        v.mass_kg * ax * v.cg_height_m / v.wheelbase_m.max(1e-6)
    }

    fn attitude(&self, _v: &VehicleParams, ay_g: f64, ax_g: f64,
                roll_grad_deg_g: f64, pitch_grad_deg_g: f64) -> Attitude {
        Attitude {
            roll_deg: ay_g * roll_grad_deg_g,
            pitch_deg: ax_g * pitch_grad_deg_g,
        }
    }

    fn name(&self) -> &'static str {
        "elastic-plus-geometric"
    }
}

/// A rigid car: no load transfer at all.
///
/// Not a toy. It is the reference that isolates how much of a handling result
/// comes from load transfer rather than from the tyre, and it is the correct
/// model for checking a solver against a textbook derivation that assumes a
/// rigid chassis.
#[derive(Clone, Copy, Debug, Default)]
pub struct RigidSuspension;

impl SuspensionModel for RigidSuspension {
    fn lateral_transfer(&self, _v: &VehicleParams, _ay: f64) -> AxleTransfer {
        AxleTransfer::default()
    }

    fn longitudinal_transfer(&self, _v: &VehicleParams, _ax: f64) -> f64 {
        0.0
    }

    fn attitude(&self, _v: &VehicleParams, _ay_g: f64, _ax_g: f64,
                _roll_grad_deg_g: f64, _pitch_grad_deg_g: f64) -> Attitude {
        Attitude::default()
    }

    fn name(&self) -> &'static str {
        "rigid"
    }
}

/// Fixed coefficients with a constant front split, from a CFD map at nominal
/// ride height. Ignores ride-height and yaw sensitivity, both of which are real
/// and both of which the CFD module has data for.
#[derive(Clone, Copy, Debug)]
pub struct FixedCoefficientAero;

impl AeroModel for FixedCoefficientAero {
    fn forces(&self, v: &VehicleParams, speed_ms: f64) -> AeroForces {
        let q = 0.5 * v.aero.air_density * speed_ms * speed_ms;
        let down = q * v.aero.cla_m2;
        AeroForces {
            drag_n: q * v.aero.cda_m2,
            downforce_front_n: down * v.aero.front_frac,
            downforce_rear_n: down * (1.0 - v.aero.front_frac),
        }
    }

    fn name(&self) -> &'static str {
        "fixed-coefficient"
    }
}

/// No aero. The right model for a car that has none, and the fastest way to
/// find out how much of a lap time is downforce.
#[derive(Clone, Copy, Debug)]
pub struct NoAero;

impl AeroModel for NoAero {
    fn forces(&self, _v: &VehicleParams, _speed_ms: f64) -> AeroForces {
        AeroForces::default()
    }

    fn name(&self) -> &'static str {
        "none"
    }
}

// ---------------------------------------------------------------------------
// Definitions
// ---------------------------------------------------------------------------

/// Which implementation a vehicle uses for each subsystem.
///
/// Strings rather than an enum so this matches the JSON the web build writes.
/// An enum would be tidier here and would stop matching the moment the two
/// sides were built at different times.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ModuleSelection {
    pub tyre: String,
    pub powertrain: String,
    pub suspension: String,
    pub aero: String,
    pub engine_audio: String,
}

impl Default for ModuleSelection {
    fn default() -> Self {
        Self {
            tyre: "magic-formula-fitted".into(),
            powertrain: "cfd-sweep-geared".into(),
            suspension: "elastic-plus-geometric".into(),
            aero: "fixed-coefficient".into(),
            engine_audio: "cbr600rr-sdm26".into(),
        }
    }
}

/// A complete vehicle: which models, and what numbers.
#[derive(Clone, Debug)]
pub struct VehicleDefinition {
    pub schema: u32,
    pub id: String,
    pub name: String,
    pub modules: ModuleSelection,
    pub params: VehicleParams,
}

impl VehicleDefinition {
    pub const SCHEMA: u32 = 1;

    pub fn sdm26() -> Self {
        Self {
            schema: Self::SCHEMA,
            id: "sdm26".into(),
            name: "SDM26".into(),
            modules: ModuleSelection::default(),
            params: vehicle::sdm26(),
        }
    }

    /// Copy under a new identity. The numbers start identical, so "duplicate,
    /// then change one thing" is an honest experiment rather than a fork.
    pub fn duplicate(&self, id: &str, name: &str) -> Self {
        Self {
            schema: self.schema,
            id: id.into(),
            name: name.into(),
            modules: self.modules.clone(),
            params: self.params.clone(),
        }
    }

    /// Every problem at once, rather than throwing on the first.
    ///
    /// A definition written by hand or exported by an older build usually has
    /// several things wrong, and fixing them one message at a time is
    /// miserable.
    pub fn validate(&self) -> Vec<String> {
        let mut out = vec![];
        if self.schema != Self::SCHEMA {
            out.push(format!("schema {} is not {}", self.schema, Self::SCHEMA));
        }
        if self.id.is_empty()
            || !self
                .id
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
        {
            out.push("id must be lowercase letters, digits and hyphens".into());
        }
        if self.name.is_empty() {
            out.push("needs a name".into());
        }
        let p = &self.params;
        for (label, value) in [
            ("mass_kg", p.mass_kg),
            ("wheelbase_m", p.wheelbase_m),
            ("track_front_m", p.track_front_m),
            ("track_rear_m", p.track_rear_m),
            ("tyre_radius_m", p.tyre_radius_m),
        ] {
            if !value.is_finite() || value <= 0.0 {
                out.push(format!("params.{label} must be a positive number"));
            }
        }
        if p.weight_dist_front <= 0.0 || p.weight_dist_front >= 1.0 {
            out.push("params.weight_dist_front must be between 0 and 1".into());
        }
        out
    }
}

/// Build a suspension model by name.
pub fn suspension_by_name(id: &str) -> Option<Box<dyn SuspensionModel>> {
    match id {
        "elastic-plus-geometric" => Some(Box::new(ElasticGeometricSuspension)),
        "rigid" => Some(Box::new(RigidSuspension)),
        _ => None,
    }
}

/// Build an aero model by name.
pub fn aero_by_name(id: &str) -> Option<Box<dyn AeroModel>> {
    match id {
        "fixed-coefficient" => Some(Box::new(FixedCoefficientAero)),
        "none" => Some(Box::new(NoAero)),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_reference_definition_validates() {
        assert!(VehicleDefinition::sdm26().validate().is_empty());
    }

    #[test]
    fn validation_reports_every_problem_at_once() {
        let mut d = VehicleDefinition::sdm26();
        d.id = "Not Valid".into();
        d.name = String::new();
        d.params.mass_kg = -1.0;
        let problems = d.validate();
        assert!(problems.len() >= 3, "only found {problems:?}");
    }

    #[test]
    fn duplicating_does_not_alias_the_original() {
        let a = VehicleDefinition::sdm26();
        let mut b = a.duplicate("mule", "SDM27 mule");
        b.params.mass_kg = 240.0;
        assert_eq!(a.params.mass_kg, vehicle::sdm26().mass_kg);
        assert_eq!(b.params.mass_kg, 240.0);
        assert_eq!(b.modules, a.modules);
    }

    #[test]
    fn modules_resolve_by_name_and_reject_unknown_ones() {
        assert!(suspension_by_name("elastic-plus-geometric").is_some());
        assert!(suspension_by_name("rigid").is_some());
        assert!(suspension_by_name("nonexistent").is_none());
        assert!(aero_by_name("fixed-coefficient").is_some());
        assert!(aero_by_name("none").is_some());
        assert!(aero_by_name("nonexistent").is_none());
    }

    #[test]
    fn load_transfer_is_signed_and_scales_with_acceleration() {
        let v = vehicle::sdm26();
        let s = ElasticGeometricSuspension;
        let one_g = s.lateral_transfer(&v, 9.81);
        let two_g = s.lateral_transfer(&v, 19.62);
        assert!(one_g.front_n > 0.0 && one_g.rear_n > 0.0);
        assert!((two_g.front_n / one_g.front_n - 2.0).abs() < 1e-9);
        // And it reverses with the acceleration.
        let left = s.lateral_transfer(&v, -9.81);
        assert!((left.front_n + one_g.front_n).abs() < 1e-9);
    }

    #[test]
    fn total_lateral_transfer_is_near_the_rigid_body_answer() {
        // Whatever the split between axles, the sum has to come out close to
        // m * ay * h / t -- that is a free-body result and does not care how the
        // suspension is modelled. This is the check that catches a term being
        // double counted.
        let v = vehicle::sdm26();
        let s = ElasticGeometricSuspension;
        let ay = 9.81 * 1.5;
        let got = s.lateral_transfer(&v, ay);
        let total = got.front_n + got.rear_n;
        // With the roll arm derived from the sprung CG and the roll axis, the
        // three paths sum to the free-body answer exactly on equal tracks and
        // to within the track difference otherwise.
        let track = 0.5 * (v.track_front_m + v.track_rear_m);
        let expected = v.mass_kg * ay * v.cg_height_m / track;
        let err = (total - expected).abs() / expected;
        assert!(err < 0.01, "total {total:.0} N against rigid-body {expected:.0} N");
    }

    #[test]
    fn a_rigid_car_transfers_nothing() {
        let v = vehicle::sdm26();
        let s = RigidSuspension;
        assert_eq!(s.lateral_transfer(&v, 20.0), AxleTransfer::default());
        assert_eq!(s.longitudinal_transfer(&v, 20.0), 0.0);
    }

    #[test]
    fn aero_goes_as_the_square_of_speed() {
        let v = vehicle::sdm26();
        let a = FixedCoefficientAero;
        let at10 = a.forces(&v, 10.0);
        let at20 = a.forces(&v, 20.0);
        assert!((at20.drag_n / at10.drag_n - 4.0).abs() < 1e-9);
        assert!((at20.total_downforce_n() / at10.total_downforce_n() - 4.0).abs() < 1e-9);
    }

    #[test]
    fn aero_split_sums_to_the_total() {
        let v = vehicle::sdm26();
        let f = FixedCoefficientAero.forces(&v, 25.0);
        let q = 0.5 * v.aero.air_density * 625.0;
        assert!((f.total_downforce_n() - q * v.aero.cla_m2).abs() < 1e-9);
        // And the front share matches the parameter.
        assert!((f.downforce_front_n / f.total_downforce_n() - v.aero.front_frac).abs() < 1e-9);
    }

    #[test]
    fn no_aero_is_actually_none() {
        let v = vehicle::sdm26();
        assert_eq!(NoAero.forces(&v, 30.0), AeroForces::default());
    }
}
