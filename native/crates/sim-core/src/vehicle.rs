//! Vehicle parameters.
//!
//! A "vehicle" here is nothing but data. Every solver, tyre and powertrain
//! reads from this struct, so a new car — SDM25, an EV concept, next year's
//! chassis — is a new `VehicleParams` value and not a line of new physics.

pub const G: f64 = 9.81;

#[derive(Debug, Clone, PartialEq)]
pub struct AeroParams {
    /// Drag area, Cd·A (m²).
    pub cda_m2: f64,
    /// Lift area, Cl·A (m²), positive downward.
    pub cla_m2: f64,
    /// Fraction of total downforce carried by the front axle.
    pub front_frac: f64,
    pub air_density: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RollParams {
    /// Front share of total roll stiffness. The single biggest balance knob.
    pub rsd_front: f64,
    /// Sprung-CG to roll-axis arm (m).
    pub roll_arm_m: f64,
    pub rc_front_m: f64,
    pub rc_rear_m: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BrakeParams {
    /// Total brake torque at the wheels at full pedal (N·m).
    pub max_torque_nm: f64,
    pub bias_front: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SteeringParams {
    /// Road-wheel angle at full lock (rad).
    pub max_steer_rad: f64,
    /// First-order lag standing in for hands and rack compliance (s).
    pub lag_s: f64,
    /// Rate limit at the road wheel (rad/s).
    pub rate_rad_s: f64,
    /// Acceleration limit at the road wheel (rad/s^2). Effectively infinite
    /// by default, which reduces the servo to rate limit + lag; a control
    /// profile lowers it so a step input (a key) cannot become a step in
    /// steering velocity.
    pub accel_rad_s2: f64,
    /// Front slip-angle cap (rad) for devices with no feel, or 0 for none:
    /// the road wheel is never commanded past the angle that puts the front
    /// tyre this far into slip against the car's actual motion. See the
    /// bicycle solver.
    pub slip_cap_rad: f64,
    /// Above this speed (m/s) the servo's rate and acceleration limits scale
    /// by (ref / u)^exp. 0 = off. Devices with no feel only.
    pub rate_speed_ref_mps: f64,
    pub rate_speed_exp: f64,
    /// Rim angle per road-wheel angle. With force feedback it is also the
    /// torque ratio the other way: rim torque = kingpin torque / ratio.
    pub ratio: f64,
    /// Caster angle (rad). With the tyre radius it sets the mechanical trail.
    pub caster_rad: f64,
    /// Extra mechanical trail from the kingpin axis sitting ahead of the hub
    /// centre (m).
    pub kingpin_offset_trail_m: f64,
    /// Fraction of the kingpin moment that reaches the rim.
    pub rack_efficiency: f64,
    /// Rim torque per kingpin torque, if the real rack says something other
    /// than 1/ratio.
    pub torque_ratio: Option<f64>,
    /// 0 = parallel steer, 1 = full Ackermann. Only the double-track solver
    /// can use this; a bicycle model has one front wheel by definition.
    pub ackermann: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct VehicleParams {
    pub name: String,
    pub mass_kg: f64,
    /// Static front weight distribution, 0..1.
    pub weight_dist_front: f64,
    pub cg_height_m: f64,
    pub wheelbase_m: f64,
    pub track_front_m: f64,
    pub track_rear_m: f64,
    pub tyre_radius_m: f64,
    pub izz_kg_m2: f64,
    /// Unsprung mass per corner, front and rear (kg). They differ: the front
    /// carries the steering upright, the rear the driveshaft and sprocket.
    pub unsprung_front_kg: f64,
    pub unsprung_rear_kg: f64,
    pub wheel_inertia_front_kg_m2: f64,
    pub wheel_inertia_rear_kg_m2: f64,
    pub crr: f64,
    /// Front axle peak lateral grip relative to the rear. Applied to the
    /// front tyres' lateral force by the solvers. With one tyre character for
    /// both axles the limit balance is set only by load transfer and the aero
    /// split, which leaves this car neutral to within 1% of force and spinning
    /// from any step steer at the limit; a real front lets go first (camber
    /// loss on the steered upright, inside-front drag, compliance). Estimate,
    /// pinned together with the tyre's mu_y to the 5.02 s skidpad.
    pub front_grip_factor: f64,
    pub aero: AeroParams,
    pub roll: RollParams,
    pub brakes: BrakeParams,
    pub steering: SteeringParams,
}

impl VehicleParams {
    /// CG to front axle (m).
    pub fn a(&self) -> f64 {
        self.wheelbase_m * (1.0 - self.weight_dist_front)
    }

    /// CG to rear axle (m).
    pub fn b(&self) -> f64 {
        self.wheelbase_m * self.weight_dist_front
    }

    /// Static load on one tyre (N) — the reference for load sensitivity.
    pub fn nominal_tyre_load(&self) -> f64 {
        self.mass_kg * G / 4.0
    }

    pub fn sprung_mass(&self) -> f64 {
        self.mass_kg - 2.0 * (self.unsprung_front_kg + self.unsprung_rear_kg)
    }

    pub fn weight(&self) -> f64 {
        self.mass_kg * G
    }

    /// Mechanical trail from caster and kingpin offset (m).
    pub fn mechanical_trail(&self) -> f64 {
        self.tyre_radius_m * self.steering.caster_rad.tan() + self.steering.kingpin_offset_trail_m
    }

    /// Rim torque per unit kingpin torque, losses included.
    pub fn rim_torque_ratio(&self) -> f64 {
        let r = self.steering.torque_ratio.unwrap_or(1.0 / self.steering.ratio.max(1e-6));
        r * self.steering.rack_efficiency
    }

    /// Downforce and drag at a given speed (N).
    pub fn aero_forces(&self, speed: f64) -> (f64, f64) {
        let q = 0.5 * self.aero.air_density * speed * speed;
        (q * self.aero.cla_m2, q * self.aero.cda_m2)
    }
}

/// SDM26 — Sun Devil Motorsports' 2026 car.
///
/// Provenance is the same as the JS build it is ported from: mass, geometry,
/// gearing, aero map and roll config are team data out of Helios; the
/// inertias, steering lock and brake numbers are engineering estimates.
pub fn sdm26() -> VehicleParams {
    VehicleParams {
        name: "SDM26".into(),
        mass_kg: 267.0,
        weight_dist_front: 0.485,
        cg_height_m: 0.2845,
        wheelbase_m: 1.53,
        track_front_m: 1.207,
        track_rear_m: 1.194,
        tyre_radius_m: 0.20,
        // Team workbook ('SDM26 Full-Vehicle Sim Parameters'): Izz, unsprung
        // per corner and wheel spin inertia. See params.js for the sources.
        izz_kg_m2: 93.66,
        unsprung_front_kg: 7.56,
        unsprung_rear_kg: 7.77,
        wheel_inertia_front_kg_m2: 0.154,
        wheel_inertia_rear_kg_m2: 0.152,
        crr: 0.02,
        front_grip_factor: 0.90,
        aero: AeroParams {
            cda_m2: 1.294,
            cla_m2: 3.146,
            // 50%, not the CFD map's 55.3%: see params.js `aeroFrontFrac`.
            // At 55.3% the rear limits first above ~20 m/s and the car spins
            // through a steady steer ramp; at 50% the front limits first at
            // every speed. Kept identical to the JS build for parity.
            front_frac: 0.50,
            air_density: 1.162,
        },
        roll: RollParams {
            rsd_front: 0.512,
            roll_arm_m: 0.2626,
            rc_front_m: 0.0186,
            rc_rear_m: 0.0251,
        },
        // 70 bar max working pressure through the measured callipers and
        // 54% bias bar; see params.js.
        brakes: BrakeParams { max_torque_nm: 1235.0, bias_front: 0.72 },
        steering: SteeringParams {
            max_steer_rad: 28.0_f64.to_radians(),
            lag_s: 0.06,
            rate_rad_s: 360.0_f64.to_radians(),
            accel_rad_s2: 1e9,
            slip_cap_rad: 0.0,
            rate_speed_ref_mps: 0.0,
            rate_speed_exp: 1.5,
            // OptimumK 'Designed vs Actual Kinematics' (2026-06-27): ratio
            // 4.411, caster 4.743 deg, mechanical trail 18.85 mm.
            ratio: 4.411,
            caster_rad: 4.743_f64.to_radians(),
            kingpin_offset_trail_m: 0.00225,
            rack_efficiency: 0.85,
            torque_ratio: None,
            ackermann: 0.0,
        },
    }
}

/// SDM25 — same chassis family, heavier, shorter final drive.
pub fn sdm25() -> VehicleParams {
    VehicleParams {
        name: "SDM25".into(),
        mass_kg: 281.0,
        roll: RollParams { rsd_front: 0.36, ..sdm26().roll },
        ..sdm26()
    }
}
