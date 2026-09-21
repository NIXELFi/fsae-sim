//! Vehicle parameters.
//!
//! A "vehicle" here is nothing but data. Every solver, tyre and powertrain
//! reads from this struct, so a new car -- SDM25, an EV concept, next year's
//! chassis -- is a new `VehicleParams` value and not a line of new physics.

pub const G: f64 = 9.81;

#[derive(Debug, Clone, PartialEq)]
pub struct AeroParams {
    /// Drag area, Cd.A (m^2).
    pub cda_m2: f64,
    /// Lift area, Cl.A (m^2), positive downward.
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
    /// Total brake torque at the wheels at full pedal (N.m).
    pub max_torque_nm: f64,
    pub bias_front: f64,
}

/// Salisbury (clutch-pack) limited-slip differential.
///
/// SDM26 runs a Drexler Formula Student V3, a 1.5-way Salisbury LSD. The team's
/// April 2026 study ("The Differential Drexler Study") reduces the wedge
/// mechanics to one line:
///
/// ```text
///   T_c = C * |T_in| + B          the most torque the clutch can hold across
///   C   = mu n_f r_eff cot(beta) (1+k) / (2 N_p r_ramp)
///   B   = mu n_f r_eff F_preload
/// ```
///
/// and then makes the point that matters for us: do NOT try to derive
/// `r_ramp` and `r_eff` from first principles, because Drexler does not publish
/// them. Back-calculate the aggregated `C` from the lock percentages in the
/// manual and treat it as one identified constant. Those percentages are
/// 30 deg -> 0.88, 40 -> 0.60, 45 -> 0.51, 50 -> 0.42, 60 -> 0.29, read as
/// `eta = T_c / T_in`, so `C` IS the lock fraction for the ramp that is fitted.
///
/// The study also warns that the manual's numbers are marketing-optimistic and
/// that measured on-track values run 60-80% of them. They are used here as
/// quoted, because the AC mod is pinned to the same table and the two should
/// agree; `power_lock` and `coast_lock` are exposed so that can be derated
/// against real wheel-force data when the team has it.
#[derive(Debug, Clone, PartialEq)]
pub struct DiffParams {
    /// Lock fraction on the drive ramp: `eta` at the fitted ramp angle.
    pub power_lock: f64,
    /// Lock fraction on the coast ramp. Lower than `power_lock` on a 1.5-way.
    pub coast_lock: f64,
    /// Breakaway preload, N.m measured wheel to wheel, as Drexler specifies it.
    pub preload_nm: f64,
    /// Width of the stick band, rad/s of wheel-speed difference. Inside it the
    /// clutch behaves as a spring rather than a Coulomb element, which is what
    /// keeps the split continuous at a 500 Hz substep.
    pub stick_rad_s: f64,
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
    /// Scrub radius at the contact patch (m). With the KPI it sets how far the
    /// car lifts as the wheel turns, which is the jacking torque the driver
    /// feels returning the rim at a standstill.
    pub scrub_m: f64,
    /// Kingpin inclination (rad).
    pub kpi_rad: f64,
    /// Fraction of the kingpin moment that reaches the rim.
    pub rack_efficiency: f64,
    /// Rim torque per kingpin torque, if the real rack says something other
    /// than 1/ratio.
    pub torque_ratio: Option<f64>,
    /// 0 = parallel steer, 1 = full Ackermann. Only the double-track solver
    /// can use this; a bicycle model has one front wheel by definition.
    pub ackermann: f64,
}

// ------------------------------------------------------------ steering map --

/// Rim-angle step of [`STEER_MAP_ROAD_DEG`] (deg).
pub const STEER_MAP_STEP_DEG: f64 = 5.0;

/// Rim angle at the rack's measured stop, one side (deg).
pub const STEER_RIM_LOCK_DEG: f64 = 179.0;

/// Measured steering-wheel angle -> road-wheel angle, 0..180 deg of rim in
/// 5 deg steps.
///
/// TEAM: Drive `Steer_Force_Calculator/wheel_toe_angles.csv` (2026-07-26),
/// 704 rows of both road wheels' toe against rim angle, averaged into the one
/// road-wheel angle a bicycle model carries. Static toe cancels in the mean,
/// so the table passes through zero.
///
/// The point of it is that a real rack is PROGRESSIVE. This one needs 5.27 deg
/// of rim per road degree on centre, falling to about 3.4 by 90 deg -- so the
/// nominal 4.411 constant is 19 % too quick where most of the driving happens
/// and then stops 18 deg of road wheel short of the real lock. Driving the
/// fixed ratio felt darty on centre and hit a wall in hairpins that the car
/// does not have.
pub const STEER_MAP_ROAD_DEG: [f64; 37] = [
    0.0000, 0.9486, 1.9018, 2.8643, 3.8407, 4.8355,
    5.8534, 6.8987, 7.9756, 9.0882, 10.2396, 11.4326,
    12.6689, 13.9491, 15.2722, 16.6359, 18.0360, 19.4671,
    20.9221, 22.3935, 23.8733, 25.3539, 26.8282, 28.2906,
    29.7370, 31.1647, 32.5728, 33.9618, 35.3334, 36.6907,
    38.0376, 39.3790, 40.7206, 42.0688, 43.4308, 44.8146,
    46.2288,
];

/// Road-wheel angle (deg, signed) for a rim angle, by linear interpolation.
pub fn road_from_rim_deg(rim_deg: f64) -> f64 {
    let n = STEER_MAP_ROAD_DEG.len();
    let sign = if rim_deg < 0.0 { -1.0 } else { 1.0 };
    let mag = rim_deg.abs();
    let x = mag / STEER_MAP_STEP_DEG;
    if x >= (n - 1) as f64 {
        // Past the table: hold the last slope, so over-travel is a ramp and
        // not a cliff. The end stop is what should be resisting by then.
        let last = STEER_MAP_ROAD_DEG[n - 1];
        let slope = (last - STEER_MAP_ROAD_DEG[n - 2]) / STEER_MAP_STEP_DEG;
        return sign * (last + slope * (mag - (n - 1) as f64 * STEER_MAP_STEP_DEG));
    }
    let i = x.floor() as usize;
    let t = x - i as f64;
    sign * (STEER_MAP_ROAD_DEG[i] + t * (STEER_MAP_ROAD_DEG[i + 1] - STEER_MAP_ROAD_DEG[i]))
}

/// Rim angle (deg, signed) that produces a given road-wheel angle -- the
/// inverse of [`road_from_rim_deg`], by the same linear interpolation.
///
/// This exists so the SOFT LOCK and the FFB END STOP can be the same place.
/// The soft lock clamps the road-wheel angle at the car's live `max_steer_rad`,
/// which is editable while driving; the end stop was built from the constant
/// [`STEER_RIM_LOCK_DEG`]. They coincide only at the default 46 deg of lock.
/// Set lock to 20 and the road angle saturated well before rim 179, leaving
/// tens of degrees of rim travel that steered nothing and pushed back with
/// nothing -- the wheel simply went slack.
pub fn rim_from_road_deg(road_deg: f64) -> f64 {
    let n = STEER_MAP_ROAD_DEG.len();
    let sign = if road_deg < 0.0 { -1.0 } else { 1.0 };
    let mag = road_deg.abs();
    let last = STEER_MAP_ROAD_DEG[n - 1];
    if mag >= last {
        // Past the table, hold the last slope -- the mirror of what
        // `road_from_rim_deg` does out there.
        let slope = (last - STEER_MAP_ROAD_DEG[n - 2]) / STEER_MAP_STEP_DEG;
        if slope <= 0.0 {
            return sign * (n - 1) as f64 * STEER_MAP_STEP_DEG;
        }
        return sign * ((n - 1) as f64 * STEER_MAP_STEP_DEG + (mag - last) / slope);
    }
    // The table is monotonic, so a scan is enough and is clearer than a
    // binary search over 37 entries.
    for i in 0..n - 1 {
        let (a, b) = (STEER_MAP_ROAD_DEG[i], STEER_MAP_ROAD_DEG[i + 1]);
        if mag <= b {
            let t = if (b - a).abs() < 1e-12 { 0.0 } else { (mag - a) / (b - a) };
            return sign * (i as f64 + t) * STEER_MAP_STEP_DEG;
        }
    }
    sign * (n - 1) as f64 * STEER_MAP_STEP_DEG
}

/// Local d(road)/d(rim) at a rim angle -- the reciprocal of the LOCAL steering
/// ratio, and with it the rim/kingpin torque ratio at that angle. About 1/5.27
/// on centre and 1/3.4 past 90 deg, so the rim gets roughly 16 % less torque
/// per unit kingpin moment on centre than the nominal ratio says, and ~30 %
/// more in a hairpin.
pub fn road_per_rim(rim_deg: f64) -> f64 {
    let n = STEER_MAP_ROAD_DEG.len();
    let i = ((rim_deg.abs() / STEER_MAP_STEP_DEG).floor() as usize).min(n - 2);
    (STEER_MAP_ROAD_DEG[i + 1] - STEER_MAP_ROAD_DEG[i]) / STEER_MAP_STEP_DEG
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
    /// Give each front wheel its own speed state (and so its own slip ratio,
    /// brake-torque half and Fx), instead of one front rotor. Lets the
    /// unloaded inside front lock first under braking in a corner, and adds
    /// the yaw moment of the front brake-force difference. Off by default --
    /// the validated, skidpad-pinned model -- and switched on by the rig for
    /// FFB/steering model v2.1.
    pub split_front_wheels: bool,
    pub aero: AeroParams,
    pub roll: RollParams,
    pub brakes: BrakeParams,
    pub steering: SteeringParams,
    pub diff: DiffParams,
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

    /// Static load on one tyre (N) -- the reference for load sensitivity.
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

/// SDM26 -- Sun Devil Motorsports' 2026 car.
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
        // 0.015: mid-band for a 10 in slick at 10 psi. 0.02 was a road-tyre
        // number worth ~20 N of phantom drag at every speed.
        crr: 0.015,
        front_grip_factor: 0.80,
        split_front_wheels: false,
        aero: AeroParams {
            // 2026 full-car CFD ride-height map at nominal ride height
            // ('Ride Height Data (BW)'); see params.js `aeroFrontFrac`.
            cda_m2: 1.267,
            cla_m2: 3.132,
            front_frac: 0.524,
            air_density: 1.162,
        },
        roll: RollParams {
            // TEAM: the measured front 4-7 / rear 1-1 blades, which is what
            // the team runs on the acceleration car. 1-1/1-1 is 0.46 and is
            // the event target, but the driver wants the car to rotate less
            // and front roll stiffness is the most direct lever.
            // 2026-09-21 (Nick): 0.48, between the two; see params.js.
            rsd_front: 0.48,
            roll_arm_m: 0.2626,
            rc_front_m: 0.0186,
            rc_rear_m: 0.0251,
        },
        // 70 bar max working pressure through the measured callipers and
        // 54% bias bar; see params.js. Bias set to 0.65 front torque share
        // 2026-09-21 (Nick), a bar near 45% front.
        brakes: BrakeParams { max_torque_nm: 1235.0, bias_front: 0.65 },
        // TEAM: Drexler Formula Student V3 in its default 40 deg drive /
        // 50 deg coast configuration, which the manual's lock table reads as
        // 0.60 and 0.42, with the fixed unit's 25 N.m breakaway preload. The
        // same three numbers the AC mod runs.
        diff: DiffParams { power_lock: 0.60, coast_lock: 0.42, preload_nm: 25.0, stick_rad_s: 0.1 },
        steering: SteeringParams {
            // The rack's measured limit. `wheel_toe_angles.csv` reaches
            // 46.0 deg of road wheel at its 179.24 deg of rim; the old 28 was
            // an estimate bounded by a MoTeC STEERING channel that three of
            // four runs capped at 121-124 deg of rim, which is simply where
            // the driver stopped turning, not where the rack stops.
            max_steer_rad: 46.0_f64.to_radians(),
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
            // Same OptimumK export ('Actual' column): scrub radius 25.5 mm,
            // KPI 8.745 deg. They do not enter the vehicle's motion at all --
            // only the jacking torque the rig adds to the force feedback.
            scrub_m: 0.0255,
            kpi_rad: 8.745_f64.to_radians(),
            rack_efficiency: 0.85,
            torque_ratio: None,
            ackermann: 0.0,
        },
    }
}

/// SDM25 -- same chassis family, heavier, shorter final drive.
pub fn sdm25() -> VehicleParams {
    VehicleParams {
        name: "SDM25".into(),
        mass_kg: 281.0,
        roll: RollParams { rsd_front: 0.36, ..sdm26().roll },
        ..sdm26()
    }
}

#[cfg(test)]
mod steer_map_tests {
    use super::*;

    #[test]
    fn rim_from_road_inverts_road_from_rim() {
        let mut rim = 0.0_f64;
        while rim <= 200.0 {
            let road = road_from_rim_deg(rim);
            let back = rim_from_road_deg(road);
            assert!((back - rim).abs() < 1e-6, "rim {rim} -> road {road} -> rim {back}");
            rim += 2.5;
        }
    }

    #[test]
    fn it_is_odd_about_zero() {
        for road in [1.0_f64, 12.5, 30.0, 46.2288, 60.0] {
            assert!((rim_from_road_deg(-road) + rim_from_road_deg(road)).abs() < 1e-9);
        }
    }

    #[test]
    fn full_lock_lands_on_the_measured_rack_stop() {
        let rim = rim_from_road_deg(STEER_MAP_ROAD_DEG[STEER_MAP_ROAD_DEG.len() - 1]);
        assert!((rim - 180.0).abs() < 1e-6, "got {rim}");
    }
}
