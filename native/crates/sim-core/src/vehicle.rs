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
    // The sprung-CG to roll-axis arm is NOT stored: it is
    // [`VehicleParams::roll_arm`], derived from the CG height and the roll
    // centres, so editing either one moves it the way the car would.
    pub rc_front_m: f64,
    pub rc_rear_m: f64,
}

/// The suspension as the double-track solver runs it: a sprung body that
/// rolls and pitches on springs and dampers, and wheels whose camber follows
/// the body, the bumps and the steer.
///
/// Sources (SDM26, 2026):
///  * 'SDM26 Ride Roll Calc' (Drive): wheel rates 363.9 / 249.8 lb/in, tyre
///    vertical rate 520 lb/in, roll gradient 0.602 and pitch gradient
///    0.805 deg/g WITH tyres, damping-ratio targets 0.80 front / 0.90 rear.
///  * 'SDM26 Designed vs Actual Kinematics' (OptimumK, actual column): static
///    camber -0.8 / -0.7 deg; camber gain in roll 0.657 / 0.738 deg per deg
///    (camber to GROUND per degree of body roll, i.e. 1 - t / (2 FVSA)); in
///    heave -0.826 / -0.678 deg per inch of bump; anti-dive 12.8 %, rear
///    anti-lift 15.8 %, rear anti-squat 11.5 % (mean of 10.4 / 12.6).
///  * 'SDM26 Full-Vehicle Sim Parameters' workbook: Ixx 24.8, Iyy 85.3 kg m^2.
///
/// Roll and pitch STIFFNESS are not stored: they come from the gradients, so
/// the gradients the team validates are what the car does, and the roll
/// split between axles is the existing `RollParams::rsd_front` setup knob.
#[derive(Debug, Clone, PartialEq)]
pub struct SuspensionParams {
    /// Body roll per g of lateral acceleration, tyres included (deg/g).
    pub roll_gradient_deg_g: f64,
    /// Fraction of critical damping, jounce and rebound (the dampers are
    /// asymmetric). Heave and pitch use them by direction at each axle; roll,
    /// which has one side in jounce and the other in rebound, their mean.
    pub damping_jounce: f64,
    pub damping_rebound: f64,
    /// Chassis torsional stiffness between the axles (N.m/deg). In series
    /// between the front and rear roll springs, it pulls the lateral load
    /// transfer split toward the mass split.
    pub chassis_torsion_nm_deg: f64,
    /// Sprung-body inertias about its own CG (kg m^2).
    pub ixx_kg_m2: f64,
    pub iyy_kg_m2: f64,
    /// Tyre vertical rate (N/m). Splits body roll into what the springs take
    /// (camber follows the linkage) and what the tyres take (the whole axle
    /// leans, camber follows 1:1).
    pub tyre_rate_n_m: f64,
    /// Front and rear wheel rates (N/m), springs through their motion
    /// ratios, no tyre. Only used for that split.
    pub wheel_rate_front_n_m: f64,
    pub wheel_rate_rear_n_m: f64,
    /// Anti geometry, fractions of the sprung longitudinal transfer that the
    /// links carry instead of the springs.
    pub anti_dive_front: f64,
    pub anti_lift_rear: f64,
    pub anti_squat_rear: f64,
    /// Static camber, SAE: negative is the top of the wheel leaning inboard.
    pub static_camber_front_deg: f64,
    pub static_camber_rear_deg: f64,
    /// Camber to ground per degree of suspension roll (outer wheel goes
    /// positive by this much).
    pub camber_gain_roll_front: f64,
    pub camber_gain_roll_rear: f64,
    /// Camber change per metre of bump (deg/m; negative = more negative in
    /// bump).
    pub camber_gain_bump_front_deg_m: f64,
    pub camber_gain_bump_rear_deg_m: f64,
    /// Static toe-in per wheel (deg; positive = the front of the wheel
    /// pointing toward the centreline).
    pub toe_in_front_deg: f64,
    pub toe_in_rear_deg: f64,
    /// Fraction of true Ackermann (0 parallel, 1 full). Here rather than in
    /// `SteeringParams` so the bicycle's parameters are untouched.
    pub ackermann: f64,
    /// Bump steer: toe-IN per metre of wheel travel into bump (deg/m), from
    /// the suspension geometry. Roll steer falls out of it (one wheel in
    /// bump, the other in droop), and so does toe under pitch and aero squat.
    pub bump_steer_front_deg_m: f64,
    pub bump_steer_rear_deg_m: f64,
    /// Steering compliance: road-wheel degrees each front wheel gives way per
    /// 100 N.m of moment about its kingpin -- rack, column, tie rods, rod
    /// ends, upright. 0 is a rigid system.
    pub steer_compliance_deg_per_100nm: f64,
    /// How much of `front_grip_factor`'s deficit this model resolves: the
    /// front runs at front_grip_factor x this (capped at 1). See
    /// double_track.rs for the calibration.
    pub front_grip_scale: f64,
    /// The rear tyres' lateral grip relative to the tyre's mu_y. See
    /// double_track.rs for the calibration.
    pub rear_grip_scale: f64,
    /// Aero that follows ride height (the 2026 CFD ride-height map). Off, the
    /// double track carries the fixed nominal ClA / CdA / front share exactly
    /// as the bicycle does.
    pub aero_map: AeroRideMap,
}

/// The 2026 full-car CFD ride-height map ('Ride Height Data (BW)', Drive
/// Aero/Aero Map/Ride Height, 2026-04; 5 x 5 grid, front and rear ride height
/// each -1..+1 in from nominal) reduced to how each force moves per inch.
///
/// The grid itself is CFD-noisy: adjacent cells differ by 10 % with no
/// trend, and the (front -1, rear +1) corner has the front wing in the
/// ground and is flagged bad in the sheet. So it is a least-squares PLANE
/// through the other 24 cells of front downforce (total x % front), rear
/// downforce and drag. A quadratic takes the front residual only from 5.0 to
/// 4.1 lbf, i.e. into the noise. Fit: sim/tools/aero_map_fit.py.
///
///   front DF = 52.13 - 9.87 dRH_f + 5.69 dRH_r   (lbf at 15.65 m/s; rms 5.0)
///   rear  DF = 49.06 + 1.05 dRH_f - 0.10 dRH_r   (rms 2.8)
///   drag     = 41.86 + 0.30 dRH_f + 0.55 dRH_r   (rms 1.0)
///
/// Stored RELATIVE to the plane's own value at nominal, and applied as a
/// multiplier on the car's nominal ClA / front share / CdA, so at nominal
/// ride height the double track's aero is exactly `AeroParams`: the map
/// supplies only how it moves. The front wing is in ground effect (front DF
/// rises as the nose drops) and rake loads it further; the rear barely moves.
#[derive(Debug, Clone, PartialEq)]
pub struct AeroRideMap {
    pub enabled: bool,
    /// Fractional change per inch of (front, rear) ride height RISE.
    pub front_df_per_in: [f64; 2],
    pub rear_df_per_in: [f64; 2],
    pub drag_per_in: [f64; 2],
    /// The map's extent; ride heights beyond it are held at its edge.
    pub limit_in: f64,
}

impl AeroRideMap {
    pub fn sdm26() -> Self {
        Self {
            enabled: true,
            front_df_per_in: [-9.870 / 52.127, 5.688 / 52.127],
            rear_df_per_in: [1.050 / 49.062, -0.103 / 49.062],
            drag_per_in: [0.298 / 41.862, 0.553 / 41.862],
            limit_in: 1.0,
        }
    }

    /// (front DF, rear DF, drag) multipliers at ride-height deltas (m, +
    /// = higher than static).
    pub fn factors(&self, rh_front_m: f64, rh_rear_m: f64) -> (f64, f64, f64) {
        if !self.enabled {
            return (1.0, 1.0, 1.0);
        }
        let lim = self.limit_in;
        let f = (rh_front_m / 0.0254).clamp(-lim, lim);
        let r = (rh_rear_m / 0.0254).clamp(-lim, lim);
        let lin = |k: [f64; 2]| (1.0 + k[0] * f + k[1] * r).max(0.0);
        (lin(self.front_df_per_in), lin(self.rear_df_per_in), lin(self.drag_per_in))
    }
}

impl SuspensionParams {
    pub fn sdm26() -> Self {
        Self {
            // 2026-09-22: re-derived for the car AS RUN, not as the Ride Roll
            // sheet's 280 lb/in front spring had it. Front spring 200 lb/in
            // (spec sheet, every 2026 setup sheet); axle roll stiffness from
            // the 'SDM26 Anti-Roll Bar Calculator' at the event setting
            // (~47 % RSD, e.g. F1-1 / R6-7: ~746 / ~914 N.m/deg), each in
            // series with the tyres at 520 lb/in -> ~959 N.m/deg, so
            // ms g arm / K = 0.66 deg/g. Pitch: the front ride rate with tyre
            // falls from 214 to 173 lb/in, 0.805 -> 0.91 deg/g. Roll is to be
            // checked against the shock pots in the SDM26 test logs.
            roll_gradient_deg_g: 0.66,
            // PITCH is no longer a gradient: 2026-09-22 the body heaves and
            // pitches on the axle ride rates themselves (springs through the
            // motion ratio, in series with the tyre, both wheels), which gives
            // ~0.45 deg/g. The team's 0.91 (Ride Roll Calc 0.805 at 280 lb/in)
            // is its formula k_f k_r / (k_f + k_r) L^2 on ONE wheel's rate per
            // axle -- recomputed from the sheet; with both wheels it halves,
            // and the Suspension Design Report's own target is 0.5 deg/g.
            //
            // Ohlins TTX25, force-matched to target damping ratios (Suspension
            // Design Report 4.4); the Overall Vehicle Design Report's spec
            // table gives 70 % critical in jounce and 80 % in rebound at
            // 50 mm/s, front and rear.
            damping_jounce: 0.70,
            damping_rebound: 0.80,
            // Overall Vehicle Design Report sec. 2: "Torsional stiffness |
            // 1300 target; 1482 simulated; 960 physical test". The test.
            chassis_torsion_nm_deg: 960.0,
            ixx_kg_m2: 24.8,
            iyy_kg_m2: 85.3,
            tyre_rate_n_m: 520.0 * 175.126_835,
            wheel_rate_front_n_m: 200.0 * 1.14 * 1.14 * 175.126_835,
            wheel_rate_rear_n_m: 249.833 * 175.126_835,
            anti_dive_front: 0.128,
            anti_lift_rear: 0.158,
            anti_squat_rear: 0.115,
            static_camber_front_deg: -0.8,
            static_camber_rear_deg: -0.7,
            camber_gain_roll_front: 0.657,
            camber_gain_roll_rear: 0.738,
            camber_gain_bump_front_deg_m: -0.826 / 0.0254,
            camber_gain_bump_rear_deg_m: -0.678 / 0.0254,
            // AS RUN, per wheel, spec-sheet sign "- out, + in": front +1.1
            // (toe-in), rear +0.5 (toe-in) for autocross and endurance
            // (Overall Vehicle DR 5.3; 4/23 and 4/25 setup sheets). The
            // skidpad setup runs the rear at -0.7 (toe-out). The OptimumK
            // export's rear -0.5 was the DESIGN value, not what was run.
            toe_in_front_deg: 1.1,
            toe_in_rear_deg: 0.5,
            // Measured from both road wheels against rim angle
            // (Steer_Force_Calculator/wheel_toe_angles.csv): 18.5 % of true
            // Ackermann to 60 deg of rim, rising to 24 % at full lock. Not
            // the OptimumK 0 % nor the spec sheet's 85 %.
            ackermann: 0.185,
            // From the OptimumK hardpoints ('SDM26 V1.4.6', transcribed in
            // sdm26-assetto-corsa/data/sdm26_team_data.json), a small-
            // displacement solve of the wishbones and tie rod
            // (sim/docs/analysis-2026-09-22/scripts/bump_steer.py): 0.214 deg
            // of toe-in per inch of bump at the front, 0.040 at the rear. The
            // same solve reproduces OptimumK's camber gains to 1-3 %.
            bump_steer_front_deg_m: 0.2139 / 0.0254,
            bump_steer_rear_deg_m: 0.0402 / 0.0254,
            // EST. No SDM26 K&C measurement exists (the 2026 compliance
            // project published no results; the FEA of the upright alone is
            // 0.002 deg at 1.6 g). 0.5 deg per 100 N.m per wheel is a tight
            // FSAE steering system; loose ones measure several times that.
            // Adjustable on the setup card; 0 turns it off.
            steer_compliance_deg_per_100nm: 0.5,
            // Calibrated, see double_track.rs (fifth pass).
            front_grip_scale: 1.08,
            rear_grip_scale: 1.00,
            aero_map: AeroRideMap::sdm26(),
        }
    }
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
    /// Steering-feel calibration: the fraction of the modelled tyre moment
    /// that reaches the driver's hands, on top of `rack_efficiency`. Feel
    /// only -- it scales `rim_torque_ratio` (and the rig's local-ratio rim
    /// torque), never the kingpin moment, so nothing about the car's motion
    /// depends on it. See `sdm26()` for how it was set.
    pub feel_scale: f64,
}

/// The radius at which the driver holds the rim (m), for turning a rim
/// FORCE into a torque: 5.7 in, MEASURED on the SDM26 steering wheel
/// (2026-09-23). The design report's "Steer Force Targets" plot (SDM26
/// design report p.20, a design calculation, not a measurement) is in
/// pounds-force at the rim and does not say where the force acts; this is
/// where the driver's hands are. `feel_scale` was calibrated through it.
pub const STEER_RIM_GRIP_RADIUS_M: f64 = 0.1448;

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

// The map is a monotone cubic (PCHIP, Fritsch-Carlson) through the measured
// points, not straight lines between them. With linear interpolation the
// position was only C0, so its slope -- the local ratio, and with it the
// rim/kingpin TORQUE ratio -- jumped by up to 3.6 % every 5 deg of rim: about
// 0.5 N.m of step at 14 N.m, felt as notches on a direct-drive base. The
// cubic passes through every measured point, cannot overshoot between them,
// and its derivative is continuous; `road_per_rim` is that derivative, so the
// angle and the torque come from one curve. `sim/src/vehicle/params.js`
// carries the same construction and the two must agree.

/// Secant slope of segment `i` (deg road per deg rim).
fn steer_seg_slope(i: usize) -> f64 {
    (STEER_MAP_ROAD_DEG[i + 1] - STEER_MAP_ROAD_DEG[i]) / STEER_MAP_STEP_DEG
}

/// PCHIP node slope at point `k`. The map is odd about zero, so the slope at
/// the centre is the first secant (the mirrored point makes the two secants
/// either side equal); at the far end it is the last secant, so the curve
/// joins the straight over-travel ramp without a kink.
fn steer_node_slope(k: usize) -> f64 {
    let n = STEER_MAP_ROAD_DEG.len();
    if k == 0 {
        return steer_seg_slope(0);
    }
    if k >= n - 1 {
        return steer_seg_slope(n - 2);
    }
    let (d0, d1) = (steer_seg_slope(k - 1), steer_seg_slope(k));
    if d0 * d1 <= 0.0 {
        0.0
    } else {
        2.0 / (1.0 / d0 + 1.0 / d1)
    }
}

/// The cubic on segment `i` at local position `t` in 0..1: (road deg, slope).
fn steer_hermite(i: usize, t: f64) -> (f64, f64) {
    let h = STEER_MAP_STEP_DEG;
    let (y0, y1) = (STEER_MAP_ROAD_DEG[i], STEER_MAP_ROAD_DEG[i + 1]);
    let (m0, m1) = (steer_node_slope(i), steer_node_slope(i + 1));
    let (t2, t3) = (t * t, t * t * t);
    let y = (2.0 * t3 - 3.0 * t2 + 1.0) * y0
        + (t3 - 2.0 * t2 + t) * h * m0
        + (-2.0 * t3 + 3.0 * t2) * y1
        + (t3 - t2) * h * m1;
    let dy = ((6.0 * t2 - 6.0 * t) * y0
        + (3.0 * t2 - 4.0 * t + 1.0) * h * m0
        + (-6.0 * t2 + 6.0 * t) * y1
        + (3.0 * t2 - 2.0 * t) * h * m1)
        / h;
    (y, dy)
}

/// Road-wheel angle (deg, signed) for a rim angle.
pub fn road_from_rim_deg(rim_deg: f64) -> f64 {
    let n = STEER_MAP_ROAD_DEG.len();
    let sign = if rim_deg < 0.0 { -1.0 } else { 1.0 };
    let mag = rim_deg.abs();
    let x = mag / STEER_MAP_STEP_DEG;
    if x >= (n - 1) as f64 {
        // Past the table: hold the last slope, so over-travel is a ramp and
        // not a cliff. The end stop is what should be resisting by then.
        let last = STEER_MAP_ROAD_DEG[n - 1];
        return sign * (last + steer_seg_slope(n - 2) * (mag - (n - 1) as f64 * STEER_MAP_STEP_DEG));
    }
    let i = x.floor() as usize;
    sign * steer_hermite(i, x - i as f64).0
}

/// Rim angle (deg, signed) that produces a given road-wheel angle -- the
/// inverse of [`road_from_rim_deg`], on the same curve.
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
        let slope = steer_seg_slope(n - 2);
        if slope <= 0.0 {
            return sign * (n - 1) as f64 * STEER_MAP_STEP_DEG;
        }
        return sign * ((n - 1) as f64 * STEER_MAP_STEP_DEG + (mag - last) / slope);
    }
    // The table is monotonic and so is the cubic through it: find the segment
    // by its end points, then bisect inside it. Sixty halvings of one 5 deg
    // segment is far below double precision, and deterministic -- the JS port
    // does exactly the same.
    for i in 0..n - 1 {
        let (a, b) = (STEER_MAP_ROAD_DEG[i], STEER_MAP_ROAD_DEG[i + 1]);
        if mag <= b {
            if (b - a).abs() < 1e-12 {
                return sign * i as f64 * STEER_MAP_STEP_DEG;
            }
            let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if steer_hermite(i, mid).0 < mag {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            return sign * (i as f64 + 0.5 * (lo + hi)) * STEER_MAP_STEP_DEG;
        }
    }
    sign * (n - 1) as f64 * STEER_MAP_STEP_DEG
}

/// Local d(road)/d(rim) at a rim angle -- the reciprocal of the LOCAL steering
/// ratio, and with it the rim/kingpin torque ratio at that angle. About 1/5.27
/// on centre and 1/3.4 past 90 deg, so the rim gets roughly 16 % less torque
/// per unit kingpin moment on centre than the nominal ratio says, and ~30 %
/// more in a hairpin. Continuous: it is the derivative of the same cubic.
pub fn road_per_rim(rim_deg: f64) -> f64 {
    let n = STEER_MAP_ROAD_DEG.len();
    let x = rim_deg.abs() / STEER_MAP_STEP_DEG;
    if x >= (n - 1) as f64 {
        return steer_seg_slope(n - 2);
    }
    let i = x.floor() as usize;
    steer_hermite(i, x - i as f64).1
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
    /// Suspension and camber. Only the double-track solver uses it; the
    /// bicycle keeps its validated quasi-static transfer.
    pub suspension: SuspensionParams,
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

    /// Height of the SPRUNG mass's CG (m). `cg_height_m` is the whole car's,
    /// unsprung included; the unsprung mass sits at the wheel centre, well
    /// below it, so the sprung CG is higher than the total one.
    pub fn sprung_cg_height(&self) -> f64 {
        let mu = 2.0 * (self.unsprung_front_kg + self.unsprung_rear_kg);
        (self.mass_kg * self.cg_height_m - mu * self.tyre_radius_m) / self.sprung_mass().max(1e-6)
    }

    /// Roll-axis height under the CG (m), on the line between the two roll
    /// centres.
    pub fn roll_axis_height_at_cg(&self) -> f64 {
        self.roll.rc_front_m + (self.roll.rc_rear_m - self.roll.rc_front_m) * self.a() / self.wheelbase_m
    }

    /// Sprung-CG to roll-axis arm (m): the lever the elastic (spring and bar)
    /// load transfer works through.
    ///
    /// Derived, not a parameter. It used to be a stored 0.2626 m -- which is
    /// the TOTAL CG height less the roll axis, so the elastic path ran 4 % short
    /// and total lateral transfer 3.4 % under the rigid-body m.ay.h/t -- and,
    /// being stored, it ignored edits to the CG height and the roll centres:
    /// raising a roll centre added geometric transfer without taking any from
    /// the elastic path, which no car does. With it derived, elastic +
    /// geometric + unsprung sums to exactly m.ay.h/t whatever is edited.
    pub fn roll_arm(&self) -> f64 {
        self.sprung_cg_height() - self.roll_axis_height_at_cg()
    }

    pub fn weight(&self) -> f64 {
        self.mass_kg * G
    }

    /// Mechanical trail from caster and kingpin offset (m).
    pub fn mechanical_trail(&self) -> f64 {
        self.tyre_radius_m * self.steering.caster_rad.tan() + self.steering.kingpin_offset_trail_m
    }

    /// Rim torque per unit kingpin torque through the rack, losses included:
    /// the mechanical path, for torques that are not the tyres' (jacking).
    pub fn rim_mech_ratio(&self) -> f64 {
        let r = self.steering.torque_ratio.unwrap_or(1.0 / self.steering.ratio.max(1e-6));
        r * self.steering.rack_efficiency
    }

    /// Rim torque per unit kingpin torque of the TYRES' moment: the
    /// mechanical path times the steering-feel calibration `feel_scale`.
    pub fn rim_torque_ratio(&self) -> f64 {
        self.rim_mech_ratio() * self.steering.feel_scale
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
        suspension: SuspensionParams::sdm26(),
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
            // Steering-feel calibration, 2026-09-23. The modelled rim torque
            // was well above the design target: SDM26 design report p.20
            // "Steer Force Targets" (a design calculation, not a
            // measurement), autocross curve 12.5 lbf at 10 deg of
            // steering-wheel angle, 11.7 at 40, 16.2 at 170 -- 8.05 / 7.54
            // / 10.43 N.m at the measured 0.1448 m grip radius
            // (`STEER_RIM_GRIP_RADIUS_M`). The model, in steady cornering at
            // 1.0 g (below the front's limit; the report's calculation has
            // no tyre saturation), made 12.00 / 12.11 / 17.82 N.m at those
            // angles through the measured rack (examples/steer_torque.rs,
            // AY=1.0). Least squares over the three points: 0.61. The shape
            // already agreed -- flat to ~40 deg, rising toward lock as the
            // progressive rack gains leverage -- apart from the report's
            // small hump at 10 deg (a 6 % dip to 40 deg; the model's is 4 %,
            // at 20) and a steeper rise to lock (x1.48 from 10 to 170 deg
            // against the report's x1.30). Where the rest goes is not
            // modelled: the TTC flat-belt pneumatic trail, column and rack
            // friction and reverse efficiency are all candidates. Feel only
            // (see the field).
            feel_scale: 0.61,
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
    fn map_passes_through_every_measured_point() {
        for (k, want) in STEER_MAP_ROAD_DEG.iter().enumerate() {
            let got = road_from_rim_deg(k as f64 * STEER_MAP_STEP_DEG);
            assert!((got - want).abs() < 1e-9, "point {k}: {got} vs {want}");
        }
    }

    #[test]
    fn torque_ratio_has_no_steps() {
        // The old linear map's slope jumped up to 3.6 % at every table point.
        let mut prev = road_per_rim(0.0);
        let mut rim = 0.05;
        while rim <= 200.0 {
            let now = road_per_rim(rim);
            assert!(now > 0.0, "slope went non-positive at {rim}");
            assert!(((now - prev) / prev).abs() < 0.002, "step of {:.3} % at rim {rim}", (now / prev - 1.0) * 100.0);
            prev = now;
            rim += 0.05;
        }
    }

    #[test]
    fn slope_is_the_derivative_of_the_map() {
        for rim in [0.3_f64, 7.0, 44.9, 91.0, 150.0, 178.0] {
            let h = 1e-5;
            let fd = (road_from_rim_deg(rim + h) - road_from_rim_deg(rim - h)) / (2.0 * h);
            assert!((fd - road_per_rim(rim)).abs() < 1e-6, "rim {rim}: fd {fd} vs {}", road_per_rim(rim));
        }
    }

    #[test]
    fn full_lock_lands_on_the_measured_rack_stop() {
        let rim = rim_from_road_deg(STEER_MAP_ROAD_DEG[STEER_MAP_ROAD_DEG.len() - 1]);
        assert!((rim - 180.0).abs() < 1e-6, "got {rim}");
    }
}
