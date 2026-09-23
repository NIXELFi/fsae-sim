// SDM26 vehicle parameters.
//
// Everything above the "DRIVING-SIM ADDITIONS" line is lifted verbatim from
// Helios (`apps/desktop/src/modules/cfd/lib/performance/vehicle.ts` -> SDM26_VEHICLE and
// SDM26_ROLL), so this car is the same car the lap sim scores. Grip is the
// skidpad-pinned muLat 1.368 that reproduces SDM26's real 5.02 s skidpad, and
// the aero map is the 2026 CFD map at nominal ride height.
//
// Below that line are the states a *transient* model needs that a quasi-steady
// lap sim never had to name: rotational inertias, steering lock and rate, brake
// torque, relaxation length. Those are engineering estimates, each marked EST
// with its basis. They are the numbers to replace when the team measures them.

/**
 * How high the driver's eye can sit (m above the ground), from the SDM26
 * driver-interface CAD: the head restraint pad (275 mm tall, on two mounting
 * positions, spanning 0.612-0.919 m in the chassis frame) has to meet the
 * helmet at least 50 mm from any edge (FSAE T.2.8), and the drawn helmet's
 * centre is 30 mm above the eye. Both positions together: 0.632-0.839,
 * rounded in. The rollover line (helmet 50 mm under the main-to-front hoop
 * line) only bites near 0.95, so it is not the limit here.
 */
export const EYE_HEIGHT_RANGE_M = [0.635, 0.835];

/**
 * The driver's eye fore-aft (m ahead of the CG) in the team's CAD car, from
 * its driver-interface assembly: the head restraint's foam face is 0.204 m
 * behind the CG and the rules want the helmet on it (no more than 25 mm
 * off, FSAE T.2.8); the drawn helmet's back is 0.175 m behind the eye
 * (carmesh driverPose), so 10 mm of gap puts the eye at -0.02. At the
 * classic car's -0.12 the helmet sat 90 mm into the pad and through the
 * main hoop. Camera and driver model only.
 */
export const CAD_EYE_AHEAD_OF_CG_M = -0.02;

export const SDM26 = {
  name: "SDM26",

  // ---- mass & geometry (Helios) ----
  // Which vehicle model the desktop rig runs: 2 = the validated transient
  // bicycle, 3 = the double track with a rolling, pitching body and camber
  // (beta; see sim-core `solver/double_track.rs`). A model choice, not a
  // setup: a lap on the beta model does not count. The browser build has
  // only the bicycle and ignores it.
  vehicleModel: 2,
  massKg: 267,            // 199 kg car + 68 kg driver
  weightDistFront: 0.485, // with driver
  cgHeightM: 0.2845,
  wheelbaseM: 1.53,
  trackFrontM: 1.207,
  trackRearM: 1.194,
  tireRadiusM: 0.2,       // Hoosier 16x7.5-10, loaded radius

  // ---- grip ----
  // muLat is RE-CALIBRATED for this model and deliberately differs from the
  // lap sim's 1.368. Helios pins 1.368 at the skidpad in a quasi-steady model
  // that applies load sensitivity to the axle as a whole. This model also
  // splits each axle left/right and derates grip for the lateral load transfer
  // (see tire.axleMu), which costs a further ~6% of mu on a skidpad. Reusing
  // 1.368 here therefore double-counts the derate and yields a 5.38 s skidpad
  // against the 5.02 s SDM26 actually ran. Same measurement, different model,
  // so a different constant. (Oracle hit the same thing and solved it the same
  // way, with mu_scale.)
  //
  // This is the REAR axle's peak lateral mu; the front runs at
  // muLat * frontGripFactor. Together they are pinned so that the skidpad
  // comes out at 5.02 s through this model: 1.573 did that with equal axles,
  // 1.66 x 0.90 = 1.49 at the front did it once the front limited first, and
  // 1.72 x 0.88 = 1.51 does it with the team's own unsprung masses, yaw
  // inertia, TTC load sensitivity (0.12) and the 2026 CFD aero split in
  // place: 11.40 m/s sustained on the 9.125 m circle = 5.03 s, the harness's
  // 0.05 m/s resolution. The TTC belt peak for this tyre is 1.51 at 700 N,
  // unscaled, which the front now happens to match.
  // 2026-09-17, wheel testing: was 1.72 with frontGripFactor 0.88. Off throttle
  // the model was neutral to within the load shift of engine braking, so a
  // held 14 deg at 13 m/s spun it where the real car pushes. The product
  // muLat * frontGripFactor (the skidpad-pinned front peak) is unchanged;
  // the rear gets 10% more margin. Skidpad check moves 5.03 -> 5.14 s.
  muLat: 1.89,
  muLatHeliosQss: 1.368,  // kept for traceability to the lap sim
  // EST: front axle peak lateral grip relative to the rear.
  //
  // With equal tyres front and rear, the only things that set this model's
  // limit balance are weight distribution, load transfer and the aero split,
  // and those leave it NEUTRAL to within 1% of force at every speed: with the
  // team's roll-stiffness baseline the rear axle reached its peak first at
  // 10, 15 and 20 m/s, and a steering input at the limit -- 12 deg at 15 m/s
  // over 150 ms, a keyboard tap -- spun the car every time, because once both
  // axles are past the peak the yaw moment a*FyF - b*FyR stays positive (a > b
  // on a 48.5% front car) and nothing arrests the yaw. Roll stiffness alone
  // cannot fix that: moving rsdFront to 0.63 only shifts the steady-state
  // balance, and the same step steer still spun it.
  //
  // A real car's front lets go first by a clear margin, through things a
  // bicycle model with one tyre character cannot see: the steered upright
  // cannot carry as much camber as the rear and loses more in roll, the inside
  // front drags at parallel steer, and steering compliance. 0.90 put the front
  // at its peak while the rear still had ~5% of force in hand, a mild,
  // recoverable push at 10-20 m/s that survived a keyboard step to the
  // speed-limited lock -- with the aero split held at 50%. With the 2026 CFD
  // split (52.4% front) 0.90 leaves the rear limiting at 28 m/s (utilF - utilR
  // 0.07) and a held keyboard lock at 20 m/s spins the car (18 deg of body
  // slip); 0.88 restores the margin at every speed (0.19-0.36) and the 20 m/s
  // keyboard case pushes at 10.6 deg. This is the one knob that stands in for
  // everything the bicycle model cannot see about the front end, so it is the
  // one that moves when measured data replaces an estimate elsewhere. Replace
  // with a measured understeer gradient when the team has one (the 2026-04-08
  // test plan lists one; no result is on Drive).
  frontGripFactor: 0.80,
  muLong: 1.5,            // launch-traction estimate (75 m accel ~4.2 s)
  // TEAM (TTC): the team's PAC2002 fit of the R20 (workbook TIRES block,
  // FNOMIN 700 N) has PDY1 1.2169, PDY2 -0.14729, so peak lateral mu falls
  // PDY2/PDY1 = 12.1% per 100% of load -- the same linear law this model
  // uses. Longitudinal is 10.5% (PDX2/PDX1). Was Helios' 0.15 estimate.
  tireLoadSensitivity: 0.12,

  // ---- aero (2026 full-car CFD ride-height map, nominal RH) ----
  // Drive: Aero/Aero Map/Ride Height/'Ride Height Data (BW)' (2026-04):
  // 105.64 lbf down, 42.72 lbf drag at 15.65 m/s, rho 1.225, 52.42% front.
  // The Cl 2.918 / Cd 1.200 / 55.3% this file used to carry as the "2026
  // map" is the 2025-01 half-car 'Aero Map Data' sheet; the Aero Design
  // Binder's own headline (Cl 3.064 x 1.0224 m^2 = 3.13, CoP 53% front at
  // 15.64 m/s) agrees with the 2026 map, not the 2025 one. See
  // sim/tools/team_data.py.
  cdaM2: 1.267,           // 42.72 lbf / q at 15.65 m/s
  claM2: 3.132,           // 105.64 lbf / q
  // Front share of downforce: 0.524 at nominal ride height, CFD. Two
  // independent 2026 sweeps (ride-height and pitch maps) both give 52.42%
  // at the nominal point. Read where the car actually sits on its measured
  // springs (27.1 / 35.0 N/mm wheel rates, springs only), the map gives
  // 0.52 at 10-15 m/s rising to 0.54-0.57 at 20-30 m/s as the front wing
  // nears the ground; that edge of the map is coarse and the tyre and bump
  // stops are not in the ride-height estimate, so the nominal figure is
  // used as the single constant this model takes.
  //
  // History: this ran at 0.50 because 55.3% (the 2025 sheet) made the rear
  // the limiting axle above 20 m/s through this model. With the 2026 map's
  // 52.4% the front still limits first at every speed once frontGripFactor
  // is 0.88 (utilF - utilR 0.19-0.36 at 10-28 m/s), so the CFD number is
  // drivable and the estimate, not the data, is what moved.
  aeroFrontFrac: 0.524,
  aeroFrontFrac2025Sheet: 0.553, // superseded half-car map, for traceability
  airDensityKgM3: 1.162,
  // EST: rolling resistance. 0.015 is the middle of the 0.012-0.018 band for
  // a 10 in slick at the 10 psi this car runs; 0.02 was a road-tyre number
  // and put about 20 N of phantom drag on the car at every speed.
  crr: 0.015,

  // ---- driveline (Helios; stock CBR600RR PC40 + SDM 3.0 final) ----
  drivetrainEff: 0.85,
  gearRatios: [2.75, 2.0, 1.667, 1.444, 1.304, 1.208],
  primaryReduction: 2.111, // 76/36
  finalDrive: 3.0,
  revLimitRpm: 14500,
  // ESTIMATE, tuned by feel: the drivers report the real limiter as a hard
  // cut that "bounces a little". Ignition cut at revLimitRpm, back on this
  // far under it. The ECU's actual control range is worth reading off the
  // Link tune and putting here.
  revLimitHystRpm: 150,
  // Measured on the car (Daniel): the engine idles near 2000 rpm with the
  // throttle plate held at about 14%. Those two numbers are very nearly
  // self-consistent through the CFD torque curve and the friction model, which
  // put the zero-net-torque plate position at 14% somewhere around 2350 rpm --
  // agreement to a few hundred rpm, using nothing from the measurement itself.
  idleRpm: 2000,
  // TEAM: where the car's launch control is set. Also the crank speed the
  // clutch model holds against off the line. Note it sits in a dip in the
  // measured curve -- 49.5 N.m at 7000 against 55.1 at 6000 and 57.8 at
  // 8500 -- so it is a driveability choice rather than a torque one.
  // Nothing above walking pace reads it.
  launchRpm: 7000,
  // ESTIMATE, tuned by feel: launch control "bounces hard" on the real car,
  // so its hysteresis is much wider than the main limiter's.
  launchHystRpm: 400,
  /** Throttle plate position the ETC holds at idle, 0..1. */
  // Re-solved for the measured torque curve: the real engine makes far less
  // below 4000 rpm than the CFD sweep predicted, so the idle plate has to sit
  // further open to hold 2000 rpm. 0.14 was the CFD figure and idled at 1627.
  idleThrottleFrac: 0.22,
  // The ignition cut. 80-100 ms is what the driver reports off the real
  // car's paddle shift; 90 sits in the middle of that band.
  shiftTimeS: 0.09,
  // After the cut the torque comes back over this long, on a smoothstep,
  // rather than in one step: the gear is in and the ignition returns, but a
  // quickshifter feeds the spark back rather than slamming it. A step here
  // was a kick through the driveline on every shift, and a spike in the
  // pitch camera to go with it.
  shiftReintroS: 0.05,

  // ---- roll balance (Helios SDM26_ROLL, from the team's ARB calculator) ----
  roll: {
    // TEAM: 0.51, the measured front 4-7 / rear 1-1 blade setting -- the one
    // the team runs on the acceleration car. The 1-1/1-1 baseline is 0.46 and
    // is what the event setups target, but the driver reports the car rotating
    // too easily, and more front roll stiffness is the first and most direct
    // answer: it moves lateral load transfer forward, which costs the front
    // grip and makes the car push. Adjustable live from the wheel, 30-70%.
    // 2026-09-21 (Nick): set to 0.48, between the 1-1/1-1 baseline (0.46) and
    // the 4-7/1-1 setting (0.51) this ran at before.
    rsdFront: 0.48,
    // The sprung-CG to roll-axis arm is derived (`rollArm` below) from the
    // CG height and these roll centres, so an edit to either moves it.
    rcFrontM: 0.0186,  // front roll-centre height
    rcRearM: 0.0251,   // rear roll-centre height
  },

  // =====================================================================
  // DRIVING-SIM ADDITIONS -- estimates, not team measurements.
  // =====================================================================

  // TEAM: yaw inertia from the team's 'SDM26 Full-Vehicle Sim Parameters'
  // workbook (Drive, BODY block: Izz 93 660 784 kg.mm^2 at 253.3 kg listed,
  // with Ixx 24.8 and Iyy 85.3 kg.m^2). Not a bifilar measurement -- the
  // workbook does not say how it was obtained and it reads like CAD mass
  // properties -- but it is the team's number for this car, and it sits
  // inside the 85-130 range a bifilar rig gives FSAE cars. Kept as listed
  // rather than scaled to 267 kg: the 14 kg difference is driver and fuel,
  // which sit near the CG and add little to Izz. Dynamic index k^2/(ab)
  // = 0.60. See sim/tools/team_data.py.
  izzKgM2: 93.66,

  // TEAM: unsprung mass per corner from the same workbook (7.56 front,
  // 7.77 rear). The team's Ride Roll Calc sheet says 33 lb front / 38 lb
  // rear per axle (7.5 / 8.6 kg per corner) and the quarter-car script
  // 36.5 / 38 lb; the workbook is the most recent and most detailed, and
  // the spread is under 1 kg. Used to split lateral load transfer and to
  // find sprung mass.
  unsprungFrontKg: 7.56,
  unsprungRearKg: 7.77,

  // Rotational inertias. The WHEEL figures are TEAM data (workbook
  // wheel_spin_inertia: 0.154 front, 0.152 rear kg.m^2 per corner; the
  // brakes calculator's own build-up -- tyre 0.156, rim 0.042, rotor 0.001,
  // spindle 0.0004 = 0.200 -- lands within 0.05 of them). The DRIVELINE
  // figures are still EST.
  //
  // Driveline inertia is split at the primary because that is where the clutch
  // physically sits on a CBR600RR: crank -> primary gears -> clutch basket ->
  // gearbox. So the crank spins at primary x gear x final (17.4:1 in first)
  // while the basket, shafts and sprocket only see gear x final (8.25:1).
  // Lumping both at the crank overstates the reflected inertia by ~45% and
  // makes first gear feel like the car gained 135 kg. Split properly it is
  // ~3.7 kg.m^2 at the wheels in first, an apparent +94 kg -- still the single
  // biggest reason a bike-engined car is sluggish off the line, but the right
  // size. This is real physics the quasi-steady lap sim does not model at all,
  // and it is why the 75 m time here is honestly slower than the QSS 4.2 s.
  wheelInertiaFrontKgM2: 0.154, // TEAM: workbook, per wheel
  wheelInertiaRearKgM2: 0.152,  // TEAM: workbook, per wheel (disc is inboard)
  engineInertiaKgM2: 0.011,     // EST: crank + primary drive gear (crank-referenced)
  gearboxInertiaKgM2: 0.006,    // EST: clutch basket + shafts + sprocket (post-primary)

  // TEAM: steering lock, 46 deg at the road wheel -- the rack's MEASURED
  // limit, from the toe-vs-rim table below (46.0 deg of road wheel at 179.2
  // deg of rim). This replaces a long-standing 28 deg estimate that was
  // bounded by a MoTeC STEERING channel three of four 2026-04-01 runs capped
  // at 121-124 deg of rim; that is where the driver stopped turning, not
  // where the rack stops. Devices with no force feedback are still held to a
  // usable lock by the speed-sensitive cap in `controlProfiles.js`.
  // The lag and rate limit remain EST for the driver's hands plus rack
  // compliance.
  maxSteerDeg: 46,
  steerLagS: 0.06,
  steerRateDegS: 360,
  // TEAM: rim angle per road-wheel angle, 4.411 from the OptimumK 'SDM26
  // Designed vs Actual Kinematics' export (2026-06-27). Sets how far the
  // wheel rotates in the cockpit and the rim/kingpin torque ratio.
  steeringRatio: 4.411,

  // Steering geometry, for the force the driver feels. None of this affects
  // how the car goes round a corner; all of it sets what comes back through
  // the rim. Caster and trail are TEAM data from the same OptimumK export
  // (Actual column: caster 4.743 deg, KPI 8.745 deg, scrub 25.5 mm,
  // mechanical trail 18.85 mm, Ackermann 0). See sim/tools/team_data.py.
  steering: {
    /** Caster angle, deg. Sets the mechanical trail with the tyre radius. */
    casterDeg: 4.743,
    /**
     * Extra mechanical trail from the kingpin axis being ahead of the hub
     * centre, m. R tan(caster) at R = 0.2 m is 16.6 mm; OptimumK's 18.85 mm
     * of mechanical trail leaves 2.25 mm for the kingpin offset.
     */
    kingpinOffsetTrailM: 0.00225,
    /**
     * Fraction of the kingpin moment that reaches the rim. Rack and column
     * friction eat the rest. 0.85 is a plain rack with rod ends.
     */
    rackEfficiency: 0.85,
    /**
     * Ratio of rim torque to kingpin torque. Mechanically this is the inverse
     * of `steeringRatio` (rim angle per road-wheel angle), and it stays derived
     * from that unless the real rack says otherwise.
     */
    torqueRatio: null,
    /**
     * Steering-feel calibration: the fraction of the modelled tyre moment
     * that reaches the rim, on top of `rackEfficiency`. Feel only -- it never
     * touches the car's motion. 0.61 puts the model's steady-state rim
     * torque (1.0 g cornering) on the design report's autocross steer-force
     * targets (p.20, at the measured 0.1448 m grip radius); see `feel_scale`
     * in sim-core's vehicle.rs.
     */
    feelScale: 0.61,
    /** Scrub radius, m, and kingpin inclination, deg (same OptimumK export). */
    scrubM: 0.0255,
    kpiDeg: 8.745,
    /** Rim angle at the rack's measured stop, one side, deg. */
    rimLockDeg: 179,
    /**
     * TEAM: measured rim angle -> road-wheel angle, 0..180 deg of rim in 5 deg
     * steps (Drive `Steer_Force_Calculator/wheel_toe_angles.csv`, 2026-07-26;
     * both wheels' toe averaged into the axle angle, so static toe cancels).
     *
     * A real rack is PROGRESSIVE: this one takes 5.27 deg of rim per road
     * degree on centre and about 3.4 by 90 deg. The nominal 4.411 constant is
     * therefore 19% too quick where most of the driving happens, and stops
     * 18 deg of road wheel short of the real lock. The desktop rig steers
     * through this table and takes the rim/kingpin torque ratio from its local
     * slope; see `native/crates/sim-core/src/vehicle.rs`, which carries the
     * identical array.
     */
    rimToRoadDeg: [
    0.0000, 0.9486, 1.9018, 2.8643, 3.8407, 4.8355,
    5.8534, 6.8987, 7.9756, 9.0882, 10.2396, 11.4326,
    12.6689, 13.9491, 15.2722, 16.6359, 18.0360, 19.4671,
    20.9221, 22.3935, 23.8733, 25.3539, 26.8282, 28.2906,
    29.7370, 31.1647, 32.5728, 33.9618, 35.3334, 36.6907,
    38.0376, 39.3790, 40.7206, 42.0688, 43.4308, 44.8146,
    46.2288,
    ],
  },

  // Brakes, from the workbook BRAKES block and the Drive 'SDM26 Brakes
  // Calculator (Ideal Brake Bias)': Brembo P4.24 front (1809.6 mm^2 total
  // piston area, 78.7 mm effective radius), P2.24 rear (904.8 mm^2, 69.5 mm),
  // pad mu 0.45, Tilton 78-625 master cylinders, pedal ratio 3.5, bias bar
  // 54% front by force (OptimumK 'Brake Bias 54.0'; "54% fr" on the car,
  // 2026-04-11). See sim/tools/team_data.py.
  //
  // Max torque is the 70 bar max working pressure in the calculator: 1235
  // N.m at the wheels, which is 916 N (206 lbf) on the pedal. The pedal
  // force a driver actually reaches is not measured, so this is DERIVED, not
  // measured, but it is bounded by the team's own system limit rather than
  // the old 1500 N.m guess. All four still lock well before it (786 N.m at
  // 1.5 g), so threshold braking is a skill.
  brakeTorqueMaxNm: 1235,
  // TEAM/derived: front TORQUE share from the calliper geometry and the 54%
  // bias bar is 0.727 (the calculator's own sheet says 0.713 with slightly
  // different radii). The bias bar figure alone is a pressure split, not a
  // torque split; the front callipers are twice the rear.
  // 2026-09-21 (Nick): set to 0.65 front torque share. Through the same
  // calliper geometry that is a bias bar near 45% front, not the 54% on the
  // car on 2026-04-11.
  brakeBiasFront: 0.65,

  // TEAM: the differential. SDM26 runs a Drexler Formula Student V3, a 1.5-way
  // Salisbury (clutch-pack) LSD, in its default 40 deg drive / 50 deg coast
  // configuration.
  //
  // The team's April 2026 study ("The Differential Drexler Study") reduces the
  // wedge mechanics to `T_c = C |T_in| + B`, where T_c is the largest torque
  // DIFFERENCE the ramps and preload can hold across the two outputs. Its
  // central piece of advice is not to derive the internal geometry -- Drexler
  // does not publish the pin radius or the mean clutch radius -- but to
  // back-calculate C from the lock percentages in the manual and treat it as
  // one identified constant. Those are 30 deg -> 0.88, 40 -> 0.60, 45 -> 0.51,
  // 50 -> 0.42, 60 -> 0.29, read as eta = T_c / T_in, so C IS the lock
  // fraction of the fitted ramp. B is the breakaway preload, which Drexler
  // specifies in N.m wheel to wheel: 25-35 on the fixed unit, 0-75 adjustable.
  //
  // The same study warns that the manual's numbers are marketing-optimistic
  // and that measured on-track values run 60-80% of them; they are used as
  // quoted here because the AC mod is pinned to the same table, and all three
  // are exposed so they can be derated against real wheel-force data. Other
  // configurations the hardware allows: 30/45 -> 0.88/0.51, 45/60 -> 0.51/0.29,
  // and any of them reversed (50/40 -> 0.42/0.60 is the mild-power, strong-
  // coast setup the team's own tuning notes suggest for a rear-heavy car).
  // The double track's own setup and model (sim-core SuspensionParams; the
  // defaults are SuspensionParams::sdm26(), mirrored here for the setup
  // card). The JS bicycle does not read any of it; the rig applies it when
  // `vehicleModel` is 3.
  dt: {
    /** Static toe per wheel, deg, + = toe-in (spec sheet "- out, + in"). */
    toeInFrontDeg: 1.1,
    toeInRearDeg: 0.5,
    /** Static camber, deg, SAE (negative = top inboard). OptimumK actual. */
    staticCamberFrontDeg: -0.8,
    staticCamberRearDeg: -0.7,
    /** Fraction of true Ackermann, measured 18.5 %. */
    ackermann: 0.185,
    /** Bump steer, deg of toe-in per inch of bump, from the hardpoints. */
    bumpSteerFrontDegPerIn: 0.2139,
    bumpSteerRearDegPerIn: 0.0402,
    /** EST: road-wheel deg per 100 N.m about each kingpin. 0 = rigid. */
    steerComplianceDegPer100Nm: 0.5,
    /** Grip calibration: the bicycle's steer-ramp peak at skidpad speed
     *  (see double_track.rs, fourth pass). Mirrors vehicle.rs. */
    frontGripScale: 1.06,
    rearGripScale: 1.00,
    /** 1 = aero follows ride height (2026 CFD map), 0 = fixed split. */
    aeroRideMap: 1,
    /** Damping, fraction of critical at 50 mm/s (OVDR spec table). */
    dampingJounce: 0.70,
    dampingRebound: 0.80,
    /** Chassis torsional stiffness, N.m/deg (OVDR sec. 2, physical test). */
    chassisTorsionNmDeg: 960,
  },

  diff: {
    /** Lock fraction on the drive ramp. */
    powerLock: 0.60,
    /** Lock fraction on the coast ramp. Lower than power on a 1.5-way. */
    coastLock: 0.42,
    /** Breakaway preload, N.m wheel to wheel, as Drexler specifies it. */
    preloadNm: 25,
    /** Stick-band width, rad/s of wheel-speed difference. */
    stickRadS: 0.1,
  },

  // EST: tyre relaxation length -- the distance the tyre must roll to build
  // slip force. ~0.35 m is right for a 10" slick, and it is what makes the
  // car's response to a steering input transient rather than instantaneous.
  relaxLengthM: 0.35,

  // Attitude gradients for the camera and the visual body motion (no ride
  // DOF in the physics). rollGradientDegG is the Setup module's validated
  // with-tyre figure; the team's 'SDM26 Ride Roll Calc' sheet (Drive)
  // computes 0.602 deg/g with tyre for the same car. pitchGradientDegG is
  // TEAM (calculated, not measured): the same sheet's 'Pitch Gradient w/
  // Tire' 0.888 deg/g (0.596 springs only, which sim/tools/team_data.py
  // reproduces from the workbook springs and motion ratios). Braking dive is
  // ~13% less than that through OptimumK's anti-dive; camera only, so the
  // undiluted figure is used. Was 0.35 EST.
  rollGradientDegG: 0.595,
  pitchGradientDegG: 0.89,
  heaveMmG: 6,             // EST

  // EST: how much surface texture comes through the seat. 1.0 is the baseline
  // that felt right for a 267 kg car on a lot; 0 is a perfectly smooth world.
  // Purely a camera effect -- it does not touch the physics.
  vibrationScale: 0.2,

  // EST: driver eye point, relative to the CG (x forward, z up).
  // For the classic (procedural) car. 2026-09-23: 30 mm closer to the wheel
  // than it was, from the team driver ("slightly closer"). The team's CAD car
  // seats the driver against its own head restraint instead:
  // CAD_EYE_AHEAD_OF_CG_M.
  eyeAheadOfCgM: -0.12,
  // EST: eye height above the ground, settled by looking at the cockpit view
  // rather than by arithmetic alone. A reclined FSAE driver sits very low and
  // the geometry argues for 0.8 or so, but at that height the eye is over the
  // bodywork: the rim disappears behind the dash readout and the car is
  // cropped out of its own cockpit. At 0.70 the steering wheel, the roll hoop
  // and both front tyres frame the view the way a real onboard does, which is
  // what this number is actually for. The old 0.66 sat a little too deep.
  // 2026-09-23: +25 mm, the team driver's "slightly higher". Adjustable on
  // the setup card within EYE_HEIGHT_RANGE_M.
  eyeHeightM: 0.725,

  // EST: the driver's head slides forward under braking and back under
  // acceleration. Purely camera. There is deliberately NO sideways lean and
  // no eye-lead into corners (removed 2026-09-23: in the cockpit they read as
  // the camera wobbling), and the in-car views never roll with the body.
  headLongMPerG: 0.018,
};

// The rack map is a monotone cubic (PCHIP, Fritsch-Carlson) through the
// measured points rather than straight lines between them: the linear map's
// slope -- the local ratio, and so the rim/kingpin TORQUE ratio -- jumped by up
// to 3.6 % at every 5 deg of rim, notches on a direct-drive base. The cubic
// passes through every point, cannot overshoot between them, and its
// derivative is continuous. Port of `vehicle.rs`; the two must agree.

function steerSegSlope(t, i) {
  return (t[i + 1] - t[i]) / 5;
}

// Node slope: the first secant at the centre (the map is odd about zero), the
// last at the far end (so the curve joins the over-travel ramp without a
// kink), and the harmonic mean of the neighbouring secants in between.
function steerNodeSlope(t, k) {
  const n = t.length;
  if (k === 0) return steerSegSlope(t, 0);
  if (k >= n - 1) return steerSegSlope(t, n - 2);
  const d0 = steerSegSlope(t, k - 1), d1 = steerSegSlope(t, k);
  return d0 * d1 <= 0 ? 0 : 2 / (1 / d0 + 1 / d1);
}

// The cubic on segment `i` at local position `u` in 0..1: [road deg, slope].
function steerHermite(t, i, u) {
  const h = 5;
  const y0 = t[i], y1 = t[i + 1];
  const m0 = steerNodeSlope(t, i), m1 = steerNodeSlope(t, i + 1);
  const u2 = u * u, u3 = u * u * u;
  const y = (2 * u3 - 3 * u2 + 1) * y0
    + (u3 - 2 * u2 + u) * h * m0
    + (-2 * u3 + 3 * u2) * y1
    + (u3 - u2) * h * m1;
  const dy = ((6 * u2 - 6 * u) * y0
    + (3 * u2 - 4 * u + 1) * h * m0
    + (-6 * u2 + 6 * u) * y1
    + (3 * u2 - 2 * u) * h * m1) / h;
  return [y, dy];
}

/**
 * Road-wheel angle (deg, signed) for a rim angle, through the measured rack.
 *
 * Port of `road_from_rim_deg` in `native/crates/sim-core/src/vehicle.rs`; the
 * two must stay identical or the desktop and browser builds steer differently.
 */
export function roadFromRimDeg(steering, rimDeg) {
  const t = steering?.rimToRoadDeg;
  if (!t || t.length < 2) return rimDeg / (steering?.ratio ?? 4.411);
  const step = 5;
  const n = t.length;
  const sign = rimDeg < 0 ? -1 : 1;
  const mag = Math.abs(rimDeg);
  const x = mag / step;
  if (x >= n - 1) {
    // Past the table: hold the last slope, so over-travel is a ramp and not a
    // cliff. The end stop is what should be resisting by then.
    return sign * (t[n - 1] + steerSegSlope(t, n - 2) * (mag - (n - 1) * step));
  }
  const i = Math.floor(x);
  return sign * steerHermite(t, i, x - i)[0];
}

/**
 * Rim angle (deg, signed) for a road-wheel angle -- the inverse of
 * `roadFromRimDeg`, and the port of `rim_from_road_deg` in
 * `native/crates/sim-core/src/vehicle.rs`.
 *
 * It exists so the soft lock and the force-feedback end stop can be the same
 * place. The soft lock clamps the ROAD wheel at the car's live `maxSteerDeg`,
 * which is editable while driving; the end stop was pinned to the rack's
 * measured 179 deg stop. Those coincide only at the default 46 deg of lock --
 * set lock to 20 and the wheel had tens of degrees of travel that steered
 * nothing and resisted nothing.
 */
export function rimFromRoadDeg(steering, roadDeg) {
  const t = steering?.rimToRoadDeg;
  if (!t || t.length < 2) return roadDeg * (steering?.ratio ?? 4.411);
  const step = 5;
  const n = t.length;
  const sign = roadDeg < 0 ? -1 : 1;
  const mag = Math.abs(roadDeg);
  const last = t[n - 1];
  if (mag >= last) {
    const slope = steerSegSlope(t, n - 2);
    if (slope <= 0) return sign * (n - 1) * step;
    return sign * ((n - 1) * step + (mag - last) / slope);
  }
  // Monotonic table and a monotonic cubic through it: find the segment by its
  // end points, then bisect inside it -- sixty halvings, exactly as Rust does.
  for (let i = 0; i < n - 1; i++) {
    const a = t[i];
    const b = t[i + 1];
    if (mag <= b) {
      if (Math.abs(b - a) < 1e-12) return sign * i * step;
      let lo = 0, hi = 1;
      for (let k = 0; k < 60; k++) {
        const mid = 0.5 * (lo + hi);
        if (steerHermite(t, i, mid)[0] < mag) lo = mid; else hi = mid;
      }
      return sign * (i + 0.5 * (lo + hi)) * step;
    }
  }
  return sign * (n - 1) * step;
}

/**
 * Local d(road)/d(rim) at a rim angle -- the reciprocal of the local ratio.
 * The derivative of the same cubic, so it is continuous.
 */
export function roadPerRimDeg(steering, rimDeg) {
  const t = steering?.rimToRoadDeg;
  if (!t || t.length < 2) return 1 / (steering?.ratio ?? 4.411);
  const n = t.length;
  const x = Math.abs(rimDeg) / 5;
  if (x >= n - 1) return steerSegSlope(t, n - 2);
  const i = Math.floor(x);
  return steerHermite(t, i, x - i)[1];
}

/** Distance CG -> front axle (m). */
export function lengthToFrontAxle(v) {
  return v.wheelbaseM * (1 - v.weightDistFront);
}

/** Distance CG -> rear axle (m). */
export function lengthToRearAxle(v) {
  return v.wheelbaseM * v.weightDistFront;
}

/**
 * The tyre's reference load Fz0 (N) -- the load its mu is quoted at, and the
 * point load sensitivity is measured from.
 *
 * A property of the TYRE FIT, so a constant: SDM26's static corner load as
 * shipped (267 kg / 4). It used to be recomputed from the live mass, which
 * moved the tyre's reference with the car -- add 20 kg of ballast and the
 * tyre re-centred on the heavier load, and the car lost none of the grip load
 * sensitivity says it should (about 0.9 %). The argument is kept so callers
 * do not change; it is ignored. Rust: `MagicFormulaTyre::sdm26`, `nominal_load`.
 */
export const TYRE_FZ0_N = (267 * 9.81) / 4;
export function nominalTyreLoad(_v) {
  return TYRE_FZ0_N;
}

/**
 * Height of the SPRUNG mass's CG (m). `cgHeightM` is the whole car's; the
 * unsprung mass sits at the wheel centre, below it.
 */
export function sprungCgHeight(v) {
  const mu = 2 * (v.unsprungFrontKg + v.unsprungRearKg);
  return (v.massKg * v.cgHeightM - mu * v.tireRadiusM) / Math.max(v.massKg - mu, 1e-6);
}

/**
 * Sprung-CG to roll-axis arm (m), derived rather than stored. Port of
 * `VehicleParams::roll_arm`: it used to be a stored 0.2626 m -- the TOTAL CG
 * height less the roll axis, 4 % short of the sprung one -- and being stored it
 * ignored CG-height and roll-centre edits, so raising a roll centre added
 * transfer without taking any from the springs. Derived, elastic + geometric
 * + unsprung sums to m.ay.h/t whatever is edited.
 */
export function rollArm(v) {
  const a = v.wheelbaseM * (1 - v.weightDistFront);
  const axis = v.roll.rcFrontM + (v.roll.rcRearM - v.roll.rcFrontM) * a / v.wheelbaseM;
  return sprungCgHeight(v) - axis;
}

/** Total reduction engine -> wheel in gear index `g` (0-based). */
export function totalReduction(v, g) {
  const r = v.gearRatios[g];
  if (r == null || r <= 0) return 0;
  return v.primaryReduction * r * v.finalDrive;
}

/** Road speed (m/s) per engine rpm in gear `g`. */
export function gearVps(v, g) {
  const t = totalReduction(v, g);
  return t <= 0 ? 0 : (2 * Math.PI * v.tireRadiusM) / (60 * t);
}
