// SDM26 vehicle parameters.
//
// Everything above the "DRIVING-SIM ADDITIONS" line is lifted verbatim from
// Helios (`modules/oracle/lib/performance/vehicle.ts` -> SDM26_VEHICLE and
// SDM26_ROLL), so this car is the same car the lap sim scores. Grip is the
// skidpad-pinned muLat 1.368 that reproduces SDM26's real 5.02 s skidpad, and
// the aero map is the 2026 CFD map at nominal ride height.
//
// Below that line are the states a *transient* model needs that a quasi-steady
// lap sim never had to name: rotational inertias, steering lock and rate, brake
// torque, relaxation length. Those are engineering estimates, each marked EST
// with its basis. They are the numbers to replace when the team measures them.

export const SDM26 = {
  name: "SDM26",

  // ---- mass & geometry (Helios) ----
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
  muLat: 1.72,
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
  frontGripFactor: 0.88,
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
  crr: 0.02,

  // ---- driveline (Helios; stock CBR600RR PC40 + SDM 3.0 final) ----
  drivetrainEff: 0.85,
  gearRatios: [2.75, 2.0, 1.667, 1.444, 1.304, 1.208],
  primaryReduction: 2.111, // 76/36
  finalDrive: 3.0,
  revLimitRpm: 14500,
  // Measured on the car (Daniel): the engine idles near 2000 rpm with the
  // throttle plate held at about 14%. Those two numbers are very nearly
  // self-consistent through the CFD torque curve and the friction model, which
  // put the zero-net-torque plate position at 14% somewhere around 2350 rpm --
  // agreement to a few hundred rpm, using nothing from the measurement itself.
  idleRpm: 2000,
  /** Throttle plate position the ETC holds at idle, 0..1. */
  idleThrottleFrac: 0.14,
  shiftTimeS: 0.1,

  // ---- roll balance (Helios SDM26_ROLL, from the team's ARB calculator) ----
  roll: {
    rsdFront: 0.512,   // roll-stiffness distribution, front share (no ARB baseline)
    hRollArmM: 0.2626, // sprung-CG to roll-axis arm
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

  // Steering lock: 28 deg at the road wheel. Originally an estimate (clears
  // the tightest 4.5 m radius on the endurance course with slip angle to
  // spare); the rack stops have not been measured, but three of the four
  // 2026-04-01 MoTeC runs that exercise the wheel to the stop cap the
  // STEERING channel at 121-124 deg at the rim, which through the measured
  // 4.411 ratio is 27.4-28.1 deg. The fourth run reads 196 deg and is either
  // uncalibrated or wrapped. Kept at 28; still not a rack measurement. The
  // lag and rate limit remain EST for the driver's hands plus rack
  // compliance.
  maxSteerDeg: 28,
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
  brakeBiasFront: 0.72,

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
  vibrationScale: 1.0,

  // EST: driver eye point, relative to the CG (x forward, z up).
  eyeAheadOfCgM: -0.15,
  eyeHeightM: 0.66,
};

/** Distance CG -> front axle (m). */
export function lengthToFrontAxle(v) {
  return v.wheelbaseM * (1 - v.weightDistFront);
}

/** Distance CG -> rear axle (m). */
export function lengthToRearAxle(v) {
  return v.wheelbaseM * v.weightDistFront;
}

/** Nominal per-tyre static load (N) -- the reference for load sensitivity. */
export function nominalTyreLoad(v) {
  return (v.massKg * 9.81) / 4;
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
