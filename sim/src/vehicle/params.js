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
  // against the 5.02 s SDM26 actually ran. 1.573 reproduces 5.02 s through
  // THIS model -- same measurement, different model, so a different constant.
  // (Oracle hit the same thing and solved it the same way, with mu_scale.)
  muLat: 1.573,
  muLatHeliosQss: 1.368,  // kept for traceability to the lap sim
  muLong: 1.5,            // launch-traction estimate (75 m accel ~4.2 s)
  tireLoadSensitivity: 0.15, // Hoosier R20 slick: mu falls 15% per 100% load

  // ---- aero (2026 CFD aero map @ nominal RH) ----
  cdaM2: 1.294,           // Cd 1.200 x A_ref 1.078 m^2
  claM2: 3.146,           // Cl 2.918 x 1.078 (downforce)
  aeroFrontFrac: 0.553,   // %front downforce
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
    // Team setup choice: 48% front. Deviates from Helios SDM26_ROLL, whose
    // 0.512 is the ARB calculator's with-tyre no-ARB baseline.
    rsdFront: 0.48,    // roll-stiffness distribution, front share
    hRollArmM: 0.2626, // sprung-CG to roll-axis arm
    rcFrontM: 0.0186,  // front roll-centre height
    rcRearM: 0.0251,   // rear roll-centre height
  },

  // =====================================================================
  // DRIVING-SIM ADDITIONS -- estimates, not team measurements.
  // =====================================================================

  // EST: yaw inertia. FSAE cars measured on a bifilar rig typically land
  // 85-130 kg.m^2. 105 gives a dynamic index k^2/(ab) = 0.67, which is the
  // mass-concentrated character of a small formula car (a road car is ~1.0).
  izzKgM2: 105,

  // EST: unsprung mass per corner (upright + hub + brake + wheel + tyre +
  // half the arms). Split front/rear because they are not the same corner:
  // the front carries the steering upright and the rear the driveshaft and
  // sprocket. Used to split lateral load transfer and to find sprung mass.
  unsprungFrontKg: 11,
  unsprungRearKg: 11,

  // EST: rotational inertias. Wheel assemblies from a 10" wheel + slick.
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
  wheelInertiaFrontKgM2: 0.22,
  wheelInertiaRearKgM2: 0.25,   // + sprocket and disc
  engineInertiaKgM2: 0.011,     // crank + primary drive gear (crank-referenced)
  gearboxInertiaKgM2: 0.006,    // clutch basket + shafts + sprocket (post-primary)

  // EST: steering. 28 deg of road-wheel lock clears the tightest 4.5 m radius
  // on the endurance course with slip angle to spare. The lag and rate limit
  // stand in for the driver's hands plus rack compliance.
  maxSteerDeg: 28,
  steerLagS: 0.06,
  steerRateDegS: 360,
  // Steering-wheel turns per road-wheel angle. Visual only -- it sets how far
  // the wheel rotates in the cockpit, not how the car responds.
  steeringRatio: 4.0,

  // EST: brakes. 1500 N.m total at the wheels is enough to lock all four at
  // low speed (a rules requirement), so threshold braking is a skill.
  brakeTorqueMaxNm: 1500,
  brakeBiasFront: 0.65,

  // EST: tyre relaxation length -- the distance the tyre must roll to build
  // slip force. ~0.35 m is right for a 10" slick, and it is what makes the
  // car's response to a steering input transient rather than instantaneous.
  relaxLengthM: 0.35,

  // EST: attitude gradients for the camera and the visual body motion.
  // rollGradientDegG is the Setup module's validated with-tyre figure.
  rollGradientDegG: 0.595,
  pitchGradientDegG: 0.35,
  heaveMmG: 6,

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
