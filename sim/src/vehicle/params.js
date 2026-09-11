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
  // 1.66 x 0.90 = 1.49 at the front does it now that the front limits first.
  muLat: 1.66,
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
  // front drags at parallel steer, and steering compliance. 0.90 puts the front
  // at its peak while the rear still has ~5% of force in hand (utilisation
  // ~0.65 vs 0.93), which is a mild, recoverable push at 10-20 m/s and
  // survives a keyboard step to the speed-limited lock. Replace with a
  // measured understeer gradient when the team has one.
  frontGripFactor: 0.90,
  muLong: 1.5,            // launch-traction estimate (75 m accel ~4.2 s)
  tireLoadSensitivity: 0.15, // Hoosier R20 slick: mu falls 15% per 100% load

  // ---- aero (2026 CFD aero map @ nominal RH) ----
  cdaM2: 1.294,           // Cd 1.200 x A_ref 1.078 m^2
  claM2: 3.146,           // Cl 2.918 x 1.078 (downforce)
  // Front share of downforce. The 2026 CFD map says 55.3% at nominal ride
  // height; this model runs 50%, and the CFD figure is kept below for
  // traceability, the same way muLat is re-pinned against the lap sim.
  //
  // Why: driven through this model, 55.3% front on a 48.5%-front car makes
  // the rear the limiting axle from about 20 m/s up -- a steady steer ramp
  // spins it at 25 m/s, and it spins while COASTING at 20 m/s (rear axle
  // utilisation 0.98 at peak lateral with the engine braking it). That is a
  // car nobody could drive fast, and it is not what SDM26 does on track.
  // At 50% the front limits first at every speed (rear utilisation 0.70 to
  // 0.84 at peak lateral, 20-28 m/s). The aero group should say which of
  // the two the car actually runs at rake and ride height under load; until
  // then the drivable number is the honest one for a driver-in-loop tool.
  aeroFrontFrac: 0.50,
  aeroFrontFracCfd: 0.553,
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

  // EST: steering. 28 deg of road-wheel lock clears the tightest 4.5 m radius
  // on the endurance course with slip angle to spare. The lag and rate limit
  // stand in for the driver's hands plus rack compliance.
  maxSteerDeg: 28,
  steerLagS: 0.06,
  steerRateDegS: 360,
  // Steering-wheel turns per road-wheel angle. Visual only -- it sets how far
  // the wheel rotates in the cockpit, not how the car responds.
  steeringRatio: 4.0,

  // EST: steering geometry, for the force the driver feels. None of this
  // affects how the car goes round a corner; all of it sets what comes back
  // through the rim. Take caster and trail off the real uprights when known.
  steering: {
    /** Caster angle, deg. Sets the mechanical trail with the tyre radius. */
    casterDeg: 5.0,
    /**
     * Extra mechanical trail from the kingpin axis being ahead of the hub
     * centre, m. Zero when the kingpin passes through the hub.
     */
    kingpinOffsetTrailM: 0.0,
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

  // EST: brakes. 1500 N.m total at the wheels is enough to lock all four at
  // low speed (a rules requirement), so threshold braking is a skill.
  brakeTorqueMaxNm: 1500,
  brakeBiasFront: 0.62,

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
