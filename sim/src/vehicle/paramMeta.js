// Provenance metadata for the vehicle model.
//
// The single most useful thing this simulator can tell an engineer is which
// numbers it is standing on. Some come off the real SDM26 and out of Helios;
// some are outputs of the team's own CFD models; one is fitted so this model
// reproduces a measured result; and a good number are engineering estimates
// generated to build the simulator, which no one has measured. Treating those
// as equivalent is how a driving model quietly becomes a source of false
// confidence.
//
// Values are read live off SDM26 rather than retyped, so this sheet can never
// disagree with what the physics is actually running.

import { SDM26, EYE_HEIGHT_RANGE_M, lengthToFrontAxle, lengthToRearAxle, nominalTyreLoad, rollArm } from "./params.js";
import { TIRE_INFO } from "./tire.js";

export const PROVENANCE = {
  team: {
    label: "Team data",
    short: "TEAM",
    blurb: "Measured, specified or validated by Sun Devil Motorsports and carried in Helios.",
  },
  cfd: {
    label: "Team CFD model",
    short: "CFD",
    blurb: "Output of a Helios CFD model the team built -- not a hand measurement, but their own simulation.",
  },
  calibrated: {
    label: "Calibrated to a real run",
    short: "CAL",
    blurb: "Fitted so this model reproduces a measured SDM26 result. Anchored in real data, but specific to this model.",
  },
  estimate: {
    label: "Generated estimate",
    short: "EST",
    blurb: "An engineering estimate produced for this simulator. NOT team data -- the first thing to replace when measured.",
  },
};

/** What the model actually solves, and what it deliberately does not. */
export const MODEL = {
  name: "Transient bicycle model",
  integrator: "Semi-implicit Euler, fixed 500 Hz substeps, frame-rate independent",
  summary:
    "Planar rigid body with each axle lumped to one contact patch. Grip still " +
    "responds to left/right load transfer: the axle's inner and outer loads are " +
    "reconstructed from the roll-stiffness distribution and the load-sensitive " +
    "friction averaged across the pair, so the ARB setting still moves the balance.",

  dof: [
    {
      group: "Chassis (3)",
      items: [
        ["u", "Longitudinal velocity, body frame", "m/s"],
        ["v", "Lateral velocity, body frame", "m/s"],
        ["r", "Yaw rate", "rad/s"],
      ],
      note: "The lateral equation keeps the m.u.r term, so yaw response overshoots rather than settling straight onto a cornering balance.",
    },
    {
      group: "Wheel rotation (2)",
      items: [
        ["omega_f", "Front axle speed", "rad/s"],
        ["omega_r", "Rear axle speed", "rad/s"],
      ],
      note: "Slip ratio is therefore dynamic: the rears spin up and the fronts lock.",
    },
    {
      group: "Driveline (1)",
      items: [["omega_e", "Crankshaft speed", "rad/s"]],
      note: "A real clutch: locked or slipping against a torque capacity. Free when slipping, tied to omega_r when locked.",
    },
    {
      group: "Tyre relaxation (2)",
      items: [
        ["alpha_f", "Lagged front slip angle", "rad"],
        ["alpha_r", "Lagged rear slip angle", "rad"],
      ],
      note: "Force builds over a relaxation length rather than instantly, which is what makes the steering response transient at the tyre as well as the chassis.",
    },
    {
      group: "Steering actuator (1)",
      items: [["delta", "Road-wheel steer angle", "rad"]],
      note: "Rate-limited then first-order lagged, standing in for the driver's hands and rack compliance.",
    },
    {
      group: "Pose (3)",
      items: [
        ["X", "Global east position", "m"],
        ["Y", "Global north position", "m"],
        ["psi", "Heading", "rad"],
      ],
      note: "Integrated from the body-frame velocities; not dynamic states.",
    },
  ],

  discrete: [
    "Gear (1-6) and shift timer, with the driveline open and ignition cut during a shift",
    "Clutch lock/slip state",
    "Rev limiter latch, with hysteresis",
  ],

  notModelled: [
    ["Heave, pitch and roll as degrees of freedom",
     "They are applied as validated deg/g gradients for load transfer and camera attitude. No ride model, so kerb strikes and damper behaviour are absent."],
    ["Individual wheel vertical travel",
     "A bicycle model has two contact patches. Per-corner camber, toe and spring rate belong in Helios Setup and Oracle."],
    ["Tyre thermal and wear state", "mu is constant over a run."],
    ["Surface elevation and grip variation", "The venue is a flat lot, so this costs less here than it would elsewhere."],
  ],
};

const G = 9.81;
const n = (v, d = 3) => Number(v.toFixed(d));

/**
 * Every parameter the model runs on, grouped, with its provenance.
 * @returns {Array<{title:string, rows:Array<object>}>}
 */
export function parameterGroups() {
  const v = SDM26;
  const rows = (title, list) => ({ title, rows: list });

  return [
    rows("Vehicle model", [
      p("Vehicle model", v.vehicleModel ?? 2, "", "estimate",
        "2 = transient bicycle (validated). 3 = double track with a rolling, pitching body and camber from the team's kinematics and tyre fit (BETA, desktop rig only). Laps on 3 count on their own board in Helios (beside the bicycle), from simulator 0.7.2.",
        { path: "vehicleModel", min: 2, max: 3, step: 1 }),
    ]),
    rows("4-wheel β setup (model 3 only)", [
      p("Toe, front (per wheel)", v.dt.toeInFrontDeg, "deg", "team",
        "+ = toe-in, spec-sheet sign. As run: 1.1 in (Overall Vehicle DR 5.3, setup sheets).",
        { path: "dt.toeInFrontDeg", min: -2, max: 2, step: 0.05 }),
      p("Toe, rear (per wheel)", v.dt.toeInRearDeg, "deg", "team",
        "+ = toe-in. As run 0.5 in for autocross/endurance; the skidpad setup runs 0.7 out (-0.7). Rear toe-in is what keeps this model stable at speed.",
        { path: "dt.toeInRearDeg", min: -2, max: 2, step: 0.05 }),
      p("Static camber, front", v.dt.staticCamberFrontDeg, "deg", "team", "SAE, negative = top inboard. OptimumK actual.",
        { path: "dt.staticCamberFrontDeg", min: -4, max: 1, step: 0.05 }),
      p("Static camber, rear", v.dt.staticCamberRearDeg, "deg", "team", "SAE, negative = top inboard. OptimumK actual.",
        { path: "dt.staticCamberRearDeg", min: -4, max: 1, step: 0.05 }),
      p("Ackermann", v.dt.ackermann * 100, "%", "team",
        "Of true Ackermann. Measured 18.5 % from both road wheels against rim angle (OptimumK says 0, the spec sheet 85).",
        { path: "dt.ackermann", min: -50, max: 120, step: 1, factor: 100 }),
      p("Bump steer, front", v.dt.bumpSteerFrontDegPerIn, "deg/in", "team",
        "Toe-in per inch of bump, from the OptimumK hardpoints (small-displacement solve that reproduces OptimumK's camber gain to 1 %). Roll steer, and toe under pitch and aero squat, fall out of it.",
        { path: "dt.bumpSteerFrontDegPerIn", min: -1, max: 1, step: 0.005 }),
      p("Bump steer, rear", v.dt.bumpSteerRearDegPerIn, "deg/in", "team", "As the front.",
        { path: "dt.bumpSteerRearDegPerIn", min: -1, max: 1, step: 0.005 }),
      p("Steering compliance", v.dt.steerComplianceDegPer100Nm, "deg/100 N.m", "estimate",
        "Road-wheel degrees each front wheel gives way per 100 N.m about its kingpin. No SDM26 K&C measurement exists; 0.5 is a tight FSAE system, loose ones measure several times that. 0 = rigid.",
        { path: "dt.steerComplianceDegPer100Nm", min: 0, max: 5, step: 0.05 }),
      p("Front grip scale", v.dt.frontGripScale, "", "calibrated",
        "Front lateral grip x the front grip factor. Pinned so this model's steady-state limit on the skidpad circle equals the bicycle's (1.294 g at 8.6 m) while still pushing at 10/15/20 m/s.",
        // Max 1.25: the solver caps front grip at the tyre's own mu
        // (front grip factor 0.80 x scale <= 1, double_track.rs), so the
        // slider did nothing past 1/0.80.
        { path: "dt.frontGripScale", min: 0.8, max: 1.25, step: 0.005 }),
      p("Rear grip scale", v.dt.rearGripScale, "", "calibrated",
        "Rear lateral grip relative to the tyre's mu. 1.0: no correction needed once the per-load peak slip stopped extrapolating.",
        { path: "dt.rearGripScale", min: 0.8, max: 1.4, step: 0.005 }),
      p("Aero follows ride height", v.dt.aeroRideMap, "", "cfd",
        "1 = downforce, balance and drag move with ride height (2026 CFD ride-height map, plane fit; exactly nominal at static). 0 = the fixed nominal split.",
        { path: "dt.aeroRideMap", min: 0, max: 1, step: 1 }),
      p("Damping, jounce", v.dt.dampingJounce, "", "team",
        "Fraction of critical at 50 mm/s (Overall Vehicle Design Report spec table: 70 %). Heave and pitch run on the axle ride springs themselves; roll uses the mean of jounce and rebound.",
        { path: "dt.dampingJounce", min: 0.2, max: 1.5, step: 0.05 }),
      p("Damping, rebound", v.dt.dampingRebound, "", "team", "As jounce; the spec table gives 80 %.",
        { path: "dt.dampingRebound", min: 0.2, max: 1.5, step: 0.05 }),
      p("Chassis torsional stiffness", v.dt.chassisTorsionNmDeg, "N.m/deg", "team",
        "Physical test, 960 (target 1300, FEA 1482). In series between the axles' roll springs: it pulls the lateral load-transfer split toward the mass split, so front roll stiffness moves the balance less than a rigid frame says. Does not change total roll stiffness.",
        { path: "dt.chassisTorsionNmDeg", min: 200, max: 20000, step: 10 }),
    ]),
    rows("Mass & geometry", [
      p("Total mass", v.massKg, "kg", "team", "199 kg car (confirmed by Nick) + 68 kg driver.", { path: "massKg", min: 180, max: 400, step: 0.5 }),
      p("Front weight distribution", v.weightDistFront * 100, "%", "team", "With driver aboard.", { path: "weightDistFront", min: 38, max: 62, step: 0.1, factor: 100 }),
      p("CG height", v.cgHeightM * 1000, "mm", "team", "From the 2026 spec sheet.", { path: "cgHeightM", min: 180, max: 420, step: 1, factor: 1000 }),
      p("Wheelbase", v.wheelbaseM, "m", "team", "", { path: "wheelbaseM", min: 1.4, max: 1.85, step: 0.005 }),
      p("Track, front", v.trackFrontM, "m", "team", "", { path: "trackFrontM", min: 1.0, max: 1.45, step: 0.005 }),
      p("Track, rear", v.trackRearM, "m", "team", "", { path: "trackRearM", min: 1.0, max: 1.45, step: 0.005 }),
      p("CG -> front axle (a)", n(lengthToFrontAxle(v)), "m", "team", "Derived from wheelbase and weight distribution."),
      p("CG -> rear axle (b)", n(lengthToRearAxle(v)), "m", "team", "Derived from wheelbase and weight distribution."),
      p("Yaw inertia Izz", v.izzKgM2, "kg.m^2", "team",
        "Team 'Full-Vehicle Sim Parameters' workbook, BODY block (93.66 at 253.3 kg listed). Not a bifilar measurement -- reads like CAD mass properties -- but the team's own number, inside the 85-130 a rig gives FSAE cars. Dynamic index 0.60.", { path: "izzKgM2", min: 50, max: 220, step: 1 }),
      p("Unsprung mass, front corner", v.unsprungFrontKg, "kg", "team",
        "Team workbook (7.56 kg); the Ride Roll Calc sheet says 7.5 and the quarter-car script 8.3. Sets sprung mass and the unsprung share of lateral load transfer.",
        { path: "unsprungFrontKg", min: 4, max: 22, step: 0.1 }),
      p("Unsprung mass, rear corner", v.unsprungRearKg, "kg", "team",
        "Team workbook (7.77 kg); the Ride Roll Calc sheet says 8.6.",
        { path: "unsprungRearKg", min: 4, max: 22, step: 0.1 }),
    ]),

    rows("Tyres & grip", [
      p("Peak lateral mu (rear axle)", v.muLat, "", "calibrated",
        "With the front grip factor below, reproduces SDM26's real 5.02 s skidpad THROUGH THIS MODEL. Differs from the lap sim's 1.368 because that figure already absorbs the axle load-transfer derate this model computes explicitly; reusing it would double-count and give 5.38 s."),
      p("Front grip factor", v.frontGripFactor, "", "estimate",
        "Front axle peak lateral grip relative to the rear. With one tyre character on both axles the balance is neutral to within 1% of force and the car spins from any step steer at the limit; a real front lets go first (camber loss on the steered upright, inside-front drag, compliance). 0.88 gives a mild, recoverable push at every speed with the 2026 CFD aero split. The one knob standing in for the unmodelled front end; replace with a measured understeer gradient.",
        { path: "frontGripFactor", min: 0.75, max: 1.05, step: 0.005 }),
      p("Lap sim's peak lateral mu", v.muLatHeliosQss, "", "team",
        "Helios' skidpad-pinned value, kept for traceability. Correct for a quasi-steady model, wrong for this one."),
      p("Peak longitudinal mu", v.muLong, "", "calibrated",
        "Helios' launch-traction figure, anchored so the 75 m accel lands near the real ~4.2 s."),
      p("Load sensitivity", v.tireLoadSensitivity, "", "team",
        "From the team's PAC2002 TTC fit of the R20: PDY2/PDY1 = 12.1% of peak lateral mu lost per 100% of load (longitudinal 10.5%). Was Helios' 0.15 estimate."),
      p("Pneumatic trail at 700 N", TIRE_INFO.pneumaticTrailM * 1000, "mm", "team",
        "Fitted to the raw TTC Round 9 Mz channel for this tyre at 12 psi (sim/tools/ttc_trail.py); zero by 15 deg of slip, square root of load."),
      p("Nominal tyre load (Fz0)", Math.round(nominalTyreLoad(v)), "N", "team", "The tyre's reference load for load sensitivity: SDM26's static corner load as shipped. A tyre property, so it does not move with a mass edit."),
      p("Loaded radius", v.tireRadiusM, "m", "team", "Hoosier 16x7.5-10."),
      p("Peak slip angle", n(TIRE_INFO.peakSlipAngleDeg, 1), "deg", "estimate",
        "Model shape choice. The team's TTC fit of this tyre peaks at 12.4-12.9 deg (222-1112 N) and Oracle's MF6.1.2 at 13-16 deg, which is fine for a peak-grip lap sim but makes steering feel vague to drive."),
      p("Peak slip ratio", TIRE_INFO.peakSlipRatio, "", "estimate", "Model shape choice."),
      p("Cornering stiffness / tyre", Math.round(TIRE_INFO.corneringStiffness(v.muLat, nominalTyreLoad(v)) / 57.3), "N/deg", "estimate",
        "Falls out of the fitted Magic Formula shape at static load."),
      p("Tyre relaxation length", v.relaxLengthM, "m", "estimate",
        "Distance the tyre must roll to build slip force. ~0.35 m suits a 10 in slick."),
    ]),

    rows("Aerodynamics", [
      p("CdA", v.cdaM2, "m^2", "cfd", "2026 full-car CFD ride-height map at nominal ride height: 42.72 lbf of drag at 15.65 m/s.", { path: "cdaM2", min: 0.4, max: 2.6, step: 0.005 }),
      p("ClA", v.claM2, "m^2", "cfd", "2026 full-car CFD ride-height map at nominal ride height: 105.64 lbf of downforce at 15.65 m/s. The Aero Design Binder's Cl 3.064 x 1.0224 m^2 agrees.", { path: "claM2", min: 0, max: 5.5, step: 0.005 }),
      p("Aero balance (front downforce share)", v.aeroFrontFrac * 100, "%", "cfd", "What the wings ARE, not a setup knob: nothing on SDM26 changes this between two runs, so it is here rather than on the setup card, and moving it stops a lap counting as a time. 2026 CFD map at nominal ride height (both the ride-height and pitch sweeps give 52.42%). Read where the car sits on its measured springs it is 52% at 10-15 m/s and 54-57% at 20-30 m/s, at the coarse edge of the map. The 55.3% previously quoted was the 2025 half-car sheet.", { path: "aeroFrontFrac", min: 25, max: 75, step: 0.1, factor: 100 }),
      p("Air density", v.airDensityKgM3, "kg/m^3", "team", "Ambient used across Helios."),
      p("Rolling resistance", v.crr, "", "team", "Helios model constant."),
    ]),

    rows("Roll balance", [
      p("Roll stiffness distribution, front", v.roll.rsdFront * 100, "%", "team",
        "Team setup choice, between the measured blade settings: front 1-1 / rear 1-1 is 46%, front 4-7 / rear 1-1 is 51%.", { path: "roll.rsdFront", min: 30, max: 70, step: 0.1, factor: 100 }),
      p("CG to roll-axis arm", rollArm(v) * 1000, "mm", "team",
        "Derived: sprung-CG height less the roll axis under it, so it follows CG-height and roll-centre edits. Was a stored 262.6 mm (the SDM25 RSD sheet's 10.34 in), which used the total CG height."),
      p("Roll centre, front", v.roll.rcFrontM * 1000, "mm", "team", "", { path: "roll.rcFrontM", min: -60, max: 160, step: 0.5, factor: 1000 }),
      p("Roll centre, rear", v.roll.rcRearM * 1000, "mm", "team", "", { path: "roll.rcRearM", min: -60, max: 160, step: 0.5, factor: 1000 }),
      p("Roll gradient", v.rollGradientDegG, "deg/g", "team", "Validated with-tyre figure from the Helios Setup module.", { path: "rollGradientDegG", min: 0, max: 3, step: 0.005 }),
      p("Pitch gradient", v.pitchGradientDegG, "deg/g", "team", "Team 'SDM26 Ride Roll Calc' sheet, with tyre (0.596 springs only, reproduced from the workbook springs and motion ratios). Calculated, not measured. Camera attitude only.", { path: "pitchGradientDegG", min: 0, max: 2, step: 0.005 }),
      p("Heave", v.heaveMmG, "mm/g", "estimate", "Camera motion only."),
    ]),

    rows("Powertrain", [
      p("Engine", "Honda CBR600RR (PC40), 20 mm restricted", "", "team"),
      p("Torque curve", "23-point RPM sweep, 4 000-15 000", "", "cfd",
        "Helios CFD engine-sim, 1-D finite-volume solver, characteristic junctions. Wave-action features are preserved, not smoothed."),
      p("Engine braking", "From the sweep's own fmep", "", "cfd", "T = fmep.Vd/4pi; about 12 N.m of overrun drag at 10 000 rpm."),
      p("Gear ratios", v.gearRatios.join(" / "), "", "team", "Stock CBR600RR (PC40)."),
      p("Primary reduction", v.primaryReduction, "", "team", "76/36."),
      p("Final drive", v.finalDrive, "", "team", "SDM sprocket choice.", { path: "finalDrive", min: 2.5, max: 4, step: 0.01 }),
      p("Rev limit", v.revLimitRpm, "rpm", "team", "Confirmed by Nick."),
      p("Rev limiter hysteresis", v.revLimitHystRpm, "rpm", "estimate", "Hard ignition cut at the limit, back on this far under it. Tuned to the drivers' 'bounces a little'; the ECU's control range would replace it."),
      p("Launch control", v.launchRpm, "rpm", "team", "Where the car's LC is set.", { path: "launchRpm", min: 4000, max: 12000, step: 100 }),
      p("Launch control hysteresis", v.launchHystRpm, "rpm", "estimate", "Wider than the main limiter's: the real LC bounces hard."),
      p("Shift time", v.shiftTimeS * 1000, "ms", "team", "The ignition cut: 80-100 ms off the real paddle shift, per Nick."),
      p("Shift torque blend", v.shiftReintroS * 1000, "ms", "estimate", "How long the torque takes to come back after the cut, on a smoothstep. A step was a kick through the driveline on every shift."),
      p("Drivetrain efficiency", v.drivetrainEff, "", "team"),
      p("Idle speed", v.idleRpm, "rpm", "team", "Measured on the car. Helios has no idle at all."),
      p("Idle throttle plate", v.idleThrottleFrac * 100, "%", "team",
        "The opening the ETC holds at idle. Cross-checks against the torque curve: 14% is the zero-net-torque plate position at about 2350 rpm."),
      p("Crank + primary inertia", v.engineInertiaKgM2, "kg.m^2", "estimate",
        "Crank-referenced. Through 1st (17.4:1) this is the dominant part of the ~+94 kg apparent mass the quasi-steady lap sim ignores entirely."),
      p("Gearbox + sprocket inertia", v.gearboxInertiaKgM2, "kg.m^2", "estimate",
        "Referenced after the primary, because that is where the clutch physically sits on a CBR600RR."),
      p("Wheel inertia, front axle", v.wheelInertiaFrontKgM2, "kg.m^2", "team", "Per wheel, team workbook. The brakes calculator's build-up (tyre + rim + rotor + spindle) gives 0.20.", { path: "wheelInertiaFrontKgM2", min: 0.05, max: 0.7, step: 0.005 }),
      p("Wheel inertia, rear axle", v.wheelInertiaRearKgM2, "kg.m^2", "team", "Per wheel, team workbook. The rear disc is inboard, so it is in the driveline figure, not here.", { path: "wheelInertiaRearKgM2", min: 0.05, max: 0.7, step: 0.005 }),
    ]),

    rows("Steering & brakes", [
      p("Steering lock, road wheel", v.maxSteerDeg, "deg", "team",
        "The rack's measured limit: the Drive toe-vs-steering-wheel table reaches 46 deg of road wheel at 179 deg of rim. Replaces a 28 deg estimate that was really where the driver stopped turning in a MoTeC run.", { path: "maxSteerDeg", min: 10, max: 55, step: 0.5 }),
      p("Steering ratio", v.steeringRatio, ":1", "team",
        "OptimumK 'Designed vs Actual Kinematics' export, 2026-06-27. Sets the cockpit wheel angle and the rim torque per kingpin torque.",
        { path: "steeringRatio", min: 2, max: 10, step: 0.1 }),
      p("Caster", v.steering.casterDeg, "deg", "team", "OptimumK export; KPI 8.745 deg, scrub 25.5 mm, mechanical trail 18.85 mm (kingpin offset adds 2.25 mm to R tan(caster))."),
      p("Steering lag", v.steerLagS * 1000, "ms", "estimate", "Driver's hands plus rack compliance.", { path: "steerLagS", min: 10, max: 300, step: 1, factor: 1000 }),
      p("Steering rate limit", v.steerRateDegS, "deg/s", "estimate", "At the road wheel."),
      p("Max brake torque", v.brakeTorqueMaxNm, "N.m", "estimate",
        "Derived, not measured: the brakes calculator's 70 bar max working pressure through the P4.24/P2.24 callipers, pad mu 0.45 and the 54% bias bar. That is 206 lbf on the pedal; the force a driver reaches is the unmeasured part. All four lock at 786 N.m (1.5 g)."),
      p("Brake bias, front", v.brakeBiasFront * 100, "%", "team",
        "Team setup choice, as a torque share. The 54% bias bar measured on the car gives 72% through the calliper geometry (calculator: 71.3%, workbook radii: 72.7%); 65% is a bar near 45%. The bias bar figure is a pressure split.", { path: "brakeBiasFront", min: 45, max: 75, step: 0.1, factor: 100 }),
    ]),

    rows("Driver & environment", [
      p("Diff lock, on throttle", v.diff.powerLock, "", "team",
        "The DRIVE ramp: how hard the two rear wheels are tied together under power. Drexler V3, from the lock table in the Formula Student LSD manual: 30 deg is 0.88, 40 is 0.60, 45 is 0.51, 50 is 0.42, 60 is 0.29. The car ships on 40/50. The team's study notes the manual is optimistic and on-track values run 60-80% of it.", { path: "diff.powerLock", min: 0, max: 0.95, step: 0.01 }),
      p("Diff lock, off throttle", v.diff.coastLock, "", "team",
        "The COAST ramp: the same, off the throttle and under braking. Same table. Lower than the power ramp on a 1.5-way. This is the number that steadies the rear on a lift.", { path: "diff.coastLock", min: 0, max: 0.95, step: 0.01 }),
      p("Diff preload", v.diff.preloadNm, "N.m", "team",
        "Breakaway torque wheel to wheel, as Drexler specifies it: 25-35 on the fixed unit, 0-75 adjustable. Raising it adds understeer on entry and kills lock-up lag on exit; lowering it lets the rear rotate more freely off throttle.", { path: "diff.preloadNm", min: 0, max: 75, step: 1 }),
      p("Eye height", v.eyeHeightM, "m", "estimate", "Cockpit camera.", { path: "eyeHeightM", min: EYE_HEIGHT_RANGE_M[0], max: EYE_HEIGHT_RANGE_M[1], step: 0.005 }),
      p("Eye position vs CG", v.eyeAheadOfCgM, "m", "estimate", "Negative is behind the CG."),
      p("Car vibration", v.vibrationScale * 100, "%", "estimate",
        "How much surface texture comes through the seat. Camera only -- it does not touch the physics. 0% is a perfectly smooth world.",
        { path: "vibrationScale", min: 0, max: 250, step: 5, factor: 100 }),
      p("Gravity", G, "m/s^2", "team"),
    ]),
  ];
}

/**
 * @param edit  optional {path, min, max, step, factor} making the row live.
 *   `path`   dotted path into SDM26, e.g. "roll.rcFrontM"
 *   `factor` stored -> displayed (CG height is stored in m, shown in mm)
 *   min/max/step are in DISPLAY units, so the slider reads like the label.
 */
function p(label, value, unit, prov, note, edit) {
  const row = { label, value: format(value), unit, prov, note: note ?? "" };
  if (edit) row.edit = { factor: 1, ...edit, raw: Number(value) };
  return row;
}

/** Read a dotted path off the live parameter object. */
export function readParam(path) {
  return path.split(".").reduce((o, k) => o?.[k], SDM26);
}

/** Write a dotted path on the live parameter object, in STORED units. */
export function writeParam(path, stored) {
  const keys = path.split(".");
  const last = keys.pop();
  const target = keys.reduce((o, k) => o[k], SDM26);
  target[last] = stored;
}

/**
 * EVERY number in the car, flattened to dotted paths -- not just the ones
 * with a slider.
 *
 * The run recorder used to snapshot the spec sheet's editable rows, which is
 * 31 of the model's 68 numbers. The other 37 include the ones that would
 * actually be worth cheating with: peak grip, the gear ratios, driveline
 * efficiency, the rev limit, brake torque, tyre radius. A run driven on a
 * locally edited build looked identical in the log to an honest one.
 *
 * Arrays of numbers (gear ratios, the rack's toe table) are joined rather
 * than expanded: they are still compared exactly, and a 60-entry table does
 * not belong in a manifest as 60 keys.
 */
export function flattenParams(obj = SDM26, prefix = "", out = {}) {
  for (const [k, v] of Object.entries(obj)) {
    const path = prefix ? `${prefix}.${k}` : k;
    if (typeof v === "number") out[path] = v;
    else if (Array.isArray(v) && v.every((x) => typeof x === "number")) out[path] = v.join(",");
    else if (v && typeof v === "object" && !Array.isArray(v)) flattenParams(v, path, out);
  }
  return out;
}

/**
 * The whole car as it shipped, captured at import -- before `loadParams()`
 * restores anything a driver changed, which is the only moment this is
 * knowable at runtime.
 */
export const AS_SHIPPED = Object.freeze(flattenParams());

/** Every parameter path in the model, sliders or not. */
export const ALL_PARAM_PATHS = Object.keys(AS_SHIPPED);

/** The as-shipped value of every editable parameter, for Reset. */
export const PARAM_DEFAULTS = (() => {
  const out = {};
  for (const g of parameterGroups()) {
    for (const r of g.rows) if (r.edit) out[r.edit.path] = readParam(r.edit.path);
  }
  return out;
})();

function format(value) {
  if (typeof value !== "number") return String(value);
  if (Number.isInteger(value)) return String(value);
  const abs = Math.abs(value);
  if (abs >= 100) return value.toFixed(1);
  if (abs >= 10) return value.toFixed(2);
  if (abs >= 1) return value.toFixed(3).replace(/0+$/, "").replace(/\.$/, "");
  return value.toFixed(4).replace(/0+$/, "").replace(/\.$/, "");
}

/** Counts by provenance, for the summary line. */
export function provenanceTally() {
  const tally = { team: 0, cfd: 0, calibrated: 0, estimate: 0 };
  let total = 0;
  for (const group of parameterGroups()) {
    for (const row of group.rows) { tally[row.prov]++; total++; }
  }
  return { tally, total };
}
