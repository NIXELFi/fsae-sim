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

import { SDM26, lengthToFrontAxle, lengthToRearAxle, nominalTyreLoad } from "./params.js";
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
    blurb: "Output of a Helios CFD model the team built — not a hand measurement, but their own simulation.",
  },
  calibrated: {
    label: "Calibrated to a real run",
    short: "CAL",
    blurb: "Fitted so this model reproduces a measured SDM26 result. Anchored in real data, but specific to this model.",
  },
  estimate: {
    label: "Generated estimate",
    short: "EST",
    blurb: "An engineering estimate produced for this simulator. NOT team data — the first thing to replace when measured.",
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
      note: "The lateral equation keeps the m·u·r term, so yaw response overshoots rather than settling straight onto a cornering balance.",
    },
    {
      group: "Wheel rotation (2)",
      items: [
        ["ω_f", "Front axle speed", "rad/s"],
        ["ω_r", "Rear axle speed", "rad/s"],
      ],
      note: "Slip ratio is therefore dynamic: the rears spin up and the fronts lock.",
    },
    {
      group: "Driveline (1)",
      items: [["ω_e", "Crankshaft speed", "rad/s"]],
      note: "A real clutch: locked or slipping against a torque capacity. Free when slipping, tied to ω_r when locked.",
    },
    {
      group: "Tyre relaxation (2)",
      items: [
        ["α_f", "Lagged front slip angle", "rad"],
        ["α_r", "Lagged rear slip angle", "rad"],
      ],
      note: "Force builds over a relaxation length rather than instantly, which is what makes the steering response transient at the tyre as well as the chassis.",
    },
    {
      group: "Steering actuator (1)",
      items: [["δ", "Road-wheel steer angle", "rad"]],
      note: "Rate-limited then first-order lagged, standing in for the driver's hands and rack compliance.",
    },
    {
      group: "Pose (3)",
      items: [
        ["X", "Global east position", "m"],
        ["Y", "Global north position", "m"],
        ["ψ", "Heading", "rad"],
      ],
      note: "Integrated from the body-frame velocities; not dynamic states.",
    },
  ],

  discrete: [
    "Gear (1–6) and shift timer, with the driveline open and ignition cut during a shift",
    "Clutch lock/slip state",
    "Rev limiter latch, with hysteresis",
  ],

  notModelled: [
    ["Heave, pitch and roll as degrees of freedom",
     "They are applied as validated deg/g gradients for load transfer and camera attitude. No ride model, so kerb strikes and damper behaviour are absent."],
    ["Individual wheel vertical travel",
     "A bicycle model has two contact patches. Per-corner camber, toe and spring rate belong in Helios Setup and Oracle."],
    ["Differential",
     "The rear axle is a single lumped wheel; no torque split or locking effect."],
    ["Tyre thermal and wear state", "μ is constant over a run."],
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
    rows("Mass & geometry", [
      p("Total mass", v.massKg, "kg", "team", "199 kg car (confirmed by Nick) + 68 kg driver.", { path: "massKg", min: 180, max: 400, step: 0.5 }),
      p("Front weight distribution", v.weightDistFront * 100, "%", "team", "With driver aboard.", { path: "weightDistFront", min: 38, max: 62, step: 0.1, factor: 100 }),
      p("CG height", v.cgHeightM * 1000, "mm", "team", "From the 2026 spec sheet.", { path: "cgHeightM", min: 180, max: 420, step: 1, factor: 1000 }),
      p("Wheelbase", v.wheelbaseM, "m", "team", "", { path: "wheelbaseM", min: 1.4, max: 1.85, step: 0.005 }),
      p("Track, front", v.trackFrontM, "m", "team", "", { path: "trackFrontM", min: 1.0, max: 1.45, step: 0.005 }),
      p("Track, rear", v.trackRearM, "m", "team", "", { path: "trackRearM", min: 1.0, max: 1.45, step: 0.005 }),
      p("CG → front axle (a)", n(lengthToFrontAxle(v)), "m", "team", "Derived from wheelbase and weight distribution."),
      p("CG → rear axle (b)", n(lengthToRearAxle(v)), "m", "team", "Derived from wheelbase and weight distribution."),
      p("Yaw inertia Izz", v.izzKgM2, "kg·m²", "team",
        "Team 'Full-Vehicle Sim Parameters' workbook, BODY block (93.66 at 253.3 kg listed). Not a bifilar measurement — reads like CAD mass properties — but the team's own number, inside the 85–130 a rig gives FSAE cars. Dynamic index 0.60.", { path: "izzKgM2", min: 50, max: 220, step: 1 }),
      p("Unsprung mass, front corner", v.unsprungFrontKg, "kg", "team",
        "Team workbook (7.56 kg); the Ride Roll Calc sheet says 7.5 and the quarter-car script 8.3. Sets sprung mass and the unsprung share of lateral load transfer.",
        { path: "unsprungFrontKg", min: 4, max: 22, step: 0.1 }),
      p("Unsprung mass, rear corner", v.unsprungRearKg, "kg", "team",
        "Team workbook (7.77 kg); the Ride Roll Calc sheet says 8.6.",
        { path: "unsprungRearKg", min: 4, max: 22, step: 0.1 }),
    ]),

    rows("Tyres & grip", [
      p("Peak lateral μ (rear axle)", v.muLat, "—", "calibrated",
        "With the front grip factor below, reproduces SDM26's real 5.02 s skidpad THROUGH THIS MODEL. Differs from the lap sim's 1.368 because that figure already absorbs the axle load-transfer derate this model computes explicitly; reusing it would double-count and give 5.38 s."),
      p("Front grip factor", v.frontGripFactor, "—", "estimate",
        "Front axle peak lateral grip relative to the rear. With one tyre character on both axles the balance is neutral to within 1% of force and the car spins from any step steer at the limit; a real front lets go first (camber loss on the steered upright, inside-front drag, compliance). 0.88 gives a mild, recoverable push at every speed with the 2026 CFD aero split. The one knob standing in for the unmodelled front end; replace with a measured understeer gradient.",
        { path: "frontGripFactor", min: 0.75, max: 1.05, step: 0.005 }),
      p("Lap sim's peak lateral μ", v.muLatHeliosQss, "—", "team",
        "Helios' skidpad-pinned value, kept for traceability. Correct for a quasi-steady model, wrong for this one."),
      p("Peak longitudinal μ", v.muLong, "—", "calibrated",
        "Helios' launch-traction figure, anchored so the 75 m accel lands near the real ~4.2 s."),
      p("Load sensitivity", v.tireLoadSensitivity, "—", "team",
        "From the team's PAC2002 TTC fit of the R20: PDY2/PDY1 = 12.1% of peak lateral μ lost per 100% of load (longitudinal 10.5%). Was Helios' 0.15 estimate."),
      p("Pneumatic trail at 700 N", TIRE_INFO.pneumaticTrailM * 1000, "mm", "team",
        "Fitted to the raw TTC Round 9 Mz channel for this tyre at 12 psi (sim/tools/ttc_trail.py); zero by 15 deg of slip, square root of load."),
      p("Nominal tyre load (Fz0)", Math.round(nominalTyreLoad(v)), "N", "team", "Static corner load, the reference for load sensitivity."),
      p("Loaded radius", v.tireRadiusM, "m", "team", "Hoosier 16x7.5-10."),
      p("Peak slip angle", n(TIRE_INFO.peakSlipAngleDeg, 1), "deg", "estimate",
        "Model shape choice. The real MF6.1.2 fit in Oracle does not peak until 13–16°, which is fine for a peak-grip lap sim but makes steering feel vague to drive."),
      p("Peak slip ratio", TIRE_INFO.peakSlipRatio, "—", "estimate", "Model shape choice."),
      p("Cornering stiffness / tyre", Math.round(TIRE_INFO.corneringStiffness(v.muLat, nominalTyreLoad(v)) / 57.3), "N/deg", "estimate",
        "Falls out of the fitted Magic Formula shape at static load."),
      p("Tyre relaxation length", v.relaxLengthM, "m", "estimate",
        "Distance the tyre must roll to build slip force. ~0.35 m suits a 10 in slick."),
    ]),

    rows("Aerodynamics", [
      p("CdA", v.cdaM2, "m²", "cfd", "2026 full-car CFD ride-height map at nominal ride height: 42.72 lbf of drag at 15.65 m/s.", { path: "cdaM2", min: 0.4, max: 2.6, step: 0.005 }),
      p("ClA", v.claM2, "m²", "cfd", "2026 full-car CFD ride-height map at nominal ride height: 105.64 lbf of downforce at 15.65 m/s. The Aero Design Binder's Cl 3.064 × 1.0224 m² agrees.", { path: "claM2", min: 0, max: 5.5, step: 0.005 }),
      p("Front downforce split", v.aeroFrontFrac * 100, "%", "cfd", "2026 CFD map at nominal ride height (both the ride-height and pitch sweeps give 52.42%). Read where the car sits on its measured springs it is 52% at 10-15 m/s and 54-57% at 20-30 m/s, at the coarse edge of the map. The 55.3% previously quoted was the 2025 half-car sheet.", { path: "aeroFrontFrac", min: 25, max: 75, step: 0.1, factor: 100 }),
      p("Air density", v.airDensityKgM3, "kg/m³", "team", "Ambient used across Helios."),
      p("Rolling resistance", v.crr, "—", "team", "Helios model constant."),
    ]),

    rows("Roll balance", [
      p("Roll stiffness distribution, front", v.roll.rsdFront * 100, "%", "team",
        "From the team's 2026 Anti-Roll Bar Calculator; the with-tyre no-ARB baseline."),
      p("CG to roll-axis arm", v.roll.hRollArmM * 1000, "mm", "team",
        "Matches the SDM25 RSD test sheet's measured 10.34 in."),
      p("Roll centre, front", v.roll.rcFrontM * 1000, "mm", "team", "", { path: "roll.rcFrontM", min: -60, max: 160, step: 0.5, factor: 1000 }),
      p("Roll centre, rear", v.roll.rcRearM * 1000, "mm", "team", "", { path: "roll.rcRearM", min: -60, max: 160, step: 0.5, factor: 1000 }),
      p("Roll gradient", v.rollGradientDegG, "deg/g", "team", "Validated with-tyre figure from the Helios Setup module.", { path: "rollGradientDegG", min: 0, max: 3, step: 0.005 }),
      p("Pitch gradient", v.pitchGradientDegG, "deg/g", "team", "Team 'SDM26 Ride Roll Calc' sheet, with tyre (0.596 springs only, reproduced from the workbook springs and motion ratios). Calculated, not measured. Camera attitude only.", { path: "pitchGradientDegG", min: 0, max: 2, step: 0.005 }),
      p("Heave", v.heaveMmG, "mm/g", "estimate", "Camera motion only."),
    ]),

    rows("Powertrain", [
      p("Engine", "Honda CBR600RR (PC40), 20 mm restricted", "", "team"),
      p("Torque curve", "23-point RPM sweep, 4 000–15 000", "", "cfd",
        "Helios CFD engine-sim, 1-D finite-volume solver, characteristic junctions. Wave-action features are preserved, not smoothed."),
      p("Engine braking", "From the sweep's own fmep", "", "cfd", "T = fmep·Vd/4π; about 12 N·m of overrun drag at 10 000 rpm."),
      p("Gear ratios", v.gearRatios.join(" / "), "", "team", "Stock CBR600RR (PC40)."),
      p("Primary reduction", v.primaryReduction, "—", "team", "76/36."),
      p("Final drive", v.finalDrive, "—", "team", "SDM sprocket choice."),
      p("Rev limit", v.revLimitRpm, "rpm", "team", "Confirmed by Nick."),
      p("Shift time", v.shiftTimeS * 1000, "ms", "team", "Confirmed by Nick."),
      p("Drivetrain efficiency", v.drivetrainEff, "—", "team"),
      p("Idle speed", v.idleRpm, "rpm", "team", "Measured on the car. Helios has no idle at all."),
      p("Idle throttle plate", v.idleThrottleFrac * 100, "%", "team",
        "The opening the ETC holds at idle. Cross-checks against the torque curve: 14% is the zero-net-torque plate position at about 2350 rpm."),
      p("Crank + primary inertia", v.engineInertiaKgM2, "kg·m²", "estimate",
        "Crank-referenced. Through 1st (17.4:1) this is the dominant part of the ~+94 kg apparent mass the quasi-steady lap sim ignores entirely."),
      p("Gearbox + sprocket inertia", v.gearboxInertiaKgM2, "kg·m²", "estimate",
        "Referenced after the primary, because that is where the clutch physically sits on a CBR600RR."),
      p("Wheel inertia, front axle", v.wheelInertiaFrontKgM2, "kg·m²", "team", "Per wheel, team workbook. The brakes calculator's build-up (tyre + rim + rotor + spindle) gives 0.20.", { path: "wheelInertiaFrontKgM2", min: 0.05, max: 0.7, step: 0.005 }),
      p("Wheel inertia, rear axle", v.wheelInertiaRearKgM2, "kg·m²", "team", "Per wheel, team workbook. The rear disc is inboard, so it is in the driveline figure, not here.", { path: "wheelInertiaRearKgM2", min: 0.05, max: 0.7, step: 0.005 }),
    ]),

    rows("Steering & brakes", [
      p("Steering lock, road wheel", v.maxSteerDeg, "deg", "estimate",
        "Not a rack measurement, but three of four 2026-04-01 MoTeC runs cap the rim at 121-124 deg, which is 27.4-28.1 deg through the 4.411 ratio. Measure the rack stops to close it.", { path: "maxSteerDeg", min: 10, max: 45, step: 0.5 }),
      p("Steering ratio", v.steeringRatio, ":1", "team",
        "OptimumK 'Designed vs Actual Kinematics' export, 2026-06-27. Sets the cockpit wheel angle and the rim torque per kingpin torque.",
        { path: "steeringRatio", min: 2, max: 10, step: 0.1 }),
      p("Caster", v.steering.casterDeg, "deg", "team", "OptimumK export; KPI 8.745 deg, scrub 25.5 mm, mechanical trail 18.85 mm (kingpin offset adds 2.25 mm to R tan(caster))."),
      p("Steering lag", v.steerLagS * 1000, "ms", "estimate", "Driver's hands plus rack compliance.", { path: "steerLagS", min: 10, max: 300, step: 1, factor: 1000 }),
      p("Steering rate limit", v.steerRateDegS, "deg/s", "estimate", "At the road wheel."),
      p("Max brake torque", v.brakeTorqueMaxNm, "N·m", "estimate",
        "Derived, not measured: the brakes calculator's 70 bar max working pressure through the P4.24/P2.24 callipers, pad mu 0.45 and the 54% bias bar. That is 206 lbf on the pedal; the force a driver reaches is the unmeasured part. All four lock at 786 N·m (1.5 g)."),
      p("Brake bias, front", v.brakeBiasFront * 100, "%", "team",
        "Torque share from the calliper geometry and the 54% bias bar on the car (calculator: 71.3%, workbook radii: 72.7%). The bias bar's 54% is a pressure split."),
    ]),

    rows("Driver & environment", [
      p("Eye height", v.eyeHeightM, "m", "estimate", "Cockpit camera.", { path: "eyeHeightM", min: 0.4, max: 1.0, step: 0.005 }),
      p("Eye position vs CG", v.eyeAheadOfCgM, "m", "estimate", "Negative is behind the CG."),
      p("Car vibration", v.vibrationScale * 100, "%", "estimate",
        "How much surface texture comes through the seat. Camera only — it does not touch the physics. 0% is a perfectly smooth world.",
        { path: "vibrationScale", min: 0, max: 250, step: 5, factor: 100 }),
      p("Gravity", G, "m/s²", "team"),
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
