// Starting points for the wheels people actually own.
//
// Two things about a wheel base cannot be read from the device and matter a
// great deal: what torque its motor is rated for, and how far it is set to
// rotate. Everything else -- which axis is which, whether the pedals rest at
// +1 or -1 -- can be found with the live axis monitor and the calibration
// buttons in the controls panel, and the numbers here are only the first
// guess for that.
//
// The rated torque is the one number that makes the force feedback portable:
// the mix is in newton-metres at the rim, and `maxForceNm` is what "full
// output" means on this base. Get it right and the same settings feel the
// same on a 2 N.m belt wheel and a 25 N.m direct drive -- the belt wheel just
// runs out earlier. The default gain is derived from it so that a car
// making ~9 N.m per g does not spend every corner in the clip on a small
// base, and does not feel dead on a big one.
//
// Axis order is DirectInput's (X Y Z Rx Ry Rz Slider0 Slider1) on the base,
// then the same eight for each further device the rig reads: axis 8 is the
// X of a separate pedal set. Pedals that come through the base sit in the
// first eight. Where a preset's pedal axes are marked `verify`, the vendor
// mapping is known to vary by model and firmware and the driver should press
// each pedal and watch which number moves before trusting it.

/**
 * @typedef {object} WheelPreset
 * @property {RegExp} match     tested against the lowercased product name
 * @property {string} label
 * @property {number} ratedNm   motor torque the base is rated for
 * @property {number} rotationDeg  rotation the base ships set to
 * @property {number} steerAxis
 * @property {{axis:number, rawMin:number, rawMax:number}|null} throttle
 * @property {{axis:number, rawMin:number, rawMax:number}|null} brake
 * @property {{axis:number, rawMin:number, rawMax:number}|null} clutch
 * @property {boolean} verify   pedal axes are a guess to check in the monitor
 * @property {string} note
 */

// A pedal resting at +1 and pressed to -1 is the common DirectInput shape.
const inv = (axis) => ({ axis, rawMin: 1, rawMax: -1 });
const fwd = (axis) => ({ axis, rawMin: -1, rawMax: 1 });

/**
 * Bump when a preset's values change. `Input.applyWheelPreset` records
 * `name@version`, so a corrected preset is applied once more to a profile
 * that had the old one, and never again after that.
 */
export const PRESET_VERSION = 5;

/** @type {WheelPreset[]} Most specific first: "r5" must beat "moza". */
export const WHEEL_PRESETS = [
  // ---- MOZA. Pedals through the base or over their own USB (SR-P): both seen.
  // A MOZA base reports its pedal axes at the BOTTOM of the range at rest
  // (raw 0 over DirectInput, -1 here) and rising with travel -- measured on
  // an R5 with the rig trace, 2026-09-17 -- so these are `fwd`, not `inv`.
  // Same session, confirmed by the driver: throttle is Z (axis 2) and brake
  // is Rz (axis 5) on an SR-P set through the base; Y (axis 1) is the
  // clutch on a three-pedal set.
  { match: /moza.*\br3\b/, label: "MOZA R3", ratedNm: 3.9, rotationDeg: 900, steerAxis: 0, throttle: fwd(2), brake: fwd(5), clutch: fwd(1), verify: true, note: "Set Pit House to PC mode. Zero the base's spring and damper." },
  { match: /moza.*\br5\b/, label: "MOZA R5", ratedNm: 5.5, rotationDeg: 900, steerAxis: 0, throttle: fwd(2), brake: fwd(5), clutch: fwd(1), verify: true, note: "Set Pit House to PC mode. Zero the base's spring and damper." },
  { match: /moza.*\br9\b/, label: "MOZA R9", ratedNm: 9, rotationDeg: 900, steerAxis: 0, throttle: fwd(2), brake: fwd(5), clutch: fwd(1), verify: true, note: "Set Pit House to PC mode. Zero the base's spring and damper." },
  { match: /moza.*\br12\b/, label: "MOZA R12", ratedNm: 12, rotationDeg: 900, steerAxis: 0, throttle: fwd(2), brake: fwd(5), clutch: fwd(1), verify: true, note: "Set Pit House to PC mode. Zero the base's spring and damper." },
  { match: /moza.*\br16\b/, label: "MOZA R16", ratedNm: 16, rotationDeg: 900, steerAxis: 0, throttle: fwd(2), brake: fwd(5), clutch: fwd(1), verify: true, note: "Set Pit House to PC mode. Zero the base's spring and damper." },
  { match: /moza.*\br21\b/, label: "MOZA R21", ratedNm: 21, rotationDeg: 900, steerAxis: 0, throttle: fwd(2), brake: fwd(5), clutch: fwd(1), verify: true, note: "Set Pit House to PC mode. Zero the base's spring and damper." },
  { match: /moza/, label: "MOZA (unknown model)", ratedNm: 5.5, rotationDeg: 900, steerAxis: 0, throttle: fwd(2), brake: fwd(5), clutch: fwd(1), verify: true, note: "Model not recognised; rated torque set to 5.5 N.m. Correct it in the slider." },

  // ---- Logitech. Gear driven, ~2 N.m; pedals through the base on Y / Rz / Slider.
  { match: /g923/, label: "Logitech G923", ratedNm: 2.2, rotationDeg: 900, steerAxis: 0, throttle: inv(1), brake: inv(5), clutch: inv(6), verify: false, note: "Set G HUB to 900 deg and centring spring off." },
  { match: /g920/, label: "Logitech G920", ratedNm: 2.2, rotationDeg: 900, steerAxis: 0, throttle: inv(1), brake: inv(5), clutch: inv(6), verify: false, note: "Set G HUB to 900 deg and centring spring off. Must be in PC mode, not Xbox." },
  { match: /g29/, label: "Logitech G29", ratedNm: 2.2, rotationDeg: 900, steerAxis: 0, throttle: inv(1), brake: inv(5), clutch: inv(6), verify: false, note: "Set G HUB to 900 deg and centring spring off. Switch on the base set to PS3." },
  { match: /g27/, label: "Logitech G27", ratedNm: 2.0, rotationDeg: 900, steerAxis: 0, throttle: inv(1), brake: inv(5), clutch: inv(6), verify: true, note: "Logitech Profiler: 900 deg, centring spring off." },
  { match: /driving force|logitech/, label: "Logitech (other)", ratedNm: 2.0, rotationDeg: 900, steerAxis: 0, throttle: inv(1), brake: inv(5), clutch: inv(6), verify: true, note: "Model not recognised; treated as a 2 N.m gear-driven wheel." },

  // ---- Thrustmaster. Belt driven. Pedal axes vary by model; verify.
  { match: /t-gt|tgt/, label: "Thrustmaster T-GT", ratedNm: 5.0, rotationDeg: 1080, steerAxis: 0, throttle: inv(5), brake: inv(1), clutch: inv(6), verify: true, note: "Control Panel: 1080 deg, spring and damper 0%." },
  { match: /t300|t248|tx /, label: "Thrustmaster T300 / T248 / TX", ratedNm: 3.9, rotationDeg: 1080, steerAxis: 0, throttle: inv(5), brake: inv(1), clutch: inv(6), verify: true, note: "Control Panel: 1080 deg, spring and damper 0%. PC mode on the base switch." },
  { match: /t150|tmx/, label: "Thrustmaster T150 / TMX", ratedNm: 2.5, rotationDeg: 1080, steerAxis: 0, throttle: inv(5), brake: inv(1), clutch: null, verify: true, note: "Control Panel: 1080 deg, spring and damper 0%." },
  { match: /thrustmaster/, label: "Thrustmaster (other)", ratedNm: 3.9, rotationDeg: 1080, steerAxis: 0, throttle: inv(5), brake: inv(1), clutch: inv(6), verify: true, note: "Model not recognised; treated as a T300-class belt wheel." },

  // ---- Fanatec. Direct drive; pedals usually on their own USB (axis 8+) unless
  //      plugged into the base.
  { match: /dd2|podium.*2/, label: "Fanatec Podium DD2", ratedNm: 25, rotationDeg: 1080, steerAxis: 0, throttle: inv(1), brake: inv(5), clutch: inv(2), verify: true, note: "PC mode. Set FF to 100, SPR and DPR to OFF in the tuning menu." },
  { match: /dd1|podium/, label: "Fanatec Podium DD1", ratedNm: 20, rotationDeg: 1080, steerAxis: 0, throttle: inv(1), brake: inv(5), clutch: inv(2), verify: true, note: "PC mode. Set FF to 100, SPR and DPR to OFF in the tuning menu." },
  { match: /csl dd|gt dd|clubsport dd/, label: "Fanatec CSL DD / GT DD", ratedNm: 8, rotationDeg: 1080, steerAxis: 0, throttle: inv(1), brake: inv(5), clutch: inv(2), verify: true, note: "8 N.m with the boost kit, 5 without: set the rated torque to match. SPR and DPR OFF." },
  { match: /fanatec|csl|clubsport/, label: "Fanatec (other)", ratedNm: 8, rotationDeg: 1080, steerAxis: 0, throttle: inv(1), brake: inv(5), clutch: inv(2), verify: true, note: "Model not recognised; rated torque set to 8 N.m." },

  // ---- Simucube. Direct drive, steering only; pedals are always separate.
  { match: /simucube.*ultimate/, label: "Simucube 2 Ultimate", ratedNm: 32, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "True Drive: set a profile with no centre spring or extra damping. Pedals are a separate device: calibrate them." },
  { match: /simucube.*pro/, label: "Simucube 2 Pro", ratedNm: 25, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "True Drive: no centre spring or extra damping. Pedals are a separate device: calibrate them." },
  { match: /simucube/, label: "Simucube 2 Sport", ratedNm: 17, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "True Drive: no centre spring or extra damping. Pedals are a separate device: calibrate them." },

  // ---- Simagic, Cammus, Asetek, VRS: direct drive; pedals separate.
  { match: /simagic.*mini/, label: "Simagic Alpha Mini", ratedNm: 10, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "SimPro Manager: spring and damper 0. Pedals are a separate device." },
  { match: /simagic.*ultimate/, label: "Simagic Alpha Ultimate", ratedNm: 23, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "SimPro Manager: spring and damper 0. Pedals are a separate device." },
  { match: /simagic/, label: "Simagic Alpha", ratedNm: 15, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "SimPro Manager: spring and damper 0. Pedals are a separate device." },
  { match: /cammus.*c12/, label: "Cammus C12", ratedNm: 12, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "Pedals are a separate device." },
  { match: /cammus/, label: "Cammus C5", ratedNm: 5, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "Pedals are a separate device." },
  { match: /asetek.*invicta/, label: "Asetek Invicta", ratedNm: 27, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "RaceHub: no centre spring. Pedals are a separate device." },
  { match: /asetek.*forte/, label: "Asetek Forte", ratedNm: 18, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "RaceHub: no centre spring. Pedals are a separate device." },
  { match: /asetek/, label: "Asetek La Prima", ratedNm: 12, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "RaceHub: no centre spring. Pedals are a separate device." },
  { match: /vrs/, label: "VRS DirectForce Pro", ratedNm: 20, rotationDeg: 900, steerAxis: 0, throttle: null, brake: null, clutch: null, verify: true, note: "Pedals are a separate device." },
];

/** The unknown wheel: a cautious 5 N.m and generic axes. */
export const GENERIC_WHEEL = {
  match: /./, label: "Wheel (unrecognised)", ratedNm: 5, rotationDeg: 900, steerAxis: 0,
  throttle: inv(1), brake: inv(2), clutch: inv(5), verify: true,
  note: "Not in the preset table. Set the rated torque to what the base is rated for, and check the pedal axes in the monitor.",
};

/** @returns {WheelPreset} the preset for a product name, or the generic one. */
export function presetFor(productName) {
  const id = (productName || "").toLowerCase();
  return WHEEL_PRESETS.find((p) => p.match.test(id)) || GENERIC_WHEEL;
}

/**
 * A default gain for a base of this rating.
 *
 * SDM26 makes about 12 N.m per g at the rim, and the aligning torque PEAKS
 * near 15 N.m at roughly 4 degrees of front slip -- well before the tyre's
 * own force peak, which is exactly why the rim goes light before the front
 * lets go. A base that can make 15 N.m gets unity; a smaller one is scaled so
 * the peak lands at full output and the fall-off past it is still inside the
 * motor's range rather than buried in the clip. Floored so a 2 N.m gear wheel
 * still gets a usable shape.
 *
 * The old divisor was 11, from a 9 N.m/g estimate that predates the fitted
 * pneumatic trail; it put a MOZA R5 in the clip from 0.8 g upward.
 */
export function defaultGainFor(ratedNm) {
  const r = Math.max(0.1, ratedNm || 5);
  return Math.round(Math.min(1, Math.max(0.3, r / 15)) * 100) / 100;
}

/**
 * The settings this preset would set on the wheel profile, as dotted paths.
 * Applied once per newly seen base; anything the driver has since changed is
 * left alone (see `Input.applyWheelPreset`).
 */
export function presetPaths(preset) {
  const out = {
    "wheel.rotationDeg": preset.rotationDeg,
    "axes.steer": preset.steerAxis,
    "forceFeedback.maxForceNm": preset.ratedNm,
    "forceFeedback.gain": defaultGainFor(preset.ratedNm),
  };
  for (const which of ["throttle", "brake", "clutch"]) {
    const p = preset[which];
    if (p) {
      out[`pedals.${which}.source`] = p.axis;
      out[`pedals.${which}.isAxis`] = true;
      out[`pedals.${which}.rawMin`] = p.rawMin;
      out[`pedals.${which}.rawMax`] = p.rawMax;
    }
  }
  return out;
}
