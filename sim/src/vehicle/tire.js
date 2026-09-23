// Magic-Formula tyre with load sensitivity and combined slip.
//
// This is a shape-fitted MF, not the full MF6.1.2 .tir evaluator that lives in
// the Oracle Rust core. The reason is calibration, not laziness: Oracle found
// the raw R20 TTC fit peaks at |mu| ~ 2.32 and does not reach peak Fy until
// 13-16 deg of slip, which is a fine basis for a peak-grip lap sim but gives a
// driving sim a vague, disconnected steering feel. Here the peaks are pinned to
// the numbers Helios validated against real runs -- muLat 1.368 (5.02 s
// skidpad), muLong 1.5 (4.2 s accel) -- and the slip at which those peaks
// arrive is set to the values a driver actually feels through a 10" slick.
//
// Combined slip uses Pacejka's similarity method (normalised slip vector), so
// pure-slip limits are exact and the friction ellipse falls out of the model
// rather than being pasted on afterwards.

// TEAM (MF6.1): where Fy peaks. 7.3 deg, from the suspension lead's
// MF612 Hoosier 16x7.5-10 R20 fit evaluated at the 10 psi the car actually
// runs (5.7 deg at 200 N, 6.6 at 600, 7.0 at 700, 7.6 at 800), which is the
// same number the AC mod runs. It replaces an 8.5 deg estimate justified
// against a "13-16 deg" TTC fit -- the distorted MF62 fit the team's own
// TireModelingReport flags. The peak force is pinned by mu and does not
// move; what changes is that the tyre reaches it about 1.2 deg earlier, so
// the front loads up and the rim firms up sooner.
const PEAK_SLIP_ANGLE_RAD = (7.3 * Math.PI) / 180;
const PEAK_SLIP_RATIO = 0.11;                      // where Fx peaks

// Shape and curvature factors. E is negative on purpose: a positive E pushes
// the peak far out in normalised slip, which forces B (and with it the
// cornering stiffness) to absurd values to keep the peak at 8.5 deg. Negative E
// gives the sharp build and gentle post-peak fall a slick actually has, and
// lands the cornering stiffness near 250 N/deg per tyre at static load.
const CY = 1.45, EY = -0.35;
const CX = 1.55, EX = -0.40;

/** Magic Formula core, normalised: returns force/(mu*Fz) for slip `x`. */
function mf(x, B, C, E) {
  const bx = B * x;
  return Math.sin(C * Math.atan(bx - E * (bx - Math.atan(bx))));
}

/** Solve the stiffness factor B that puts the MF peak exactly at `xPeak`. */
function solveB(xPeak, C, E) {
  // The peak moves monotonically closer to zero as B grows, so bisect on B.
  const peakOf = (B) => {
    let best = 0, at = 0;
    for (let i = 1; i <= 400; i++) {
      const x = (i / 400) * (xPeak * 4);
      const y = mf(x, B, C, E);
      if (y > best) { best = y; at = x; }
    }
    return at;
  };
  let lo = 0.5, hi = 60;
  for (let i = 0; i < 60; i++) {
    const mid = 0.5 * (lo + hi);
    if (peakOf(mid) > xPeak) lo = mid; else hi = mid;
  }
  return 0.5 * (lo + hi);
}

/** Normalising scale so the fitted curve peaks at exactly 1.0 (i.e. at mu*Fz). */
function peakValue(B, C, E, xPeak) {
  let best = 0;
  for (let i = 1; i <= 400; i++) best = Math.max(best, mf((i / 400) * xPeak * 4, B, C, E));
  return best || 1;
}

const BY = solveB(PEAK_SLIP_ANGLE_RAD, CY, EY);
const BX = solveB(PEAK_SLIP_RATIO, CX, EX);
const KY = 1 / peakValue(BY, CY, EY, PEAK_SLIP_ANGLE_RAD);
const KX = 1 / peakValue(BX, CX, EX, PEAK_SLIP_RATIO);

export const TIRE_INFO = {
  peakSlipAngleDeg: (PEAK_SLIP_ANGLE_RAD * 180) / Math.PI,
  peakSlipRatio: PEAK_SLIP_RATIO,
  BY, BX,
  /** Cornering stiffness (N/rad) at load Fz for peak grip mu. */
  corneringStiffness: (mu, Fz) => BY * CY * KY * mu * Fz,
};

/**
 * Load-sensitive peak friction. mu falls linearly with load ratio at the rate
 * Helios calls `tireLoadSensitivity` (0.15 -> mu drops 15% when load doubles).
 * Clamped so an unloaded or hugely loaded tyre stays physical.
 */
export function muAtLoad(mu0, Fz, Fz0, sensitivity) {
  if (Fz <= 0) return 0;
  const m = mu0 * (1 - sensitivity * (Fz / Fz0 - 1));
  return Math.max(0.25 * mu0, Math.min(1.6 * mu0, m));
}

/**
 * Effective peak friction for an AXLE once lateral load transfer has moved
 * load from the inner to the outer tyre. Because mu falls with load, the pair
 * always makes less grip than two equally loaded tyres would -- this is the
 * mechanism that turns roll-stiffness distribution into understeer balance,
 * and it is why the bicycle model below still responds to the ARB setting.
 *
 * Returns the load-weighted mean mu of the two tyres.
 */
export function axleMu(mu0, FzAxle, dFzLateral, Fz0, sensitivity) {
  if (FzAxle <= 0) return 0;
  const half = FzAxle / 2;
  const shift = Math.min(Math.abs(dFzLateral), half); // inner tyre lifts, no further
  const outer = half + shift;
  const inner = half - shift;
  const muO = muAtLoad(mu0, outer, Fz0, sensitivity);
  const muI = muAtLoad(mu0, inner, Fz0, sensitivity);
  return (muO * outer + muI * inner) / FzAxle;
}

/**
 * Combined-slip tyre forces for one axle of the bicycle model.
 *
 * @param slipAngle  lateral slip angle (rad, SAE: positive slip -> positive Fy)
 * @param slipRatio  longitudinal slip ratio kappa
 * @param Fz         axle normal load (N)
 * @param muY        effective lateral peak friction for this axle
 * @param muX        effective longitudinal peak friction for this axle
 * @returns {fx, fy, utilisation} forces in N, utilisation 0..1+ of the ellipse
 */
/** How much of the fitted longitudinal fall-off past the peak is kept: a
 *  fully locked tyre holds 0.90 of its peak. Mirrors tyre.rs. */
export const FX_FALLOFF_KEEP = 0.357;

export function tyreForces(slipAngle, slipRatio, Fz, muY, muX) {
  if (Fz <= 1) return { fx: 0, fy: 0, utilisation: 0, trail: 0 };

  // Normalised slip vector (Pacejka similarity): each channel measured in
  // units of its own peak, so the combined limit is a true ellipse.
  const sx = slipRatio / PEAK_SLIP_RATIO;
  const sy = Math.tan(slipAngle) / Math.tan(PEAK_SLIP_ANGLE_RAD);
  const s = Math.hypot(sx, sy);
  if (s < 1e-6) return { fx: 0, fy: 0, utilisation: 0, trail: pneumaticTrail(0, Fz) };

  // Evaluate each pure curve at the combined slip magnitude, then split the
  // resulting force along the slip direction.
  // 2026-09-22: past the peak the longitudinal force keeps FX_FALLOFF_KEEP of
  // the fitted shape's fall-off, so a locked or spinning tyre holds ~0.90 of its
  // peak instead of 0.72. The C 1.55 / E -0.40 shape was chosen to put the PEAK
  // where a driver feels it and was never fitted past it; the team's MF6.1 .tir
  // holds 0.96-0.98 of peak fully locked. The steep fall-off made every front
  // lock-up snap (decel collapsed and the wheel stayed locked until the pedal came
  // well up) and starved a wheelspin launch the real car pulls ~1 g through. Peak
  // height and peak slip are unchanged; below the peak nothing moves.
  // Same expression and order as tyre.rs, for parity.
  let nx = KX * mf(s * PEAK_SLIP_RATIO, BX, CX, EX);
  if (s > 1) nx = 1 - (1 - nx) * FX_FALLOFF_KEEP;
  const fx0 = muX * Fz * nx;
  const fy0 = muY * Fz * KY * mf(Math.atan(s * Math.tan(PEAK_SLIP_ANGLE_RAD)), BY, CY, EY);

  return {
    fx: (sx / s) * fx0,
    fy: (sy / s) * fy0,
    utilisation: Math.min(s, 3),
    trail: pneumaticTrail(s, Fz),
  };
}

// ---- aligning torque -------------------------------------------------------
//
// Self-aligning torque is Fy acting through the pneumatic trail: the lateral
// force is centred behind the contact-patch centre while the patch is mostly
// gripping, and moves forward to the centre as the rear of the patch starts to
// slide. So the trail is longest at zero slip and collapses to zero at the
// point the tyre is fully sliding -- which is why a steering wheel goes light
// BEFORE the front end lets go. That collapse, not the peak force, is the
// signal a driver reads through force feedback.
//
// Shape is the brush model's: trail falls as (1 - s)^2 in normalised slip and
// is zero from full sliding on. The brush model actually puts full sliding at
// the force peak; the TTC data says this slick keeps trail well past it (a
// fifth of the zero-slip value at the force peak, gone near 15 deg), so full
// sliding is placed beyond the peak (TRAIL_ZERO_SLIP). The scale grows with
// the square root of load because contact-patch length does.
//
// TEAM (TTC): fitted to the raw FSAE TTC Round 9 Mz channel for this exact
// tyre (Hoosier 43075 16x7.5-10 R20, 7 in rim, run 6: 12 psi, zero camber,
// 25 mph) by sim/tools/ttc_trail.py. Least squares of this shape over 7359
// samples at 222-1112 N gives t0 = 39.2 mm at 700 N with the square-root
// load law (a free exponent fits 0.60) and zero trail at 1.845 x the peak
// slip, i.e. 15.4 deg; rms 8 mm, the shape being an approximation. The
// measured near-zero-slip trail is 16 / 21 / 27 / 35 / 43 mm at 222 / 445 /
// 667 / 890 / 1112 N. Belt-to-road scaling is a force effect and does not
// touch a contact-patch length, so the trail is used unscaled.

const PNEUMATIC_TRAIL_M = 0.0392;
const TRAIL_ZERO_SLIP = 1.845;        // normalised slip at which trail hits zero
const TRAIL_REF_LOAD_N = 700;         // load at which t0 applies (~ one SDM26 corner)

/**
 * Pneumatic trail (m) at normalised combined slip `s` and load `Fz`.
 * Multiply by Fy for the aligning torque that opposes the slip.
 */
export function pneumaticTrail(s, Fz) {
  if (Fz <= 0) return 0;
  const x = Math.min(Math.abs(s) / TRAIL_ZERO_SLIP, 1);
  const shape = (1 - x) * (1 - x);
  return PNEUMATIC_TRAIL_M * Math.sqrt(Fz / TRAIL_REF_LOAD_N) * shape;
}

TIRE_INFO.pneumaticTrailM = PNEUMATIC_TRAIL_M;
TIRE_INFO.trailZeroSlip = TRAIL_ZERO_SLIP;
TIRE_INFO.pneumaticTrail = pneumaticTrail;
