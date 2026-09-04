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

const PEAK_SLIP_ANGLE_RAD = (8.5 * Math.PI) / 180; // where Fy peaks
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
export function tyreForces(slipAngle, slipRatio, Fz, muY, muX) {
  if (Fz <= 1) return { fx: 0, fy: 0, utilisation: 0 };

  // Normalised slip vector (Pacejka similarity): each channel measured in
  // units of its own peak, so the combined limit is a true ellipse.
  const sx = slipRatio / PEAK_SLIP_RATIO;
  const sy = Math.tan(slipAngle) / Math.tan(PEAK_SLIP_ANGLE_RAD);
  const s = Math.hypot(sx, sy);
  if (s < 1e-6) return { fx: 0, fy: 0, utilisation: 0 };

  // Evaluate each pure curve at the combined slip magnitude, then split the
  // resulting force along the slip direction.
  const fx0 = muX * Fz * KX * mf(s * PEAK_SLIP_RATIO, BX, CX, EX);
  const fy0 = muY * Fz * KY * mf(Math.atan(s * Math.tan(PEAK_SLIP_ANGLE_RAD)), BY, CY, EY);

  return {
    fx: (sx / s) * fx0,
    fy: (sy / s) * fy0,
    utilisation: Math.min(s, 3),
  };
}
