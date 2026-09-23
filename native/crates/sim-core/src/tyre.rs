//! Tyre models.
//!
//! A tyre takes a slip state and a vertical load and returns forces. Crucially
//! it owns its own load sensitivity, which is what lets the solvers stay
//! simple: a double-track model just calls it four times with four loads, and
//! a bicycle model calls it twice per axle with the inner and outer loads and
//! sums. Neither needs to know how mu varies with load.
//!
//! That is a change from the JS build, where the bicycle model computed a
//! load-weighted mean mu for the axle and handed it to the tyre. Summing the two
//! contact patches is the same idea done properly, and it means the bicycle and
//! double-track solvers share one code path into the tyre.

/// Slip state of one contact patch.
#[derive(Debug, Clone, Copy, Default)]
pub struct Slip {
    /// Lateral slip angle (rad). Positive slip gives positive Fy.
    pub alpha: f64,
    /// Longitudinal slip ratio.
    pub kappa: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct TyreForces {
    pub fx: f64,
    pub fy: f64,
    /// Normalised slip magnitude: 1.0 is peak, above is past the limit.
    pub utilisation: f64,
    /// Pneumatic trail (m): the lever arm behind the contact-patch centre
    /// through which `fy` makes the self-aligning moment. Longest at zero
    /// slip, zero once the patch is fully sliding. Models without an aligning
    /// moment leave it at zero.
    pub trail: f64,
}

impl TyreForces {
    pub fn zero() -> Self {
        Self::default()
    }
}

pub trait TyreModel: Send + Sync {
    fn name(&self) -> &'static str;

    /// For a host that needs the concrete model back (to edit its constants
    /// live). Models that do not care return `None`.
    fn as_any_mut(&mut self) -> Option<&mut dyn core::any::Any> {
        None
    }

    /// Forces from one contact patch at vertical load `fz` (N).
    fn forces(&self, slip: Slip, fz: f64) -> TyreForces;

    /// Peak (mux, muy) available at this load. Used by the simplest solvers,
    /// which never form a slip state at all.
    fn peak_mu(&self, fz: f64) -> (f64, f64);

    /// Forces with the wheel leaning over. `gamma` is the inclination to the
    /// road (rad), positive when the top of the wheel leans toward +y (the
    /// direction positive `fy` points), so camber thrust is positive with it.
    /// Models without camber sensitivity ignore it.
    fn forces_cambered(&self, slip: Slip, fz: f64, gamma: f64) -> TyreForces {
        let _ = gamma;
        self.forces(slip, fz)
    }

    /// Distance the tyre must roll to build slip force (m). Zero disables the
    /// relaxation lag entirely.
    fn relaxation_length(&self) -> f64 {
        0.0
    }
}

// ---------------------------------------------------------------- linear ---

/// Level 1: linear up to a friction ceiling.
///
/// Cornering stiffness until the friction circle runs out, then flat. No
/// falloff past the peak, so it cannot spin -- which is exactly what you want
/// when the point is to study grip budget rather than car control.
#[derive(Debug, Clone)]
pub struct LinearTyre {
    pub mu_x: f64,
    pub mu_y: f64,
    pub load_sensitivity: f64,
    pub nominal_load: f64,
    /// Cornering stiffness per unit load (1/rad).
    pub cornering_stiffness_per_n: f64,
    pub slip_stiffness_per_n: f64,
}

impl LinearTyre {
    pub fn sdm26() -> Self {
        Self {
            mu_x: 1.5,
            mu_y: 1.89,
            load_sensitivity: 0.12,
            nominal_load: 654.8,
            cornering_stiffness_per_n: 25.0,
            slip_stiffness_per_n: 22.0,
        }
    }

    fn mu_at(&self, base: f64, fz: f64) -> f64 {
        if fz <= 0.0 {
            return 0.0;
        }
        let m = base * (1.0 - self.load_sensitivity * (fz / self.nominal_load - 1.0));
        m.clamp(0.25 * base, 1.6 * base)
    }
}

impl TyreModel for LinearTyre {
    fn name(&self) -> &'static str {
        "Linear + friction ceiling"
    }

    fn forces(&self, slip: Slip, fz: f64) -> TyreForces {
        if fz <= 1.0 {
            return TyreForces::zero();
        }
        let (mux, muy) = self.peak_mu(fz);
        let fy_lin = self.cornering_stiffness_per_n * fz * slip.alpha;
        let fx_lin = self.slip_stiffness_per_n * fz * slip.kappa;

        // Scale both back together if the combined demand leaves the ellipse,
        // so the split between them is preserved.
        let demand = ((fx_lin / (mux * fz)).powi(2) + (fy_lin / (muy * fz)).powi(2)).sqrt();
        let scale = if demand > 1.0 { 1.0 / demand } else { 1.0 };
        TyreForces {
            fx: fx_lin * scale,
            fy: fy_lin * scale,
            utilisation: demand,
            trail: 0.0,
        }
    }

    fn peak_mu(&self, fz: f64) -> (f64, f64) {
        (self.mu_at(self.mu_x, fz), self.mu_at(self.mu_y, fz))
    }
}

// --------------------------------------------------------- magic formula ---

/// Level 2: shape-fitted Magic Formula with combined slip.
///
/// Peaks are pinned to the values Helios validated against real SDM26 runs,
/// and the slip at which they arrive is set to what a driver feels through a
/// 10 in slick. Combined slip uses Pacejka's similarity method, so the
/// friction ellipse falls out of the model rather than being pasted on.
#[derive(Debug, Clone)]
pub struct MagicFormulaTyre {
    pub mu_x: f64,
    pub mu_y: f64,
    pub load_sensitivity: f64,
    pub nominal_load: f64,
    pub peak_alpha: f64,
    pub peak_kappa: f64,
    pub relaxation_m: f64,
    /// Pneumatic trail at `trail_ref_load_n` and zero slip (m).
    pub trail_m: f64,
    /// Normalised slip at which the trail reaches zero.
    pub trail_zero_slip: f64,
    pub trail_ref_load_n: f64,
    /// Camber: the slip angle one radian of inclination is worth, against
    /// load, as (Fz N, ratio) points interpolated linearly and held flat past
    /// the ends. Camber enters as a horizontal shift of the lateral curve --
    /// how the Magic Formula itself carries it -- so thrust builds and
    /// saturates with the tyre rather than being added on top of it.
    pub camber_ratio_at: [(f64, f64); 3],
    /// Peak grip loss with camber, gamma in rad:
    /// mu x (1 - k ((gamma - s g0)^2 - g0^2)), s the sign of the lateral force.
    /// Camber leaning the way the tyre is pushing ("favourable") costs less
    /// than the same camber the other way; `camber_mu_offset_rad` is g0.
    pub camber_mu_quad: f64,
    pub camber_mu_offset_rad: f64,
    /// Peak slip angle against load, as (Fz N, multiple of `peak_alpha`)
    /// points, linear between and flat past the ends. `None` holds the peak
    /// at `peak_alpha` at every load -- the validated bicycle's tyre, bit for
    /// bit. The double track switches it on (`DoubleTrackSolver::new`).
    ///
    /// A real slick peaks at a smaller slip when lightly loaded: TEAM MF6.1
    /// at 10 psi 5.7 / 7.0 / 7.6 deg at 200 / 700 / 800 N, and the MF6.1.2
    /// 7 in rim fit 6.5 / 8.2 / 9.6 deg (mean of both sides) at 300 / 655 /
    /// 800 N -- 0.79 and 1.17 of its own 655 N value. That fit is not
    /// believable above ~800 N (its cornering stiffness collapses and the
    /// peak runs off to 18 deg at 1200 N), so the top point is an
    /// extrapolation of the trend below it, flagged as an estimate.
    pub peak_alpha_scale_at: Option<[(f64, f64); 4]>,
    by: f64,
    /// Normalising scales so each fitted curve peaks at exactly mu*Fz. The
    /// solved B puts the peak at the right slip; these put it at the right
    /// height. Same scan as the JS build, so the two agree bit for bit.
    ky: f64,
    kx: f64,
    cy: f64,
    ey: f64,
    bx: f64,
    cx: f64,
    ex: f64,
}

impl MagicFormulaTyre {
    /// Hoosier 16x7.5-10 R20 as run on SDM26.
    pub fn sdm26() -> Self {
        // Nominal load is the SDM26 static corner load, derived from the mass
        // rather than typed in: a rounded 654.8 here put the Rust and JS
        // models 4e-6 apart in force, which compounds into centimetres by the
        // end of a lap.
        // mu_y is the REAR axle's peak; the front runs at mu_y times
        // `VehicleParams::front_grip_factor`. The pair is pinned to the 5.02 s
        // skidpad through the bicycle solver (1.89 x 0.80 at the front; was
        // 1.72 x 0.88 -- same front peak, 10% more rear margin so a held
        // 14 deg at 13 m/s off throttle pushes instead of spinning).
        // Peak slip angle 7.3 deg: TEAM MF6.1 fit at the 10 psi the car runs
        // (5.7 deg at 200 N, 7.0 at 700, 7.6 at 800), and what the AC mod
        // uses. Replaces an 8.5 deg estimate taken against the distorted MF62
        // fit. Peak FORCE is pinned by mu and does not move.
        Self::new(1.5, 1.89, 0.12, crate::vehicle::sdm26().nominal_tyre_load(), 7.3_f64.to_radians(), 0.11, 0.35)
    }

    /// Pneumatic trail (m) at normalised combined slip `s` and load `fz`.
    ///
    /// Brush-model shape: trail falls as (1 - s)^2 and is zero from full
    /// sliding on, which is placed a little past the force peak because a
    /// slick keeps some trail beyond it. Scale grows with the square root of
    /// load, as contact-patch length does. Same constants as the JS build,
    /// fitted to the TTC Round 9 Mz data for the R20.
    pub fn pneumatic_trail(&self, s: f64, fz: f64) -> f64 {
        if fz <= 0.0 {
            return 0.0;
        }
        let x = (s.abs() / self.trail_zero_slip).min(1.0);
        self.trail_m * (fz / self.trail_ref_load_n).sqrt() * (1.0 - x) * (1.0 - x)
    }

    pub fn new(
        mu_x: f64,
        mu_y: f64,
        load_sensitivity: f64,
        nominal_load: f64,
        peak_alpha: f64,
        peak_kappa: f64,
        relaxation_m: f64,
    ) -> Self {
        // E is negative on purpose. A positive E pushes the peak far out in
        // normalised slip, which forces B -- and with it the cornering
        // stiffness -- to absurd values to keep the peak where it belongs.
        let (cy, ey) = (1.45, -0.35);
        let (cx, ex) = (1.55, -0.40);
        let by = solve_b(peak_alpha, cy, ey);
        let bx = solve_b(peak_kappa, cx, ex);
        Self {
            mu_x,
            mu_y,
            load_sensitivity,
            nominal_load,
            peak_alpha,
            peak_kappa,
            relaxation_m,
            // Fitted to the raw TTC Round 9 Mz channel for this tyre by
            // sim/tools/ttc_trail.py; see tire.js for the numbers.
            trail_m: 0.0392,
            trail_zero_slip: 1.845,
            trail_ref_load_n: 700.0,
            // TEAM: 'MF612-Hoosier 16x7_5-10 R20 7in Rim.tir' (the newest fit
            // of the tyre the car runs, 2026-09), evaluated with the full MF6.1
            // lateral equations: Ky_gamma / Ky_alpha = 0.089 at 300 N, 0.107
            // at 655 N, 0.159 at 1000 N, and PDY3 = 18.66. The Drive
            // 'ISO-CamberSens-612' fit agrees to within its scatter (0.091,
            // PDY3 13.0). Camber is a small effect on this tyre: a degree of
            // it is worth about a tenth of a degree of slip.
            camber_ratio_at: [(300.0, 0.089), (655.0, 0.107), (1000.0, 0.159)],
            // 2026-09-22: from the full MF6.1 evaluation of the same .tir
            // (sdm26-assetto-corsa/tools/mf_eval.py, peak Fy at 655 N and
            // 10 psi against camber, -3..+3 deg), fitted to the asymmetric
            // form above: k 19.63 /rad^2, g0 0.394 deg, residual < 5e-4.
            // PDY3 alone (18.66, symmetric) missed that the file's camber
            // force (PVY3/PVY4) makes favourable camber nearly free up to a
            // degree (0.999 at +1 vs 0.989 at -1). Only the double track uses
            // camber; the bicycle never calls `forces_cambered`.
            camber_mu_quad: 19.63,
            camber_mu_offset_rad: 0.394_f64.to_radians(),
            peak_alpha_scale_at: None,
            by,
            ky: 1.0 / peak_value(by, cy, ey, peak_alpha),
            kx: 1.0 / peak_value(bx, cx, ex, peak_kappa),
            cy,
            ey,
            bx,
            cx,
            ex,
        }
    }

    fn mu_at(&self, base: f64, fz: f64) -> f64 {
        if fz <= 0.0 {
            return 0.0;
        }
        let m = base * (1.0 - self.load_sensitivity * (fz / self.nominal_load - 1.0));
        m.clamp(0.25 * base, 1.6 * base)
    }

    /// The load-dependent table the double track runs (see
    /// `peak_alpha_scale_at`).
    ///
    /// 2026-09-22: held FLAT past 800 N (was 1.30 at 1200 N). The top point
    /// was an extrapolation of a fit that is not believable above ~800 N, and
    /// with it the light inside rear peaked at 6 deg while the loaded outside
    /// one was still building to 9.5: the rear axle saturated unevenly, and
    /// with the LSD driving the outside wheel the car spun under power past
    /// the limit at 20 m/s -- which neither constant peak slip nor an open
    /// diff does. No data says the peak keeps moving past 800 N, so it doesn't.
    pub const PEAK_ALPHA_SCALE_SDM26: [(f64, f64); 4] = [(200.0, 0.82), (655.0, 1.0), (800.0, 1.12), (1200.0, 1.12)];

    /// Peak slip angle and the matching stiffness factor at a load. The MF
    /// core depends only on B.x, so moving the peak to alpha_p is B scaled by
    /// peak_alpha / alpha_p, and the normalising height is unchanged.
    fn lateral_peak(&self, fz: f64) -> (f64, f64) {
        let Some(t) = &self.peak_alpha_scale_at else {
            return (self.peak_alpha, self.by);
        };
        let k = if fz <= t[0].0 {
            t[0].1
        } else if fz >= t[t.len() - 1].0 {
            t[t.len() - 1].1
        } else {
            let mut k = t[t.len() - 1].1;
            for i in 0..t.len() - 1 {
                if fz <= t[i + 1].0 {
                    k = t[i].1 + (fz - t[i].0) / (t[i + 1].0 - t[i].0) * (t[i + 1].1 - t[i].1);
                    break;
                }
            }
            k
        };
        let ap = self.peak_alpha * k;
        (ap, self.by * self.peak_alpha / ap)
    }

    /// Camber-to-slip ratio at a load (see `camber_ratio_at`).
    pub fn camber_ratio(&self, fz: f64) -> f64 {
        let t = &self.camber_ratio_at;
        if fz <= t[0].0 {
            return t[0].1;
        }
        for i in 0..t.len() - 1 {
            if fz <= t[i + 1].0 {
                let f = (fz - t[i].0) / (t[i + 1].0 - t[i].0);
                return t[i].1 + f * (t[i + 1].1 - t[i].1);
            }
        }
        t[t.len() - 1].1
    }

    /// Cornering stiffness (N/rad) at a given load -- a headline tyre number.
    pub fn cornering_stiffness(&self, fz: f64) -> f64 {
        let (_, muy) = self.peak_mu(fz);
        self.by * self.cy * self.ky * muy * fz
    }
}

impl TyreModel for MagicFormulaTyre {
    fn name(&self) -> &'static str {
        "Magic Formula (fitted, combined slip)"
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn core::any::Any> {
        Some(self)
    }

    fn forces(&self, slip: Slip, fz: f64) -> TyreForces {
        if fz <= 1.0 {
            return TyreForces::zero();
        }
        let (mux, muy) = self.peak_mu(fz);

        // Normalised slip vector: each channel in units of its own peak, so the
        // combined limit is a true ellipse.
        let (peak_alpha, by) = self.lateral_peak(fz);
        let sx = slip.kappa / self.peak_kappa;
        let sy = slip.alpha.tan() / peak_alpha.tan();
        let s = (sx * sx + sy * sy).sqrt();
        if s < 1e-9 {
            return TyreForces { trail: self.pneumatic_trail(0.0, fz), ..TyreForces::zero() };
        }

        // 2026-09-22: past the peak the longitudinal force keeps FX_FALLOFF_KEEP of
        // the fitted shape's fall-off, so a locked or spinning tyre holds ~0.90 of its
        // peak instead of 0.72. The C 1.55 / E -0.40 shape was chosen to put the PEAK
        // where a driver feels it and was never fitted past it; the team's MF6.1 .tir
        // holds 0.96-0.98 of peak fully locked. The steep fall-off made every front
        // lock-up snap (decel collapsed and the wheel stayed locked until the pedal came
        // well up) and starved a wheelspin launch the real car pulls ~1 g through. Peak
        // height and peak slip are unchanged; below the peak nothing moves.
        let mut nx = self.kx * mf(s * self.peak_kappa, self.bx, self.cx, self.ex);
        if s > 1.0 {
            nx = 1.0 - (1.0 - nx) * FX_FALLOFF_KEEP;
        }
        let fx0 = mux * fz * nx;
        let fy0 =
            muy * fz * self.ky * mf((s * peak_alpha.tan()).atan(), by, self.cy, self.ey);

        TyreForces {
            fx: (sx / s) * fx0,
            fy: (sy / s) * fy0,
            utilisation: s.min(3.0),
            trail: self.pneumatic_trail(s, fz),
        }
    }

    fn peak_mu(&self, fz: f64) -> (f64, f64) {
        (self.mu_at(self.mu_x, fz), self.mu_at(self.mu_y, fz))
    }

    fn forces_cambered(&self, slip: Slip, fz: f64, gamma: f64) -> TyreForces {
        if gamma == 0.0 {
            return self.forces(slip, fz);
        }
        let shifted = Slip { alpha: slip.alpha + self.camber_ratio(fz) * gamma, kappa: slip.kappa };
        let mut f = self.forces(shifted, fz);
        let s = if f.fy >= 0.0 { 1.0 } else { -1.0 };
        let g0 = self.camber_mu_offset_rad;
        let d = gamma - s * g0;
        let k = (1.0 - self.camber_mu_quad * (d * d - g0 * g0)).clamp(0.5, 1.01);
        f.fx *= k;
        f.fy *= k;
        f
    }

    fn relaxation_length(&self) -> f64 {
        self.relaxation_m
    }
}

/// How much of the fitted longitudinal fall-off past the peak is kept (see
/// `forces`): 0.357 leaves a fully locked tyre at 0.90 of its peak.
pub const FX_FALLOFF_KEEP: f64 = 0.357;

/// Magic Formula core, normalised so the peak is exactly 1.0.
fn mf(x: f64, b: f64, c: f64, e: f64) -> f64 {
    let bx = b * x;
    (c * (bx - e * (bx - bx.atan())).atan()).sin()
}

/// Largest value the fitted curve reaches inside 4x the peak slip, on the
/// same 400-sample scan the JS build uses.
fn peak_value(b: f64, c: f64, e: f64, x_peak: f64) -> f64 {
    let mut best: f64 = 0.0;
    for i in 1..=400 {
        best = best.max(mf((i as f64 / 400.0) * x_peak * 4.0, b, c, e));
    }
    if best == 0.0 {
        1.0
    } else {
        best
    }
}

/// Solve the stiffness factor B that puts the MF peak exactly at `x_peak`.
fn solve_b(x_peak: f64, c: f64, e: f64) -> f64 {
    let peak_of = |b: f64| -> f64 {
        let (mut best, mut at) = (0.0, 0.0);
        for i in 1..=400 {
            let x = (i as f64 / 400.0) * (x_peak * 4.0);
            let y = mf(x, b, c, e);
            if y > best {
                best = y;
                at = x;
            }
        }
        at
    };
    let (mut lo, mut hi) = (0.5, 60.0);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if peak_of(mid) > x_peak {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn magic_formula_peaks_where_asked() {
        let t = MagicFormulaTyre::sdm26();
        let fz = 654.8;
        let mut best = (0.0, 0.0);
        for i in 0..600 {
            let a = (i as f64 / 600.0) * 30.0_f64.to_radians();
            let f = t.forces(Slip { alpha: a, kappa: 0.0 }, fz);
            if f.fy > best.1 {
                best = (a.to_degrees(), f.fy);
            }
        }
        assert!(
            (best.0 - 7.3).abs() < 0.5,
            "peak Fy at {:.2} deg, expected 7.3",
            best.0
        );
    }

    #[test]
    fn peak_force_matches_mu_times_load() {
        let t = MagicFormulaTyre::sdm26();
        let fz = 654.8;
        let (_, muy) = t.peak_mu(fz);
        let f = t.forces(Slip { alpha: t.peak_alpha, kappa: 0.0 }, fz);
        assert!((f.fy / (muy * fz) - 1.0).abs() < 0.01);
    }

    #[test]
    fn mu_falls_with_load() {
        let t = MagicFormulaTyre::sdm26();
        let (_, low) = t.peak_mu(400.0);
        let (_, high) = t.peak_mu(1200.0);
        assert!(high < low, "load sensitivity has the wrong sign");
    }

    #[test]
    fn combined_slip_stays_inside_the_ellipse() {
        let t = MagicFormulaTyre::sdm26();
        let fz = 700.0;
        let (mux, muy) = t.peak_mu(fz);
        for i in 0..40 {
            for j in 0..40 {
                let a = (i as f64 / 40.0) * 0.4;
                let k = (j as f64 / 40.0) * 0.5;
                let f = t.forces(Slip { alpha: a, kappa: k }, fz);
                let r = (f.fx / (mux * fz)).powi(2) + (f.fy / (muy * fz)).powi(2);
                assert!(r <= 1.05, "outside the friction ellipse: {r}");
            }
        }
    }

    #[test]
    fn camber_thrust_leans_the_way_the_wheel_does() {
        let t = MagicFormulaTyre::sdm26();
        let fz = 655.0;
        let lean = 2.0_f64.to_radians();
        let f = t.forces_cambered(Slip { alpha: 0.0, kappa: 0.0 }, fz, lean);
        assert!(f.fy > 0.0, "camber thrust {} N should follow the lean", f.fy);
        // A degree of camber is worth about 0.107 deg of slip at this load.
        let eq = t.forces(Slip { alpha: 0.107 * lean, kappa: 0.0 }, fz);
        let g0 = t.camber_mu_offset_rad;
        let k = 1.0 - t.camber_mu_quad * ((lean - g0).powi(2) - g0 * g0);
        assert!((f.fy - eq.fy * k).abs() < 1e-9);
        // And costs peak grip, a little.
        let peak0 = t.forces(Slip { alpha: t.peak_alpha, kappa: 0.0 }, fz).fy;
        let mut peak3: f64 = 0.0;
        for i in 0..400 {
            let a = i as f64 * 0.0005;
            peak3 = peak3.max(t.forces_cambered(Slip { alpha: a, kappa: 0.0 }, fz, 3.0_f64.to_radians()).fy);
        }
        // The MF6.1 file: 3 deg leaning WITH the force costs 4.0 %, against
        // it 6.8 %.
        let loss = 1.0 - peak3 / peak0;
        assert!((0.035..0.045).contains(&loss), "3 deg favourable camber cost {:.1} % of peak", loss * 100.0);
        let mut peak_m3: f64 = 0.0;
        for i in 0..400 {
            let a = i as f64 * 0.0005;
            peak_m3 = peak_m3.max(t.forces_cambered(Slip { alpha: a, kappa: 0.0 }, fz, -3.0_f64.to_radians()).fy);
        }
        let loss_m = 1.0 - peak_m3 / peak0;
        assert!((0.06..0.075).contains(&loss_m), "3 deg unfavourable camber cost {:.1} %", loss_m * 100.0);
    }

    #[test]
    fn linear_tyre_saturates() {
        let t = LinearTyre::sdm26();
        let fz = 654.8;
        let (_, muy) = t.peak_mu(fz);
        let f = t.forces(Slip { alpha: 0.5, kappa: 0.0 }, fz);
        assert!(f.fy <= muy * fz * 1.01);
    }
}
