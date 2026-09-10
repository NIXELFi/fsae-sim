//! Tyre models.
//!
//! A tyre takes a slip state and a vertical load and returns forces. Crucially
//! it owns its own load sensitivity, which is what lets the solvers stay
//! simple: a double-track model just calls it four times with four loads, and
//! a bicycle model calls it twice per axle with the inner and outer loads and
//! sums. Neither needs to know how μ varies with load.
//!
//! That is a change from the JS build, where the bicycle model computed a
//! load-weighted mean μ for the axle and handed it to the tyre. Summing the two
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

    /// Peak (μx, μy) available at this load. Used by the simplest solvers,
    /// which never form a slip state at all.
    fn peak_mu(&self, fz: f64) -> (f64, f64);

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
/// falloff past the peak, so it cannot spin — which is exactly what you want
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
            mu_y: 1.573,
            load_sensitivity: 0.15,
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
        Self::new(1.5, 1.573, 0.15, crate::vehicle::sdm26().nominal_tyre_load(), 8.5_f64.to_radians(), 0.11, 0.35)
    }

    /// Pneumatic trail (m) at normalised combined slip `s` and load `fz`.
    ///
    /// Brush-model shape: trail falls as (1 - s)^2 and is zero from full
    /// sliding on, which is placed a little past the force peak because a
    /// slick keeps some trail beyond it. Scale grows with the square root of
    /// load, as contact-patch length does. Same constants as the JS build;
    /// the 20 mm is an estimate until a direct Mz fit from TTC data exists.
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
        // normalised slip, which forces B — and with it the cornering
        // stiffness — to absurd values to keep the peak where it belongs.
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
            trail_m: 0.020,
            trail_zero_slip: 1.25,
            trail_ref_load_n: 700.0,
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

    /// Cornering stiffness (N/rad) at a given load — a headline tyre number.
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
        let sx = slip.kappa / self.peak_kappa;
        let sy = slip.alpha.tan() / self.peak_alpha.tan();
        let s = (sx * sx + sy * sy).sqrt();
        if s < 1e-9 {
            return TyreForces { trail: self.pneumatic_trail(0.0, fz), ..TyreForces::zero() };
        }

        let fx0 = mux * fz * self.kx * mf(s * self.peak_kappa, self.bx, self.cx, self.ex);
        let fy0 =
            muy * fz * self.ky * mf((s * self.peak_alpha.tan()).atan(), self.by, self.cy, self.ey);

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

    fn relaxation_length(&self) -> f64 {
        self.relaxation_m
    }
}

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
            (best.0 - 8.5).abs() < 0.5,
            "peak Fy at {:.2} deg, expected 8.5",
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
    fn linear_tyre_saturates() {
        let t = LinearTyre::sdm26();
        let fz = 654.8;
        let (_, muy) = t.peak_mu(fz);
        let f = t.forces(Slip { alpha: 0.5, kappa: 0.0 }, fz);
        assert!(f.fy <= muy * fz * 1.01);
    }
}
