//! Single-zone cylinder pressure, resolved by crank angle.
//!
//! This is the part engine-sim does with a full gas-dynamics network and we do
//! with a textbook single-zone model. The trade is deliberate: engine-sim is
//! trying to *predict* how an engine breathes, and we already know how this one
//! breathes because the Helios CFD module swept it. What we need from the
//! cylinder is the shape and amplitude of the pressure pulse that leaves the
//! exhaust valve, and a single-zone model with a Wiebe burn gives that.
//!
//! The useful consequence of already knowing the torque curve is that the heat
//! release can be *calibrated* to reproduce it. The sound is then generated
//! from a cylinder trace that does the validated amount of work, so the note
//! and the acceleration are answering to the same number.

use crate::engine::EngineSpec;

/// The state of one cylinder, integrated over crank angle.
#[derive(Clone, Debug)]
pub struct Cylinder {
    /// This cylinder's firing TDC, crank degrees in [0, 720).
    pub phase_deg: f32,
    /// Current in-cylinder pressure, Pa.
    pub pressure_pa: f32,
    /// Current in-cylinder temperature, K.
    pub temperature_k: f32,
    /// Volumetric flow out of the exhaust valve on the last step, m^3/s.
    /// Positive is out of the cylinder. This is the source term the exhaust
    /// waveguide integrates -- volumetric rather than mass, because what the
    /// waveguide needs is a particle velocity, and that is flow over area.
    pub exhaust_flow: f32,
    /// Gas velocity in the exhaust port, m/s. State, because the slug of gas in
    /// the port has mass and cannot change velocity instantaneously.
    pub port_velocity: f32,
    /// This cycle's heat release relative to the calibrated one: the
    /// cylinder's own trim times this cycle's variation.
    pub heat_scale: f32,
    /// Crank angle on the previous sample, for spotting intake-valve close.
    /// Negative until the first sample.
    pub last_theta: f32,
}

impl Cylinder {
    pub fn new(phase_deg: f32, ambient_pa: f32, ambient_k: f32) -> Self {
        Self {
            phase_deg,
            pressure_pa: ambient_pa,
            temperature_k: ambient_k,
            exhaust_flow: 0.0,
            port_velocity: 0.0,
            heat_scale: 1.0,
            last_theta: -1.0,
        }
    }

    /// This cylinder's own crank angle, given the engine's.
    #[inline]
    pub fn theta(&self, crank_deg: f32) -> f32 {
        let mut t = crank_deg - self.phase_deg;
        while t < 0.0 {
            t += 720.0;
        }
        while t >= 720.0 {
            t -= 720.0;
        }
        t
    }
}

/// Wiebe mass-fraction burned.
///
/// `x_b(theta) = 1 - exp(-a * ((theta - theta_0) / dtheta)^(m+1))`
#[inline]
pub fn wiebe(theta_deg: f32, start_deg: f32, duration_deg: f32, a: f32, m: f32) -> f32 {
    if theta_deg <= start_deg {
        return 0.0;
    }
    let x = ((theta_deg - start_deg) / duration_deg.max(1e-3)).min(1.0);
    1.0 - (-a * x.powf(m + 1.0)).exp()
}

/// Effective exhaust-valve flow area at a crank angle, m^2.
///
/// A raised cosine lift profile rather than a real cam file. The flow area is
/// the curtain area (pi * D * lift) until it exceeds the port area, at which
/// point the port chokes it -- which is the real behaviour and stops the model
/// dumping the cylinder in one sample at high lift.
pub fn exhaust_flow_area(engine: &EngineSpec, theta_deg: f32) -> f32 {
    let t = &engine.timing;
    let (open, close) = (t.evo_deg, t.evc_deg);
    // The exhaust event can wrap past 720; normalise onto a 0..1 fraction.
    let span = if close > open { close - open } else { close + 720.0 - open };
    let mut rel = theta_deg - open;
    while rel < 0.0 {
        rel += 720.0;
    }
    if rel > span {
        return 0.0;
    }
    let frac = rel / span;
    // Raised cosine: zero at both ends, unity in the middle.
    let lift_frac = 0.5 * (1.0 - (2.0 * core::f32::consts::PI * frac).cos());
    let max_lift = t.exhaust_valve_diameter_m * 0.25; // ~D/4 is a normal peak lift
    let lift = lift_frac * max_lift;
    let curtain = core::f32::consts::PI * t.exhaust_valve_diameter_m * lift;
    let port = core::f32::consts::PI * t.exhaust_valve_diameter_m * t.exhaust_valve_diameter_m * 0.25;
    // Per valve, times the number of exhaust valves: a four-valve head has two.
    t.exhaust_cd * curtain.min(port) * t.exhaust_valve_count
}


/// Crank-angle resolution of the precomputed tables, in bins per 720 degrees.
/// 2880 bins is 0.25 degrees, four times finer than the 1.0 degree the model
/// advances per sample at redline.
pub const TABLE_BINS: usize = 2880;

/// Everything in the cylinder inner loop that is a pure function of crank
/// angle, evaluated once and looked up thereafter.
///
/// This is not premature. The naive version spends most of its time in `powf`,
/// `exp`, `cos` and `sqrt` inside `wiebe`, `exhaust_flow_area` and `volume_at`,
/// all of which are called several times per cylinder per sample and none of
/// which depend on anything but the angle.
///
/// Worth being honest about the size of the win: on its own this barely moved
/// the needle, because the real bottleneck was elsewhere -- see `Scratch` in
/// `exhaust.rs`. It matters more in the JavaScript port, where `Math.pow` is
/// relatively more expensive, and 34 KB of tables is a cheap thing to carry.
#[derive(Clone, Debug)]
pub struct CycleTables {
    pub volume_m3: Vec<f32>,
    pub valve_area_m2: Vec<f32>,
    pub wiebe: Vec<f32>,
}

impl CycleTables {
    pub fn build(engine: &EngineSpec) -> Self {
        let c = &engine.combustion;
        let mut volume_m3 = Vec::with_capacity(TABLE_BINS);
        let mut valve_area_m2 = Vec::with_capacity(TABLE_BINS);
        let mut wiebe_t = Vec::with_capacity(TABLE_BINS);
        for i in 0..TABLE_BINS {
            let theta = i as f32 * 720.0 / TABLE_BINS as f32;
            volume_m3.push(engine.volume_at(theta));
            valve_area_m2.push(exhaust_flow_area(engine, theta));
            // The burn is specified around TDC, which is angle 0 == 720. Sample
            // it on the signed axis so the tail before TDC is not truncated.
            let signed = if theta > 360.0 { theta - 720.0 } else { theta };
            wiebe_t.push(wiebe(signed, c.start_deg, c.duration_deg, c.wiebe_a, c.wiebe_m));
        }
        Self {
            volume_m3,
            valve_area_m2,
            wiebe: wiebe_t,
        }
    }

    #[inline]
    fn lookup(table: &[f32], theta_deg: f32) -> f32 {
        let n = table.len();
        let pos = theta_deg * (n as f32 / 720.0);
        let i = pos.floor();
        let frac = pos - i;
        let a = (i as usize) % n;
        let b = (a + 1) % n;
        table[a] * (1.0 - frac) + table[b] * frac
    }

    #[inline]
    pub fn volume(&self, theta_deg: f32) -> f32 {
        Self::lookup(&self.volume_m3, theta_deg)
    }

    #[inline]
    pub fn valve_area(&self, theta_deg: f32) -> f32 {
        Self::lookup(&self.valve_area_m2, theta_deg)
    }

    #[inline]
    pub fn burned_fraction(&self, theta_deg: f32) -> f32 {
        Self::lookup(&self.wiebe, theta_deg)
    }
}

/// The operating point the cylinder model is currently reproducing.
#[derive(Clone, Copy, Debug)]
pub struct OperatingPoint {
    pub rpm: f32,
    /// Throttle plate position, 0..1. Sets manifold pressure, and with it how
    /// hard the engine is actually working.
    pub throttle: f32,
    /// Indicated torque the cycle should produce, N.m, across the whole engine.
    /// Feed this the Helios CFD sweep value and the audio inherits it.
    pub target_torque_nm: f32,
    /// Exhaust gas temperature, K.
    pub exhaust_k: f32,
}

/// Per-cycle constants derived from an operating point.
#[derive(Clone, Copy, Debug)]
pub struct CycleCalibration {
    /// Pressure in the cylinder at intake valve closing, Pa.
    pub p_ivc: f32,
    /// Temperature at intake valve closing, K.
    pub t_ivc: f32,
    /// Total heat released per cylinder per cycle, J.
    pub heat_release_j: f32,
}

/// Solve for the heat release that makes one cycle do the requested work.
///
/// Indicated work per cylinder per cycle, `W = integral p dV`, relates to
/// engine torque by `T = W * n_cyl / (4 pi)` for a four-stroke. Work is very
/// nearly affine in heat release -- double the fuel and you very nearly double
/// the area of the p-V loop -- so two secant iterations land it, and a third
/// is wasted effort.
pub fn calibrate(engine: &EngineSpec, op: OperatingPoint) -> CycleCalibration {
    let gas = &engine.gas;

    // Manifold pressure. A 20 mm restrictor plus a closed throttle is a hard
    // pumping restriction; wide open at peak torque still does not reach one
    // atmosphere on a restricted engine.
    let map_frac = 0.14 + 0.72 * op.throttle;
    let p_ivc = gas.ambient_pa * map_frac;
    let t_ivc = gas.ambient_k + 40.0 + 90.0 * op.throttle;

    let per_cylinder_target =
        op.target_torque_nm * 4.0 * core::f32::consts::PI / engine.cylinders() as f32;

    // Two probes, then a secant step, then one correction.
    let w0 = indicated_work(engine, p_ivc, t_ivc, 0.0);
    let guess = 600.0; // J, a plausible scale for a 150 cc cylinder
    let w1 = indicated_work(engine, p_ivc, t_ivc, guess);
    let slope = (w1 - w0) / guess;
    let mut q = if slope.abs() < 1e-9 {
        guess
    } else {
        (per_cylinder_target - w0) / slope
    };
    q = q.clamp(0.0, 5_000.0);

    let w2 = indicated_work(engine, p_ivc, t_ivc, q);
    if slope.abs() > 1e-9 {
        q = (q + (per_cylinder_target - w2) / slope).clamp(0.0, 5_000.0);
    }

    CycleCalibration {
        p_ivc,
        t_ivc,
        heat_release_j: q,
    }
}

/// Integrate `p dV` over the closed part of the cycle for a given heat release.
///
/// Closed period runs IVC -> EVO. Outside it the cylinder is connected to a
/// manifold and does pumping work, which is real but is not what sets the
/// torque scale, so it is left out of the calibration.
fn indicated_work(engine: &EngineSpec, p_ivc: f32, _t_ivc: f32, heat_j: f32) -> f32 {
    let c = &engine.combustion;
    let gamma = engine.gas.gamma;
    let ivc = engine.timing.ivc_deg - 720.0; // put IVC before TDC on this axis
    let evo = engine.timing.evo_deg;

    let step = 0.5f32;
    let mut theta = ivc;
    let mut p = p_ivc;
    let mut v = engine.volume_at(theta);
    let mut work = 0.0f32;

    while theta < evo {
        let next = theta + step;
        let v_next = engine.volume_at(next);
        let dv = v_next - v;

        // Heat released over this step, from the Wiebe derivative.
        let xb0 = wiebe(theta, c.start_deg, c.duration_deg, c.wiebe_a, c.wiebe_m);
        let xb1 = wiebe(next, c.start_deg, c.duration_deg, c.wiebe_a, c.wiebe_m);
        let dq = heat_j * (xb1 - xb0);

        // Single-zone first law: dp = (gamma-1)/V * dQ - gamma * p/V * dV
        let dp = (gamma - 1.0) / v * dq - gamma * p / v * dv;

        work += p * dv;
        p = (p + dp).max(1.0);
        v = v_next;
        theta = next;
    }
    work
}

/// Advance one cylinder by `dt` seconds at a given crank position.
///
/// Returns the volumetric flow leaving the exhaust valve, m^3/s, which is what
/// the waveguide wants. `back_pressure_pa` is the pressure the exhaust runner
/// currently presents at the port -- feeding that back is what makes header
/// tuning audible, because a returning wave that arrives while the valve is
/// still open changes how much the cylinder can dump.
pub fn step_cylinder(
    engine: &EngineSpec,
    tables: &CycleTables,
    cyl: &mut Cylinder,
    crank_deg: f32,
    dt: f32,
    cal: &CycleCalibration,
    op: &OperatingPoint,
    back_pressure_pa: f32,
) -> f32 {
    let theta = cyl.theta(crank_deg);
    let gas = &engine.gas;
    let t = &engine.timing;

    let deg_per_s = op.rpm * 6.0; // rpm -> crank degrees per second
    let dtheta = deg_per_s * dt;

    let v = tables.volume(theta);
    let v_next = tables.volume((theta + dtheta) % 720.0);
    let dv = v_next - v;

    let area = tables.valve_area(theta);
    let in_closed_period = is_between(theta, t.ivc_deg, t.evo_deg);
    let exhaust_open = area > 0.0;
    let intake_open = is_between(theta, t.ivo_deg, t.ivc_deg);

    let mut flow_out = 0.0f32;

    if in_closed_period {
        // Compression, combustion, expansion.
        let xb0 = tables.burned_fraction(theta);
        let xb1 = tables.burned_fraction((theta + dtheta) % 720.0);
        let dq = cal.heat_release_j * cyl.heat_scale * (xb1 - xb0);
        let dp = (gas.gamma - 1.0) / v * dq - gas.gamma * cyl.pressure_pa / v * dv;
        cyl.pressure_pa = (cyl.pressure_pa + dp).max(1_000.0);
    } else if exhaust_open {
        // Blowdown, then the exhaust stroke pushing the rest out.
        let dp_across = cyl.pressure_pa - back_pressure_pa;
        let rho = gas.density(cyl.pressure_pa.max(back_pressure_pa), cyl.temperature_k);

        // Orifice flow, regularised near zero pressure difference.
        //
        // The plain law `u = sign(dp) sqrt(2|dp|/rho)` has INFINITE slope at
        // dp = 0. That matters because after blowdown the cylinder sits close
        // to the runner pressure for the whole exhaust stroke, so dp hovers
        // near zero -- and there the square root turns every small returning
        // wave into a large swing in flow, which injects back into the runner
        // and moves the pressure again. The result was a limit cycle: cylinder
        // pressure a clean 76 Hz at 3000 rpm while the valve flow oscillated at
        // 3 kHz and dominated the entire output.
        //
        // Below DP_LAMINAR the law is linear in dp, matched in value at the
        // crossover so the curve is continuous with finite slope everywhere.
        // This is also the more physical choice: flow through a restriction at
        // a small pressure difference is viscosity-dominated and proportional
        // to dp, not to its square root. The square-root law is the fully
        // turbulent limit.
        const DP_LAMINAR: f32 = 1500.0; // Pa
        let mag = dp_across.abs();
        let turbulent = (2.0 * DP_LAMINAR / rho.max(1e-6)).sqrt();
        let mut u_target = if mag >= DP_LAMINAR {
            (2.0 * mag / rho.max(1e-6)).sqrt() * dp_across.signum()
        } else {
            dp_across / DP_LAMINAR * turbulent
        };

        // Choke it. A converging port cannot pass gas faster than the local
        // speed of sound however large the pressure ratio across it, and at
        // blowdown the ratio is enormous -- 50 bar against 1. Left unchoked the
        // incompressible orifice equation returns about 930 m/s, which is not
        // merely too big but qualitatively wrong: it empties the cylinder in a
        // few degrees, slams the waveguide with a supersonic particle velocity,
        // and the reflection that comes back inverts the next pulse.
        let c_cyl = (gas.gamma * gas.r_specific * cyl.temperature_k.max(1.0)).sqrt();
        u_target = u_target.clamp(-c_cyl, c_cyl);

        // An exhaust port flows worse backwards than forwards. The valve seat
        // and the port are shaped for gas leaving the cylinder; reversed, the
        // jet separates and the effective discharge coefficient drops. 0.7 is
        // the usual ballpark. This only matters on a closed throttle, which is
        // exactly where reverse flow dominates -- idle and the overrun.
        if u_target < 0.0 {
            u_target *= 0.7;
        }

        // Port inertance: the slug of gas in the port has mass, so its velocity
        // cannot change instantaneously. A short first-order lag is what that
        // amounts to, and it removes what the regularisation above leaves
        // behind. The time constant is kept well below the blowdown edge --
        // about three samples at 48 kHz against roughly eight even at the
        // limiter -- so it damps the chatter without softening the pulse that
        // makes the sound.
        const PORT_TAU: f32 = 6e-5;
        let k = (dt / PORT_TAU).min(1.0);
        cyl.port_velocity += k * (u_target - cyl.port_velocity);
        let u = cyl.port_velocity;

        let volumetric = area * u; // m^3/s
        // The waveguide wants the volume flow the PIPE sees: gas leaving the
        // cylinder at several bar expands across the valve, isentropically, by
        // (p_cyl / p_ambient)^(1/gamma). Without it the blowdown was no bigger
        // than the displacement flow after it, and the note lost its harmonics.
        // Referenced to ambient, not to the instantaneous back pressure: the
        // latter closes a loop through the pipe that went unstable. Only the
        // source is scaled; the cylinder still empties by its own volume flow.
        let expansion = if u > 0.0 {
            (cyl.pressure_pa / gas.ambient_pa).max(1.0).powf(1.0 / gas.gamma)
        } else {
            1.0
        };
        flow_out = volumetric * expansion;

        // Losing gas drops the pressure; the piston moving changes it too.
        let dp_flow = -gas.gamma * cyl.pressure_pa * (volumetric / v) * dt;
        let dp_vol = -gas.gamma * cyl.pressure_pa / v * dv;
        cyl.pressure_pa = (cyl.pressure_pa + dp_flow + dp_vol).max(1_000.0);
    } else if intake_open {
        // Relax toward manifold pressure as the cylinder fills.
        let tau = 0.35 / deg_per_s.max(1.0) * 180.0;
        let k = (dt / tau.max(1e-6)).min(1.0);
        cyl.pressure_pa += k * (cal.p_ivc - cyl.pressure_pa);
        cyl.temperature_k += k * (cal.t_ivc - cyl.temperature_k);
    } else {
        // Valve overlap and everything else: drift toward ambient.
        cyl.pressure_pa += 0.05 * (gas.ambient_pa - cyl.pressure_pa);
    }

    // Temperature follows pressure and volume through the ideal gas law during
    // the closed period; during exhaust it is pinned to the gas temperature.
    if in_closed_period {
        cyl.temperature_k =
            (cyl.temperature_k * (1.0 + dv / v * (gas.gamma - 1.0)).max(0.1)).clamp(250.0, 3_200.0);
    } else if exhaust_open {
        cyl.temperature_k = op.exhaust_k;
    }

    cyl.exhaust_flow = flow_out;
    flow_out
}

/// Is `theta` inside the arc from `start` to `end`, going forwards, on a
/// 720-degree circle?
#[inline]
pub(crate) fn is_between(theta: f32, start: f32, end: f32) -> bool {
    let span = if end >= start { end - start } else { end + 720.0 - start };
    let mut rel = theta - start;
    while rel < 0.0 {
        rel += 720.0;
    }
    rel <= span
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::cbr600rr_sdm26;

    #[test]
    fn wiebe_runs_from_zero_to_nearly_one() {
        assert_eq!(wiebe(-30.0, -20.0, 50.0, 5.0, 2.0), 0.0);
        let end = wiebe(30.0, -20.0, 50.0, 5.0, 2.0);
        assert!(end > 0.99 && end <= 1.0, "burn ended at {end}");
    }

    #[test]
    fn wiebe_is_monotone() {
        let mut last = -1.0;
        let mut th = -25.0;
        while th < 40.0 {
            let x = wiebe(th, -20.0, 50.0, 5.0, 2.0);
            assert!(x >= last, "went backwards at {th}");
            last = x;
            th += 0.25;
        }
    }

    #[test]
    fn exhaust_valve_is_shut_outside_its_event_and_open_inside() {
        let e = cbr600rr_sdm26();
        assert_eq!(exhaust_flow_area(&e, 0.0), 0.0, "open at firing TDC");
        assert_eq!(exhaust_flow_area(&e, 100.0), 0.0, "open before EVO");
        assert!(exhaust_flow_area(&e, 250.0) > 0.0, "shut mid-exhaust-stroke");
    }

    #[test]
    fn calibration_reproduces_the_requested_torque() {
        let e = cbr600rr_sdm26();
        for &target in &[20.0f32, 40.0, 62.6] {
            let op = OperatingPoint {
                rpm: 9_000.0,
                throttle: 1.0,
                target_torque_nm: target,
                exhaust_k: 1_100.0,
            };
            let cal = calibrate(&e, op);
            let w = indicated_work(&e, cal.p_ivc, cal.t_ivc, cal.heat_release_j);
            let torque = w * e.cylinders() as f32 / (4.0 * core::f32::consts::PI);
            assert!(
                (torque - target).abs() < 0.5,
                "asked for {target} N.m, cycle produced {torque:.2}"
            );
        }
    }

    #[test]
    fn more_heat_means_more_work() {
        let e = cbr600rr_sdm26();
        let p = 90_000.0;
        let a = indicated_work(&e, p, 340.0, 200.0);
        let b = indicated_work(&e, p, 340.0, 400.0);
        assert!(b > a, "work did not rise with heat release: {a} then {b}");
    }

    #[test]
    fn peak_cylinder_pressure_is_physically_plausible() {
        let e = cbr600rr_sdm26();
        let op = OperatingPoint {
            rpm: 9_000.0,
            throttle: 1.0,
            target_torque_nm: 62.6,
            exhaust_k: 1_100.0,
        };
        let cal = calibrate(&e, op);
        let tables = CycleTables::build(&e);
        let mut cyl = Cylinder::new(0.0, e.gas.ambient_pa, e.gas.ambient_k);
        cyl.pressure_pa = cal.p_ivc;
        cyl.temperature_k = cal.t_ivc;

        let dt = 1.0 / 192_000.0;
        let mut crank = e.timing.ivc_deg - 720.0;
        let mut peak = 0.0f32;
        while crank < 180.0 {
            step_cylinder(&e, &tables, &mut cyl, crank, dt, &cal, &op, 101_325.0);
            peak = peak.max(cyl.pressure_pa);
            crank += op.rpm * 6.0 * dt;
        }
        let bar = peak / 1e5;
        // A naturally aspirated restricted four at peak torque lives in the
        // 40-90 bar range. Outside that, something is wrong with the model
        // rather than interestingly different.
        assert!(
            (25.0..110.0).contains(&bar),
            "peak cylinder pressure {bar:.1} bar is not plausible"
        );
    }

    #[test]
    fn is_between_handles_the_wrap_past_720() {
        // Exhaust event 132 -> 372 does not wrap.
        assert!(is_between(200.0, 132.0, 372.0));
        assert!(!is_between(100.0, 132.0, 372.0));
        // Closed period 576 -> 132 does wrap.
        assert!(is_between(600.0, 576.0, 132.0));
        assert!(is_between(50.0, 576.0, 132.0));
        assert!(!is_between(300.0, 576.0, 132.0));
    }
}
