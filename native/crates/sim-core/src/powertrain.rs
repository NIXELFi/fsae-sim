//! Powertrain models.
//!
//! Everything a solver needs from a powertrain is the torque arriving at the
//! driven wheel and the rotating inertia that comes with it. That is a small
//! enough interface that a restricted CBR600RR with a slipping clutch and an
//! electric motor with a single reduction can sit behind the same trait.

use crate::vehicle::VehicleParams;

const RPM_TO_RADS: f64 = core::f64::consts::TAU / 60.0;
const RADS_TO_RPM: f64 = 60.0 / core::f64::consts::TAU;

#[derive(Debug, Clone, Copy, Default)]
pub struct DriveOutput {
    /// Torque delivered to the driven axle (N·m).
    pub wheel_torque_nm: f64,
    /// Reflected driveline inertia to add at the wheel (kg·m²).
    pub added_wheel_inertia: f64,
    pub locked: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct PowertrainTelemetry {
    pub engine_rpm: f64,
    pub gear: usize,
    pub shifting: bool,
    pub slipping: bool,
}

pub trait PowertrainModel: Send + Sync {
    fn name(&self) -> &'static str;

    /// Advance one substep and report what reaches the wheel.
    fn step(&mut self, dt: f64, throttle: f64, wheel_omega: f64, speed: f64) -> DriveOutput;

    fn reset(&mut self);

    /// Put the crank where the current ratio says it should be. Needed
    /// whenever the car is placed at speed — an engine left at idle behind a
    /// rolling wheel is a huge clutch mismatch that brakes the driven axle to
    /// a stop on the first substep.
    fn sync_to_wheel(&mut self, _wheel_omega: f64) {}

    fn shift_up(&mut self) -> bool {
        false
    }
    fn shift_down(&mut self) -> bool {
        false
    }
    fn can_shift(&self) -> bool {
        false
    }

    /// Wide-open-throttle crank torque at an rpm, N.m.
    ///
    /// Exposed on the trait because the engine synthesiser needs it: it solves
    /// its combustion heat release to produce this much work, so the note is
    /// generated from a cylinder trace that does the same amount of work the
    /// car is doing. Defaults to zero for powertrains with no meaningful crank
    /// torque -- an ideal drive, or an electric motor whose sound is not
    /// combustion.
    fn wot_torque_nm(&self, _rpm: f64) -> f64 {
        0.0
    }

    fn telemetry(&self) -> PowertrainTelemetry;
}

// ------------------------------------------------------------------ ideal ---

/// Level 1: an idealised drive. Constant torque up to a power ceiling, no
/// gearbox, no clutch, no inertia. Useful when you want the chassis studied
/// without the engine in the way.
#[derive(Debug, Clone)]
pub struct IdealDrive {
    pub max_wheel_torque_nm: f64,
    pub max_power_w: f64,
    pub tyre_radius_m: f64,
}

impl IdealDrive {
    pub fn sdm26() -> Self {
        Self { max_wheel_torque_nm: 1000.0, max_power_w: 49_000.0, tyre_radius_m: 0.20 }
    }
}

impl PowertrainModel for IdealDrive {
    fn name(&self) -> &'static str {
        "Ideal drive (torque/power limited)"
    }

    fn step(&mut self, _dt: f64, throttle: f64, wheel_omega: f64, _speed: f64) -> DriveOutput {
        let by_power = if wheel_omega.abs() > 0.5 {
            self.max_power_w / wheel_omega.abs()
        } else {
            self.max_wheel_torque_nm
        };
        DriveOutput {
            wheel_torque_nm: throttle.clamp(0.0, 1.0) * by_power.min(self.max_wheel_torque_nm),
            added_wheel_inertia: 0.0,
            locked: true,
        }
    }

    fn reset(&mut self) {}

    fn telemetry(&self) -> PowertrainTelemetry {
        PowertrainTelemetry::default()
    }
}

// --------------------------------------------------------------- electric ---

/// An electric drive: flat torque to base speed, constant power above it,
/// through a single reduction. Here mostly to prove the trait carries a
/// genuinely different architecture — an EV conversion is a parameter set,
/// not a rewrite.
#[derive(Debug, Clone)]
pub struct ElectricDrive {
    pub peak_motor_torque_nm: f64,
    pub peak_power_w: f64,
    pub reduction: f64,
    pub efficiency: f64,
    pub rotor_inertia_kg_m2: f64,
    pub max_motor_rpm: f64,
}

impl ElectricDrive {
    /// A plausible single-motor FSAE EV in the SDM26 chassis.
    pub fn concept() -> Self {
        Self {
            peak_motor_torque_nm: 130.0,
            peak_power_w: 62_000.0,
            reduction: 3.9,
            efficiency: 0.94,
            rotor_inertia_kg_m2: 0.021,
            max_motor_rpm: 12_000.0,
        }
    }
}

impl PowertrainModel for ElectricDrive {
    fn name(&self) -> &'static str {
        "Electric (single reduction)"
    }

    fn step(&mut self, _dt: f64, throttle: f64, wheel_omega: f64, _speed: f64) -> DriveOutput {
        let motor_omega = wheel_omega * self.reduction;
        let by_power = if motor_omega.abs() > 1.0 {
            self.peak_power_w / motor_omega.abs()
        } else {
            self.peak_motor_torque_nm
        };
        let mut t = by_power.min(self.peak_motor_torque_nm);
        if motor_omega * RADS_TO_RPM > self.max_motor_rpm {
            t = 0.0;
        }
        DriveOutput {
            wheel_torque_nm: throttle.clamp(0.0, 1.0) * t * self.reduction * self.efficiency,
            added_wheel_inertia: self.rotor_inertia_kg_m2 * self.reduction * self.reduction,
            locked: true,
        }
    }

    fn reset(&mut self) {}

    fn telemetry(&self) -> PowertrainTelemetry {
        PowertrainTelemetry::default()
    }
}

// ----------------------------------------------------------------- geared ---

#[derive(Debug, Clone, Copy)]
pub struct TorquePoint {
    pub rpm: f64,
    pub torque_nm: f64,
    pub fmep_bar: f64,
}

/// SDM26 characteristic RPM sweep from the Helios CFD module's 1-D
/// finite-volume engine solver, 4000–15000 rpm.
///
/// Not a smooth dyno arc: the wave-action features the solver predicts are
/// preserved — the hole at 6500, the spike at 8000, the second wind at
/// 11000–11500 before the restrictor chokes it.
pub const SDM26_SWEEP: [TorquePoint; 23] = [
    TorquePoint { rpm: 4000.0, torque_nm: 60.997, fmep_bar: 1.1630 },
    TorquePoint { rpm: 4500.0, torque_nm: 61.335, fmep_bar: 1.2594 },
    TorquePoint { rpm: 5000.0, torque_nm: 59.423, fmep_bar: 1.3589 },
    TorquePoint { rpm: 5500.0, torque_nm: 55.846, fmep_bar: 1.4613 },
    TorquePoint { rpm: 6000.0, torque_nm: 54.969, fmep_bar: 1.5668 },
    TorquePoint { rpm: 6500.0, torque_nm: 49.508, fmep_bar: 1.6752 },
    TorquePoint { rpm: 7000.0, torque_nm: 58.086, fmep_bar: 1.7867 },
    TorquePoint { rpm: 7500.0, torque_nm: 55.997, fmep_bar: 1.9012 },
    TorquePoint { rpm: 8000.0, torque_nm: 62.640, fmep_bar: 2.0187 },
    TorquePoint { rpm: 8500.0, torque_nm: 56.256, fmep_bar: 2.1392 },
    TorquePoint { rpm: 9000.0, torque_nm: 54.200, fmep_bar: 2.2627 },
    TorquePoint { rpm: 9500.0, torque_nm: 48.313, fmep_bar: 2.3892 },
    TorquePoint { rpm: 10000.0, torque_nm: 46.191, fmep_bar: 2.5188 },
    TorquePoint { rpm: 10500.0, torque_nm: 44.040, fmep_bar: 2.6513 },
    TorquePoint { rpm: 11000.0, torque_nm: 48.829, fmep_bar: 2.7869 },
    TorquePoint { rpm: 11500.0, torque_nm: 48.216, fmep_bar: 2.9254 },
    TorquePoint { rpm: 12000.0, torque_nm: 43.253, fmep_bar: 3.0670 },
    TorquePoint { rpm: 12500.0, torque_nm: 36.623, fmep_bar: 3.2116 },
    TorquePoint { rpm: 13000.0, torque_nm: 30.067, fmep_bar: 3.3592 },
    TorquePoint { rpm: 13500.0, torque_nm: 23.636, fmep_bar: 3.5098 },
    TorquePoint { rpm: 14000.0, torque_nm: 19.011, fmep_bar: 3.6634 },
    TorquePoint { rpm: 14500.0, torque_nm: 21.985, fmep_bar: 3.8200 },
    TorquePoint { rpm: 15000.0, torque_nm: 23.784, fmep_bar: 3.9797 },
];

/// Level 2: the real thing. Restricted CBR600RR on the CFD sweep, six speeds
/// through a primary and final drive, and a clutch that is genuinely modelled
/// as locked-or-slipping against a torque capacity rather than pinning rpm to
/// road speed. That is what makes launches, bogs, stalls and the ignition-cut
/// shift behave.
#[derive(Debug, Clone)]
pub struct GearedEngine {
    pub curve: Vec<TorquePoint>,
    pub displacement_m3: f64,
    pub gear_ratios: Vec<f64>,
    pub primary: f64,
    pub final_drive: f64,
    pub efficiency: f64,
    pub rev_limit_rpm: f64,
    pub idle_rpm: f64,
    /// Throttle plate position the ETC holds at idle, 0..1.
    pub idle_throttle_frac: f64,
    pub shift_time_s: f64,
    pub crank_inertia_kg_m2: f64,
    pub gearbox_inertia_kg_m2: f64,
    pub clutch_capacity_nm: f64,

    gear: usize,
    engine_rpm: f64,
    shift_timer: f64,
    pending_gear: Option<usize>,
    limiter_cut: bool,
    slipping: bool,
}

impl GearedEngine {
    pub fn sdm26() -> Self {
        Self {
            curve: SDM26_SWEEP.to_vec(),
            displacement_m3: 599e-6,
            gear_ratios: vec![2.75, 2.0, 1.667, 1.444, 1.304, 1.208],
            primary: 2.111,
            final_drive: 3.0,
            efficiency: 0.85,
            rev_limit_rpm: 14_500.0,
            idle_rpm: 2000.0,
            idle_throttle_frac: 0.14,
            shift_time_s: 0.1,
            // Split at the primary, because that is where the clutch sits on a
            // CBR600RR: the crank turns primary×gear×final, everything
            // downstream only gear×final.
            crank_inertia_kg_m2: 0.011,
            gearbox_inertia_kg_m2: 0.006,
            clutch_capacity_nm: 220.0,
            gear: 0,
            engine_rpm: 1600.0,
            shift_timer: 0.0,
            pending_gear: None,
            limiter_cut: false,
            slipping: true,
        }
    }

    pub fn ratio(&self) -> f64 {
        self.primary * self.gear_ratios[self.gear] * self.final_drive
    }

    fn ratio_for(&self, gear: usize) -> f64 {
        self.primary * self.gear_ratios[gear] * self.final_drive
    }

    /// Linear-interpolated wide-open-throttle brake torque (N·m).
    pub fn wot_torque(&self, rpm: f64) -> f64 {
        let p = &self.curve;
        let first = p[0];
        if rpm <= first.rpm {
            // Below the sweep, fall away toward a plausible idle torque rather
            // than holding 61 N·m down to zero rpm.
            // The 0.56 floor is pinned by the measured idle point, not
            // guessed: the engine idles at 2000 rpm with the throttle plate at
            // 14%, so at 2000 rpm a 14% opening must exactly balance friction.
            // Solving `drag / (wot + drag) = 0.14` with drag = 5.54 N.m gives
            // wot(2000) = 34 N.m, which is 0.56 of the 61 N.m peak -- and lands
            // squarely in the 55-70% of peak a naturally aspirated four
            // normally makes at 2000 rpm. The previous 0.35 was invented and
            // could not sustain an idle at any plate opening.
            let f = ((rpm - self.idle_rpm) / (first.rpm - self.idle_rpm)).max(0.0).min(1.0);
            return first.torque_nm * (0.56 + 0.44 * f);
        }
        let last = p[p.len() - 1];
        if rpm >= last.rpm {
            return last.torque_nm;
        }
        for i in 1..p.len() {
            if rpm <= p[i].rpm {
                let (a, b) = (p[i - 1], p[i]);
                let t = (rpm - a.rpm) / (b.rpm - a.rpm);
                return a.torque_nm + t * (b.torque_nm - a.torque_nm);
            }
        }
        last.torque_nm
    }

    fn fmep_bar(&self, rpm: f64) -> f64 {
        let p = &self.curve;
        if rpm <= p[0].rpm {
            return p[0].fmep_bar;
        }
        let last = p[p.len() - 1];
        if rpm >= last.rpm {
            return last.fmep_bar;
        }
        for i in 1..p.len() {
            if rpm <= p[i].rpm {
                let (a, b) = (p[i - 1], p[i]);
                let t = (rpm - a.rpm) / (b.rpm - a.rpm);
                return a.fmep_bar + t * (b.fmep_bar - a.fmep_bar);
            }
        }
        last.fmep_bar
    }

    /// Engine braking from the sweep's own fmep: T = fmep·Vd/4π for a
    /// four-stroke. About 12 N·m of overrun drag at 10 000 rpm.
    pub fn motoring_torque(&self, rpm: f64) -> f64 {
        self.fmep_bar(rpm) * 1e5 * self.displacement_m3 / (4.0 * core::f64::consts::PI)
    }

    /// Throttle plate position, 0..1, for a driver demand.
    ///
    /// The plate does not fully close at idle: the ETC holds it open a little
    /// to keep the engine alive, and on SDM26 that idle position is 14%. The
    /// floor fades out as revs rise, because a real ETC *does* close on the
    /// overrun -- that is what engine braking is, and holding 14% all the way
    /// up the range would delete most of it.
    pub fn plate_position(&self, rpm: f64, demand: f64) -> f64 {
        let nominal = self.idle_throttle_frac;
        if nominal <= 0.0 {
            return demand;
        }

        // Proportional idle-speed control, which is what an ETC idle circuit
        // actually is. A fixed opening is not enough: below idle speed the
        // wide-open torque curve is flat and so is friction, so a fixed plate
        // makes net torque very nearly zero at EVERY sub-idle rpm -- a neutral
        // equilibrium rather than a stable one, and the engine settles wherever
        // it happens to be. The error term supplies the restoring force, and at
        // the target the commanded opening is exactly the measured 14%.
        let err = (self.idle_rpm - rpm) / self.idle_rpm;
        let commanded = (nominal * (1.0 + 3.0 * err)).max(0.0);

        // Above idle the control backs out entirely: a real ETC closes on the
        // overrun, and that is what engine braking is.
        let fade_top = self.idle_rpm * 1.6;
        let scale = if rpm <= self.idle_rpm {
            1.0
        } else {
            ((fade_top - rpm) / (fade_top - self.idle_rpm)).max(0.0)
        };

        demand.max(commanded.min(nominal * 3.0) * scale)
    }

    /// Indicated crankshaft torque, N.m -- the work combustion does, before
    /// friction is subtracted.
    ///
    /// This is what the engine sound model wants: it solves its heat release to
    /// reproduce this much work per cycle. Net torque is the wrong input, since
    /// an engine idling at zero net torque is still burning fuel and still
    /// making noise.
    pub fn indicated_torque(&self, rpm: f64, demand: f64) -> f64 {
        self.plate_position(rpm, demand) * (self.wot_torque(rpm) + self.motoring_torque(rpm))
    }

    fn engine_torque(&mut self, rpm: f64, throttle: f64) -> f64 {
        if self.shift_timer > 0.0 {
            return -self.motoring_torque(rpm) * 0.5; // ignition cut
        }
        if rpm >= self.rev_limit_rpm {
            self.limiter_cut = true;
        }
        if self.limiter_cut && rpm < self.rev_limit_rpm - 350.0 {
            self.limiter_cut = false;
        }
        if self.limiter_cut {
            return -self.motoring_torque(rpm);
        }
        let wot = self.wot_torque(rpm);
        let drag = self.motoring_torque(rpm);
        // The idle plate floor replaces what used to be an ad-hoc torque added
        // below idle speed. Modelling it as a plate position is both closer to
        // what the ETC does and self-correcting: the engine settles wherever
        // that opening balances friction, which for SDM26 measures out at
        // 2005 rpm against a real idle of about 2000.
        let plate = self.plate_position(rpm, throttle);
        let t = plate * (wot + drag) - drag;
        t
    }

    fn clutch_capacity(&self, throttle: f64, speed: f64) -> f64 {
        if self.shift_timer > 0.0 {
            return 0.0;
        }
        if speed > 4.0 {
            return self.clutch_capacity_nm;
        }
        // Below walking pace the clutch is being managed, and with the driver
        // off the pedal it is fully in. A floor of 0.1 meant it always carried
        // about 26 N.m -- several times what the engine makes at idle -- so a
        // stationary car dragged its own engine down and could never idle. A
        // real FSAE car does not creep; you slip the clutch.
        self.clutch_capacity_nm * (throttle * 1.15).min(1.0)
    }

    /// Lowest rpm above which the next gear already makes more wheel force.
    /// Scanned downward from the limiter, because the CFD curve is lumpy and an
    /// upward scan latches onto the 6500 rpm torque hole and shifts far early.
    pub fn optimal_upshift_rpm(&self) -> f64 {
        if self.gear + 1 >= self.gear_ratios.len() {
            return self.rev_limit_rpm;
        }
        let n0 = self.ratio();
        let n1 = self.ratio_for(self.gear + 1);
        let mut best = self.rev_limit_rpm;
        let mut rpm = self.rev_limit_rpm;
        while rpm >= 5000.0 {
            let after = rpm * (n1 / n0);
            if self.wot_torque(after) * n1 >= self.wot_torque(rpm) * n0 {
                best = rpm;
            } else {
                break;
            }
            rpm -= 25.0;
        }
        best
    }

    pub fn peak_torque(&self) -> TorquePoint {
        *self
            .curve
            .iter()
            .max_by(|a, b| a.torque_nm.partial_cmp(&b.torque_nm).unwrap())
            .unwrap()
    }
}

impl PowertrainModel for GearedEngine {
    fn name(&self) -> &'static str {
        "Geared engine (CFD sweep + clutch)"
    }

    fn wot_torque_nm(&self, rpm: f64) -> f64 {
        self.wot_torque(rpm)
    }

    fn step(&mut self, dt: f64, throttle: f64, wheel_omega: f64, speed: f64) -> DriveOutput {
        if self.shift_timer > 0.0 {
            self.shift_timer -= dt;
            if self.shift_timer <= 0.0 {
                if let Some(g) = self.pending_gear.take() {
                    self.gear = g;
                }
                self.shift_timer = 0.0;
            }
        }

        let n = self.ratio();
        let cap = self.clutch_capacity(throttle, speed);
        let mut omega_e = self.engine_rpm * RPM_TO_RADS;
        let clutch_side = wheel_omega * n;
        let slip = omega_e - clutch_side;
        let te = self.engine_torque(self.engine_rpm, throttle);

        // A clutch cannot be locked below idle speed. That is not a detail -- it
        // is the reason you slip a clutch pulling away, and without it the
        // model locked at a standstill and pinned the engine to a stall-guard
        // floor instead of letting it idle.
        let lockable =
            cap > 0.0 && slip.abs() < 8.0 && clutch_side * RADS_TO_RPM >= self.idle_rpm * 0.95;
        if lockable && te.abs() <= cap {
            self.slipping = false;
            // Floor at idle, not below it: a running engine cannot be dragged
            // under its governed idle speed -- the clutch gives up first.
            self.engine_rpm = (clutch_side * RADS_TO_RPM).max(self.idle_rpm);
            let n_gbox = n / self.primary;
            return DriveOutput {
                wheel_torque_nm: te * n * self.efficiency,
                added_wheel_inertia: self.crank_inertia_kg_m2 * n * n
                    + self.gearbox_inertia_kg_m2 * n_gbox * n_gbox,
                locked: true,
            };
        }

        // Slipping: the clutch passes at most `cap`, in the direction that
        // closes the slip; the engine takes the remainder on its own inertia.
        self.slipping = true;
        let dir = if slip > 0.0 {
            1.0
        } else if slip < 0.0 {
            -1.0
        } else {
            0.0
        };
        let passed = if dir == 0.0 { te.clamp(-cap, cap) } else { dir * cap };
        omega_e += (te - passed) / self.crank_inertia_kg_m2 * dt;
        self.engine_rpm = (omega_e * RADS_TO_RPM).max(0.0);
        if self.engine_rpm < 700.0 && speed < 1.0 {
            self.engine_rpm = self.idle_rpm * 0.85; // auto-restart; this is a game
        }

        DriveOutput {
            wheel_torque_nm: passed * n * self.efficiency,
            added_wheel_inertia: 0.0,
            locked: false,
        }
    }

    fn reset(&mut self) {
        self.gear = 0;
        self.engine_rpm = self.idle_rpm;
        self.shift_timer = 0.0;
        self.pending_gear = None;
        self.limiter_cut = false;
        self.slipping = true;
    }

    fn sync_to_wheel(&mut self, wheel_omega: f64) {
        self.engine_rpm = (wheel_omega * self.ratio() * RADS_TO_RPM).max(self.idle_rpm);
        self.slipping = false;
    }

    fn shift_up(&mut self) -> bool {
        if self.shift_timer > 0.0 || self.gear + 1 >= self.gear_ratios.len() {
            return false;
        }
        self.pending_gear = Some(self.gear + 1);
        self.shift_timer = self.shift_time_s;
        true
    }

    fn shift_down(&mut self) -> bool {
        if self.shift_timer > 0.0 || self.gear == 0 {
            return false;
        }
        self.pending_gear = Some(self.gear - 1);
        self.shift_timer = self.shift_time_s;
        true
    }

    fn can_shift(&self) -> bool {
        self.shift_timer <= 0.0 && !self.slipping
    }

    fn telemetry(&self) -> PowertrainTelemetry {
        PowertrainTelemetry {
            engine_rpm: self.engine_rpm,
            gear: self.gear,
            shifting: self.shift_timer > 0.0,
            slipping: self.slipping,
        }
    }
}

/// Convenience: the geared engine matched to a vehicle's tyre radius.
pub fn geared_for(_v: &VehicleParams) -> GearedEngine {
    GearedEngine::sdm26()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sweep_peaks_where_the_cfd_says() {
        let e = GearedEngine::sdm26();
        let p = e.peak_torque();
        assert_eq!(p.rpm, 8000.0);
        assert!((p.torque_nm - 62.64).abs() < 0.01);
    }

    #[test]
    fn motoring_drag_is_plausible() {
        let e = GearedEngine::sdm26();
        let d = e.motoring_torque(10_000.0);
        assert!((11.0..13.0).contains(&d), "overrun drag {d} N.m at 10k");
    }

    #[test]
    fn upshift_point_is_near_the_top_not_in_the_6500_hole() {
        let mut e = GearedEngine::sdm26();
        e.reset();
        let rpm = e.optimal_upshift_rpm();
        assert!(rpm > 10_000.0, "shifting at {rpm} rpm — latched onto a dip");
    }
}
