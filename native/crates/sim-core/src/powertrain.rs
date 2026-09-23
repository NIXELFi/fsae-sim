//! Powertrain models.
//!
//! Everything a solver needs from a powertrain is the torque arriving at the
//! driven wheel and the rotating inertia that comes with it. That is a small
//! enough interface that a restricted CBR600RR with a slipping clutch and an
//! electric motor with a single reduction can sit behind the same trait.

use crate::vehicle::VehicleParams;

/// How far under the limiter the auto-blip on a downshift lands the engine.
const LIMITER_BAND_RPM: f64 = 300.0;
/// How long the clutch stays dumped after launch control is released, s.
const LAUNCH_DUMP_S: f64 = 0.8;
/// Rev limiter hysteresis, rpm; see `GearedEngine::rev_limit_hyst_rpm`.
pub const REV_LIMIT_HYST_RPM: f64 = 150.0;
/// Launch control hysteresis, rpm; see `GearedEngine::launch_hyst_rpm`.
pub const LAUNCH_HYST_RPM: f64 = 400.0;

/// Crank torque to wheel torque through the ratio and the driveline
/// efficiency. Losses always oppose motion: drive is reduced by them, engine
/// braking is increased by them.
fn to_wheel(crank_nm: f64, n: f64, eff: f64) -> f64 {
    if crank_nm > 0.0 { crank_nm * n * eff } else { (crank_nm * n) / eff }
}

const RPM_TO_RADS: f64 = core::f64::consts::TAU / 60.0;
const RADS_TO_RPM: f64 = 60.0 / core::f64::consts::TAU;

#[derive(Debug, Clone, Copy, Default)]
pub struct DriveOutput {
    /// Torque delivered to the driven axle (N.m).
    pub wheel_torque_nm: f64,
    /// Reflected driveline inertia to add at the wheel (kg.m^2).
    pub added_wheel_inertia: f64,
    pub locked: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct PowertrainTelemetry {
    pub engine_rpm: f64,
    pub gear: usize,
    pub shifting: bool,
    pub slipping: bool,
    /// The rev limiter (or launch control's) has the ignition cut right now.
    pub limiter_cut: bool,
}

pub trait PowertrainModel: Send + Sync {
    fn name(&self) -> &'static str;

    /// For a host that needs the concrete model back. See `TyreModel`.
    fn as_any_mut(&mut self) -> Option<&mut dyn core::any::Any> {
        None
    }

    /// Would a downshift right now over-rev the engine? Powertrains without
    /// gears have nothing to over-rev.
    fn downshift_safe(&self, _wheel_omega: f64) -> bool {
        true
    }

    /// Rpm above which the next gear already makes more wheel force.
    fn optimal_upshift_rpm(&self) -> f64 {
        f64::INFINITY
    }

    /// Indicated crank torque for a throttle demand (what the sound model
    /// wants), and the throttle plate position the ETC actually holds.
    fn indicated_torque_nm(&self, _rpm: f64, _demand: f64) -> f64 {
        0.0
    }
    fn plate_position(&self, _rpm: f64, demand: f64) -> f64 {
        demand
    }

    /// Advance one substep and report what reaches the wheel.
    fn step(&mut self, dt: f64, throttle: f64, wheel_omega: f64, speed: f64) -> DriveOutput;

    /// Launch control held. Only a geared engine with a clutch has anything to
    /// do with this, so it defaults to doing nothing.
    fn set_launch(&mut self, _held: bool) {}

    fn reset(&mut self);

    /// Put the crank where the current ratio says it should be. Needed
    /// whenever the car is placed at speed -- an engine left at idle behind a
    /// rolling wheel is a huge clutch mismatch that brakes the driven axle to
    /// a stop on the first substep.
    fn sync_to_wheel(&mut self, _wheel_omega: f64) {}

    /// Select a gear directly, no shift cut. For placing a car at speed;
    /// single-speed drives ignore it.
    fn set_gear(&mut self, _gear: usize) {}

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
/// genuinely different architecture -- an EV conversion is a parameter set,
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

/// SDM26 torque sweep. Since 2026-09-18 this is the MEASURED chassis-dyno
/// curve (Downloads/SDM.CSV, 4500-13500 rpm, measured at the wheels) divided
/// by the drivetrain efficiency to give flywheel torque; below 4500 rpm the
/// Helios CFD sweep's shape is carried, scaled to meet it, and above 13500
/// the measurement's own slope is continued. The fmep column is still the
/// CFD sweep's. Generated together with sim/data/sdm26-torque.json.
pub const SDM26_SWEEP: [TorquePoint; 23] = [
TorquePoint { rpm: 4000.0, torque_nm: 35.461, fmep_bar: 1.1630 },
    TorquePoint { rpm: 4500.0, torque_nm: 35.657, fmep_bar: 1.2594 },
    TorquePoint { rpm: 5000.0, torque_nm: 39.074, fmep_bar: 1.3589 },
    TorquePoint { rpm: 5500.0, torque_nm: 48.603, fmep_bar: 1.4613 },
    TorquePoint { rpm: 6000.0, torque_nm: 55.062, fmep_bar: 1.5668 },
    TorquePoint { rpm: 6500.0, torque_nm: 51.036, fmep_bar: 1.6752 },
    TorquePoint { rpm: 7000.0, torque_nm: 49.542, fmep_bar: 1.7867 },
    TorquePoint { rpm: 7500.0, torque_nm: 49.439, fmep_bar: 1.9012 },
    TorquePoint { rpm: 8000.0, torque_nm: 53.055, fmep_bar: 2.0187 },
    TorquePoint { rpm: 8500.0, torque_nm: 57.820, fmep_bar: 2.1392 },
    TorquePoint { rpm: 9000.0, torque_nm: 55.678, fmep_bar: 2.2627 },
    TorquePoint { rpm: 9500.0, torque_nm: 53.540, fmep_bar: 2.3892 },
    TorquePoint { rpm: 10000.0, torque_nm: 50.361, fmep_bar: 2.5188 },
    TorquePoint { rpm: 10500.0, torque_nm: 45.582, fmep_bar: 2.6513 },
    TorquePoint { rpm: 11000.0, torque_nm: 43.355, fmep_bar: 2.7869 },
    TorquePoint { rpm: 11500.0, torque_nm: 40.337, fmep_bar: 2.9254 },
    TorquePoint { rpm: 12000.0, torque_nm: 39.496, fmep_bar: 3.0670 },
    TorquePoint { rpm: 12500.0, torque_nm: 40.634, fmep_bar: 3.2116 },
    TorquePoint { rpm: 13000.0, torque_nm: 38.801, fmep_bar: 3.3592 },
    TorquePoint { rpm: 13500.0, torque_nm: 35.285, fmep_bar: 3.5098 },
    TorquePoint { rpm: 14000.0, torque_nm: 32.610, fmep_bar: 3.6634 },
    TorquePoint { rpm: 14500.0, torque_nm: 29.936, fmep_bar: 3.8200 },
    TorquePoint { rpm: 15000.0, torque_nm: 27.261, fmep_bar: 3.9797 },
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
    /// Rev limiter hysteresis: the ignition is cut AT the limit and comes
    /// back this far under it. See `engine_torque`.
    pub rev_limit_hyst_rpm: f64,
    pub idle_rpm: f64,
    /// Crank speed a driver holds on the clutch off the line, and what launch
    /// control limits the engine to while it is held.
    pub launch_rpm: f64,
    /// Launch control's hysteresis: wider than the main limiter's, so the
    /// engine bounces hard on it, as the real one does.
    pub launch_hyst_rpm: f64,
    /// Launch control: driver holding it, and the window after they drop it
    /// during which the clutch is dumped rather than fed in.
    pub launch_held: bool,
    pub launch_dump_s: f64,
    /// Throttle plate position the ETC holds at idle, 0..1.
    pub idle_throttle_frac: f64,
    pub shift_time_s: f64,
    /// After the cut, how long the torque takes to come back (smoothstep).
    pub shift_reintro_s: f64,
    pub crank_inertia_kg_m2: f64,
    pub gearbox_inertia_kg_m2: f64,
    pub clutch_capacity_nm: f64,
    /// Rear wheel pair inertia at the wheel, for the clutch's landing torque.
    /// Mirrors 2 x VehicleParams::wheel_inertia_rear_kg_m2; the rig keeps it
    /// in step when that parameter is edited.
    pub wheel_side_inertia_kg_m2: f64,

    gear: usize,
    engine_rpm: f64,
    shift_timer: f64,
    /// >0 while the torque is being fed back in after the cut.
    reintro_timer: f64,
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
            rev_limit_hyst_rpm: REV_LIMIT_HYST_RPM,
            idle_rpm: 2000.0,
            launch_rpm: 7000.0,
            launch_hyst_rpm: LAUNCH_HYST_RPM,
            launch_held: false,
            launch_dump_s: 0.0,
            // Re-solved for the measured curve; see params.js. The real
            // engine makes far less below 4000 rpm than the CFD sweep said,
            // so the plate sits further open to hold 2000 rpm.
            idle_throttle_frac: 0.22,
            // The ignition cut: 80-100 ms off the real paddle shift (see
            // params.js), and the torque comes back over 50 ms on a
            // smoothstep rather than in a step.
            shift_time_s: 0.09,
            shift_reintro_s: 0.05,
            // Split at the primary, because that is where the clutch sits on a
            // CBR600RR: the crank turns primaryxgearxfinal, everything
            // downstream only gearxfinal.
            crank_inertia_kg_m2: 0.011,
            gearbox_inertia_kg_m2: 0.006,
            clutch_capacity_nm: 220.0,
            wheel_side_inertia_kg_m2: 2.0 * 0.152,
            gear: 0,
            engine_rpm: 1600.0,
            shift_timer: 0.0,
            reintro_timer: 0.0,
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

    /// Linear-interpolated wide-open-throttle brake torque (N.m).
    /// Launch control. Held, the engine sits on the LC limiter with the clutch
    /// out; released, the clutch is DUMPED rather than fed in. See powertrain.js.
    pub fn set_launch(&mut self, held: bool) {
        if self.launch_held && !held {
            self.launch_dump_s = LAUNCH_DUMP_S;
        }
        self.launch_held = held;
    }

    /// The rev limit in force; launch control lowers it while it is held.
    pub fn limit_rpm(&self) -> f64 {
        if self.launch_held {
            self.launch_rpm.min(self.rev_limit_rpm)
        } else {
            self.rev_limit_rpm
        }
    }

    pub fn wot_torque(&self, rpm: f64) -> f64 {
        let p = &self.curve;
        let first = p[0];
        if rpm <= first.rpm {
            // Below the sweep, fall away toward a plausible idle torque rather
            // than holding 61 N.m down to zero rpm.
            // The 0.56 floor is pinned by the measured idle point, not
            // guessed. The engine idles at 2000 rpm with the throttle plate at
            // 22% (`idle_throttle_frac`), so at 2000 rpm a 22% opening must
            // exactly balance friction. Solving `drag / (wot + drag) = 0.22`
            // with drag = 5.543 N.m gives wot(2000) = 19.65 N.m, and this
            // floor is a fraction of the SWEEP'S FIRST POINT -- 35.461 N.m at
            // 4000 rpm, not the peak -- so 19.65 / 35.461 = 0.554, which is
            // the 0.56 below.
            //
            // That is 34% of the 57.82 N.m peak, lower than the 55-70% a
            // naturally aspirated four is usually quoted at 2000 rpm. This
            // engine peaks near 11k, so a low fraction at 2000 is expected;
            // the number to be suspicious of is the drag figure, not the
            // floor, and it comes from the same dyno sheet as the sweep.
            //
            // (An earlier version of this comment derived 0.56 from a 14%
            // plate and a 61 N.m peak. Both numbers moved with the measured
            // dyno curve; the plate was re-solved and the code is correct, but
            // the derivation was left behind and no longer checked out -- it
            // produced 34% of peak while claiming 55-70%.)
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

    /// Engine braking from the sweep's own fmep: T = fmep.Vd/4pi for a
    /// four-stroke. About 12 N.m of overrun drag at 10 000 rpm.
    pub fn motoring_torque(&self, rpm: f64) -> f64 {
        self.fmep_bar(rpm) * 1e5 * self.displacement_m3 / (4.0 * core::f64::consts::PI)
    }

    /// Throttle plate position, 0..1, for a driver demand.
    ///
    /// The plate does not fully close at idle: the ETC holds it open a little
    /// to keep the engine alive, and on SDM26 that idle position is 22%
    /// (`idle_throttle_frac`; see the note in `sim/tools/validate.js`). The
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
        // On the limiter the ignition is cut: no combustion, no note.
        if self.limiter_cut {
            return 0.0;
        }
        self.plate_position(rpm, demand) * (self.wot_torque(rpm) + self.motoring_torque(rpm))
    }

    /// How much of the engine's torque is back after a shift, 0..1. See
    /// `reintroFraction` in powertrain.js -- the two must agree exactly.
    fn reintro_fraction(&self) -> f64 {
        if self.shift_reintro_s <= 0.0 || self.reintro_timer <= 0.0 {
            return 1.0;
        }
        let x = (1.0 - self.reintro_timer / self.shift_reintro_s).clamp(0.0, 1.0);
        x * x * (3.0 - 2.0 * x)
    }

    fn engine_torque(&mut self, rpm: f64, throttle: f64) -> f64 {
        if self.shift_timer > 0.0 {
            return -self.motoring_torque(rpm) * 0.5; // ignition cut
        }
        let wot = self.wot_torque(rpm);
        let drag = self.motoring_torque(rpm);
        // The idle plate floor replaces what used to be an ad-hoc torque added
        // below idle speed. Modelling it as a plate position is both closer to
        // what the ETC does and self-correcting: the engine settles wherever
        // that opening balances friction, which for SDM26 measures out at
        // 2005 rpm against a real idle of about 2000.
        let plate = self.plate_position(rpm, throttle);
        let mut t = plate * (wot + drag) - drag;
        // Hard-cut limiter with hysteresis (see powertrain.js): the ignition
        // is cut at the limit and comes back `hyst` under it, so the engine
        // bounces -- a little on the rev limiter, hard on launch control.
        let limit = self.limit_rpm();
        let hyst = if self.launch_held { self.launch_hyst_rpm } else { self.rev_limit_hyst_rpm };
        if rpm >= limit {
            self.limiter_cut = true;
        } else if rpm <= limit - hyst {
            self.limiter_cut = false;
        }
        if self.limiter_cut {
            t = -drag;
        }
        // Coming back from a shift: blend from the cut's value to the full one.
        let f = self.reintro_fraction();
        if f < 1.0 {
            let cut = -self.motoring_torque(rpm) * 0.5;
            t = cut + (t - cut) * f;
        }
        t
    }

    /// `_speed` is no longer consulted: the clutch is decided by the two
    /// shaft speeds, which is what actually settles whether it is slipping.
    /// `te` is the engine's torque this step (idle plate included), which the
    /// pull-away capacity is sized from.
    fn clutch_capacity(&self, throttle: f64, speed: f64, clutch_side_rpm: f64, te: f64) -> f64 {
        if self.shift_timer > 0.0 {
            return 0.0;
        }
        // Off the throttle below idle speed the clutch comes in (driver or
        // slipper clutch); see powertrain.js.
        if throttle < 0.05 && clutch_side_rpm < self.idle_rpm * 0.95 {
            return 0.0;
        }
        // The driveline turning the engine rather than the other way round:
        // the clutch is in, and this is engine braking, not a launch.
        if clutch_side_rpm >= self.engine_rpm {
            return self.clutch_capacity_nm;
        }
        // Caught up: there is nothing left to slip.
        let target = self.launch_rpm;
        // Launch control held: the clutch is in and the engine is sitting on
        // the LC limiter. Nothing goes through until the pedal comes up -- and
        // then it is DUMPED, not fed in; see powertrain.js.
        if self.launch_held {
            return 0.0;
        }
        if self.launch_dump_s > 0.0 {
            return self.clutch_capacity_nm;
        }
        if clutch_side_rpm >= target {
            return self.clutch_capacity_nm;
        }
        // Rolling: the clutch is in; see powertrain.js.
        if speed > 12.0 {
            return self.clutch_capacity_nm;
        }
        // Pulling away: a fraction of what the engine is making RIGHT NOW --
        // half of it at rest, all of it at the launch rpm, more above. See
        // powertrain.js for why it must not be anchored to the launch rpm.
        // Wide-open torque x pedal, or what the engine is actually making if
        // that is more (at a trickle the idle plate is). See powertrain.js.
        let avail = (self.wot_torque_nm(self.engine_rpm) * (throttle * 1.15).min(1.0)).max(te);
        // The rpm the left foot holds the engine at: the launch rpm at full
        // throttle, near idle at a trickle -- a driver creeping off at 15 %
        // does not rev it to 7000 first. See powertrain.js.
        let hold = self.idle_rpm + (target - self.idle_rpm) * (throttle * 1.15).clamp(0.0, 1.0);
        let frac = 0.5 + 0.5 * (self.engine_rpm / hold.max(1.0));
        let slipping = (avail * frac).clamp(0.0, self.clutch_capacity_nm);
        // Blended into the full capacity as the slip closes, and the blend is
        // CONTINUOUS on purpose; see powertrain.js.
        // Blended into the full capacity as the DRIVELINE spins up toward the
        // launch rpm -- not on the slip; see powertrain.js.
        // 2026-09-22: blended on the SLIP closing, not on the driveline's
        // speed. Keyed to driveline speed the capacity reached the full
        // 220 N.m while the crank was still far above the clutch, so a
        // pull-away dragged the engine to ~1100 rpm and the car crawled out
        // of the low-rpm hole for a second. The old objection to a slip key --
        // the clutch giving way in ordinary low-gear driving -- came from the
        // clutch never re-locking (see `step`), which is fixed; locked, the
        // two speeds are equal and this is the full capacity. See powertrain.js.
        let ratio = (clutch_side_rpm / self.engine_rpm.max(1.0)).clamp(0.0, 1.0);
        // r^8 by squaring, the same three multiplies as the JS, for parity.
        let r2 = ratio * ratio;
        let r4 = r2 * r2;
        let w = r4 * r4;
        slipping + (self.clutch_capacity_nm - slipping) * w
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

    fn as_any_mut(&mut self) -> Option<&mut dyn core::any::Any> {
        Some(self)
    }

    fn downshift_safe(&self, wheel_omega: f64) -> bool {
        if self.gear == 0 {
            return false;
        }
        wheel_omega * self.ratio_for(self.gear - 1) * RADS_TO_RPM < self.rev_limit_rpm
    }

    fn optimal_upshift_rpm(&self) -> f64 {
        GearedEngine::optimal_upshift_rpm(self)
    }

    fn indicated_torque_nm(&self, rpm: f64, demand: f64) -> f64 {
        self.indicated_torque(rpm, demand)
    }

    fn plate_position(&self, rpm: f64, demand: f64) -> f64 {
        GearedEngine::plate_position(self, rpm, demand)
    }

    fn wot_torque_nm(&self, rpm: f64) -> f64 {
        self.wot_torque(rpm)
    }

    fn set_launch(&mut self, held: bool) {
        GearedEngine::set_launch(self, held);
    }

    fn step(&mut self, dt: f64, throttle: f64, wheel_omega: f64, speed: f64) -> DriveOutput {
        if self.launch_dump_s > 0.0 {
            self.launch_dump_s = (self.launch_dump_s - dt).max(0.0);
        }
        if self.shift_timer > 0.0 {
            self.shift_timer -= dt;
            if self.shift_timer <= 0.0 {
                if let Some(g) = self.pending_gear.take() {
                    let downshift = g < self.gear;
                    self.gear = g;
                    // Auto-blip on a downshift: a paddle-shifted car rev-matches
                    // before the clutch comes back in. Without it the clutch
                    // "landed" by dragging the rear axle down to crank speed --
                    // in first at 15 m/s the crank's reflected inertia is four
                    // times the wheel side and 220 N.m at the primary is
                    // 3400 N.m at the axle against a tyre that passes ~540 --
                    // and the rear sat at 20% of road speed with no lateral
                    // grip (measured on the rig and offline, 2026-09-17).
                    if downshift {
                        let matched = wheel_omega * self.ratio() * RADS_TO_RPM;
                        self.engine_rpm = matched.clamp(self.idle_rpm, self.rev_limit_rpm - LIMITER_BAND_RPM);
                    }
                }
                self.shift_timer = 0.0;
                self.reintro_timer = self.shift_reintro_s;
            }
        }
        if self.reintro_timer > 0.0 {
            self.reintro_timer = (self.reintro_timer - dt).max(0.0);
        }

        let n = self.ratio();
        let mut omega_e = self.engine_rpm * RPM_TO_RADS;
        let clutch_side = wheel_omega * n;
        let slip = omega_e - clutch_side;
        let te = self.engine_torque(self.engine_rpm, throttle);
        let cap = self.clutch_capacity(throttle, speed, clutch_side * RADS_TO_RPM, te);

        // A clutch cannot be locked below idle speed. That is not a detail -- it
        // is the reason you slip a clutch pulling away, and without it the
        // model locked at a standstill and pinned the engine to a stall-guard
        // floor instead of letting it idle.
        //
        // It locks when the torque that closes the slip this step is within
        // the clutch's capacity -- it is stuck, whatever the slip -- as well
        // as below 8 rad/s. 2026-09-22: with only the 8 rad/s test, the tyre's
        // reaction (which `stick` does not see) held ~29 rad/s of permanent
        // slip under hard acceleration: the clutch never read as locked,
        // `can_shift` stayed false, auto-shift held first to 14,300 rpm and
        // rpm read 2-3 % high everywhere.
        let n_gbox = n / self.primary;
        let iw_side = self.wheel_side_inertia_kg_m2 + self.gearbox_inertia_kg_m2 * n_gbox * n_gbox;
        let stick = (slip / dt + te / self.crank_inertia_kg_m2) / (1.0 / self.crank_inertia_kg_m2 + (n * n) / iw_side);
        let lockable = cap > 0.0
            && (slip.abs() < 8.0 || stick.abs() <= cap)
            && clutch_side * RADS_TO_RPM >= self.idle_rpm * 0.95;
        if lockable && te.abs() <= cap {
            self.slipping = false;
            // Floor at idle, not below it: a running engine cannot be dragged
            // under its governed idle speed -- the clutch gives up first.
            self.engine_rpm = (clutch_side * RADS_TO_RPM).max(self.idle_rpm);
            let n_gbox = n / self.primary;
            return DriveOutput {
                wheel_torque_nm: to_wheel(te, n, self.efficiency),
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
        // Stiction first: the torque that lands the engine exactly on the
        // clutch-side speed this step (see powertrain.js). Only a slip that
        // needs more than the clutch has slips at capacity.
        // Both sides move (see powertrain.js): crank at Ie, wheel side at
        // IwSide, coupled through n. `stick` is computed above, same
        // expression, same order.
        let passed = if stick.abs() <= cap {
            stick
        } else if dir == 0.0 {
            te.clamp(-cap, cap)
        } else {
            dir * cap
        };
        omega_e += (te - passed) / self.crank_inertia_kg_m2 * dt;
        self.engine_rpm = (omega_e * RADS_TO_RPM).max(0.0);
        if self.engine_rpm < 700.0 && speed < 1.0 {
            self.engine_rpm = self.idle_rpm * 0.85; // auto-restart; this is a game
        }

        DriveOutput {
            wheel_torque_nm: to_wheel(passed, n, self.efficiency),
            added_wheel_inertia: 0.0,
            locked: false,
        }
    }

    fn reset(&mut self) {
        self.gear = 0;
        self.engine_rpm = self.idle_rpm;
        self.shift_timer = 0.0;
        self.reintro_timer = 0.0;
        self.pending_gear = None;
        self.limiter_cut = false;
        self.slipping = true;
        // The launch timer is free-running and pins the clutch at its full
        // capacity for 0.8 s after a dump. Leaving it set across a reset meant
        // that pressing restart within that window started the new run with a
        // locked clutch, no matter what the throttle or the engine speed were
        // doing -- and it corrects itself so quickly that it reads as the car
        // being twitchy rather than as a bug.
        self.launch_held = false;
        self.launch_dump_s = 0.0;
    }

    fn sync_to_wheel(&mut self, wheel_omega: f64) {
        self.engine_rpm = (wheel_omega * self.ratio() * RADS_TO_RPM).max(self.idle_rpm);
        self.slipping = false;
    }

    fn set_gear(&mut self, gear: usize) {
        self.gear = gear.min(self.gear_ratios.len().saturating_sub(1));
        self.pending_gear = None;
        self.shift_timer = 0.0;
        self.reintro_timer = 0.0;
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
            limiter_cut: self.limiter_cut,
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
    fn sweep_peaks_where_the_dyno_says() {
        let e = GearedEngine::sdm26();
        let p = e.peak_torque();
        assert_eq!(p.rpm, 8500.0);
        assert!((p.torque_nm - 57.820).abs() < 0.01, "peak {} N.m", p.torque_nm);
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
        assert!(rpm > 10_000.0, "shifting at {rpm} rpm -- latched onto a dip");
    }
}
