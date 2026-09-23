//! What an engine *is*, as data.
//!
//! Everything the synthesiser needs is in `EngineSpec`, and nothing in it is
//! specific to the SDM26. Adding a different engine -- a single, a V-twin, a
//! cross-plane V8 -- means writing another one of these, not touching the
//! model. That is the whole point: the sound comes out different because the
//! bore, the firing angles and the pipe lengths are different, which is also
//! why it comes out different on a real engine.

/// Ambient and gas properties. Separated out because exhaust gas is hot, and
/// the speed of sound in it is what sets the tuned length of the header.
#[derive(Clone, Copy, Debug)]
pub struct GasProperties {
    /// Ambient pressure, Pa.
    pub ambient_pa: f32,
    /// Ambient temperature, K.
    pub ambient_k: f32,
    /// Exhaust gas temperature at full load, K. Real FSAE headers run hot.
    pub exhaust_k_max: f32,
    /// Exhaust gas temperature at idle / overrun, K.
    pub exhaust_k_min: f32,
    /// Ratio of specific heats for the burned gas.
    pub gamma: f32,
    /// Specific gas constant, J/(kg K).
    pub r_specific: f32,
}

impl Default for GasProperties {
    fn default() -> Self {
        Self {
            ambient_pa: 101_325.0,
            ambient_k: 293.0,
            exhaust_k_max: 1_150.0,
            exhaust_k_min: 650.0,
            gamma: 1.33,
            r_specific: 287.0,
        }
    }
}

impl GasProperties {
    /// Speed of sound in the exhaust at a given gas temperature.
    ///
    /// At 1150 K this is about 660 m/s against 343 in ambient air, so a header
    /// that is tuned at temperature is badly mistuned cold. Modelling it means
    /// the note actually shifts as the engine comes up to load.
    pub fn speed_of_sound(&self, temperature_k: f32) -> f32 {
        (self.gamma * self.r_specific * temperature_k.max(1.0)).sqrt()
    }

    /// Gas density at a temperature and pressure.
    pub fn density(&self, pressure_pa: f32, temperature_k: f32) -> f32 {
        pressure_pa / (self.r_specific * temperature_k.max(1.0))
    }
}

/// A single exhaust pipe run.
#[derive(Clone, Copy, Debug)]
pub struct PipeSpec {
    /// Length, m.
    pub length_m: f32,
    /// Cross-sectional area, m^2.
    pub area_m2: f32,
    /// Fraction of a wave's amplitude lost per traverse, to wall friction and
    /// heat transfer. Small, but without it the waveguide rings forever.
    pub loss: f32,
    /// Cutoff of the per-traverse loss filter, Hz.
    ///
    /// Real pipe losses are frequency-dependent -- viscous and thermal losses
    /// in the boundary layer grow roughly as the square root of frequency, so a
    /// wave loses its high frequencies fastest. A flat multiplier leaves every
    /// mode with the same Q, and the consequence is audible: at low rpm the
    /// combustion pulses are far apart and the lightly damped 1-2 kHz pipe
    /// modes ring on between them until they are all you can hear.
    pub damping_hz: f32,
}

impl PipeSpec {
    pub fn from_diameter(length_m: f32, diameter_m: f32, loss: f32, damping_hz: f32) -> Self {
        Self {
            length_m,
            area_m2: core::f32::consts::PI * diameter_m * diameter_m * 0.25,
            loss,
            damping_hz,
        }
    }
}

/// Valve timing, in crank degrees, with 0 = TDC of the firing stroke.
///
/// The convention matters and is easy to get backwards: 0 is combustion TDC,
/// 0..180 is expansion, 180..360 exhaust, 360..540 intake, 540..720
/// compression. So EVO at 130 is 50 degrees before BDC on the power stroke,
/// which is where a real cam opens it.
#[derive(Clone, Copy, Debug)]
pub struct ValveTiming {
    pub evo_deg: f32,
    pub evc_deg: f32,
    pub ivo_deg: f32,
    pub ivc_deg: f32,
    /// Effective valve curtain diameter, m, per valve.
    pub exhaust_valve_diameter_m: f32,
    /// Exhaust valves per cylinder. A four-valve head has two.
    pub exhaust_valve_count: f32,
    /// Discharge coefficient of the exhaust port.
    pub exhaust_cd: f32,
}

/// Combustion, as a Wiebe heat-release function.
#[derive(Clone, Copy, Debug)]
pub struct CombustionSpec {
    /// Crank angle at which burning starts, degrees (negative = before TDC).
    pub start_deg: f32,
    /// Burn duration, crank degrees (10-90% plus tails).
    pub duration_deg: f32,
    /// Wiebe efficiency parameter. 5.0 gives ~99.3% burned by `duration_deg`.
    pub wiebe_a: f32,
    /// Wiebe form factor. 2.0 is the usual value for an SI engine.
    pub wiebe_m: f32,
}

impl Default for CombustionSpec {
    fn default() -> Self {
        Self {
            start_deg: -18.0,
            duration_deg: 55.0,
            wiebe_a: 5.0,
            wiebe_m: 2.0,
        }
    }
}

/// The intake as an acoustic source: plenum and restrictor neck as a
/// Helmholtz resonator, driven by the cylinders' intake draw, radiating the
/// time derivative of the neck flow from the intake mouth. See `intake` in
/// `cbr600rrSdm26()` (sim/src/audio/engineAudio.js) for the reasoning.
#[derive(Clone, Copy, Debug)]
pub struct IntakeSpec {
    /// Helmholtz resonance of plenum + restrictor, Hz.
    pub helmholtz_hz: f32,
    /// Quality factor of that resonance.
    pub q: f32,
    /// Level relative to the exhaust. 0 turns the intake off.
    pub level: f32,
}

impl IntakeSpec {
    pub const OFF: IntakeSpec = IntakeSpec { helmholtz_hz: 55.0, q: 1.8, level: 0.0 };
}

/// A complete engine.
#[derive(Clone, Debug)]
pub struct EngineSpec {
    pub name: &'static str,
    pub bore_m: f32,
    pub stroke_m: f32,
    pub conrod_m: f32,
    pub compression_ratio: f32,
    /// Crank angle at which each cylinder reaches firing TDC, degrees in
    /// [0, 720). The *set* of these angles is the firing order and the crank
    /// layout together, which is exactly what makes a cross-plane V8 burble
    /// and a flat-plane one scream.
    pub firing_angles_deg: Vec<f32>,
    pub timing: ValveTiming,
    pub combustion: CombustionSpec,
    /// One primary per cylinder, in the same order as `firing_angles_deg`.
    pub primaries: Vec<PipeSpec>,
    /// The collector each primary merges into. Length must match `primaries`.
    /// Two collectors on a V8 with no crossover; one on a 4-1; two -- the
    /// secondaries -- on a 4-2-1.
    pub primary_to_collector: Vec<usize>,
    pub collectors: Vec<PipeSpec>,
    /// The tailpipe each collector merges into. One per collector on a V8
    /// with twin pipes; every collector into one on a 4-2-1.
    pub collector_to_tailpipe: Vec<usize>,
    pub tailpipes: Vec<PipeSpec>,
    /// Sections after each tailpipe, in order, before the open end: a
    /// muffler's inlet, body and outlet. Empty, or one list per tailpipe.
    pub mufflers: Vec<Vec<PipeSpec>>,
    /// Charge per cylinder relative to the calibrated one. Empty = all 1.
    pub cylinder_trim: Vec<f32>,
    /// Cycle-to-cycle variation of the heat release (coefficient of
    /// variation). 0 makes every cycle identical.
    pub combustion_cov: f32,
    /// Finite-amplitude steepening in the pipes, as a fraction of the
    /// ideal-gas coefficient. 0 is a linear waveguide.
    pub wave_nonlinearity: f32,
    /// The intake as a sound source; `IntakeSpec::OFF` for none.
    pub intake: IntakeSpec,
    pub idle_rpm: f32,
    pub redline_rpm: f32,
    pub gas: GasProperties,
}

impl EngineSpec {
    pub fn cylinders(&self) -> usize {
        self.firing_angles_deg.len()
    }

    /// Swept volume of one cylinder, m^3.
    pub fn displacement_per_cylinder_m3(&self) -> f32 {
        self.piston_area_m2() * self.stroke_m
    }

    /// Total swept volume, m^3.
    pub fn displacement_m3(&self) -> f32 {
        self.displacement_per_cylinder_m3() * self.cylinders() as f32
    }

    pub fn piston_area_m2(&self) -> f32 {
        core::f32::consts::PI * self.bore_m * self.bore_m * 0.25
    }

    /// Clearance volume, m^3.
    pub fn clearance_volume_m3(&self) -> f32 {
        self.displacement_per_cylinder_m3() / (self.compression_ratio - 1.0).max(1e-6)
    }

    /// Cylinder volume at a crank angle, m^3, from the slider-crank relation.
    pub fn volume_at(&self, theta_deg: f32) -> f32 {
        let theta = theta_deg.to_radians();
        let r = self.stroke_m * 0.5;
        let l = self.conrod_m;
        let s = r * theta.cos() + (l * l - (r * theta.sin()).powi(2)).max(0.0).sqrt();
        let x = (l + r) - s; // piston displacement below TDC
        self.clearance_volume_m3() + self.piston_area_m2() * x
    }

    /// Firing frequency, Hz: how many combustion events happen per second
    /// across the whole engine. A four-stroke fires every other revolution.
    pub fn firing_frequency(&self, rpm: f32) -> f32 {
        rpm / 60.0 * 0.5 * self.cylinders() as f32
    }

    /// Sanity check: the topology has to be wired up consistently or the
    /// waveguide silently loses cylinders.
    pub fn validate(&self) -> Result<(), String> {
        if self.firing_angles_deg.is_empty() {
            return Err("engine has no cylinders".into());
        }
        if self.primaries.len() != self.cylinders() {
            return Err(format!(
                "{} primaries for {} cylinders",
                self.primaries.len(),
                self.cylinders()
            ));
        }
        if self.primary_to_collector.len() != self.cylinders() {
            return Err("primary_to_collector must have one entry per cylinder".into());
        }
        for (i, &c) in self.primary_to_collector.iter().enumerate() {
            if c >= self.collectors.len() {
                return Err(format!("primary {i} points at collector {c}, which does not exist"));
            }
        }
        if self.collector_to_tailpipe.len() != self.collectors.len() {
            return Err("collector_to_tailpipe must have one entry per collector".into());
        }
        if self.tailpipes.is_empty() {
            return Err("no tailpipe".into());
        }
        for (c, &t) in self.collector_to_tailpipe.iter().enumerate() {
            if t >= self.tailpipes.len() {
                return Err(format!("collector {c} points at tailpipe {t}, which does not exist"));
            }
        }
        if self.mufflers.len() > self.tailpipes.len() {
            return Err("more mufflers than tailpipes".into());
        }
        if self.compression_ratio <= 1.0 {
            return Err("compression ratio must exceed 1".into());
        }
        if self.conrod_m <= self.stroke_m * 0.5 {
            return Err("conrod is shorter than the crank throw".into());
        }
        Ok(())
    }
}

/// The SDM26's engine: a Honda CBR600RR (PC40) inline four, FSAE-restricted.
///
/// Geometry is the published Honda specification; the exhaust is the car's
/// own 4-2-1, measured off the CAD. Kept number-for-number identical to
/// `cbr600rrSdm26()` in sim/src/audio/engineAudio.js, which carries the full
/// reasoning for each value.
pub fn cbr600rr_sdm26() -> EngineSpec {
    let pipe = PipeSpec::from_diameter;
    EngineSpec {
        name: "Honda CBR600RR PC40 (SDM26, FSAE-restricted)",
        bore_m: 0.067,
        stroke_m: 0.0425,
        conrod_m: 0.0905,
        compression_ratio: 12.2,
        // 180-degree crank, firing order 1-2-4-3, nominally 0/180/540/360.
        // The degree or two off is each cylinder's effective event phasing
        // (valve lash and cam-lobe tolerance): a real four is never exactly
        // even, and the recorded car carries the half orders that makes.
        firing_angles_deg: vec![0.0, 182.0, 538.0, 361.0],
        timing: ValveTiming {
            evo_deg: 132.0,
            evc_deg: 372.0,
            ivo_deg: 348.0,
            ivc_deg: 576.0,
            // Two 22.5 mm exhaust valves per cylinder (Honda stock, 2007-on).
            exhaust_valve_diameter_m: 0.0225,
            exhaust_valve_count: 2.0,
            exhaust_cd: 0.72,
        },
        combustion: CombustionSpec {
            start_deg: -20.0,
            duration_deg: 52.0,
            wiebe_a: 5.0,
            wiebe_m: 2.0,
        },
        // The real 4-2-1 (SDM26Exhaust_Assm): 1+4 and 2+3 pair into the two
        // primary collectors; lengths valve (incl. ~50 mm of head port) to
        // mid-merge; inside diameters.
        primaries: [0.4224, 0.4243, 0.4245, 0.4262]
            .iter()
            .map(|&l| pipe(l, 0.0297, 0.005, 8000.0))
            .collect(),
        primary_to_collector: vec![0, 1, 1, 0],
        collectors: vec![pipe(0.57, 0.0361, 0.005, 8000.0), pipe(0.5775, 0.0361, 0.005, 8000.0)],
        collector_to_tailpipe: vec![0, 0],
        tailpipes: vec![pipe(0.343, 0.046, 0.005, 8000.0)],
        // Muffler inlet cone, 382 mm packed straight-through body, outlet tip.
        mufflers: vec![vec![
            pipe(0.054, 0.0555, 0.005, 8000.0),
            pipe(0.382, 0.0614, 0.1, 1500.0),
            pipe(0.059, 0.0614, 0.005, 8000.0),
        ]],
        // Uneven charge from a restricted single-plenum intake.
        cylinder_trim: vec![1.06, 0.955, 1.015, 0.97],
        combustion_cov: 0.03,
        wave_nonlinearity: 0.8,
        // Plenum (~3 L, CAD bounding box 178 x 185 x 175 mm) behind the 20 mm
        // restrictor; level judged by ear against the IMG_5128 recording.
        intake: IntakeSpec { helmholtz_hz: 55.0, q: 1.8, level: 2.0 },
        idle_rpm: 2_000.0,
        redline_rpm: 14_500.0,
        gas: GasProperties::default(),
    }
}

/// A single-cylinder thumper, for testing that the model is not secretly
/// hard-wired to four cylinders.
pub fn single_cylinder_450() -> EngineSpec {
    EngineSpec {
        name: "450 single",
        bore_m: 0.096,
        stroke_m: 0.0622,
        conrod_m: 0.104,
        compression_ratio: 12.0,
        firing_angles_deg: vec![0.0],
        timing: ValveTiming {
            evo_deg: 125.0,
            evc_deg: 375.0,
            ivo_deg: 345.0,
            ivc_deg: 570.0,
            exhaust_valve_diameter_m: 0.031,
            exhaust_valve_count: 1.0,
            exhaust_cd: 0.70,
        },
        combustion: CombustionSpec::default(),
        primaries: vec![PipeSpec::from_diameter(0.55, 0.038, 0.010, 3200.0)],
        primary_to_collector: vec![0],
        collectors: vec![PipeSpec::from_diameter(0.30, 0.042, 0.012, 2400.0)],
        collector_to_tailpipe: vec![0],
        tailpipes: vec![PipeSpec::from_diameter(0.40, 0.040, 0.030, 1100.0)],
        mufflers: Vec::new(),
        cylinder_trim: Vec::new(),
        combustion_cov: 0.0,
        wave_nonlinearity: 0.0,
        intake: IntakeSpec::OFF,
        idle_rpm: 1_500.0,
        redline_rpm: 11_000.0,
        gas: GasProperties::default(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_validate() {
        cbr600rr_sdm26().validate().unwrap();
        single_cylinder_450().validate().unwrap();
    }

    #[test]
    fn displacement_matches_the_real_engine() {
        let e = cbr600rr_sdm26();
        let cc = e.displacement_m3() * 1e6;
        assert!((cc - 599.0).abs() < 3.0, "expected ~599 cc, got {cc:.1}");
    }

    #[test]
    fn volume_is_minimum_at_tdc_and_maximum_at_bdc() {
        let e = cbr600rr_sdm26();
        let v_tdc = e.volume_at(0.0);
        let v_bdc = e.volume_at(180.0);
        assert!((v_tdc - e.clearance_volume_m3()).abs() < 1e-12);
        assert!(
            (v_bdc / v_tdc - e.compression_ratio).abs() < 0.02,
            "compression ratio came out {}",
            v_bdc / v_tdc
        );
    }

    #[test]
    fn firing_frequency_is_two_per_rev_on_a_four() {
        let e = cbr600rr_sdm26();
        // 6000 rpm = 100 rev/s, a four-stroke four fires twice per rev.
        assert!((e.firing_frequency(6_000.0) - 200.0).abs() < 1e-3);
    }

    #[test]
    fn exhaust_sound_speed_is_far_above_ambient() {
        let g = GasProperties::default();
        assert!(g.speed_of_sound(g.exhaust_k_max) > 600.0);
        assert!(g.speed_of_sound(g.ambient_k) < 360.0);
    }

    #[test]
    fn validate_catches_a_dangling_collector() {
        let mut e = cbr600rr_sdm26();
        e.primary_to_collector = vec![0, 0, 0, 7];
        assert!(e.validate().is_err());
    }
}
