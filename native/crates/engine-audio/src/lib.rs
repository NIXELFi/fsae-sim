//! Physically-modelled internal-combustion engine audio.
//!
//! Adapted from [`ange-yaghi/engine-sim`](https://github.com/ange-yaghi/engine-sim)
//! (MIT). The repository at `Engine-Simulator/engine-sim-community-edition`
//! distributes the built application and contains no source; the algorithms
//! here follow the original open codebase.
//!
//! # What this is
//!
//! Sound is *generated*, not sampled. There are no recordings. The chain is:
//!
//! ```text
//!   crank angle
//!     -> per-cylinder pressure (single-zone, Wiebe heat release)
//!     -> flow through the exhaust valve
//!     -> exhaust waveguide (primaries -> collector -> tailpipe -> open end)
//!     -> synthesiser (jitter, DC removal, derivative, noise, convolution)
//!     -> samples
//! ```
//!
//! Because every stage is physical, the things that change an engine's voice in
//! reality change it here: cylinder count and crank phasing, bore and stroke,
//! valve timing, primary length, gas temperature. Nothing is special-cased for
//! the SDM26 -- swapping in a V-twin means writing a different [`EngineSpec`].
//!
//! # Why the torque curve is an input
//!
//! [`EngineAudio::set_operating_point`] takes the torque the engine is actually
//! making. The heat release is then solved so the modelled cycle does that much
//! work. For the SDM26 that number comes from the Helios CFD sweep the vehicle
//! model is already using, so the note and the acceleration are answering to
//! the same data rather than drifting apart.
//!
//! # Cost
//!
//! One sample is: a cylinder update per cylinder, a handful of adds per pipe,
//! and the synthesiser chain, of which the convolution dominates at `taps`
//! multiply-accumulates, which is most of the total. The default is 256 taps
//! rather than engine-sim's much longer responses because the browser audio
//! worklet is the binding constraint, and 256 taps at 48 kHz still covers the
//! whole early-reflection structure the impulse response exists to provide
//! (the cockpit response's last discrete reflection lands at 4.8 ms, or 230
//! samples). Measured: roughly 20x real time natively.

#[cfg(test)]
pub(crate) mod testutil;

pub mod cylinder;
pub mod engine;
pub mod exhaust;
pub mod filters;
pub mod ir;
pub mod synth;

pub use engine::{
    cbr600rr_sdm26, single_cylinder_450, CombustionSpec, EngineSpec, GasProperties, IntakeSpec,
    PipeSpec, ValveTiming,
};
pub use ir::Cabin;
pub use synth::AudioParameters;

use cylinder::{calibrate, step_cylinder, CycleCalibration, CycleTables, Cylinder, OperatingPoint};
use filters::Rng;

/// Seed of the cycle-to-cycle combustion variation; `CYCLE_SEED` in the JS port.
pub const CYCLE_SEED: u64 = 0xC7_C1E5;

/// Scale from the intake mouth's radiated flow derivative to output full scale
/// at intake level 1; `INTAKE_RADIATION_SCALE` in the JS port.
pub const INTAKE_RADIATION_SCALE: f32 = 0.015;
use exhaust::ExhaustNetwork;
use synth::Synthesizer;

/// How the audio engine is set up. Distinct from [`EngineSpec`], which is the
/// engine itself: these are choices about rendering it.
#[derive(Clone, Copy, Debug)]
pub struct AudioConfig {
    pub sample_rate: f32,
    /// Convolution impulse-response length, samples. 0 disables convolution.
    pub ir_taps: usize,
    pub cabin: Cabin,
    /// Seed for the impulse response and the noise sources. Fixed by default so
    /// output is reproducible and can be checked against golden vectors.
    pub seed: u64,
}

impl Default for AudioConfig {
    fn default() -> Self {
        Self {
            sample_rate: 48_000.0,
            ir_taps: 256,
            cabin: Cabin::Cockpit,
            seed: 0x5DAE_2026,
        }
    }
}

/// A running engine that produces samples.
pub struct EngineAudio {
    spec: EngineSpec,
    config: AudioConfig,
    cylinders: Vec<Cylinder>,
    exhaust: ExhaustNetwork,
    synth: Synthesizer,
    crank_deg: f32,
    op: OperatingPoint,
    cal: CycleCalibration,
    tables: CycleTables,
    /// Scratch, reused every sample so the render loop does not allocate.
    flow: Vec<f32>,
    valve_area: Vec<f32>,
    /// Output trim for the current operating point; see `set_operating_point`.
    /// Slewed toward `level_target` per sample: applied as a step it clicked
    /// on every shift (a ~29 dB drop between two samples, twice per shift).
    level: f32,
    level_target: f32,
    /// Combustion power at the limiter, the reference for `level`.
    reference_power_w: f32,
    inputs: Vec<f32>,
    running: bool,
    /// Each cylinder's own charge trim (`EngineSpec::cylinder_trim`).
    trim: Vec<f32>,
    /// Draws the cycle-to-cycle combustion variation.
    cycle_rng: Rng,
    /// Intake resonator: neck flow, its rate, and last sample's flow.
    intake_x: f32,
    intake_v: f32,
    intake_prev: f32,
}

impl EngineAudio {
    pub fn new(spec: EngineSpec, config: AudioConfig) -> Result<Self, String> {
        spec.validate()?;

        let ir = ir::build(config.cabin, config.ir_taps, config.sample_rate, config.seed);
        let synth = Synthesizer::new(
            spec.tailpipes.len(),
            config.sample_rate,
            ir,
            AudioParameters::default(),
        );
        let exhaust = ExhaustNetwork::new(&spec, config.sample_rate);
        let trim: Vec<f32> = (0..spec.cylinders())
            .map(|i| spec.cylinder_trim.get(i).copied().unwrap_or(1.0))
            .collect();
        let cylinders = spec
            .firing_angles_deg
            .iter()
            .enumerate()
            .map(|(i, &phase)| {
                let mut c = Cylinder::new(phase, spec.gas.ambient_pa, spec.gas.ambient_k);
                c.heat_scale = trim[i];
                c
            })
            .collect::<Vec<_>>();

        let spec_for_ref = spec.clone();

        let op = OperatingPoint {
            rpm: spec.idle_rpm,
            throttle: 0.0,
            target_torque_nm: 0.0,
            exhaust_k: spec.gas.exhaust_k_min,
        };
        let cal = calibrate(&spec, op);
        let tables = CycleTables::build(&spec);

        let n_cyl = cylinders.len();
        let n_tail = spec.tailpipes.len();

        Ok(Self {
            spec,
            config,
            cylinders,
            exhaust,
            synth,
            crank_deg: 0.0,
            op,
            cal,
            tables,
            flow: vec![0.0; n_cyl],
            valve_area: vec![0.0; n_cyl],
            level: 0.3,
            level_target: 0.3,
            reference_power_w: Self::reference_power(&spec_for_ref),
            inputs: vec![0.0; n_tail],
            running: true,
            trim,
            cycle_rng: Rng::new(CYCLE_SEED),
            intake_x: 0.0,
            intake_v: 0.0,
            intake_prev: 0.0,
        })
    }

    pub fn spec(&self) -> &EngineSpec {
        &self.spec
    }

    pub fn config(&self) -> &AudioConfig {
        &self.config
    }

    pub fn parameters(&self) -> &AudioParameters {
        self.synth.parameters()
    }

    pub fn set_parameters(&mut self, p: AudioParameters) {
        self.synth.set_parameters(p);
    }

    pub fn set_cabin(&mut self, cabin: Cabin) {
        self.config.cabin = cabin;
        let ir = ir::build(cabin, self.config.ir_taps, self.config.sample_rate, self.config.seed);
        self.synth.set_impulse_response(ir);
    }

    /// Stop or start combustion. A stopped engine goes silent, but the
    /// waveguide is left to ring out rather than being cleared, which is what
    /// a real exhaust does.
    pub fn set_running(&mut self, running: bool) {
        self.running = running;
    }

    pub fn crank_angle_deg(&self) -> f32 {
        self.crank_deg
    }

    pub fn operating_point(&self) -> OperatingPoint {
        self.op
    }

    /// Move the engine to a new operating point.
    ///
    /// Call this at the physics rate (a few hundred Hz), not per audio sample:
    /// it re-solves the heat release and retunes every delay line for the new
    /// gas temperature, neither of which changes at audio rate.
    pub fn set_operating_point(&mut self, rpm: f32, throttle: f32, torque_nm: f32) {
        let throttle = throttle.clamp(0.0, 1.0);
        // Exhaust gas temperature tracks load. This is what shifts the tuned
        // length of the header, so the note moves as the engine takes load
        // rather than only with rpm.
        let load = (torque_nm.max(0.0) / 70.0).clamp(0.0, 1.0) * 0.6 + throttle * 0.4;
        let gas = self.spec.gas;
        let exhaust_k = gas.exhaust_k_min + (gas.exhaust_k_max - gas.exhaust_k_min) * load;

        let retune = (exhaust_k - self.op.exhaust_k).abs() > 5.0;
        self.op = OperatingPoint {
            rpm: rpm.max(0.0),
            throttle,
            target_torque_nm: torque_nm,
            exhaust_k,
        };
        self.cal = calibrate(&self.spec, self.op);
        if retune {
            self.exhaust.set_gas_state(&gas, exhaust_k);
        }

        // How loud this operating point should be, from the chemical power the
        // engine is actually releasing.
        //
        // Heat release per cylinder per cycle times the firing rate is the fuel
        // power going in, and that is what drives the exhaust. Using it rather
        // than a hand-blended mix of throttle and rpm means the overrun falls
        // away on its own -- no combustion, no power, no noise beyond the floor
        // -- and every intermediate point lands where the physics puts it.
        let p = *self.synth.parameters();
        let firing_per_second = (self.op.rpm / 120.0) * self.cylinders.len() as f32;
        let chemical_power_w = self.cal.heat_release_j * firing_per_second;
        let rel = (chemical_power_w / self.reference_power_w).clamp(0.0, 1.0);
        self.level_target = p.level_floor + (1.0 - p.level_floor) * rel.powf(p.level_exponent);
    }

    /// Combustion power at the limiter on full throttle, W.
    ///
    /// The reference the level curve is measured against. Computed from the
    /// engine's own spec rather than hard-coded, so a different engine scales
    /// itself instead of coming out silent or clipped.
    fn reference_power(spec: &EngineSpec) -> f32 {
        let op = OperatingPoint {
            rpm: spec.redline_rpm,
            throttle: 1.0,
            // Peak torque is not in the spec, so use an indicated mean
            // effective pressure of 10 bar -- normal for a naturally aspirated
            // engine at full load, and only a scale factor here.
            target_torque_nm: 10e5 * spec.displacement_m3() / (4.0 * core::f32::consts::PI),
            exhaust_k: spec.gas.exhaust_k_max,
        };
        let cal = calibrate(spec, op);
        cal.heat_release_j * ((spec.redline_rpm / 120.0) * spec.cylinders() as f32)
    }

    /// Fill `out` with mono samples in [-1, 1].
    pub fn render(&mut self, out: &mut [f32]) {
        let dt = 1.0 / self.config.sample_rate;
        let deg_per_sample = self.op.rpm * 6.0 * dt;
        let ambient = self.spec.gas.ambient_pa;
        let load = self.op.throttle;
        // 15 ms one-pole on the level: fast enough to follow a blip, slow
        // enough that a cut is a fall, not a click.
        let level_k = 1.0 - (-dt / 0.015_f32).exp();
        let ivc = self.spec.timing.ivc_deg;
        let cov = self.spec.combustion_cov;
        // Intake (see `IntakeSpec`).
        let intake = self.spec.intake;
        let intake_on = intake.level > 0.0;
        let iw0 = 2.0 * core::f32::consts::PI * intake.helmholtz_hz;
        let i_damp = iw0 / intake.q.max(1e-3);
        let i_gain = intake.level * INTAKE_RADIATION_SCALE * self.synth.parameters().volume;
        let ivo = self.spec.timing.ivo_deg;
        let map_frac = 0.14 + 0.72 * self.op.throttle;

        for sample in out.iter_mut() {
            self.level += (self.level_target - self.level) * level_k;
            // Cylinders, each seeing the pressure its own primary presents.
            for (i, cyl) in self.cylinders.iter_mut().enumerate() {
                let theta = cyl.theta(self.crank_deg);
                // Cycle-to-cycle variation, drawn once per cycle per cylinder
                // at intake-valve close: a near-Gaussian spread (sum of three
                // uniforms, unit variance) around the cylinder's own trim.
                let trapped = cyl.last_theta >= 0.0 && cyl.last_theta < ivc && theta >= ivc;
                if trapped && cov > 0.0 {
                    let g = self.cycle_rng.uniform() + self.cycle_rng.uniform() + self.cycle_rng.uniform();
                    cyl.heat_scale = self.trim[i] * (1.0 + cov * g).max(0.0);
                }
                cyl.last_theta = theta;
                let area = self.tables.valve_area(theta);
                self.valve_area[i] = area;
                // The port velocity from the previous sample closes the
                // impedance loop. Strictly implicit -- the pressure depends on
                // the flow and the flow on the pressure -- so this is one
                // Gauss-Seidel step. The port inertance already limits how fast
                // the velocity can move, so the lag is shorter than the
                // physical time constant and the loop is stable.
                let back = self.exhaust.port_pressure(i, ambient, area, cyl.port_velocity);
                let q = if self.running {
                    step_cylinder(
                        &self.spec,
                        &self.tables,
                        cyl,
                        self.crank_deg,
                        dt,
                        &self.cal,
                        &self.op,
                        back,
                    )
                } else {
                    0.0
                };
                self.flow[i] = q;
            }

            self.exhaust.step(&self.flow, &self.valve_area);
            self.inputs.copy_from_slice(self.exhaust.outputs());
            let mut y = self.synth.render(&self.inputs, load, self.level);

            if intake_on {
                // Volume drawn by the cylinders whose intake valves are open,
                // each scaled by its own charge trim and by manifold pressure.
                let mut q = 0.0f32;
                if self.running {
                    for (i, cyl) in self.cylinders.iter().enumerate() {
                        let th = cyl.theta(self.crank_deg);
                        if cylinder::is_between(th, ivo, ivc) {
                            let dvol = self.tables.volume((th + deg_per_sample) % 720.0) - self.tables.volume(th);
                            if dvol > 0.0 {
                                q += self.trim[i] * dvol;
                            }
                        }
                    }
                    q = q / dt * map_frac;
                }
                let a = iw0 * iw0 * (q - self.intake_x) - i_damp * self.intake_v;
                self.intake_v += a * dt;
                self.intake_x += self.intake_v * dt;
                let rad = (self.intake_x - self.intake_prev) / dt;
                self.intake_prev = self.intake_x;
                y = (y + i_gain * self.level * rad).clamp(-1.0, 1.0);
            }
            *sample = y;

            self.crank_deg += deg_per_sample;
            if self.crank_deg >= 720.0 {
                self.crank_deg -= 720.0;
            }
        }
    }

    /// Reset the acoustics without changing the operating point.
    pub fn reset(&mut self) {
        self.exhaust.reset();
        self.crank_deg = 0.0;
        for c in &mut self.cylinders {
            c.pressure_pa = self.spec.gas.ambient_pa;
            c.temperature_k = self.spec.gas.ambient_k;
            c.exhaust_flow = 0.0;
            c.port_velocity = 0.0;
        }
        self.intake_x = 0.0;
        self.intake_v = 0.0;
        self.intake_prev = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn running_engine(rpm: f32, torque: f32) -> EngineAudio {
        let mut e = EngineAudio::new(cbr600rr_sdm26(), AudioConfig::default()).unwrap();
        e.set_operating_point(rpm, 1.0, torque);
        e
    }

    use testutil::fundamental_hz;

    #[test]
    fn it_makes_a_sound() {
        let mut e = running_engine(9_000.0, 62.6);
        let mut buf = vec![0.0f32; 48_000];
        e.render(&mut buf);
        let rms = (buf.iter().map(|v| v * v).sum::<f32>() / buf.len() as f32).sqrt();
        assert!(rms > 0.01, "output is effectively silent, rms {rms}");
    }

    #[test]
    fn every_sample_is_finite_and_bounded() {
        let mut e = running_engine(14_000.0, 55.0);
        let mut buf = vec![0.0f32; 96_000];
        e.render(&mut buf);
        for (i, &v) in buf.iter().enumerate() {
            assert!(v.is_finite(), "sample {i} is not finite");
            assert!((-1.0..=1.0).contains(&v), "sample {i} = {v} escaped");
        }
    }

    #[test]
    fn pitch_tracks_the_firing_frequency() {
        // The headline check. A four-stroke four fires twice per revolution,
        // so the fundamental must be rpm/30. If this drifts, the crank phasing
        // or the cycle length is wrong.
        let fs = AudioConfig::default().sample_rate;
        for &rpm in &[6_000.0f32, 9_000.0, 12_000.0] {
            let mut e = running_engine(rpm, 55.0);
            let mut warm = vec![0.0f32; 24_000];
            e.render(&mut warm); // let the leveller and waveguide settle
            let mut buf = vec![0.0f32; 48_000];
            e.render(&mut buf);

            let expected = rpm / 30.0;
            let got = fundamental_hz(&buf, fs, expected * 0.4, expected * 2.5);
            let err = (got - expected).abs() / expected;
            assert!(
                err < 0.12,
                "at {rpm} rpm expected ~{expected:.0} Hz, measured {got:.0} Hz"
            );
        }
    }

    #[test]
    fn doubling_the_rpm_doubles_the_pitch() {
        let fs = AudioConfig::default().sample_rate;
        let measure = |rpm| {
            let mut e = running_engine(rpm, 55.0);
            let mut warm = vec![0.0f32; 24_000];
            e.render(&mut warm);
            let mut buf = vec![0.0f32; 48_000];
            e.render(&mut buf);
            fundamental_hz(&buf, fs, 60.0, 900.0)
        };
        let low = measure(6_000.0);
        let high = measure(12_000.0);
        assert!(
            (high / low - 2.0).abs() < 0.25,
            "{low:.0} Hz -> {high:.0} Hz is not an octave"
        );
    }

    #[test]
    fn a_single_cylinder_sounds_an_octave_and_a_bit_below_a_four() {
        // Same rpm, quarter the combustion events: the fundamental must drop.
        let fs = AudioConfig::default().sample_rate;
        let run = |spec: EngineSpec| {
            let mut e = EngineAudio::new(spec, AudioConfig::default()).unwrap();
            e.set_operating_point(8_000.0, 1.0, 40.0);
            let mut warm = vec![0.0f32; 24_000];
            e.render(&mut warm);
            let mut buf = vec![0.0f32; 48_000];
            e.render(&mut buf);
            fundamental_hz(&buf, fs, 40.0, 900.0)
        };
        let four = run(cbr600rr_sdm26());
        let single = run(single_cylinder_450());
        assert!(
            single < four * 0.6,
            "single {single:.0} Hz should be well below four {four:.0} Hz"
        );
    }

    #[test]
    fn a_stopped_engine_falls_silent() {
        let mut e = running_engine(9_000.0, 62.6);
        let mut buf = vec![0.0f32; 24_000];
        e.render(&mut buf);
        e.set_running(false);
        e.render(&mut buf); // ring-out
        let mut quiet = vec![0.0f32; 48_000];
        e.render(&mut quiet);
        let rms = (quiet.iter().map(|v| v * v).sum::<f32>() / quiet.len() as f32).sqrt();
        assert!(rms < 1e-3, "stopped engine still making noise, rms {rms}");
    }

    #[test]
    fn changing_the_cabin_changes_the_output() {
        let mut a = running_engine(9_000.0, 62.6);
        let mut b = running_engine(9_000.0, 62.6);
        b.set_cabin(Cabin::Trackside);
        let mut buf_a = vec![0.0f32; 8_000];
        let mut buf_b = vec![0.0f32; 8_000];
        a.render(&mut buf_a);
        b.render(&mut buf_b);
        assert!(buf_a != buf_b, "the impulse response made no difference");
    }

    #[test]
    fn output_is_reproducible() {
        let mut a = running_engine(9_000.0, 62.6);
        let mut b = running_engine(9_000.0, 62.6);
        let mut buf_a = vec![0.0f32; 16_000];
        let mut buf_b = vec![0.0f32; 16_000];
        a.render(&mut buf_a);
        b.render(&mut buf_b);
        assert_eq!(buf_a, buf_b, "same inputs gave different audio");
    }

    #[test]
    fn a_bad_spec_is_rejected_rather_than_rendered() {
        let mut spec = cbr600rr_sdm26();
        spec.collectors.clear();
        assert!(EngineAudio::new(spec, AudioConfig::default()).is_err());
    }
}
