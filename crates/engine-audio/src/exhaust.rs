//! The exhaust, as a digital waveguide.
//!
//! engine-sim solves the exhaust as a gas-dynamics network. This solves it as
//! a network of bidirectional delay lines with scattering junctions, which is
//! the standard acoustic-modelling equivalent: it reproduces the wave
//! behaviour that actually shapes the sound -- travel time down a primary,
//! reflection off the open end, the scatter at the collector -- for a few
//! arithmetic operations per sample instead of a CFD step.
//!
//! What that buys, concretely: the header length is *audible*. Lengthen the
//! primaries and the resonance drops. Let the gas cool and the speed of sound
//! falls, the tuned length shifts and the note changes on the overrun. Those
//! are emergent here, not scripted.

use crate::engine::{EngineSpec, GasProperties, PipeSpec};
use crate::filters::LowPassFilter;

/// A fractional-delay line. One direction of travel in one pipe.
#[derive(Clone, Debug)]
struct DelayLine {
    buf: Vec<f32>,
    cursor: usize,
    delay: f32,
}

impl DelayLine {
    fn new(capacity: usize) -> Self {
        Self {
            buf: vec![0.0; capacity.max(4)],
            cursor: 0,
            delay: 1.0,
        }
    }

    fn set_delay(&mut self, samples: f32) {
        // Two samples of headroom at each end so the interpolated read never
        // walks off the buffer.
        self.delay = samples.clamp(1.0, self.buf.len() as f32 - 2.0);
    }

    /// Read the value written `delay` samples ago, linearly interpolated.
    #[inline]
    fn read(&self) -> f32 {
        let n = self.buf.len();
        let d = self.delay;
        let i = d.floor();
        let frac = d - i;
        let a = (self.cursor + n - i as usize) % n;
        let b = (a + n - 1) % n;
        self.buf[a] * (1.0 - frac) + self.buf[b] * frac
    }

    #[inline]
    fn write(&mut self, x: f32) {
        self.buf[self.cursor] = x;
        self.cursor = (self.cursor + 1) % self.buf.len();
    }

    fn clear(&mut self) {
        for v in &mut self.buf {
            *v = 0.0;
        }
    }
}

/// One pipe: a wave running toward the outlet, and one running back.
#[derive(Clone, Debug)]
struct Pipe {
    fwd: DelayLine,
    bwd: DelayLine,
    area_m2: f32,
    length_m: f32,
    /// Amplitude retained per traverse.
    gain: f32,
    /// Admittance, A / (rho c). Set whenever the gas state changes.
    admittance: f32,
    /// Frequency-dependent loss, one filter per direction of travel.
    damp_fwd: LowPassFilter,
    damp_bwd: LowPassFilter,
}

impl Pipe {
    fn new(spec: &PipeSpec, sample_rate: f32) -> Self {
        // Size for the slowest sound speed we will ever see (cold gas, ~300
        // m/s) so the buffer is always long enough to hold the delay.
        let capacity = ((spec.length_m / 300.0) * sample_rate).ceil() as usize + 8;
        Self {
            fwd: DelayLine::new(capacity),
            bwd: DelayLine::new(capacity),
            area_m2: spec.area_m2,
            length_m: spec.length_m,
            gain: (1.0 - spec.loss).clamp(0.0, 1.0),
            admittance: 1.0,
            damp_fwd: LowPassFilter::new(spec.damping_hz, sample_rate),
            damp_bwd: LowPassFilter::new(spec.damping_hz, sample_rate),
        }
    }

    /// Delay output with this traverse's losses applied.
    #[inline]
    fn read_fwd(&mut self) -> f32 {
        let v = self.fwd.read();
        self.damp_fwd.f(v) * self.gain
    }

    #[inline]
    fn read_bwd(&mut self) -> f32 {
        let v = self.bwd.read();
        self.damp_bwd.f(v) * self.gain
    }

    fn retune(&mut self, c: f32, rho: f32, sample_rate: f32) {
        let samples = self.length_m / c.max(1.0) * sample_rate;
        self.fwd.set_delay(samples);
        self.bwd.set_delay(samples);
        self.admittance = self.area_m2 / (rho.max(1e-6) * c.max(1.0));
    }

    fn clear(&mut self) {
        self.fwd.clear();
        self.bwd.clear();
    }
}

/// The whole exhaust system for one engine.
pub struct ExhaustNetwork {
    primaries: Vec<Pipe>,
    collectors: Vec<Pipe>,
    tailpipes: Vec<Pipe>,
    primary_to_collector: Vec<usize>,
    sample_rate: f32,
    rho_c: f32,
    /// Reflection magnitude at the open end of a tailpipe. A real open pipe
    /// reflects most of a low-frequency wave back inverted and radiates the
    /// rest; 0.85 is a normal value for a pipe of this diameter.
    open_end_reflection: f32,
    /// Radiated signal, one entry per tailpipe.
    outputs: Vec<f32>,
    /// Preallocated working buffers.
    ///
    /// `step` runs 48,000 times a second and used to build twelve `Vec`s on
    /// each call. Half a million heap allocations per second dominated the
    /// whole synthesiser -- more than the convolution, and far more than the
    /// transcendental functions in the cylinder model.
    scratch: Scratch,
}

#[derive(Default, Clone, Debug)]
struct Scratch {
    prim_fwd_out: Vec<f32>,
    prim_bwd_out: Vec<f32>,
    coll_fwd_out: Vec<f32>,
    coll_bwd_out: Vec<f32>,
    tail_fwd_out: Vec<f32>,
    tail_bwd_out: Vec<f32>,
    prim_fwd_in: Vec<f32>,
    prim_bwd_in: Vec<f32>,
    coll_fwd_in: Vec<f32>,
    coll_bwd_in: Vec<f32>,
    tail_fwd_in: Vec<f32>,
    tail_bwd_in: Vec<f32>,
}

impl Scratch {
    fn sized(n_prim: usize, n_coll: usize, n_tail: usize) -> Self {
        Self {
            prim_fwd_out: vec![0.0; n_prim],
            prim_bwd_out: vec![0.0; n_prim],
            coll_fwd_out: vec![0.0; n_coll],
            coll_bwd_out: vec![0.0; n_coll],
            tail_fwd_out: vec![0.0; n_tail],
            tail_bwd_out: vec![0.0; n_tail],
            prim_fwd_in: vec![0.0; n_prim],
            prim_bwd_in: vec![0.0; n_prim],
            coll_fwd_in: vec![0.0; n_coll],
            coll_bwd_in: vec![0.0; n_coll],
            tail_fwd_in: vec![0.0; n_tail],
            tail_bwd_in: vec![0.0; n_tail],
        }
    }
}

impl ExhaustNetwork {
    pub fn new(engine: &EngineSpec, sample_rate: f32) -> Self {
        let mut net = Self {
            primaries: engine.primaries.iter().map(|p| Pipe::new(p, sample_rate)).collect(),
            collectors: engine.collectors.iter().map(|p| Pipe::new(p, sample_rate)).collect(),
            tailpipes: engine.tailpipes.iter().map(|p| Pipe::new(p, sample_rate)).collect(),
            primary_to_collector: engine.primary_to_collector.clone(),
            sample_rate,
            rho_c: 1.0,
            open_end_reflection: 0.85,
            outputs: vec![0.0; engine.tailpipes.len()],
            scratch: Scratch::sized(
                engine.primaries.len(),
                engine.collectors.len(),
                engine.tailpipes.len(),
            ),
        };
        net.set_gas_state(&engine.gas, engine.gas.exhaust_k_max);
        net
    }

    pub fn outputs(&self) -> &[f32] {
        &self.outputs
    }

    /// Characteristic impedance of the gas, rho * c. The source term needs it
    /// to turn a mass flow into a pressure wave.
    pub fn rho_c(&self) -> f32 {
        self.rho_c
    }

    /// Retune every delay line for a new exhaust gas temperature.
    ///
    /// Call this when the operating point moves, not every sample: it walks
    /// every pipe, and the temperature does not change at audio rate.
    pub fn set_gas_state(&mut self, gas: &GasProperties, exhaust_k: f32) {
        let c = gas.speed_of_sound(exhaust_k);
        let rho = gas.density(gas.ambient_pa, exhaust_k);
        self.rho_c = rho * c;
        for p in self
            .primaries
            .iter_mut()
            .chain(self.collectors.iter_mut())
            .chain(self.tailpipes.iter_mut())
        {
            p.retune(c, rho, self.sample_rate);
        }
    }

    pub fn reset(&mut self) {
        for p in self
            .primaries
            .iter_mut()
            .chain(self.collectors.iter_mut())
            .chain(self.tailpipes.iter_mut())
        {
            p.clear();
        }
        for o in &mut self.outputs {
            *o = 0.0;
        }
    }

    /// Pressure currently presented at the cylinder end of primary `i`.
    ///
    /// The cylinder model needs this as its back pressure. A wave returning
    /// from the collector while the exhaust valve is still open is precisely
    /// the mechanism header tuning works by, so this feedback path is the
    /// point rather than a refinement.
    pub fn port_pressure(&self, i: usize, ambient_pa: f32, valve_area_m2: f32) -> f32 {
        let p = &self.primaries[i];
        let r = port_reflection(p.area_m2, valve_area_m2);
        // Deliberately reads the raw delay line rather than `read_bwd`. The
        // damping filters are stateful and are advanced exactly once per sample
        // by `step`; running one here as well would double-filter the primary
        // and clock its state twice per sample.
        ambient_pa + (1.0 + r) * p.bwd.read() * p.gain
    }

    /// Advance the network one sample.
    ///
    /// `source_flow` is the volumetric flow leaving each cylinder's exhaust
    /// valve, m^3/s, one entry per cylinder. `valve_area` is that valve's
    /// effective flow area, m^2, which sets how much of a returning wave the
    /// port reflects.
    pub fn step(&mut self, source_flow: &[f32], valve_area: &[f32]) {
        // Move the scratch out so the rest of `self` can be borrowed alongside
        // it, then put it back. Cheaper than any interior-mutability dance, and
        // the compiler still proves the access is exclusive.
        let mut w = core::mem::take(&mut self.scratch);

        // ---- read every pipe end before writing anything -------------------
        // Doing this in one pass would let a junction see this sample's own
        // output, which is an algebraic loop and turns into a howl.
        for (i, p) in self.primaries.iter_mut().enumerate() {
            w.prim_fwd_out[i] = p.read_fwd();
            w.prim_bwd_out[i] = p.read_bwd();
        }
        for (i, p) in self.collectors.iter_mut().enumerate() {
            w.coll_fwd_out[i] = p.read_fwd();
            w.coll_bwd_out[i] = p.read_bwd();
        }
        for (i, p) in self.tailpipes.iter_mut().enumerate() {
            w.tail_fwd_out[i] = p.read_fwd();
            w.tail_bwd_out[i] = p.read_bwd();
        }

        // ---- cylinder end of each primary ----------------------------------
        // The valve is a velocity source -- pressure wave = rho * c * u, with
        // u = Q / A -- sitting at a junction whose reflection depends on how
        // far the valve is open.
        //
        // Treating this end as a rigid wall regardless of valve position is
        // wrong and audibly so. A wide-open exhaust valve is a hole into a
        // large volume, not a mirror: it should swallow most of a returning
        // wave. Reflecting all of it back leaves the primary with a Q high
        // enough that pulses interfere with their own echoes and the note
        // stops tracking the firing rate.
        for i in 0..self.primaries.len() {
            let q = source_flow.get(i).copied().unwrap_or(0.0);
            let a_pipe = self.primaries[i].area_m2.max(1e-9);
            let u = q / a_pipe;
            let r = port_reflection(a_pipe, valve_area.get(i).copied().unwrap_or(0.0));
            w.prim_fwd_in[i] = r * w.prim_bwd_out[i] + self.rho_c * u;
        }

        // ---- primaries into collectors -------------------------------------
        for c in 0..self.collectors.len() {
            // Kelly-Lochbaum scattering: p_junction = 2 * sum(Y_i p_i+) / sum(Y_i)
            let mut num = 0.0f32;
            let mut den = 0.0f32;
            for (i, &target) in self.primary_to_collector.iter().enumerate() {
                if target == c {
                    num += self.primaries[i].admittance * w.prim_fwd_out[i];
                    den += self.primaries[i].admittance;
                }
            }
            num += self.collectors[c].admittance * w.coll_bwd_out[c];
            den += self.collectors[c].admittance;

            let p_j = if den > 1e-12 { 2.0 * num / den } else { 0.0 };

            for (i, &target) in self.primary_to_collector.iter().enumerate() {
                if target == c {
                    w.prim_bwd_in[i] = p_j - w.prim_fwd_out[i];
                }
            }
            w.coll_fwd_in[c] = p_j - w.coll_bwd_out[c];
        }

        // ---- collector into tailpipe ---------------------------------------
        for c in 0..self.collectors.len() {
            let yc = self.collectors[c].admittance;
            let yt = self.tailpipes[c].admittance;
            let den = yc + yt;
            let p_j = if den > 1e-12 {
                2.0 * (yc * w.coll_fwd_out[c] + yt * w.tail_bwd_out[c]) / den
            } else {
                0.0
            };
            w.coll_bwd_in[c] = p_j - w.coll_fwd_out[c];
            w.tail_fwd_in[c] = p_j - w.tail_bwd_out[c];
        }

        // ---- open end -------------------------------------------------------
        // An open pipe end reflects the wave back inverted. What escapes is
        // the radiated sound, proportional to (1 + R) times the outgoing wave.
        for t in 0..self.tailpipes.len() {
            w.tail_bwd_in[t] = -self.open_end_reflection * w.tail_fwd_out[t];
            self.outputs[t] = (1.0 + self.open_end_reflection) * w.tail_fwd_out[t];
        }

        // ---- commit ---------------------------------------------------------
        for i in 0..self.primaries.len() {
            self.primaries[i].fwd.write(w.prim_fwd_in[i]);
            self.primaries[i].bwd.write(w.prim_bwd_in[i]);
        }
        for c in 0..self.collectors.len() {
            self.collectors[c].fwd.write(w.coll_fwd_in[c]);
            self.collectors[c].bwd.write(w.coll_bwd_in[c]);
        }
        for t in 0..self.tailpipes.len() {
            self.tailpipes[t].fwd.write(w.tail_fwd_in[t]);
            self.tailpipes[t].bwd.write(w.tail_bwd_in[t]);
        }

        self.scratch = w;
    }
}

/// Reflection coefficient at the cylinder end of a primary.
///
/// Derived the same way as any area discontinuity: the pipe sees an opening of
/// area `valve_area`, and `r = (A_pipe - A_valve) / (A_pipe + A_valve)`. A shut
/// valve gives +1, a rigid wall. A valve open to the full pipe area gives 0,
/// perfectly absorbing. Everything in between is a partial reflection, which is
/// what makes the cylinder feel the pipe and the pipe feel the cylinder.
#[inline]
fn port_reflection(pipe_area_m2: f32, valve_area_m2: f32) -> f32 {
    let a = pipe_area_m2.max(1e-12);
    let v = valve_area_m2.max(0.0);
    ((a - v) / (a + v)).clamp(-1.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::{cbr600rr_sdm26, single_cylinder_450};

    #[test]
    fn delay_line_delays_by_a_whole_number_of_samples() {
        let mut d = DelayLine::new(64);
        d.set_delay(5.0);
        d.write(1.0);
        for _ in 0..4 {
            assert_eq!(d.read(), 0.0);
            d.write(0.0);
        }
        assert!((d.read() - 1.0).abs() < 1e-6, "got {}", d.read());
    }

    #[test]
    fn silence_in_silence_out() {
        let e = cbr600rr_sdm26();
        let mut net = ExhaustNetwork::new(&e, 48_000.0);
        let zero = vec![0.0; e.cylinders()];
        for _ in 0..10_000 {
            net.step(&zero, &zero);
        }
        assert!(net.outputs().iter().all(|o| o.abs() < 1e-9));
    }

    #[test]
    fn an_impulse_comes_out_of_the_tailpipe() {
        let e = single_cylinder_450();
        let mut net = ExhaustNetwork::new(&e, 48_000.0);
        let mut peak = 0.0f32;
        for i in 0..4_000 {
            let src = if i == 0 { vec![0.05] } else { vec![0.0] };
            net.step(&src, &[0.0]);
            peak = peak.max(net.outputs()[0].abs());
        }
        assert!(peak > 0.0, "nothing radiated from the tailpipe");
    }

    #[test]
    fn the_network_is_stable_under_continuous_excitation() {
        let e = cbr600rr_sdm26();
        let mut net = ExhaustNetwork::new(&e, 48_000.0);
        let mut rng = crate::filters::Rng::new(7);
        let mut peak = 0.0f32;
        for _ in 0..200_000 {
            let src: Vec<f32> = (0..e.cylinders()).map(|_| rng.uniform() * 0.02).collect();
            let shut = vec![0.0f32; e.cylinders()];
            net.step(&src, &shut);
            peak = peak.max(net.outputs()[0].abs());
            assert!(net.outputs()[0].is_finite(), "waveguide diverged");
        }
        assert!(peak.is_finite());
    }

    /// Ring an impulse through the system and report where it resonates.
    fn ring_hz(primary_m: f32) -> f32 {
        let mut e = single_cylinder_450();
        e.primaries[0].length_m = primary_m;
        let fs = 48_000.0;
        let mut net = ExhaustNetwork::new(&e, fs);
        let mut buf = vec![0.0f32; 8_192];
        for (i, out) in buf.iter_mut().enumerate() {
            let src = if i == 0 { vec![0.05] } else { vec![0.0] };
            net.step(&src, &[0.0]);
            *out = net.outputs()[0];
        }
        crate::testutil::spectral_peak_hz(&buf, fs, 60.0, 2_500.0, 500)
    }

    #[test]
    fn a_longer_primary_resonates_lower() {
        // The physical claim this whole module exists to support: pipe length
        // sets pitch. The primary is only part of the acoustic path -- the
        // collector and tailpipe are fixed -- so doubling it lengthens the
        // system by a third, and the resonance should fall by about as much.
        let short = ring_hz(0.35);
        let long = ring_hz(0.70);
        println!("primary 0.35 m -> {short:.0} Hz, 0.70 m -> {long:.0} Hz");
        assert!(short > 0.0 && long > 0.0, "no resonance measured");
        assert!(
            long < short * 0.90,
            "lengthening the primary did not drop the pitch: {short:.0} Hz -> {long:.0} Hz"
        );
    }

    #[test]
    fn hotter_gas_raises_the_resonance() {
        // Speed of sound rises with temperature, so the same pipe rings higher
        // when hot. This is the mechanism behind the note changing with load.
        let e = single_cylinder_450();
        let fs = 48_000.0;
        let mut cold = ExhaustNetwork::new(&e, fs);
        cold.set_gas_state(&e.gas, 400.0);
        let mut hot = ExhaustNetwork::new(&e, fs);
        hot.set_gas_state(&e.gas, 1_200.0);
        // Delay is length / c, so hot must have the shorter delay.
        assert!(hot.primaries[0].fwd.delay < cold.primaries[0].fwd.delay);
    }
}
