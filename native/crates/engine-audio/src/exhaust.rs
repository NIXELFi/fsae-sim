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
        self.read_at(self.delay)
    }

    #[inline]
    fn read_at(&self, delay: f32) -> f32 {
        let n = self.buf.len();
        let i = delay.floor();
        let frac = delay - i;
        let a = (self.cursor + n - i as usize) % n;
        let b = (a + n - 1) % n;
        self.buf[a] * (1.0 - frac) + self.buf[b] * frac
    }

    /// Read with amplitude-dependent travel time: finite-amplitude steepening.
    ///
    /// A sound wave's crest travels faster than its trough, by (gamma+1)/2
    /// times the particle velocity -- and in an exhaust primary a 50 kPa
    /// blowdown pulse moves the gas at a couple of hundred metres a second.
    /// Over half a metre of header the pulse's front catches up with itself
    /// and steepens toward a shock; that steep front is the rasp in a real
    /// exhaust note, and a linear waveguide cannot make it. The delay is
    /// shortened in proportion to the wave's own pressure:
    /// `c_eff = c (1 + k p)`, `k = (gamma+1) / (2 gamma p_ambient)`.
    #[inline]
    fn read_nonlinear(&self, k: f32) -> f32 {
        if k == 0.0 {
            return self.read_at(self.delay);
        }
        let w0 = self.read_at(self.delay);
        let speedup = (1.0 + k * w0).clamp(0.5, 2.0);
        self.read_at((self.delay / speedup).clamp(1.0, self.buf.len() as f32 - 2.0))
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
    /// Finite-amplitude coefficient, 1/Pa; see `DelayLine::read_nonlinear`.
    nonlinear_k: f32,
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
            nonlinear_k: 0.0,
        }
    }

    /// Delay output with this traverse's losses applied.
    #[inline]
    fn read_fwd(&mut self) -> f32 {
        let v = self.fwd.read_nonlinear(self.nonlinear_k);
        self.damp_fwd.f(v) * self.gain
    }

    #[inline]
    fn read_bwd(&mut self) -> f32 {
        let v = self.bwd.read_nonlinear(self.nonlinear_k);
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

/// Where each pipe's outlet goes: into the inlet of another pipe, or out of
/// the open end.
const OPEN_END: usize = usize::MAX;

/// The exhaust as a tree of pipes, each feeding the one downstream of it.
///
/// Built from the spec in stages: one primary per cylinder, merging into
/// `collectors`, which merge into `tailpipes`, each of which may feed a chain
/// of `mufflers` sections before the open end. A 4-1 is four primaries into
/// one collector; a 4-2-1 is four primaries into two secondaries into one
/// final pipe. Every merge and every change of section is the same
/// Kelly-Lochbaum junction, so the topology is data. Mirrors
/// `exhaustTopology` in sim/src/audio/engineAudio.js, pipe for pipe.
fn topology(engine: &EngineSpec) -> (Vec<PipeSpec>, Vec<usize>) {
    let np = engine.primaries.len();
    let nc = engine.collectors.len();
    let nt = engine.tailpipes.len();
    let mut pipes = Vec::new();
    let mut down = Vec::new();
    for i in 0..np {
        pipes.push(engine.primaries[i]);
        down.push(np + engine.primary_to_collector[i]);
    }
    for c in 0..nc {
        pipes.push(engine.collectors[c]);
        down.push(np + nc + engine.collector_to_tailpipe[c]);
    }
    for t in 0..nt {
        pipes.push(engine.tailpipes[t]);
        down.push(OPEN_END);
    }
    for t in 0..nt {
        let Some(chain) = engine.mufflers.get(t) else { continue };
        let mut prev = np + nc + t;
        for sec in chain {
            down[prev] = pipes.len();
            prev = pipes.len();
            pipes.push(*sec);
            down.push(OPEN_END);
        }
    }
    (pipes, down)
}

/// The junction at the inlet of a non-primary pipe, and the pipes feeding it.
#[derive(Clone, Debug)]
struct Junction {
    pipe: usize,
    upstream: Vec<usize>,
}

/// The whole exhaust system for one engine.
pub struct ExhaustNetwork {
    pipes: Vec<Pipe>,
    n_primaries: usize,
    junctions: Vec<Junction>,
    open_ends: Vec<usize>,
    sample_rate: f32,
    rho_c: f32,
    /// Scale on the ideal finite-amplitude coefficient; 0 is linear.
    nonlinearity: f32,
    /// Reflection magnitude at the open end of a tailpipe. A real open pipe
    /// reflects most of a low-frequency wave back inverted and radiates the
    /// rest; 0.85 is a normal value for a pipe of this diameter.
    open_end_reflection: f32,
    /// Radiated signal, one entry per open end.
    outputs: Vec<f32>,
    /// Preallocated working buffers: `step` runs 48,000 times a second, and
    /// allocating in it once dominated the whole synthesiser.
    fo: Vec<f32>,
    bo: Vec<f32>,
    fi: Vec<f32>,
    bi: Vec<f32>,
}

impl ExhaustNetwork {
    pub fn new(engine: &EngineSpec, sample_rate: f32) -> Self {
        let (specs, down) = topology(engine);
        let n = specs.len();
        let n_primaries = engine.primaries.len();
        let junctions = (n_primaries..n)
            .map(|j| Junction {
                pipe: j,
                upstream: (0..n).filter(|&k| down[k] == j).collect(),
            })
            .collect();
        let open_ends: Vec<usize> = (0..n).filter(|&k| down[k] == OPEN_END).collect();
        let mut net = Self {
            pipes: specs.iter().map(|p| Pipe::new(p, sample_rate)).collect(),
            n_primaries,
            junctions,
            outputs: vec![0.0; open_ends.len()],
            open_ends,
            sample_rate,
            rho_c: 1.0,
            nonlinearity: engine.wave_nonlinearity,
            open_end_reflection: 0.85,
            fo: vec![0.0; n],
            bo: vec![0.0; n],
            fi: vec![0.0; n],
            bi: vec![0.0; n],
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
        let k = self.nonlinearity * (gas.gamma + 1.0) / (2.0 * gas.gamma * gas.ambient_pa);
        for p in self.pipes.iter_mut() {
            p.retune(c, rho, self.sample_rate);
            p.nonlinear_k = k;
        }
    }

    pub fn reset(&mut self) {
        for p in self.pipes.iter_mut() {
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
    /// `port_velocity_ms` is the gas velocity currently going through the
    /// valve, positive out of the cylinder. It matters, and leaving it out was
    /// the single biggest error in the model.
    ///
    /// A pipe is not an infinite reservoir. Push gas in and the pressure at the
    /// inlet rises immediately by `rho c u`; pull gas out and it falls. That is
    /// the pipe's characteristic impedance, and it acts within the sample --
    /// not after the round trip a wave takes to come back. Without it the
    /// exhaust valve saw a constant one atmosphere and could draw from it at
    /// will, which at small throttle openings meant gas rushing backwards into
    /// the cylinder at sonic velocity and injecting a wave as large as a
    /// full-power blowdown.
    pub fn port_pressure(
        &self,
        i: usize,
        ambient_pa: f32,
        valve_area_m2: f32,
        port_velocity_ms: f32,
    ) -> f32 {
        let p = &self.pipes[i];
        let r = port_reflection(p.area_m2, valve_area_m2);
        // Deliberately reads the raw delay line rather than `read_bwd`. The
        // damping filters are stateful and are advanced exactly once per sample
        // by `step`; running one here as well would double-filter the primary
        // and clock its state twice per sample.
        let wave = (1.0 + r) * p.bwd.read() * p.gain;
        let u_pipe = port_velocity_ms * valve_area_m2 / p.area_m2.max(1e-9);
        ambient_pa + wave + self.rho_c * u_pipe
    }

    /// Advance the network one sample.
    ///
    /// `source_flow` is the volumetric flow leaving each cylinder's exhaust
    /// valve, m^3/s, one entry per cylinder. `valve_area` is that valve's
    /// effective flow area, m^2, which sets how much of a returning wave the
    /// port reflects.
    pub fn step(&mut self, source_flow: &[f32], valve_area: &[f32]) {
        let n = self.pipes.len();

        // ---- read every pipe end before writing anything -------------------
        // Doing this in one pass would let a junction see this sample's own
        // output, which is an algebraic loop and turns into a howl.
        for k in 0..n {
            self.fo[k] = self.pipes[k].read_fwd();
            self.bo[k] = self.pipes[k].read_bwd();
        }

        // ---- cylinder end of each primary ----------------------------------
        // The valve is a velocity source -- pressure wave = rho * c * u, with
        // u = Q / A -- sitting at a junction whose reflection depends on how
        // far the valve is open. A wide-open valve is a hole into a large
        // volume, not a mirror: it swallows most of a returning wave.
        for i in 0..self.n_primaries {
            let q = source_flow.get(i).copied().unwrap_or(0.0);
            let a_pipe = self.pipes[i].area_m2.max(1e-9);
            let u = q / a_pipe;
            let r = port_reflection(a_pipe, valve_area.get(i).copied().unwrap_or(0.0));
            self.fi[i] = r * self.bo[i] + self.rho_c * u;
        }

        // ---- every merge and change of section -----------------------------
        // Kelly-Lochbaum scattering: p_junction = 2 * sum(Y_i p_i+) / sum(Y_i).
        for jn in &self.junctions {
            let j = jn.pipe;
            let mut num = 0.0f32;
            let mut den = 0.0f32;
            for &k in &jn.upstream {
                num += self.pipes[k].admittance * self.fo[k];
                den += self.pipes[k].admittance;
            }
            num += self.pipes[j].admittance * self.bo[j];
            den += self.pipes[j].admittance;
            let p_j = if den > 1e-12 { 2.0 * num / den } else { 0.0 };
            for &k in &jn.upstream {
                self.bi[k] = p_j - self.fo[k];
            }
            self.fi[j] = p_j - self.bo[j];
        }

        // ---- open end -------------------------------------------------------
        // An open pipe end reflects the wave back inverted. What escapes is
        // the radiated sound, proportional to (1 + R) times the outgoing wave.
        for (o, &k) in self.open_ends.iter().enumerate() {
            self.bi[k] = -self.open_end_reflection * self.fo[k];
            self.outputs[o] = (1.0 + self.open_end_reflection) * self.fo[k];
        }

        // ---- commit ---------------------------------------------------------
        for k in 0..n {
            let (f, b) = (self.fi[k], self.bi[k]);
            self.pipes[k].fwd.write(f);
            self.pipes[k].bwd.write(b);
        }
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
        assert!(hot.pipes[0].fwd.delay < cold.pipes[0].fwd.delay);
    }
}
