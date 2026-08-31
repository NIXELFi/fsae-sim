//! Emit reference output for the JavaScript port to check itself against.
//!
//!     cargo run -p engine-audio --release --example golden_vectors > out.json
//!
//! Two kinds of reference, because they catch different mistakes and neither
//! is sufficient alone:
//!
//! * **A short prefix of raw samples.** Catches a filter coefficient typed
//!   wrong, a sign flipped, a table indexed off by one -- anything that is
//!   wrong from the first sample. It cannot run long: the model is nonlinear
//!   and f32-in-Rust against f64-in-JavaScript will diverge given enough
//!   samples, so a long sample-wise comparison would fail for a reason that
//!   does not matter.
//!
//! * **Spectral magnitudes at the firing harmonics.** Survives that
//!   divergence, because the numerical noise does not move where the energy
//!   sits. Catches the mistakes that matter musically -- wrong firing order,
//!   wrong cycle length, a pipe tuned to the wrong length.

use engine_audio::{cbr600rr_sdm26, AudioConfig, Cabin, EngineAudio};

const PREFIX: usize = 512;

fn mag_at(buf: &[f32], fs: f32, f: f32) -> f32 {
    let mean = buf.iter().sum::<f32>() / buf.len() as f32;
    let w = 2.0 * std::f32::consts::PI * f / fs;
    let (mut re, mut im) = (0.0f32, 0.0f32);
    for (i, &v) in buf.iter().enumerate() {
        let x = v - mean;
        re += x * (w * i as f32).cos();
        im += x * (w * i as f32).sin();
    }
    (re * re + im * im).sqrt() / buf.len() as f32
}

fn main() {
    let cfg = AudioConfig {
        sample_rate: 48_000.0,
        ir_taps: 256,
        cabin: Cabin::Cockpit,
        seed: 0x5DAE_2026,
    };

    println!("{{");
    println!("  \"generatedBy\": \"cargo run -p engine-audio --release --example golden_vectors\",");
    println!("  \"engine\": \"{}\",", cbr600rr_sdm26().name);
    println!("  \"sampleRate\": {},", cfg.sample_rate);
    println!("  \"irTaps\": {},", cfg.ir_taps);
    println!("  \"cabin\": \"cockpit\",");
    println!("  \"seed\": {},", cfg.seed);
    println!("  \"prefixLength\": {PREFIX},");
    println!("  \"cases\": [");

    let cases: &[(f32, f32, f32)] = &[
        // rpm, throttle, torque N.m
        (6_000.0, 1.0, 55.0),
        (12_000.0, 1.0, 60.0),
        (3_000.0, 0.15, 8.0),
    ];

    for (n, &(rpm, throttle, torque)) in cases.iter().enumerate() {
        let mut e = EngineAudio::new(cbr600rr_sdm26(), cfg).unwrap();
        e.set_operating_point(rpm, throttle, torque);

        // Prefix, from a cold start, so the JS port is compared from an
        // identical initial state rather than from wherever it happens to be.
        let mut prefix = vec![0.0f32; PREFIX];
        e.render(&mut prefix);

        // Then settle and take a long buffer for the spectrum.
        let mut warm = vec![0.0f32; 48_000];
        e.render(&mut warm);
        let mut buf = vec![0.0f32; 24_000];
        e.render(&mut buf);

        let f_fire = rpm / 30.0;
        let harmonics: Vec<String> = [0.5f32, 1.0, 1.5, 2.0, 3.0, 4.0]
            .iter()
            .map(|&k| {
                format!(
                    "{{\"order\": {k}, \"hz\": {:.2}, \"mag\": {:.9}}}",
                    f_fire * k,
                    mag_at(&buf, cfg.sample_rate, f_fire * k)
                )
            })
            .collect();

        let samples: Vec<String> = prefix.iter().map(|v| format!("{v:.9}")).collect();

        println!("    {{");
        println!("      \"rpm\": {rpm}, \"throttle\": {throttle}, \"torqueNm\": {torque},");
        println!("      \"firingHz\": {f_fire},");
        println!("      \"harmonics\": [{}],", harmonics.join(", "));
        println!("      \"prefix\": [{}]", samples.join(","));
        print!("    }}");
        println!("{}", if n + 1 < cases.len() { "," } else { "" });
    }

    println!("  ]");
    println!("}}");
}
