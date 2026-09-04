//! Engine sound for the Bevy build.
//!
//! Same synthesiser as the web build -- the `engine-audio` crate -- driven from
//! the solver's rpm, throttle and torque. What differs is only how samples
//! reach a speaker.
//!
//! This uses `cpal` directly rather than `bevy_audio`. Bevy's audio is built
//! around decoding assets: you hand it a sound and it plays it. There is no
//! asset here, only a generator that must be pulled at exactly the rate the
//! output device consumes, forever, without ever being restarted -- and the
//! shortest path to that is to own the output stream. It is also the same
//! arrangement the web build ends up with, where an AudioWorklet owns the
//! model on the audio thread.
//!
//! The lock is the one subtle part. The audio callback runs on a real-time
//! thread and must never block, so the main thread only ever `try_lock`s to
//! post an operating point. Dropping an update is harmless -- the next frame
//! posts a newer one a few milliseconds later -- whereas stalling the audio
//! thread is an audible dropout.

use std::sync::{Arc, Mutex};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use engine_audio::{cbr600rr_sdm26, AudioConfig, Cabin, EngineAudio};

/// Keeps the output stream alive. Dropping this stops the sound.
pub struct EngineSound {
    _stream: cpal::Stream,
    shared: Arc<Mutex<EngineAudio>>,
    /// Last operating point that actually reached the model, for the HUD.
    pub last: OperatingPoint,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OperatingPoint {
    pub rpm: f32,
    pub throttle: f32,
    pub torque_nm: f32,
}

impl EngineSound {
    /// Open the default output device and start generating.
    ///
    /// Returns `None` rather than failing the app when there is no audio
    /// device: a machine in a test rig or a CI runner has none, and a driving
    /// simulator that refuses to start because it cannot make noise is worse
    /// than a silent one.
    pub fn start() -> Option<Self> {
        let host = cpal::default_host();
        let device = host.default_output_device()?;
        let config = device.default_output_config().ok()?;
        let sample_rate = config.sample_rate().0 as f32;
        let channels = config.channels() as usize;

        let engine = EngineAudio::new(
            cbr600rr_sdm26(),
            AudioConfig {
                // Build the model at the device's real rate. The waveguide
                // delay lines are sized in samples, so a model built for 48 kHz
                // running on a 44.1 kHz device would put every pipe resonance
                // about 9% sharp.
                sample_rate,
                cabin: Cabin::Cockpit,
                ..Default::default()
            },
        )
        .ok()?;

        let shared = Arc::new(Mutex::new(engine));
        let render = Arc::clone(&shared);
        let mut scratch: Vec<f32> = Vec::new();

        let err_fn = |e| eprintln!("engine audio stream error: {e}");
        let stream = match config.sample_format() {
            cpal::SampleFormat::F32 => device.build_output_stream(
                &config.into(),
                move |out: &mut [f32], _: &cpal::OutputCallbackInfo| {
                    let frames = out.len() / channels.max(1);
                    scratch.resize(frames, 0.0);

                    // Never block the audio thread. A missed buffer is a
                    // dropout; a missed parameter update is nothing.
                    match render.try_lock() {
                        Ok(mut e) => e.render(&mut scratch),
                        Err(_) => scratch.iter_mut().for_each(|s| *s = 0.0),
                    }

                    for (frame, &s) in out.chunks_mut(channels.max(1)).zip(scratch.iter()) {
                        for sample in frame.iter_mut() {
                            *sample = s;
                        }
                    }
                },
                err_fn,
                None,
            ),
            // Only f32 is wired up. Every desktop host this targets offers it,
            // and silently converting formats here would hide a real problem.
            other => {
                eprintln!("engine audio: unsupported sample format {other:?}, running silent");
                return None;
            }
        }
        .ok()?;

        stream.play().ok()?;
        Some(Self {
            _stream: stream,
            shared,
            last: OperatingPoint::default(),
        })
    }

    /// Post a new operating point. Called once a frame, not per sample.
    pub fn set_operating_point(&mut self, rpm: f32, throttle: f32, torque_nm: f32) {
        self.last = OperatingPoint {
            rpm,
            throttle,
            torque_nm,
        };
        if let Ok(mut e) = self.shared.try_lock() {
            e.set_operating_point(rpm, throttle, torque_nm);
        }
    }

    pub fn set_running(&mut self, running: bool) {
        if let Ok(mut e) = self.shared.try_lock() {
            e.set_running(running);
        }
    }

    /// Cockpit or trackside. Changes the convolution impulse response.
    pub fn set_cabin(&mut self, cabin: Cabin) {
        if let Ok(mut e) = self.shared.try_lock() {
            e.set_cabin(cabin);
        }
    }
}
