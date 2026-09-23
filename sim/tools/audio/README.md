# Engine-audio analysis tools

Tools to measure the sim's engine sound and compare it with a real recording,
aligned on rpm rather than time. Everything writes to `out/`, which git ignores.
Nothing here changes the sim: `render_sim.mjs` imports
`src/audio/engineAudio.js` directly, so it always renders the live model.

**Scope.** Only the physical engine model (`engineAudio.js`: cylinders, exhaust
waveguide, synthesiser) is rendered. The WebAudio layers in `src/game/audio.js`
(intake/induction noise, tyre squeal, wind, shift clunk, master mix) are not in
it. A real clip has all of those plus the environment, so the comparison looks
at engine orders and relative balance, not absolute level.

Requirements: Node 24, Python 3 with numpy, scipy and matplotlib, and ffmpeg for
video. `extract.py` looks for ffmpeg in `--ffmpeg`, then `$FFMPEG`, then `PATH`,
then the ffmpeg-static copy in the Claude scratchpad.

Run everything from this folder (`sim/tools/audio`).

## One command each

| Step | Command | Output |
|---|---|---|
| Extract audio from a video | `python extract.py clip.MOV -o out/real.wav` | mono 32-bit float WAV, 48 kHz |
| Render the sim from telemetry | `node render_sim.mjs --csv ../../runs/<run>/telemetry.csv` | `out/sim_<run>.wav` + `.ops.json` |
| Render a synthetic sweep | `node render_sim.mjs --sweep standard` (`wot`, `idle`, `limiter`, `overrun`, `steps`, `accel`) | `out/sweep_<name>.wav` + `.ops.json` |
| Render along an rpm trace | `node render_sim.mjs --rpm-track trace.csv` (columns `t,rpm[,phase]`) | WAV + ops |
| Analyse any WAV | `python analyze.py out/x.wav [--rpm-min 4000 --rpm-max 15000]` | `out/x_analysis/` |
| Compare real vs sim | `python compare.py out/real.wav out/sim.wav --real-rpm-min 4000 --real-rpm-max 15000` | `out/compare_*/` |
| Real launch/accel clip, start to finish | `python accel_ab.py clip.MOV` | `out/<clip>_ab/` (listening files, video with sim audio), then run the printed `compare.py` command |
| Prove the tooling | `python validate.py` | `out/validation/`, exits 1 on any failure |
| Score a model change against a clip | `python score_ab.py out/<clip>_ab [--overrides tweak.json] [--t-max 2.4]` | one JSON line: per-order sim-minus-real, slope, centroid |

## What each step does

**render_sim.mjs** maps inputs to operating points the same way the game does.
Telemetry uses the replay path (`main.js replayAudioState`): throttle is the
plate (`engine.tps`), torque is the indicated torque `plate * (wot + motoring)`,
the cut applies on the limiter, overrun is a closed plate above 6000 rpm at
speed, and throttle and torque are zero while shifting. Updates run at 100 Hz,
with a 1 s pre-roll so the resonance compressor settles. The limiter and launch
control bounce the rpm between the limit and the limit minus the hysteresis,
with the cut on the falling half, the way the live game toggles it. The sidecar
`.ops.json` records rpm, throttle, torque, cut, overrun and segment against time.

**analyze.py**
- *rpm track.* Harmonic-sum salience on a whitened log-frequency spectrogram,
  with 120 bins per octave. Firing harmonics sit at `k * rpm/30`. The salience
  is penalised at the odd crank orders to stop octave jumps. A Viterbi path adds
  a cost per rpm step, then parabolic refinement. `--rpm-min/--rpm-max` sets the
  search band (the "hint"). `--fund-order` is 2 for an inline four.
- *orders.* Angle-domain order tracking: the signal is resampled to 64 samples
  per revolution and split into 16-revolution Hann blocks. That gives exact
  1/16-order bins at any ramp rate. Each order's level has the local
  between-order noise floor subtracted, and its SNR is kept.
- *loudness.* A-weighted and unweighted RMS in 100 ms windows, in dBFS.
- *balance.* Spectral centroid; broadband tilt, a regression of 1/3-octave band
  levels from 100 Hz to 8 kHz, in dB/oct; and harmonic tilt, the slope of firing
  harmonics 2 to 12, in dB/oct.
- *holds.* Stretches of flat rpm with a cut in the envelope, which is what the
  limiter and launch control sound like. For each one it reports the rpm, the
  bounce, the envelope cut rate and the depth. The envelope is high-passed above
  the wind before it is measured.
- *outputs.* `spectrogram.png` (plain, and with orders 1/2/4/8 overlaid),
  `rpm_track.png`, `orders_vs_rpm.png`, `order_map.png`,
  `loudness_balance.png`, `frames.csv`, `orders.csv` and `summary.json`
  (tables per rpm bin). A sidecar next to the WAV is picked up automatically, and
  the tracking error against it is reported.

**compare.py** analyses both clips and bins them on rpm. By default it keeps
only rising-rpm frames, which are the pulls. It compares:
- order levels relative to order 2. An order is only used where it clears the
  floor by at least 8 dB in at least half the blocks of an rpm bin.
- harmonic and broadband tilt, and centroid.
- the shape of the loudness curve.
- holds.

Outputs: `report.txt`, `compare.json`, `spectrograms_side_by_side.png`,
`order_profiles.png`, `order_maps_side_by_side.png` and `balance_vs_rpm.png`.
Relative levels cancel the microphone gain, AGC and distance. Median filtering
and gating keep wind, tyre and road noise out.

**accel_ab.py** is for a fixed camera watching a launch and accel run, where the
car drives away from the camera. It works in five steps:
1. Tracks the clip and finds the end of the launch-control hold and the upshifts.
2. Assigns gears and undoes the Doppler shift from the gearbox and tyre radius.
   It assumes no wheelspin. The camera geometry is set by `--cam-behind-m`
   (default 6.1 m, about 20 ft) and `--cam-side-m`.
3. Renders the live sim model along that source rpm, with launch control and the
   shift cuts included.
4. Re-radiates the render to the camera, adding the propagation delay (and so the
   Doppler) and 1/r spreading.
5. Writes the listening files:
   - `listen_AB_real_then_sim.wav`: real, a gap, then sim, matched on A-weighted
     loudness.
   - `listen_sim_at_camera.wav` and `listen_sim_onboard.wav`.
   - `<clip>_with_sim_audio.mp4`: the original video with the sim audio.

It checks each measured shift drop against the gearbox ratios, which is a
sanity check on the tracking (`accel_info.json`).

**Useful flags.**
- `render_sim.mjs --cabin trackside` sets the listener. The game uses
  `trackside` whenever the camera is outside the car, so exterior videos are
  compared with it. `accel_ab.py` and `score_ab.py` default to trackside.
- `render_sim.mjs --overrides tweak.json` lets you try a change without editing
  the model: `{"spec": {...}, "params": {...}, "config": {...}}`. The spec
  entries are deep-merged into `cbr600rrSdm26()`, and pipe entries may be given
  as `{lengthM, diameterM, loss, dampingHz}`.
- `accel_ab.py --tag _v2` keeps each version's listening files side by side.
- `compare.py --t-max S` and `score_ab.py --t-max S` use only the first S
  seconds. In a drive-away clip, that is the part where the car is still close
  to the camera. Later, distance takes level and treble off the recording, so
  the early part is the better reference for spectral balance.

## Validation (`python validate.py`)

| Test | Result (2026-09-23, 4-2-1 model) |
|---|---|
| A. Synthetic pull, two shifts and a 24 Hz limiter, with road noise at -15 dB, wind **6 dB louder than the engine**, a camera AGC and 16-bit | rpm median error 0.28%, 99% of frames within 5%; orders 4-12 within 0.8 dB; buried orders (0.5, 1, 1.5, 3) withheld, not misread; limiter cut rate 24.0 Hz |
| A2. Same pull, quiet field | all 9 orders, including half orders, within 1.3 dB |
| B. Octave traps (order 4 +6 dB over order 2; order 2 at -10 dB re order 4) | 100% of frames within 5% |
| C. Sim renders vs the rpm that drove them (throttle open) | standard sweep 0.25% median / 100% within 5%; WOT steps 0.24% / 99.5%; accel sweep 0.25% / 99%+; xu5e autocross telemetry 0.34% / 99%+ |
| D. compare.py: a sim render vs the same render with road noise and AGC (and again with wind 6 dB over the engine) | orders within 1.6 dB (8 orders compared); with wind, the few orders that clear the floor stay within 2.7 dB |

Known weak spots:
- The sim's own launch-control stutter at 7000 rpm is broadband, and it tracks
  only 52% of frames within 5%.
- Shift instants track at about 2.4% error.
- Closed-throttle overrun in the sim is weakly tonal.
