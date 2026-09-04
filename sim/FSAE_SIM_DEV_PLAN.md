# FSAE Sim — session resume

Sister doc to `helios-dev/HELIOS_DEV_PLAN.md` and `ORACLE_DEV_PLAN.md`. Read this
first; update it at the end of each session.

**Status: working and shipped.** Both codebases build, run and pass their tests
(52 + 22 + 8 in Rust, the full harness in JS). Nothing is half-finished — the
open items below are decisions, follow-ups, and one thing that genuinely needs
hardware plugged in.

---

## 1. What exists, and where

Two sibling folders under `Documents/Claude Code/`. They are deliberately
separate.

**Both are now git repositories.** They were initialised partway through the
session that added the engine audio, so neither has an honest "before" commit
and both baselines say so in their message. `fsae-sim-rs` has two branches:
`main` carries the shared crates, `bevy-frontend` carries the Bevy app on top.

### `fsae-sim/` — the app

First-person SDM26 driving simulator. **Tauri v2 desktop app**; the frontend is
plain ES modules + WebGL2 with **zero dependencies and no build step**.

```bash
cargo build --release --manifest-path fsae-sim/src-tauri/Cargo.toml   # -> 6.3 MB exe
python fsae-sim/tools/serve.py                                        # browser, port 5273
node fsae-sim/tools/validate.js                                       # physics + ETC map + engine audio
node fsae-sim/tools/smoke_desktop.mjs                                 # does the EXE boot the game
python fsae-sim/tools/prepare_data.py                                 # regenerate data/ from helios-dev
```

The exe lands at `src-tauri/target/release/fsae-sim.exe`, and there is a Desktop
shortcut (**SDM26 Driver-in-Loop**) pointing at it.

**Iterate in the browser, not the exe.** `generate_context!` embeds the frontend
at compile time, so any `.js` edit forces a recompile and relink (~60-75 s).

### `fsae-sim-rs/` — Rust solver + Bevy build

**The Bevy build is not inside the desktop exe and cannot be.** They are two
separate programs with two different renderers: `fsae-sim.exe` is the Tauri app
with the WebGL2 frontend embedded at compile time, and `bevy-spike.exe` is a
native binary that draws with Bevy. Only the `engine-audio` and `sim-core`
crates are shared.

```bash
cargo test -p sim-core --release        # 22 tests
cargo test -p engine-audio --release    # 52 tests, about a second
cargo run  -p bevy-spike                # WASD, Q/E shift  (bevy-frontend branch)
cargo run  -p bevy-spike -- --chase --screenshot shot.png

# Regenerate the reference the JS engine-audio port checks itself against:
cargo run -p engine-audio --release --example golden_vectors \
  > ../fsae-sim/data/engine-audio-golden.json
```

- `crates/sim-core` — dependency-free solver. Pluggable tyre / powertrain /
  suspension / aero / solver, three fidelity levels. **Valuable on its own; not
  tied to Bevy.**
- `crates/engine-audio` — physically-modelled internal-combustion sound. Shared
  by both packages; the reference implementation the JS port is checked against.
- `apps/bevy-spike` — the Bevy build, on the `bevy-frontend` branch. Still one
  line of UI; it now has real engine sound and a glTF bodywork hook.

---

## 2. The numbers that must not move

`validate.js` (JS) and `cargo test -p sim-core` (Rust) both check these. The
Rust port reproduces the JS build almost exactly, which is the result that
matters.

| Check | Value | Anchor |
|---|---|---|
| Skidpad, 9.125 m | 4.986 s, 1.477 g | SDM26 ran **5.02 s** |
| 75 m accel, managed launch | 4.874 s | QSS says 4.2 s — see below |
| Braking from 25 m/s | 23.08 m, 1.74 g | — |
| ETC map, 4000 random curves | 0 overshoot | monotone guarantee |

**Two deliberate deviations from Helios, both load-bearing:**

1. **`muLat` is 1.573, not the lap sim's 1.368.** Helios pins 1.368 in a
   quasi-steady model that applies load sensitivity to the axle as a whole.
   This model *also* splits each axle left/right and derates for lateral
   transfer, so reusing 1.368 double-counts it and gives a 5.38 s skidpad.
   Oracle hit the same thing and solved it the same way with `mu_scale`.
2. **75 m is honestly slower than the QSS 4.2 s.** This model carries driveline
   rotational inertia (~+94 kg apparent in first) that a quasi-steady sim
   ignores entirely. The test band is set *above* 4.2 s on purpose. If it ever
   comes in at 4.2 s, something has stopped modelling the inertia.

---

## 3. Decisions worth not relitigating

- **`build.rs` stages the frontend into `dist/`**, not `beforeBuildCommand` —
  that hook only runs under the Tauri CLI, so a plain `cargo build` would embed
  a stale bundle. Per-*file* `rerun-if-changed`; directory watching misses
  in-place edits.
- **`lto = "thin"`, not fat.** Fat LTO measured 2m24s for a one-line JS change.
- **Tyre is a fitted Magic Formula, not Oracle's MF6.1.2 `.tir` evaluator.**
  The real R20 fit peaks at |mu| ~ 2.32 and does not reach peak Fy until 13-16
  degrees of slip — fine for a peak-grip lap sim, vague and disconnected to
  *drive*.
- **ETC map uses monotone cubic Hermite (Fritsch-Carlson).** A natural cubic or
  Catmull-Rom overshoots between breakpoints, which on a throttle map means
  more pedal briefly *closes* the plate.
- **Sharing sim-core with Oracle was considered and declined** (Daniel, this
  session): keep them separate for now.

---

## 4. Traps found the hard way

**The recurring one: a value captured once ignores later edits.** This bit twice
in a single session. If you make something adjustable, grep for every place it
is cached.

- `BicycleModel` cached `a` / `b` / `Fz0` / `mSprung` / axle inertias in the
  constructor -> now `refresh()` every substep.
- Cone hitbox `BODY` and car mesh `GEO` / `HUBS` were hardcoded -> now
  `bodyBoxFor()` / `hubsFor()` / `renderer.rebuildCar()`.

Others:

- **`python -m http.server` sends no `Cache-Control`.** The browser serves a
  fresh `index.html` next to a stale `main.js`; the page looks updated while the
  behaviour is yesterday's, and nothing errors. **Use `tools/serve.py`.**
- **WebView2**, all three handled in `smoke_desktop.mjs`: it exposes an
  `about:blank` target before navigating; a target can report the app URL while
  the document is still committing ("Execution context was destroyed"); and
  `child.kill()` leaves children alive that lock the user-data folder, so the
  *next* run fails with "no ready page" — pointing nowhere near the cause.
- **A running instance locks the exe** — `cargo build` fails with
  `Access is denied (os error 5)`. Kill `fsae-sim.exe` first.
- **`tail` in a pipe masks cargo's exit code.** A build that reported "exit 0"
  had actually failed. Use `${PIPESTATUS[0]}`.
- **Bevy cameras look down local -Z**, the car is modelled +X forward. Without a
  quarter turn the cockpit camera faces backwards into the roll hoop.
- **Bevy screenshots are async.** Exiting on a frame count silently writes no
  file; wait for the file to exist.
- **Bevy 0.19 API** (differs from 0.15/0.16 docs): `GlobalAmbientLight` is the
  resource, `AmbientLight` is a per-camera component; `DirectionalLight
  .shadow_maps_enabled`; `Hdr` is a marker component, not `Camera.hdr`;
  `MessageWriter<AppExit>` not `EventWriter`; `FontSize::Px`;
  `bevy::asset::RenderAssetUsages`; bloom lives in `bevy::post_process`.

---

## 5. Model results that look wrong and are not

- **A heavier car accelerates faster at pinned throttle** — 3.62 s at 340 kg
  against 3.85 s at 267 kg over 40 m. The extra rear load stops it spinning
  (slip ratio 1.44 vs 4.13). Give both a managed, traction-limited launch and
  the order flips back (3.66 vs 3.53 s).
- **Yaw inertia does not change steady-state cornering radius.** Correct — it
  only affects transient response. Useful as a sanity check.
- **Bicycle and double-track solvers agree to three figures in a straight
  line.** Correct — a double track degenerates to a bicycle under symmetric
  load.

---

## 6. Provenance — which numbers are real

The home screen tags all 55 parameters. **34 trace to the team; 21 (38%) are
estimates generated for the simulator that nobody has measured.** The biggest:

- **Yaw inertia 105 kg·m²** — wants a bifilar rig. Highest value to measure.
- Wheel and driveline rotational inertias
- Steering lock, rate and lag
- Brake torque and bias

23 parameters are live-adjustable from the home screen (sliders + number boxes),
persisted as a diff from as-shipped so moving a default later does not silently
reinterpret saved overrides. Wheelbase, weight distribution and track also
restretch the drawn body and the cone hitbox.

---

## 7. Open items

1. **No physical device has ever been tested.** This is now the largest gap and
   it needs Daniel, not more code. The gamepad path in WebView2 is unverified
   (it is Chromium, so it should behave as Edge does; `bevy_gilrs` detected
   three pads instantly, so a native reader is the known fallback). The wheel
   profile is worse than unverified — **its axis assignments are a guess**.
   Wheels do not use the Gamepad API's standard mapping and every vendor
   differs, which is exactly why the panel has a live axis monitor and a
   calibration capture. Plug a wheel in, read which axis each pedal moves, and
   the defaults can stop being a guess.
2. **The Bevy decision is still open, but the bill went down.** The spike
   answers "does it look better" — yes, modestly, mostly from shadows and PBR.
   The estimate used to include "replacing the WebAudio engine synth"; that is
   now done and shared, so the remaining cost is the ~2,500 lines of HUD / ETC
   editor / spec sheet in `bevy_ui`. Recommendation unchanged: the WebGL build
   is the product, Bevy is where the renderer question gets answered.
3. **Per-source audio levels** (master, engine, tyres, wind, cone strikes) are
   on the home screen and persist. The engine slider sits on top of a level
   that already tracks combustion power, so it is a preference rather than a
   correction.

4. **`sim-core` is not wired into `fsae-sim`.** The WASM path is designed for
   but not built, and it costs the zero-build-step frontend. The engine audio
   faced the same choice and answered it the other way — ported by hand, kept
   honest by golden vectors — which is now a worked precedent for doing the
   same with the solver if the duplication is ever judged worth it.

5. **Suspension and aero are registered but not extracted.** Both are still
   embedded in `bicycle.js` and in the Rust solvers; the registry marks them
   `extracted: false` and the picker says so rather than offering a choice that
   does nothing. Extracting them is what makes the modular framework real
   rather than shaped-correctly.

6. **Force feedback needs Mz before it needs an API.** The tyre model returns Fy
   only, and there is no kingpin or caster geometry in the parameters, so there
   is nothing to compute self-aligning torque from. Doing the API first would
   mean inventing the force, which is the one thing worth not doing.
7. **The endurance centreline has one curvature spike** at s ~ 607 m with
   R = 2.83 m, below SDM26's 2.88 m minimum turning circle. A tracing artifact
   — fix it upstream in the Helios track data, not here.
8. **MIS next steps (Daniel's):** explore and refine the venue,
   then make copies with the autocross and endurance layouts laid out inside the
   infield. The plumbing for that already exists — `Venue` and `Track` are
   interchangeable — but a combined venue-plus-course object does not, so
   whichever way it goes will need one new decision: either a `Venue` that
   carries a course, or a `Track` that carries scenery.
9. **Bevy spike rough edges** (only matter if it is promoted): the asphalt
   texture has no mipmaps and sparkles at grazing angles; no cone strikes,
   timing, audio or real UI.

---

## 8. Layout

```
fsae-sim/
  src/vehicle/   params, paramMeta (provenance + edit ranges), tyre,
                 powertrain, bicycle, etcMap, setupAdjust,
                 modules + library (vehicles as data)
  src/audio/     engineAudio (the model), engineWorklet (the audio thread)
  src/track/     course geometry, progress, cone strikes; venue.js (MIS)
  src/render/    WebGL2 renderer, procedural SDM26 geometry, venuemesh, mat4
  src/game/      input, controlProfiles, controlsPanel, HUD, timing, audio,
                 ETC editor, spec sheet, desktop
  tools/         prepare_data.py, serve.py, make_icons.py, make_mis.py,
                 plan_view.py, validate.js, smoke_desktop.mjs
  src-tauri/     build.rs stages dist/ on every cargo build

fsae-sim-rs/                       (main | bevy-frontend)
  crates/sim-core/     vehicle, tyre, powertrain, modular (suspension, aero,
                       VehicleDefinition), solver/{point_mass, bicycle,
                       double_track}
  crates/engine-audio/ engine, cylinder, exhaust, synth, filters, ir
  apps/bevy-spike/     main, car, track, ground, audio, cadmodel
```

`window.__sim` is exposed in the browser console — `__sim.car.telemetry`,
`__sim.etc.describe()`, `__sim.setup.state(0)`.

## 9. Michigan International Speedway venue

`tools/make_mis.py` -> `data/venue-mis.json`, `tools/plan_view.py` -> `mis-plan.png`.

**Built to published figures (defensible):** 2.000 mi perimeter (exact), 73 ft
width (22.251 m against 22.250 published), banking 18 deg turns / 12 front /
5 back eased across the joins, 7.23 m of rise across the turns, 1.07 m concrete
wall, 6.4 m catchfence, 9 m flat apron.

**Placeholder, and labelled as such in the JSON:** the plan outline is a stadium
oval, not MIS's D, and the infield layout (grass, 8 m access road, paddock) is
arranged plausibly rather than surveyed.

Why a stadium: a closed loop of two straights and two circular arcs **requires
equal straights**. The closure equations reduce to `S1 - S2 = 0` for any radii
and any sweep angles; MIS's published straights differ by 414 m. A three-arc
version is also inconsistent - it forces the back radius above the turn radius
by `S / (2 sin t)`, which cannot then fit a 683 m backstretch. Real ovals use
compound curves that a perimeter and a banking angle cannot recover. Swapping in
a traced outline touches only `make_mis.py`; every consumer reads the sampled
arrays.

**Driveability (Daniel's call):** infield + flat apron only. The banking is
visual, with an invisible barrier along `apronInner`. No banking physics, so the
validated reference numbers are untouched.

**Integrated and drivable.** Pick it from the track menu like a course.

- `src/track/venue.js` - `Venue` mirrors the `Track` interface (startPose,
  locate, strikeCones, conesNear, resetCones, plus the centreline arrays) so
  main.js, the HUD and the renderer need no branches. Cones and sectors are
  inert. `locate().lateral` is **outward-positive**, matching the ring offsets,
  and `constrain(car)` clamps the car back to the barrier and strips the outward
  velocity component so it slides along the wall instead of sticking or
  bouncing. `main.js` calls `constrain` before `locate` each frame.
- `src/render/venuemesh.js` - 20,766 vertex-coloured triangles: infield grass
  (fan from the centre, mown stripes by world x), paddock, access road, grass
  shoulder, concrete apron, banked surface, painted line at the barrier, wall
  inner face / face / cap, catchfence posts every 6 samples plus a top rail.
- `renderer.js` - `setTrack` branches on `kind === "venue"`, builds the venue
  mesh and nulls the ribbon; the venue draws through `progCar` (already
  two-sided with distance haze, which is what a 1.3 km bowl needs) on an
  identity model matrix. The ground shader gained `uDrop` so the procedural lot
  sinks 0.35 m underneath.
- The catchfence is posts and a rail, not a mesh sheet - there is no alpha
  blending in this renderer and a solid panel would wall the sky off.

Verified in the exe by `smoke_desktop.mjs`, which now loads the venue, drives at
the banking at 12 m/s on full throttle for 4 s and asserts the car is held:
worst lateral −11.125 m against a −11.125 m barrier, `glError` 0.

**Second colour-space trap.** The venue first rendered near-black. This
renderer's colours are in **display space, not linear** — its own procedural
asphalt is 0.30. The palette had been converted to linear first (asphalt 0.04),
which is correct for a PBR pipeline and wrong for this one. Same shape of bug as
the cached-value trap in §4: an assumption that is right somewhere else.

**Lesson worth keeping:** the ring offsets were initially inverted. The loop is
built turning left, so it runs counterclockwise and its enclosed area is on the
LEFT of travel - the outward normal is `(sin h, -cos h)`, not the left normal.
Every cross-section measurement still checked out because the rings were
self-consistent, just collectively inside out. It survived a numeric review and
was only caught by drawing the plan. **Draw geometry before trusting it.**

## 10. Engine audio

**Sound is generated, not sampled. There are no recordings anywhere.**

Adapted from [`ange-yaghi/engine-sim`](https://github.com/ange-yaghi/engine-sim),
**MIT**. Note that the repository usually linked —
`Engine-Simulator/engine-sim-community-edition` — ships the built application
and contains **no source at all** ("You'll notice that there is no application
code here"). The algorithms here follow the original open codebase.

```text
crank angle
  -> per-cylinder pressure (single-zone, Wiebe heat release)
  -> flow through the exhaust valve
  -> exhaust waveguide (primaries -> collector -> tailpipe -> open end)
  -> synthesiser (jitter, DC removal, derivative, noise, convolution)
  -> samples
```

Implemented twice: `fsae-sim-rs/crates/engine-audio` (reference) and
`fsae-sim/src/audio/engineAudio.js` (port). WASM was the alternative and was
declined for the same reason as before — it costs the zero-build-step frontend.
They are kept in lockstep the way `sim-core` is: the Rust side emits golden
vectors, `tools/validate.js` checks the JS against them. Measured agreement is
2e-3 over 512 samples, which is f32-against-f64 rounding and nothing else.

**The torque curve is an input.** `set_operating_point(rpm, throttle, torque)`
solves the Wiebe heat release so the modelled cycle does that much indicated
work. Feed it the Helios CFD sweep — which is what both builds do — and the note
is generated from a cylinder trace doing the same work the car is doing.

**Things that are emergent rather than scripted**, because the model is physical:
cylinder count and crank phasing set the firing pattern; primary length sets the
resonance; exhaust gas temperature changes the speed of sound, so the tuned
length shifts with load and the note moves on the overrun.

### Four bugs that measurement caught and review would not have

The first two stopped the pitch tracking rpm. The last two made it correct and
unlistenable, which is a distinct failure mode and one that no test asserting
correctness would ever have caught.


1. **The port flow was never choked.** The incompressible orifice equation
   returns about 930 m/s through the exhaust valve at blowdown — supersonic. It
   emptied the cylinder in a few crank degrees and slammed the waveguide.
2. **The cylinder end of each primary reflected rigidly** regardless of valve
   position. A wide-open exhaust valve is a hole into a large volume, not a
   mirror.

Together these made **consecutive firing pulses correlate at −0.35** — inverted,
so the pitch did not track rpm. Fixed (choke at Mach 1; reflection
`r = (A_pipe − A_valve)/(A_pipe + A_valve)`), the spectrum is a clean harmonic
series on the firing frequency with off-harmonics **200–500× down** and no
half-order content, which is exactly right for an even-firing four.

3. **The output was clipped flat at every operating point.** Measured RMS was
   0.99 against a ±1 clamp -- a square wave, not levelled audio. The leveller
   was a peak follower with a 1 ms attack, and a blowdown transient is about
   *ten samples* wide at 48 kHz. It could never respond in time, so the gain was
   always set by the quiet stretch between pulses and every pulse then arrived
   into a gain far too high. Replaced with an RMS follower averaging over 150 ms
   -- longer than a firing period even at idle -- plus `tanh` soft clipping, so
   transients are rounded rather than sheared.

4. **The derivative was 2500x too large.** `DerivativeFilter` divided by `dt`,
   which multiplies by the sample rate, so `df_f_mix = 0.01` blended in a term
   already vastly larger than the signal it was seasoning. Since a derivative's
   gain rises linearly with frequency, the result was 84% of the output energy
   between 1.5 and 4 kHz at 3000 rpm against 2.8% below 500 Hz. Now normalised
   to unity gain at a reference frequency, so the mix fraction means what it
   says.

Two further changes were needed to make low rpm bearable, and both were missing
physics rather than tuning:

- **The orifice law is singular at zero pressure difference.**
  `u = sign(dp) sqrt(2|dp|/rho)` has *infinite* slope at `dp = 0`, and after
  blowdown the cylinder sits near the runner pressure for the whole exhaust
  stroke -- so every small returning wave was amplified into a large flow swing
  that injected straight back into the runner. A limit cycle: cylinder pressure
  a clean 76 Hz at 3000 rpm while the valve flow oscillated at **3 kHz** and
  dominated the entire output. Fixed with a linear (viscous) region below
  1500 Pa, which is also the more physical law there, plus a short port
  inertance. Both are real: the square-root law is the fully turbulent limit,
  and the gas in the port has mass.

- **Pipe losses were frequency-independent, and there was no muffler.** A flat
  multiplier gives every mode the same Q, so at low rpm -- where pulses are far
  apart -- the 1-2 kHz pipe modes rang on between them until they were all you
  could hear. Real losses grow with frequency (boundary-layer viscous and
  thermal), so each pipe now damps through a one-pole low-pass. The tailpipe
  carries heavy damping standing in for the muffler the car must have anyway:
  FSAE caps noise at 110 dBA.

**Result:** every operating point from idle to the limiter is now 99%+ energy
below 1.5 kHz, peaks at 1x or 2x the firing frequency, peak amplitude 0.11-0.28
with real headroom, and levels that scale with effort -- idle 2.7x quieter than
full throttle, overrun 2x quieter than pulling. `validate.js` asserts all of it.

### And then it was correct, and idle was still too loud

A second round, reported the same way -- idle louder than the rest of the rev
range. The measurement that had passed said idle was already 2.7x quieter than
full throttle by RMS. The measurement that mattered said something else:

**A-weighted, the limiter was 3.2 dB above idle**, where a real engine spans
25-35 dB, and 10000 rpm was 6.8 dB *louder* than the limiter. Idle also had 30%
of its A-WEIGHTED energy above 1.5 kHz against 1% of its raw energy -- the ear
weights 2 kHz about 30 dB above 50 Hz, so a metric that ignores that cannot
answer "is this too loud". **Loudness checks are A-weighted now.**

Three causes, in order of size:

1. **The automatic gain control was the problem, not the solution.** It was
   doing exactly what it was asked -- driving every operating point to the same
   output RMS -- which for an engine is the wrong goal. It erased the loudness
   curve, and it boosted the quiet, ring-dominated idle signal fourfold,
   bringing its high-frequency noise floor up with it. Replaced by a **fixed
   pressure reference**: 90 kPa maps to full scale, and loudness is then
   whatever the physics produced.

2. **Level now tracks combustion power** -- heat release per cycle times firing
   rate -- rather than a hand-blended mix of throttle and rpm. That is the
   quantity that actually drives an exhaust, and it falls to nothing on a closed
   throttle with no special case, so the overrun goes quiet on its own.

3. **The port could draw from an infinite reservoir.** `port_pressure` returned
   ambient plus the returning wave, with no term for the flow going through it.
   A pipe is not a reservoir: push gas in and the inlet pressure rises by
   `rho c u` immediately, not after the wave's round trip. At idle the cylinder
   is at ~0.2 atm when the valve opens, so gas rushed backwards into it at sonic
   velocity and injected a **52 kPa** wave -- against 58 kPa for a full-power
   blowdown. Idle was as loud as the limiter *in the physics*, before any
   processing. Now the flow throttles itself against the pipe's characteristic
   impedance, and reverse flow additionally carries the lower discharge
   coefficient a port really has backwards (0.7).

A fourth piece was needed because the waveguide swings ~12 dB across the rev
range purely on which pipe mode the firing harmonics land on. Tuned-length
resonance is real, but 12 dB of it swamped the loudness curve. A **range-limited
compressor** (+/-8 dB, slow) removes that swing; being range-limited is exactly
what stops it becoming the AGC again -- it cannot flatten a 20 dB curve.

**Measured through the running worklet**, monotonic with load:

| | idle | 2500 part | 4000 | 7000 | 10000 | 13000 | 3000 ovr | 9000 ovr |
|---|---|---|---|---|---|---|---|---|
| dBA vs limiter | −37 | −29 | −14 | −8.6 | −3.6 | 0 | −42 | −29 |

**Correctness and listenability are different properties, and only one of them
had tests.** The spectral checks that proved the model right -- energy on the
firing harmonics, no half-order -- were all passing throughout. They are
insensitive to *where else* the energy is, to clipping, and to level. The tonal
balance suite exists because of that gap.

**Test the spectrum, not the waveform.** The first pitch test used zero-crossing
counting and measured the 3.9 kHz pipe resonance rather than the 200 Hz firing
rate, reporting roughly the same answer at every rpm. Autocorrelation fixed that
but needed an octave guard restricted to *local maxima* — a smooth signal's
autocorrelation rises gradually, so "first lag above a threshold" lands on the
slope. The assertion that actually earns its keep is the spectral one.

### Performance

Rust 24× real time, JS 12.5× at 48 kHz with four cylinders and a 256-tap
impulse response. The dominant cost was **not** the transcendental functions —
tabulating those barely moved it. It was twelve `Vec` allocations per sample in
the waveguide, half a million a second. Preallocating doubled throughput.

**The RNG is `xorshift32`, deliberately.** A better 64-bit generator cannot be
reproduced in JavaScript without `BigInt`, and since jitter and air noise both
reach the output, different noise would make the two waveforms diverge from the
first sample — leaving nothing to compare.

---

## 11. Control profiles

`src/game/controlProfiles.js` defines what each device *is*; `input.js` reads
whatever the active profile says. Adding a device means adding a profile.

Four shipped: keyboard/mouse, Xbox, PlayStation, wheel-and-pedals. They are not
variations on a theme — they differ in what the driver can physically command:

| | steering rate | accel | deadzone | curve |
|---|---|---|---|---|
| keyboard | 180 °/s | 700 °/s² | 0 | linear |
| gamepad | 300 °/s | 2200 °/s² | 0.10 | 1.7 expo |
| wheel | 720 °/s | 12000 °/s² | **0** | **1.0** |

All at the **road wheel**, matching `BicycleModel.delta`. Divide by
`steeringRatio` for rim figures.

**Steering is now a rate- *and* acceleration-limited servo.** The acceleration
limit is new. Without it a step input produces a step in steering *velocity*,
which no hand and no steering motor can do — and on a keyboard, where every
input is a step, it is the difference between the car darting and the car being
steered. `steeringServo` on the model is **configuration, not state**: it is
deliberately not reset by `reset()`, or a respawn would silently drop the
driver's profile. `steerRateDegPerS` *is* state and is reset — same shape as the
clutch-mismatch bug in §4.

With no profile attached the servo uses an effectively infinite acceleration and
reduces exactly to the previous behaviour, so **the validated numbers do not
move.**

**Wheels get the accuracy work**, because they are the device where software
smoothing is a defect rather than a feature:

- **Rim-to-road-wheel mapping.** `match-car` turns the rim through the car's real
  ratio: SDM26's 28° lock through 4.0 is **112° at the rim, lock to lock**. Set
  the driver software to 112 and hand position *is* front-wheel angle.
  `scale-to-lock` spreads whatever rotation the wheel is set to across full lock
  — forgiving, but the ratio becomes a fiction and the steering is eight times
  slower than the real car's.
- **Pedal calibration against real axis travel.** A G29 brake rests near −1 and
  tops out near +1; a load cell may never reach +1 at any force a person can
  apply. Without per-axis min/max the same code reads full brake at rest on one
  device and half brake at the stop on another.
- **A live axis monitor.** Wheels do not use the Gamepad API's standard mapping
  and every vendor differs, so there is no table that is right for all of them.
  Watching the numbers move while you press a pedal is the only reliable way.

**Force feedback is declared and documented but not implemented.**
`forceFeedback.enabled` is false everywhere. What it needs: the tyre model to
return **Mz** (it returns Fy only) and a kingpin/caster geometry block in the
parameters, so self-aligning torque can come out of the physics rather than
being invented. The web platform has no force-feedback API beyond dual-rumble,
so the desktop build will need a native path.

---

## 12. Modular vehicles

A vehicle is **data**, not a class: JSON naming which model to use for each
subsystem, plus the parameters. That is what makes it savable, exportable,
diffable and readable by both builds.

```
tyre · powertrain · suspension · aero · engineAudio
```

- `src/vehicle/modules.js` — the registry, and the SDM26 reference definition.
  Its `params` block **references** the live `SDM26` object rather than copying
  it, so there is still exactly one source of truth and `paramMeta.js` still
  describes it.
- `src/vehicle/library.js` — duplicate, edit, swap a module, export, import.
- `sim-core/src/modular.rs` — the same shape in Rust, plus `SuspensionModel` and
  `AeroModel` traits with `rigid` and `none` alternatives (useful references:
  they isolate how much of a result comes from load transfer or downforce).

**Saved vehicles store a sparse diff against their ancestor**, not a full copy —
the same rule the parameter and control overrides use. Improving a shipped
default then reaches every car derived from it, instead of each being silently
pinned to whatever shipped the day it was created.

**Module ids are strings, not an enum**, including in Rust. An enum would be
tidier and would immediately stop matching a JSON file written by the other
build.

**No second vehicle exists, on purpose.** The framework is the deliverable;
inventing a plausible-looking car would put numbers in the repository nobody has
measured. Two subsystems are also still *embedded in the solver* rather than
extracted — suspension and aero are registered and described but marked
`extracted: false`, and the picker says so instead of offering a choice that
silently does nothing. **Extracting them is the next real step.**

---

## 13. CAD bodywork

**Working in both builds.** Drop a `.glb` at `fsae-sim/data/car.glb` or
`fsae-sim-rs/apps/bevy-spike/assets/car.glb` and it replaces the procedural
body. Absent is the normal case and draws the procedural SDM26.

Neither model is committed: the reference car is a crude box assembly and
shipping it would replace a good procedural body with a worse one.

```
tools/glb.py               dependency-free glTF binary reader/writer
tools/make_reference_car.py  a model in the expected frame -- fixture and template
tools/check_car_glb.py     validates an export, and says what to change
src/render/glbcar.js       the loader for the WebGL2 build
apps/bevy-spike/src/cadmodel.rs  the Bevy side (glTF is native there)
```

**Frame:** origin at the CG projected to the ground, +X forward, +Y up, +Z left,
metres. Front axle x = +0.788, rear x = −0.742, hubs y = +0.200. Nodes:
`body`, `wheel_fl/fr/rl/rr`, `steering_wheel`.

**The division of labour: the file supplies geometry, the simulator supplies
motion.** Wheel geometry is re-centred on its own hub as it is read, and the hub
position is taken from the node. A wheel merged into the body cannot turn; one
left at its world position orbits the car.

### What was wrong with the first attempt

The module existed, compiled, had tests, and **was never called**. Worth
recording because everything looked finished:

- `spawn_if_present` was defined and no code invoked it.
- Its existence check looked in `assets/` relative to the working directory
  while Bevy looked next to the executable. Fixed by computing the root once
  and configuring `AssetPlugin` with it -- the failure mode when those disagree
  is "found the file, then reported Path not found", which is maximally
  confusing.
- The `NODES` table was documentation nothing read, so even once loaded the
  wheels would not have steered.
- The documented origin was the **front axle**, which is wrong -- both builds
  put the car root at the CG. A model built to that doc would have sat 0.788 m
  out.

### Bugs the work turned up

- **`mesh_bounds` read node-local bounds**, so a correctly built car -- wheels
  centred on their own nodes, as required -- was reported as buried 200 mm
  underground. A checker giving confident wrong answers is worse than none.
- **glTF indices are per-accessor, not per-buffer.** The first in-memory test
  fixture indexed 3,4,5 into a 3-vertex accessor and produced NaN geometry,
  which renders as nothing with no error anywhere. The loader now detects
  out-of-range indices and reports them.
- **An optional file could stop the simulator starting.** The desktop asset
  server answers a request for a MISSING file with the index page rather than a
  404, so an absent `car.glb` arrived as HTML, failed to parse, rejected the
  load, and the game never booted -- in the desktop build only. `loadCarModel`
  now checks the magic bytes and never throws. Caught by the smoke test, which
  is the whole reason it exists.

### Not done

The WebGL2 loader reads positions, normals, indices and one base colour per
material. It ignores textures, node rotation and scale, skinning and animation.
Rotation and scale on a node are *reported* by the checker rather than applied,
because an export carrying them usually means the model was not baked into the
right frame -- but a model that legitimately needs them will come in wrong.

## 14. The idle point, and what it pinned down

Daniel measured the car: **idle is about 2000 rpm with the throttle plate at
about 14%.** Two numbers, and between them they fixed four things that had been
guesses.

**They cross-check each other.** Before using either, the model already said a
14% plate balances friction at about 2350 rpm — from the CFD torque curve and
the friction model alone. Agreement to a few hundred rpm on numbers that had
never seen the measurement.

**They pin the low-rpm end of the torque curve.** Below the sweep's first point
(4000 rpm) the curve was extrapolated to `0.35 x peak`, invented. Requiring a
14% plate to balance friction at 2000 rpm forces `wot(2000) = 34 N.m`, which is
**0.56 x peak** — and that independently lands in the 55-70% a naturally
aspirated four really makes there. The old 0.35 could not sustain an idle at any
plate opening.

**Idle is now modelled as a plate position, not a torque fudge.** What used to
be `t += (idleRpm - rpm) * 0.02` is a proportional idle-speed control on the
plate, which is what an ETC idle circuit actually is. A *fixed* 14% opening is
not enough and the reason is worth remembering: below idle the WOT curve is flat
and so is friction, so a fixed plate makes net torque very nearly zero at every
sub-idle rpm. That is a **neutral** equilibrium, and the engine settled wherever
it happened to be -- 982 rpm in the running game. The error term supplies the
restoring force; at the target the commanded opening is exactly 14%.

**Two clutch faults surfaced only because the idle became observable.** Neither
would have been found by looking at lap times.

1. **The clutch could lock below idle speed.** At a standstill it locked and
   pinned the engine to a `0.6 x idleRpm` stall guard. A clutch cannot be locked
   below idle -- that is precisely why you slip one pulling away.
2. **The clutch never fully released.** `max(0.1, launch)` meant it always
   carried about 26 N.m, several times what the engine makes at idle, so a
   stationary car dragged its own engine down and could never idle at all. A
   real FSAE car does not creep.

**Cost: the 75 m went from 4.762 s to 4.874 s.** The clutch now slips until the
wheel reaches idle-equivalent speed (2.4 m/s in first) instead of locking early
against a fictitious 960 rpm floor. That is more honest and in the same
direction as the driveline-inertia note in section 2 -- the launch is slower
because less of it is free. Skidpad and braking did not move.

**Verified in the running game:** stationary, pedal at rest, the engine settles
at 2001 rpm with the plate at 14.0%, and the measured spectral peak of the
engine note is 67.4 Hz against a firing frequency of 66.7 Hz.
