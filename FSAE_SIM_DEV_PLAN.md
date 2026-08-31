# FSAE Sim — session resume

Sister doc to `helios-dev/HELIOS_DEV_PLAN.md` and `ORACLE_DEV_PLAN.md`. Read this
first; update it at the end of each session.

**Status: working and shipped.** Both codebases build, run and pass their tests.
Nothing is half-finished — the open items below are decisions and follow-ups,
not broken work.

---

## 1. What exists, and where

Two sibling folders under `Documents/Claude Code/`. They are deliberately
separate.

### `fsae-sim/` — the app

First-person SDM26 driving simulator. **Tauri v2 desktop app**; the frontend is
plain ES modules + WebGL2 with **zero dependencies and no build step**.

```bash
cargo build --release --manifest-path fsae-sim/src-tauri/Cargo.toml   # -> 6.3 MB exe
python fsae-sim/tools/serve.py                                        # browser, port 5273
node fsae-sim/tools/validate.js                                       # physics + ETC map
node fsae-sim/tools/smoke_desktop.mjs                                 # does the EXE boot the game
python fsae-sim/tools/prepare_data.py                                 # regenerate data/ from helios-dev
```

The exe lands at `src-tauri/target/release/fsae-sim.exe`, and there is a Desktop
shortcut (**SDM26 Driver-in-Loop**) pointing at it.

**Iterate in the browser, not the exe.** `generate_context!` embeds the frontend
at compile time, so any `.js` edit forces a recompile and relink (~60-75 s).

### `fsae-sim-rs/` — Rust solver + Bevy spike

```bash
cargo test -p sim-core --release      # 21 tests, under a second
cargo run  -p bevy-spike              # WASD, Q/E shift
cargo run  -p bevy-spike -- --chase --screenshot shot.png
```

- `crates/sim-core` — dependency-free solver. Pluggable tyre / powertrain /
  solver, three fidelity levels. **Valuable on its own; not tied to Bevy.**
- `apps/bevy-spike` — throwaway visual spike. One line of UI, on purpose.

---

## 2. The numbers that must not move

`validate.js` (JS) and `cargo test -p sim-core` (Rust) both check these. The
Rust port reproduces the JS build almost exactly, which is the result that
matters.

| Check | Value | Anchor |
|---|---|---|
| Skidpad, 9.125 m | 4.986 s, 1.477 g | SDM26 ran **5.02 s** |
| 75 m accel, managed launch | 4.762 s | QSS says 4.2 s — see below |
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

1. **Gamepad in WebView2 is still unverified with a physical pad.** It is
   Chromium so it should behave as Edge does, but it has never been confirmed.
   `bevy_gilrs` detected three pads instantly, so a native reader is the known
   fallback. Thirty seconds of Daniel's time to settle.
2. **The Bevy decision is open.** The spike answers "does it look better" — yes,
   but modestly, and the win came mostly from shadows and PBR. The real bill is
   ~2,500 lines of HUD / ETC editor / spec sheet rebuilt in `bevy_ui`, plus
   replacing the WebAudio engine synth. Recommendation on file: adopt
   `sim-core` under the existing app via WASM and treat Bevy as optional.
3. **`sim-core` is not wired into `fsae-sim`.** The WASM path is designed for
   but not built. Note that it costs the zero-build-step frontend.
4. **The endurance centreline has one curvature spike** at s ~ 607 m with
   R = 2.83 m, below SDM26's 2.88 m minimum turning circle. A tracing artifact
   — fix it upstream in the Helios track data, not here.
5. **MIS next steps (Daniel's, this session):** explore and refine the venue,
   then make copies with the autocross and endurance layouts laid out inside the
   infield. The plumbing for that already exists — `Venue` and `Track` are
   interchangeable — but a combined venue-plus-course object does not, so
   whichever way it goes will need one new decision: either a `Venue` that
   carries a course, or a `Track` that carries scenery.
6. **Bevy spike rough edges** (only matter if it is promoted): the asphalt
   texture has no mipmaps and sparkles at grazing angles; no cone strikes,
   timing, audio or real UI.

---

## 8. Layout

```
fsae-sim/
  src/vehicle/   params, paramMeta (provenance + edit ranges), tyre,
                 powertrain, bicycle, etcMap, setupAdjust
  src/track/     course geometry, progress, cone strikes; venue.js (MIS)
  src/render/    WebGL2 renderer, procedural SDM26 geometry, venuemesh, mat4
  src/game/      input, HUD, timing, audio, ETC editor, spec sheet, desktop
  tools/         prepare_data.py, serve.py, make_icons.py, make_mis.py,
                 plan_view.py, validate.js, smoke_desktop.mjs
  src-tauri/     build.rs stages dist/ on every cargo build

fsae-sim-rs/
  crates/sim-core/   vehicle, tyre, powertrain,
                     solver/{point_mass, bicycle, double_track}
  apps/bevy-spike/   main, car, track, ground
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
