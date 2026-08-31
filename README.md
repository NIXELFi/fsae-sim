# SDM26 Driver-in-Loop

A first-person FSAE driving simulator for Sun Devil Motorsports' SDM26, on the
traced 2026 FSAE Michigan autocross and endurance courses, driven with an Xbox
One controller.

Everything that could come from real team data does. The car is the Helios lap
sim's SDM26. The engine is the Helios CFD module's 1-D FV engine sweep. The
courses are the same traced geometry the Oracle lap sim times.

## Running it

It is a desktop app — a Tauri v2 window with the whole game embedded in the
executable. No server, no install, nothing to keep running in a terminal.

```bash
cargo build --release --manifest-path fsae-sim/src-tauri/Cargo.toml
```

That produces `src-tauri/target/release/fsae-sim.exe`. Double-click it.

You need the Rust toolchain and VS Build Tools, which are already set up on this
machine for Helios. You do **not** need Node, pnpm, or the Tauri CLI: `build.rs`
stages the frontend into `dist/` on every build, so plain `cargo build` can
never ship a stale bundle. `cargo tauri build` additionally produces an NSIS
installer if you want one.

The frontend is still plain ES modules and WebGL2 with zero dependencies, so it
also runs in a browser — and that is the loop to use while working on the game:

```bash
python fsae-sim/tools/serve.py
```

The reason matters. `generate_context!` embeds the frontend into the binary at
compile time, so editing any `.js` file forces the crate to recompile and
relink. Reload a browser tab instead and the change is instant; build the exe
when you want to hand it to someone.

Use `tools/serve.py`, not `python -m http.server`. The stdlib server sends no
`Cache-Control`, so the browser caches the ES modules and a reload will serve a
fresh `index.html` alongside a stale `main.js` — the page looks updated while
the behaviour is yesterday's. `serve.py` is the same static server with caching
switched off.

## Controls

| Xbox One | Keyboard | |
|---|---|---|
| Right trigger | `W` / `↑` | Throttle (analog on the pad) |
| Left trigger | `S` / `↓` | Brake (analog on the pad) |
| Left stick | `A` `D` / `←` `→` | Steering |
| RB / LB | `E` / `Q` | Upshift / downshift |
| B | `R` | Recover to the course |
| X / Y | `T` / `C` | Traction control / camera |
| View / Menu | `Backspace` / `Esc` | Restart run / pause |
| D-pad ◄ ► | `[` `]` | Pick the setup item to adjust |
| D-pad ▲ ▼ | `-` `=` | Adjust it, 0.1% a press |
| L3 | `H` | Back to the home screen |
| — | `M` | Throttle-map editor |
| — | `F11` | Fullscreen |

Steering uses a 10% deadzone and a 1.7-power response curve. That curve is not
a feel preference: full lock is 28° and the tyre peaks at 8.5° of slip, so a
linear stick would compress everything that matters into the first third of the
travel.

Three cameras: cockpit (driver's eye, 0.66 m off the deck), nose, and chase. All
of them carry the HUD, including live APS / TPS / brake bars — APS against TPS
is worth watching, because on anything but a linear pedal map the gap between
those two bars *is* the map.

## What the model is standing on

The home screen lists the model, its degrees of freedom, and **every parameter
tagged with where it came from**. That distinction is the point: 55 parameters
run this simulator, and they are not equally trustworthy.

| Tag | Meaning | Count |
|---|---|---|
| `TEAM` | Measured, specified or validated by SDM, carried in Helios | 27 |
| `CFD` | Output of a Helios CFD model the team built (aero map, engine sweep) | 5 |
| `CAL` | Fitted so **this** model reproduces a measured SDM26 result | 2 |
| `EST` | An engineering estimate I generated. Nobody has measured it | 21 |

So 34 of 55 trace back to the team, and **21 (38%) are estimates**. Those are
the ones to replace first, and the big ones are the yaw inertia (105 kg·m², not
measured — put the car on a bifilar rig), the rotational inertias, the steering
lock and rate, and the brake torque and bias. Each carries its reasoning in
`params.js` and as a tooltip on the home screen, so you can judge whether the
guess is good enough for what you are asking of it.

The two calibrated values are the interesting ones — see the μ note above.

## Adjustable parameters

23 of the parameters on the home screen are live: a slider and a number box
under each row, writing straight into the object the physics already holds, so
a change lands on the next 500 Hz substep with no restart. Changed rows are
highlighted gold, each has a per-row reset, and only the differences from
as-shipped are persisted — so if a default moves later, your overrides still
mean what you meant.

Mass, front weight distribution, CG height, wheelbase, both tracks, yaw
inertia, unsprung mass (front and rear separately), CdA, ClA, front downforce
split, both axle inertias, both roll-centre heights, roll and pitch gradient,
steering lock, ratio and lag, driver eye height, and car vibration.

Two notes on that list. **Unsprung mass is now split front/rear** rather than
one per-corner figure — they are not the same corner. And **car vibration is
camera-only**: it scales how much surface texture comes through the seat, and
touches no physics at all.

**Geometry changes reach the drawn car and the hitbox too.** Wheelbase, weight
distribution and both tracks feed three things that used to be hardcoded: the
wheel hub positions, the cone-strike footprint, and the body mesh itself. The
bodywork is authored once at SDM26's real stations and then stretched — forward
of the CG by a/a₀ and behind it by b/b₀, so moving the wheelbase *or* the weight
distribution moves the nose and tail the way the axles moved. Laterally only the
outboard half scales, because widening the track lengthens the wishbones and
pushes the wheels out; it does not make the tub wider.

That is a rendering approximation rather than a re-body — at large changes the
car is stretched, not redesigned — but it keeps what you see and what knocks
cones over consistent with what you are driving. At 1.85 m wheelbase and 1.40 m
tracks the body goes from 2.42 m to 2.93 m long and the hitbox from
1.25/1.05/0.70 m to 1.41/1.21/0.80 m.

This needed one real fix underneath. The derived quantities — CG-to-axle
distances, sprung mass, nominal tyre load, axle inertias — were computed once in
the constructor, so moving the mass or wheelbase slider would have silently done
nothing. They are now recomputed every substep, which is half a dozen divisions
at 500 Hz.

One result worth knowing about, because it looks like a bug and is not: at
pinned throttle a **heavier car accelerates faster** over 40 m (3.62 s at 340 kg
against 3.85 s at 267 kg). The extra rear load stops it spinning — the light car
reaches slip ratio 4.1, the heavy one only 1.4. Give both a managed launch so
they are traction-limited and the order flips back to what you would expect
(3.53 s against 3.66 s).

## The cockpit

The car is modelled and drawn around the driver: tub, nose, wings, side pods,
roll hoops, dash, steering wheel, and all four wheels with their wishbones. The
geometry is real SDM26 dimensions — front axle at +0.788 m, rear at −0.742 m,
1.207/1.194 m tracks, 0.20 m loaded radius — so what you see out of the cockpit
is where the car's corners actually are.

- **Front wheels steer and spin.** Steer rotates about the kingpin, spin about
  the hub axis *after* the steer, so a steered wheel rolls about its own axis.
  28° of lock gives 112° of steering-wheel rotation (4:1).
- **The steering wheel is the team's own**, laid out from the asset in
  `packages/widgets/src/steering-wheel`: a carbon plate with two kidney
  cut-outs, grips wrapping their outer edge, gold buttons in the top corners,
  amber paddles either side of the Sparky mark, three gold rotaries along the
  bottom. 208 × 148 mm, a 1.42 on-screen aspect against the reference's 1.40,
  and 2.8% of the frame against 4.6% for the round wheel it started as. There
  is deliberately **no display screen** — the real wheel does not have one.
- **Wishbones are static, and that is correct.** An A-arm does not move when you
  steer; only the upright rotates about the kingpin.
- **The rims fade out with speed.** Five spokes at 20 rev/s would strobe into a
  stationary-looking mess at 60 Hz; blending them toward the tyre reads as
  motion blur instead.
- **The cockpit camera is rigidly bolted to the chassis**, so the dash and wheel
  never move relative to your head and the roll you see is the world rolling.
  Chase damps roll and pitch, because a chase camera that rolls with the car is
  unwatchable.

There are no hands on the wheel. At 112° of lock a glove modelled at 3 o'clock
swings round to 10 o'clock, high enough to break the horizon in the middle of
the frame; without modelled arms that reads as a floating black box sitting on
the road.

## Setup changes from the wheel

The d-pad adjusts the car while it is moving — left/right picks the item,
up/down moves it in 0.1 percentage-point steps (held, it repeats and then
speeds up). These write straight into the parameter object the physics already
holds, so a change lands on the next 500 Hz substep. The HUD shows both values
and how far each has drifted from the baseline.

| Item | Range | Effect |
|---|---|---|
| `RSD-F` roll stiffness, front share | 30–70% | up = more understeer |
| `BB-F` brake bias, front | 45–75% | up = more stable on entry |

0.1% is finer than anything you can set on the real car — roll stiffness comes
in bar holes, bias in turns of a bar. That is deliberate: find where the balance
actually moves first, then decide what the nearest real setting is.

Both were checked for direction, not just for changing something:

- **Roll stiffness**, constant-steer test at 9 m/s: radius grows monotonically
  7.18 → 7.28 → 7.37 → 7.45 m as front share goes 40 → 50 → 60 → 70%. More front
  roll stiffness, more understeer, exactly as it should.
- **Brake bias**, pedal-ramp test from 26 m/s: at 48% the rear locks first (71%
  pedal vs 88%), at 62% the front locks first (73% vs 90%), and above 70% the
  rear never locks. Crossover lands around 57%.

## ETC pedal map

`M` (or the menu button) opens an editor for the electronic throttle map —
accelerator pedal position to throttle plate position, with any number of
breakpoints. Drag a point, click the plot to add one, right-click to remove it.
Presets cover linear, progressive, aggressive, wet and endurance. The map
persists in local storage and is applied once per frame, before traction
control, so everything downstream is dealing with plate position rather than
pedal position.

Interpolation is a **monotone cubic Hermite spline** (Fritsch–Carlson), and that
choice is the point of the feature. A natural cubic or Catmull-Rom through the
same breakpoints overshoots between them — meaning somewhere in that span,
pushing the pedal harder *closes* the throttle, and the plate can leave 0–100%
entirely. Monotone cubic is smooth (no kinks at the breakpoints the way straight
linear interpolation has) and provably cannot overshoot. The editor also clamps
a dragged point between its neighbours, so the map is always physically valid.

`node tools/validate.js` checks that guarantee against 4000 randomly generated
maps of 2–11 breakpoints: zero backwards steps, zero out-of-range, zero
overshoot.

## The vehicle model

A transient bicycle model, twelve integrated states, semi-implicit at a fixed
500 Hz regardless of frame rate (`src/vehicle/bicycle.js`).

```
u, v      body-frame longitudinal / lateral velocity   2
r         yaw rate                                     1
wF, wR    front / rear axle speed                      2
aF, aR    relaxation-lagged slip angles                2
delta     road-wheel steer angle                       1
X, Y, psi global pose                                  3
we        crankshaft speed (carried by Powertrain)     1
```

Mechanically that is 3 chassis degrees of freedom (surge, sway, yaw), 2 wheel
rotational DOF, and 1 driveline DOF that exists only while the clutch slips.
Heave, pitch and roll are *not* degrees of freedom — they are applied as deg/g
gradients, so there is no ride model. The home screen lists all of this, plus
every parameter tagged with where it came from.

"Transient" is meant literally, in four places:

1. The lateral equation keeps `m·u·r`, so yaw response has real overshoot
   instead of settling instantly onto a cornering balance.
2. Wheel speeds are states, so slip ratio is dynamic — the rears spin up and
   the fronts lock.
3. Slip angles pass through a relaxation-length lag (0.35 m), so tyre force
   builds over the first third of a metre after a steering input.
4. Load transfer is fed by the previous step's measured accelerations, so it
   settles rather than teleporting.

**Grip still knows about the left/right split.** A bicycle model lumps each
axle into one contact patch, which normally throws away the load-transfer
effect on grip. Here `tire.axleMu` reconstructs the inner/outer loads from the
SDM26 roll-stiffness distribution (elastic + geometric + unsprung) and takes the
load-weighted mean μ across the pair. Because μ falls with load, a transferring
axle always makes less grip than an evenly loaded one — which is the mechanism
that turns the ARB setting into an understeer/oversteer balance. The HUD's
balance bar reads that utilisation difference directly.

The tyre is a shape-fitted Magic Formula with combined slip via Pacejka's
similarity method, so pure-slip limits are exact and the friction ellipse falls
out of the model rather than being pasted on.

### Why the tyre is not the Oracle MF6.1.2 evaluator

Oracle reads a real MF6.1.2 `.tir` and is the right tool for peak-grip lap
times. But that R20 TTC fit peaks at |μ| ≈ 2.32 and does not reach peak Fy until
13–16° of slip. For a driving sim that produces vague, disconnected steering.
Here the peaks are pinned to the numbers Helios validated against real runs, and
the slip at which they arrive is set to what a driver feels through a 10" slick
(8.5° lateral, 0.11 slip ratio, ~291 N/deg per tyre at static load).

### Where the constants come from

Lifted verbatim from Helios `SDM26_VEHICLE` / `SDM26_ROLL`: mass 267 kg, 48.5%
front, CG 284.5 mm, wheelbase 1.53 m, tracks 1.207/1.194, tyre radius 0.20 m,
CdA 1.294 / ClA 3.146 at 55.3% front (2026 CFD aero map), Crr 0.02, driveline
0.85, CBR600RR ratios with 2.111 primary and SDM's 3.0 final, 14 500 rpm limit,
100 ms shift, roll-stiffness distribution 0.512 on a 262.6 mm roll arm.

**One deliberate deviation.** `muLat` is 1.573 here, not the lap sim's 1.368.
Helios pins 1.368 at the skidpad in a quasi-steady model that applies load
sensitivity to the axle as a whole. This model *also* derates for the lateral
transfer within the axle, which costs a further ~6% of μ. Reusing 1.368 would
double-count that and give a 5.38 s skidpad against the 5.02 s SDM26 actually
ran. 1.573 reproduces 5.02 s through *this* model — same measurement, different
model, so a different constant. Oracle hit the same thing and solved it the same
way with `mu_scale`. The original value is kept as `muLatHeliosQss` for
traceability.

Estimates the team has not measured are marked `EST` in `params.js`, each with
its basis: yaw inertia 105 kg·m², unsprung 11 kg/corner, wheel and driveline
inertias, 28° lock, 1500 N·m of brake torque at 62% front, 0.35 m relaxation
length. Those are the numbers to replace first when real data exists.

## Powertrain

Torque comes straight from
`helios-dev/crates/engine-sim/tests/fixtures/sweep_python_v1/sdm26_characteristic_4k_to_15k.csv`
— the SDM26 characteristic-junction sweep, 4000–15 000 rpm in 23 points, from
the CFD module's 1-D FV engine solver. Peak 62.6 N·m at 8000 rpm, 58.1 kW at
11 500 rpm.

That curve is not a smooth dyno arc. It has the wave-action features the solver
predicts — a hole at 6500, the spike at 8000, a second wind at 11 000–11 500
before the restrictor chokes it — and those survive into the driving model, so
gear choice matters the way it does in the car. Engine braking is taken from the
sweep's own `fmep` (T = fmep·Vd/4π), giving ~12 N·m of overrun drag at 10k.

The clutch is modelled properly — locked or slipping against a torque capacity,
with the crank on one side and the wheels on the other — rather than pinning rpm
to road speed. That is what makes launches, bogs, stalls and the ignition-cut
shift behave. Driveline inertia is split at the primary, because that is where
the clutch physically sits on a CBR600RR: the crank sees 17.4:1 in first while
the basket, shafts and sprocket only see 8.25:1. Lumping both at the crank
overstates reflected inertia by about 45%.

## Courses

Traced 2026 FSAE Michigan geometry from the Helios lap sim's `-visual` track
JSONs, resampled to 1 m by `tools/prepare_data.py`:

- **Autocross** — 685 m, single timed run, 192 cones, 3 sectors
- **Endurance** — 2122 m closed circuit, 586 cones, 4 sectors

Cones are placed on both 3.5 m edges with spacing that tightens through corners
the way a real course does (7 m on straights, 3 m in hairpins). Scoring follows
the rules: +2.000 s per cone knocked down or out, and because Off Course is a
DNF on a real run rather than something a game can end on, it is scored +10 s
per excursion and flagged. Penalties reset per lap on endurance.

Cone contact is an exact rectangle-versus-circle test against the chassis
footprint, not sampled points with a slop radius — with only 1.05 m of clearance
each side of a 1.39 m car in a 3.5 m corridor, slop would eat the usable width
of the course.

### Michigan International Speedway

A third entry in the track menu, and a different thing to the two courses: a
**venue** — a bounded place to drive around rather than a run you are timed on.
No cones, no sectors, no penalties.

Built to the published specification where that is defensible: 2.000 mi
perimeter exactly, 73 ft width, banking 18 deg in the turns / 12 front / 5 back
eased across the joins, 7.23 m of rise across the width, a 1.07 m concrete wall
and a 6.4 m catchfence.

The plan outline is **a stadium oval, not MIS's D shape**, and it is labelled as
placeholder in the JSON. A closed loop of two straights and two circular arcs
can only close when the straights are *equal* — the closure equations reduce to
`S1 - S2 = 0` for any radii and any sweep angles — and MIS's published straights
differ by 414 m. Real ovals use compound curves that a perimeter and a banking
angle cannot recover. Rather than invent a D that merely looks right, the
outline is the honest generic. A traced outline would touch only
`tools/make_mis.py`; every consumer reads the sampled arrays.

**You can drive the infield and the flat apron. The banking is scenery.** An
invisible barrier runs along its foot: hit it and the car is clamped back with
its outward velocity removed, so it slides along rather than sticking or
bouncing. No banking physics, which means the validated numbers above are
untouched by any of this.

```bash
python tools/make_mis.py     # -> data/venue-mis.json
python tools/plan_view.py    # -> mis-plan.png, plan + true-angle cross-section
```

## Engine sound

**Generated, not sampled.** There are no recordings in this repository.

Adapted from [`ange-yaghi/engine-sim`](https://github.com/ange-yaghi/engine-sim)
(MIT). The repository usually linked for this — `engine-sim-community-edition` —
ships the built application and contains no source; the algorithms follow the
original open codebase.

```text
crank angle
  -> per-cylinder pressure (single-zone, Wiebe heat release)
  -> flow through the exhaust valve
  -> exhaust waveguide (primaries -> collector -> tailpipe -> open end)
  -> synthesiser (jitter, DC removal, derivative, noise, convolution)
  -> samples
```

It runs in an **AudioWorklet**, on the audio thread. Rendering it on the main
thread would put every garbage collection, layout and WebGL draw between the
engine and the speaker, and the result crackles whenever the frame time moves —
which in a driving game is exactly when the engine is doing something
interesting.

**The torque curve is an input.** The heat release is solved so the modelled
cycle does the work the Helios CFD sweep says the engine is making, so the note
and the acceleration answer to the same number rather than drifting apart.

Because every stage is physical, the things that change an engine's voice in
reality change it here. Cylinder count and crank phasing set the firing pattern.
Primary length sets the resonance. Exhaust gas temperature changes the speed of
sound, so the tuned length of the header shifts with load and the note moves on
the overrun. None of that is scripted.

The same model exists as the `engine-audio` Rust crate in `fsae-sim-rs`. They
are kept in lockstep the way `sim-core` is — the Rust side emits golden vectors
and `tools/validate.js` checks this build against them, currently agreeing to
2e-3 over 512 samples, which is f32-against-f64 rounding and nothing else.

The old oscillator bank is still there as a fallback for browsers without
AudioWorklet. It sounds recognisably like the same engine and obviously
synthetic next to the real thing.

## Control devices

Four profiles: **keyboard & mouse**, **Xbox**, **PlayStation**, **wheel &
pedals**. Pick one on the home screen; a connected device selects its own unless
you have chosen by hand. Settings persist as a diff from the shipped defaults.

They are not variations on a theme — they differ in what the driver can
physically command, so each gets its own steering dynamics:

| | max steering speed | acceleration | deadzone | curve |
|---|---|---|---|---|
| keyboard | 180 °/s | 700 °/s² | 0 | linear |
| gamepad | 300 °/s | 2200 °/s² | 0.10 | 1.7 expo |
| wheel | 720 °/s | 12000 °/s² | **0** | **1.00** |

All at the road wheel. Divide by the 4.0 steering ratio for rim figures.

**Steering is a rate- and acceleration-limited servo**, and both limits are
adjustable per device. The acceleration limit matters most on a keyboard, where
every input is a step: without a bound on how fast the steering *speed* can
change, a step in position becomes a step in velocity, which no hand and no
steering motor can produce.

### Wheels

The mapping from rim angle to road wheel is the setting that matters most:

- **Match the car** — the rim turns through SDM26's real 4.0 ratio, so its 28°
  of lock is **112° at the rim, lock to lock**. Set your wheel's driver software
  to 112° and hand position *is* front-wheel angle. Leaving a 900° wheel at 900
  makes this mapping use only the first 12% of its travel: correct, and it feels
  wrong, because the wheel is configured wrong.
- **Scale to lock** — whatever rotation the wheel is set to becomes full lock.
  Nothing to reconfigure, but the ratio is then a fiction and the steering is
  eight times slower than the real car's.

Deadzone 0 and curve 1.00 are correct on a wheel and not defaults to tune away:
the device measures hand position directly, so smoothing it discards real
information.

Pedals calibrate against the travel your set actually produces — a G29 brake
rests near −1 and tops out near +1, a load cell may never reach +1 at any force
a person can apply. The panel shows a **live axis monitor**, because wheels do
not use the Gamepad API's standard mapping and every vendor assigns axes
differently, so watching which number moves is the only reliable way to find a
pedal.

**Force feedback is not implemented.** The settings are declared and documented
so the shape does not have to change later, but nothing drives them. It needs
the tyre model to return self-aligning torque (it returns lateral force only)
and a kingpin/caster geometry block in the parameters — otherwise the force
would be invented rather than derived, which is the one thing worth not doing.

## Vehicles as data

A vehicle is a JSON definition naming which model to use for each subsystem —
tyre, powertrain, suspension, aero, engine sound — plus its parameters. That is
what makes it savable, exportable and readable by both builds.

Duplicate a vehicle, edit it, swap a subsystem, export it, import it. Saved
vehicles store a **sparse diff against their ancestor**, so improving a shipped
default still reaches everything derived from it instead of each copy being
pinned to whatever shipped the day it was made.

Only the SDM26 is defined. The framework is the deliverable; inventing a
plausible-looking second car would put numbers in the repository that nobody has
measured. Suspension and aero are registered but still embedded in the solver,
and the picker says so rather than offering a choice that silently does nothing.


## Validation

`node tools/validate.js` runs the same model headless against the three events
the team has real numbers for.

| Check | Result | Reference |
|---|---|---|
| Skidpad lap, 9.125 m radius | **4.96 s** | SDM26 ran 5.02 s |
| Skidpad lateral | 1.49 g | above μ because 11.5 m/s is worth ~250 N of downforce |
| 75 m accel, managed launch | 4.76 s | QSS says 4.2 s — see below |
| 75 m accel, throttle pinned | 5.19 s | +0.43 s lost to wheelspin |
| Braking from 25 m/s | 23.1 m, 1.74 g peak | — |
| Cornering stiffness | 291 N/deg per tyre | 10" slick at 655 N |
| ETC map, 4000 random curves | 0 overshoot, 0 backwards steps | monotone guarantee |
| Roll stiffness 40→70% front | radius 7.18 → 7.45 m | monotonic understeer |
| Brake bias sweep 48→75% | rear-locks-first → front-locks-first | crossover ~57% |

The skidpad is the anchor: it is a clean μ measurement, low speed, no gearing,
no line freedom.

The 75 m time is honestly slower than the lap sim's 4.2 s and the test band says
so. This model carries driveline rotational inertia — about +94 kg apparent in
first — that a quasi-steady lap sim ignores entirely. If that check ever comes
back at 4.2 s, something has stopped modelling the inertia.

Also verified in-browser: lap detection is exact on both courses (endurance
141.53 s against 141.49 s expected for a constant 15 m/s walk of the centreline;
autocross finishes exactly once), no console errors, and the frame costs about
1.0 ms of update plus render.

`node tools/smoke_desktop.mjs` checks the built executable, which is a different
question from "does the code work" — it launches the exe, attaches to WebView2
over the DevTools protocol, and asks the running page whether the game actually
booted. A process that stays alive and a blank window look identical from the
outside, so the test confirms the embedded track and engine data loaded and
WebGL 2.0 came up clean rather than just that nothing crashed.

## Known limitations

- **One curvature spike below the car's turning circle.** The traced endurance
  centreline has a single point at s ≈ 607 m with a 2.83 m radius, against the
  SDM26's 2.88 m kinematic minimum at full lock. It is one isolated point — a
  tracing artifact, not a real hairpin — and you drive through it without
  noticing, but the geometry is worth cleaning up at the source.
- **Bicycle model, not four-corner.** Grip responds to lateral load transfer,
  but there is no individual wheel state, no differential, and no per-corner
  camber or toe. Setup work belongs in Helios Setup and Oracle; this is a
  driving model.
- **No tyre thermal or wear model.** μ is constant over a run.
- **Flat ground.** The venue is a lot, so this costs less than it would
  elsewhere, but there is no surface elevation or grip variation.
- **Suspension is a gradient, not a state.** Roll and pitch come from the
  validated deg/g gradients for camera and load transfer; there is no ride
  model, so kerb strikes and heave dynamics are not simulated.

## Layout

```
data/           generated: course geometry + the CFD torque curve
dist/           generated by build.rs: what gets embedded in the exe
tools/
  prepare_data.py   rebuilds data/ from the Helios sources
  serve.py          no-cache dev server for browser iteration
  make_icons.py     regenerates the app icons
  make_mis.py       generates the Michigan International Speedway venue
  plan_view.py      draws the venue plan + cross-section for review
  validate.js       headless physics + ETC-map checks
  smoke_desktop.mjs launches the built exe and interrogates it over DevTools
src/
  vehicle/      params, paramMeta (provenance), tyre, powertrain, bicycle,
                ETC map, live setup adjustments, modules + library
  audio/        the engine model, and the worklet that runs it
  track/        course geometry, progress, cone strikes; venue.js for MIS
  render/       WebGL2 renderer, procedural SDM26 car geometry, venue mesh
  game/         input, control profiles + panel, HUD, timing, audio,
                ETC editor, spec sheet, desktop shell
  main.js       bootstrap and loop
src-tauri/
  build.rs      stages the frontend into dist/ on every cargo build
  src/main.rs   the native window; all logic lives in the frontend
  tauri.conf.json
```

`window.__sim` is exposed for poking at the model live —
`__sim.car.telemetry`, `__sim.powertrain.wotTorque(9000)`, `__sim.etc.describe()`.
In the desktop app, open devtools with `cargo run` (a debug build enables them).

Regenerate the data (needs `helios-dev/` alongside this folder):

```bash
python fsae-sim/tools/prepare_data.py
```
