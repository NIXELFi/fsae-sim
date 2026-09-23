# SDM26 Driver-in-Loop

A first-person FSAE driving simulator for Sun Devil Motorsports' SDM26, on the
traced 2026 FSAE Michigan autocross and endurance courses, driven with an Xbox
One controller.

Everything that could come from real team data does. The car is the Helios lap
sim's SDM26. The engine is the Helios CFD module's 1-D FV engine sweep. The
courses are the same traced geometry the Oracle lap sim times.

## Running it

It is a desktop app -- a Tauri v2 window with the whole game embedded in the
executable. No server, no install, nothing to keep running in a terminal.

```bash
cargo build --release --manifest-path sim/src-tauri/Cargo.toml
```

That produces `src-tauri/target/release/fsae-sim.exe`. Double-click it.

You need the Rust toolchain and VS Build Tools, which are already set up on this
machine for Helios. You do **not** need Node, pnpm, or the Tauri CLI: `build.rs`
stages the frontend into `dist/` on every build, so plain `cargo build` can
never ship a stale bundle. `cargo tauri build` additionally produces an NSIS
installer if you want one.

The frontend is still plain ES modules and WebGL2 with zero dependencies, so it
also runs in a browser -- and that is the loop to use while working on the game:

```bash
python sim/tools/serve.py
```

The reason matters. `generate_context!` embeds the frontend into the binary at
compile time, so editing any `.js` file forces the crate to recompile and
relink. Reload a browser tab instead and the change is instant; build the exe
when you want to hand it to someone.

Use `tools/serve.py`, not `python -m http.server`. The stdlib server sends no
`Cache-Control`, so the browser caches the ES modules and a reload will serve a
fresh `index.html` alongside a stale `main.js` -- the page looks updated while
the behaviour is yesterday's. `serve.py` is the same static server with caching
switched off.

Before you hand anything to anyone:

```bash
cd sim && npm test          # shaders, bindings, respawn, replay, delta, physics
npm run smoke               # launches the built exe and interrogates it
```

`npm test` needs nothing installed -- every one of those is a plain Node script
against the same modules the game loads.

## Controls

| Xbox One | Keyboard | |
|---|---|---|
| Right trigger | `W` / `up` | Throttle (analog on the pad) |
| Left trigger | `S` / `down` | Brake (analog on the pad) |
| Left stick | `A` `D` / `<-` `->` | Steering |
| RB / LB | `E` / `Q` | Upshift / downshift |
| B | `R` | Recover to the course |
| X / Y | `T` / `C` | Traction control / camera |
| View / Menu | `Backspace` / `Esc` | Restart run / pause |
| D-pad < > | `[` `]` | Pick the setup item to adjust |
| D-pad ^ v | `-` `=` | Adjust it, one step a press |
| -- | `Num 8` / `Num 2` | Brake bias up / down (bindable to any button) |
| -- | `Num 0` | Every setup value back to this session's baseline |
| -- | `Num 5` | Switch between setup A and setup B |
| -- | `Num 9` / `Num 3` | Front roll stiffness up / down (bindable to any button) |
| L3 | `H` | Back to the home screen |
| -- | `U` / `J` | Overlay density / which dash |
| -- | `M` | Throttle-map editor |
| -- | `F11` | Fullscreen |

**Everything in the table above is rebindable** -- not `F11`, and not the
walkaround camera's own nudges, which are the keyboard's copy of what the mouse
already does in that view. Home screen -> Controls -> Bindings: click the cell
for a control and press the key, button or paddle you want it to be. The same
table drives what `Input` reads, so there is no such thing as a binding the
panel offers and the game ignores. Binding something that is already taken
moves it, and says so. Right-click a cell to clear it.

Axes are found the same way: under **Axes and calibration**, click a pedal's
row and sweep it end to end. The axis that moved is the one it binds to, and
the travel it saw is the calibration -- one gesture for both, because wheels
and pedal sets do not use a standard mapping and there is no table that is
right for every device.

Autocross ends at the finish line, and the end-of-run card is what happens
there: the raw time, the cones and off-courses it cost, and the score. From it
you can run it again (`Enter`), watch the replay (`W`), keep driving (`Esc`),
go to the home screen (`H`) or quit (`Q`). It comes up two seconds after the
line, so the roll-out and the time on the dash are still yours to see.
Endurance and the venue never end on their own, so the pause card has **End
session** for them: the lap table, the best, the theoretical best and the
sectors, on the same card.

The pause card also carries the things a driver stops to change -- camera,
overlay density, which dash, traction control, master volume -- so changing
one no longer means going Home, which ends the recording.

**Everything on screen answers to the pad.** The d-pad (or a wheel's hat)
moves, `A` selects, `B` backs out, the shift paddles switch tabs and `Menu`
starts the engine, on the home screen, on both cards and in the replay
(`A` plays, left/right step a second, up/down change speed, paddles jump
five seconds, `B` closes). A driver on the rig should never have to reach for
a mouse. The Controls tab prints the key reference from the live bindings
table, so it is never out of date with a rebinding.

The **Runs** tab lists the archive for the current course (or every course).
**Replay** watches a run; **Chase** puts its best lap on the live delta,
switching course first if it was set elsewhere; and while watching a replay,
the Ghost picker in the session panel puts any other run on that course in
the scene beside it. The session card says what the delta is chasing and the
archive best on the course. A replay plays the engine from the log.

Steering uses a 10% deadzone and a 1.7-power response curve. That curve is not
a feel preference: full lock is 46 deg and the tyre peaks at 7.3 deg of slip, so
a linear stick would compress everything that matters into the first sixth of
the travel.

Four cameras: cockpit (driver's eye, 0.66 m off the deck), nose, chase, and a
free walkaround (drag to turn and raise, wheel to zoom; `,` `.` `G` `F` `'` `;`
on the keyboard). The driving views all
of them carry the HUD, including live APS / TPS / brake bars -- APS against TPS
is worth watching, because on anything but a linear pedal map the gap between
those two bars *is* the map.

## What the model is standing on

The home screen lists the model, its degrees of freedom, and **every parameter
tagged with where it came from**. That distinction is the point: 61 parameters
run this simulator, and they are not equally trustworthy.

| Tag | Meaning | Count |
|---|---|---|
| `TEAM` | Measured, specified or validated by SDM: Helios, the team's Drive (sim-parameters workbook, OptimumK export, brakes calculator, ride/roll sheet) and the raw TTC tyre data | 39 |
| `CFD` | Output of a team CFD model (2026 ride-height aero map, engine sweep) | 5 |
| `CAL` | Fitted so **this** model reproduces a measured SDM26 result | 2 |
| `EST` | An engineering estimate I generated. Nobody has measured it | 15 |

So 46 of 61 trace back to the team, and **15 (25%) are estimates**. The
2026-09-11 data pass (`sim/tools/team_data.py`, `sim/tools/ttc_trail.py`)
replaced the yaw inertia, unsprung masses, wheel inertias, steering ratio,
caster and trail, brake bias, tyre load sensitivity, pneumatic trail and
pitch gradient with the team's own numbers, and moved the aero map to the
2026 full-car sweep. What is still estimated: `frontGripFactor` (the one
balance knob, waiting on a measured understeer gradient), the driveline
inertias, the steering lock (consistent with the MoTeC steering channel but
not a rack measurement), the steering lag and rate, the pedal force behind
the max brake torque, the tyre's peak slip angle and relaxation length, and
the camera-only eye point, heave and vibration. Each carries its reasoning
in `params.js` and as a tooltip on the home screen.

The two calibrated values are the interesting ones -- see the mu note above.

## Adjustable parameters

24 of the parameters on the home screen are live: a slider and a number box
under each row, writing straight into the object the physics already holds, so
a change lands on the next 500 Hz substep with no restart. Changed rows are
highlighted gold, each has a per-row reset, and only the differences from
as-shipped are persisted -- so if a default moves later, your overrides still
mean what you meant.

Mass, front weight distribution, CG height, wheelbase, both tracks, yaw
inertia, unsprung mass (front and rear separately), CdA, ClA, front downforce
split, both axle inertias, both roll-centre heights, roll and pitch gradient,
steering lock, ratio and lag, driver eye height, and car vibration.

Two notes on that list. **Unsprung mass is now split front/rear** rather than
one per-corner figure -- they are not the same corner. And **car vibration is
camera-only**: it scales how much surface texture comes through the seat, and
touches no physics at all.

**Geometry changes reach the drawn car and the hitbox too.** Wheelbase, weight
distribution and both tracks feed three things that used to be hardcoded: the
wheel hub positions, the cone-strike footprint, and the body mesh itself. The
bodywork is authored once at SDM26's real stations and then stretched -- forward
of the CG by a/a0 and behind it by b/b0, so moving the wheelbase *or* the weight
distribution moves the nose and tail the way the axles moved. Laterally only the
outboard half scales, because widening the track lengthens the wishbones and
pushes the wheels out; it does not make the tub wider.

That is a rendering approximation rather than a re-body -- at large changes the
car is stretched, not redesigned -- but it keeps what you see and what knocks
cones over consistent with what you are driving. At 1.85 m wheelbase and 1.40 m
tracks the body goes from 2.42 m to 2.93 m long and the hitbox from
1.25/1.05/0.70 m to 1.41/1.21/0.80 m.

This needed one real fix underneath. The derived quantities -- CG-to-axle
distances, sprung mass, nominal tyre load, axle inertias -- were computed once in
the constructor, so moving the mass or wheelbase slider would have silently done
nothing. They are now recomputed every substep, which is half a dozen divisions
at 500 Hz.

One result worth knowing about, because it looks like a bug and is not: at
pinned throttle a **heavier car accelerates faster** over 40 m (3.62 s at 340 kg
against 3.85 s at 267 kg). The extra rear load stops it spinning -- the light car
reaches slip ratio 4.1, the heavy one only 1.4. Give both a managed launch so
they are traction-limited and the order flips back to what you would expect
(3.53 s against 3.66 s).

### Setup files

A setup can leave the machine. The **Setup** toolbar at the top of the Vehicle
model tab exports the whole parameter set as a `.hset` file (Helios setup):
JSON text with a name, author, date, course, notes, the simulator version, and
a `values` block holding every parameter on the sheet (which includes
everything the driver can move from the wheel), under the same dotted paths a
run manifest records its `setup` in.

It is a **full snapshot, not a diff**. A file means the same thing whatever the
as-shipped defaults are on the machine that opens it, and a run is only
interpretable next to every number that was in force -- not just the ones
somebody remembered to change.

Four ways in: **Import setup...** on the same toolbar, **drag a `.hset` onto
the window** (any tab, browser or desktop), **double-click the file** (the
desktop installer associates `.hset` with the app; a running app is re-used
rather than started twice), or `fsae-sim --setup <file>` from a launcher. All
of them open a summary card -- name, author, notes, and the list of parameters
that differ from as-shipped with values and units -- and nothing is written
into the car until you press **Apply**. Applied values persist the way a slider
change does and reach the native car immediately. A file for another car, a
parameter this build does not have, or a value outside the sheet's range is
warned about on the card (unknown parameters are dropped, out-of-range values
clamped) rather than refused; only a file that is not a Helios setup at all, or
one from a newer simulator, is.

On the desktop an export lands in `sim-setups` beside the `sim-runs` folder in
the Helios data directory; in a browser it downloads.

## The cockpit

The car is modelled and drawn around the driver: tub, nose, wings, side pods,
roll hoops, dash, steering wheel, and all four wheels with their wishbones. The
geometry is real SDM26 dimensions -- front axle at +0.788 m, rear at -0.742 m,
1.207/1.194 m tracks, 0.20 m loaded radius -- so what you see out of the cockpit
is where the car's corners actually are.

- **Front wheels steer and spin.** Steer rotates about the kingpin, spin about
  the hub axis *after* the steer, so a steered wheel rolls about its own axis.
  The rack is progressive and was measured: 46 deg of lock, and 179 deg of rim
  each way to reach it, so 358 lock to lock.
- **The steering wheel is the team's own**, laid out from the asset in
  `packages/widgets/src/steering-wheel`: a carbon plate with two kidney
  cut-outs, grips wrapping their outer edge, gold buttons in the top corners,
  amber paddles either side of the Sparky mark, three gold rotaries along the
  bottom. 208 x 148 mm, a 1.42 on-screen aspect against the reference's 1.40,
  and 2.8% of the frame against 4.6% for the round wheel it started as. There
  is deliberately **no display screen** -- the real wheel does not have one.
- **Wishbones are static, and that is correct.** An A-arm does not move when you
  steer; only the upright rotates about the kingpin.
- **The rims fade out with speed.** Five spokes at 20 rev/s would strobe into a
  stationary-looking mess at 60 Hz; blending them toward the tyre reads as
  motion blur instead.
- **The cockpit camera is rigidly bolted to the chassis**, so the dash and wheel
  never move relative to your head and the roll you see is the world rolling.
  The driver's head is not: it leans outboard, slides under braking and leads
  into the corner, all from lateral and longitudinal g. Chase damps roll and
  pitch, because a chase camera that rolls with the car is unwatchable, and
  has its own heading -- a damped follower on the car's, pulled toward the
  velocity vector -- so the car yaws inside the frame under oversteer.
- **The body rolls and pitches about the CG, on its springs.** The wheels hang
  off an unsprung frame that only translates and yaws, so they stay on the
  road and the travel between wheel and arch is visible from outside.
- **A struck cone tumbles away from the car** over a third of a second,
  about the base edge on the far side, and you feel it through the camera.
- **Tyres leave rubber.** A wheel past about 92% of its grip, locked or
  spinning lays a mark on the surface that fades with distance; a replay
  lays them from the log; a new course starts clean.
- **The replay ghost is translucent** and always drawn, so a lap that runs
  within a car's length of yours still shows through. The sky is drawn before
  it, so a ghost against the sky is no longer erased by it.
- **Colours are authored in display space** (`carmesh.js` says so at the top):
  the shaders decode them to linear before lighting. A slick is 0.175 on that
  scale, not the 0.105 it was, which had been decoding to a black hole; carbon,
  grips and the dash case were lifted the same way.
- **Curved surfaces are smooth-shaded.** Tyres, rims, hoops, grips and the tub
  carry analytic normals; boxes, plates and wings keep their hard edges.
- **The wheel, dash and rims cast shadows** into the cockpit from the near
  shadow cascade; the shadow bias was 45 mm along the sun, which floated every
  cone, and is now 7 mm.
- **The asphalt does not sparkle.** Every noise octave fades by its own
  screen footprint rather than by distance, so nothing is sampled past
  Nyquist; the hash is an integer mix that stays stable a kilometre out at
  MIS; the stall lines are one-way bays with drive aisles; and the surface
  has a gentle relief normal so it reads as a pour, not a plastic sheet.

- **There is a driver in the car -- from outside.** Helmet in the team
  colours with a dark visor, HANS, shoulders and belts in the suit, gloves
  on the wheel at 9 and 3, and forearms and upper arms posed every frame by
  a two-bone solve from the fixed shoulders to the gloves, so at any rim
  angle the arm runs from the glove to an elbow inside the tub and up to
  the shoulder. None of him is drawn from the cockpit camera: the real
  driver's hands are on the real rim, and a second pair over the dash read
  as wrong rather than as presence. The replay ghost carries its own driver.
- **The lot has a venue around it.** Conifers and broadleaf trees in clumps,
  a stepped grandstand with rows, aisles and a roof on posts, garages,
  a two-storey office and a gabled shed with doors and window reveals, and a
  jagged treeline and low hills out in the haze so the horizon is never a
  straight line. All procedural, one draw, ~90-110k triangles; the ground
  runs out to 3 km and is fully fogged there, so it has no edge.

### Frame pacing

The renderer takes about 1.5 ms of GPU time and under 1.5 ms of JavaScript
per 144 Hz frame on an RTX 4070 laptop, which turned out to be the cause of
the stutter rather than the cure for it. A GeForce that is 15-20 % busy
drops to its lowest clocks (P4, 800-1000 MHz on that machine) and has to
ramp for every frame; about one frame in forty then arrives 7-25 ms late.
A page that only clears its canvas, loaded into the simulator's own window,
hitched at exactly the same rate, so it is not something in the renderer.

**GPU clock hold** (`render/gpuHold.js`, on the pause card, on by default on
the desktop build) pads each frame with a fragment-heavy pass into a small
offscreen target so that the whole frame lands on about 3 ms of GPU time,
measured with a timer query and adjusted every dozen frames. As the real
rendering gets heavier the padding shrinks to nothing on its own. Measured
on the same machine it holds the card at 2300 MHz in P0 and cuts the late
frames by two thirds; the rest are the compositor's. The same effect is
available from the NVIDIA control panel by setting *Power management mode*
to *Prefer maximum performance* for `msedgewebview2.exe`, and a driver newer
than the 2023 one that machine was running is worth having either way.

The other per-frame costs that were found and removed in the same pass: the
shadow pass looked up the cones near the car through 2401 string keys every
frame (55 % of all allocation), the minimap redrew the whole course and every
cone every frame (25 %), the controls panel ran two animation loops into
hidden DOM while driving (a forced layout per frame), the dash texture was
reallocated at 30 Hz with no mipmaps, and the rig thread re-enumerated
DirectInput on its own 1 kHz loop every two seconds whenever no wheel was
attached (a 600 ms stall). Allocation while driving is down from 22 MB/s to
3.5 MB/s. The drawn pose of the native car is also dead-reckoned forward by
the snapshot's age using its own body velocities, so a 1 kHz snapshot taken
at an arbitrary phase no longer judders at 144 Hz.

## Sound

The engine is a physical exhaust model (see `NOTICE`), not a sample. What
sits around it: the cockpit and nose cameras hear the tub's close
reflections, the wind on the helmet and the road through the seat; the
chase and walkaround cameras hear a wider space with little wind. The rev
limiter is an ignition cut that stutters at 24 Hz, a shift is the dogs
engaging with the blip's chuff on the way down, the overrun pops, and a
locked wheel and gravel each have a voice. Green, the sectors, a lap and the
flag beep. Levels, including a channel for the timing cues, are on the Audio
tab.

There are no hands on the wheel. At 179 deg of lock a glove modelled at 3 o'clock
swings round to 10 o'clock, high enough to break the horizon in the middle of
the frame; without modelled arms that reads as a floating black box sitting on
the road.

## Setup changes from the wheel

The d-pad adjusts the car while it is moving -- left/right picks the item,
up/down moves it one step (held, it repeats and then goes five steps at a
time). Every item also has its own **up / down pair** in Settings -> Controls
-> Setup, so brake bias or roll stiffness can sit on two rim buttons and skip
the menu; brake bias ships on `Num 8` / `Num 2` and roll stiffness on
`Num 9` / `Num 3`, the rest unbound. These write straight into the parameter
object the physics already holds, so a change lands on the next 500 Hz
substep. The HUD shows every value and how far each has drifted from where the
session started.

A change sticks: it is remembered between runs and between launches like a
slider change, and the Vehicle model sheet shows it (every item is a slider
there too, and in a `.hset`). **Reset all parameters** puts them back.

| Item | Range | Step | Effect |
|---|---|---|---|
| `RSD-F` roll stiffness, front share | 30-70% | 0.1% | up = more understeer |
| `BB-F` brake bias, front | 45-75% | 0.1% | up = more stable on entry |
| `PRELD` diff preload | 0-75 N.m | 5 | up = steadier entry, less rotation |
| `LOCK-OFF` diff lock, off throttle | 0-0.95 | 0.01 | up = steadier on a lift (50 deg ramp = 0.42) |
| `LC` launch control | 4000-12000 rpm | 100 | up = more wheelspin off the line |
| `FINAL` final drive | 2.50-4.00 | 0.05 | up = shorter gearing |

### The setup card, A / B, and back to baseline

The same values are on four surfaces, and all four follow each other: the
wheel buttons, the **Car tab's run-to-run block**, the full **Vehicle model**
sheet, and a **setup card** that is the point of this section.

The card appears twice. On the launch screen it sits beside **Start engine**,
because the second before a run starts is when "what is this car on?" is worth
asking. And in the car it comes up **the moment a run is armed and clears when
the flag goes green** -- which is the only place it appears for anyone
launching straight into a run (`--autostart`, or Helios starting the rig), who
never sees the launch screen at all. Both are mouse *and* keyboard: every row
is a number you can type into or arrow-key, with a minus and a plus either
side. While the staging card is up the pointer stays free, so mouse steering
does not swallow the clicks, and the keyboard goes back to the car the moment
it clears.

Three buttons on it, and the last two are also bindable:

- **Save / Load A and B.** Two whole setups, kept between sessions. A test day
  is back-to-back runs on two setups until one of them is clearly quicker, and
  `Num 5` (or a rim button) flips between them without a menu. The chip at the
  top says which one the car is on, and it says `unsaved` the moment anything
  moves -- because once brake bias has been nudged, the car is not slot A any
  more and a switch that pretended otherwise would throw the nudge away.
- **Baseline** (`Num 0`) puts every setup value back to where the session
  started. Ten minutes of fiddling used to be undone by reading the HUD's
  deltas and reversing each one by hand.

**Aero balance is not here at all**, and not on the setup card either. It
reads like a setup knob, and on a car with adjustable flaps it would be one --
but nothing on SDM26 changes the front downforce share between two runs. It is
what the wings *are*, so it sits on the **Vehicle model** sheet with the rest
of the car's description, and moving it stops a lap counting as a time (below).

The diff's two ramps are named for what they do rather than for the ramp:
**lock on throttle** is the drive ramp, **lock off throttle** the coast ramp --
the one that steadies the rear on a lift.

0.1% is finer than anything you can set on the real car -- roll stiffness comes
in bar holes, bias in turns of a bar. That is deliberate: find where the balance
actually moves first, then decide what the nearest real setting is.

Both were checked for direction, not just for changing something:

- **Roll stiffness**, constant-steer test at 9 m/s: radius grows monotonically
  7.18 -> 7.28 -> 7.37 -> 7.45 m as front share goes 40 -> 50 -> 60 -> 70%. More front
  roll stiffness, more understeer, exactly as it should.
- **Brake bias**, pedal-ramp test from 26 m/s: at 48% the rear locks first (71%
  pedal vs 88%), at 62% the front locks first (73% vs 90%), and above 70% the
  rear never locks. Crossover lands around 57%.

## ETC pedal map

`M` (or the menu button) opens an editor for the electronic throttle map -- 
accelerator pedal position to throttle plate position, with any number of
breakpoints. Drag a point, click the plot to add one, right-click to remove it.
Presets cover linear, progressive, aggressive, wet and endurance. The map
persists in local storage and is applied once per frame, before traction
control, so everything downstream is dealing with plate position rather than
pedal position.

Interpolation is a **monotone cubic Hermite spline** (Fritsch-Carlson), and that
choice is the point of the feature. A natural cubic or Catmull-Rom through the
same breakpoints overshoots between them -- meaning somewhere in that span,
pushing the pedal harder *closes* the throttle, and the plate can leave 0-100%
entirely. Monotone cubic is smooth (no kinks at the breakpoints the way straight
linear interpolation has) and provably cannot overshoot. The editor also clamps
a dragged point between its neighbours, so the map is always physically valid.

`node tools/validate.js` checks that guarantee against 4000 randomly generated
maps of 2-11 breakpoints: zero backwards steps, zero out-of-range, zero
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
Heave, pitch and roll are *not* degrees of freedom -- they are applied as deg/g
gradients, so there is no ride model. The home screen lists all of this, plus
every parameter tagged with where it came from.

"Transient" is meant literally, in four places:

1. The lateral equation keeps `m.u.r`, so yaw response has real overshoot
   instead of settling instantly onto a cornering balance.
2. Wheel speeds are states, so slip ratio is dynamic -- the rears spin up and
   the fronts lock.
3. Slip angles pass through a relaxation-length lag (0.35 m), so tyre force
   builds over the first third of a metre after a steering input.
4. Load transfer is fed by the previous step's measured accelerations, so it
   settles rather than teleporting.

**Grip still knows about the left/right split.** A bicycle model lumps each
axle into one contact patch, which normally throws away the load-transfer
effect on grip. Here `tire.axleMu` reconstructs the inner/outer loads from the
SDM26 roll-stiffness distribution (elastic + geometric + unsprung) and takes the
load-weighted mean mu across the pair. Because mu falls with load, a transferring
axle always makes less grip than an evenly loaded one -- which is the mechanism
that turns the ARB setting into an understeer/oversteer balance. The HUD's
balance bar reads that utilisation difference directly.

The tyre is a shape-fitted Magic Formula with combined slip via Pacejka's
similarity method, so pure-slip limits are exact and the friction ellipse falls
out of the model rather than being pasted on.

### Why the tyre is not the Oracle MF6.1.2 evaluator

Oracle reads a real MF6.1.2 `.tir` and is the right tool for peak-grip lap
times. But that R20 TTC fit peaks at |mu| ~ 2.32 and does not reach peak Fy until
13-16 deg of slip. For a driving sim that produces vague, disconnected steering.
Here the peaks are pinned to the numbers Helios validated against real runs, and
the slip at which they arrive is set to what a driver feels through a 10" slick
(8.5 deg lateral, 0.11 slip ratio, ~307 N/deg per tyre at static load).

### Where the constants come from

Lifted verbatim from Helios `SDM26_VEHICLE` / `SDM26_ROLL`: mass 267 kg, 48.5%
front, CG 284.5 mm, wheelbase 1.53 m, tracks 1.207/1.194, tyre radius 0.20 m
(the TTC loaded radius at 12 psi and 667 N is 196.5 mm), Crr 0.02, driveline
0.85, CBR600RR ratios with 2.111 primary and SDM's 3.0 final, 14 500 rpm limit,
100 ms shift, and a 262.6 mm roll arm. Roll-stiffness distribution is the
team's 0.48 setup, between the measured 1-1/1-1 (0.46) and 4-7/1-1 (0.51)
blade settings, rather than Helios's 0.512.

Aero is the 2026 full-car CFD ride-height map from the team's Drive ('Ride
Height Data (BW)'): CdA 1.267 / ClA 3.132 at 52.4% front at nominal ride
height. The Cl 2.918 / Cd 1.200 / 55.3% Helios carries is the 2025 half-car
sheet. From the team's Drive as well, via the AC mod's transcription
(`sdm26-assetto-corsa/data/sdm26_team_data.json`) and `sim/tools/team_data.py`:
Izz 93.7 kg.m^2, unsprung 7.56 / 7.77 kg per corner, wheel inertia 0.154 /
0.152 kg.m^2 ('SDM26 Full-Vehicle Sim Parameters'); steering ratio 4.411,
caster 4.743 deg, 18.85 mm mechanical trail ('SDM26 Designed vs Actual
Kinematics', OptimumK); the 70 bar system limit and a 0.72 brake torque share
at the 54% bias bar ('SDM26 Brakes Calculator'; the sim runs the team's 0.65
setup); tyre load sensitivity 0.12 (the team's PAC2002
TTC fit); pitch gradient 0.89 deg/g ('SDM26 Ride Roll Calc'). Pneumatic trail
is fitted straight to the raw TTC Round 9 Mz data for the R20 by
`sim/tools/ttc_trail.py` (see `docs/tyre-models-review.md`).

**One deliberate deviation.** `muLat` is 1.72 here, not the lap sim's 1.368.
Helios pins 1.368 at the skidpad in a quasi-steady model that applies load
sensitivity to the axle as a whole. This model *also* derates for the lateral
transfer within the axle, which costs a further ~6% of mu. Reusing 1.368 would
double-count that and give a 5.38 s skidpad against the 5.02 s SDM26 actually
ran. Same measurement, different model, so a different constant. Oracle hit the
same thing and solved it the same way with `mu_scale`. The original value is
kept as `muLatHeliosQss` for traceability.

**And one estimate the balance needs: `frontGripFactor` 0.88.** `muLat` is the
rear axle's peak; the front runs at 0.88 of it (1.51), and the pair is pinned
so the skidpad still comes out at ~5.03 s. With one tyre character on both
axles the limit balance is set only by load transfer and the aero split, which
left this car neutral to within 1% of force: the rear reached its peak first
at 10, 15 and 20 m/s, and a 12 deg steering step at 15 m/s spun it every time,
because once both axles are past the peak the yaw moment `a.FyF - b.FyR` stays
positive on a 48.5% front car and nothing arrests the yaw. Roll stiffness
alone cannot fix that. A real front lets go first, through things a bicycle
model cannot see -- camber loss on the steered upright, inside-front drag at
parallel steer, steering compliance. 0.88 puts the front at its peak with the
rear holding 17-36% of its force in hand from 10 to 28 m/s: a mild,
recoverable push. It was 0.90 while the aero split was held at 50%; with the
2026 map's 52.4% front the rear limited at 28 m/s and a held keyboard lock at
20 m/s spun the car, so the estimate moved and the data did not. Replace it
with a measured understeer gradient (on the team's 2026-04-08 test plan; no
result on Drive).

Estimates the team has not measured are marked `EST` in `params.js`, each with
its basis: driveline inertias, the 206 lbf pedal force behind the 1235 N.m max
brake torque, and 0.35 m relaxation length. (Steering lock and the peak slip
angle used to be on this list at 28 deg and 8.5 deg; both have since been
measured -- 46 deg through the progressive rack, and 7.3 deg from the team's
own tyre data.) Those are the numbers to
replace first when real data exists.

### The differential

The rear axle is two wheels with a **Drexler Formula Student V3** between them,
a 1.5-way Salisbury clutch-pack LSD, which is what SDM26 runs. It is modelled
from the team's own April 2026 study ("The Differential Drexler Study"):

```
T_c = C |T_in| + B        the largest torque DIFFERENCE the pack can hold
```

`C` is the lock fraction of the ramp in use and `B` the breakaway preload. The
study's central piece of advice is not to derive the internal geometry, because
Drexler publishes neither the pin radius nor the mean clutch radius, but to
back-calculate `C` from the lock percentages in the manual and carry it as one
identified constant. Those are 30 deg -> 0.88, 40 -> 0.60, 45 -> 0.51,
50 -> 0.42, 60 -> 0.29. The car ships on the default 40/50 ramps, so
**0.60 on power and 0.42 on coast, with 25 N.m of preload** -- the same three
numbers the Assetto Corsa mod is pinned to. All three are live in the spec
sheet, because the study also warns that the manual is marketing-optimistic and
that measured on-track values run 60-80% of it.

The clutch opposes the wheel-speed difference, saturating at half the capacity,
with a soft sign through the stick band so the split stays continuous at a
500 Hz substep. Torque leaves the faster wheel and arrives at the slower one,
and the driveline's reflected inertia is solved on the **carrier**, which turns
at the mean of the two side gears -- hanging half of it on each wheel would make
the axle behave far more locked than the clutch pack actually makes it.

What that buys, and why it is worth two extra states:

- **On a lift** the coast ramp drags the faster, outer wheel and steadies the
  car. Measured in `validate.js`: a 16 m/s corner dropped to zero throttle
  spikes the yaw rate 56% with an open diff and 12% with the Drexler, and body
  slip goes from double figures to about a degree. This is the single largest
  change to how the car behaves.
- **On the throttle** it sends torque to the slower, inner wheel, and the inner
  wheel pushing harder than the outer pushes the nose wide. That is power
  understeer, and it is why going from open to the ramp the car runs moves the
  steady-corner balance measurably toward the front.
- **In a tight, low-speed corner** it lifts the inside rear and spins it, which
  is the behaviour the study describes as the cost of a locked diff on an FSAE
  autocross hairpin. Past about 0.6 of lock, more lock stops buying understeer
  for exactly that reason; there is a genuine optimum per corner type rather
  than a monotone trend.

## Powertrain

Torque comes straight from
`helios-dev/crates/engine-sim/tests/fixtures/sweep_python_v1/sdm26_characteristic_4k_to_15k.csv`
 -- the SDM26 characteristic-junction sweep, 4000-15 000 rpm in 23 points, from
the CFD module's 1-D FV engine solver. Peak 62.6 N.m at 8000 rpm, 58.1 kW at
11 500 rpm.

That curve is not a smooth dyno arc. It has the wave-action features the solver
predicts -- a hole at 6500, the spike at 8000, a second wind at 11 000-11 500
before the restrictor chokes it -- and those survive into the driving model, so
gear choice matters the way it does in the car. Engine braking is taken from the
sweep's own `fmep` (T = fmep.Vd/4pi), giving ~12 N.m of overrun drag at 10k.

The clutch is modelled properly -- locked or slipping against a torque capacity,
with the crank on one side and the wheels on the other -- rather than pinning rpm
to road speed. That is what makes launches, bogs, stalls and the ignition-cut
shift behave. Driveline inertia is split at the primary, because that is where
the clutch physically sits on a CBR600RR: the crank sees 17.4:1 in first while
the basket, shafts and sprocket only see 8.25:1. Lumping both at the crank
overstates reflected inertia by about 45%.

## Courses

Traced 2026 FSAE Michigan geometry from the Helios lap sim's `-visual` track
JSONs, resampled to 1 m by `tools/prepare_data.py`:

- **Autocross** -- 685 m, single timed run, 192 cones, 3 sectors
- **Endurance** -- 2122 m closed circuit, 586 cones, 4 sectors

Cones are placed on both 3.5 m edges with spacing that tightens through corners
the way a real course does (7 m on straights, 3 m in hairpins). Scoring follows
the rules: +2.000 s per cone knocked down or out, and because Off Course is a
DNF on a real run rather than something a game can end on, it is scored +10 s
per excursion and flagged. Penalties reset per lap on endurance.

Cone contact is an exact rectangle-versus-circle test against the chassis
footprint, not sampled points with a slop radius -- with only 1.05 m of clearance
each side of a 1.39 m car in a 3.5 m corridor, slop would eat the usable width
of the course.

### Generated courses

**Generated autocross** and **Generated endurance** in the course menu lay out a
course nobody has driven, from a seed. Type any seed (letters and digits, up to
12) or press **New seed**; the same seed gives the same course on every machine,
so a seed is something to put in the group chat. The course id is
`gen-ax-K7Q2` / `gen-en-K7Q2`, and it works everywhere a course id does:
`--track gen-en-K7Q2` on the command line, the Runs tab, the ghost picker, the
archive best. Personal bests are per seed.

The generator (`src/track/generate.js`) is the rulebook as a grammar. FSAE Rules
2021 V1 D.11.1.1 (autocross) and D.12.2.2 (endurance) describe a course as
straights, constant turns, hairpins, slaloms and "chicanes, multiple turns,
decreasing radius turns", each with dimensions, so it draws elements from that
list with the rules' numbers -- 3.5 m / 4.5 m wide; straights no longer than
45 m / 61 m, or 60 m / 77 m with hairpins at both ends; constant turns 23-45 m /
30-54 m diameter; hairpins at least 9 m outside diameter; slaloms 7.62-12.19 m /
9-15 m apart; about 0.8 km per run, 1.15-1.4 km per lap -- and chains them nose
to tail as arcs and straights. Chains that cross themselves, leave the pad or
(endurance) cannot be closed back onto the start line with a legal pair of turns
are thrown away and the seed's next attempt is tried, so the result is
deterministic and always legal. A quasi-static lap-time estimate holds each
course to the rulebook's average-speed band (40-48 km/h autocross, 48-57 km/h
endurance), which is the only thing the rules say about speed. Endurance courses
also get the rule's designated passing zones: their two or three longest
straights open to twice the width, cones and ribbon following.

`tools/test_generate.mjs` checks forty seeds per event against every dimension
above.

### Slaloms and gates

A slalom is a straight line of cones, on the traced courses and the generated
ones alike. The centreline runs straight through it -- the driver supplies the
weave -- and the corridor opens up 3 m into a pen around it, cones following,
so there is room to. Slalom cones are the third kind of cone (`side` 2), on the
line at the rulebook spacing, and each one carries a **gate**: the line's
direction and the side the car must pass it on, alternating cone to cone
(`[x, y, 2, dx, dy, pass, slalom]` in the JSON).

The gates are what stop a driver straightlining a slalom for a tenth, which the
width test alone cannot see. Each cone is judged the moment the car's CG
crosses the plane through it perpendicular to the line, going forward; on the
wrong side of the line at that moment is a missed gate. D.8.1.7.a makes a
missed gate an off course and D.11.3.2.b scores missing any gates of one slalom
as ONE off course, so that is exactly what it is here: the lap is invalid, once
per slalom per lap, logged as a `missed-gate` event. A recover resets the gates,
so the jump itself never reads as a pass.

The traced 2026 courses carry their slaloms from the published course maps
(fsaeonline.com, Formula SAE IC 2026 MapViewer), read against the maps' 100 ft
grid:

- **Autocross** -- outbound leg at 850-950 ft, five cones at 25 ft (7.62 m);
  return leg at 1030-1180 ft, four cones at 40 ft (12.19 m). The bigger waves
  at 1200-1450 ft on both legs are 60-70 ft apart: esses, not a slalom.
- **Endurance** -- two dashed pens on the pit-side (bottom) straight, at
  1171-1360 ft and -25-165 ft, six cones each at 11 m.

The trace follows the slalom line through each, which is why they read as
straights in the geometry; `tools/prepare_data.py` places the cones from the
`SLALOMS` table and opens the pens.

### Michigan International Speedway

A third entry in the track menu, and a different thing to the two courses: a
**venue** -- a bounded place to drive around rather than a run you are timed on.
No cones, no sectors, no penalties.

Built to the published specification where that is defensible: 2.000 mi
perimeter exactly, 73 ft width, banking 18 deg in the turns / 12 front / 5 back
eased across the joins, 7.23 m of rise across the width, a 1.07 m concrete wall
and a 6.4 m catchfence.

The plan outline is **a stadium oval, not MIS's D shape**, and it is labelled as
placeholder in the JSON. A closed loop of two straights and two circular arcs
can only close when the straights are *equal* -- the closure equations reduce to
`S1 - S2 = 0` for any radii and any sweep angles -- and MIS's published straights
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
(MIT). The repository usually linked for this -- `engine-sim-community-edition` -- 
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
engine and the speaker, and the result crackles whenever the frame time moves -- 
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

The same model exists as the `engine-audio` Rust crate in `native`. They
are kept in lockstep the way `sim-core` is -- the Rust side emits golden vectors
and `tools/validate.js` checks this build against them, currently agreeing to
2e-3 over 512 samples, which is f32-against-f64 rounding and nothing else.

### Tone and loudness

An exhaust is a bass instrument, and the model is checked against that: every
operating point from idle to the limiter puts 99%+ of its energy below 1.5 kHz,
peaking at the firing frequency.

**Loudness tracks combustion power** -- heat release per cycle times firing rate
-- rather than being normalised flat. Measured A-weighted through the running
synthesiser, relative to the limiter:

| idle | 2500 part | 4000 | 7000 | 10000 | 13000 | 3000 overrun |
|---|---|---|---|---|---|---|
| -37 dB | -29 dB | -14 dB | -8.6 dB | -3.6 dB | 0 dB | -42 dB |

Loudness is checked A-weighted, not by raw energy, and the distinction is not
academic: an earlier version was already quieter at idle by RMS while sounding
louder, because 30% of its A-weighted energy sat above 1.5 kHz against 1% of its
raw energy. The ear weights 2 kHz roughly 30 dB above 50 Hz.

Three pieces of that are physics rather than equalisation: pipe losses grow with
frequency the way boundary-layer losses really do, the tailpipe carries heavy
damping standing in for the muffler the car must have (FSAE caps noise at
110 dBA), and the exhaust-port flow law is linear rather than square-root at
small pressure differences, which is both the correct viscous limit and the
thing that stops the port chattering against its own returning waves.

The old oscillator bank is still there as a fallback for browsers without
AudioWorklet.

### Levels

Per-source sliders on the home screen -- master, engine, tyres, wind, cone
strikes -- because the sources are not interchangeable. The engine is continuous
and sets the mood, tyre scrub is information about how much grip is left, and a
cone strike is a discrete event telling you that you have just taken a two-second
penalty. Which you want louder depends on whether you are learning the car or
chasing a time. Levels persist. It sounds recognisably like the same engine and obviously
synthetic next to the real thing.

## Control devices

Four profiles: **keyboard & mouse**, **Xbox**, **PlayStation**, **wheel &
pedals**. Pick one on the home screen; a connected device selects its own unless
you have chosen by hand. Settings persist as a diff from the shipped defaults.

They are not variations on a theme -- they differ in what the driver can
physically command, so each gets its own steering dynamics:

| | max steering speed | acceleration | deadzone | curve |
|---|---|---|---|---|
| keyboard | 180  deg/s | 700  deg/s^2 | 0 | linear, speed-limited lock |
| gamepad | 300  deg/s | 2200  deg/s^2 | 0.10 | 1.7 expo |
| wheel | 720  deg/s | 12000  deg/s^2 | **0** | **1.00** |

All at the road wheel. Divide by the 4.411 steering ratio for rim figures.

**On a keyboard the lock itself shrinks with speed**: Ackermann for 1.4 g plus
the peak slip angle plus 3 deg, so all 46 deg below ~8 m/s, ~17 deg at 15 m/s, ~14.5 deg at
20 m/s. A key has no position, and without this it went to full lock at any
speed -- at 15 m/s that is 10 deg past the front tyres' peak, and the car spun.
The keyboard pedals are ramped too (throttle 0->1 in 0.4 s, brake in 0.25 s,
both off in 0.1 s), and traction control and ABS are ticked by default for the
keyboard profile, because a step to full throttle spins the rears within 20 ms
and a step to full brake locks the fronts within 140 ms. The physics never sees
any of this; it is the input layer standing in for a foot and a hand.

**Steering is a rate- and acceleration-limited servo**, and both limits are
adjustable per device. The acceleration limit matters most on a keyboard, where
every input is a step: without a bound on how fast the steering *speed* can
change, a step in position becomes a step in velocity, which no hand and no
steering motor can produce.

### Wheels

The mapping from rim angle to road wheel is the setting that matters most:

- **Match the car** -- the rim turns the road wheels through SDM26's
  **measured** rack: the toe-vs-steering-wheel table the team recorded on the
  real car (`params.steering.rimToRoadDeg`, and the identical array in
  `sim-core`'s `vehicle.rs`). That rack is *progressive* -- about 5.3 deg of
  rim per road degree on centre, falling to 3.4 past 90 deg -- and it reaches
  **46 deg of road wheel at 179 deg of rim, so 358 deg lock to lock**. Set both
  your wheel's driver software and the rotation slider in the panel to 358 and
  hand position *is* front-wheel angle, with the base's own stop at the car's.
  The two settings must always agree; if they disagree everything downstream is
  scaled by the ratio between them.

  This replaced a constant 4.411 ratio to a 28 deg stop at 123.5 deg of rim,
  which was 19% too quick on centre -- where most of the driving happens -- and
  then hit a wall 18 deg of road wheel before the real rack does.
- **Scale to lock** -- whatever rotation the wheel is set to becomes full lock.
  Nothing to reconfigure, but the ratio is then a fiction and the steering is
  far slower than the real car's.

Deadzone 0 and curve 1.00 are correct on a wheel and not defaults to tune away:
the device measures hand position directly, so smoothing it discards real
information.

Pedals calibrate against the travel your set actually produces -- a G29 brake
rests near -1 and tops out near +1, a load cell may never reach +1 at any force
a person can apply. The panel shows a **live axis monitor**, because wheels do
not use the Gamepad API's standard mapping and every vendor assigns axes
differently, so watching which number moves is the only reliable way to find a
pedal.

### Force feedback

A direct-drive wheel is driven from the vehicle model, not from a canned
effect. The signal is the **self-aligning torque of the front tyres**:

- lateral force through the **pneumatic trail**, which is longest at zero slip
  and collapses to zero as the contact patch starts to slide (`tire.js`,
  brush-model shape). This collapse is why the rim goes light *before* the
  front lets go, and it is the signal a driver actually reads;
- plus the **mechanical trail** from caster (`params.steering`: 4.743 deg caster
  and 18.85 mm of trail from the team's OptimumK export);
- summed per tyre with the axle's load split, so the outside tyre dominates as
  the car rolls; through the 4.411 steering ratio and a rack efficiency to the
  rim. The vehicle model reports it as `telemetry.rimTorqueNm`.

`forceFeedback.js` adds what a real column has that the model does not --
damping, friction, the end stops at the car's lock -- and describes texture
(wheelspin, lockup, grass) and impacts (cones) for the motor. Everything is in
newton-metres at the rim until the last line, where it is divided by the
wheel's **rated torque** (5.5 N.m for a MOZA R5): the same settings feel the
same on any base once each is told what it is.

The web platform has no path to a wheel motor, and a wheel is a closed loop
through the driver's hands where every millisecond between rim and motor is
felt. So in the desktop build the **physics moves out of the webview**: the
rig thread (`src-tauri/src/rig.rs`) reads the wheel over DirectInput, steps
the vehicle model, applies the driver aids and the oval's barrier, mixes the
force feedback and writes the motor, all at **1 kHz on one native thread**.
The webview keeps rendering, HUD, audio, cones and timing, and exchanges one
message per frame with the rig (`vehicle/nativeCar.js` stands in for
`BicycleModel`). A tick costs a few tens of microseconds; the panel shows the
live rate, tick time and overrun count.

The native model is `native/crates/sim-core`, and it is the same model: the
JS build and the Rust crate are checked against each other to floating-point
noise on a scripted drive (`validate.js`, "RUST PARITY", against
`data/vehicle-golden.json`). In a browser the JS model runs and the force
feedback is computed for display only.

SDM26 puts about **12 N.m per g** into the rim, and the aligning torque
**peaks near 15 N.m at about 4 deg of front slip** -- well before the tyre's
own force peak at 7.3 deg, which is exactly why the rim goes light before the
front lets go. The peak is what the gain is set against: `defaultGainFor` is
`rated / 15`, so 0.37 on a MOZA R5 and unity from 15 N.m up.

Above that the mix is **compressed, not clipped**. A hard clamp at the rated
torque erases the one cue the whole model exists to deliver: at the old gain an
R5 sat pinned at full output from 0.8 g, through the 1.5 g torque peak, and
through the fall-off past it, so the driver felt a wall and then a slightly
lighter wall. Two settings shape it, both in the panel:

- **`gamma` (0.75)** lifts everything below full scale, the job AC's
  `ff_post_process` GAMMA does.
- **`knee` (0.6)** bends everything above 60% of output through a tanh, so the
  peak and the drop past it stay readable as an arc.

On an R5 the command now runs 0.38 at 0.3 g, 0.83 at 1.0 g, 0.91 at the torque
peak, and falls to 0.58 in a full slide -- a 36% drop the hands can read, where
before it was a flat 1.00 from 0.8 g to 1.6 g. The **end stop** and the
standstill terms sit outside that compressor: a stop that scaled with a taste
setting was not a stop, and a stationary tyre being scrubbed about its kingpin
has weight whatever the gain says.

### When a time does not count

Every number in the car is editable, which is the point of a sheet where the
provenance of each one is on the row beside it -- but a lap on a 220 kg car
with 1.4x the grip is not a lap, and it cannot sit in the same list as the runs
the team is judged on.

So the simulator draws one line. The **run-to-run setup list** -- roll
stiffness, brake bias, the diff's three numbers, launch rpm, final drive -- is
every change the real SDM26 can be given between two runs, and a lap driven on
any combination of them is a lap the car could have driven. It counts.
**Anything else** -- mass, grip, aero area, **aero balance**, the gear ratios,
driveline efficiency, brake torque, tyre radius, an inertia, a geometry
number -- means the car is not the car, and from that moment:

- the HUD says **TIME NOT COUNTED - CAR MODIFIED**, top centre, not behind a
  density setting and not inside a panel the cockpit camera hides;
- the lap is still driven, timed, shown and recorded, but it cannot become a
  best, a reference, a sector record or the archive's best on the course --
  exactly like an off-course lap;
- the run's manifest carries `counted: false` and names the parameters
  responsible, and the Runs tab says so on the row.

It **latches for the run**: putting mass back mid-lap does not un-drive the
part of the lap that was driven light. A restart clears it.

### What a run records about the car

A run's manifest has always carried its setup. It now carries the whole car:

- `setup` -- the 31 parameters with sliders, which is also what a `.hset` is.
- `car` -- **every** number in the model, all 68, flattened to dotted paths.
  The 37 that had no slider include the ones actually worth cheating with:
  peak grip, the gear ratios, driveline efficiency, the rev limit, brake
  torque, tyre radius. A run driven on a locally edited build used to look
  identical in the log to an honest one.
- `engine` -- name, point count, peak torque and an FNV-1a hash of the torque
  curve, because `data/sdm26-torque.json` is a file on disk and nothing in the
  parameter snapshot can see it. Hashing it does not stop anyone editing it --
  nothing running on the driver's own machine can -- but two runs claiming the
  same car and the same sim version with different engine hashes did not use
  the same engine.
- `counted` and `modelChanges` -- the verdict above, and why.

In the **Runs tab** each run now says what it was driven on, flags a setup
that moved mid-run (the laps before and after it were driven on different
cars), and has a **Setup** button that diffs that run against as-shipped, the
car as it is now, or **another run** -- the question a test day actually asks.
**Load this setup** puts it back in the car.

**Small bases.** The shape above is the same on every base; what differs is
the torque behind it, and on a 5.5 N.m base the limit cue is inside the noise
of the driver's own arms. Measured with `tools/ffb_sweep.mjs` (the model,
headless, through the same compressor): the aligning torque drops only 18% by
the force peak and 40% in a full slide, because the 19 mm of mechanical trail
holds it up under the collapsing pneumatic trail; after the compressor an R5
renders that as 0.4 and 1.3 N.m, where a 20 N.m base gets 1.9 and 4.8. And
with the steer held, oversteer arrives as the *same* lightening -- the front
slip grows with the rotation, so the torque falls but never reverses. Two
optional effects in the panel, **both off by default**, spend a small base's
range on the cues instead:

- **Understeer effect** scales the aligning torque down past the front's
  grip peak (`1 - effect * smoothstep(front slip, 0.7, 1.3)` in normalised
  slip, so nothing changes before the model's own torque peak). At 0.6 with
  gamma 1.0 and knee 0.85 an R5 goes 5.2 N.m at the peak, 3.1 at the grip
  peak, 1.3 in the slide: a 3.9 N.m cue in place of 1.3.
- **Oversteer effect** pushes the rim toward counter-steer as the rear runs
  ahead of the front (`telemetry.balance`, rear minus front normalised slip,
  through a smoothstep from 0.15 to 0.65), as a fraction of rated torque. It
  gives the wheel a direction: understeer is light, oversteer pulls.

A 15 N.m base needs neither. Both are mixed identically in `forceFeedback.js`
and `rig.rs`, checked in `validate.js` and the rig's own tests.

**Asphalt vibration**, a third optional effect and also off by default, is the
surface coming up through the rack. The vehicle model's road is perfectly
smooth, so unlike everything else in the mix this one is *synthesised rather
than simulated* -- which is why it ships off. It is a feel setting, and this
simulator is also how the team judges a setup change; an invented texture sits
on top of the cue they are reading. Turned up, it is a continuous buzz worth
up to 8% of rated torque (0.44 N.m on an R5), faded in from 2 to 12 m/s so the
paddock stays quiet, pitched at 3.2x wheel-rotation frequency between 22 and
75 Hz, and silent off the course where the grass rumble already owns the
channel. The rig renders it as two tones an irrational ratio apart so it never
settles into a hum, inside whatever headroom the base torque and the slip
texture leave; a straight stays centred. Try 0.3-0.5. Wheel only: on a pad the
rumble still fires for wheelspin, lockup and grass, and nothing else.

If the wheel pulls the wrong way, there is an **Invert** switch -- and that
would be worth reporting, because the sign convention is worked out rather than
guessed.

**Any wheel, not one wheel.** The rig reads whatever DirectInput can see: the
base plus up to three more devices (a separate pedal set, a shifter, a button
box; their axes appear at 8 and up, buttons at 32 and up). When a base is
recognised, `wheelPresets.js` fills in what cannot be read from the device --
the motor's rated torque, the rotation it ships with, a first guess at the
pedal axes -- and derives a gain that fits the motor. MOZA R3 to R21, Logitech
G27/G29/G920/G923, Thrustmaster T150 to T-GT, Fanatec CSL DD to DD2, Simucube
2, Simagic, Cammus, Asetek and VRS are in the table; anything else is treated
as a 5 N.m base until you set the slider. Presets apply once per base, so a
calibration you did is never overwritten by a relaunch. A base without an
actuator DirectInput can drive (console mode, or a wheel without PC-mode FFB)
still steers and reads its pedals; the panel says why it is silent. With more
than one controller plugged in, the panel has a picker.

Whatever the base, zero its own centring spring and damping in the vendor
software (Pit House, G HUB, the Fanatec tuning menu, True Drive): the rig
switches DirectInput auto-centre off, but a base-level spring would fight the
tyre model.

## Vehicles as data

A vehicle is a JSON definition naming which model to use for each subsystem -- 
tyre, powertrain, suspension, aero, engine sound -- plus its parameters. That is
what makes it savable, exportable and readable by both builds.

Duplicate a vehicle, edit it, swap a subsystem, export it, import it. Saved
vehicles store a **sparse diff against their ancestor**, so improving a shipped
default still reaches everything derived from it instead of each copy being
pinned to whatever shipped the day it was made.

Only the SDM26 is defined. The framework is the deliverable; inventing a
plausible-looking second car would put numbers in the repository that nobody has
measured. Suspension and aero are registered but still embedded in the solver,
and the picker says so rather than offering a choice that silently does nothing.



## CAD bodywork

Drop a glTF binary at **`data/car.glb`** and both the browser and the desktop
build draw it instead of the procedural body. Nothing else to configure. With no
file, the procedural SDM26 is drawn, which is the default and is deliberately
not shipped over.

The Bevy build reads the same model from `apps/bevy-spike/assets/car.glb`.

```bash
python tools/make_reference_car.py reference-car.glb   # a model in the right frame
node tools/check_car_glb.mjs your-export.glb           # check yours before driving it
```

### You do not have to match the simulator's coordinate system

Export in whatever frame your assembly is already in. The loader solves the
frame from the four wheel hubs and fits the model itself:

```
forward   rear hub midpoint -> front hub midpoint
right     left hub -> right hub
up        right x forward
origin    the CG, placed from the vehicle parameters relative to the axles
scale     wheelbase ratio, snapped to a real unit conversion
```

A Z-up, millimetre, arbitrary-origin export facing +Y comes out identical to one
authored in the simulator's own frame -- that is a regression test, not a claim.
The checker prints what it did, e.g. *"fitted: scaled from millimetres, rotated
90 deg, origin moved 0.743 m"*.

Two conventions are **not** negotiable, and neither of them is ours: glTF
mandates **Y-up** and **metres**. Blender's exporter converts Z-up to Y-up on
its own, so in practice this costs you nothing.

Scale is snapped to a recognised unit conversion (mm, cm, inches, feet) rather
than applied continuously -- a wheelbase that genuinely disagrees with the
vehicle parameters is reported rather than silently stretched to fit.

If you would rather set the frame in CAD anyway, you can do it without moving
geometry: `Insert -> Reference Geometry -> Coordinate System`, then choose it as
the **Output coordinate system** in the STEP export options.

### The frame it fits into

| | |
|---|---|
| **Origin** | the centre of gravity, projected onto the ground |
| **Axes** | +X forward, +Y up, **+Z to the right** |
| **Units** | metres |
| **Front axle** | x = +0.788 |
| **Rear axle** | x = -0.742 |
| **Wheel centres** | y = +0.200 |

`+Z` is to the **right**, matching `carmesh.js` (which puts FL at `z = -track/2`)
and the only choice that makes the triad right-handed, since forward x up =
right. Backwards mirrors the car, which on a symmetric model is completely
invisible -- it is how the reference car was wrong for a while.

**The Bevy build does not auto-fit.** It uses the glTF node transforms directly,
so a model for it does have to be in the frame above.

### Node names

`body`, `wheel_fl`, `wheel_fr`, `wheel_rl`, `wheel_rr`, `steering_wheel`.

The wheels and the steering wheel must be **separate nodes** -- the simulator
animates them by setting the node's rotation, so a wheel merged into the body
cannot turn. Everything not named is treated as bodywork and drawn fixed to the
chassis.

You do **not** need to set each wheel's origin at its hub. The loader measures
where the geometry actually sits and moves the hub to match, so a wheel can be
anywhere within its node and still spin about its own axle. Part origins do not
survive STEP as object origins anyway, so requiring it would only have meant
redoing the work in Blender.

**Hub positions come from the file**, not from the vehicle parameters. If the
two disagree the fix is the model, and seeing the wheels in the wrong place is
how you find out.

### SolidWorks to glTF

1. **SolidWorks -> STEP AP214.** Not STL -- STL is triangles with no part names,
   no materials and no hierarchy, so there is no way to find the front wheels
   afterwards in order to steer them.
2. **STEP -> Blender** (the free `STEPper` add-on) or **FreeCAD** (import STEP,
   export glTF). Tessellate at 1-2 mm; 0.1 mm is CAD-accurate and produces a
   model far too heavy to render.
3. **Decimate to ~150k triangles** for the visible body, and *delete* internal
   parts rather than decimating them -- most of an assembly is inside the car.
4. **Name the nodes** as above, apply any node scale (Ctrl+A in Blender), and
   orient to the frame in the table.
5. **Export .glb** with normals and materials, to `data/car.glb`. Orientation
   and origin do not matter; the loader fits them.
6. **Run the checker.** It runs the simulator's own loader rather than
   restating its rules, so what it reports is by construction what the
   simulator will do -- the fit it applied, the hub positions against the
   vehicle parameters, triangle count and missing materials.

## Validation

`node tools/validate.js` runs the same model headless against the three events
the team has real numbers for.

| Check | Result | Reference |
|---|---|---|
| Skidpad lap, 9.125 m radius | **5.21 s** | SDM26 ran 5.02 s |
| Skidpad lateral | 1.35 g | above mu because 11 m/s is worth ~250 N of downforce |
| Limit balance, 10/15/20 m/s | front peaks first, utilF - utilR = +0.43 / +0.45 / +0.43 | pushes, peak body slip 2-4 deg |
| Yaw mode at 15 m/s | 4.5 Hz, zeta 0.99 | linear 2-DOF from the model's stiffnesses |
| Keyboard key held to the lock, 10/15 m/s | 1.52 / 1.69 g, body slip < 4 deg | pushes, does not spin |
| Tyre peak slip angle | 7.3 deg | team MF6.1 fit at the 10 psi the car runs |
| Tyre past the peak | 94% at 2x, 89% at 3x peak slip | a slick keeps most of its force |
| 75 m accel, managed launch | 4.79 s | QSS says 4.2 s -- see below |
| 75 m accel, throttle pinned | 5.27 s | +0.48 s lost to wheelspin |
| Braking from 25 m/s | 22.9 m, 1.90 g peak | -- |
| Cornering stiffness | 407 N/deg per tyre (rear) | 10" slick at 655 N -- see below |
| Steering rack | 46 deg of road wheel at 179 deg of rim, ratio 5.27 on centre to 3.4 at 90 deg | the team's measured toe-vs-rim table |
| ETC map, 4000 random curves | 0 overshoot, 0 backwards steps | monotone guarantee |
| Roll stiffness 40->70% front | monotonic | monotonic understeer |
| Brake bias sweep 48->75% | rear-locks-first -> front-locks-first | crossover ~57% |

The skidpad is the anchor: it is a clean mu measurement, low speed, no gearing,
no line freedom. **It has drifted**: 5.02 s real against 5.21 s here, where it
used to sit at 5.05. Moving the tyre's peak slip angle from an estimated 8.5 deg
to the team's measured 7.3 cost a little lateral on a skidpad, because a sharper
tyre sits further along its own curve at a given angle and couples harder with
the throttle the car is carrying. The honest repair is to re-pin `muLat`, and
the right moment for that is when `frontGripFactor` stops carrying the car's
entire understeer margin -- see the differential work in the dev plan. Widening
the band again instead would be the wrong move.

Cornering stiffness is above the 310-340 N/deg the team's own MF6.1 fit implies,
and that is the price of a fixed-shape Magic Formula: `B`, and with it the
stiffness, is whatever puts the peak where the data says it is. A load-dependent
peak slip angle is the fix and is not done.

The 75 m time is honestly slower than the lap sim's 4.2 s and the test band says
so. This model carries driveline rotational inertia -- about +94 kg apparent in
first -- that a quasi-steady lap sim ignores entirely. If that check ever comes
back at 4.2 s, something has stopped modelling the inertia.

Also verified in-browser: lap detection is exact on both courses (endurance
141.53 s against 141.49 s expected for a constant 15 m/s walk of the centreline;
autocross finishes exactly once), no console errors, and the frame costs about
1.0 ms of update plus render.

`node tools/smoke_desktop.mjs` checks the built executable, which is a different
question from "does the code work" -- it launches the exe, attaches to WebView2
over the DevTools protocol, and asks the running page whether the game actually
booted. A process that stays alive and a blank window look identical from the
outside, so the test confirms the embedded track and engine data loaded and
WebGL 2.0 came up clean rather than just that nothing crashed.

## Known limitations

- **One curvature spike in the traced endurance centreline.** A single point at
  s ~ 607 m has a 2.83 m radius. It used to be below the car's turning circle;
  now that the rack's measured 46 deg of lock has replaced a 28 deg estimate the
  kinematic minimum is 1.48 m, so the car goes round it. It is still a tracing
  artifact rather than a real hairpin and still worth cleaning up at the source.
- **75 m acceleration is about 0.8 s slower than the real car.** SDM26 runs
  4.2-4.4 s; the model takes 5.15 s on the measured torque curve. This is a
  defect, not a modelling choice, and `validate.js` says so where the check
  lives. It is not engine power: 60% more torque buys 0.28 s. It is not
  driveline inertia, grip, mass, drag or shift time either, each of which was
  measured on its own and is worth between 0.02 and 0.17 s. The 2026-09-22
  review (sim/docs/HANDOFF-physics-2026-09-22-b.md) found the real causes,
  none of them the slip floor: the pull-away clutch bogs the engine to ~1100
  rpm (its capacity follows driveline speed, not engine speed); the clutch
  never locks under hard acceleration, so the harness holds 1st to 14,300
  rpm; and the rules time from the line with the car staged 0.3 m behind it,
  worth 0.36 s on its own. The fixes are in the shared powertrain, so they
  wait on a decision about the frozen bicycle. A real 5/3 launch log agrees:
  the real car's power matches the model, and its launch does not.
- **Single-track at the front, two wheels at the rear.** The front axle is one
  unit: there is nothing between the front wheels but the road, and grip still
  responds to lateral load transfer through the two contact patches. The rear
  is two wheels with the differential between them (below). There is still no
  per-corner camber or toe, and no front per-wheel state, so brake-pull under a
  locked front and scrub-radius kick are not modelled. Setup work belongs in
  Helios Setup and Oracle; this is a driving model.
- **`frontGripFactor` (0.80) is still a hand-set understeer margin.** With one
  tyre character at both ends the model was neutral to within 1% of force and
  spun from any step steer, so the front is derated to make it push. The
  differential now supplies real understeer on the throttle and real stability
  on a lift, which is what that number was standing in for, so it should come
  back toward 0.90 -- but only together with re-pinning `muLat` to the skidpad,
  and only after someone has driven it.
- **No tyre thermal or wear model.** mu is constant over a run.
- **Flat ground.** The venue is a lot, so this costs less than it would
  elsewhere, but there is no surface elevation or grip variation.
- **Suspension is a gradient, not a state.** Roll and pitch come from the
  validated deg/g gradients for camera and load transfer; there is no ride
  model, so kerb strikes and heave dynamics are not simulated.

## Recording, replay and the delta

Two channels were wrong before 0.5.7, and every log from before then carries
the mistake: `sim.diff_locked_nm` recorded the clutch's locked flag (so it
reads 0 everywhere) instead of the differential's transfer torque, and the
desktop build sent one rear wheel's slip ratio and utilisation for both, so
`sim.kappa_rl` and `sim.kappa_rr` were always identical. From 0.5.7 the diff
channel is the clutch pack's actual transfer torque in N.m and the two rear
wheels are logged separately, which is what tells a setup engineer whether an
exit wheelspin was one wheel or two. See `docs/setup-autocross-2026-09-20.md`
for the analysis that found it.

Every run is logged. Not a summary -- the whole car, at 100 Hz, in a form that
loads into Helios beside the real car's telemetry with no conversion step.

100 Hz is a ceiling, not a promise: the sampler writes at most one row per
simulation step, so a machine rendering at 60 fps logs at 60 Hz. The physics is
frame-rate independent either way (it substeps at 500 Hz) but the LOG is not, so
the manifest carries `sampleRateActualHz` alongside `sampleRateHz` -- what was
achieved, next to what was asked for.

A finished run is a directory:

```
%LOCALAPPDATA%\Helios\sim-runs\<runId>  run.json        who drove, on what, with which setup; lap and sector times;
                  every cone, excursion, shift and flag; summary statistics
  telemetry.csv   76 channels, sampled up to 100 Hz on the SIM clock
```

`FSAE_SIM_RUNS_DIR` moves it -- point it at a shared drive and the whole team's
runs land in one place.

**The channel names are Helios's own.** Engine speed is `engine.rpm`, lateral
acceleration is `imu.lat_g`, the position is `gps.lat` / `gps.lon` projected onto
the real venue. So a simulator run opens in the Logs module through exactly the
same path as a test-day export and overlays on the same axes -- no importer, no
mapping table, no "sim" special case anywhere in the pipeline. The channels only
a model has (slip angles, how much grip each axle is using, the force-feedback
command, distance round the course) are `sim.*`. A `system.beacon` pulse marks
the start line and every lap, so Helios's lap detection finds exactly the laps
the driver saw on the HUD rather than inferring them from the GPS trace.

The pose is in there too -- `sim.pos_x`, `sim.pos_y`, `sim.yaw_deg`, the wheel
angles -- which is what makes a run replayable. That is deliberate: the replay is
driven from the same file the analysis reads, so it cannot drift from the numbers
sitting next to it. Nothing is re-simulated.

```bash
fsae-sim --replay <runId>              # watch it back
fsae-sim --replay <runId> --ghost <id> # with another run alongside

# One sector of one lap, against one lap of the ghost -- how Helios opens a
# team sector record. All three are 1-based; N and M are `laps[].lap`.
fsae-sim --replay <runId> --replay-lap N --ghost <id> --ghost-lap M --sector I
```

`--replay-lap N` opens on that lap. Add `--sector I` and it opens 1.5 s before
the car enters sector I, **paused** (a launch takes a few seconds to bring the
window up, and a replay that played on its own would have run through the
approach before anyone was looking), with the ghost **synchronised at the
sector entry**: both cars cross the boundary at the same instant, so the gap
only moves with what happens inside the sector. A banner says what is being
compared, shows the running in-sector gap, and once the car leaves the sector
shows the sector delta on scored times (2 s per cone in that sector).
`--ghost-lap M` compares against that lap of the ghost instead of its best.
`--sector` without `--replay-lap` is ignored, and a value that is not a
positive integer in range is dropped on its own -- the replay still opens. In
the browser build the same options are `?replayLap=&ghostLap=&sector=`.

Space plays and pauses, the arrows step a second (hold shift for a tenth), up and
down change speed, `L` jumps to the best lap, `C` changes camera, and the bar at
the bottom scrubs -- with a tick for every lap, cone and excursion, so you can go
straight to the moment rather than hunting for it.

The **ghost** is placed by time into the lap, not by distance. Distance-matching
pins it alongside you the whole way round and teaches nothing; on the clock it
pulls away where it was quicker, which is the point. The gap NUMBER stays
distance-based, because that is the only comparison that means anything once two
laps diverge.

### The delta

While you drive, a delta against a reference lap: a signed number, a bar, and --
under it -- the delta plotted against distance for the whole lap so far. That
trace is the answer to *where* you are losing it. It is flat through the corners
you matched and rises through the ones that cost you.

The reference is your own best lap this session; it updates whenever you beat it.
Or Helios can hand you one to chase from the first corner:

```bash
fsae-sim --reference <runId>   # your best from the archive, or a teammate's
```

Comparison is always against DISTANCE round the course, never the clock. Two laps
pass the same cone at different clock readings, so "what was the clock at this
instant" tells you nothing; "how long had it taken to reach this point" is the
gap a pit wall reads off two transponders.

## Helios

Helios launches the simulator, keeps the archive and ranks the times. It is not
required -- the simulator runs perfectly well on its own -- but a run started
from Helios carries the driver's account with it, and that is what lets a lap
time go on a leaderboard.

Which settings belong where is written down in [docs/SETTINGS.md](docs/SETTINGS.md).
The short version: **Helios owns the run, the simulator owns the rig.** Anything
you can only get right with the wheel in your hands lives here and stays with
this machine; anything describing what a particular run *is* comes in on the
command line and applies to that session only.

```bash
fsae-sim --track autocross --profile wheel --driver "..." --driver-id <uuid>          --session "Tuesday test" --tc off --abs off --autostart
```

## Layout

```
data/           generated: course geometry + the CFD torque curve
dist/           generated by build.rs: what gets embedded in the exe
tools/
  prepare_data.py   rebuilds data/ from the Helios sources
  serve.py          no-cache dev server for browser iteration
  make_icons.py     regenerates the app icons
  make_mis.py       generates the Michigan International Speedway venue
  glb.py            minimal glTF binary reader/writer, no dependencies
  make_reference_car.py  a car.glb in the frame the simulator expects
  check_car_glb.mjs runs the real loader over an export and reports what it did
  plan_view.py      draws the venue plan + cross-section for review
  validate.js       headless physics + ETC-map checks
  smoke_desktop.mjs launches the built exe and interrogates it over DevTools
  test_replay.mjs   recorder -> CSV -> parser -> replay, against known answers
  test_delta.mjs    the live delta, against laps whose gaps are arithmetic
  test_bindings.mjs every control is bindable, and what you bind is what the
                    game reads -- it drives a real Input and presses keys at it
  test_timing.mjs   laps, sectors as DURATIONS, penalties, a jumped sector
  test_respawn.mjs  a respawn is not undone by a snapshot from before it
  check_shaders.mjs imports every render module and checks each shader literal
                    (a backtick in a GLSL comment silently truncates a shader)
  make_sample_run.mjs  a robot driver that files real runs, for demos and tests
  publish_build.mjs a build to the feed Helios downloads from. Needs
                    SUPABASE_URL + SUPABASE_SERVICE_KEY; creates the bucket on
                    the first run, so there is no dashboard step. --dry-run
                    prints what it would publish and touches nothing.
src/
  vehicle/      params, paramMeta (provenance), tyre, powertrain, bicycle,
                ETC map, live setup adjustments, modules + library
  audio/        the engine model, and the worklet that runs it
  track/        course geometry, progress, cone strikes; venue.js for MIS
  render/       WebGL2 renderer, procedural SDM26 car geometry, venue mesh,
                glbcar (CAD import)
                post.js       HDR target, ambient occlusion, bloom, tonemap
                ground.glsl.js  the lot: a height field, not a painted plane
  game/         input, control profiles + panel, HUD, timing, audio,
                ETC editor, spec sheet, desktop shell
                controlBindings.js  what the controls ARE, and rebinding them
                recorder.js   the 100 Hz run log
                runStore.js   reading and writing runs; the CSV parser
                replay.js     playing a recorded run back
                replayPanel.js  the replay's transport and instruments
                delta.js      the live delta and its reference lap
  main.js       bootstrap and loop
src-tauri/
  build.rs      stages the frontend into dist/ on every cargo build
  src/main.rs   the native window, the launch arguments, single-instance
  src/rig.rs    the vehicle model, wheel read and force feedback at 1 kHz
  src/runs.rs   where a recorded run is filed, and how it is read back
  src/wheel.rs  DirectInput
  tauri.conf.json
```

`window.__sim` is exposed for poking at the model live -- 
`__sim.car.telemetry`, `__sim.powertrain.wotTorque(9000)`, `__sim.etc.describe()`.
In the desktop app, open devtools with `cargo run` (a debug build enables them).

Regenerate the data (needs `helios-dev/` alongside this folder):

```bash
python sim/tools/prepare_data.py
```
