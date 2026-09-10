# fsae-sim

A first-person driving simulator for Sun Devil Motorsports' Formula Student car,
on the traced 2026 FSAE Michigan courses and the Michigan International Speedway
oval.

Everything that can come from real team data does. The vehicle model is the
Helios lap sim's SDM26. The engine torque curve is the Helios CFD module's 1-D
finite-volume engine sweep. The courses are the same traced geometry the Oracle
lap sim times. Nothing here is a guess dressed up as a measurement — where a
number is invented, the code says so.

## Two programs, one repository

```
sim/      the app you drive — Tauri v2 desktop, WebGL2 renderer, zero deps
native/   Rust workspace — the solver, the audio model, the Bevy build
```

They are genuinely separate programs with different renderers, and neither
embeds the other. They live in one repository for one reason:
`native/crates/engine-audio` is the **reference implementation** of the engine
sound model, and `sim/src/audio/engineAudio.js` is a port of it that is checked
sample-for-sample against golden vectors the Rust crate emits. A change to the
model that lands in one but not the other is a broken build. Keeping them in
one repository makes that change one commit instead of two.

### `sim/` — the simulator

The one to run. A Tauri v2 window with the entire game embedded in the
executable: no server, no install, nothing to keep alive in a terminal.

```bash
cargo build --release --manifest-path sim/src-tauri/Cargo.toml
```

That writes `sim/src-tauri/target/release/fsae-sim.exe`. Double-click it.

You need the Rust toolchain and VS Build Tools. You do **not** need Node, pnpm,
or the Tauri CLI — `build.rs` stages the frontend into `dist/` on every build,
so a plain `cargo build` can never ship a stale bundle.

The frontend is plain ES modules and WebGL2 with no dependencies and no build
step, so it also runs in a browser — and that is the loop to use while working,
because `generate_context!` embeds the frontend at compile time and any `.js`
edit forces a recompile and relink of the whole crate.

```bash
python sim/tools/serve.py       # http://localhost:5273
```

Use that server rather than `python -m http.server`: the stdlib one sends no
`Cache-Control`, so a reload serves a fresh `index.html` beside a stale
`main.js` and the page looks updated while the behaviour is yesterday's.

### `native/` — solver, audio model, Bevy build

```bash
cargo test -p sim-core     --release   # the vehicle dynamics solver
cargo test -p engine-audio --release   # the engine sound model
cargo run  -p bevy-spike               # the Bevy renderer, WASD + Q/E
```

`sim-core` is a dependency-free vehicle dynamics crate with pluggable tyre,
powertrain, suspension and aero models at three fidelity levels. It is what
the desktop build actually drives with: `sim/src-tauri` links it and runs the
model, the steering wheel and the force feedback on one native 1 kHz thread.
The JS model in `sim/src/vehicle` is the browser's copy and is checked against
it to floating-point noise (`sim/data/vehicle-golden.json`).

`bevy-spike` exists to answer one question — does Bevy look better than the
hand-written WebGL2 renderer? — and is a spike, not a product.

## Checks

```bash
node   sim/tools/validate.js        # physics, ETC map, engine audio vs golden vectors
node   sim/tools/smoke_desktop.mjs  # does the built exe actually boot the game
python sim/tools/prepare_data.py    # regenerate data/ from the Helios source of truth
```

`validate.js` is the one that matters. It re-derives the numbers that must not
move — skidpad, acceleration, the ETC map's monotonicity, the audio model's
output against the Rust reference — and it has caught every regression worth
catching so far, including one where a stray `git checkout` reverted a fix that
had already been committed.

`smoke_desktop.mjs` refuses to run against an executable older than the newest
source file, because a passing smoke test on a stale binary is worse than no
smoke test.

## What is not in the repository

Large binaries, deliberately:

| Path | What |
|---|---|
| `sim/data/body.glb` | CAD bodywork, ~14 MB |
| `sim/data/wheel.glb` | CAD wheel and tyre, ~23 MB |
| `sim/data/car.glb` | A whole-car CAD export, if you have one |
| `native/apps/bevy-spike/assets/car.glb` | The same, for the Bevy build |

All optional. Without them both builds draw the procedural SDM26 body, which is
the default and is checked by the test suite. Drop a `.glb` in and it is picked
up on the next load.

They are excluded because a binary that large re-enters the history on every
revision and never leaves. If they should be versioned, Git LFS is the way to
do it, not a plain commit.

## Importing CAD

Both builds read binary glTF (`.glb`). The expected frame is:

```
origin   the centre of gravity, projected onto the ground
+X       forward        +Y  up        +Z  to the RIGHT
units    metres
```

`+Z` right is not a preference: forward × up = right is the only right-handed
choice, and it is what the procedural geometry already uses. Get it backwards
and the car is silently mirrored — invisible on a symmetric model, baffling on
a real one.

Check an export before trusting it:

```bash
node sim/tools/check_car_glb.mjs your-model.glb
```

That runs the real loader rather than a re-implementation of it, so a file it
passes is a file the simulator can draw. `python sim/tools/make_reference_car.py`
writes a model that is correct by construction — open it beside your assembly in
Blender and the required frame, scale and node names are visible rather than
described.

A body-only export (a CFD assembly, or anything through an STL round trip that
lost its part names) is also fine. The loader recovers the orientation from the
shape: the lateral axis is the one the car mirrors about, up points away from
the mass because a car is bottom-heavy, forward points away from the taller end
because the rear wing is the tallest thing on the car, and the units come from
the overall length. The wheel bays are found from where the bodywork pinches in
to clear the wheels.

## History

Both directories began as separate repositories and were merged in with their
history intact, so all 22 pre-merge commits are still here. They refer to the
old paths, which means `git log -- src/main.js` reaches them where
`git log -- sim/src/main.js` stops at the import. `git log --full-history` sees
everything.

Neither has an honest "before" commit — both were initialised partway through
the session that added the engine audio, and both baseline commits say so.

## Credits and licence

The engine sound model is adapted from
[ange-yaghi/engine-sim](https://github.com/ange-yaghi/engine-sim) (MIT). Sound
is generated rather than sampled: cylinder pressure from a Wiebe heat-release
law drives a digital waveguide exhaust, through a synthesiser chain of jitter,
DC removal, differentiation, noise, convolution and levelling. There are no
recordings anywhere in this repository.

This project is MIT licensed — see [LICENSE](LICENSE).
