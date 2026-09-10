# native — Rust solver and Bevy build

Two things, kept deliberately separate:

- **`crates/sim-core`** — the vehicle dynamics solver as a pure, dependency-free
  Rust crate. Valuable on its own merits and *not* tied to Bevy.
- **`apps/bevy-spike`** — a throwaway visual spike to answer one question: does
  Bevy look better than the hand-written WebGL2 renderer in `../sim`?

`../sim` (the Tauri + WebGL2 build) is untouched and still the working app.

```bash
cargo test -p sim-core --release        # 21 tests, under a second
cargo run  -p bevy-spike                # drive it: WASD, Q/E shift
cargo run  -p bevy-spike -- --chase
cargo run  -p bevy-spike -- --screenshot shot.png
```

## sim-core

Three things swap independently, which is the whole point of the design.

**Tyre** (`TyreModel`)

| Model | Use |
|---|---|
| `LinearTyre` | Cornering stiffness to a friction ceiling. Cannot spin — right when you want grip budget, not car control. |
| `MagicFormulaTyre` | Fitted MF with combined slip via Pacejka similarity. The driving model. |

The tyre owns its own load sensitivity. That is a real improvement on the JS
build, where the bicycle model computed a load-weighted mean μ for the axle and
handed it to the tyre. Here the bicycle solver calls the tyre twice per axle at
the inner and outer loads and sums — the same idea done properly, and it means
the bicycle and double-track solvers share one path into the tyre.

**Powertrain** (`PowertrainModel`)

| Model | Use |
|---|---|
| `IdealDrive` | Torque/power limited, no gearbox. Chassis study without the engine in the way. |
| `GearedEngine` | Restricted CBR600RR on the Helios CFD sweep, six speeds, real slipping clutch. |
| `ElectricDrive` | Flat torque to base speed then constant power. An EV conversion is a parameter set, not a rewrite. |

**Solver** (`Solver`) — three fidelity levels

| Level | Model | What it adds |
|---|---|---|
| 1 | `PointMassSolver` | Speed and heading, grip-limited. No yaw dynamics, cannot spin. |
| 2 | `BicycleSolver` | 3 chassis DOF, transient. Grip still responds to lateral load transfer. |
| 3 | `DoubleTrackSolver` | Four contact patches. Real Ackermann, per-corner loads, and yaw moment from longitudinal forces through their lateral offset — the term a bicycle model structurally cannot have. |

A vehicle is pure data (`VehicleParams`), so SDM25 is `sdm26()` with three
fields changed, and a new car is a new value rather than new code.

### Validation

`cargo test -p sim-core --release`. The Rust port reproduces the JS build almost
exactly, which is the result that matters — the physics survived the move.

| Check | Rust | JS build | Reference |
|---|---|---|---|
| Skidpad, 9.125 m | **4.986 s, 1.477 g** | 4.986 s, 1.477 g | SDM26 ran 5.02 s |
| 75 m accel, managed launch | **4.776 s** | 4.762 s | QSS says 4.2 s (see below) |
| Braking from 25 m/s | **23.11 m, 1.74 g** | 23.08 m, 1.74 g | — |
| Roll stiffness 40→70% front | **7.18 → 7.45 m radius** | 7.18 → 7.45 m | monotonic understeer |
| Brake bias 48% / 74% front | **rear-first / front-first** | same | crossover ~57% |
| ETC map, 2000 random curves | **0 overshoot** | 0 overshoot | monotone guarantee |

The 75 m time is deliberately banded above the quasi-steady lap sim's 4.2 s:
this model carries driveline rotational inertia (~+94 kg apparent in first) that
a QSS sim ignores entirely.

Two sanity results worth noting: the bicycle and double-track solvers agree to
three figures in a straight line, which is exactly right since a double track
degenerates to a bicycle under symmetric load; and the point mass is *faster*
off the line than either, because with no wheel-speed states it cannot
wheelspin.

## bevy-spike

Minimal by design — the UI is one line of text, and the effort went into the
things WebGL2 cannot cheaply do: real shadow maps, PBR metallic/roughness per
part, ACES tonemapping, HDR + bloom, distance fog.

`--screenshot <path>` captures a frame and exits, which is how the look was
iterated on. It waits for the file to appear rather than a fixed frame count,
because the GPU readback is asynchronous and a frame counter silently produces
no file.

### Known rough edges

- The asphalt texture has no mipmaps, so it sparkles at grazing angles. The fog
  is tighter than reality partly to hide that.
- No cone-strike detection, timing, audio, ETC editor, spec sheet or setup
  adjustment. It is a spike, not the app.
- Gamepad works (bevy_gilrs detected the pads immediately) but is untested
  beyond enumeration.


## Parity with the JS model

`sim-core` is what the desktop build drives with (the Tauri shell's rig
thread), and `sim/src/vehicle/*.js` is what a browser drives with. They are
ports of each other and are held to it: `examples/golden_vehicle.rs` emits a
scripted 12 s drive and `sim/tools/validate.js` replays it through the JS
model, requiring agreement to 1e-6 m. Regenerate after any model change:

```bash
cargo run --release -p sim-core --example golden_vehicle > ../sim/data/vehicle-golden.json
```

The drive stays inside the tyre and above walking pace on purpose. At the
limit, or with the clutch chattering at a crawl, the model is discontinuous
and one-ulp differences between JS `Math` and Rust libm flip a branch -- which
says nothing about whether the models agree.
