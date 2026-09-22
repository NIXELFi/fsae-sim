# Handoff — physics review, bug fixes, beta 4-wheel model (2026-09-22)

This handoff covers branch `fix/physics-review-0922`, which is on origin. It isn't merged, tagged or released. The branch was cut from `feat/wheel-buttons` @ d6e322b (sim 0.6.13) and has no version bump yet.

Start here on the other machine:

```
git fetch origin && git checkout fix/physics-review-0922
cd native && cargo test -p sim-core --release      # all green
cd ../sim/src-tauri && cargo test --release         # 38 green
cd .. && npm test                                   # all green
npm run app                                         # drive it
```

## Status

**Waiting on Nick to drive both models on the rig.** None of this has been felt through a wheel yet. The switch is on the staging setup card (the card that shows until the green): **MODEL: Bicycle / 4-wheel β**. It's desktop only, and a lap on β does not count toward times.

## Commits on this branch

| Commit | What |
|---|---|
| 999e4b8 | Bug fixes from the two-reviewer physics audit (Rust and JS in step; golden regenerated) |
| 8567849 | Beta double-track + suspension + camber, rig live-swap, staging-card MODEL switch, cone sweep |
| 3a39472 | Study examples |
| 6d84ead | As-run suspension inputs, load-dependent peak slip (DT only), **bicycle frozen** |

## 1. Bug fixes (these change the validated bicycle; approved before the freeze)

- **Brakes are friction, not a torque source** (`brake_torque` / `brakeTorque`). Rust's `0.0.signum()` is +1, so a car held on the brakes with any throttle crept backwards: 7 cm in 3 s at the staging line. JS crept forward, because `Math.sign(0)` is 0. Locked wheels also flickered through reverse.
- **Front slip angle is computed in the wheel frame.** It used to pass 90° in a slide at lock (110° measured), which made the front tyre push *with* the slide. It also kept the steer sign when the car rolled backwards. The front slip ratio is now taken against the speed along the steered wheel; it was 30% off at lock.
- **Drag acts along −V, not −x.**
- **The roll arm is derived** from the sprung CG and the roll axis. The stored value was 3.4% short and ignored CG and roll-centre edits.
- **The tyre's reference load Fz0 is fixed.** Mass edits no longer re-centre load sensitivity.
- **The steering map is a single PCHIP spline for both angle and torque.** The rim-torque steps of up to 3.6% every 5° are gone.
- **Rig fixes:**
  - ABS and the understeer FFB effect read both fronts.
  - Jacking torque re-derived: KPI lift minus caster, plus a load-transfer tilt term. The standstill return is now about 0.5 N·m at the rim, down from 1.2.
  - The aligning torque is no longer faded twice at low speed.
- Skidpad moved 5.212 s → 5.189 s.

Regression tests: `native/crates/sim-core/tests/physics_review_0922.rs` and the `validate.js` "PHYSICS REVIEW 0922" block.

## 2. The bicycle is FROZEN

Nick's instruction: the bicycle must not change in any capacity. `tests/bicycle_frozen.rs` reruns the golden drive and requires `sim/data/vehicle-golden.json` to match to the last printed digit. Do not regenerate the golden to make it pass; regenerating it means changing the validated car.

- DT-only parameters live in `SuspensionParams` (`vehicle.rs`), and that includes Ackermann.
- DT-only tyre features are `forces_cambered` and `peak_alpha_scale_at`. The latter is `None` for the bicycle and gets switched on in `DoubleTrackSolver::new`.

## 3. Beta 4-wheel model (`native/crates/sim-core/src/solver/double_track.rs`)

**What's in it:**
- Four patches, each with a wheel-frame slip.
- Salisbury diff (same model as the bicycle).
- Friction brakes.
- FFB torque (aligning plus scrub).
- Rear toe and front toe.
- Measured Ackermann.
- A sprung body that rolls and pitches as damped states.
  - Geometric (roll-centre and anti) transfer and unsprung transfer are instant.
  - Elastic transfer goes through the body angles.
  - Stiffness comes from the gradients, split by `rsd_front`.
- Per-wheel camber to the road:
  - static camber;
  - roll through the linkage gain, plus tyre-squash roll at 1:1;
  - bump from pitch;
  - caster and KPI steer camber.

**The rig** swaps solvers live on `params.vehicleModel` (2 = bicycle, 3 = double track). It keeps pose, speed and gear across the swap, and the test `vehicle_model_switch_swaps_the_solver_live` covers it. Real roll, pitch and camber go to the camera and to the log: `sim.roll_deg`, `sim.pitch_deg`, `sim.camber_fl..rr`, `sim.vehicle_model`.

**Inputs and where they come from** (Drive):

| Parameter | Value | Source |
|---|---|---|
| Front / rear spring | 200 / 225 lb/in | spec sheet, 2026 setup sheets (the Ride Roll sheet's 280 is out of date) |
| Roll gradient (with tyres) | 0.66 deg/g | re-derived: ARB calculator at ~47 % RSD, in series with 520 lb/in tyres |
| Pitch gradient (with tyres) | 0.91 deg/g | re-derived for the 200 lb/in front spring |
| Damping ratio, roll & pitch | 0.70 | Öhlins TTX25 force-matched to 0.7 (Suspension DR 4.4) |
| Static camber | −0.8 / −0.7 deg | OptimumK actual |
| Camber gain in roll | 0.657 / 0.738 deg/deg | OptimumK |
| Camber gain in bump | −0.826 / −0.678 deg/in | OptimumK |
| Anti-dive / anti-lift / anti-squat | 12.8 % / 15.8 % / 11.5 % | OptimumK |
| Toe (sign "− out, + in") | front +1.1 in, rear +0.5 in (skidpad setup rear −0.7 out) | Overall Vehicle DR 5.3, setup sheets |
| Ackermann | 18.5 % | measured from `Steer_Force_Calculator/wheel_toe_angles.csv` (OptimumK said 0 %, the spec sheet says 85 %) |
| Ixx / Iyy | 24.8 / 85.3 kg m² | Full-Vehicle Sim Parameters workbook |
| Tyre camber | slip shift 0.089 / 0.107 / 0.159 rad per rad at 300 / 655 / 1000 N; PDY3 18.66 | `MF612-Hoosier 16x7_5-10 R20 7in Rim.tir` (Nick's `models.zip`) |
| Peak slip vs load | ×0.82 / 1.0 / 1.12 / 1.30 of 7.3° at 200 / 655 / 800 / 1200 N | team MF6.1 note and the MF6.1.2 fit. The fit isn't credible above ~800 N, so the top point is an estimate. |
| Front grip | `front_grip_factor` × 1.07 (0.80 → 0.856) | calibrated: skidpad setup 5.26 s, and it still pushes at 10/15/20 m/s; 1.10 spins at 20 m/s |

The MF5.2 fits in `models.zip` look broken (peak slip past 11°, almost no camber response), so they aren't used.

## 4. Numbers

**Bicycle vs 4-wheel** (from `examples/dt_study.rs`, `dt_balance.rs` and `dt_calibrate.rs`, and the tests):

| Check | Bicycle | 4-wheel β |
|---|---|---|
| Skidpad | 5.189 s | 5.26 s (skidpad setup) |
| 75 m accel | 5.06 s | 5.17 s |
| 25 m/s stop | 23.0 m | 23.5 m |

The 4-wheel model reproduces its gradients within about 1 %. Yaw response to a 3° step is essentially the same as the bicycle: 63 % at 0.092 s vs 0.093 s.

**Findings:**
- About half of the bicycle's 0.80 front deficit is the single-track itself. The as-run toe, Ackermann and load-dependent slip explain a bit more. About 14 % is still unexplained: compliance, the tyre fit, 9 vs 10 psi.
- Camber is small on this tyre. It costs about 1.3 % on skidpad, and static camber moves skidpad by only about ±0.5 %.
- RSD matters only near the limit. At 40 % RSD the 4-wheel model spins; the bicycle doesn't.
- Rear toe-in is what keeps the 4-wheel model stable at speed. The aero balance (52.4 %) sits ahead of the weight distribution (48.5 %).

**Real skidpad times:** 5.01 s (3/14), and 5.21–5.40 s on the 4/23 ARB day. The 4/23 data is confounded by run order, and it spans only ~47–50 % RSD, so it can't validate the RSD knob.

**The sim's skidpad isn't a timed lap.** It's the steady-state 2πR/v on a 9.125 m path. Real drivers hug the cones: at 8.3–8.6 m radius the time is about 3.5 % faster. They also throttle up at the exit, which is worth a few hundredths. So the sim reads slow against a good real run. See Next #1.

## 5. Open conflicts, NOT applied because they're shared with the frozen bicycle

These need a team decision:

- **Mass as run** is 203–212 kg car + driver (the 4/23 sheet has 602 lb total). The sim has 199 + 68.
- **Brake split:** a 54 % bias bar works out to about 0.73 front torque share from the brake geometry. The sim has 0.65, which matches the design pressures. It's unclear what "54 %" means.
- **Tyre vertical rate:** 91 N/mm in the Ride Roll sheet vs 52.5 N/mm in the Brakes Calculator.
- **Diff preload per event:** skidpad 0, autocross/endurance 25, accel 50.
- **Pressures:** 9 psi at skidpad, 10 at autocross.
- **RSD at blades 1-1** is 51.7 % per the ARB calculator; the sim note says 46 %.

## 6. Next, in priority order

1. **A timed skidpad.** The figure-8 cone layout with the timing gate, a driver running the tightest line it can hold, and throttle up at the exit. Replaces the 9.125 m steady-state assumption.
2. **Fit roll gradient and understeer gradient from the logs.** Drive folder "SDM26 Test Log Packs" `1KGbGN-r2-ks13JXusbrtTIO6Ho4B8O26`, with `.llgx` files and CSV exports. The parser is at `Documents\CodeStuff\llgx\llgx.py`.
   - Usable channels: shock pots, steering, lateral g.
   - Unusable: yaw gyro, wheel speeds (they saturate), tyre temps.
   - See "SDM26 Data Channels Review.xlsx".
3. **Per-event setup presets** (toe, preload, pressure, RSD).
4. **A heave DOF plus the 5×5 ride-height aero map** (`SDM26_Aero_Coefficients.xlsx`, CFD). Front aero share ranges from 39 % to 57 % across the map.
5. **A controlled test session:** constant-radius circles at 3 speeds and a step steer, same driver, alternating settings.

## Rules for whoever picks this up

- Never change bicycle physics or parameters the bicycle reads. `bicycle_frozen.rs` guards this.
- JS and Rust bicycles must stay in parity (`validate.js` RUST PARITY, 5e-10).
- Release: tag `sim-v*` on the working branch, not `main`. Bump the version in 5 places. Publish with `--build windows=… --build macos=…` in one run.
- A lap on a modified model doesn't count, and `vehicleModel` = 3 counts as modified. That's intended.
