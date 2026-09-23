# Handoff, part 2 (2026-09-22, afternoon): timed skidpad, ride-height aero, drivable skidpad, log fits

Continues `HANDOFF-physics-2026-09-22.md` on the same branch, `fix/physics-review-0922`. Nick's instruction for this session: **no setup number changes**. Every setup value in the sim (mass, brake split, RSD 0.48, toe, preload, pressures, gradients) is as it was. The bicycle is still frozen, and `bicycle_frozen.rs` passes.

## Commits

| Commit | What |
|---|---|
| 8080ae3 | Timed skidpad harness (FSAE D.10), both models (handoff Next #1) |
| ecb787a | 4-wheel β: aero follows ride height, from the 2026 CFD map (Next #4) |
| a91f1e4 | Drivable skidpad course in the game, scored per D.10.4.1 |
| (this) | Log-fit analysis (Next #2), accel launch analysis, gap-hunt review, 4-wheel anti geometry fix, stale docs, this note |

## 1. Timed skidpad (`tests/common/skidpad.rs`, `tests/timed_skidpad.rs`, `examples/timed_skidpad.rs`)

This is a robot driver that runs the real course. It enters perpendicular, runs the right circle twice, then the left circle twice, and exits. Laps are timed at the start/stop line: lap 2 (right) and lap 4 (left), averaged.

- **"Clean"** means no tyre edge crosses an inner circle and no tyre leaves both lanes. This is judged at the four contact patches, padded by half a tyre width.
- **The search** covers the line radius, an optional exit opening ("throttle up at the exit"), and speed.

Results on the default (as-run) setup:

| | Lane centre (9.125 m) | Tightest clean line |
|---|---|---|
| Bicycle | 5.352 s | **5.143 s** on 8.57 m |
| 4-wheel β | 5.513 s | **5.322 s** on 8.62 m (after the aero commit; 5.344 before) |

- The tight line is worth about 4%. Opening the line and throttling up before the timing line bought nothing in either model.
- The steady-state validation number (5.189 s) is optimistic by about 2%. Its "held" criterion accepts a mean radius up to 4% off 9.125 m.
- The real best is 5.01 s. The gap is therefore about 2.7% for the bicycle and 6% for the 4-wheel model on the as-run setup.
- The robot's line is limited by its tracking accuracy (±5–10 cm) and by rear off-tracking. Tyres touch the cones at an 8.32 m line; the robot holds 8.57–8.62 m. A real driver may run closer.

## 2. Aero follows ride height (4-wheel only; `AeroRideMap` in vehicle.rs)

**Source.** The 2026 'Ride Height Data (BW)' CFD map (Drive: Aero/Aero Map/Ride Height). It is a 5 × 5 grid of front and rear ride height, each −1 to +1 in. The grid is taken from `sdm26_team_data.json` in the AC repo.

**The fit.** The grid is CFD-noisy, so it is reduced to a least-squares plane through 24 of the 25 cells. The front −1 / rear +1 corner has the wing in the ground and is flagged bad in the sheet. The fit script is `sim/tools/aero_map_fit.py`.

| Force | Plane (lbf at 15.65 m/s; ride heights in inches) | rms |
|---|---|---|
| Front DF | 52.13 − 9.87 dRH_f + 5.69 dRH_r | 5.0 |
| Rear DF | 49.06 + 1.05 dRH_f − 0.10 dRH_r | 2.8 |
| Drag | 41.86 + 0.30 dRH_f + 0.55 dRH_r | 1.0 |

**How it's applied.** The map acts as a multiplier that is exactly 1 at static ride height. So ClA 3.132, CdA 1.267 and 52.4% front are untouched. The map only says how they move.

**Ride height.** Each axle squats on its ride rate (spring in series with the tyre, 0.7 damping) under its own downforce, and the body's pitch adds to that. The springs' share of the squat also feeds bump camber.

**Effect:**
- At 25 m/s the car sits about 10 mm down at the front and 9 mm at the rear, with a front share of 0.535.
- Under 1.4 g braking the nose drops about 21 mm and the front share goes to about 0.58. That is the brake-in balance shift a fixed split can't produce.

**Telemetry:** `sim.ride_height_f_mm`, `sim.ride_height_r_mm` and `sim.aero_front_frac`.

## 3. Drivable skidpad (course id `skidpad`)

The course is `sim/src/track/skidpad.js`, built from the D.10.1 dimensions:
- 16 inner and 13 outer cones per circle;
- the outer arc stops short of the other circle's lane and of the entry/exit path;
- entry and exit gates.

Sectors are S1 staging, S2 R1, **S3 R timed**, S4 L1, **S5 L timed**, S6 exit.

Timing scores the run as (R + L) / 2 plus 0.125 s per cone. An off course, or a run without both timed laps, is a DNF. The test is `tools/test_skidpad.mjs` (in `npm test`).

**Helios.** Boards group by course id, so the skidpad gets its own board. The Helios Launch tab can't start it yet; that needs `skidpad` in `launch.rs valid_track` and in `api.ts TRACKS`, which means a Helios release.

## 4. Test logs vs the sim (Next #2): `sim/docs/analysis-2026-09-22/`

The full report is `log-fit-report.md`, with scripts in `scripts/`. Headlines:

- **Where the data is.** The chassis channels are in the MoTeC DAQ exports, `Downloads\VAULT\SDM26\MOTEC\testing_data`. The Drive "Test Log Packs" CSV and llgx files are ECU-only.
- **Roll gradient, springs only (shock pots).** Real **0.51 front / 0.64 rear deg/g** (tight confidence intervals, linear to 1.2 g, consistent across drivers). The sim's 4-wheel model gives 0.40 / 0.37.
  - The real car's roll stiffness is about 1110 N·m/deg, which is what the coil springs give with no bars. The sim uses about 1660 from the ARB calculator.
  - ARB settings for those days (4/16 and 4/19) weren't found. **The setup numbers were not changed; this needs a team decision.**
- **Motion ratios.** 1.143 front and 1.054 rear (damper travel per wheel travel, from the OptimumK hardpoints). This confirms the sim's 1.14². The AC repo's `sdm26_team_data.json` "0.88 / 0.943 spring/wheel" values are the reciprocals, so that label is backwards.
- **Understeer gradient.** It can't be pinned down: robust fits give about 0, ordinary least squares about 1.45 deg/g. The sim's 0.55–1.1 is inside the spread.
- **Yaw gyro.** It works. It logs millidegrees per second, which is why the channel review called it unusable. `GP_SPEED` reads about 1.28× true speed.

## 5. The 75 m accel gap is in the launch, not the power

**The real run.** Only one clean run was found: `SDM26 (5.3.1) Accel.csv`, an ECU log with rpm and gearbox-output speed.
- It launches at 8000 rpm. The rears spin to about 55 km/h within 0.6 s and the engine never bogs (6000–14000 rpm).
- From the wheel speed, the car is doing about 52–55 km/h by about 1.6 s.
- The 75 m time is bracketed at 4.28–4.63 s, with a trap speed of 94–99 km/h.

**The sim.** 75 m in 5.05–5.17 s at a 96–98 km/h trap, depending on launch technique.

**Power agrees.** 52 → 88 km/h takes 2.10 s on the real car and 2.06–2.23 s in the sim. The 46 kW chassis-dyno curve is not the problem.

**The launch doesn't.** The sim reaches 52 km/h at 2.0–2.5 s, against the real 1.67 s. What was tried in the 4-wheel model (not committed; the probe is in `scripts/accel_probe.rs`):

| Change | Result |
|---|---|
| Slip-ratio denominator floor 2 → 5–11 m/s | 0.05–0.2 s better |
| A sliding-friction floor on Fx past the peak: the fitted C = 1.55 / E = −0.40 shape leaves only 0.66 of peak at κ = 2–4 | Even at 0.9 of peak, 52 km/h comes at 1.93 s and 75 m at 4.73 s |
| The sim's peak traction | About 0.92 g at κ 0.14. The real launch averages about 1 g while spinning, which exceeds that peak |

- **Tyre fit.** The team's MF6.1 `.tir` doesn't support a stronger longitudinal tyre: its μx/μy is 0.46, TTC-inflated, and its FNOMIN is −6000 N.
- **Verdict.** No change was shipped. It needs a real vehicle-speed trace of a launch (working GPS, or undriven-wheel speed that doesn't saturate) and the TTC drive/brake data re-checked at high slip.
- **The over-rev is not only the harness.** The accel runs over-rev to about 14,400 and can double-shift (1→2→3 within 0.1 s). The gap-hunt traced this to the clutch never locking under hard acceleration (B2, §6), which also bogs the launch (B1). Both are in the shared powertrain, so they are reported rather than fixed.

## 6. Gap-hunt review (an independent agent, read-only; full list in `analysis-2026-09-22/gap-hunt.md`)

**Fixed here** (4-wheel only, so the bicycle is untouched):
- **B6.** Anti-squat was applied at half strength (a 50/50 average) on a rear-drive car. It now applies in full.
- **Braking anti** is now weighted by brake bias, not averaged 50/50. The pitch gradient still reproduces 0.918 deg/g.

**Fixed, docs only:**
- `paramMeta.js` no longer says there is no differential.
- The `powertrain.rs` torque-table comment says it is the dyno curve, not the CFD sweep.
- The README and validate.js explanations of the accel gap are corrected.

**NOT fixed. Each needs Nick's sign-off, because it changes the frozen bicycle or scoring:**
- **B1. Launch clutch bogs the engine.** The pull-away capacity follows driveline speed, not engine speed (`powertrain.rs` ~582, mirrored in `powertrain.js`). The crank drops to about 1100 rpm within 0.2 s of a launch, including an LC dump from 7000. A clutch that holds launch rpm is worth 0.13 s, and 0.35 s with longitudinal grip corrected.
- **B2. The clutch never locks under hard acceleration.** The stick torque ignores the tyre reaction (~739), leaving 29 rad/s of permanent slip. `can_shift()` then stays false, so auto-shift and the harnesses hold 1st to 14,300 rpm, and rpm reads 2–3 % high everywhere.
- **B3. Timing from the line.** Accel (D.9.2.3, staged 0.3 m back) and autocross should time from line crossing, not from motion. That alone is worth 0.36 s on accel. It changes every stored time, so it would need `COURSE_REVISED_AT` handling in Helios. There is also no accel event in the game.
- **B5.** With B1 + B2 + B3 fixed and mu_x 1.89 (UNVERIFIED; re-derive from raw TTC drive/brake data), the model does 4.36 s timed from the line, inside the real 4.2–4.4 s.
- **B4. The Salisbury stick band makes steady cornering step-size dependent.** At 20 m/s and 3 deg of steer, yaw rate is 29.83 deg/s at 500 Hz (tests and golden) and 29.50 at 1 kHz (rig). The validated car is not exactly the car on the rig.
- **T2. Cornering stiffness** is 23.3 kN/rad at 655 N, against 31–38 kN/rad from the MF6.1.2 fit. The README claims the opposite.
- **T1 / S1–S3. front_grip_factor** may be standing in for steering compliance, bump steer and roll steer, none of which are modelled. Front toe-in of 1.1 deg per wheel is large; check its units against the setup sheets.
- **S4.** The 4-wheel setup (toe, camber, Ackermann, gradients, damping, anti, aero map) can't be edited from the app, the `.hset` files or the rig param set.
- **B7.** The braking test is a four-wheel lockup (21.4 m threshold vs 23.0 m locked).
- **Aero / heave:**
  - Roll and sideslip don't feed the aero map.
  - The aero squat (wheel rates) and the pitch (from the gradient) take their stiffness from different sources.
  - A two-state front/rear heave body replacing both would reconcile them.
  - Whether the 0.91 deg/g pitch gradient already includes the anti geometry is UNVERIFIED.
- **Other:**
  - No throttle/manifold lag, fuel model, brake lag or fade.
  - Camber can't raise peak grip, so static-camber studies read about zero.
  - No tyre temperature or wear.
  - Endurance has no driver change.
  - CI never runs the tests.

## 7. Still open, in priority order

1. **Decide on B1/B2/B3** (§6). They close most of the accel gap but change the frozen bicycle, and B3 changes every stored time.
2. **Roll stiffness.** Is the ARB calculator's 1660 N·m/deg what was actually on the car? The logs say 1110. This is a team decision; setup numbers are unchanged.
3. **The launch** (§5): get a speed trace, and re-derive mu_x from raw TTC data.
4. **Per-event setup presets** (handoff Next #3): not done, because Nick said the current setup numbers are correct.
5. **Skidpad in the Helios Launch tab** (§3).
6. **Pull fresh DAQ logs** of a constant-radius test (handoff Next #5). A real timed skidpad from the DAQ, with the yaw gyro that works, would validate §1 directly.
