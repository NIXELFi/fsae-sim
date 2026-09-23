# Gap hunt: what fsae-sim is missing (2026-09-22)

This was an independent, read-only review of `fix/physics-review-0922` at about a91f1e4. The reviewer's probe programs are not in the repo. Everything here is confirmed unless it is marked UNVERIFIED. What was acted on is in `../HANDOFF-physics-2026-09-22-b.md` §6.

## Bugs, ranked

1. **B1: the launch clutch bogs the engine.**
   - **Where:** the pull-away clutch capacity follows driveline speed, not engine speed (`powertrain.rs` ~582-593, mirrored in `powertrain.js`).
   - **Effect:** the crank drops from 2012 to 1129 rpm by 0.2 s and stays under 5000 rpm until about 1.1 s. The car pulls 0.34-0.7 g with almost no wheelspin (κ 0.03-0.05). A launch-control dump drops it from 7000 to about 1100 rpm in 0.2 s.
   - **Worth:** a clutch that holds the launch rpm gains 0.13 s alone, and 0.35 s with longitudinal grip corrected.
2. **B3: timing starts from rest, not at the line.**
   - **The rules:** accel D.9.2.3 stages the car 0.30 m behind the line, and autocross D.11.2.3 stages it "a specific distance" behind; timing starts at the line.
   - **The sim:** the harnesses time from standstill (`validation.rs` 113-131, `validate.js` 146-194). The game starts the clock at 0.6 m/s (`timing.js` MOVING_MPS).
   - **Worth:** timed from 0.3 m, the stock accel is 4.70 s instead of 5.06 s. There is also no accel event in the game.
3. **B2: the clutch never locks under hard acceleration.**
   - **Where:** the stick torque ignores the tyre's reaction torque (`powertrain.rs` ~739). That leaves about 29 rad/s of slip, above the 8 rad/s lock threshold (~706).
   - **Effect:** `can_shift()` stays false, so auto-shift and the harnesses hold 1st to 14,271 rpm instead of 12,830 and effectively skip 2nd. Engine rpm reads 2-3 % high in the HUD, audio and log. Manual paddles are unaffected.
4. **B5: the documented accel-gap cause was wrong** (the kDen floor). B1 hid the tyre.
   - **Evidence:** with B1 fixed, μx 1.5 → 1.8 is worth 0.18 s. With a good launch, fixed shifting and μx 1.89, the car does 4.69 s standing and 4.36 s timed from the line.
   - **UNVERIFIED:** μx itself. The MF6.1.2 fit is unstable on pressure (μx/μy comes out 1.01 or 0.44), so re-derive it from raw TTC data.
5. **B4: the diff's stick band makes steady cornering step-size dependent.**
   - **Effect:** steady yaw rate at 20 m/s and 3 deg of steer:

     | Model | 500 Hz | 1 kHz | 4 kHz (converged) |
     |---|---|---|---|
     | Bicycle (deg/s) | 29.83 | 29.50 | 29.24 |
     | 4-wheel (deg/s) | 30.50 | 30.13 | 29.71 |

     An open diff, or a 1.0 rad/s stick band, gives the same answer at every step.
   - **Consequence:** the validated car (500 Hz) is not the rig's car (1 kHz).
   - **Where:** `bicycle.rs` 515-516.
6. **T2: cornering stiffness is 23.3 kN/rad (407 N/deg) at 655 N.** `mf_eval.py` on the MF6.1.2 fit gives 31-38 kN/rad. Grip was scaled down while the 7.3 deg peak was kept, which dragged the stiffness down with it. The README claims the reverse.
7. **T1 / S1-S3: `front_grip_factor` may cover for unmodelled physics.** It is 0.80, or 0.856 on the 4-wheel model. Candidates:
   - steering compliance (road angle is a pure function of rim angle);
   - bump steer and roll steer;
   - toe: 1.1 deg per wheel at the front is large (UNVERIFIED units). It puts the two fronts at 0.80 vs 3.05 deg of slip at 0.85 g. Skidpad goes from 5.309 to 5.236 s with zero toe.
8. **S4: the 4-wheel setup can't be edited from the app.** Toe, camber, Ackermann, gradients, damping, anti geometry and the aero map aren't in the rig param set, `params.js` or `.hset` files.
9. **B6: 4-wheel anti-squat was halved,** and braking anti was averaged 50/50 instead of weighted by brake bias. **FIXED** in this branch.
10. **B7: the braking test is a four-wheel lockup.** Full pedal locks every wheel within about 0.3 s. Threshold braking stops in 21.4 m, against 23.0 m locked.
11. **B8: stale docs.** `paramMeta.js` said there was no differential. The README validation and limitations were out of date. The `powertrain.rs` table comment said CFD sweep. **FIXED** in this branch.

## Other gaps

- **Engine:** no electronic throttle or manifold lag, and no fuel model, so the efficiency event can't be scored. Part-throttle torque is linear in throttle-plate position.
- **Brakes:** no hydraulic lag or fade.
- **Tyre:**
  - camber can't raise peak grip, because it is a slip shift only (the fit has PVY3/4);
  - no temperature, pressure or wear;
  - relaxation length is fixed at 0.35 m;
  - no turn slip.
- **Aero:** no sensitivity to sideslip or roll. The ride-height map is a plane, clamped at ±1 in.
- **Heave:**
  - The aero squat (wheel rates) and the pitch (from the gradient) take their stiffness from different sources; a two-state front/rear heave body would reconcile them.
  - UNVERIFIED: whether the 0.91 deg/g pitch gradient already includes the anti geometry. If it doesn't, pitch is about 14 % too large.
- **Events:**
  - no driver change in endurance;
  - UNVERIFIED: whether Helios filters out laps driven with TC/ABS on.
- **Rig:** UNVERIFIED: a missing `lagS` in a wheel profile may fall back to a 60 ms default.
- **Repo:** CI builds only on tags and never runs the tests.

## Checked and found correct in `double_track.rs`

- **Frames:** sign conventions, and patch velocities in each wheel frame.
- **Load transfer:** the lateral and longitudinal sums match m·h·a.
- **Vertical loads:** they sum to weight plus downforce within 0.1 N.
- **Low speed:** no creep on the brakes and no low-speed ringing.
- **Steady cornering:** within 1 % of the bicycle.
