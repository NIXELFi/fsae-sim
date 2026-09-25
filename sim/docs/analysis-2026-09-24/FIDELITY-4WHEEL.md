# 4-wheel model fidelity pass, 2026-09-24

Branch `feat/physics-fidelity-0924`. Not released.

The bicycle model is frozen and did not move. Proof: `bicycle_frozen.rs`, the bicycle fingerprint in `physics_rev.rs`, and `validate.js`'s bit-for-bit JS parity all pass unchanged.

The scorecard is `cargo run --release -p sim-core --example reality [-- skidpad]`. The real-trace replay (the car's own steering through the model, compared with its gyro, ECU accelerometer and shock pots) is the audit harness `replay` plus `compare2.py`.

## The car the logs describe had no anti-roll bars

- **The logs:** the shock pots give about 1110 N·m/deg of suspension roll stiffness. That is the coil springs alone (the Chassis Design Binder gives 1122.3).
- **The skidpad:** the 3/21 slides say the 5.01 s skidpad was run with "no arbs".
- **Matching the model to that car:** set RSD 0.516 and roll gradient 0.86 deg/g (`reality.rs` `springs_only`). The 4-wheel model then rolls about 0.57 deg/g at the springs, against the pots' 0.51 front / 0.64 rear (mean 0.57).
- **Result:** the "rolls 25–45 % too little" finding was the bar setting, not the physics.

## What changed (4-wheel only)

1. **The tyres' aligning moment now reaches yaw.**
   - Each tyre's Fy acts a pneumatic trail behind its patch, so the yaw equation gets a pure −Fy·t at all four wheels.
   - Before, it was computed for the steering only.
   - Effect: +0.2 deg/g understeer gradient and about 3 % off the peak.
2. **Front grip re-pinned from 1.06 to 1.08,** back to this model's own steer-ramp peaks.

   | Steer-ramp peak (g) | Before | After |
   |---|---|---|
   | 10 m/s | 1.441 | 1.437 |
   | 15 m/s | 1.608 | 1.598 |
   | 20 m/s | 1.832 | 1.816 |

   The front fudge shrinks from 0.848 to 0.864 of the rear.
3. **Full friction through the ignition cut** (`GearedEngine::full_cut_drag`). It was half before.
   - The 5/3 ECU logs show the crank falling at 10,000–14,300 rpm/s.
   - The model now falls at 11,100–11,700 rpm/s; before, it fell at about 6000.
4. **Crank momentum is handed over at engagement instead of deleted** (`GearedEngine::conserve_engagement`).
   - It is handed over only on the step that engages. Once locked, the crank inertia rides on the wheel.
   - The standing 75 m does not depend on the step size (4.772 s at 500 Hz, 4.768 s at 2 kHz).
5. **Telemetry fixes, 4-wheel only** (the bicycle's recorded channels are unchanged):
   - `sim.slip_front/rear`, `util_*` and `kappa_*` are now each axle's two wheels, load-weighted. They were the left wheel only.
   - The wheel-speed channels now carry each of the four wheels.

Physics revision: 4-wheel 3 → 4 (a new leaderboard era). The unmerged `feat/throttle-body-model` branch also bumps it to 4, so reconcile the two at merge.

## 4-wheel against the car

The reference is the springs-only car. "Before" is 0.7.15.

| | Before | After | Real |
|---|---|---|---|
| Yaw gain, replaying real steering, 0.3–0.5 Hz (deg/s per deg) | 4.90–4.94 | 4.82–4.86 | 4.60–4.99 |
| Suspension roll gain at 0.3 Hz | 0.047 | 0.047 | 0.046 |
| Crank fall in the cut (rpm/s) | ~6000 | 11,100–11,700 | 10,000–14,300 |
| Meets the new gear, with the logged 185 ms cut | +14 % | +3.7 % | −1..+4 % |
| Understeer gradient @15 m/s (deg/g) | 0.50 | 0.59 | the logs can't pin it (−0.5..+2) |
| Pitch in braking, springs share (deg/g) | ~0.34 | ~0.34 | 0.38 |

## Open, needs the owner or the car

- **Cut length.** The sim uses 90 ms (Nick: "80–100 ms off the paddle"). The 5/3 logs show 175–200 ms plus 50 ms from switch to cut. With full friction, only the logged length meets the new gear near sync (90 ms lands 14 % high). The shifter may have been retuned since May.
- **Bars or no bars at competition.** Default setups run the bars (RSD 0.48, 0.66 deg/g); every logged day ran without them.
- **Launch traction.** The 0–20 m estimate from the logs (1.70–1.85 s) depends on an assumed accelerometer cap. Only driven-wheel speed was logged. With a 1.0 g cap the logs give 1.96 s, and the sim gives 2.01 s. Not changed. A GPS or true-speed accel run would settle it.
- **Response timing.** Steer to roll is 61–69 ms on the car (analog to analog) against about 43 ms in the replay. The car's yaw, ay and roll gains also fall 25–45 % by 1 Hz, and the model's don't. No physical parameter explains both. The likely causes are driver-in-the-loop bias in the estimate and logger filtering. A step-steer test on the car would settle it.
