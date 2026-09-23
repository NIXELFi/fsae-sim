# Releasing a physics change (leaderboard eras)

Every run records the **physics revision** of the vehicle model it was driven on (`stats.physicsRev`). Helios ranks each course × model × revision separately. The newest revision is shown by default, and older ones stay browsable as eras. A physics change never wipes a board or mixes two different cars on one.

**Eras are decided update by update. Nothing creates one automatically.**

`native/crates/sim-core/tests/physics_rev.rs` fingerprints each model:
- the bicycle by its golden drive;
- the 4-wheel β by steer-ramp peaks, 75 m and a stop, all quantised.

If a change moves a fingerprint, that test fails and you choose:

| The change… | Do this |
|---|---|
| Moves lap times (you'd want a fresh board) | Bump that model's revision in `src/physics_rev.rs` **and** `sim/src/vehicle/physicsRev.js`, then re-record the fingerprint in the test. |
| Doesn't meaningfully move lap times | Re-record the fingerprint only, and say in the commit why times are unaffected. The board carries on. |

Most releases don't touch physics and never see the question.

History:

| Model | Revision | Sims | Notes |
|---|---|---|---|
| Bicycle | 1 | up to now | Includes the 0.7.0 launch-clutch and 0.7.1 tyre fixes. The team chose to keep the bicycle board's history across those. |
| 4-wheel β | 1 | 0.6.13 – 0.7.2 | About 8 % grippier than the bicycle. |
| 4-wheel β | 2 | 0.7.3 → | Grip pinned to the bicycle's steady-state limit. |

Runs from before 0.7.4 don't carry the stamp. Helios places them by simulator version (`physicsEraOf`).
