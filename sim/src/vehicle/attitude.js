// Body attitude: the ONE place the physics core's sign convention is turned
// into the one everything on this side uses.
//
// The convention on this side -- the JS bicycle, the run log
// (`sim.roll_deg`, `sim.pitch_deg`), replay, the renderer and the
// suspension rig -- is:
//
//   roll   positive in a LEFT turn (the car leans onto its right springs)
//   pitch  positive NOSE UP: braking dives, so braking is NEGATIVE
//
// sim-core (Rust) documents and emits (solver/mod.rs `Telemetry`):
//
//   roll   positive right side down: a left turn -- the same
//   pitch  positive NOSE DOWN -- the opposite
//
// Until 0.7.7 the native 4-wheel car's pitch went into the log unconverted,
// so every 4-wheel run logged its pitch backwards and replayed the car
// squatting under braking (Replay.pitchSign undoes that for old runs).
// tools/test_attitude.mjs pins every link of this chain: change a sign
// anywhere and it fails.

/** sim-core attitude (deg) -> this side's convention (deg). */
export function attitudeFromNative(rollDeg, pitchDeg) {
  return { rollDeg, pitchDeg: -pitchDeg };
}
