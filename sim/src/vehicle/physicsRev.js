// Physics revisions, by vehicle model -- the leaderboard era a lap is on.
//
// Mirrors native/crates/sim-core/src/physics_rev.rs, which explains the rule
// and keeps the history; tests/physics_rev.rs fails if the two disagree, or if
// a model's physics moved without a decision about its revision. Every run
// records `stats.physicsRev`, and Helios ranks course x model x revision
// separately, keeping older revisions as eras instead of wiping them.

export const PHYSICS_REV = {
  2: 1, // bicycle
  3: 4, // 4-wheel beta (4: aligning moment in yaw, shift fixes; see physics_rev.rs)
};

/** The revision for a vehicleModel number (2 bicycle, 3 double track). */
export function physicsRevFor(vehicleModel) {
  return PHYSICS_REV[vehicleModel === 3 ? 3 : 2];
}
