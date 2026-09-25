//! Physics revisions: which leaderboard "era" a lap belongs to.
//!
//! Every run records the physics revision of the vehicle model it was driven
//! on (`stats.physicsRev`, from the JS mirror `sim/src/vehicle/physicsRev.js`).
//! Helios ranks each course x model x revision separately, and keeps older
//! revisions as browsable eras -- so a physics change that moves lap times
//! starts a NEW board instead of wiping the old one or, worse, mixing times
//! from two different cars on one board.
//!
//! The rule, enforced by `tests/physics_rev.rs`:
//!
//! * Each model has a FINGERPRINT: the bicycle's is its golden drive
//!   (`sim/data/vehicle-golden.json`) PLUS a handful of limit numbers
//!   (steer-ramp peaks, a standing 75 m, a stop from 25 m/s, compared within
//!   a driver's tolerance) -- the golden drive is gentle by design and
//!   cannot see a launch or a grip change; the double track's is the same
//!   limit numbers. The double track also has a golden drive of its own
//!   (`tests/double_track_frozen.rs`) that catches any change at all.
//! * If a change moves a fingerprint, the test fails. Then decide:
//!   - lap times move (a real physics change): BUMP that model's revision here
//!     AND in physicsRev.js, and re-record the fingerprint -- a new era;
//!   - they do not (a refactor, a rounding): re-record the fingerprint only,
//!     and say in the commit why times are unaffected.
//!
//! Revision history:
//!   bicycle      1  everything up to and including simulator 0.7.3, and
//!                   the 2026-09-23 physics-review fixes (no phantom wheel
//!                   inertia, step-independent diff; standing 75 m 4.86 ->
//!                   4.75 s) -- kept in era 1 by owner decision
//!   double track 1  0.6.13 .. 0.7.2 (beta; ~8 % grippier than the bicycle)
//!                2  0.7.3 .. 0.7.8 (grip pinned to the bicycle's steady limit;
//!                   still +2.8 % peak at skidpad speed, ~0.1 s round it)
//!                3  front grip 1.08 -> 1.06: the bicycle's steer-ramp peak
//!                   at skidpad speed
//!                4  2026-09-24 fidelity pass: the tyres' aligning moment in
//!                   the yaw equation (+0.2 deg/g understeer), front grip
//!                   re-pinned 1.06 -> 1.08, full friction through the
//!                   ignition cut and a lock that keeps the crank's momentum

/// Revision of the bicycle's physics.
pub const PHYSICS_REV_BICYCLE: u32 = 1;
/// Revision of the double track's physics.
pub const PHYSICS_REV_DOUBLE_TRACK: u32 = 4;

/// The revision for a `vehicleModel` number (2 bicycle, 3 double track).
pub fn physics_rev(vehicle_model: u8) -> u32 {
    if vehicle_model == 3 { PHYSICS_REV_DOUBLE_TRACK } else { PHYSICS_REV_BICYCLE }
}

/// FNV-1a over a string: a stable, dependency-free fingerprint.
pub fn fnv1a(s: &str) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}
