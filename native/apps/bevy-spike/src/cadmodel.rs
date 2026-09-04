//! Optional CAD bodywork, loaded from glTF.
//!
//! If `assets/car.glb` exists it is spawned under the car root and the
//! procedural body is hidden. If it does not, nothing happens and the
//! procedural body stays. That asymmetry is deliberate: the simulator has to
//! keep working for anyone without the CAD, and a missing asset should not be
//! an error.
//!
//! # Getting a SolidWorks assembly in here
//!
//! glTF 2.0 (`.glb`) is the target format. Bevy loads it natively, it carries
//! materials and a node hierarchy, and it is the one interchange format that is
//! actually well specified. SolidWorks will not write it directly, so:
//!
//! 1. **SolidWorks → STEP AP214.** Not STL. STL is triangles with no structure:
//!    no part names, no materials, no hierarchy, so there is no way to find the
//!    front wheels afterwards in order to steer them.
//! 2. **STEP → Blender**, via the free `STEPper` importer add-on, or through
//!    FreeCAD (import STEP, export glTF). Tessellate at a visual tolerance --
//!    0.1 mm chord height is CAD-accurate and produces a model far too heavy to
//!    render; 1-2 mm is invisible at cockpit distance.
//! 3. **Decimate.** A full assembly is often several million triangles, most of
//!    them inside the car where nobody will ever see them. Target 150k or fewer
//!    for the whole visible body. Delete internal parts entirely rather than
//!    decimating them.
//! 4. **Name the nodes** listed in [`NODES`] below. This is the only step that
//!    cannot be skipped: the wheels have to be separate nodes with known names
//!    or they cannot rotate or steer, and the steering wheel has to be separate
//!    or it cannot turn.
//! 5. **Orient it**: +X forward, +Y up, **+Z to the RIGHT**, and put the origin at
//!    the **centre of gravity projected onto the ground** -- X = 0 at the CG,
//!    Y = 0 at the ground plane. For SDM26 that puts the front axle at
//!    x = +0.788 and the rear at x = -0.742. This matches what both builds
//!    already use for the car root, so the model rotates about the same point
//!    the physics yaws about. Building it about the front axle instead is the
//!    mistake with the least visible symptom: it looks fine standing still and
//!    pivots about the wrong place the moment the car turns.
//!
//!    +Z right is what `carmesh.js` uses -- it puts FL at z = -track/2 -- and it
//!    is the only choice that makes the triad right-handed, since
//!    forward x up = right. Backwards mirrors the car, which on a symmetric
//!    model is invisible.
//!
//!    Run `node ../../../sim/tools/check_car_glb.mjs your.glb` before trusting
//!    an export.
//!
//!    NOTE: unlike the WebGL build, this one does NOT solve the frame from the
//!    wheel hubs -- it uses the glTF node transforms as they are, so a model for
//!    Bevy has to be in the frame described above. Porting the fit from
//!    `glbcar.js` is worthwhile and has not been done.
//! 6. **Export .glb** (binary glTF, textures embedded) to `assets/car.glb`.
//!
//! Scale is metres. A model exported in millimetres arrives a thousand times
//! too big, which is the single most common mistake and the easiest to spot.

use bevy::prelude::*;

/// Node names the simulator looks for inside the glTF.
///
/// Anything not found is simply not animated -- a model with only a body still
/// loads and looks right standing still. The wheels are the ones worth getting
/// right.
pub const NODES: &[(&str, &str)] = &[
    ("body", "chassis, bodywork, nose, sidepods -- everything static"),
    ("wheel_fl", "front left wheel and tyre, origin at the hub centre, axis +Z"),
    ("wheel_fr", "front right wheel and tyre, origin at the hub centre, axis +Z"),
    ("wheel_rl", "rear left wheel and tyre, origin at the hub centre, axis +Z"),
    ("wheel_rr", "rear right wheel and tyre, origin at the hub centre, axis +Z"),
    ("steering_wheel", "rim and hub, origin on the column axis"),
];

/// Path, relative to `assets/`, of the optional CAD bodywork.
pub const CAR_MODEL: &str = "car.glb";

#[derive(Resource, Default)]
pub struct CadModel {
    /// The scene was found on disk and spawning was requested.
    pub loaded: bool,
    /// Its named nodes have been found and hooked up to the animation.
    pub wired: bool,
}

/// Where Bevy is told to look for assets, and where this module checks.
///
/// Computed rather than assumed so that the existence check and the asset
/// server cannot disagree -- the failure mode when they do is that the file is
/// found, the scene silently never loads, and nothing says why. Next to the
/// executable first, which is how a shipped build finds it; then the crate's
/// own assets directory, which is how `cargo run` does.
pub fn asset_root() -> std::path::PathBuf {
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            let p = dir.join("assets");
            if p.is_dir() {
                return p;
            }
        }
    }
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets")
}

/// Marker for anything spawned from the CAD model.
#[derive(Component)]
pub struct CadBody;

/// Spawn the CAD model if the file is there.
///
/// The existence check is done against the filesystem rather than by asking the
/// asset server, because Bevy's loader reports a missing asset asynchronously
/// and treats it as an error worth logging loudly. Here a missing file is the
/// normal case, not a fault.
pub fn spawn_if_present(
    commands: &mut Commands,
    assets: &AssetServer,
    parent: Entity,
    state: &mut CadModel,
) {
    let path = asset_root().join(CAR_MODEL);
    if !path.exists() {
        info!(
            "no {} — using the procedural body. See cadmodel.rs for the \
             SolidWorks export recipe.",
            path.display()
        );
        return;
    }

    // `GltfAssetLabel::Scene(0)` is the file's default scene, which is what a
    // single-model export produces. The label is not optional -- without it
    // Bevy does not know which part of the glTF to load.
    //
    // The component is `WorldAssetRoot`, not the `SceneRoot` that every
    // pre-0.19 example uses: 0.19 reworked scenes around a new `Scene` trait
    // and `SceneRoot` no longer exists. This is the same class of API drift
    // already recorded for `GlobalAmbientLight` and `Hdr`.
    let scene = assets.load(GltfAssetLabel::Scene(0).from_asset(CAR_MODEL));
    commands.entity(parent).with_children(|p| {
        p.spawn((WorldAssetRoot(scene), CadBody, Transform::IDENTITY));
    });
    state.loaded = true;
    info!("loaded CAD bodywork from {}", path.display());
}

/// Which simulator part a glTF node name corresponds to.
pub fn role_of(name: &str) -> Option<Role> {
    match name {
        "wheel_fl" | "wheel_fr" => Some(Role::Wheel { front: true }),
        "wheel_rl" | "wheel_rr" => Some(Role::Wheel { front: false }),
        "steering_wheel" => Some(Role::SteeringWheel),
        _ => None,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Wheel { front: bool },
    SteeringWheel,
}

#[cfg(test)]
mod role_tests {
    use super::*;

    #[test]
    fn wheel_names_map_to_the_right_axle() {
        assert_eq!(role_of("wheel_fl"), Some(Role::Wheel { front: true }));
        assert_eq!(role_of("wheel_rr"), Some(Role::Wheel { front: false }));
        assert_eq!(role_of("steering_wheel"), Some(Role::SteeringWheel));
        assert_eq!(role_of("body"), None);
        assert_eq!(role_of("Wheel_FL"), None, "matching is case sensitive");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_documented_node_has_a_description() {
        for (name, desc) in NODES {
            assert!(!name.is_empty());
            assert!(desc.len() > 10, "{name} needs a usable description");
        }
    }

    #[test]
    fn wheel_nodes_cover_all_four_corners() {
        let names: Vec<&str> = NODES.iter().map(|(n, _)| *n).collect();
        for corner in ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"] {
            assert!(names.contains(&corner), "missing {corner}");
        }
    }
}
