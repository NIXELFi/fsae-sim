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
//! 5. **Orient it**: +X forward, +Y up, +Z to the left, origin at the centre of
//!    the front axle line projected to the ground. This matches the vehicle
//!    model's convention, so a correctly oriented export needs no fudge factors
//!    and a wrongly oriented one is obvious immediately.
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
    pub loaded: bool,
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
    let path = std::path::Path::new("assets").join(CAR_MODEL);
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

/// Hide the procedural body once CAD bodywork is in place.
///
/// Hidden rather than despawned so that a failed or half-finished export can be
/// compared against the known-good procedural geometry by toggling one flag,
/// which is exactly what you want while getting an export right.
pub fn hide_procedural(visibility: &mut Visibility) {
    *visibility = Visibility::Hidden;
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
