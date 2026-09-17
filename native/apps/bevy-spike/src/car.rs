//! SDM26 geometry, composed from Bevy primitives.
//!
//! Deliberately not a ported mesh builder. Bevy gives each part its own PBR
//! material, and metallic/roughness per component is most of what makes this
//! look different from the flat-shaded WebGL build: the carbon reads as carbon,
//! the wheel rims catch a highlight, the tyres stay matte.
//!
//! Real SDM26 dimensions throughout -- front axle at +0.788 m, rear at -0.742 m,
//! 1.207/1.194 m tracks, 0.20 m loaded radius.
//!
//! Every mesh handle is created up front, before anything is spawned. That is
//! not style: `Commands` and `Assets<Mesh>` are both borrowed mutably, so a
//! spawn helper that closes over one cannot coexist with `meshes.add` calls.

use bevy::prelude::*;

pub const FRONT_AXLE: f32 = 0.788;
pub const REAR_AXLE: f32 = -0.742;
pub const TRACK_F: f32 = 1.207;
pub const TRACK_R: f32 = 1.194;
pub const TYRE_R: f32 = 0.20;
pub const TYRE_W: f32 = 0.19;

/// Marks the car root; the solver drives its Transform.
#[derive(Component)]
pub struct CarRoot;

/// A road wheel. `front` wheels steer, all of them spin.
#[derive(Component)]
pub struct RoadWheel {
    pub front: bool,
}

/// The steering wheel, rotated about its column.
#[derive(Component)]
pub struct SteeringWheel;

/// Where the driver's eye sits, in the chassis frame.
pub const EYE: Vec3 = Vec3::new(-0.15, 0.66, 0.0);

pub struct Materials {
    pub maroon: Handle<StandardMaterial>,
    pub maroon_dark: Handle<StandardMaterial>,
    pub carbon: Handle<StandardMaterial>,
    pub carbon_light: Handle<StandardMaterial>,
    pub gold: Handle<StandardMaterial>,
    pub metal: Handle<StandardMaterial>,
    pub tyre: Handle<StandardMaterial>,
    pub rim: Handle<StandardMaterial>,
    pub cone: Handle<StandardMaterial>,
    pub cone_band: Handle<StandardMaterial>,
    pub emblem: Handle<StandardMaterial>,
    pub amber: Handle<StandardMaterial>,
}

impl Materials {
    pub fn new(m: &mut Assets<StandardMaterial>) -> Self {
        let pbr = |base: Color, metallic: f32, roughness: f32| StandardMaterial {
            base_color: base,
            metallic,
            perceptual_roughness: roughness,
            ..default()
        };
        Self {
            maroon: m.add(pbr(Color::srgb(0.42, 0.07, 0.18), 0.15, 0.35)),
            maroon_dark: m.add(pbr(Color::srgb(0.28, 0.05, 0.12), 0.15, 0.45)),
            carbon: m.add(pbr(Color::srgb(0.035, 0.037, 0.042), 0.35, 0.42)),
            carbon_light: m.add(pbr(Color::srgb(0.07, 0.072, 0.078), 0.35, 0.38)),
            gold: m.add(pbr(Color::srgb(0.85, 0.60, 0.06), 0.85, 0.28)),
            metal: m.add(pbr(Color::srgb(0.42, 0.44, 0.48), 0.90, 0.30)),
            tyre: m.add(pbr(Color::srgb(0.022, 0.023, 0.026), 0.0, 0.92)),
            rim: m.add(pbr(Color::srgb(0.55, 0.57, 0.60), 0.92, 0.22)),
            cone: m.add(pbr(Color::srgb(0.85, 0.20, 0.02), 0.0, 0.72)),
            cone_band: m.add(pbr(Color::srgb(0.86, 0.86, 0.84), 0.0, 0.65)),
            emblem: m.add(pbr(Color::srgb(0.80, 0.81, 0.82), 0.3, 0.35)),
            amber: m.add(pbr(Color::srgb(0.85, 0.45, 0.04), 0.2, 0.4)),
        }
    }
}

/// One spawnable piece: mesh, material, placement.
struct Piece(Handle<Mesh>, Handle<StandardMaterial>, Transform);

pub fn spawn_car(commands: &mut Commands, meshes: &mut Assets<Mesh>, m: &Materials) -> Entity {
    let lay = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let quarter = std::f32::consts::FRAC_PI_2;

    // ---------------- every mesh handle, before any spawning ----------------
    let mut cub = |x: f32, y: f32, z: f32| meshes.add(Cuboid::new(x, y, z));
    let floor = cub(1.45, 0.05, 0.62);
    let tub_side = cub(1.45, 0.30, 0.045);
    let tub_pad = cub(1.45, 0.028, 0.052);
    let tub_stripe = cub(1.30, 0.045, 0.010);
    let nose_a = cub(0.30, 0.34, 0.56);
    let nose_b = cub(0.30, 0.25, 0.40);
    let nose_c = cub(0.24, 0.13, 0.20);
    let nose_stripe = cub(0.62, 0.02, 0.075);
    let engine_cover = cub(0.50, 0.46, 0.58);
    let pod = cub(0.70, 0.26, 0.22);
    let fw_main = cub(0.20, 0.022, 1.06);
    let fw_flap = cub(0.13, 0.020, 1.00);
    let fw_plate = cub(0.30, 0.20, 0.016);
    let rw_pylon = cub(0.05, 0.55, 0.030);
    let rw_plate = cub(0.36, 0.24, 0.016);
    let rw_main = cub(0.26, 0.024, 1.00);
    let rw_flap = cub(0.17, 0.022, 1.00);
    let dash = cub(0.16, 0.035, 0.54);
    let sw_top = cub(0.009, 0.036, 0.180);
    let sw_bottom = cub(0.009, 0.028, 0.180);
    let sw_centre = cub(0.009, 0.084, 0.054);
    let sw_rail = cub(0.009, 0.084, 0.024);
    let sw_paddle = cub(0.006, 0.024, 0.010);
    let sw_emblem = cub(0.004, 0.032, 0.030);
    let spoke = cub(0.022, TYRE_W * 1.03, 0.19);

    let hoop_main = meshes.add(Cylinder::new(0.024, 1.0));
    let hoop_front = meshes.add(Cylinder::new(0.020, 1.0));
    let link = meshes.add(Cylinder::new(0.013, 1.0));
    let upright = meshes.add(Cylinder::new(0.017, 0.17));
    let tyre_mesh = meshes.add(Cylinder::new(TYRE_R, TYRE_W));
    let rim_mesh = meshes.add(Cylinder::new(0.127, TYRE_W * 1.02));
    let sw_grip = meshes.add(Cylinder::new(0.019, 0.110));
    let sw_button = meshes.add(Cylinder::new(0.011, 0.012));
    let sw_rotary = meshes.add(Cylinder::new(0.012, 0.016));

    // ---------------- body pieces ----------------
    let mut body: Vec<Piece> = vec![
        Piece(floor, m.carbon.clone(), Transform::from_xyz(-0.15, 0.045, 0.0)),
        Piece(nose_a, m.maroon.clone(), Transform::from_xyz(0.68, 0.235, 0.0)),
        Piece(nose_b, m.maroon.clone(), Transform::from_xyz(0.97, 0.185, 0.0)),
        Piece(nose_c, m.maroon.clone(), Transform::from_xyz(1.16, 0.115, 0.0)),
        Piece(nose_stripe, m.gold.clone(), Transform::from_xyz(0.92, 0.315, 0.0)),
        Piece(engine_cover, m.maroon_dark.clone(), Transform::from_xyz(-0.80, 0.26, 0.0)),
        Piece(fw_main, m.carbon_light.clone(), Transform::from_xyz(1.10, 0.105, 0.0)),
        Piece(fw_flap, m.carbon_light.clone(), Transform::from_xyz(0.97, 0.175, 0.0)),
        Piece(rw_main, m.carbon_light.clone(), Transform::from_xyz(-0.96, 0.845, 0.0)),
        Piece(rw_flap, m.carbon_light.clone(), Transform::from_xyz(-1.05, 0.930, 0.0)),
        Piece(dash, m.carbon.clone(), Transform::from_xyz(0.495, 0.455, 0.0)),
        // Main hoop crossbar, and the front hoop's.
        Piece(
            hoop_main.clone(),
            m.metal.clone(),
            Transform::from_xyz(-0.40, 1.01, 0.0)
                .with_rotation(Quat::from_rotation_x(quarter))
                .with_scale(Vec3::new(1.0, 0.57, 1.0)),
        ),
        Piece(
            hoop_front.clone(),
            m.metal.clone(),
            Transform::from_xyz(0.52, 0.56, 0.0)
                .with_rotation(Quat::from_rotation_x(quarter))
                .with_scale(Vec3::new(1.0, 0.55, 1.0)),
        ),
    ];

    for side in [-1.0f32, 1.0] {
        body.push(Piece(tub_side.clone(), m.maroon.clone(), Transform::from_xyz(-0.15, 0.215, side * 0.315)));
        body.push(Piece(tub_pad.clone(), m.carbon_light.clone(), Transform::from_xyz(-0.15, 0.372, side * 0.315)));
        body.push(Piece(tub_stripe.clone(), m.gold.clone(), Transform::from_xyz(-0.15, 0.155, side * 0.340)));
        body.push(Piece(pod.clone(), m.maroon.clone(), Transform::from_xyz(-0.20, 0.20, side * 0.45)));
        body.push(Piece(fw_plate.clone(), m.maroon.clone(), Transform::from_xyz(1.06, 0.150, side * 0.545)));
        body.push(Piece(rw_pylon.clone(), m.carbon.clone(), Transform::from_xyz(-0.94, 0.55, side * 0.16)));
        body.push(Piece(rw_plate.clone(), m.maroon.clone(), Transform::from_xyz(-0.99, 0.885, side * 0.505)));
        body.push(Piece(
            hoop_main.clone(),
            m.metal.clone(),
            Transform::from_xyz(-0.40, 0.62, side * 0.285).with_scale(Vec3::new(1.0, 0.78, 1.0)),
        ));
        // Front hoop kept below the 0.66 m eye line so the driver looks over it.
        body.push(Piece(
            hoop_front.clone(),
            m.metal.clone(),
            Transform::from_xyz(0.52, 0.33, side * 0.275).with_scale(Vec3::new(1.0, 0.46, 1.0)),
        ));
    }

    // Suspension: wishbones as cylinders rotated onto each link's axis.
    for (axle, track) in [(FRONT_AXLE, TRACK_F), (REAR_AXLE, TRACK_R)] {
        for side in [-1.0f32, 1.0] {
            let out_z = side * (track / 2.0 - 0.075);
            for (dx, y) in [(0.20, 0.115f32), (-0.22, 0.115), (0.18, 0.285), (-0.20, 0.285)] {
                let inner = Vec3::new(axle + dx, y, side * 0.26);
                let outer = Vec3::new(axle, y, out_z);
                let dir = outer - inner;
                body.push(Piece(
                    link.clone(),
                    m.metal.clone(),
                    Transform::from_translation((inner + outer) * 0.5)
                        .with_rotation(Quat::from_rotation_arc(Vec3::Y, dir.normalize()))
                        .with_scale(Vec3::new(1.0, dir.length(), 1.0)),
                ));
            }
            body.push(Piece(
                upright.clone(),
                m.carbon_light.clone(),
                Transform::from_xyz(axle, 0.20, out_z),
            ));
        }
    }

    // ---------------- steering wheel pieces ----------------
    let mut wheel_face: Vec<Piece> = vec![
        Piece(sw_top, m.carbon_light.clone(), Transform::from_xyz(0.0, 0.056, 0.0)),
        Piece(sw_bottom, m.carbon_light.clone(), Transform::from_xyz(0.0, -0.060, 0.0)),
        Piece(sw_centre, m.carbon_light.clone(), Transform::from_xyz(0.0, -0.004, 0.0)),
        Piece(sw_emblem, m.emblem.clone(), Transform::from_xyz(-0.008, -0.004, 0.0)),
    ];
    for side in [-1.0f32, 1.0] {
        wheel_face.push(Piece(sw_rail.clone(), m.carbon_light.clone(), Transform::from_xyz(0.0, -0.004, side * 0.084)));
        wheel_face.push(Piece(sw_grip.clone(), m.carbon.clone(), Transform::from_xyz(-0.020, -0.004, side * 0.085)));
        wheel_face.push(Piece(
            sw_button.clone(),
            m.gold.clone(),
            Transform::from_xyz(-0.012, 0.055, side * 0.062).with_rotation(Quat::from_rotation_z(quarter)),
        ));
        wheel_face.push(Piece(sw_paddle.clone(), m.amber.clone(), Transform::from_xyz(-0.012, 0.020, side * 0.019)));
    }
    for i in -1..=1 {
        wheel_face.push(Piece(
            sw_rotary.clone(),
            m.gold.clone(),
            Transform::from_xyz(-0.014, -0.061, i as f32 * 0.035).with_rotation(Quat::from_rotation_z(quarter)),
        ));
    }

    // ---------------- spawn ----------------
    let root = commands
        .spawn((CarRoot, Transform::default(), Visibility::default()))
        .id();

    for Piece(mesh, mat, t) in body {
        let e = commands.spawn((Mesh3d(mesh), MeshMaterial3d(mat), t)).id();
        commands.entity(root).add_child(e);
    }

    for (axle, track, front) in [(FRONT_AXLE, TRACK_F, true), (REAR_AXLE, TRACK_R, false)] {
        for side in [-1.0f32, 1.0] {
            let wheel = commands
                .spawn((
                    RoadWheel { front },
                    Transform::from_xyz(axle, TYRE_R, side * track / 2.0),
                    Visibility::default(),
                ))
                .id();
            commands.entity(root).add_child(wheel);

            // A Bevy Cylinder stands on Y, so lay it down to make a wheel.
            let t = commands
                .spawn((Mesh3d(tyre_mesh.clone()), MeshMaterial3d(m.tyre.clone()), Transform::from_rotation(lay)))
                .id();
            let r = commands
                .spawn((Mesh3d(rim_mesh.clone()), MeshMaterial3d(m.rim.clone()), Transform::from_rotation(lay)))
                .id();
            commands.entity(wheel).add_children(&[t, r]);
            for i in 0..5 {
                let a = i as f32 / 5.0 * std::f32::consts::TAU;
                let s = commands
                    .spawn((
                        Mesh3d(spoke.clone()),
                        MeshMaterial3d(m.gold.clone()),
                        Transform::from_rotation(lay * Quat::from_rotation_y(a)),
                    ))
                    .id();
                commands.entity(wheel).add_child(s);
            }
        }
    }

    let sw = commands
        .spawn((
            SteeringWheel,
            Transform::from_xyz(0.28, 0.50, 0.0)
                .with_rotation(Quat::from_rotation_z(-22.0f32.to_radians())),
            Visibility::default(),
        ))
        .id();
    commands.entity(root).add_child(sw);
    for Piece(mesh, mat, t) in wheel_face {
        let e = commands.spawn((Mesh3d(mesh), MeshMaterial3d(mat), t)).id();
        commands.entity(sw).add_child(e);
    }

    root
}
