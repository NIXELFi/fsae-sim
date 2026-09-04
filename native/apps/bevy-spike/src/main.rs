//! SDM26 driving simulator — Bevy visual spike.
//!
//! Purpose: judge whether Bevy's renderer is worth the rewrite. So the UI is
//! deliberately one line of text and the effort goes into the things the WebGL
//! build cannot do — real shadows, PBR materials, fog and tonemapping.
//!
//! Physics is `sim-core`, the same solver the eventual app would use.
//!
//!   cargo run -p bevy-spike --release
//!   cargo run -p bevy-spike --release -- --screenshot shot.png

mod audio;
mod cadmodel;
mod car;
mod ground;
mod track;

use bevy::asset::AssetPlugin;
use bevy::camera::Hdr;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::light::GlobalAmbientLight;
use bevy::pbr::{DistanceFog, FogFalloff};
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use bevy::window::WindowResolution;

use audio::EngineSound;
use car::{CarRoot, Materials, RoadWheel, SteeringWheel, EYE};
use sim_core::prelude::*;
use track::{to_world, TrackData};

/// The car and its solver.
#[derive(Resource)]
struct Sim {
    solver: Box<dyn Solver>,
    etc: EtcMap,
    spin_front: f32,
    spin_rear: f32,
    controls: Controls,
}

#[derive(Resource)]
struct Course(TrackData);

/// The engine synthesiser and its output stream.
///
/// A non-send resource because `cpal::Stream` is not `Sync` -- on some hosts it
/// is bound to the thread that created it. Bevy keeps non-send resources on the
/// main thread, which is exactly the guarantee cpal wants.
struct Engine(Option<EngineSound>);

#[derive(Resource)]
struct ShotRequest {
    path: Option<String>,
    frames: u32,
}

/// Where the camera sits. Chase is not just a nicety — you cannot diagnose the
/// scale or placement of a cockpit from inside it.
#[derive(Resource, Clone, Copy, PartialEq)]
enum View {
    Cockpit,
    Chase,
}

#[derive(Component)]
struct Hud;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let shot = args
        .iter()
        .position(|a| a == "--screenshot")
        .and_then(|i| args.get(i + 1).cloned());
    let view = if args.iter().any(|a| a == "--chase") {
        View::Chase
    } else {
        View::Cockpit
    };

    let course = TrackData::load();
    let (sx, sy, spsi) = course.start_pose();
    let mut solver = sim_core::sdm26_default();
    solver.reset(sx, sy, spsi, 0.0);

    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "SDM26 Driver-in-Loop — Bevy spike".into(),
                        resolution: WindowResolution::new(1600, 900),
                        ..default()
                    }),
                    ..default()
                })
                // Point the asset server at the same directory `cadmodel` checks.
                // Bevy's default is `assets` beside the executable, which for a
                // cargo build is target/release -- so the CAD file was found on
                // disk, reported as loaded, and then failed to load from a path
                // nothing had put it in. Computing the root in one place and
                // telling both halves about it is what stops that.
                .set(AssetPlugin {
                    file_path: cadmodel::asset_root().to_string_lossy().into_owned(),
                    ..default()
                }),
        )
        .insert_resource(ClearColor(Color::srgb(0.42, 0.56, 0.75)))
        // Sky bounce. In 0.19 AmbientLight is a per-camera component and the
        // scene-wide one is GlobalAmbientLight.
        .insert_resource(GlobalAmbientLight {
            color: Color::srgb(0.62, 0.72, 0.88),
            // The cockpit is a box that faces away from the sun, so almost all
            // of its light is sky bounce. Too low and the whole interior — the
            // part the driver actually looks at — goes to mud.
            brightness: 450.0,
            ..default()
        })
        .insert_resource(Sim {
            solver,
            etc: EtcMap::linear(),
            spin_front: 0.0,
            spin_rear: 0.0,
            controls: Controls::default(),
        })
        .insert_resource(Course(course))
        .insert_resource(ShotRequest { path: shot, frames: 0 })
        .insert_resource(view)
        .insert_resource(Time::<Fixed>::from_hz(120.0))
        // Non-send: cpal's stream is not Sync and on some hosts is bound to the
        // thread that made it. Bevy keeps non-send resources on the main
        // thread, which is the guarantee cpal wants.
        .insert_non_send(Engine(EngineSound::start()))
        .init_resource::<cadmodel::CadModel>()
        .add_systems(Startup, (setup, report_audio))
        .add_systems(
            Update,
            (
                read_input,
                drive_visuals,
                update_hud,
                update_engine_audio,
                wire_cad_model,
                maybe_screenshot,
            ),
        )
        .add_systems(FixedUpdate, step_physics)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    course: Res<Course>,
    assets: Res<AssetServer>,
    mut cad: ResMut<cadmodel::CadModel>,
) {
    let mats = Materials::new(&mut materials);
    let asphalt = ground::asphalt_image(&mut images);

    // ---- ground: the lot ----
    const LOT: f32 = 1200.0;
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(LOT, LOT))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            base_color_texture: Some(asphalt.clone()),
            uv_transform: bevy::math::Affine2::from_scale(Vec2::splat(
                LOT / ground::TILE_METRES,
            )),
            perceptual_roughness: 0.93,
            ..default()
        })),
        Transform::default(),
    ));

    // ---- driving surface: rubbered-in, so darker than the lot around it ----
    commands.spawn((
        Mesh3d(meshes.add(track::ribbon_mesh(&course.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.075, 0.076, 0.080),
            perceptual_roughness: 0.80,
            ..default()
        })),
        Transform::default(),
    ));

    // ---- cones ----
    let cone_mesh = meshes.add(Cone { radius: 0.145, height: 0.46 });
    // Band radius follows the cone's taper at the height it sits, so it hugs
    // the body instead of floating off it like a collar.
    let band_mesh = meshes.add(Cylinder::new(0.072, 0.052));
    let base_mesh = meshes.add(Cuboid::new(0.31, 0.014, 0.31));
    for c in &course.0.cones {
        let p = to_world(c[0], c[1], 0.0);
        commands.spawn((
            Mesh3d(cone_mesh.clone()),
            MeshMaterial3d(mats.cone.clone()),
            Transform::from_translation(p + Vec3::Y * 0.23),
        ));
        commands.spawn((
            Mesh3d(band_mesh.clone()),
            MeshMaterial3d(mats.cone_band.clone()),
            Transform::from_translation(p + Vec3::Y * 0.30),
        ));
        commands.spawn((
            Mesh3d(base_mesh.clone()),
            MeshMaterial3d(mats.carbon.clone()),
            Transform::from_translation(p + Vec3::Y * 0.007),
        ));
    }

    // ---- the car ----
    let car_root = car::spawn_car(&mut commands, &mut meshes, &mats);

    // Optional CAD bodywork. Spawned as a child of the car root so it inherits
    // the chassis position and attitude for free; if there is no file, nothing
    // happens and the procedural body stays.
    cadmodel::spawn_if_present(&mut commands, &assets, car_root, &mut cad);

    // ---- sun ----
    commands.spawn((
        // 32 klx blew the maroon out to hot pink under ACES; 11 klx buried the
        // whole scene. Bright overcast is ~20 klx and lands the livery as
        // maroon with the asphalt still readable.
        DirectionalLight {
            illuminance: 20_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(60.0, 90.0, 40.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // ---- camera: driver's eye ----
    commands.spawn((
        Camera3d::default(),
        Hdr,
        Tonemapping::AcesFitted,
        Bloom::NATURAL,
        // Tighter than reality on purpose: the asphalt texture has no mipmaps,
        // so it sparkles at grazing angles, and hazing the far field both hides
        // that and matches how the WebGL build fades surface detail.
        DistanceFog {
            color: Color::srgb(0.60, 0.69, 0.83),
            falloff: FogFalloff::Linear { start: 45.0, end: 240.0 },
            ..default()
        },
        Projection::from(PerspectiveProjection {
            fov: 78.0f32.to_radians(),
            near: 0.05,
            far: 900.0,
            ..default()
        }),
        Transform::from_xyz(0.0, 1.0, 0.0),
    ));

    // ---- the entire UI ----
    commands.spawn((
        Hud,
        Text::new(""),
        TextFont { font_size: FontSize::Px(20.0), ..default() },
        TextColor(Color::srgb(0.96, 0.97, 0.99)),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(18.0),
            bottom: Val::Px(16.0),
            ..default()
        },
    ));
}

fn read_input(
    keys: Res<ButtonInput<KeyCode>>,
    gamepads: Query<&Gamepad>,
    mut sim: ResMut<Sim>,
) {
    let mut steer = 0.0f64;
    let mut throttle = 0.0f64;
    let mut brake = 0.0f64;
    let mut up = false;
    let mut down = false;

    for pad in &gamepads {
        let lx = pad.get(GamepadAxis::LeftStickX).unwrap_or(0.0) as f64;
        if lx.abs() > 0.10 {
            let m = (lx.abs() - 0.10) / 0.90;
            steer = -lx.signum() * m.powf(1.7);
        }
        throttle = pad.get(GamepadButton::RightTrigger2).unwrap_or(0.0) as f64;
        brake = pad.get(GamepadButton::LeftTrigger2).unwrap_or(0.0) as f64;
        up |= pad.just_pressed(GamepadButton::RightTrigger);
        down |= pad.just_pressed(GamepadButton::LeftTrigger);
    }

    if keys.pressed(KeyCode::KeyA) || keys.pressed(KeyCode::ArrowLeft) {
        steer = 1.0;
    }
    if keys.pressed(KeyCode::KeyD) || keys.pressed(KeyCode::ArrowRight) {
        steer = -1.0;
    }
    if keys.pressed(KeyCode::KeyW) || keys.pressed(KeyCode::ArrowUp) {
        throttle = 1.0;
    }
    if keys.pressed(KeyCode::KeyS) || keys.pressed(KeyCode::ArrowDown) {
        brake = 1.0;
    }
    up |= keys.just_pressed(KeyCode::KeyE);
    down |= keys.just_pressed(KeyCode::KeyQ);

    let plate = sim.etc.evaluate(throttle);
    sim.controls = Controls { steer, throttle: plate, brake };

    if up {
        sim.solver.powertrain_mut().shift_up();
    }
    if down {
        sim.solver.powertrain_mut().shift_down();
    }
}

/// Push the solver's operating point at the synthesiser.
///
/// Once a frame, not per physics tick: re-solving the heat release and
/// retuning the waveguide is not free, and neither the rpm nor the exhaust gas
/// temperature changes meaningfully in 2 ms.
fn update_engine_audio(mut sim: ResMut<Sim>, mut engine: NonSendMut<Engine>) {
    let Some(sound) = engine.0.as_mut() else {
        return;
    };
    let tel = sim.solver.telemetry();
    let rpm = tel.engine_rpm as f32;
    let throttle = sim.controls.throttle as f32;

    // The torque the engine is actually making. The synthesiser solves its
    // combustion to produce this much work, so the note and the acceleration
    // answer to the same number rather than drifting apart.
    let torque = sim.solver.powertrain_mut().wot_torque_nm(tel.engine_rpm) as f32 * throttle;
    sound.set_operating_point(rpm, throttle, torque);
}

/// Say whether the engine synthesiser got an output device.
///
/// Worth a line of log: a silent run is otherwise indistinguishable from a
/// working one with the volume down, and the failure is silent by design --
/// the simulator starts without audio rather than refusing to start.
fn report_audio(engine: NonSend<Engine>) {
    match &engine.0 {
        Some(_) => info!("engine audio: physical model running on the output device"),
        None => warn!("engine audio: no output device, running silent"),
    }
}

/// Hook the CAD model's named nodes up to the animation, once it has loaded.
///
/// Scene spawning is asynchronous, so this polls until the names appear rather
/// than assuming they are there the frame after the spawn was requested. It
/// runs exactly once thereafter.
///
/// What it does is attach the SAME marker components the procedural body uses,
/// so `drive_visuals` animates a CAD wheel without knowing it is one. The
/// alternative -- a second code path for CAD models -- would mean the two could
/// drift apart, and the one nobody is looking at would be the broken one.
fn wire_cad_model(
    mut commands: Commands,
    mut cad: ResMut<cadmodel::CadModel>,
    named: Query<(Entity, &Name)>,
    car_root: Query<&Children, With<CarRoot>>,
    cad_bodies: Query<(), With<cadmodel::CadBody>>,
    mut visibility: Query<&mut Visibility>,
) {
    if !cad.loaded || cad.wired {
        return;
    }

    let mut found = 0;
    for (entity, name) in named.iter() {
        match cadmodel::role_of(name.as_str()) {
            Some(cadmodel::Role::Wheel { front }) => {
                commands.entity(entity).insert(RoadWheel { front });
                found += 1;
            }
            Some(cadmodel::Role::SteeringWheel) => {
                commands.entity(entity).insert(SteeringWheel);
                found += 1;
            }
            None => {}
        }
    }
    if found == 0 {
        return; // scene has not finished spawning yet
    }

    // Hide the procedural body, but not the CAD model that now sits beside it.
    //
    // Hidden rather than despawned: a half-finished export can be compared
    // against the known-good procedural geometry by flipping this back, which
    // is exactly what you want while getting an export right.
    if let Ok(children) = car_root.single() {
        for child in children.iter() {
            if cad_bodies.get(child).is_err() {
                if let Ok(mut v) = visibility.get_mut(child) {
                    *v = Visibility::Hidden;
                }
            }
        }
    }

    cad.wired = true;
    info!("CAD model wired: {found} named nodes hooked to the animation");
}

fn step_physics(time: Res<Time<Fixed>>, mut sim: ResMut<Sim>) {
    let dt = time.delta_secs_f64();
    let c = sim.controls;
    sim.solver.step(dt, c);

    let tel = sim.solver.telemetry();
    sim.spin_front += (tel.wheel_omega_front * dt) as f32;
    sim.spin_rear += (tel.wheel_omega_rear * dt) as f32;
}

#[allow(clippy::type_complexity)]
fn drive_visuals(
    sim: Res<Sim>,
    view: Res<View>,
    mut sets: ParamSet<(
        Query<&mut Transform, With<CarRoot>>,
        Query<(&mut Transform, &RoadWheel)>,
        Query<&mut Transform, With<SteeringWheel>>,
        Query<&mut Transform, With<Camera3d>>,
    )>,
) {
    let s = sim.solver.state();
    let tel = sim.solver.telemetry();
    let p = sim.solver.params();

    // Chassis attitude. Roll and pitch are gradients here, not DOF: positive ay
    // is a left turn and the car leans onto its outside (right) springs.
    let roll = (tel.ay_g * 0.595f64).to_radians() as f32;
    let pitch = (tel.ax_g * 0.35f64).to_radians() as f32;

    let pos = to_world(s.x, s.y, 0.0);
    let yaw = Quat::from_rotation_y(s.psi as f32);
    let body = yaw * Quat::from_rotation_z(pitch) * Quat::from_rotation_x(roll);

    if let Ok(mut t) = sets.p0().single_mut() {
        t.translation = pos;
        t.rotation = body;
    }

    let steer = tel.steer_rad as f32;
    for (mut t, w) in sets.p1().iter_mut() {
        let steer_q = if w.front {
            Quat::from_rotation_y(steer)
        } else {
            Quat::IDENTITY
        };
        let spin = if w.front { sim.spin_front } else { sim.spin_rear };
        t.rotation = steer_q * Quat::from_rotation_z(-spin);
    }

    if let Ok(mut t) = sets.p2().single_mut() {
        let ratio = p.steering.ratio as f32;
        t.rotation = Quat::from_rotation_z(-22.0f32.to_radians())
            * Quat::from_rotation_x(-steer * ratio);
    }

    if let Ok(mut cam) = sets.p3().single_mut() {
        match *view {
            View::Cockpit => {
                // A Bevy camera looks down its own -Z, but the car is modelled
                // with +X forward. Without this quarter turn the driver faces
                // backwards into the roll hoop.
                cam.translation = pos + body * EYE;
                cam.rotation = body * Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2);
            }
            View::Chase => {
                // Fixed offset in the car's yaw frame, looking at the tub.
                let yaw_only = Quat::from_rotation_y(s.psi as f32);
                let eye = pos + yaw_only * Vec3::new(-4.2, 1.9, 2.6);
                cam.translation = eye;
                cam.look_at(pos + Vec3::Y * 0.35, Vec3::Y);
            }
        }
    }
}

fn update_hud(sim: Res<Sim>, mut q: Query<&mut Text, With<Hud>>) {
    let tel = sim.solver.telemetry();
    let s = sim.solver.state();
    if let Ok(mut text) = q.single_mut() {
        **text = format!(
            "{:>3.0} km/h    gear {}    {:>5.0} rpm    {:+.2} g lat  {:+.2} g long\n{}  ·  {}",
            s.speed() * 3.6,
            tel.gear + 1,
            tel.engine_rpm,
            tel.ay_g,
            tel.ax_g,
            sim.solver.name(),
            sim.solver.tyre().name(),
        );
    }
}

/// Capture a frame to disk so the look can actually be judged and iterated on.
///
/// The readback is asynchronous: the observer fires some frames after the
/// request, so exiting on a fixed frame count silently produces no file. Wait
/// for the file to actually exist instead, with a timeout so a failure is
/// loud rather than a hang.
fn maybe_screenshot(
    mut commands: Commands,
    mut req: ResMut<ShotRequest>,
    mut exit: MessageWriter<AppExit>,
) {
    let Some(path) = req.path.clone() else {
        return;
    };
    req.frames += 1;

    // Give the renderer a few frames to settle shadows, bloom and exposure.
    if req.frames == 40 {
        info!("requesting screenshot -> {path}");
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path.clone()));
    }

    if req.frames > 40 && std::path::Path::new(&path).exists() {
        info!("screenshot written, exiting");
        exit.write(AppExit::Success);
    } else if req.frames > 900 {
        error!("screenshot never appeared at {path}");
        exit.write(AppExit::error());
    }
}
