mod handler;
mod robomaster;
mod ros2;
mod statistic;
mod util;

use crate::ros2::plugin::ROS2Plugin;
use crate::{
    handler::{on_activate, on_hit},
    robomaster::power_rune::{PowerRunePlugin, PowerRuneRoot, Projectile},
    statistic::{accurate_count, accurate_pct, increase_launch, launch_count},
};
use avian3d::prelude::*;
use bevy::camera::Exposure;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::light::light_consts::lux;
use bevy::pbr::Atmosphere;
use bevy::render::view::screenshot::{save_to_disk, Capturing, Screenshot};
use bevy::window::{CursorIcon, PresentMode, SystemCursorIcon};
use bevy::{
    anti_alias::fxaa::Fxaa,
    core_pipeline::tonemapping::Tonemapping,
    input::mouse::MouseMotion,
    post_process::motion_blur::MotionBlur,
    prelude::*,
    render::view::Hdr,
    scene::{SceneInstance, SceneInstanceReady},
};
use bevy_inspector_egui::bevy_egui::{EguiGlobalSettings, PrimaryEguiContext};
use bevy_inspector_egui::{bevy_egui::EguiPlugin, quick::WorldInspectorPlugin};
use std::collections::HashSet;

#[derive(Component)]
struct MainCamera {
    follow_offset: Vec3,
}

#[derive(Component)]
struct LocalInfantry;

#[derive(Component)]
struct InfantryRoot;

#[derive(Resource, PartialEq)]
struct CameraMode(pub FollowingType);

#[derive(PartialEq)]
enum FollowingType {
    Free,
    Robot,
    ThirdPerson,
}

#[derive(Component, Default)]
struct InfantryChassis {
    yaw: f32,
}

#[derive(Component, Default)]
struct InfantryGimbal {
    local_yaw: f32,
    pitch: f32,
}

#[derive(Component)]
struct InfantryViewOffset(Transform);

#[derive(Component)]
struct InfantryLaunchOffset(Transform);

#[derive(PhysicsLayer, Default, Clone, Copy, Debug)]
enum GameLayer {
    #[default]
    Default,
    Vehicle,
    ProjectileSelf,
    ProjectileOther,
    Environment,
}

#[derive(Component)]
struct DisplayOnly;

#[derive(Resource)]
struct Cooldown(Timer);

/// Creates help text at the bottom of the screen.
fn create_help_text() -> Text {
    format!(
        "total={} accurate={} pct={}\nControls: F2-Screenshot F3-Change Camera | WASD-Move Mouse-Look Space-Shoot",
        launch_count(),
        accurate_count(),
        accurate_pct()
    )
        .into()
}

/// Spawns the help text at the bottom of the screen.
fn spawn_text(commands: &mut Commands) {
    commands.spawn((
        create_help_text(),
        Node {
            position_type: PositionType::Absolute,
            bottom: px(12),
            left: px(12),
            ..default()
        },
    ));
}

fn update_help_text(mut text: Query<&mut Text>) {
    for mut text in text.iter_mut() {
        *text = create_help_text();
    }
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    present_mode: PresentMode::AutoVsync,
                    fit_canvas_to_parent: true,
                    ..default()
                }),
                ..default()
            }),
            PhysicsPlugins::default(),
        ))
        .add_plugins(ROS2Plugin::default())
        .add_plugins((EguiPlugin::default(), WorldInspectorPlugin::new()))
        //.add_plugins(PhysicsDebugPlugin::default())
        .add_plugins(PowerRunePlugin)
        .add_plugins((
            FrameTimeDiagnosticsPlugin::default(),
            LogDiagnosticsPlugin::default(),
        ))
        .insert_resource(CameraMode(FollowingType::Robot))
        .insert_resource(Gravity(Vec3::NEG_Y * 9.81))
        .insert_resource(SubstepCount(20))
        .insert_resource(Cooldown(Timer::from_seconds(0.1, TimerMode::Once)))
        .add_systems(Startup, (setup, setup_projectile))
        .add_observer(setup_vehicle)
        .add_observer(setup_collision)
        .add_observer(on_hit)
        .add_observer(on_activate)
        .add_systems(
            Update,
            (
                update_help_text,
                following_controls,
                vehicle_controls,
                remote_vehicle_controls,
                gimbal_controls,
                remote_gimbal_controls,
                freecam_controls,
                update_camera_follow,
                screenshot_on_f2,
                screenshot_saving,
            ),
        )
        .add_systems(
            PostUpdate,
            projectile_launch.after(TransformSystems::Propagate),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut egui_global_settings: ResMut<EguiGlobalSettings>,
) {
    egui_global_settings.auto_create_primary_context = false;
    spawn_text(&mut commands);
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(0.9, 0.95, 1.0),
            shadows_enabled: true,
            illuminance: lux::DIRECT_SUNLIGHT,
            ..default()
        },
        Transform::from_xyz(0.0, 4.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        RigidBody::Static,
        SceneRoot(asset_server.load("GROUND_DISPLAY.glb#Scene0")),
        CollisionMargin(0.01),
        Restitution::ZERO,
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMeshWithConfig(
            TrimeshFlags::MERGE_DUPLICATE_VERTICES,
        )),
        CollisionLayers::new(
            [GameLayer::Environment],
            [
                GameLayer::Default,
                GameLayer::Vehicle,
                GameLayer::ProjectileSelf,
                GameLayer::ProjectileOther,
            ],
        ),
        Transform::IDENTITY,
        Visibility::Hidden,
    ));
    commands.spawn((
        SceneRoot(asset_server.load("GROUND_DISPLAY.glb#Scene0")),
        Transform::IDENTITY,
        DisplayOnly,
    ));
    commands.spawn((
        SceneRoot(asset_server.load("CALIB.glb#Scene0")),
        Transform::IDENTITY
            .with_scale(Vec3::splat(1.0))
            .with_translation(Vec3::new(1.0, 0.5, 1.0)),
        DisplayOnly,
    ));

    commands.spawn((
        RigidBody::Static,
        CollisionMargin(0.01),
        Restitution::ZERO,
        CollisionLayers::new(
            [GameLayer::Environment],
            [
                GameLayer::Default,
                GameLayer::Vehicle,
                GameLayer::ProjectileSelf,
                GameLayer::ProjectileOther,
            ],
        ),
        SceneRoot(asset_server.load("POWER.glb#Scene0")),
        Transform::IDENTITY,
        PowerRuneRoot,
    ));

    commands.spawn((
        RigidBody::Dynamic,
        Collider::cuboid(0.494884, 0.433951, 0.518837),
        CollisionLayers::new(
            GameLayer::Vehicle,
            [
                GameLayer::Default,
                GameLayer::Vehicle,
                GameLayer::ProjectileOther,
                GameLayer::Environment,
            ],
        ),
        Mass(20.0),
        Friction::new(0.5),
        Restitution::ZERO,
        LinearDamping(0.0),
        AngularDamping(109.8),
        LockedAxes::new().lock_rotation_x().lock_rotation_z(),
        SceneRoot(asset_server.load("vehicle.glb#Scene0")),
        Transform::from_xyz(0.0, 1.0, 0.0),
        InfantryRoot,
        LocalInfantry,
    ));

    commands.spawn((
        RigidBody::Dynamic,
        Collider::cuboid(0.494884, 0.433951, 0.518837),
        CollisionLayers::new(
            GameLayer::Vehicle,
            [
                GameLayer::Default,
                GameLayer::Vehicle,
                GameLayer::ProjectileOther,
                GameLayer::Environment,
            ],
        ),
        Mass(20.0),
        Friction::new(0.5),
        Restitution::ZERO,
        LinearDamping(0.0),
        AngularDamping(109.8),
        LockedAxes::new().lock_rotation_x().lock_rotation_z(),
        SceneRoot(asset_server.load("vehicle.glb#Scene0")),
        Transform::from_xyz(1.0, 1.0, 1.0),
        InfantryRoot,
    ));

    commands.spawn((
        Camera3d::default(),
        Atmosphere::EARTH,
        Camera::default(),
        MotionBlur {
            shutter_angle: 0.25,
            samples: 4,
        },
        PrimaryEguiContext,
        Projection::Perspective(PerspectiveProjection {
            fov: std::f32::consts::PI / 180.0 * 45.0,
            near: 0.1,
            far: 500000000.0,
            ..default()
        }),
        AmbientLight {
            brightness: 4000.0,
            color: Color::WHITE,
            ..default()
        },
        Exposure::SUNLIGHT,
        Tonemapping::AcesFitted,
        Msaa::Off,
        Fxaa::default(),
        Hdr,
        Transform::from_xyz(0.0, 10.0, 15.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        MainCamera {
            follow_offset: Vec3::new(0.0, 3.0, 2.0),
        },
        ros2::plugin::MainCamera,
    ));
}

fn setup_vehicle(
    events: On<SceneInstanceReady>,
    mut commands: Commands,
    root_query: Query<(Entity, Option<&LocalInfantry>), With<InfantryRoot>>,
    secondary_query: Query<&ChildOf, (Without<InfantryRoot>, Without<SceneInstance>)>,
    node_query: Query<
        (Entity, &Name, &ChildOf, &Transform),
        (Without<InfantryRoot>, Without<SceneInstance>),
    >,
) {
    let root = events.entity;
    if root_query.get(root).is_err() {
        return;
    }
    let is_local = root_query.get(root).unwrap().1.is_some();

    let mut despawn = HashSet::new();

    for (node, name, &ChildOf(secondary), transform) in node_query {
        if let Ok(&ChildOf(root2)) = secondary_query.get(secondary) {
            if root == root2 {
                despawn.insert(secondary);
                commands.entity(secondary).remove_child(node);
                commands.entity(root).add_child(node);
                let mut ent = commands.entity(node);
                if is_local {
                    ent.insert(LocalInfantry);
                }
                match name.as_str() {
                    "BASE" => {
                        ent.insert(InfantryChassis::default());
                    }
                    "GIMBAL" => {
                        ent.insert(InfantryGimbal::default());
                    }
                    "SHOT_DIRECTION" => {
                        ent.insert(InfantryLaunchOffset(transform.clone()));
                    }
                    "CAM_DIRECTION" => {
                        ent.insert(InfantryViewOffset(transform.clone()));
                    }
                    _ => {}
                }
            }
        }
    }

    for ent in despawn {
        commands.entity(ent).despawn();
    }
}

fn setup_projectile(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(ProjectileSetting(
        meshes.add(Sphere::new(44.5 * 0.001 / 2.0)),
        materials.add(StandardMaterial {
            base_color: Color::srgba(0.132866, 1.0, 0.132869, 0.55),
            emissive: LinearRgba::new(0.132866, 1.0, 0.132869, 0.55),
            emissive_exposure_weight: -1.0,
            alpha_mode: AlphaMode::Blend,
            ..default()
        }),
    ));
}

#[derive(Resource)]
struct ProjectileSetting(Handle<Mesh>, Handle<StandardMaterial>);

fn projectile_launch(
    time: Res<Time>,
    _asset_server: Res<AssetServer>,
    mut cooldown: ResMut<Cooldown>,
    mut commands: Commands,
    setting: Res<ProjectileSetting>,
    keyboard: Res<ButtonInput<KeyCode>>,
    infantry: Single<
        (&LinearVelocity, &AngularVelocity),
        (With<InfantryRoot>, With<LocalInfantry>),
    >,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
    launch_offset: Single<&InfantryLaunchOffset, With<LocalInfantry>>,
) {
    cooldown.0.tick(time.delta());
    if !cooldown.0.is_finished() {
        return;
    }
    cooldown.0.reset();
    if keyboard.pressed(KeyCode::Space) {
        increase_launch();
        let direction = (gimbal.0.rotation() * launch_offset.0.rotation)
            .mul_vec3(Vec3::Y)
            .normalize_or_zero();
        if direction == Vec3::ZERO {
            return;
        }
        let vel = infantry.0.0 + direction * 25.0;
        commands.spawn((
            RigidBody::Dynamic,
            Collider::sphere(44.5 * 0.001 / 2.0),
            Mass(44.5 * 0.001),
            Friction::new(1.1),
            Restitution::new(0.00005),
            LinearDamping(0.05),
            CollisionLayers::new(
                GameLayer::ProjectileSelf,
                [
                    GameLayer::Default,
                    GameLayer::Vehicle,
                    GameLayer::ProjectileSelf,
                    GameLayer::ProjectileOther,
                    GameLayer::Environment,
                ],
            ),
            Mesh3d(setting.0.clone()),
            MeshMaterial3d(setting.1.clone()),
            LinearVelocity(vel),
            AngularVelocity(infantry.1.0),
            Transform::IDENTITY.with_translation(
                gimbal.0.translation() + (gimbal.0.rotation() * launch_offset.0.translation),
            ),
            //AudioPlayer::new(asset_server.load("projectile_launch.ogg")),
            Projectile,
        ));
    }
}

fn setup_collision(
    events: On<SceneInstanceReady>,
    mut commands: Commands,
    root_query: Query<
        Entity,
        (
            Without<Collider>,
            Without<ColliderConstructorHierarchy>,
            Without<DisplayOnly>,
        ),
    >,
) {
    if root_query.contains(events.entity) {
        commands
            .entity(events.entity)
            .insert((ColliderConstructorHierarchy::new(
                ColliderConstructor::VoxelizedTrimeshFromMesh {
                    voxel_size: 0.025,
                    fill_mode: FillMode::FloodFill {
                        detect_cavities: true,
                    },
                },
            ),));
    }
}

// 单位 m/s^2
const VEHICLE_ACCEL: f32 = 10.0;
// 单位 rad/s
const VEHICLE_ROTATION_SPEED: f32 = 3.0;
const GIMBAL_ROTATION_SPEED: f32 = 3.0;

const MAX_VEHICLE_VELOCITY: f32 = 6.0;

fn vehicle_controls(
    time: Res<Time>,
    mode: Res<CameraMode>,
    keyboard: Res<ButtonInput<KeyCode>>,
    infantry: Single<Forces, (With<InfantryRoot>, With<LocalInfantry>)>,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
    chassis: Single<
        (&mut Transform, &mut InfantryChassis),
        (With<LocalInfantry>, Without<InfantryGimbal>),
    >,
) {
    if mode.0 == FollowingType::Free {
        return;
    }

    let mut input = Vec2::ZERO;
    if keyboard.pressed(KeyCode::KeyW) {
        input.y += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        input.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        input.x += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        input.x -= 1.0;
    }

    let dt = time.delta_secs();
    let mut forces = infantry.into_inner();

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    let (gimbal_transform, _gimbal) = gimbal.into_inner();

    let forward = gimbal_transform.forward().with_y(0.0);
    let right = gimbal_transform.right().with_y(0.0);
    let forward_xz = forward.with_y(0.0).normalize_or_zero();
    let right_xz = right.with_y(0.0).normalize_or_zero();

    let desired_dir = (forward_xz * input.y + right_xz * input.x).normalize_or_zero();
    forces.apply_linear_acceleration(desired_dir * VEHICLE_ACCEL);
    let linear_vel = forces.linear_velocity();
    let current_velocity = linear_vel.length();
    if current_velocity > MAX_VEHICLE_VELOCITY {
        let brake_force = linear_vel.normalize() * (current_velocity - MAX_VEHICLE_VELOCITY) * 50.0;
        forces.apply_linear_acceleration(-brake_force);
    } else if input == Vec2::ZERO {
        forces.apply_linear_acceleration(-linear_vel * 10.0);
    }

    if keyboard.pressed(KeyCode::KeyQ) {
        chassis_data.yaw += VEHICLE_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyE) {
        chassis_data.yaw -= VEHICLE_ROTATION_SPEED * dt;
    }

    chassis_transform.rotation = Quat::from_euler(EulerRot::YXZ, chassis_data.yaw, 0.0, 0.0);
}

fn remote_vehicle_controls(
    time: Res<Time>,
    _mode: Res<CameraMode>,
    keyboard: Res<ButtonInput<KeyCode>>,
    infantry: Single<Forces, (With<InfantryRoot>, Without<LocalInfantry>)>,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (Without<LocalInfantry>, Without<InfantryChassis>),
    >,
    chassis: Single<
        (&mut Transform, &mut InfantryChassis),
        (Without<LocalInfantry>, Without<InfantryGimbal>),
    >,
) {
    let mut input = Vec2::ZERO;
    if keyboard.pressed(KeyCode::KeyI) {
        input.y += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyK) {
        input.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyL) {
        input.x += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyJ) {
        input.x -= 1.0;
    }

    let dt = time.delta_secs();
    let mut forces = infantry.into_inner();

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    let (gimbal_transform, _gimbal) = gimbal.into_inner();

    let forward = gimbal_transform.forward().with_y(0.0);
    let right = gimbal_transform.right().with_y(0.0);
    let forward_xz = forward.with_y(0.0).normalize_or_zero();
    let right_xz = right.with_y(0.0).normalize_or_zero();

    let desired_dir = (forward_xz * input.y + right_xz * input.x).normalize_or_zero();
    forces.apply_linear_acceleration(desired_dir * VEHICLE_ACCEL);
    let linear_vel = forces.linear_velocity();
    let current_velocity = linear_vel.length();
    if current_velocity > MAX_VEHICLE_VELOCITY {
        let brake_force = linear_vel.normalize() * (current_velocity - MAX_VEHICLE_VELOCITY) * 50.0;
        forces.apply_linear_acceleration(-brake_force);
    } else if input == Vec2::ZERO {
        forces.apply_linear_acceleration(-linear_vel * 10.0);
    }

    if keyboard.pressed(KeyCode::KeyU) {
        chassis_data.yaw += VEHICLE_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyO) {
        chassis_data.yaw -= VEHICLE_ROTATION_SPEED * dt;
    }

    chassis_transform.rotation = Quat::from_euler(EulerRot::YXZ, chassis_data.yaw, 0.0, 0.0);
}

fn gimbal_controls(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
) {
    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    if keyboard.pressed(KeyCode::ArrowLeft) {
        gimbal_data.local_yaw += GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::ArrowRight) {
        gimbal_data.local_yaw -= GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::ArrowUp) {
        gimbal_data.pitch += GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::ArrowDown) {
        gimbal_data.pitch -= GIMBAL_ROTATION_SPEED * dt;
    }

    gimbal_data.pitch = gimbal_data.pitch.clamp(-0.785, 0.785);

    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);

    gimbal_transform.rotation = gimbal_rotation;
}

fn remote_gimbal_controls(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (Without<LocalInfantry>, Without<InfantryChassis>),
    >,
) {
    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    if keyboard.pressed(KeyCode::KeyC) {
        gimbal_data.local_yaw += GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyB) {
        gimbal_data.local_yaw -= GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyF) {
        gimbal_data.pitch += GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyV) {
        gimbal_data.pitch -= GIMBAL_ROTATION_SPEED * dt;
    }

    gimbal_data.pitch = gimbal_data.pitch.clamp(-0.785, 0.785);

    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);

    gimbal_transform.rotation = gimbal_rotation;
}

fn following_controls(mut mode: ResMut<CameraMode>, keyboard: Res<ButtonInput<KeyCode>>) {
    if keyboard.just_pressed(KeyCode::F3) {
        mode.0 = match mode.0 {
            FollowingType::Free => FollowingType::Robot,
            FollowingType::Robot => FollowingType::ThirdPerson,
            FollowingType::ThirdPerson => FollowingType::Free,
        };
    }
}

fn update_camera_follow(
    camera_query: Single<(&mut Transform, &MainCamera), Without<LocalInfantry>>,
    infantry: Single<&Transform, (With<InfantryRoot>, With<LocalInfantry>)>,
    gimbal: Single<&Transform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,
    mode: Res<CameraMode>,
) {
    let gimbal_transform = gimbal.into_inner();
    let (mut camera_transform, camera_offset) = camera_query.into_inner();

    match mode.0 {
        FollowingType::Robot => {
            camera_transform.translation = infantry.translation
                + (infantry.rotation * gimbal_transform.rotation) * view_offset.0.translation;
            camera_transform.rotation = infantry.rotation * gimbal_transform.rotation;
        }
        FollowingType::ThirdPerson => {
            let base_transform = infantry.into_inner();
            let offset = base_transform.rotation * camera_offset.follow_offset;
            camera_transform.translation = base_transform.translation + offset;
            camera_transform.look_at(base_transform.translation, Vec3::Y);
        }
        FollowingType::Free => {}
    }
}

fn freecam_controls(
    time: Res<Time>,
    mode: Res<CameraMode>,
    mut mouse_motion_events: MessageReader<MouseMotion>,
    keyboard: Res<ButtonInput<KeyCode>>,
    camera_query: Single<&mut Transform, (With<MainCamera>, Without<InfantryRoot>)>,
) {
    if mode.0 != FollowingType::Free {
        return;
    }

    let delta = time.delta_secs();
    let mut camera_transform = camera_query.into_inner();

    let mut mouse_delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        mouse_delta += event.delta;
    }

    if mouse_delta != Vec2::ZERO {
        let (yaw, pitch, roll) = camera_transform.rotation.to_euler(EulerRot::YXZ);

        let new_yaw = yaw - mouse_delta.x * 0.003;
        let new_pitch = (pitch - mouse_delta.y * 0.003).clamp(-1.4, 1.4);

        camera_transform.rotation = Quat::from_euler(EulerRot::YXZ, new_yaw, new_pitch, roll);
    }

    const CAMERA_SPEED: f32 = 8.0;
    let speed = CAMERA_SPEED * delta;
    let forward = camera_transform.forward();
    let right = camera_transform.right();
    let up = camera_transform.up();

    if keyboard.pressed(KeyCode::KeyW) {
        camera_transform.translation += forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        camera_transform.translation -= forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        camera_transform.translation -= right * speed;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        camera_transform.translation += right * speed;
    }
    if keyboard.pressed(KeyCode::KeyN) {
        camera_transform.translation += up * speed;
    }
    if keyboard.pressed(KeyCode::KeyJ) {
        camera_transform.translation -= up * speed;
    }
}

fn screenshot_on_f2(
    mut commands: Commands,
    input: Res<ButtonInput<KeyCode>>,
    mut counter: Local<u32>,
) {
    if input.just_pressed(KeyCode::F2) {
        let path = format!("./screenshot-{}.png", *counter);
        *counter += 1;
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
    }
}

fn screenshot_saving(
    mut commands: Commands,
    screenshot_saving: Query<Entity, With<Capturing>>,
    window: Single<Entity, With<Window>>,
) {
    match screenshot_saving.iter().count() {
        0 => {
            commands.entity(*window).remove::<CursorIcon>();
        }
        x if x > 0 => {
            commands
                .entity(*window)
                .insert(CursorIcon::from(SystemCursorIcon::Progress));
        }
        _ => {}
    }
}
