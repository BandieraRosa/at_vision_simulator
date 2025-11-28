mod dataset;
mod handler;
mod robomaster;
mod ros2;
mod statistic;
mod util;

use crate::dataset::prelude::DatasetPlugin;
use crate::ros2::plugin::{AutoAimMode, FireCommand, ProjectileVelocity, ROS2Plugin, TargetEuler};
use crate::ros2::topic::{CurrentVelocityTopic, TopicPublisher};
use crate::util::bevy::insert_all_child;
use crate::{
    handler::{on_activate, on_hit},
    robomaster::power_rune::{PowerRunePlugin, PowerRuneRoot, Projectile},
    statistic::{accurate_count, accurate_pct, increase_launch, launch_count},
};
use avian3d::prelude::*;
use bevy::asset::embedded_asset;
use bevy::camera::Exposure;
use bevy::light::light_consts::lux;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::window::{CursorIcon, PresentMode, SystemCursorIcon};
use bevy::winit::WinitWindows;
use bevy::{
    anti_alias::fxaa::Fxaa,
    input::mouse::MouseMotion,
    prelude::*,
    scene::{SceneInstance, SceneInstanceReady},
};
use std::collections::{HashMap, HashSet, VecDeque};

#[derive(Component)]
struct MainCamera {
    follow_offset: Vec3,
}

#[derive(Component)]
pub struct LocalInfantry;

#[derive(Component)]
pub struct InfantryRoot;

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
pub struct InfantryGimbal {
    local_yaw: f32,
    pitch: f32,
}

#[derive(Component)]
pub struct InfantryViewOffset(Transform);

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

#[derive(Resource)]
struct Cooldown(Timer);

/// Creates help text at the bottom of the screen.
fn create_help_text(auto_aim_enabled: bool) -> Text {
    let mode_str = if auto_aim_enabled {
        "AUTO-AIM"
    } else {
        "MANUAL"
    };
    format!(
        "total={} accurate={} pct={:.2}% mode={}\nControls: F2-Screenshot F3-Camera T-AutoAim | WASD-Move Arrows-Gimbal Space-Shoot\n2025 Actor&Thinker",
        launch_count(),
        accurate_count(),
        accurate_pct() * 100.0,
        mode_str
    )
    .into()
}

/// Spawns the help text at the bottom of the screen.
fn spawn_text(commands: &mut Commands) {
    commands.spawn((
        create_help_text(false),
        Node {
            position_type: PositionType::Absolute,
            bottom: px(12),
            left: px(12),
            ..default()
        },
    ));
}

fn update_help_text(mut text: Query<&mut Text>, auto_aim: Res<AutoAimMode>) {
    for mut text in text.iter_mut() {
        *text = create_help_text(auto_aim.enabled);
    }
}

struct EmbeddedAssetPlugin;

impl Plugin for EmbeddedAssetPlugin {
    fn build(&self, app: &mut App) {
        embedded_asset!(app, "assets/GROUND_DISPLAY.glb");
        embedded_asset!(app, "assets/POWER.glb");
        embedded_asset!(app, "assets/projectile_launch.ogg");
        embedded_asset!(app, "assets/rune_activated.ogg");
        embedded_asset!(app, "assets/vehicle.glb");
    }
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    present_mode: PresentMode::AutoVsync,
                    name: Some("RoboMaster Simulator | Actor&Thinker".to_string()),
                    title: "RoboMaster Simulator | Actor&Thinker".to_string(),
                    fit_canvas_to_parent: true,
                    ..default()
                }),
                ..default()
            }),
            EmbeddedAssetPlugin,
            PhysicsPlugins::default(),
        ))
        .add_plugins(ROS2Plugin::default())
        .add_plugins(PowerRunePlugin)
        .add_plugins(DatasetPlugin)
        .insert_resource(CameraMode(FollowingType::Robot))
        .insert_resource(Gravity(Vec3::NEG_Y * 9.81))
        .insert_resource(SubstepCount(10))
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
                update_auto_aim_mode,
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

/// 更新自瞄模式状态（按住 T 键启用）
fn update_auto_aim_mode(keyboard: Res<ButtonInput<KeyCode>>, mut auto_aim: ResMut<AutoAimMode>) {
    auto_aim.enabled = keyboard.pressed(KeyCode::KeyT);
}

#[derive(Component)]
struct PreciousCollision(
    HashMap<String, (ColliderConstructorHierarchy, CollisionLayers, Visibility)>,
);

fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
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

    let layer_env = CollisionLayers::new(
        [GameLayer::Environment],
        [
            GameLayer::Default,
            GameLayer::Vehicle,
            GameLayer::ProjectileSelf,
            GameLayer::ProjectileOther,
        ],
    );

    let trimesh = ColliderConstructorHierarchy::new(
        ColliderConstructor::TrimeshFromMeshWithConfig(TrimeshFlags::all()),
    );
    let voxel = |size| {
        ColliderConstructorHierarchy::new(ColliderConstructor::VoxelizedTrimeshFromMesh {
            voxel_size: size,
            fill_mode: FillMode::FloodFill {
                detect_cavities: true,
            },
        })
    };

    commands.spawn((
        SceneRoot(asset_server.load("embedded://daedalus/assets/GROUND_DISPLAY.glb#Scene0")),
        Transform::IDENTITY,
        PreciousCollision(HashMap::from([(
            "GROUND_LOW".to_string(),
            (trimesh.clone(), layer_env, Visibility::Hidden),
        )])),
    ));

    let mut power_rune_col = HashMap::from([]);
    for i in 1..=2 {
        for j in 1..=5 {
            for k in ["ACTIVATED", "ACTIVE", "COMPLETED", "DISABLED"] {
                power_rune_col.insert(
                    format!("FACE_{}_TARGET_{}_{}", i, j, k).to_string(),
                    (voxel(0.015), layer_env, Visibility::Visible),
                );
            }
        }
    }
    commands.spawn((
        RigidBody::Static,
        CollisionMargin(0.001),
        Restitution::ZERO,
        SceneRoot(asset_server.load("embedded://daedalus/assets/POWER.glb#Scene0")),
        Transform::IDENTITY,
        PowerRuneRoot,
        PreciousCollision(power_rune_col),
    ));

    commands.spawn((
        RigidBody::Dynamic,
        Collider::cylinder(0.2593615, 0.433951),
        CollisionMargin(0.001),
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
        SceneRoot(asset_server.load("embedded://daedalus/assets/vehicle.glb#Scene0")),
        Transform::from_xyz(0.0, 1.0, 0.0),
        InfantryRoot,
        LocalInfantry,
    ));

    commands.spawn((
        RigidBody::Dynamic,
        Collider::cylinder(0.2593615, 0.433951),
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
        SceneRoot(asset_server.load("embedded://daedalus/assets/vehicle.glb#Scene0")),
        Transform::from_xyz(1.0, 1.0, 1.0),
        InfantryRoot,
    ));

    commands.spawn((
        Camera3d::default(),
        Camera::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: std::f32::consts::PI / 180.0 * 45.0,
            near: 0.1,
            far: 500000000.0,
            ..default()
        }),
        Exposure::SUNLIGHT,
        Msaa::Off,
        Fxaa::default(),
        Transform::from_xyz(0.0, 10.0, 15.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        MainCamera {
            follow_offset: Vec3::new(0.0, 3.0, 2.0),
        },
        ros2::plugin::MainCamera,
    ));
}

#[derive(Component, Clone)]
pub struct Armor(String);

fn setup_vehicle(
    events: On<SceneInstanceReady>,
    mut commands: Commands,
    children: Query<&Children>,
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
        let Ok(&ChildOf(root2)) = secondary_query.get(secondary) else {
            continue;
        };
        if root != root2 {
            continue;
        }
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
                let mut stack = VecDeque::from([(node, name)]);
                let mut set = HashSet::new();
                while let Some((e, name)) = stack.pop_front() {
                    if !set.insert(e) {
                        continue;
                    }
                    if name.starts_with("ARMOR_") && name.ends_with("_P") {
                        insert_all_child(&mut commands, e, &children, || Armor(name.to_string()));
                        commands.entity(e).insert(ColliderConstructorHierarchy::new(
                            ColliderConstructor::TrimeshFromMeshWithConfig(
                                TrimeshFlags::MERGE_DUPLICATE_VERTICES,
                            ),
                        ));
                    }
                    for (ee, n, &ChildOf(r), _) in node_query {
                        if r == e {
                            stack.push_back((ee, n));
                        }
                    }
                }
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
        (&Transform, &LinearVelocity, &AngularVelocity),
        (With<InfantryRoot>, With<LocalInfantry>),
    >,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
    launch_offset: Single<&InfantryLaunchOffset, With<LocalInfantry>>,
    // 自瞄相关资源
    auto_aim: Res<AutoAimMode>,
    fire_command: Res<FireCommand>,
    velocity_config: Res<ProjectileVelocity>,
) {
    cooldown.0.tick(time.delta());
    if !cooldown.0.is_finished() {
        return;
    }

    // 判断是否应该发射
    let should_fire = if auto_aim.enabled {
        // 自瞄模式：根据上位机指令发射
        fire_command.fire
    } else {
        // 手动模式：根据空格键发射
        keyboard.pressed(KeyCode::Space)
    };

    if !should_fire {
        return;
    }

    cooldown.0.reset();
    increase_launch();

    let direction = (gimbal.0.rotation() * launch_offset.0.rotation)
        .mul_vec3(Vec3::Y)
        .normalize_or_zero();
    if direction == Vec3::ZERO {
        return;
    }

    let vel = infantry.1.0 + direction * velocity_config.speed;
    commands.spawn((
        RigidBody::Dynamic,
        Collider::sphere(44.5 * 0.001 / 2.0),
        Mass(44.5 * 0.001),
        Friction::new(1.1),
        Restitution::ZERO,
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
        AngularVelocity(infantry.2.0),
        Transform::IDENTITY.with_translation(
            infantry.0.translation + (gimbal.0.rotation() * launch_offset.0.translation),
        ),
        Projectile,
    ));
}

fn setup_collision(
    events: On<SceneInstanceReady>,
    mut commands: Commands,
    children: Query<&Children>,
    name: Query<&Name, With<Children>>,
    root_query: Query<(Entity, &PreciousCollision)>,
) {
    let Ok((_, PreciousCollision(map))) = root_query.get(events.entity) else {
        return;
    };
    for e in children.iter_descendants(events.entity) {
        let Ok(name) = name.get(e) else {
            continue;
        };
        if let Some((constructor, layer, visibility)) = map.get(&name.to_string()) {
            commands.entity(e).insert((
                RigidBody::Static,
                Mass(0.0),
                Restitution::ZERO,
                constructor.clone(),
                CollisionMargin(0.001),
                *layer,
            ));
            if visibility == Visibility::Hidden {
                commands.entity(e).insert(*visibility);
            }
        }
    }
    commands.entity(events.entity).remove::<PreciousCollision>();
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
    infantry: Single<(Forces, &Mass), (With<InfantryRoot>, With<LocalInfantry>)>,
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
    let (mut forces, mass) = infantry.into_inner();

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    let (gimbal_transform, _gimbal) = gimbal.into_inner();

    let forward = gimbal_transform.forward().with_y(0.0);
    let right = gimbal_transform.right().with_y(0.0);
    let forward_xz = forward.with_y(0.0).normalize_or_zero();
    let right_xz = right.with_y(0.0).normalize_or_zero();

    let desired_dir = (forward_xz * input.y + right_xz * input.x).normalize_or_zero();
    forces.apply_linear_impulse(mass.0 * dt * desired_dir * VEHICLE_ACCEL);
    let linear_vel = forces.linear_velocity();
    let current_velocity = linear_vel.length();
    if current_velocity > MAX_VEHICLE_VELOCITY {
        let brake_force = linear_vel.normalize() * (current_velocity - MAX_VEHICLE_VELOCITY) * 50.0;
        forces.apply_linear_impulse(mass.0 * dt * -brake_force);
    } else if input == Vec2::ZERO {
        forces.apply_linear_impulse(mass.0 * dt * -linear_vel * 10.0);
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
    // 自瞄相关资源
    auto_aim: Res<AutoAimMode>,
    target_euler: Res<TargetEuler>,
) {
    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    // 从当前变换中提取欧拉角
    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    if auto_aim.enabled && target_euler.valid {
        // 自瞄模式：平滑转动到目标角度
        let target_yaw = target_euler.yaw;
        let target_pitch = target_euler.pitch;

        // 计算角度差
        let yaw_diff = angle_diff(gimbal_data.local_yaw, target_yaw);
        let pitch_diff = target_pitch - gimbal_data.pitch;

        // 按照 GIMBAL_ROTATION_SPEED 速率转动
        let max_delta = GIMBAL_ROTATION_SPEED * dt;

        // Yaw 控制
        if yaw_diff.abs() > 0.001 {
            let yaw_step = yaw_diff.clamp(-max_delta, max_delta);
            gimbal_data.local_yaw += yaw_step;
        }

        // Pitch 控制
        if pitch_diff.abs() > 0.001 {
            let pitch_step = pitch_diff.clamp(-max_delta, max_delta);
            gimbal_data.pitch += pitch_step;
        }
    } else {
        // 手动模式：键盘控制
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
    }

    // 限制 pitch 角度范围
    gimbal_data.pitch = gimbal_data.pitch.clamp(-0.785, 0.785);

    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);

    gimbal_transform.rotation = gimbal_rotation;
}

/// 计算两个角度之间的最短差值（处理 ±π 边界）
fn angle_diff(current: f32, target: f32) -> f32 {
    let diff = target - current;
    // 将差值归一化到 [-π, π] 范围
    let mut normalized = diff % (2.0 * std::f32::consts::PI);
    if normalized > std::f32::consts::PI {
        normalized -= 2.0 * std::f32::consts::PI;
    } else if normalized < -std::f32::consts::PI {
        normalized += 2.0 * std::f32::consts::PI;
    }
    normalized
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
    let (mut camera_transform, main_camera) = camera_query.into_inner();
    let infantry_transform = infantry.into_inner();

    match mode.0 {
        FollowingType::Free => {
            // 自由摄像机模式，不跟随
        }
        FollowingType::Robot => {
            // 第一人称视角
            let view = view_offset.into_inner();
            camera_transform.translation =
                infantry_transform.translation + gimbal_transform.rotation * view.0.translation;
            camera_transform.rotation = gimbal_transform.rotation * view.0.rotation;
        }
        FollowingType::ThirdPerson => {
            // 第三人称视角
            let target_pos = infantry_transform.translation;
            let offset = gimbal_transform.rotation * main_camera.follow_offset;
            camera_transform.translation = target_pos + offset;
            camera_transform.look_at(target_pos, Vec3::Y);
        }
    }
}

fn freecam_controls(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut mouse_motion: EventReader<MouseMotion>,
    mode: Res<CameraMode>,
    mut camera_query: Query<&mut Transform, With<MainCamera>>,
) {
    if mode.0 != FollowingType::Free {
        mouse_motion.clear();
        return;
    }

    let dt = time.delta_secs();
    let move_speed = 10.0;
    let look_speed = 0.002;

    for mut transform in camera_query.iter_mut() {
        // 移动控制
        let forward = transform.forward();
        let right = transform.right();

        if keyboard.pressed(KeyCode::KeyW) {
            transform.translation += forward * move_speed * dt;
        }
        if keyboard.pressed(KeyCode::KeyS) {
            transform.translation -= forward * move_speed * dt;
        }
        if keyboard.pressed(KeyCode::KeyD) {
            transform.translation += right * move_speed * dt;
        }
        if keyboard.pressed(KeyCode::KeyA) {
            transform.translation -= right * move_speed * dt;
        }
        if keyboard.pressed(KeyCode::Space) {
            transform.translation += Vec3::Y * move_speed * dt;
        }
        if keyboard.pressed(KeyCode::ShiftLeft) {
            transform.translation -= Vec3::Y * move_speed * dt;
        }

        // 视角控制
        for event in mouse_motion.read() {
            let (mut yaw, mut pitch, _) = transform.rotation.to_euler(EulerRot::YXZ);
            yaw -= event.delta.x * look_speed;
            pitch -= event.delta.y * look_speed;
            pitch = pitch.clamp(-1.5, 1.5);
            transform.rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, 0.0);
        }
    }
}

fn screenshot_on_f2(mut commands: Commands, keyboard: Res<ButtonInput<KeyCode>>) {
    if keyboard.just_pressed(KeyCode::F2) {
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis();
        let path = format!("screenshot_{}.png", timestamp);

        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));

        info!(
            "Screenshot saved to {}",
            format!("screenshot_{}.png", timestamp)
        );
    }
}

fn screenshot_saving(// mut commands: Commands,
    // screenshots: Query<(Entity, &Screenshot, &Capturing)>,
) {
    // for (entity, screenshot, capturing) in screenshots.iter() {
    //     if capturing.is_capturing() {
    //         continue;
    //     }
    //     let path = format!(
    //         "screenshot_{}.png",
    //         std::time::SystemTime::now()
    //             .duration_since(std::time::UNIX_EPOCH)
    //             .unwrap()
    //             .as_millis()
    //     );
    //     if let Err(e) = save_to_disk(screenshot.clone(), &path) {
    //         error!("Failed to save screenshot: {}", e);
    //     } else {
    //         info!("Screenshot saved to {}", path);
    //     }
    //     commands.entity(entity).remove::<Screenshot>();
    // }
}
