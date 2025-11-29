use crate::ros2::capture::{CaptureConfig, RosCaptureContext, RosCapturePlugin};
use crate::ros2::topic::*;
use crate::{
    InfantryGimbal, InfantryViewOffset, LocalInfantry, arc_mutex, publisher,
    robomaster::power_rune::{PowerRune, RuneIndex},
    subscriber,
};
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use r2r::ClockType::SystemTime;
use r2r::geometry_msgs::msg::{Pose, PoseStamped, QuaternionStamped};
use r2r::std_msgs::msg::Float64;
use r2r::{Clock, Context, Node, std_msgs::msg::Header, tf2_msgs::msg::TFMessage};
use std::f32::consts::PI;
use std::time::Duration;
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
};

pub const M_ALIGN_MAT3: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, -1.0, 0.0), // M[0,0], M[1,0], M[2,0]
    Vec3::new(0.0, 0.0, 1.0),  // M[0,1], M[1,1], M[2,1]
    Vec3::new(-1.0, 0.0, 0.0), // M[0,2], M[1,2], M[2,2]
);

#[inline]
pub fn transform(bevy_transform: Transform) -> r2r::geometry_msgs::msg::Transform {
    let align_rot_mat = M_ALIGN_MAT3;
    let align_quat = Quat::from_mat3(&align_rot_mat);
    let new_rotation = align_quat * bevy_transform.rotation * align_quat.inverse();
    let new_translation = align_rot_mat * bevy_transform.translation;
    r2r::geometry_msgs::msg::Transform {
        translation: r2r::geometry_msgs::msg::Vector3 {
            x: new_translation.x as f64,
            y: new_translation.y as f64,
            z: new_translation.z as f64,
        },
        rotation: r2r::geometry_msgs::msg::Quaternion {
            x: new_rotation.x as f64,
            y: new_rotation.y as f64,
            z: new_rotation.z as f64,
            w: new_rotation.w as f64,
        },
    }
}

/// 将 Bevy 四元数转换为 ROS2 四元数
#[inline]
pub fn quaternion_to_ros(bevy_quat: Quat) -> r2r::geometry_msgs::msg::Quaternion {
    let align_rot_mat = M_ALIGN_MAT3;
    let align_quat = Quat::from_mat3(&align_rot_mat);
    let new_rotation = align_quat * bevy_quat * align_quat.inverse();
    r2r::geometry_msgs::msg::Quaternion {
        x: new_rotation.x as f64,
        y: new_rotation.y as f64,
        z: new_rotation.z as f64,
        w: new_rotation.w as f64,
    }
}

macro_rules! res_unwrap {
    ($res:tt) => {
        $res.0.lock().unwrap()
    };
}

#[derive(Resource)]
struct StopSignal(Arc<AtomicBool>);

#[derive(Resource)]
struct SpinThreadHandle(Option<JoinHandle<()>>);

#[derive(Component)]
pub struct MainCamera;

#[derive(Resource)]
pub struct RoboMasterClock(pub Arc<Mutex<Clock>>);

/// 自瞄模式状态
#[derive(Resource, Default)]
pub struct AutoAimMode {
    pub enabled: bool,
}

#[derive(Resource, Default, Clone)]
pub struct TargetEuler {
    pub yaw: f32, // rad
    pub pitch: f32,
    pub valid: bool,
}

#[derive(Resource, Default)]
pub struct FireCommand {
    pub fire: bool,
}

#[derive(Resource)]
pub struct ProjectileVelocity {
    pub speed: f32, // m/s
}

impl Default for ProjectileVelocity {
    fn default() -> Self {
        Self { speed: 20.0 }
    }
}

#[macro_export]
macro_rules! add_tf_frame {
    ($ls:ident, $hdr:expr, $id:expr, $translation:expr, $rotation:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: crate::ros2::plugin::transform(
                Transform::IDENTITY
                    .with_translation($translation)
                    .with_rotation($rotation),
            ),
        });
    };
    ($ls:ident, $hdr:expr, $id:expr, $transform:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: crate::ros2::plugin::transform($transform),
        });
    };
}

#[macro_export]
macro_rules! pose {
    ($hdr:expr) => {
        PoseStamped {
            header: $hdr.clone(),
            pose: Pose {
                position: r2r::geometry_msgs::msg::Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    };
}

fn process_subscriptions(
    target_euler_sub: Option<Res<TopicSubscriber<TargetEulerTopic>>>,
    fire_notify_sub: Option<Res<TopicSubscriber<FireNotifyTopic>>>,
    mut target_euler: ResMut<TargetEuler>,
    mut fire_command: ResMut<FireCommand>,
    auto_aim: Res<AutoAimMode>,
) {
    if !auto_aim.enabled {
        target_euler.valid = false;
        fire_command.fire = false;
        return;
    }

    if let Some(sub) = target_euler_sub {
        if let Some(msg) = sub.get_latest() {
            target_euler.yaw = msg.vector.x as f32;
            target_euler.pitch = msg.vector.y as f32;
            target_euler.valid = true;
        }
    }

    if let Some(sub) = fire_notify_sub {
        if let Some(msg) = sub.get_latest() {
            fire_command.fire = msg.data;
        }
    }
}

fn publish_gimbal_quaternion(
    gimbal: Single<&GlobalTransform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    clock: Res<RoboMasterClock>,
    gimbal_quat_pub: Res<TopicPublisher<GimbalQuaternionTopic>>,
) {
    let gimbal_transform = gimbal.into_inner();
    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());

    let header = Header {
        stamp,
        frame_id: "odom".to_string(),
    };

    let quat_msg = QuaternionStamped {
        header,
        quaternion: quaternion_to_ros(gimbal_transform.rotation()),
    };

    gimbal_quat_pub.publish(quat_msg);
}

fn publish_projectile_velocity(
    velocity: Res<ProjectileVelocity>,
    velocity_pub: Res<TopicPublisher<CurrentVelocityTopic>>,
) {
    let msg = Float64 {
        data: velocity.speed as f64,
    };
    velocity_pub.publish(msg);
}

fn capture_rune(
    camera: Single<&GlobalTransform, With<MainCamera>>,
    gimbal: Single<&GlobalTransform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    _view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,

    runes: Query<(&GlobalTransform, &PowerRune)>,
    targets: Query<(&GlobalTransform, &RuneIndex, &Name)>,

    clock: ResMut<RoboMasterClock>,
    tf_publisher: ResMut<TopicPublisher<GlobalTransformTopic>>,
    gimbal_pose_pub: ResMut<TopicPublisher<GimbalPoseTopic>>,
    odom_pose_pub: ResMut<TopicPublisher<OdomPoseTopic>>,
    camera_pose_pub: ResMut<TopicPublisher<CameraPoseTopic>>,
) {
    let cam_transform = camera.into_inner();
    let gimbal_transform = gimbal.into_inner();
    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    let mut transform_stamped = vec![];
    let map_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "map".to_string(),
    };
    let odom_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "odom".to_string(),
    };
    // change: add gimbal_odom
    let gimbal_odom_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "gimbal_odom".to_string(),
    };
    let yaw_link_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "yaw_link".to_string(),
    };
    let pitch_link_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "pitch_link".to_string(),
    };
    let camera_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "camera_link".to_string(),
    };

    gimbal_pose_pub.publish(pose!(gimbal_odom_hdr));
    odom_pose_pub.publish(pose!(odom_hdr));
    camera_pose_pub.publish(pose!(camera_hdr));
    add_tf_frame!(
        transform_stamped,
        map_hdr.clone(),
        "odom",
        gimbal_transform.translation(),
        Quat::IDENTITY
    );
    // change: add gimbal_odom frame
    add_tf_frame!(
        transform_stamped,
        odom_hdr.clone(),
        "gimbal_odom",
        Vec3::ZERO,
        Quat::IDENTITY
    );
    let gimbal_rotation = gimbal_transform.rotation();
    let (yaw, pitch, _roll) = gimbal_rotation.to_euler(EulerRot::YXZ);

    // add yaw_link
    add_tf_frame!(
        transform_stamped,
        gimbal_odom_hdr.clone(),
        "yaw_link",
        Vec3::ZERO,
        Quat::from_rotation_y(yaw)
    );

    // yaw_link -> pitch_link (-y轴)
    add_tf_frame!(
        transform_stamped,
        yaw_link_hdr.clone(),
        "pitch_link",
        Vec3::ZERO,
        Quat::from_rotation_x(pitch)  // 绕x轴旋转 pitch
    );

    // pitch_link -> camera_link
    let cam = cam_transform.reparented_to(gimbal_transform);
    add_tf_frame!(
        transform_stamped,
        pitch_link_hdr.clone(),
        "camera_link",
        cam.translation,
        cam.rotation
    );

    // camera_link -> camera_optical_frame
    add_tf_frame!(
        transform_stamped,
        camera_hdr.clone(),
        "camera_optical_frame",
        Vec3::ZERO,
        Quat::from_euler(EulerRot::ZYX, -PI / 2.0, PI, PI / 2.0)
    );

    // 保留原有的 gimbal_link
    let gimbal_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "gimbal_link".to_string(),
    };
    add_tf_frame!(
        transform_stamped,
        odom_hdr.clone(),
        "gimbal_link",
        Vec3::ZERO,
        gimbal_transform.rotation()
    );
    add_tf_frame!(
        transform_stamped,
        gimbal_hdr.clone(),
        "camera_link",
        cam.translation,
        cam.rotation
    );

    for (transform, rune) in runes {
        add_tf_frame!(
            transform_stamped,
            map_hdr.clone(),
            format!("power_rune_{:?}", rune.mode)
                .to_string()
                .to_lowercase(),
            transform.compute_transform()
        );
    }
    for (target_transform, target, name) in targets {
        if !name.contains("_ACTIVATED") {
            continue;
        }
        if let Ok((_rune_transform, rune)) = runes.get(target.1) {
            add_tf_frame!(
                transform_stamped,
                Header {
                    stamp: stamp.clone(),
                    frame_id: format!("power_rune_{:?}", rune.mode)
                        .to_string()
                        .to_lowercase(),
                },
                format!("power_rune_{:?}_{:?}", rune.mode, target.0)
                    .to_string()
                    .to_lowercase(),
                target_transform.reparented_to(_rune_transform)
            );
        }
    }
    tf_publisher.publish(TFMessage {
        transforms: transform_stamped,
    });
}

fn cleanup_ros2_system(
    mut exit: MessageReader<AppExit>,
    stop_signal: Res<StopSignal>,
    mut handle_res: ResMut<SpinThreadHandle>,
) {
    if exit.read().len() > 0 {
        stop_signal.0.store(true, Ordering::Release);
        if let Some(handle) = handle_res.0.take() {
            info!("Waiting for ROS 2 spin thread to join...");
            match handle.join() {
                Ok(_) => info!("ROS 2 thread successfully joined. Safe to exit."),
                Err(_) => error!("WARNING: ROS 2 thread panicked or failed to join."),
            }
        }
    }
}

#[derive(Default)]
pub struct ROS2Plugin {}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        let mut node = Node::create(Context::create().unwrap(), "simulator", "robomaster").unwrap();
        let signal_arc = Arc::new(AtomicBool::new(false));

        // 创建发布者
        publisher!(
            signal_arc,
            app,
            node,
            GlobalTransformTopic,
            GimbalPoseTopic,
            OdomPoseTopic,
            CameraPoseTopic,
            GimbalQuaternionTopic,
            CurrentVelocityTopic
        );

        let camera_info = Arc::new(publisher!(signal_arc, node, CameraInfoTopic));
        let image_raw = Arc::new(publisher!(signal_arc, node, ImageRawTopic));
        let image_compressed = Arc::new(publisher!(signal_arc, node, ImageCompressedTopic));

        subscriber!(signal_arc, app, node, TargetEulerTopic, FireNotifyTopic);

        let clock = arc_mutex!(Clock::create(SystemTime).unwrap());

        app.insert_resource(RoboMasterClock(clock.clone()))
            .insert_resource(StopSignal(signal_arc.clone()))
            .insert_resource(AutoAimMode::default())
            .insert_resource(TargetEuler::default())
            .insert_resource(FireCommand::default())
            .insert_resource(ProjectileVelocity::default())
            .add_plugins(RosCapturePlugin {
                config: CaptureConfig {
                    width: 1440,
                    height: 1080,
                    texture_format: TextureFormat::bevy_default(),
                    fov_y: PI / 180.0 * 45.0,
                },
                context: RosCaptureContext {
                    clock,
                    camera_info,
                    image_raw,
                    image_compressed,
                },
            })
            .add_systems(Last, cleanup_ros2_system)
            .add_systems(
                Update,
                (
                    process_subscriptions,
                    capture_rune.after(TransformSystems::Propagate),
                    publish_gimbal_quaternion.after(TransformSystems::Propagate),
                    publish_projectile_velocity,
                ),
            )
            .insert_resource(SpinThreadHandle(Some(thread::spawn(move || {
                while !signal_arc.load(Ordering::Acquire) {
                    node.spin_once(Duration::from_millis(1));
                }
            }))));
    }
}
