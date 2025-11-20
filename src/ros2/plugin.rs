use crate::ros2::capture::{CaptureConfig, Captured, RosCapturePlugin};
use crate::ros2::topic::*;
use crate::{
    arc_mutex, publisher, robomaster::power_rune::{PowerRune, RuneIndex}, InfantryGimbal, InfantryRoot, InfantryViewOffset,
    LocalInfantry,
};
use bevy::prelude::*;
use r2r::geometry_msgs::msg::{Pose, PoseStamped};
use r2r::ClockType::SystemTime;
use r2r::{
    sensor_msgs::msg::{CameraInfo, Image, RegionOfInterest}, std_msgs::msg::Header, tf2_msgs::msg::TFMessage,
    Clock,
    Context,
    Node,
};
use std::f32::consts::PI;
use std::time::{Duration, UNIX_EPOCH};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering}, Arc,
        Mutex,
    },
    thread::{self, JoinHandle},
};

const M_ALIGN_MAT3: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, -1.0, 0.0), // M[0,0], M[1,0], M[2,0]
    Vec3::new(0.0, 0.0, 1.0),  // M[0,1], M[1,1], M[2,1]
    Vec3::new(-1.0, 0.0, 0.0), // M[0,2], M[1,2], M[2,2]
);

#[inline]
fn transform(bevy_transform: Transform) -> ::r2r::geometry_msgs::msg::Transform {
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

macro_rules! add_tf_frame {
    ($ls:ident, $hdr:expr, $id:expr, $translation:expr, $rotation:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: transform(
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
            transform: transform($transform),
        });
    };
}

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

fn capture_rune(
    camera: Single<(&GlobalTransform, &Projection), With<MainCamera>>,
    infantry: Single<&GlobalTransform, (With<InfantryRoot>, With<LocalInfantry>)>,
    gimbal: Single<&GlobalTransform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    _view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,

    runes: Query<(&GlobalTransform, &PowerRune)>,
    targets: Query<(&GlobalTransform, &RuneIndex, &Name)>,

    clock: ResMut<RoboMasterClock>,
    mut tf_publisher: ResMut<TopicPublisher<GlobalTransformTopic>>,
    mut gimbal_pose_pub: ResMut<TopicPublisher<GimbalPoseTopic>>,
    mut odom_pose_pub: ResMut<TopicPublisher<OdomPoseTopic>>,
    mut camera_pose_pub: ResMut<TopicPublisher<CameraPoseTopic>>,
) {
    let (cam_transform, perspective) = camera.into_inner();
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
    let gimbal_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "gimbal_link".to_string(),
    };
    let camera_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "camera_link".to_string(),
    };

    gimbal_pose_pub.publish(pose!(gimbal_hdr));
    odom_pose_pub.publish(pose!(odom_hdr));
    camera_pose_pub.publish(pose!(camera_hdr));

    add_tf_frame!(
        transform_stamped,
        map_hdr.clone(),
        "odom",
        gimbal.translation(),
        gimbal.rotation()
    );
    add_tf_frame!(
        transform_stamped,
        odom_hdr.clone(),
        "gimbal_link",
        Vec3::ZERO,
        Quat::IDENTITY
    );
    let cam_rel = cam_transform.reparented_to(gimbal.into_inner());
    add_tf_frame!(
        transform_stamped,
        gimbal_hdr.clone(),
        "camera_link",
        cam_rel.translation,
        cam_rel.rotation
    );
    add_tf_frame!(
        transform_stamped,
        camera_hdr.clone(),
        "camera_optical_frame",
        Vec3::ZERO,
        Quat::from_euler(EulerRot::ZYX, -PI / 2.0, PI, PI / 2.0)
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

fn capture_frame(
    ev: On<Captured>,
    camera: Single<(&GlobalTransform, &Projection), With<MainCamera>>,

    clock: ResMut<RoboMasterClock>,

    mut camera_info_pub: ResMut<TopicPublisher<CameraInfoTopic>>,
    mut image_raw_pub: ResMut<TopicPublisher<ImageRawTopic>>,
    mut image_compressed_pub: ResMut<TopicPublisher<ImageCompressedTopic>>,
) {
    let (cam_transform, perspective) = camera.into_inner();
    let eg = ev.time.duration_since(UNIX_EPOCH.into()).unwrap();
    let optical_frame_hdr = Header {
        stamp: r2r::builtin_interfaces::msg::Time {
            sec: eg.as_secs() as i32,
            nanosec: eg.subsec_nanos(),
        },
        frame_id: "camera_optical_frame".to_string(),
    };
    let img = ev.image.clone();
    //image_compressed_pub.publish(compress_image(optical_frame_hdr.clone(), &img));
    let (camera_info, image) = compute_camera(&perspective, optical_frame_hdr.clone(), img);
    camera_info_pub.publish(camera_info);
    image_raw_pub.publish(image);
}

fn compute_camera(
    perspective: &Projection,
    hdr: Header,
    img: bevy::image::Image,
) -> (CameraInfo, Image) {
    let dyn_img = img.try_into_dynamic().unwrap();
    let rgb8 = dyn_img.to_rgb8();
    let (width, height) = (rgb8.width(), rgb8.height());

    let (fov_y, fov_x) = match perspective {
        Projection::Perspective(p) => {
            let fov_y = p.fov as f64;
            let aspect = width as f64 / height as f64;
            let fov_x = 2.0 * ((fov_y / 2.0).tan() * aspect).atan();
            (fov_y, fov_x)
        }
        _ => todo!(),
    };

    let f_x = width as f64 / (2.0 * (fov_x / 2.0).tan());
    let f_y = height as f64 / (2.0 * (fov_y / 2.0).tan());

    let c_x = width as f64 / 2.0;
    let c_y = height as f64 / 2.0;

    // Removed x-axis flip; rely on optical rotation instead

    (
        CameraInfo {
            header: hdr.clone(),
            height,
            width,
            distortion_model: "plumb_bob".to_string(),
            d: vec![0.000, 0.000, 0.000, 0.000, 0.000],
            k: vec![f_x, 0.0, c_x, 0.0, f_y, c_y, 0.0, 0.0, 1.0],
            p: vec![f_x, 0.0, c_x, 0.0, 0.0, f_y, c_y, 0.0, 0.0, 0.0, 1.0, 0.0],
            r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            binning_x: 0,
            binning_y: 0,
            roi: RegionOfInterest {
                x_offset: 0,
                y_offset: 0,
                height,
                width,
                do_rectify: true,
            },
        },
        Image {
            header: hdr,
            height: rgb8.height(),
            width: rgb8.width(),
            encoding: "rgb8".to_string(),
            is_bigendian: 0,
            step: rgb8.width() * 3,
            data: rgb8.into_raw(),
        },
    )
}

fn cleanup_ros2_system(
    mut exit: MessageReader<AppExit>,
    stop_signal: Res<StopSignal>,
    mut handle_res: ResMut<SpinThreadHandle>,
) {
    if exit.read().len() > 0 {
        stop_signal.0.store(true, Ordering::Release);
        if let Some(handle) = handle_res.0.take() {
            println!("Waiting for ROS 2 spin thread to join...");
            match handle.join() {
                Ok(_) => println!("ROS 2 thread successfully joined. Safe to exit."),
                Err(_) => eprintln!("WARNING: ROS 2 thread panicked or failed to join."),
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
        publisher!(
            signal_arc,
            app,
            node,
            CameraInfoTopic,
            ImageRawTopic,
            ImageCompressedTopic,
            GlobalTransformTopic,
            GimbalPoseTopic,
            OdomPoseTopic,
            CameraPoseTopic
        );

        let clock = arc_mutex!(Clock::create(SystemTime).unwrap());

        app.insert_resource(RoboMasterClock(clock.clone()))
            .insert_resource(StopSignal(signal_arc.clone()))
            .add_plugins(RosCapturePlugin {
                config: CaptureConfig {
                    width: 1440,
                    height: 1080,
                    fov: PI / 180.0 * 45.0,
                },
            })
            .add_observer(capture_frame)
            .add_systems(Last, cleanup_ros2_system)
            .add_systems(Update, capture_rune.after(TransformSystems::Propagate))
            .insert_resource(SpinThreadHandle(Some(thread::spawn(move || {
                while !signal_arc.load(Ordering::Acquire) {
                    node.spin_once(Duration::from_millis(10));
                }
            }))));
    }
}
