use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, ScreenshotCaptured},
};
use std::time::Duration;
use std::{
    f32::consts::PI,
    sync::{
        atomic::{AtomicBool, Ordering}, Arc,
        Mutex,
    },
    thread::{self, JoinHandle},
};

use crate::{
    robomaster::power_rune::{PowerRune, RuneIndex}, InfantryGimbal, InfantryRoot, InfantryViewOffset,
    LocalInfantry,
};
use r2r::sensor_msgs::msg::JointState;
use r2r::ClockType::RosTime;
use r2r::{
    geometry_msgs::msg::TransformStamped, sensor_msgs::msg::{CameraInfo, Image, RegionOfInterest}, std_msgs::msg::Header, tf2_msgs::msg::TFMessage, Clock, Context,
    Node,
    Publisher,
    QosProfile,
    WrappedTypesupport,
};

macro_rules! arc_mutex {
    ($elem:expr) => {
        ::std::sync::Arc::new(::std::sync::Mutex::new($elem))
    };
}

macro_rules! bevy_transform_ros2 {
    ($rotation:expr) => {
        (($rotation) * Quat::from_euler(EulerRot::YXZ, 0.0, PI, 0.0))
    };
}

macro_rules! bevy_xyzw {
    ($quat:expr) => {
        r2r::geometry_msgs::msg::Quaternion {
            x: $quat.x as f64,
            y: $quat.y as f64,
            z: $quat.z as f64,
            w: $quat.w as f64,
        }
    };
}

macro_rules! bevy_rot {
    ($rotation:expr) => {
        bevy_xyzw!(bevy_transform_ros2!($rotation))
    };
}

macro_rules! bevy_xyz {
    ($t:ty, $translation:expr) => {{
        let mut tmp: $t = ::std::default::Default::default();
        tmp.x = $translation.x as f64;
        tmp.y = $translation.y as f64;
        tmp.z = $translation.z as f64;
        tmp
    }};
    ($translation:expr) => {
        bevy_xyz!(r2r::geometry_msgs::msg::Vector3, $translation)
    };
}

macro_rules! bevy_transform {
    ($transform:expr) => {
        r2r::geometry_msgs::msg::Transform {
            translation: bevy_xyz!($transform.translation()),
            rotation: bevy_rot!($transform.rotation()),
        }
    };
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

#[derive(Resource)]
pub struct SensorPublisher<T: WrappedTypesupport>(pub Arc<Mutex<Publisher<T>>>);

fn capture_power_rune(
    clock: ResMut<RoboMasterClock>,
    runes: Query<(&GlobalTransform, &PowerRune)>,
    targets: Query<(&GlobalTransform, &RuneIndex), Changed<GlobalTransform>>,
    tf_publisher: Res<SensorPublisher<TFMessage>>,
) {
    let mut ls = vec![];
    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());

    for (transform, rune) in runes {
        ls.push(TransformStamped {
            header: Header {
                stamp: stamp.clone(),
                frame_id: "map".to_string(),
            },
            child_frame_id: format!("power_rune_{:?}", rune.mode)
                .to_string()
                .to_lowercase(),
            transform: bevy_transform!(transform),
        });
    }
    for (target_transform, target) in targets {
        if let Ok((_rune_transform, rune)) = runes.get(target.1) {
            ls.push(TransformStamped {
                header: Header {
                    stamp: stamp.clone(),
                    frame_id: "map".to_string(),
                },
                child_frame_id: format!("power_rune_{:?}_{:?}", rune.mode, target.0)
                    .to_string()
                    .to_lowercase(),
                transform: bevy_transform!(target_transform),
            });
        }
    }
    res_unwrap!(tf_publisher)
        .publish(&TFMessage { transforms: ls })
        .unwrap();
}

fn capture_local_infantry(
    _infantry: Single<&Transform, (With<InfantryRoot>, With<LocalInfantry>)>,
    gimbal: Single<&GlobalTransform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    _view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,
) {
}

fn capture_frame(mut commands: Commands, _input: Res<ButtonInput<KeyCode>>) {
    commands.spawn(Screenshot::primary_window()).observe(
        |ev: On<ScreenshotCaptured>,
         perspective: Single<&Projection, With<MainCamera>>,

         infantry: Single<&Transform, (With<InfantryRoot>, With<LocalInfantry>)>,
         gimbal: Single<&Transform, (With<LocalInfantry>, With<InfantryGimbal>)>,
         view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,

         clock: ResMut<RoboMasterClock>,
         info_publisher: Res<SensorPublisher<CameraInfo>>,
         tf_publisher: Res<SensorPublisher<TFMessage>>,
         image_publisher: Res<SensorPublisher<Image>>| {
            let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
            let hdr = Header {
                stamp: stamp.clone(),
                frame_id: "camera_optical_frame".to_string(),
            };
            let img = ev.image.clone();

            let (camera_info, image) = compute_camera(*perspective, &hdr, img);
            let translation = infantry.translation
                + (infantry.rotation * gimbal.rotation) * view_offset.0.translation;
            let rotation = infantry.rotation * gimbal.rotation;

            res_unwrap!(tf_publisher)
                .publish(&TFMessage {
                    transforms: vec![TransformStamped {
                        header: Header {
                            stamp: stamp.clone(),
                            frame_id: "map".to_string(),
                        },
                        child_frame_id: "camera_optical_frame".to_string(),
                        transform: r2r::geometry_msgs::msg::Transform {
                            translation: bevy_xyz!(translation),
                            rotation: bevy_rot!(rotation),
                        },
                    }],
                })
                .unwrap();
            res_unwrap!(info_publisher).publish(&camera_info).unwrap();
            res_unwrap!(image_publisher).publish(&image).unwrap();
        },
    );
}

fn compute_camera(
    perspective: &Projection,
    hdr: &Header,
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

    (
        CameraInfo {
            header: hdr.clone(),
            height,
            width,
            distortion_model: "none".to_string(),
            d: vec![0.000, 0.000, 0.000, 0.000, 0.000],
            k: vec![f_x, 0.0, c_x, 0.0, f_y, c_y, 0.0, 0.0, 1.0],
            r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p: vec![f_x, 0.0, c_x, 0.0, 0.0, f_y, c_y, 0.0, 0.0, 0.0, 1.0, 0.0],
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
            header: hdr.clone(),
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

macro_rules! publisher {
    ($app:ident,$node:ident,$t:ty,$topic:expr) => {
        $app.insert_resource(SensorPublisher(arc_mutex!(
            $node
                .create_publisher::<$t>($topic, QosProfile::default())
                .unwrap()
        )));
    };
}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        let mut node = Node::create(Context::create().unwrap(), "simulator", "robomaster").unwrap();
        let signal_arc = Arc::new(AtomicBool::new(false));

        publisher!(app, node, CameraInfo, "/camera_info");
        publisher!(app, node, Image, "/image_raw");
        publisher!(app, node, TFMessage, "/tf");
        publisher!(app, node, JointState, "/joint_states");

        app.insert_resource(RoboMasterClock(arc_mutex!(Clock::create(RosTime).unwrap())))
            .insert_resource(StopSignal(signal_arc.clone()))
            .add_systems(Update, capture_frame)
            .add_systems(Update, capture_power_rune)
            .add_systems(Last, cleanup_ros2_system)
            .insert_resource(SpinThreadHandle(Some(thread::spawn(move || {
                while !signal_arc.load(Ordering::Acquire) {
                    node.spin_once(Duration::from_millis(10));
                }
            }))));
    }
}
