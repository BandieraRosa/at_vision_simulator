use std::{
    f32::consts::PI,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
};

use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, ScreenshotCaptured},
};

use r2r::{
    Clock, Context, Node, Publisher, QosProfile, WrappedTypesupport,
    geometry_msgs::msg::TransformStamped,
    sensor_msgs::{
        self,
        msg::{CameraInfo, Image, RegionOfInterest},
    },
    std_msgs::msg::Header,
    tf2_msgs::msg::TFMessage,
};

use crate::{
    InfantryGimbal, InfantryRoot, InfantryViewOffset, LocalInfantry,
    robomaster::power_rune::{PowerRune, PowerRuneRoot, RuneIndex},
};

#[derive(Resource)]
struct StopSignal(Arc<AtomicBool>);

#[derive(Resource)]
struct SpinThreadHandle(Option<JoinHandle<()>>);

#[derive(Component)]
pub struct MainCamera;

#[derive(Component)]
struct CaptureCamera;

#[derive(Resource)]
pub struct RoboMasterClock(pub Arc<Mutex<Clock>>);

#[derive(Resource)]
pub struct SensorPublisher<T: WrappedTypesupport>(pub Arc<Mutex<Publisher<T>>>);

fn capture_power_rune(
    clock: ResMut<RoboMasterClock>,
    runes: Query<(&GlobalTransform, &PowerRune)>,
    targets: Query<(&GlobalTransform, &RuneIndex)>,
    tf_publisher: Res<SensorPublisher<TFMessage>>,
) {
    let mut ls = vec![];
    let stamp = Clock::to_builtin_time(&clock.0.lock().unwrap().get_now().unwrap());

    for (transform, rune) in runes {
        let translation = transform.translation();
        let rotation = transform.rotation();
        ls.push(TransformStamped {
            header: Header {
                stamp: stamp.clone(),
                frame_id: "map".to_string(),
            },
            child_frame_id: format!("power_rune_{:?}", rune.mode)
                .to_string()
                .to_lowercase(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: translation.x as f64,
                    y: translation.y as f64,
                    z: translation.z as f64,
                },
                rotation: r2r::geometry_msgs::msg::Quaternion {
                    x: rotation.x as f64,
                    y: rotation.y as f64,
                    z: rotation.z as f64,
                    w: rotation.w as f64,
                },
            },
        });
    }
    for (transform, target) in targets {
        if let Ok(rune) = runes.get(target.1) {
            let translation = transform.translation();
            let rotation = transform.rotation();
            ls.push(TransformStamped {
                header: Header {
                    stamp: stamp.clone(),
                    frame_id: format!("power_rune_{:?}", rune.1.mode)
                        .to_string()
                        .to_lowercase(),
                },
                child_frame_id: format!("power_rune_{:?}_{:?}", rune.1.mode, target.0)
                    .to_string()
                    .to_lowercase(),
                transform: r2r::geometry_msgs::msg::Transform {
                    translation: r2r::geometry_msgs::msg::Vector3 {
                        x: translation.x as f64,
                        y: translation.y as f64,
                        z: translation.z as f64,
                    },
                    rotation: r2r::geometry_msgs::msg::Quaternion {
                        x: rotation.x as f64,
                        y: rotation.y as f64,
                        z: rotation.z as f64,
                        w: rotation.w as f64,
                    },
                },
            });
        }
    }
    tf_publisher
        .0
        .lock()
        .unwrap()
        .publish(&TFMessage { transforms: ls })
        .unwrap();
}

fn capture_frame(mut commands: Commands, input: Res<ButtonInput<KeyCode>>) {
    // if input.just_pressed(KeyCode::KeyG) {
    commands.spawn(Screenshot::primary_window()).observe(
        |ev: On<ScreenshotCaptured>,
         perspective: Single<&Projection, With<MainCamera>>,
         infantry: Single<&GlobalTransform, (With<InfantryRoot>, With<LocalInfantry>)>,
         gimbal: Single<&Transform, (With<LocalInfantry>, With<InfantryGimbal>)>,
         view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,
         clock: ResMut<RoboMasterClock>,
         info_publisher: Res<SensorPublisher<CameraInfo>>,
         tf_publisher: Res<SensorPublisher<TFMessage>>,
         image_publisher: Res<SensorPublisher<Image>>| {
            let dyn_img = ev.image.clone().try_into_dynamic().unwrap();
            let rgb8 = dyn_img.to_rgb8();
            let (width, height) = (rgb8.width(), rgb8.height());

            let info_publisher = info_publisher.0.lock().unwrap();
            let tf_publisher = tf_publisher.0.lock().unwrap();
            let image_publisher = image_publisher.0.lock().unwrap();

            let mut clock = clock.0.lock().unwrap();
            let hdr = Header {
                stamp: Clock::to_builtin_time(&clock.get_now().unwrap()),
                frame_id: "camera_optical_frame".to_string(),
            };

            let translation = infantry.translation()
                + (infantry.rotation() * gimbal.rotation) * view_offset.0.translation;
            let rotation = infantry.rotation() * gimbal.rotation;
            let rotation = rotation * Quat::from_euler(EulerRot::YXZ, 0.0, PI, 0.0);

            let tf = TransformStamped {
                header: Header {
                    stamp: Clock::to_builtin_time(&clock.get_now().unwrap()),
                    frame_id: "map".to_string(),
                },
                child_frame_id: "camera_optical_frame".to_string(),
                transform: r2r::geometry_msgs::msg::Transform {
                    translation: r2r::geometry_msgs::msg::Vector3 {
                        x: translation.x as f64,
                        y: translation.y as f64,
                        z: translation.z as f64,
                    },
                    rotation: r2r::geometry_msgs::msg::Quaternion {
                        x: rotation.x as f64,
                        y: rotation.y as f64,
                        z: rotation.z as f64,
                        w: rotation.w as f64,
                    },
                },
            };

            let fov_x = std::f64::consts::PI / 180.0 * 75.0;
            let fov_y = 2.0 * ((height as f64 / width as f64) * (fov_x / 2.0).tan()).atan();

            let f_x = width as f64 / (2.0 * (fov_x / 2.0).tan());
            let f_y = height as f64 / (2.0 * (fov_y / 2.0).tan());

            let c_x = width as f64 / 2.0;
            let c_y = height as f64 / 2.0;

            tf_publisher
                .publish(&TFMessage {
                    transforms: vec![tf],
                })
                .unwrap();

            info_publisher
                .publish(&CameraInfo {
                    header: hdr.clone(),
                    height,
                    width,
                    distortion_model: "plumb_bob".to_string(),
                    d: vec![0.000, 0.000, 0.000, 0.000, 0.000],
                    k: vec![f_x, 0.0, c_x, 0.0, f_y, c_y, 0.0, 0.0, 1.0],
                    r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    p: vec![f_x, 0.0, c_x, 0.0, 0.0, f_y, c_y, 0.0, 0.0, 0.0, 1.0, 0.0],
                    binning_x: 0,
                    binning_y: 0,
                    roi: RegionOfInterest {
                        x_offset: 0,
                        y_offset: 0,
                        height: 0,
                        width: 0,
                        do_rectify: true,
                    },
                })
                .unwrap();
            image_publisher
                .publish(&Image {
                    header: hdr.clone(),
                    height: rgb8.height(),
                    width: rgb8.width(),
                    encoding: "rgb8".to_string(),
                    is_bigendian: 0,
                    step: rgb8.width() as u32 * 3,
                    data: rgb8.into_raw(),
                })
                .unwrap();
        },
    );
    // }
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

macro_rules! arc_mutex {
    ($elem:expr) => {
        std::sync::Arc::new(std::sync::Mutex::new($elem))
    };
}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        let mut node = Node::create(Context::create().unwrap(), "simulator", "robomaster").unwrap();
        let signal_arc = Arc::new(AtomicBool::new(false));
        let clock = Clock::create(r2r::ClockType::RosTime).unwrap();

        app.insert_resource(RoboMasterClock(arc_mutex!(clock)))
            .insert_resource(SensorPublisher(arc_mutex!(
                node.create_publisher::<sensor_msgs::msg::CameraInfo>(
                    "/camera_info",
                    QosProfile::default()
                )
                .unwrap()
            )))
            .insert_resource(SensorPublisher(arc_mutex!(
                node.create_publisher::<Image>("/image_raw", QosProfile::default())
                    .unwrap()
            )))
            .insert_resource(SensorPublisher(arc_mutex!(
                node.create_publisher::<TFMessage>("/tf", QosProfile::default())
                    .unwrap()
            )))
            .insert_resource(StopSignal(signal_arc.clone()))
            .add_systems(Update, (capture_frame, capture_power_rune))
            .add_systems(Last, cleanup_ros2_system);

        app.insert_resource(SpinThreadHandle(Some(thread::spawn(move || {
            while !signal_arc.load(Ordering::Acquire) {
                node.spin_once(std::time::Duration::from_millis(10));
            }
        }))));
    }
}
