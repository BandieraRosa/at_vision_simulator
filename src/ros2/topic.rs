use bevy::prelude::Resource;
use r2r::geometry_msgs::msg::PoseStamped;
use r2r::sensor_msgs::msg::{CameraInfo, Image};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::WrappedTypesupport;
use std::sync::mpsc::SyncSender;

#[derive(Resource)]
pub struct TopicPublisher<T: RosTopic>(SyncSender<T::T>);

impl<T: RosTopic> TopicPublisher<T> {
    pub fn new(sender: SyncSender<T::T>) -> Self {
        TopicPublisher(sender)
    }

    pub fn publish(&self, message: T::T) {
        self.0.send(message).unwrap();
    }
}

#[macro_export]
macro_rules! publisher {
    ($atomic:expr, $app:ident, $node:ident, $topic:ty) => {
        let atomic = $atomic.clone();

        let (sender, receiver): (
            ::std::sync::mpsc::SyncSender<<$topic as crate::ros2::topic::RosTopic>::T>,
            ::std::sync::mpsc::Receiver<<$topic as crate::ros2::topic::RosTopic>::T>,
        ) = ::std::sync::mpsc::sync_channel(1024);

        let publisher = $node
            .create_publisher(
                <$topic>::TOPIC,
                ::r2r::QosProfile::default().lifespan(::std::time::Duration::from_secs_f64(1.0)),
            )
            .unwrap();

        $app.insert_resource(crate::ros2::topic::TopicPublisher::<$topic>::new(sender.clone()));

        ::std::thread::spawn(move || {
            while !atomic.load(::std::sync::atomic::Ordering::Acquire) {
                let mut did_work = false;
                loop {
                    match receiver.try_recv() {
                        Ok(m) => {
                            let mut sent = false;
                            while !sent {
                                match publisher.publish(&m) {
                                    Ok(_) => sent = true,
                                    Err(_) => {
                                        let _ = receiver.try_recv();
                                    }
                                }
                            }
                            did_work = true;
                        }
                        Err(::std::sync::mpsc::TryRecvError::Empty) => break,
                        Err(::std::sync::mpsc::TryRecvError::Disconnected) => break,
                    }
                }
                if !did_work {
                    ::std::thread::sleep(::std::time::Duration::from_millis(1));
                }
            }
        });
    };

    ($atomic:expr, $app:ident, $node:ident, $($topic:ty),* $(,)?) => {
        $(
            publisher!($atomic, $app, $node, $topic);
        )*
    };
}

pub trait RosTopic {
    type T: WrappedTypesupport + 'static;
    const TOPIC: &'static str;
}

macro_rules! define_topic {
    ($topic:ident, $typ:ty, $url:expr) => {
        pub struct $topic;
        impl RosTopic for $topic {
            type T = $typ;
            const TOPIC: &'static str = $url;
        }
    };
}

define_topic!(CameraInfoTopic, CameraInfo, "/camera_info");
define_topic!(ImageRawTopic, Image, "/image_raw");
define_topic!(GlobalTransformTopic, TFMessage, "/tf");

define_topic!(GimbalPoseTopic, PoseStamped, "/gimbal_pose");
define_topic!(OdomPoseTopic, PoseStamped, "/odom_pose");
define_topic!(CameraPoseTopic, PoseStamped, "/camera_pose");
