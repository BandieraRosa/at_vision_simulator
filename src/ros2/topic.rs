use bevy::prelude::Resource;
use r2r::geometry_msgs::msg::PoseStamped;
use r2r::sensor_msgs::msg::{CameraInfo, CompressedImage, Image};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{QosProfile, WrappedTypesupport};
use std::sync::mpsc::SyncSender;

#[derive(Resource)]
pub struct TopicPublisher<T: RosTopic> {
    sender: SyncSender<T::T>,
}

impl<T: RosTopic> TopicPublisher<T> {
    pub(crate) fn new(sender: SyncSender<T::T>) -> Self {
        TopicPublisher { sender }
    }

    pub fn publish(&self, message: T::T) {
        let _ = self.sender.try_send(message);
    }
}

#[macro_export]
macro_rules! publisher {
    ($node:ident,$topic:ty) => {
        {
            let (sender, receiver): (
                ::std::sync::mpsc::SyncSender<<$topic as crate::ros2::topic::RosTopic>::T>,
                ::std::sync::mpsc::Receiver<<$topic as crate::ros2::topic::RosTopic>::T>,
            ) = ::std::sync::mpsc::sync_channel(1024);

            let publisher = $node.create_publisher(
                <$topic>::TOPIC,
                <$topic>::QOS,
            ).unwrap();

            (receiver, sender, publisher)
        }
    };
    ($atomic:expr, $node:ident, $topic:ty) => {{
        let atomic = $atomic.clone();
        let (receiver,sender,publisher) = publisher!($node, $topic);
        ::std::thread::spawn(move || {
            while !atomic.load(::std::sync::atomic::Ordering::Acquire) {
                let mut did_work = false;
                loop {
                    match receiver.recv_timeout(Duration::from_secs(1)) {
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
                        Err(::std::sync::mpsc::RecvTimeoutError::Timeout) => continue,
                        Err(::std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
                    }
                }
                if !did_work {
                    ::std::thread::sleep(::std::time::Duration::from_millis(1));
                }
            }
        });
        crate::ros2::topic::TopicPublisher::<$topic>::new(sender)
    }};

    ($atomic:expr, $app:ident, $node:ident, $($topic:ty),* $(,)?) => {
        $(
            $app.insert_resource(publisher!($atomic, $node, $topic));
        )*
    };
}

pub trait RosTopic {
    type T: WrappedTypesupport + 'static;
    const TOPIC: &'static str;
    const QOS: QosProfile;
}

macro_rules! define_topic {
    ($topic:ident, $typ:ty, $url:expr, $qos:expr) => {
        pub struct $topic;
        impl RosTopic for $topic {
            type T = $typ;
            const TOPIC: &'static str = $url;
            const QOS: QosProfile = $qos;
        }
    };
    ($topic:ident, $typ:ty, $url:expr) => {
        define_topic!($topic, $typ, $url, ::r2r::QosProfile::default());
    };
}

define_topic!(CameraInfoTopic, CameraInfo, "/camera_info");
define_topic!(ImageRawTopic, Image, "/image_raw");
define_topic!(ImageCompressedTopic, CompressedImage, "/image_compressed");
define_topic!(GlobalTransformTopic, TFMessage, "/tf");

define_topic!(GimbalPoseTopic, PoseStamped, "/gimbal_pose");
define_topic!(OdomPoseTopic, PoseStamped, "/odom_pose");
define_topic!(CameraPoseTopic, PoseStamped, "/camera_pose");
