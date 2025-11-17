use bevy::prelude::Resource;
use r2r::geometry_msgs::msg::{Pose, PoseStamped};
use r2r::sensor_msgs::msg::{CameraInfo, Image};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{Publisher, WrappedTypesupport};
use std::sync::{Arc, Mutex};

#[derive(Resource)]
pub struct TopicPublisher<T: RosTopic>(Arc<Mutex<Publisher<T::T>>>);

impl<T: RosTopic> TopicPublisher<T> {
    pub fn new(publisher: Publisher<T::T>) -> Self {
        TopicPublisher(Arc::new(Mutex::new(publisher)))
    }

    pub fn publish(&self, message: T::T) {
        self.0.lock().unwrap().publish(&message).unwrap();
    }
}

#[macro_export]
macro_rules! publisher {
    ($app:ident,$node:ident,$topic:ty) => {
        $app.insert_resource(crate::ros2::topic::TopicPublisher::<$topic>::new(
            $node.create_publisher(<$topic>::TOPIC, ::r2r::QosProfile::default().lifespan(::std::time::Duration::from_secs_f64(1.0))).unwrap()
        ));
    };

    ($app:ident,$node:ident,$($topic:ty),* $(,)?) => {
        $(
            publisher!($app,$node,$topic);
        )*
    }
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
