use bevy::prelude::{error, Resource};
use r2r::geometry_msgs::msg::PoseStamped;
use r2r::qos::{DurabilityPolicy, HistoryPolicy, LivelinessPolicy, ReliabilityPolicy};
use r2r::sensor_msgs::msg::{CameraInfo, CompressedImage, Image};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{QosProfile, WrappedTypesupport};
use std::sync::mpsc::SyncSender;
use std::time::{Duration, Instant};

#[derive(Resource)]
pub struct TopicPublisher<T: RosTopic> {
    sender: SyncSender<T::T>,
    start: Instant,
    interval: Duration,
    count: u32,
}

impl<T: RosTopic> TopicPublisher<T> {
    pub(crate) fn new(sender: SyncSender<T::T>, freq: f64) -> Self {
        let interval = Duration::from_secs_f64(1.0 / freq);
        TopicPublisher {
            sender,
            start: Instant::now(),
            interval,
            count: 0,
        }
    }

    pub fn publish(&mut self, message: T::T) {
        let now = Instant::now();
        let ideal_time = self.start + self.interval * self.count;
        if now >= ideal_time {
            if let Err(err)=self.sender.try_send(message){
                error!("Topic send error {:?}", err);
            }
            self.count += 1;
            if self.count > 1 << 30 {
                self.count = 0;
                self.start = now;
            }
        }
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
    ($atomic:expr, $app:ident, $node:ident, $topic:ty) => {
        let atomic = $atomic.clone();
        let (receiver,sender,publisher) = publisher!($node, $topic);
        $app.insert_resource(crate::ros2::topic::TopicPublisher::<$topic>::new(sender, <$topic>::FREQUENCY));

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
    const FREQUENCY: f64;
    const QOS: QosProfile;
}

macro_rules! define_topic {
    ($topic:ident, $typ:ty, $url:expr, $freq:expr, $qos:expr) => {
        pub struct $topic;
        impl RosTopic for $topic {
            type T = $typ;
            const TOPIC: &'static str = $url;
            const FREQUENCY: f64 = $freq;
            const QOS: QosProfile = $qos;
        }
    };
    ($topic:ident, $typ:ty, $url:expr, $freq:expr) => {
        define_topic!($topic, $typ, $url, $freq, ::r2r::QosProfile::default());
    };
}

const SENSOR_QOS: QosProfile = QosProfile {
    history: HistoryPolicy::KeepLast,
    depth: 10,
    reliability: ReliabilityPolicy::BestEffort,
    durability: DurabilityPolicy::Volatile,
    deadline: Duration::ZERO,
    lifespan: Duration::ZERO,
    liveliness: LivelinessPolicy::Automatic,
    liveliness_lease_duration: Duration::ZERO,
    avoid_ros_namespace_conventions: false,
};

define_topic!(CameraInfoTopic, CameraInfo, "/camera_info", 60.0);
define_topic!(ImageRawTopic, Image, "/image_raw", 60.0);
define_topic!(
    ImageCompressedTopic,
    CompressedImage,
    "/image_compressed",
    30.0
);
define_topic!(GlobalTransformTopic, TFMessage, "/tf", 60.0);

define_topic!(GimbalPoseTopic, PoseStamped, "/gimbal_pose", 60.0);
define_topic!(OdomPoseTopic, PoseStamped, "/odom_pose", 60.0);
define_topic!(CameraPoseTopic, PoseStamped, "/camera_pose", 60.0);
