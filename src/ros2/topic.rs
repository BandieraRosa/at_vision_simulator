use bevy::prelude::{Resource, Timer, TimerMode};
use r2r::geometry_msgs::msg::PoseStamped;
use r2r::sensor_msgs::msg::{CameraInfo, Image};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::WrappedTypesupport;
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
    pub fn new(sender: SyncSender<T::T>, freq: f64) -> Self {
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
            self.sender.send(message).unwrap();
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

        $app.insert_resource(crate::ros2::topic::TopicPublisher::<$topic>::new(sender.clone(), <$topic>::FREQUENCY));

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
}

macro_rules! define_topic {
    ($topic:ident, $typ:ty, $url:expr, $freq:expr) => {
        pub struct $topic;
        impl RosTopic for $topic {
            type T = $typ;
            const TOPIC: &'static str = $url;
            const FREQUENCY: f64 = $freq;
        }
    };
}

define_topic!(CameraInfoTopic, CameraInfo, "/camera_info", 0.1);
define_topic!(ImageRawTopic, Image, "/image_raw", 60.0);
define_topic!(GlobalTransformTopic, TFMessage, "/tf", 60.0);

define_topic!(GimbalPoseTopic, PoseStamped, "/gimbal_pose", 60.0);
define_topic!(OdomPoseTopic, PoseStamped, "/odom_pose", 60.0);
define_topic!(CameraPoseTopic, PoseStamped, "/camera_pose", 60.0);
