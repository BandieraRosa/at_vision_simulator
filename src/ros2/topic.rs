use bevy::prelude::Resource;
use r2r::geometry_msgs::msg::{PoseStamped, QuaternionStamped, Vector3Stamped};
use r2r::sensor_msgs::msg::{CameraInfo, CompressedImage, Image};
use r2r::std_msgs::msg::Bool;
use r2r::std_msgs::msg::Float64;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{QosProfile, WrappedTypesupport};
use std::sync::mpsc::{Receiver, SyncSender};
use std::sync::{Arc, Mutex};

// ==================== Publisher ====================

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

// ==================== Subscriber ====================

#[derive(Resource)]
pub struct TopicSubscriber<T: RosTopic> {
    receiver: Arc<Mutex<Receiver<T::T>>>,
}

impl<T: RosTopic> TopicSubscriber<T> {
    pub(crate) fn new(receiver: Receiver<T::T>) -> Self {
        TopicSubscriber {
            receiver: Arc::new(Mutex::new(receiver)),
        }
    }

    pub fn try_recv(&self) -> Option<T::T> {
        self.receiver.lock().ok()?.try_recv().ok()
    }

    /// 获取最新消息（丢弃旧消息）
    pub fn get_latest(&self) -> Option<T::T> {
        let receiver = self.receiver.lock().ok()?;
        let mut latest = None;
        while let Ok(msg) = receiver.try_recv() {
            latest = Some(msg);
        }
        latest
    }
}

// ==================== Publisher Macro ====================

#[macro_export]
macro_rules! publisher {
    ($node:ident,$topic:ty) => {{
        let (sender, receiver): (
            ::std::sync::mpsc::SyncSender<<$topic as crate::ros2::topic::RosTopic>::T>,
            ::std::sync::mpsc::Receiver<<$topic as crate::ros2::topic::RosTopic>::T>,
        ) = ::std::sync::mpsc::sync_channel(1024);

        let publisher = $node
            .create_publisher(<$topic>::TOPIC, <$topic>::QOS)
            .unwrap();

        (receiver, sender, publisher)
    }};
    ($atomic:expr, $node:ident, $topic:ty) => {{
        let atomic = $atomic.clone();
        let (receiver, sender, publisher) = publisher!($node, $topic);
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

// ==================== Subscriber Macro ====================

/// 创建一个 no-op Waker 用于非阻塞轮询 Stream
pub fn noop_waker() -> ::std::task::Waker {
    use ::std::task::{RawWaker, RawWakerVTable, Waker};

    const VTABLE: RawWakerVTable = RawWakerVTable::new(
        |_| RawWaker::new(::std::ptr::null(), &VTABLE),
        |_| {},
        |_| {},
        |_| {},
    );
    unsafe { Waker::from_raw(RawWaker::new(::std::ptr::null(), &VTABLE)) }
}

#[macro_export]
macro_rules! subscriber {
    ($node:ident, $topic:ty) => {{
        let (sender, receiver): (
            ::std::sync::mpsc::SyncSender<<$topic as crate::ros2::topic::RosTopic>::T>,
            ::std::sync::mpsc::Receiver<<$topic as crate::ros2::topic::RosTopic>::T>,
        ) = ::std::sync::mpsc::sync_channel(64);

        let subscription = $node
            .subscribe::<<$topic as crate::ros2::topic::RosTopic>::T>(
                <$topic>::TOPIC,
                <$topic>::QOS,
            )
            .unwrap();

        (sender, receiver, subscription)
    }};

    ($atomic:expr, $node:ident, $topic:ty) => {{
        use ::std::pin::Pin;
        use ::std::task::{Context, Poll};
        use ::futures_lite::stream::StreamExt;

        let atomic = $atomic.clone();
        let (sender, receiver, mut subscription) = subscriber!($node, $topic);
        ::std::thread::spawn(move || {
            let waker = crate::ros2::topic::noop_waker();
            let mut cx = Context::from_waker(&waker);

            while !atomic.load(::std::sync::atomic::Ordering::Acquire) {
                // 使用 poll_next 非阻塞地轮询 Stream
                if let Poll::Ready(Some(msg)) = Pin::new(&mut subscription).poll_next(&mut cx) {
                    let _ = sender.try_send(msg);
                }
                ::std::thread::sleep(::std::time::Duration::from_micros(100));
            }
        });
        crate::ros2::topic::TopicSubscriber::<$topic>::new(receiver)
    }};

    ($atomic:expr, $app:ident, $node:ident, $($topic:ty),* $(,)?) => {
        $(
            $app.insert_resource(subscriber!($atomic, $node, $topic));
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

// ==================== 发布话题定义 ====================

define_topic!(CameraInfoTopic, CameraInfo, "/camera_info");
define_topic!(ImageRawTopic, Image, "/image_raw");
define_topic!(ImageCompressedTopic, CompressedImage, "/image_compressed");
define_topic!(GlobalTransformTopic, TFMessage, "/tf");

define_topic!(GimbalPoseTopic, PoseStamped, "/gimbal_pose");
define_topic!(OdomPoseTopic, PoseStamped, "/odom_pose");
define_topic!(CameraPoseTopic, PoseStamped, "/camera_pose");

// 云台四元数位姿发布
define_topic!(
    GimbalQuaternionTopic,
    QuaternionStamped,
    "/gimbal_quaternion"
);

// 弹丸速度发布
define_topic!(CurrentVelocityTopic, Float64, "/current_velocity");

// ==================== 订阅话题定义 ====================

// 云台目标欧拉角订阅
define_topic!(TargetEulerTopic, Vector3Stamped, "/target_eulr");

// 开火指令订阅
define_topic!(FireNotifyTopic, Bool, "/fire_notify");
