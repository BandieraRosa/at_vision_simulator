use ros2_client::Message;
use serde::{Deserialize, Serialize};

// 几何相关消息类型
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

// ROS2标准消息类型
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub stamp: f64, // 简化版本，实际应该使用内置的时间类型
    pub frame_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TwistStamped {
    pub header: Header,
    pub twist: Twist,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bool {
    pub data: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Int32 {
    pub data: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Float32 {
    pub data: f32,
}

// 自定义消息类型
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotState {
    pub header: Header,
    pub robot_id: i32,
    pub health: i32,
    pub ammo: i32,
    pub power_rune_activated: bool,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub velocity: [f64; 3],
    pub rotation: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerRuneState {
    pub header: Header,
    pub rune_id: i32,
    pub activated: bool,
    pub hit_count: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AimCommand {
    pub header: Header,
    pub target_position: [f64; 3],
    pub priority: i32,
}

// 消息实现
impl Message for Point {}
impl Message for Quaternion {}
impl Message for Vector3 {}
impl Message for Pose {}
impl Message for Twist {}
impl Message for Header {}
impl Message for PoseStamped {}
impl Message for TwistStamped {}
impl Message for Bool {}
impl Message for Int32 {}
impl Message for Float32 {}
impl Message for RobotState {}
impl Message for PowerRuneState {}
impl Message for AimCommand {}
