use bevy::prelude::*;
use ros2_client::Context;
use rustdds::{
    Duration, QosPolicies, QosPolicyBuilder,
    policy::{self, Deadline, Lifespan},
};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use crate::ros2::{
    messages::*,
    node::{ROS2NodeConfig, ROS2NodeManager},
    systems::{
        handle_aim_commands, handle_velocity_commands, publish_robot_pose, publish_rune_states,
        setup_ros2_infrastructure,
    },
};

// ROS2事件定义 - 使用Bevy的Message系统
#[derive(Event, Message)]
pub struct ROS2RobotPose {
    pub entity: Entity,
    pub pose: Transform,
    pub timestamp: f64,
}

#[derive(Event, Message)]
pub struct ROS2VelocityCommand {
    pub linear: Vec3,
    pub angular: Vec3,
    pub timestamp: f64,
}

#[derive(Event, Message)]
pub struct ROS2AimCommand {
    pub target_position: Vec3,
    pub priority: i32,
    pub timestamp: f64,
}

#[derive(Event, Message)]
pub struct ROS2RuneState {
    pub rune_id: i32,
    pub activated: bool,
    pub hit_count: i32,
    pub timestamp: f64,
}

// ROS2资源配置
#[derive(Resource)]
pub struct ROS2Context(pub Context);

#[derive(Resource)]
pub struct ROS2NodeManagerResource(pub Arc<Mutex<ROS2NodeManager>>);

// 使用具体的消息类型而不是Box<dyn Any>来保证线程安全
#[derive(Resource)]
pub struct ROS2Publishers {
    pub robot_pose_publisher: Option<ros2_client::Publisher<PoseStamped>>,
    pub rune_state_publisher: Option<ros2_client::Publisher<PowerRuneState>>,
}

#[derive(Resource)]
pub struct ROS2Subscriptions {
    pub velocity_subscription: Option<ros2_client::Subscription<TwistStamped>>,
    pub aim_subscription: Option<ros2_client::Subscription<AimCommand>>,
}

// 主插件结构
pub struct ROS2Plugin {
    node_configs: HashMap<String, ROS2NodeConfig>,
}

impl Default for ROS2Plugin {
    fn default() -> Self {
        let mut node_configs = HashMap::new();
        node_configs.insert(
            "robot".to_string(),
            ROS2NodeConfig {
                namespace: "/robomaster".to_string(),
                name: "simulator".to_string(),
                enable_rosout: true,
            },
        );

        Self { node_configs }
    }
}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        // 初始化ROS2上下文
        let context = Context::new().unwrap();
        let mut node_manager = ROS2NodeManager::new();

        // 创建基础设施
        for (key, config) in &self.node_configs {
            if let Err(e) = node_manager.add_node(key.clone(), &context, config) {
                error!("Failed to create ROS2 node '{}': {}", key, e);
            }
        }

        app.insert_resource(ROS2Context(context))
            .insert_resource(ROS2NodeManagerResource(Arc::new(Mutex::new(node_manager))))
            .insert_resource(ROS2Publishers {
                robot_pose_publisher: None,
                rune_state_publisher: None,
            })
            .insert_resource(ROS2Subscriptions {
                velocity_subscription: None,
                aim_subscription: None,
            })
            .add_systems(Startup, setup_ros2_infrastructure)
            .add_systems(
                Update,
                (
                    publish_robot_pose,
                    publish_rune_states,
                    handle_velocity_commands,
                    handle_aim_commands,
                ),
            );
    }
}

// QoS配置
pub fn create_reliable_qos() -> QosPolicies {
    QosPolicyBuilder::new()
        .history(policy::History::KeepLast { depth: 10 })
        .reliability(policy::Reliability::Reliable {
            max_blocking_time: Duration::from_millis(100),
        })
        .durability(policy::Durability::TransientLocal)
        .deadline(Deadline(Duration::INFINITE))
        .lifespan(Lifespan {
            duration: Duration::INFINITE,
        })
        .liveliness(policy::Liveliness::Automatic {
            lease_duration: Duration::INFINITE,
        })
        .build()
}

pub fn create_default_qos() -> QosPolicies {
    QosPolicyBuilder::new()
        .history(policy::History::KeepLast { depth: 10 })
        .build()
}
