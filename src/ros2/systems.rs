use bevy::prelude::*;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::ros2::{
    messages::*,
    plugin::{
        ROS2AimCommand, ROS2Context, ROS2NodeManagerResource, ROS2Publishers, ROS2RobotPose,
        ROS2RuneState, ROS2Subscriptions, ROS2VelocityCommand,
    },
};

use ros2_client::MessageTypeName;
use rustdds::{
    Duration, QosPolicies, QosPolicyBuilder,
    policy::{self, Deadline, Lifespan},
};

// ROS2基础设施设置系统
pub fn setup_ros2_infrastructure(
    node_manager: Res<ROS2NodeManagerResource>,
    _context: Res<ROS2Context>,
    mut publishers: ResMut<ROS2Publishers>,
    mut subscriptions: ResMut<ROS2Subscriptions>,
) {
    info!("Setting up ROS2 infrastructure...");

    // 创建可靠的QoS配置
    let qos = create_reliable_qos();

    // 创建机器人位姿发布者
    if node_manager.0.lock().unwrap().get_node("robot").is_some() {
        match node_manager
            .0
            .lock()
            .unwrap()
            .create_publisher::<PoseStamped>(
                "robot",
                "/robomaster/robot_pose",
                MessageTypeName::new("geometry_msgs", "PoseStamped"),
                qos.clone(),
            ) {
            Ok(publisher) => {
                publishers.robot_pose_publisher = Some(publisher);
                info!("Created robot pose publisher");
            }
            Err(e) => error!("Failed to create robot pose publisher: {}", e),
        }

        // 创建符文状态发布者
        match node_manager
            .0
            .lock()
            .unwrap()
            .create_publisher::<PowerRuneState>(
                "robot",
                "/robomaster/power_rune_state",
                MessageTypeName::new("robomaster_msgs", "PowerRuneState"),
                qos.clone(),
            ) {
            Ok(publisher) => {
                publishers.rune_state_publisher = Some(publisher);
                info!("Created power rune state publisher");
            }
            Err(e) => error!("Failed to create power rune state publisher: {}", e),
        }

        // 创建速度命令订阅者
        match node_manager
            .0
            .lock()
            .unwrap()
            .create_subscription::<TwistStamped>(
                "robot",
                "/robomaster/cmd_vel",
                MessageTypeName::new("geometry_msgs", "TwistStamped"),
                qos.clone(),
            ) {
            Ok(subscription) => {
                subscriptions.velocity_subscription = Some(subscription);
                info!("Created velocity command subscription");
            }
            Err(e) => error!("Failed to create velocity command subscription: {}", e),
        }

        // 创建瞄准命令订阅者
        match node_manager
            .0
            .lock()
            .unwrap()
            .create_subscription::<AimCommand>(
                "robot",
                "/robomaster/aim_command",
                MessageTypeName::new("robomaster_msgs", "AimCommand"),
                qos,
            ) {
            Ok(subscription) => {
                subscriptions.aim_subscription = Some(subscription);
                info!("Created aim command subscription");
            }
            Err(e) => error!("Failed to create aim command subscription: {}", e),
        }
    } else {
        error!("Robot node not found for ROS2 infrastructure setup");
    }

    info!("ROS2 infrastructure setup complete");
}

// 机器人位姿发布系统
pub fn publish_robot_pose(
    mut pose_events: MessageWriter<ROS2RobotPose>,
    publishers: Res<ROS2Publishers>,
    robot_query: Query<(Entity, &Transform), With<crate::LocalInfantry>>,
) {
    let current_time = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();

    for (entity, transform) in robot_query.iter() {
        pose_events.write(ROS2RobotPose {
            entity,
            pose: *transform,
            timestamp: current_time,
        });
    }

    // 发布到ROS2主题
    if let Some(ref publisher) = publishers.robot_pose_publisher {
        for (entity, transform) in robot_query.iter() {
            let pose_stamped = PoseStamped {
                header: Header {
                    stamp: current_time,
                    frame_id: "map".to_string(),
                },
                pose: transform_to_pose(transform),
            };

            if let Err(e) = publisher.publish(pose_stamped) {
                error!("Failed to publish robot pose: {}", e);
            }
        }
    }
}

// 符文状态发布系统
pub fn publish_rune_states(
    mut rune_events: MessageWriter<ROS2RuneState>,
    publishers: Res<ROS2Publishers>,
) {
    let current_time = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();

    // 示例：发布一个测试符文状态
    let test_rune_state = ROS2RuneState {
        rune_id: 1,
        activated: true,
        hit_count: 5,
        timestamp: current_time,
    };
    rune_events.write(test_rune_state);

    // 发布到ROS2主题
    if let Some(ref publisher) = publishers.rune_state_publisher {
        let power_rune_state = PowerRuneState {
            header: Header {
                stamp: current_time,
                frame_id: "map".to_string(),
            },
            rune_id: 1,
            activated: true,
            hit_count: 5,
        };

        if let Err(e) = publisher.publish(power_rune_state) {
            error!("Failed to publish power rune state: {}", e);
        }
    }
}

// 速度命令处理系统
pub fn handle_velocity_commands(
    mut velocity_commands: MessageWriter<ROS2VelocityCommand>,
    subscriptions: Res<ROS2Subscriptions>,
) {
    // 处理来自ROS2的速度命令
    if let Some(ref subscription) = subscriptions.velocity_subscription {
        while let Ok(Some((twist_stamped, _info))) = subscription.take() {
            let linear = Vec3::new(
                twist_stamped.twist.linear.x as f32,
                twist_stamped.twist.linear.y as f32,
                twist_stamped.twist.linear.z as f32,
            );
            let angular = Vec3::new(
                twist_stamped.twist.angular.x as f32,
                twist_stamped.twist.angular.y as f32,
                twist_stamped.twist.angular.z as f32,
            );

            velocity_commands.write(ROS2VelocityCommand {
                linear,
                angular,
                timestamp: twist_stamped.header.stamp,
            });

            info!(
                "Processing velocity command - linear: {:?}, angular: {:?}",
                linear, angular
            );
        }
    }
}

// 瞄准命令处理系统
pub fn handle_aim_commands(
    mut aim_commands: MessageWriter<ROS2AimCommand>,
    subscriptions: Res<ROS2Subscriptions>,
) {
    // 处理来自ROS2的瞄准命令
    if let Some(ref subscription) = subscriptions.aim_subscription {
        while let Ok(Some((aim_cmd, _info))) = subscription.take() {
            let target_position = Vec3::new(
                aim_cmd.target_position[0] as f32,
                aim_cmd.target_position[1] as f32,
                aim_cmd.target_position[2] as f32,
            );

            aim_commands.write(ROS2AimCommand {
                target_position,
                priority: aim_cmd.priority,
                timestamp: aim_cmd.header.stamp,
            });

            info!(
                "Processing aim command - target: {:?}, priority: {}",
                target_position, aim_cmd.priority
            );
        }
    }
}

// QoS配置函数
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

// ROS2消息转换辅助函数
pub fn transform_to_pose(transform: &Transform) -> Pose {
    Pose {
        position: Point {
            x: transform.translation.x as f64,
            y: transform.translation.y as f64,
            z: transform.translation.z as f64,
        },
        orientation: Quaternion {
            x: transform.rotation.x as f64,
            y: transform.rotation.y as f64,
            z: transform.rotation.z as f64,
            w: transform.rotation.w as f64,
        },
    }
}

pub fn vec3_to_vector3(vec3: Vec3) -> Vector3 {
    Vector3 {
        x: vec3.x as f64,
        y: vec3.y as f64,
        z: vec3.z as f64,
    }
}

pub fn vec3_to_twist(linear: Vec3, angular: Vec3) -> Twist {
    Twist {
        linear: vec3_to_vector3(linear),
        angular: vec3_to_vector3(angular),
    }
}

// ROS2组件标记
#[derive(Component)]
pub struct ROS2Controlled;

#[derive(Component)]
pub struct ROS2Publisher {
    pub topic_name: String,
    pub message_type: String,
}

#[derive(Component)]
pub struct ROS2Subscriber {
    pub topic_name: String,
    pub message_type: String,
}
