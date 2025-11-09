use crate::ros2::messages::*;
use bevy::prelude::*;
use ros2_client::{MessageTypeName, Name, Node, NodeName, NodeOptions};
use rustdds::QosPolicies;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

// ROS2节点配置
#[derive(Debug, Clone)]
pub struct ROS2NodeConfig {
    pub namespace: String,
    pub name: String,
    pub enable_rosout: bool,
}

impl Default for ROS2NodeConfig {
    fn default() -> Self {
        Self {
            namespace: "/robomaster".to_string(),
            name: "simulator".to_string(),
            enable_rosout: true,
        }
    }
}

// ROS2节点管理器
pub struct ROS2NodeManager {
    nodes: Arc<Mutex<HashMap<String, Arc<Mutex<Node>>>>>,
}

impl ROS2NodeManager {
    pub fn new() -> Self {
        Self {
            nodes: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn add_node(
        &mut self,
        key: String,
        context: &ros2_client::Context,
        config: &ROS2NodeConfig,
    ) -> Result<(), String> {
        let node = context
            .new_node(
                NodeName::new(&config.namespace, &config.name).unwrap(),
                NodeOptions::new().enable_rosout(config.enable_rosout),
            )
            .map_err(|e| format!("Failed to create node: {:?}", e))?;

        self.nodes
            .lock()
            .unwrap()
            .insert(key, Arc::new(Mutex::new(node)));
        Ok(())
    }

    pub fn get_node(&self, key: &str) -> Option<Arc<Mutex<Node>>> {
        self.nodes.lock().unwrap().get(key).cloned()
    }

    pub fn create_publisher<T: ros2_client::Message + 'static>(
        &self,
        node_key: &str,
        topic_name: &str,
        message_type: MessageTypeName,
        qos: QosPolicies,
    ) -> Result<ros2_client::Publisher<T>, String> {
        let nodes = self.nodes.lock().unwrap();
        let node_arc = nodes
            .get(node_key)
            .ok_or_else(|| format!("Node '{}' not found", node_key))?;

        let mut node = node_arc.lock().unwrap();
        let topic = node
            .create_topic(&Name::new("/", topic_name).unwrap(), message_type, &qos)
            .map_err(|e| format!("Failed to create topic: {:?}", e))?;

        node.create_publisher(&topic, None)
            .map_err(|e| format!("Failed to create publisher: {:?}", e))
    }

    pub fn create_subscription<T: ros2_client::Message + 'static>(
        &self,
        node_key: &str,
        topic_name: &str,
        message_type: MessageTypeName,
        qos: QosPolicies,
    ) -> Result<ros2_client::Subscription<T>, String> {
        let nodes = self.nodes.lock().unwrap();
        let node_arc = nodes
            .get(node_key)
            .ok_or_else(|| format!("Node '{}' not found", node_key))?;

        let mut node = node_arc.lock().unwrap();
        let topic = node
            .create_topic(&Name::new("/", topic_name).unwrap(), message_type, &qos)
            .map_err(|e| format!("Failed to create topic: {:?}", e))?;

        node.create_subscription(&topic, None)
            .map_err(|e| format!("Failed to create subscription: {:?}", e))
    }

    pub fn list_nodes(&self) -> Vec<String> {
        self.nodes.lock().unwrap().keys().cloned().collect()
    }
}

impl Default for ROS2NodeManager {
    fn default() -> Self {
        Self::new()
    }
}
