use crate::dataset::writer::DatasetWriter;
use crate::ros2::capture::CaptureCamera;
use bevy::app::App;
use bevy::camera::CameraProjection;
use bevy::camera::primitives::Aabb;
use bevy::prelude::*;

#[derive(Resource, Deref, DerefMut)]
struct Dataset(DatasetWriter);
pub struct DatasetPlugin;
