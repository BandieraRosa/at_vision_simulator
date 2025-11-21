use crate::dataset::writer::DatasetWriter;
use crate::ros2::capture::{CaptureCamera, CaptureConfig};
use crate::ros2::topic::{GlobalTransformTopic, TopicPublisher};
use crate::{Armor, InfantryRoot, LocalInfantry};
use bevy::mesh::VertexAttributeValues;
use bevy::prelude::*;
use std::collections::HashMap;

#[derive(Resource, Deref, DerefMut)]
struct Dataset(DatasetWriter);
pub struct DatasetPlugin;
impl Plugin for DatasetPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Dataset(DatasetWriter::new("dataset").unwrap()))
            .insert_resource(ArmorOnScreen::default())
            .add_systems(Update, query.after(TransformSystems::Propagate));
    }
}

pub fn extract_point(transform: &GlobalTransform, mesh: &Mesh) -> Vec<Vec3> {
    let mut positions: Vec<Vec3> = Vec::new();
    for (_attr, values) in mesh.attributes() {
        if let VertexAttributeValues::Float32x3(vec) = values {
            positions.extend(vec.iter().map(|&p| Vec3::from(p)));
        }
    }

    if positions.is_empty() {
        return vec![];
    }

    // 局部顶点 -> 世界空间
    positions
        .iter()
        .map(|v| transform.transform_point(*v))
        .collect()
}

pub fn world_to_screen(
    world: Vec3,
    camera_xform: &GlobalTransform,
    projection: &Projection,
    config: &CaptureConfig,
) -> Option<(u32, u32)> {
    // world -> view
    let view = camera_xform.to_matrix().inverse();
    // view -> clip
    let proj = projection.get_clip_from_view();

    let clip = proj * view * world.extend(1.0);

    // point is behind camera
    if clip.w <= 0.0 {
        return None;
    }

    // clip -> ndc
    let ndc = clip.xyz() / clip.w;

    // outside of screen view (x,y out of range)
    if ndc.x < -1.0 || ndc.x > 1.0 || ndc.y < -1.0 || ndc.y > 1.0 {
        return None;
    }

    // ndc -> screen
    let screen_x = (ndc.x + 1.0) * 0.5 * (config.width as f32);
    let screen_y = (1.0 - ndc.y) * 0.5 * (config.height as f32);

    Some((screen_x as u32, screen_y as u32))
}

#[derive(Resource, Default, DerefMut, Deref)]
pub struct ArmorOnScreen(pub HashMap<Entity, HashMap<String, Vec<(u32, u32)>>>);

fn query(
    children: Query<&Children>,
    names: Query<&Name>,
    global_transform: Query<(&GlobalTransform, &Mesh3d, &Armor)>,
    infantry: Query<Entity, (With<InfantryRoot>, Without<LocalInfantry>)>,
    camera: Single<(&Projection, &GlobalTransform), With<CaptureCamera>>,
    config: Res<CaptureConfig>,
    mut armor_screen: ResMut<ArmorOnScreen>,
    mut tf_pub: ResMut<TopicPublisher<GlobalTransformTopic>>,
    ass: Res<Assets<Mesh>>,
) {
    armor_screen.clear();
    let (projection, camera_global_transform) = camera.into_inner();
    for infantry in infantry.iter() {
        armor_screen.insert(infantry, HashMap::new());
        let armor_screen = armor_screen.get_mut(&infantry).unwrap();
        for child in children.iter_descendants(infantry) {
            if let Ok((global_transform, aabb, Armor(armor_name))) = global_transform.get(child) {
                armor_screen.insert(armor_name.clone(), vec![]);
                let vec = armor_screen.get_mut(armor_name).unwrap();
                let corners = extract_point(global_transform, ass.get(aabb).unwrap());
                for corner in corners {
                    let Some(pos) =
                        world_to_screen(corner, camera_global_transform, projection, &config)
                    else {
                        continue;
                    };
                    vec.push(pos);
                    // println!("{}: {:?}", armor_name, pos);
                }
            }
        }
    }
}
