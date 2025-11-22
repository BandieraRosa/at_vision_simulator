use crate::dataset::writer::DatasetWriter;
use crate::ros2::capture::{CaptureCamera, CaptureConfig};
use crate::ros2::topic::{GlobalTransformTopic, TopicPublisher};
use crate::{Armor, InfantryRoot, LocalInfantry};
use bevy::mesh::VertexAttributeValues;
use bevy::prelude::*;
use std::collections::HashMap;
use std::mem::swap;

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

pub fn extract_corners(transform: &GlobalTransform, mesh: &Mesh) -> Option<[Vec3; 4]> {
    let mut positions: Vec<Vec3> = Vec::new();
    for (_attr, values) in mesh.attributes() {
        if let VertexAttributeValues::Float32x3(vec) = values {
            positions.extend(vec.iter().map(|&p| Vec3::from(p)));
            break;
        }
    }

    if positions.is_empty() {
        return None;
    }

    let point: Vec<Vec3> = positions
        .iter()
        .map(|v| transform.transform_point(*v))
        .collect();
    if point.len() != 4 {
        panic!("Expected 4 points but got {}", point.len());
    }
    Some(point.as_slice().try_into().unwrap())
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

fn sort_screen_points(points: [(u32, u32); 4]) -> [(u32, u32); 4] {
    // 转为 Vec2 方便做浮点运算
    let n: [Vec2; 4] = points.map(|(x, y)| Vec2::new(x as f32, y as f32));

    let mut axis = 0.0;
    let mut diag = (0, 0);

    // 找出距离最大的两个点（矩形对角线）
    // points.cartesian_product().map().max() 总是对角线
    for i in 0..4 {
        for j in i + 1..4 {
            let d = (n[i] - n[j]).length();
            if d > axis {
                axis = d;
                diag = (i, j);
            }
        }
    }

    // 第一根对角线的两个点
    let mut p0 = n[diag.0];
    let mut p2 = n[diag.1];
    if p0.x > p2.x {
        // 左上角总是 x 较小的那个
        swap(&mut p0, &mut p2);
    }
    let [mut p1, mut p3]: [Vec2; 2] = (0..4)
        .filter(|&i| i != diag.0 && i != diag.1)
        .map(|i| n[i])
        .collect::<Vec<_>>()
        .try_into()
        .unwrap();
    if p1.x > p3.x {
        // 左上角总是 x 较小的那个
        swap(&mut p1, &mut p3);
    }
    /*
     * 记四个点为
     * | p0 p3 |
     * | p1 p2 |
     * <--x 减小
     * 现在保证：
     * - 左上列: p0.x <= p2.x
     * - 左下列: p1.x <= p3.x
     * 但是可能上下颠倒
     */

    // 同样的，根据 y 坐标调整顺序，让顺时针/逆时针正确
    if p0.y > p1.y {
        swap(&mut p0, &mut p1);
    }
    if p3.y > p2.y {
        swap(&mut p3, &mut p2);
    }

    [
        (p0.x as u32, p0.y as u32), // 左上
        (p1.x as u32, p1.y as u32), // 左下
        (p2.x as u32, p2.y as u32), // 右下
        (p3.x as u32, p3.y as u32), // 右上
    ]
}

#[derive(Resource, Default, DerefMut, Deref)]
pub struct ArmorOnScreen(pub HashMap<Entity, HashMap<String, [(u32, u32); 4]>>);

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
    mut m: MeshRayCast,
) {
    armor_screen.clear();
    let (projection, camera_global_transform) = camera.into_inner();
    for infantry in infantry.iter() {
        armor_screen.insert(infantry, HashMap::new());
        let armor_screen = armor_screen.get_mut(&infantry).unwrap();
        for child in children.iter_descendants(infantry) {
            if let Ok((global_transform, aabb, Armor(armor_name))) = global_transform.get(child) {
                let Some(corners) = extract_corners(global_transform, ass.get(aabb).unwrap())
                else {
                    continue;
                };
                let corners: Vec<_> = corners
                    .into_iter()
                    .filter_map(|corner| {
                        let dir = corner - camera_global_transform.translation();

                        world_to_screen(corner, camera_global_transform, projection, &config)
                    })
                    .collect();
                if corners.len() != 4 {
                    //不是完整出现的
                    continue;
                }
                armor_screen.insert(
                    armor_name.clone(),
                    sort_screen_points(corners.try_into().unwrap()),
                );
            }
        }
    }
}
