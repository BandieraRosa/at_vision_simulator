use std::collections::HashMap;

use bevy::{
    camera::visibility::Visibility,
    ecs::{
        bundle::Bundle,
        entity::Entity,
        hierarchy::Children,
        system::{Commands, Query},
    },
};

pub fn extract_entities_by<T, F: Fn(&T) -> bool>(
    name_map: &mut HashMap<T, Entity>,
    predicate: F,
) -> Vec<Entity> {
    return name_map
        .extract_if(|k, _v| predicate(k))
        .map(|v| v.1)
        .collect();
}

pub fn insert_recursively<B: Bundle + Clone>(
    commands: &mut Commands,
    root: Entity,
    query: &Query<&Children>,
    bundle: B,
) {
    if let Ok(children) = query.get(root) {
        for child in children {
            insert_recursively(commands, *child, query, bundle.clone());
        }
    }
    commands.entity(root).insert(bundle);
}

pub fn set_visibility_if_present(
    entity: Entity,
    value: Visibility,
    visibilities: &mut Query<&mut Visibility>,
) {
    if let Ok(mut visibility) = visibilities.get_mut(entity) {
        *visibility = value;
    }
}
