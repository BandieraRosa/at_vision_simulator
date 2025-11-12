use bevy::{
    asset::{Assets, Handle},
    camera::visibility::Visibility,
    ecs::{
        entity::Entity,
        system::{Query, ResMut, SystemParam},
    },
    pbr::{MeshMaterial3d, StandardMaterial},
};

use crate::util::set_visibility;

#[derive(SystemParam)]
pub struct Param<'w, 's> {
    pub materials: ResMut<'w, Assets<StandardMaterial>>,
    pub mesh_materials: Query<'w, 's, &'static mut MeshMaterial3d<StandardMaterial>>,
    pub visibilities: Query<'w, 's, &'static mut Visibility>,
}

pub trait Controller {
    fn set(&mut self, powered: bool, param: &mut Param);
}

pub struct Combined<T: Controller>(pub Vec<T>);
impl<T: Controller> Controller for Combined<T> {
    fn set(&mut self, powered: bool, param: &mut Param) {
        for c in &mut self.0 {
            c.set(powered, param);
        }
    }
}

pub struct MaterialBased {
    pub entity: Entity,
    pub on: Handle<StandardMaterial>,
    pub off: Handle<StandardMaterial>,
}

impl Controller for MaterialBased {
    fn set(&mut self, powered: bool, param: &mut Param) {
        if let Ok(mut mesh_material) = param.mesh_materials.get_mut(self.entity) {
            mesh_material.0 = if powered {
                self.on.clone()
            } else {
                self.off.clone()
            };
        }
    }
}

pub struct VisibilityBased {
    pub powered: Entity,
    pub unpowered: Option<Entity>,
}

impl Controller for VisibilityBased {
    fn set(&mut self, powered: bool, param: &mut Param) {
        let visibilities = match powered {
            true => (Visibility::Visible, Visibility::Hidden),
            false => (Visibility::Hidden, Visibility::Visible),
        };

        let (powered_vis, unpowered_vis) = visibilities;

        set_visibility(self.powered, powered_vis, &mut param.visibilities).unwrap();

        if let Some(unpowered) = self.unpowered {
            set_visibility(unpowered, unpowered_vis, &mut param.visibilities).unwrap();
        }
    }
}
