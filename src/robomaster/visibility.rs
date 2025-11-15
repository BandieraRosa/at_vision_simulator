use crate::util::bevy::set_visibility;
use bevy::{
    asset::{Assets, Handle},
    camera::visibility::Visibility,
    ecs::{
        entity::Entity,
        system::{Query, ResMut, SystemParam},
    },
    pbr::{MeshMaterial3d, StandardMaterial},
};
use std::hash::Hash;

#[derive(SystemParam)]
pub struct Param<'w, 's> {
    pub materials: ResMut<'w, Assets<StandardMaterial>>,
    pub mesh_materials: Query<'w, 's, &'static mut MeshMaterial3d<StandardMaterial>>,
    pub visibilities: Query<'w, 's, &'static mut Visibility>,
}

pub trait Control {
    fn set(&mut self, state: Activation, param: &mut Param);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Activation {
    Deactivated = 0,
    Activating = 1,
    Activated = 2,
    Completed = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub enum ControllerType {
    Material = 0,
    Visibility = 1,
    Combined = 2,
}

pub enum Controller {
    Material(
        Entity,
        Handle<StandardMaterial>,
        Handle<StandardMaterial>,
        Handle<StandardMaterial>,
        Handle<StandardMaterial>,
    ),

    Visibility(
        Option<Entity>,
        Option<Entity>,
        Option<Entity>,
        Option<Entity>,
    ),

    Combined(Vec<Controller>),
}

impl Control for Controller {
    fn set(&mut self, state: Activation, param: &mut Param) {
        match self {
            Self::Material(entity, deactivated, activating, activated, completed) => {
                let apply = match state {
                    Activation::Deactivated => deactivated,
                    Activation::Activating => activating,
                    Activation::Activated => activated,
                    Activation::Completed => completed,
                };
                if let Ok(mut mesh_material) = param.mesh_materials.get_mut(*entity) {
                    mesh_material.0 = apply.clone()
                }
            }
            Self::Visibility(deactivated, activating, activated, completed) => {
                let (show, hide) = match state {
                    Activation::Deactivated => (deactivated, [activating, activated, completed]),
                    Activation::Activating => (activating, [deactivated, activated, completed]),
                    Activation::Activated => (activated, [deactivated, activating, completed]),
                    Activation::Completed => (completed, [deactivated, activating, activated]),
                };
                for entity in hide {
                    if let Some(entity) = entity {
                        set_visibility(*entity, Visibility::Hidden, &mut param.visibilities)
                            .unwrap();
                    }
                }
                if let Some(show) = show {
                    set_visibility(*show, Visibility::Visible, &mut param.visibilities).unwrap();
                }
            }
            Self::Combined(vec) => {
                for c in vec {
                    c.set(state, param);
                }
            }
        }
    }
}

impl Controller {
    pub fn new_visibility(
        deactivated: Option<Entity>,
        activating: Option<Entity>,
        activated: Option<Entity>,
        completed: Option<Entity>,
    ) -> Self {
        Self::Visibility(deactivated, activating, activated, completed)
    }

    pub fn new_material(
        entity: Entity,
        deactivated: Handle<StandardMaterial>,
        activating: Handle<StandardMaterial>,
        activated: Handle<StandardMaterial>,
        completed: Handle<StandardMaterial>,
    ) -> Self {
        Self::Material(entity, deactivated, activating, activated, completed)
    }

    pub fn new_combined(v: Vec<Controller>) -> Self {
        Self::Combined(v)
    }
}
