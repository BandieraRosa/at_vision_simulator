use std::{collections::HashMap, f32::consts::PI, rc::Weak};

use avian3d::prelude::{CollisionEventsEnabled, CollisionStart};
use bevy::{
    app::Update,
    asset::{AssetId, Assets, Handle},
    camera::visibility::Visibility,
    color::LinearRgba,
    ecs::{
        bundle::Bundle,
        component::Component,
        entity::Entity,
        hierarchy::{ChildOf, Children},
        name::Name,
        observer::On,
        query::With,
        resource::Resource,
        system::{Commands, Query, Res, ResMut},
    },
    math::Dir3,
    pbr::{MeshMaterial3d, StandardMaterial},
    scene::{SceneInstanceReady, SceneSpawner},
    time::{Time, Timer, TimerMode},
    transform::components::Transform,
};

use rand::{Rng, seq::SliceRandom};

#[derive(Component)]
#[require(CollisionEventsEnabled)]
pub struct Projectile;

#[derive(Component)]
pub struct EnergyHubRoot;

#[derive(PartialEq, Eq)]
enum HubAction {
    StartActivating,
    NewRound,
    Failure,
    ResetToInactive,
}

#[derive(PartialEq, Eq)]
enum EnergyTeam {
    Red,
    Blue,
}

#[derive(Clone, PartialEq, Eq)]
enum MechanismMode {
    Small,
    Large,
}

enum MechanismState {
    Inactive { wait: Timer },
    Activating(ActivatingState),
    Activated { wait: Timer },
    Failed { wait: Timer },
}

#[derive(Clone, PartialEq, Eq)]
enum TargetState {
    Inactive,
    Highlighted,
    Completed,
}

#[derive(Clone, PartialEq, Eq)]
enum ActivationWindow {
    Primary,
    Secondary,
}

#[derive(Component)]
struct TargetIndex(usize, Entity);

fn insert_recursively<F, B>(
    commands: &mut Commands,
    root: Entity,
    query: &Query<&Children>,
    bundle: &F,
) where
    F: Fn() -> B,
    B: Bundle,
{
    commands.entity(root).insert(bundle());
    if let Ok(children) = query.get(root) {
        for child in children {
            insert_recursively(commands, *child, query, bundle);
        }
    }
}

#[derive(Resource, Default)]
struct EnergyMaterialCache {
    muted: HashMap<AssetId<StandardMaterial>, Handle<StandardMaterial>>,
}

struct TargetData {
    visual: TargetVisual,
    state: TargetState,
    applied_state: TargetState,
}

struct TargetVisual {
    active: Entity,
    disabled: Entity,
    legging_segments: Vec<ApperanceController>,
    padding_segments: Vec<ApperanceController>,
    progress_segments: Vec<ApperanceController>,
}

struct ActivatingState {
    highlighted: Vec<usize>,
    hit_flags: Vec<bool>,
    window: ActivationWindow,
    timeout: Timer,
}

struct VariableRotation {
    a: f32,
    omega: f32,
    t: f32,
}

impl VariableRotation {
    fn random(rng: &mut impl Rng) -> Self {
        let a = rng.gen_range(0.780..=1.045);
        let omega = rng.gen_range(1.884..=2.0);
        Self { a, omega, t: 0.0 }
    }

    fn advance(&mut self, dt: f32) -> f32 {
        self.t += dt;
        self.speed()
    }

    fn speed(&self) -> f32 {
        let b = 2.090 - self.a;
        self.a * (self.omega * self.t).sin() + b
    }
}

const ACTIVATION_PRIMARY_TIMEOUT: f32 = 2.5;
const LARGE_SECONDARY_TIMEOUT: f32 = 1.0;
const INACTIVE_WAIT: f32 = 1.0;
const FAILURE_RECOVER: f32 = 1.5;
const ACTIVATED_HOLD: f32 = 6.0;

impl EnergyMaterialCache {
    fn ensure_muted(
        &mut self,
        handle: &Handle<StandardMaterial>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Handle<StandardMaterial> {
        let id = handle.id();
        if let Some(existing) = self.muted.get(&id) {
            return existing.clone();
        }
        let Some(original) = materials.get(handle) else {
            return handle.clone();
        };
        let mut clone = original.clone();
        clone.emissive = LinearRgba::BLACK;
        clone.emissive_exposure_weight = 0.0;
        let muted_handle = materials.add(clone);
        self.muted.insert(id, muted_handle.clone());
        muted_handle
    }
}

struct MaterialSwap {
    entity: Entity,
    on: Handle<StandardMaterial>,
    off: Handle<StandardMaterial>,
}

struct ApperanceController {
    swaps: Vec<MaterialSwap>,
    is_on: bool,
}

impl ApperanceController {
    fn set(&mut self, enabled: bool, materials: &mut Query<&mut MeshMaterial3d<StandardMaterial>>) {
        if self.is_on == enabled {
            return;
        }
        for swap in &self.swaps {
            if let Ok(mut mesh_material) = materials.get_mut(swap.entity) {
                mesh_material.0 = if enabled {
                    swap.on.clone()
                } else {
                    swap.off.clone()
                };
            }
        }
        self.is_on = enabled;
    }
}

struct RotationController {
    baseline: f32,
    direction: Dir3,
    variable: Option<VariableRotation>,
    clockwise: bool,
}

impl RotationController {
    fn new(clockwise: bool) -> Self {
        Self {
            baseline: PI / 3.0,
            direction: Dir3::from_xyz(-1.0, 0.0, -1.0).unwrap(),
            variable: None,
            clockwise,
        }
    }

    fn reset_variable(&mut self, rng: &mut impl Rng) {
        self.variable = Some(VariableRotation::random(rng));
    }

    fn clear_variable(&mut self) {
        self.variable = None;
    }

    fn current_speed(&mut self, mode: MechanismMode, dt: f32) -> f32 {
        let sgn = if self.clockwise { 1.0 } else { -1.0 };
        if mode == MechanismMode::Small {
            return sgn * self.baseline;
        }
        if let Some(variable) = &mut self.variable {
            variable.advance(dt);
            return sgn * variable.speed();
        }
        sgn * self.baseline
    }
}

#[derive(Component)]
struct EnergyHub {
    _team: EnergyTeam,
    mode: MechanismMode,
    state: MechanismState,
    targets: Vec<TargetData>,
    rotation: RotationController,
}

impl EnergyHub {
    fn new(
        team: EnergyTeam,
        mode: MechanismMode,
        targets: Vec<TargetData>,
        clockwise: bool,
    ) -> Self {
        Self {
            _team: team,
            mode,
            state: MechanismState::Inactive {
                wait: Timer::from_seconds(INACTIVE_WAIT, TimerMode::Once),
            },
            targets,
            rotation: RotationController::new(clockwise),
        }
    }

    fn available_targets(&self) -> Vec<usize> {
        self.targets
            .iter()
            .enumerate()
            .filter_map(|(idx, target)| {
                if matches!(target.state, TargetState::Completed) {
                    None
                } else {
                    Some(idx)
                }
            })
            .collect()
    }

    fn build_new_round(&mut self, rng: &mut impl Rng) -> Option<ActivatingState> {
        let mut available = self.available_targets();
        if available.is_empty() {
            return None;
        }
        available.shuffle(rng);
        let required = match self.mode {
            MechanismMode::Small => 1,
            MechanismMode::Large => 2,
        };
        let count = required.min(available.len());
        let selection: Vec<usize> = available.into_iter().take(count).collect();

        for target in &mut self.targets {
            if !matches!(target.state, TargetState::Completed) {
                target.state = TargetState::Inactive;
            }
        }
        for &idx in &selection {
            self.targets[idx].state = TargetState::Highlighted;
        }

        Some(ActivatingState {
            highlighted: selection,
            hit_flags: vec![false; count],
            window: ActivationWindow::Primary,
            timeout: Timer::from_seconds(ACTIVATION_PRIMARY_TIMEOUT, TimerMode::Once),
        })
    }

    fn enter_activating(&mut self, rng: &mut impl Rng) {
        if self.mode == MechanismMode::Large {
            self.rotation.reset_variable(rng);
        } else {
            self.rotation.clear_variable();
        }
        if let Some(state) = self.build_new_round(rng) {
            self.state = MechanismState::Activating(state);
        } else {
            self.enter_activated();
        }
    }

    fn enter_inactive(&mut self) {
        self.reset_all_targets(TargetState::Inactive);
        self.rotation.clear_variable();
        self.state = MechanismState::Inactive {
            wait: Timer::from_seconds(INACTIVE_WAIT, TimerMode::Once),
        };
    }

    fn enter_failed(&mut self) {
        self.reset_all_targets(TargetState::Inactive);
        self.rotation.clear_variable();
        self.state = MechanismState::Failed {
            wait: Timer::from_seconds(FAILURE_RECOVER, TimerMode::Once),
        };
    }

    fn enter_activated(&mut self) {
        self.reset_all_targets(TargetState::Completed);
        self.state = MechanismState::Activated {
            wait: Timer::from_seconds(ACTIVATED_HOLD, TimerMode::Once),
        };
    }

    fn reset_all_targets(&mut self, state: TargetState) {
        for target in &mut self.targets {
            target.state = state.clone();
        }
    }

    fn on_target_hit(&mut self, target_index: usize, rng: &mut impl Rng) -> bool {
        match std::mem::replace(
            &mut self.state,
            MechanismState::Inactive {
                wait: Timer::from_seconds(0.0, TimerMode::Once),
            },
        ) {
            MechanismState::Activating(mut state) => {
                if let Some(pos) = state
                    .highlighted
                    .iter()
                    .position(|&idx| idx == target_index)
                {
                    if state.hit_flags[pos] {
                        self.state = MechanismState::Activating(state);
                        return false;
                    }
                    state.hit_flags[pos] = true;
                    self.targets[target_index].state = TargetState::Completed;

                    if self
                        .targets
                        .iter()
                        .all(|target| matches!(target.state, TargetState::Completed))
                    {
                        println!("activated");
                        self.enter_activated();
                        return true;
                    }

                    match self.mode {
                        MechanismMode::Small => {
                            if let Some(next) = self.build_new_round(rng) {
                                self.state = MechanismState::Activating(next);
                            } else {
                                self.enter_activated();
                            }
                        }
                        MechanismMode::Large => {
                            let hits = state.hit_flags.iter().filter(|&&flag| flag).count();
                            let requires_second =
                                state.highlighted.len() > 1 && hits < state.highlighted.len();
                            if requires_second {
                                state.window = ActivationWindow::Secondary;
                                state.timeout =
                                    Timer::from_seconds(LARGE_SECONDARY_TIMEOUT, TimerMode::Once);
                                self.state = MechanismState::Activating(state);
                            } else if let Some(next) = self.build_new_round(rng) {
                                self.state = MechanismState::Activating(next);
                            } else {
                                self.enter_activated();
                            }
                        }
                    }
                    true
                } else {
                    self.enter_failed();
                    true
                }
            }
            other => {
                self.state = other;
                false
            }
        }
    }
}

fn collect_entities_by<F>(name_map: &HashMap<String, Entity>, mut predicate: F) -> Vec<Entity>
where
    F: FnMut(&str) -> bool,
{
    let mut entries: Vec<_> = name_map
        .iter()
        .filter_map(|(name, &entity)| {
            if predicate(name) {
                Some((name.clone(), entity))
            } else {
                None
            }
        })
        .collect();
    entries.sort_by(|(a, _), (b, _)| a.cmp(b));
    entries.into_iter().map(|(_, entity)| entity).collect()
}

fn collect_material_swaps_recursive(
    entity: Entity,
    mesh_materials: &mut Query<&mut MeshMaterial3d<StandardMaterial>>,
    materials: &mut Assets<StandardMaterial>,
    cache: &mut EnergyMaterialCache,
    query: &Query<&Children>,
    swaps: &mut Vec<MaterialSwap>,
) {
    if let Ok(mut mesh_material) = mesh_materials.get_mut(entity) {
        let on = mesh_material.0.clone();
        let off = cache.ensure_muted(&on, materials);
        mesh_material.0 = off.clone();
        swaps.push(MaterialSwap { entity, on, off });
    }

    if let Ok(children) = query.get(entity) {
        for child in children {
            collect_material_swaps_recursive(
                *child,
                mesh_materials,
                materials,
                cache,
                query,
                swaps,
            );
        }
    }
}

fn create_material_swaps(
    entities: Vec<Entity>,
    mesh_materials: &mut Query<&mut MeshMaterial3d<StandardMaterial>>,
    materials: &mut Assets<StandardMaterial>,
    cache: &mut EnergyMaterialCache,
    children: &Query<&Children>,
) -> Vec<ApperanceController> {
    let mut controllers = Vec::new();
    for entity in entities {
        let mut swaps = Vec::new();
        collect_material_swaps_recursive(
            entity,
            mesh_materials,
            materials,
            cache,
            children,
            &mut swaps,
        );
        if swaps.is_empty() {
            continue;
        }
        controllers.push(ApperanceController { swaps, is_on: true });
    }
    controllers
}

fn set_visibility_if_present(
    entity: Entity,
    value: Visibility,
    visibilities: &mut Query<&mut Visibility>,
) {
    if let Ok(mut visibility) = visibilities.get_mut(entity) {
        *visibility = value;
    }
}

fn apply_target_visual(
    visual: &mut TargetVisual,
    state: &TargetState,
    visibilities: &mut Query<&mut Visibility>,
    materials: &mut Query<&mut MeshMaterial3d<StandardMaterial>>,
) {
    match state {
        TargetState::Inactive => {
            set_visibility_if_present(visual.active, Visibility::Hidden, visibilities);
            set_visibility_if_present(visual.disabled, Visibility::Visible, visibilities);
            for swap in &mut visual.legging_segments {
                swap.set(false, materials);
            }
            for swap in &mut visual.padding_segments {
                swap.set(false, materials);
            }
            for swap in &mut visual.progress_segments {
                swap.set(false, materials);
            }
        }
        TargetState::Highlighted => {
            set_visibility_if_present(visual.active, Visibility::Visible, visibilities);
            set_visibility_if_present(visual.disabled, Visibility::Hidden, visibilities);
            for swap in &mut visual.legging_segments {
                swap.set(true, materials);
            }
            for swap in &mut visual.padding_segments {
                swap.set(false, materials);
            }
            for swap in &mut visual.progress_segments {
                swap.set(true, materials);
            }
        }
        TargetState::Completed => {
            set_visibility_if_present(visual.active, Visibility::Visible, visibilities);
            set_visibility_if_present(visual.disabled, Visibility::Hidden, visibilities);
            for swap in &mut visual.legging_segments {
                swap.set(true, materials);
            }
            for swap in &mut visual.padding_segments {
                swap.set(true, materials);
            }
            for swap in &mut visual.progress_segments {
                swap.set(false, materials);
            }
        }
    }
}

fn build_targets(
    face_index: usize,
    face_entity: Entity,
    commands: &mut Commands,
    name_map: &HashMap<String, Entity>,
    visibilities: &mut Query<&mut Visibility>,
    mesh_materials: &mut Query<&mut MeshMaterial3d<StandardMaterial>>,
    materials: &mut Assets<StandardMaterial>,
    cache: &mut EnergyMaterialCache,
    children: &Query<&Children>,
) -> Vec<TargetData> {
    let mut targets = Vec::new();
    for target_idx in 1..=5 {
        let prefix = format!("FACE_{}_TARGET_{}", face_index, target_idx);
        let active_name = format!("{}_ACTIVE", prefix);
        let disabled_name = format!("{}_DISABLED", prefix);

        let Some(&active) = name_map.get(&active_name) else {
            continue;
        };
        let Some(&disabled) = name_map.get(&disabled_name) else {
            continue;
        };

        set_visibility_if_present(active, Visibility::Hidden, visibilities);
        set_visibility_if_present(disabled, Visibility::Visible, visibilities);

        let legging_entities = collect_entities_by(name_map, |name| {
            name.starts_with(&format!("{}_LEGGING_", prefix)) && !name.contains("PROGRESSING")
        });
        let padding_entities = collect_entities_by(name_map, |name| {
            name.starts_with(&format!("{}_PADDING", prefix))
        });
        let progress_entities = collect_entities_by(name_map, |name| {
            name.starts_with(&format!("{}_LEGGING_PROGRESSING", prefix))
        });

        let legging_segments =
            create_material_swaps(legging_entities, mesh_materials, materials, cache, children);
        let padding_segments =
            create_material_swaps(padding_entities, mesh_materials, materials, cache, children);
        let progress_segments = create_material_swaps(
            progress_entities,
            mesh_materials,
            materials,
            cache,
            children,
        );

        let collision_entity = if name_map.contains_key(&disabled_name) {
            disabled
        } else {
            active
        };

        let logical_index = targets.len();
        insert_recursively(commands, collision_entity, children, &|| {
            (
                TargetIndex(logical_index, face_entity),
                CollisionEventsEnabled,
            )
        });

        targets.push(TargetData {
            visual: TargetVisual {
                active,
                disabled,
                legging_segments,
                padding_segments,
                progress_segments,
            },
            state: TargetState::Inactive,
            applied_state: TargetState::Inactive,
        });
    }
    targets
}

fn build_shared_segments(
    face_index: usize,
    name_map: &HashMap<String, Entity>,
    mesh_materials: &mut Query<&mut MeshMaterial3d<StandardMaterial>>,
    materials: &mut Assets<StandardMaterial>,
    cache: &mut EnergyMaterialCache,
    children: &Query<&Children>,
) -> Vec<ApperanceController> {
    let prefix = format!("FACE_{}_R_", face_index);
    let entities = collect_entities_by(name_map, |name| {
        name.starts_with(&prefix) && (name.ends_with("_POWERED") || name.ends_with("_PADDING"))
    });
    create_material_swaps(entities, mesh_materials, materials, cache, children)
}

fn setup_energy(
    events: On<SceneInstanceReady>,
    mut commands: Commands,
    power_query: Query<(), With<EnergyHubRoot>>,
    scene_spawner: Res<SceneSpawner>,
    names: Query<&Name>,
    mut visibilities: Query<&mut Visibility>,
    mut mesh_materials: Query<&mut MeshMaterial3d<StandardMaterial>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut cache: ResMut<EnergyMaterialCache>,
    children: Query<&Children>,
) {
    if !power_query.contains(events.entity) {
        return;
    }

    let mut name_map = HashMap::new();
    for entity in scene_spawner.iter_instance_entities(events.instance_id) {
        if let Ok(name) = names.get(entity) {
            name_map.insert(name.as_str().to_string(), entity);
        }
    }

    if name_map.is_empty() {
        return;
    }

    let mut faces: Vec<(usize, Entity)> = name_map
        .iter()
        .filter_map(|(name, &entity)| {
            let rest = name.strip_prefix("FACE_")?;
            if rest.contains('_') {
                return None;
            }
            let index = rest.parse::<usize>().ok()?;
            Some((index, entity))
        })
        .collect();

    faces.sort_by_key(|(idx, _)| *idx);
    if faces.is_empty() {
        return;
    }

    for (index, face_entity) in faces {
        let mode = if index & 2 > 0 {
            MechanismMode::Large
        } else {
            MechanismMode::Small
        };

        let targets = build_targets(
            index,
            face_entity,
            &mut commands,
            &name_map,
            &mut visibilities,
            &mut mesh_materials,
            &mut materials,
            &mut cache,
            &children,
        );

        if targets.is_empty() {
            continue;
        }

        commands.entity(face_entity).insert(EnergyHub::new(
            if (index & 1) > 0 {
                EnergyTeam::Red
            } else {
                EnergyTeam::Blue
            },
            mode,
            targets,
            (index & 1) > 0,
        ));
    }
}

fn handle_energy_collision(
    event: On<CollisionStart>,
    mut commands: Commands,
    mut hubs: Query<&mut EnergyHub>,
    targets: Query<&TargetIndex>,
    projectiles: Query<(), With<Projectile>>,
) {
    let Ok(&TargetIndex(index, hub)) = targets.get(event.collider2) else {
        return;
    };
    let other = event.collider1;
    if !projectiles.contains(other) {
        return;
    }
    if let Ok(mut hub) = hubs.get_mut(hub) {
        println!("boom");
        let mut rng = rand::thread_rng();

        hub.on_target_hit(index, &mut rng);
    }
}

fn energy_activation_tick(time: Res<Time>, mut hubs: Query<&mut EnergyHub>) {
    let delta = time.delta();
    let mut rng = rand::thread_rng();
    for mut hub in &mut hubs {
        let action = match &mut hub.state {
            MechanismState::Inactive { wait } => {
                if wait.tick(delta).just_finished() {
                    Some(HubAction::StartActivating)
                } else {
                    None
                }
            }
            MechanismState::Activating(state) => {
                if state.timeout.tick(delta).just_finished() {
                    match state.window {
                        ActivationWindow::Primary => Some(HubAction::Failure),
                        ActivationWindow::Secondary => Some(HubAction::NewRound),
                    }
                } else {
                    None
                }
            }
            MechanismState::Activated { wait } => {
                if wait.tick(delta).just_finished() {
                    Some(HubAction::ResetToInactive)
                } else {
                    None
                }
            }
            MechanismState::Failed { wait } => {
                if wait.tick(delta).just_finished() {
                    Some(HubAction::ResetToInactive)
                } else {
                    None
                }
            }
        };

        if let Some(action) = action {
            match action {
                HubAction::StartActivating => hub.enter_activating(&mut rng),
                HubAction::NewRound => {
                    if let Some(state) = hub.build_new_round(&mut rng) {
                        hub.state = MechanismState::Activating(state);
                    } else {
                        hub.enter_activated();
                    }
                }
                HubAction::Failure => hub.enter_failed(),
                HubAction::ResetToInactive => hub.enter_inactive(),
            }
        }
    }
}

fn energy_apply_visuals(
    mut hubs: Query<&mut EnergyHub>,
    mut visibilities: Query<&mut Visibility>,
    mut materials: Query<&mut MeshMaterial3d<StandardMaterial>>,
) {
    for mut hub in &mut hubs {
        for target in &mut hub.targets {
            if target.state != target.applied_state {
                apply_target_visual(
                    &mut target.visual,
                    &target.state,
                    &mut visibilities,
                    &mut materials,
                );
                target.applied_state = target.state.clone();
            }
        }
        /*
        let shared_on = matches!(hub.state, MechanismState::Activated { .. });
        for segment in &mut hub.shared_segments {
            segment.set(shared_on, &mut materials);
        }*/
    }
}

fn energy_rotation_system(time: Res<Time>, mut hubs: Query<(&mut Transform, &mut EnergyHub)>) {
    let dt = time.delta_secs();
    for (mut transform, mut hub) in &mut hubs {
        let mode = hub.mode.clone();
        let activating = matches!(hub.state, MechanismState::Activating(_));
        let speed = hub.rotation.current_speed(mode, dt);
        let angle = speed * dt;

        transform.rotate_local_axis(hub.rotation.direction, angle);
    }
}

pub struct EnergyHubPlugin;

impl bevy::app::Plugin for EnergyHubPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_resource::<EnergyMaterialCache>()
            .add_observer(handle_energy_collision)
            .add_observer(setup_energy)
            .add_systems(
                Update,
                (
                    energy_activation_tick,
                    energy_apply_visuals,
                    energy_rotation_system,
                ),
            );
    }
}
