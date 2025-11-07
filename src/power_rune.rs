use std::collections::HashMap;

use avian3d::prelude::{CollisionEventsEnabled, CollisionStart};
use bevy::{
    app::Update,
    asset::{AssetId, Assets, Handle},
    camera::visibility::Visibility,
    color::LinearRgba,
    ecs::{
        component::Component,
        entity::Entity,
        event::EntityEvent,
        hierarchy::Children,
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

use crate::util::{collect_entities_by, insert_recursively};

#[derive(Component)]
#[require(CollisionEventsEnabled)]
pub struct Projectile;

#[derive(Component)]
pub struct PowerRuneRoot;

#[derive(Debug, PartialEq, Eq)]
enum RuneAction {
    StartActivating,
    NewRound,
    Failure,
    ResetToInactive,
}

#[derive(Debug, PartialEq, Eq)]
enum RuneTeam {
    Red,
    Blue,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum RuneMode {
    Small,
    Large,
}

#[derive(Debug, Clone)]
enum MechanismState {
    Inactive { wait: Timer },
    Activating(ActivatingState),
    Activated { wait: Timer },
    Failed { wait: Timer },
}

#[derive(Clone, PartialEq, Eq)]
enum RuneState {
    Inactive,
    Highlighted,
    Completed,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum ActivationWindow {
    Primary,
    Secondary,
}

#[derive(Component)]
struct RuneIndex(usize, Entity);

#[derive(Resource, Default)]
struct RuneMaterialCache {
    muted: HashMap<AssetId<StandardMaterial>, Handle<StandardMaterial>>,
}

struct RuneData {
    visual: RuneVisual,
    state: RuneState,
    applied_state: RuneState,
}

struct RuneVisual {
    active: Entity,
    disabled: Entity,
    legging_segments: [Vec<ApperanceController>; 3],
    padding_segments: Vec<ApperanceController>,
    progress_segments: Vec<ApperanceController>,
}

#[derive(Debug, Clone)]
struct ActivatingState {
    highlighted: Vec<usize>,
    hit_flags: Vec<bool>,
    window: ActivationWindow,
    timeout: Timer,
    global_timeout: Timer, // 20秒全局激活超时
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
const ACTIVATION_GLOBAL_TIMEOUT: f32 = 20.0; // 20秒全局激活超时
const ROTATION_BASELINE_SMALL: f32 = std::f32::consts::PI / 3.0; // 小机关固定角速度

impl RuneMaterialCache {
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
            baseline: ROTATION_BASELINE_SMALL,
            direction: Dir3::from_xyz(-1.0, 0.0, -1.0).unwrap(),
            variable: None,
            clockwise,
        }
    }

    fn reset_variable(&mut self, rng: &mut impl Rng) {
        self.variable = Some(VariableRotation::random(rng));
        // 确保重置时间参数
        if let Some(ref mut variable) = self.variable {
            variable.t = 0.0;
        }
    }

    fn clear_variable(&mut self) {
        self.variable = None;
    }

    fn current_speed(&mut self, mode: RuneMode, dt: f32) -> f32 {
        let sgn = if self.clockwise { 1.0 } else { -1.0 };
        if mode == RuneMode::Small {
            return sgn * self.baseline;
        }
        // 大机关只有在激活状态下使用变量旋转
        if let Some(variable) = &mut self.variable {
            variable.advance(dt);
            return sgn * variable.speed();
        }
        sgn * self.baseline
    }
}

const 招笑: bool = true;

#[derive(Component)]
pub struct EnergyHub {
    _team: RuneTeam,
    mode: RuneMode,
    state: MechanismState,
    targets: Vec<RuneData>,
    rotation: RotationController,
}

pub struct HitResult {
    pub accurate: bool,
    pub change_state: bool,
}

impl EnergyHub {
    fn new(team: RuneTeam, mode: RuneMode, targets: Vec<RuneData>, clockwise: bool) -> Self {
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
                if matches!(target.state, RuneState::Completed) {
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
            RuneMode::Small => 1,
            RuneMode::Large => 2,
        };
        let count = required.min(available.len());
        let selection: Vec<usize> = available.into_iter().take(count).collect();

        for target in &mut self.targets {
            if !matches!(target.state, RuneState::Completed) {
                target.state = RuneState::Inactive;
            }
        }
        for &idx in &selection {
            self.targets[idx].state = RuneState::Highlighted;
        }

        Some(ActivatingState {
            highlighted: selection,
            hit_flags: vec![false; count],
            window: ActivationWindow::Primary,
            timeout: Timer::from_seconds(ACTIVATION_PRIMARY_TIMEOUT, TimerMode::Once),
            global_timeout: Timer::from_seconds(ACTIVATION_GLOBAL_TIMEOUT, TimerMode::Once), // 20秒全局激活超时
        })
    }

    fn enter_activating(&mut self, rng: &mut impl Rng) {
        // 重新创建旋转控制器确保参数完全重置
        self.rotation.clear_variable();

        // 大机关激活时使用变量旋转，小机关使用固定速度
        if self.mode == RuneMode::Large {
            self.rotation.reset_variable(rng);
        }
        if let Some(state) = self.build_new_round(rng) {
            self.state = MechanismState::Activating(state);
        } else {
            self.enter_activated();
        }
    }

    fn enter_inactive(&mut self) {
        self.reset_all_targets(RuneState::Inactive);
        self.rotation.clear_variable();
        self.state = MechanismState::Inactive {
            wait: Timer::from_seconds(INACTIVE_WAIT, TimerMode::Once),
        };
    }

    fn enter_failed(&mut self) {
        self.reset_all_targets(RuneState::Inactive);
        self.state = MechanismState::Failed {
            wait: Timer::from_seconds(FAILURE_RECOVER, TimerMode::Once),
        };
    }

    fn enter_activated(&mut self) {
        self.reset_all_targets(RuneState::Completed);
        // 激活状态下停止旋转
        self.rotation.clear_variable();
        self.state = MechanismState::Activated {
            wait: Timer::from_seconds(ACTIVATED_HOLD, TimerMode::Once),
        };
    }

    fn reset_all_targets(&mut self, state: RuneState) {
        for target in &mut self.targets {
            target.state = state.clone();
        }
    }

    fn on_target_hit(&mut self, target_index: usize, rng: &mut impl Rng) -> HitResult {
        match &mut self.state {
            MechanismState::Activating(state) => {
                let Some(pos) = state
                    .highlighted
                    .iter()
                    .position(|&idx| idx == target_index)
                else {
                    if !招笑 {
                        println!("招笑");
                        // 击中非点亮模块，触发激活失败
                        self.enter_failed();
                    }
                    return HitResult {
                        accurate: true,
                        change_state: false,
                    };
                };

                if state.hit_flags[pos] {
                    return HitResult {
                        accurate: true,
                        change_state: false,
                    };
                }
                state.hit_flags[pos] = true;
                self.targets[target_index].state = RuneState::Completed;

                if self
                    .targets
                    .iter()
                    .all(|target| matches!(target.state, RuneState::Completed))
                {
                    println!("activated");
                    self.enter_activated();
                    return HitResult {
                        accurate: true,
                        change_state: true,
                    };
                }

                match self.mode {
                    RuneMode::Small => {
                        if let Some(next) = self.build_new_round(rng) {
                            self.state = MechanismState::Activating(next);
                            return HitResult {
                                accurate: true,
                                change_state: false,
                            };
                        }
                        self.enter_activated();
                        return HitResult {
                            accurate: true,
                            change_state: true,
                        };
                    }
                    RuneMode::Large => {
                        let hits = state.hit_flags.iter().filter(|&&flag| flag).count();

                        // 大机关逻辑：规则要求命中任意一个靶后启动1秒二次窗口
                        if hits == 1 && state.window == ActivationWindow::Primary {
                            // 命中第一个靶后启动1秒二次命中窗口
                            state.window = ActivationWindow::Secondary;
                            state.timeout =
                                Timer::from_seconds(LARGE_SECONDARY_TIMEOUT, TimerMode::Once);
                            return HitResult {
                                accurate: true,
                                change_state: false,
                            };
                        }

                        // 处理边界情况：如果二次窗口超时后仍未命中第二个靶，进入下一轮
                        if let Some(next) = self.build_new_round(rng) {
                            self.state = MechanismState::Activating(next);
                            return HitResult {
                                accurate: true,
                                change_state: false,
                            };
                        }
                        self.enter_activated();
                        return HitResult {
                            accurate: true,
                            change_state: true,
                        };
                    }
                }
            }
            _ => HitResult {
                accurate: false,
                change_state: false,
            },
        }
    }
}

fn collect_material_swaps_recursive(
    entity: Entity,
    mesh_materials: &mut Query<&mut MeshMaterial3d<StandardMaterial>>,
    materials: &mut Assets<StandardMaterial>,
    cache: &mut RuneMaterialCache,
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
    cache: &mut RuneMaterialCache,
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
    mode: &RuneMode,
    visual: &mut RuneVisual,
    state: &RuneState,
    visibilities: &mut Query<&mut Visibility>,
    materials: &mut Query<&mut MeshMaterial3d<StandardMaterial>>,
) {
    match state {
        RuneState::Inactive => {
            set_visibility_if_present(visual.active, Visibility::Hidden, visibilities);
            set_visibility_if_present(visual.disabled, Visibility::Visible, visibilities);
            for swap in &mut visual.legging_segments {
                for swap in swap {
                    swap.set(false, materials);
                }
            }
            for swap in &mut visual.padding_segments {
                swap.set(false, materials);
            }
            for swap in &mut visual.progress_segments {
                swap.set(false, materials);
            }
        }
        RuneState::Highlighted => {
            set_visibility_if_present(visual.active, Visibility::Visible, visibilities);
            set_visibility_if_present(visual.disabled, Visibility::Hidden, visibilities);
            for swap in &mut visual.padding_segments {
                swap.set(false, materials);
            }
            for swap in &mut visual.progress_segments {
                swap.set(true, materials);
            }
            // 高亮状态：关闭所有灯效，只显示进度灯效
            for level_swaps in &mut visual.legging_segments {
                for swap in level_swaps {
                    swap.set(false, materials);
                }
            }
        }
        RuneState::Completed => {
            set_visibility_if_present(visual.active, Visibility::Visible, visibilities);
            set_visibility_if_present(visual.disabled, Visibility::Hidden, visibilities);

            // 根据模式设置灯效：小机关全亮，大机关亮第1级
            for swap in &mut visual.padding_segments {
                swap.set(true, materials);
            }
            for swap in &mut visual.progress_segments {
                swap.set(false, materials);
            }

            match *mode {
                RuneMode::Small => {
                    // 小机关：完成时所有灯臂全亮 LEGGING_1,2,3
                    for level_swaps in &mut visual.legging_segments {
                        for swap in level_swaps {
                            swap.set(true, materials);
                        }
                    }
                }
                RuneMode::Large => {
                    // 大机关：完成时仅亮第1级灯效 LEGGING_1
                    if let Some(first_level) = visual.legging_segments.get_mut(0) {
                        for swap in first_level {
                            swap.set(true, materials);
                        }
                    }
                }
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
    cache: &mut RuneMaterialCache,
    children: &Query<&Children>,
) -> Vec<RuneData> {
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

        let padding_entities = collect_entities_by(name_map, |name| {
            name.starts_with(&format!("{}_PADDING", prefix))
        });
        let progress_entities = collect_entities_by(name_map, |name| {
            name.starts_with(&format!("{}_LEGGING_PROGRESSING", prefix))
        });

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
                RuneIndex(logical_index, face_entity),
                CollisionEventsEnabled,
            )
        });

        let mut legging_segments = [vec![], vec![], vec![]];
        for legging_idx in 1..=3 {
            legging_segments[legging_idx - 1] = create_material_swaps(
                collect_entities_by(name_map, |name| {
                    name.starts_with(&format!("{}_LEGGING_{}", prefix, legging_idx))
                        && !name.contains("PROGRESSING")
                }),
                mesh_materials,
                materials,
                cache,
                children,
            )
        }

        targets.push(RuneData {
            visual: RuneVisual {
                active,
                disabled,
                legging_segments,
                padding_segments,
                progress_segments,
            },
            state: RuneState::Inactive,
            applied_state: RuneState::Inactive,
        });
    }
    targets
}

fn setup_power_rune(
    events: On<SceneInstanceReady>,
    mut commands: Commands,
    power_query: Query<(), With<PowerRuneRoot>>,
    scene_spawner: Res<SceneSpawner>,
    names: Query<&Name>,
    mut visibilities: Query<&mut Visibility>,
    mut mesh_materials: Query<&mut MeshMaterial3d<StandardMaterial>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut cache: ResMut<RuneMaterialCache>,
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
            RuneMode::Large
        } else {
            RuneMode::Small
        };
        println!("{:?}", mode);

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
                RuneTeam::Red
            } else {
                RuneTeam::Blue
            },
            mode,
            targets,
            (index & 1) > 0,
        ));
    }
}

#[derive(EntityEvent)]
pub struct RuneActivated {
    #[event_target]
    pub rune: Entity,
}

#[derive(EntityEvent)]
pub struct RuneHit {
    #[event_target]
    pub rune: Entity,
    pub result: HitResult,
}

fn handle_rune_collision(
    event: On<CollisionStart>,
    mut commands: Commands,
    mut hubs: Query<&mut EnergyHub>,
    targets: Query<&RuneIndex>,
    projectiles: Query<(), With<Projectile>>,
) {
    let Ok(&RuneIndex(index, hub_ent)) = targets.get(event.collider2) else {
        return;
    };
    let other = event.collider1;
    if !projectiles.contains(other) {
        return;
    }
    if let Ok(mut hub) = hubs.get_mut(hub_ent) {
        let mut rng = rand::thread_rng();
        let result = hub.on_target_hit(index, &mut rng);

        match hub.state {
            MechanismState::Inactive { .. } => {
                commands.trigger(RuneHit {
                    rune: hub_ent,
                    result,
                });
            }
            MechanismState::Activating(_) => {
                commands.trigger(RuneHit {
                    rune: hub_ent,
                    result,
                });
            }
            MechanismState::Activated { .. } => {
                commands.trigger(RuneActivated { rune: hub_ent });
            }
            MechanismState::Failed { .. } => {
                commands.trigger(RuneHit {
                    rune: hub_ent,
                    result,
                });
            }
        }
    }
}

fn rune_activation_tick(time: Res<Time>, mut hubs: Query<&mut EnergyHub>) {
    let delta = time.delta();
    let mut rng = rand::thread_rng();
    for mut hub in &mut hubs {
        let action = match &mut hub.state {
            MechanismState::Inactive { wait } => {
                if wait.tick(delta).just_finished() {
                    Some(RuneAction::StartActivating)
                } else {
                    None
                }
            }
            MechanismState::Activating(state) => {
                let mut action = None;

                // 检查20秒全局激活超时（最高优先级）
                if state.global_timeout.tick(delta).just_finished() {
                    action = Some(RuneAction::Failure); // 20秒全局超时激活失败
                }
                // 否则检查激活窗口超时
                else if state.timeout.tick(delta).just_finished() {
                    action = match state.window {
                        ActivationWindow::Primary => Some(RuneAction::Failure), // 2.5秒超时激活失败
                        ActivationWindow::Secondary => Some(RuneAction::NewRound), // 1秒窗口过期进入下一轮
                    };
                }

                action
            }
            MechanismState::Activated { wait } => {
                if wait.tick(delta).just_finished() {
                    Some(RuneAction::ResetToInactive)
                } else {
                    None
                }
            }
            MechanismState::Failed { wait } => {
                if wait.tick(delta).just_finished() {
                    Some(RuneAction::ResetToInactive)
                } else {
                    None
                }
            }
        };

        if let Some(action) = action {
            match action {
                RuneAction::StartActivating => hub.enter_activating(&mut rng),
                RuneAction::NewRound => {
                    if let Some(state) = hub.build_new_round(&mut rng) {
                        hub.state = MechanismState::Activating(state);
                    } else {
                        hub.enter_activated();
                    }
                }
                RuneAction::Failure => hub.enter_failed(),
                RuneAction::ResetToInactive => hub.enter_inactive(),
            }
        }
    }
}

fn rune_apply_visuals(
    mut hubs: Query<&mut EnergyHub>,
    mut visibilities: Query<&mut Visibility>,
    mut materials: Query<&mut MeshMaterial3d<StandardMaterial>>,
) {
    for mut hub in &mut hubs {
        let s = hub.mode.clone();
        for target in &mut hub.targets {
            if target.state != target.applied_state {
                apply_target_visual(
                    &s,
                    &mut target.visual,
                    &target.state,
                    &mut visibilities,
                    &mut materials,
                );
                target.applied_state = target.state.clone();
            }
        }
    }
}

fn rune_rotation_system(time: Res<Time>, mut hubs: Query<(&mut Transform, &mut EnergyHub)>) {
    let dt = time.delta_secs();
    for (mut transform, mut hub) in &mut hubs {
        let mode = hub.mode.clone();
        // 只有在激活状态下大机关才使用变量旋转
        let speed = hub.rotation.current_speed(mode, dt);
        let angle = speed * dt;

        // 确保旋转方向正确：红方顺时针(正角)，蓝方逆时针(负角)
        transform.rotate_local_axis(hub.rotation.direction, angle);
    }
}

pub struct PowerRunePlugin;

impl bevy::app::Plugin for PowerRunePlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_resource::<RuneMaterialCache>()
            .add_observer(handle_rune_collision)
            .add_observer(setup_power_rune)
            .add_systems(
                Update,
                (
                    rune_activation_tick,
                    rune_apply_visuals,
                    rune_rotation_system,
                ),
            );
    }
}
