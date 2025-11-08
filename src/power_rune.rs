use std::collections::HashMap;

use avian3d::prelude::{CollisionEventsEnabled, CollisionStart};
use bevy::{
    app::Update,
    asset::{AssetId, Assets, Handle},
    ecs::{
        component::Component,
        entity::Entity,
        event::EntityEvent,
        hierarchy::Children,
        name::Name,
        observer::On,
        query::With,
        resource::Resource,
        system::{Commands, Query, Res, ResMut, SystemParam},
    },
    math::Dir3,
    pbr::StandardMaterial,
    scene::{SceneInstanceReady, SceneSpawner},
    time::{Time, Timer, TimerMode},
    transform::components::Transform,
};

use rand::{Rng, seq::SliceRandom};

use crate::{
    util::{extract_entities_by, insert_recursively},
    visibility::{Combined, Controller, MaterialBased, Param, VisibilityBased},
};

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

#[derive(Component, Clone)]
struct RuneIndex(usize, Entity);

struct RuneData {
    visual: RuneVisual,
    state: RuneState,
    applied_state: RuneState,
}

struct RuneVisual {
    target: VisibilityBased,
    legging_segments: [Vec<Combined<MaterialBased>>; 3],
    padding_segments: Vec<Combined<MaterialBased>>,
    progress_segments: Vec<Combined<VisibilityBased>>,
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
pub struct PowerRune {
    _team: RuneTeam,
    mode: RuneMode,
    r: VisibilityBased,
    state: MechanismState,
    targets: Vec<RuneData>,
    rotation: RotationController,
}

pub struct HitResult {
    pub accurate: bool,
    pub change_state: bool,
}

impl PowerRune {
    fn new(
        team: RuneTeam,
        mode: RuneMode,
        r: VisibilityBased,
        targets: Vec<RuneData>,
        clockwise: bool,
    ) -> Self {
        Self {
            _team: team,
            mode,
            r,
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

    fn apply_shared_visual(&mut self, param: &mut PowerRuneParam) {
        match &self.state {
            MechanismState::Inactive { .. } => {
                self.r.set(false, &mut param.appearance);
            }
            _ => {
                self.r.set(true, &mut param.appearance);
            }
        };
    }
}

#[derive(Resource, Default)]
pub struct MaterialCache {
    muted: HashMap<AssetId<StandardMaterial>, Handle<StandardMaterial>>,
}

impl MaterialCache {
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
        clone.emissive_exposure_weight = 0.0;
        let muted_handle = materials.add(clone);
        self.muted.insert(id, muted_handle.clone());
        muted_handle
    }
}

type Idk<'w, 's, 'g> = (
    Entity,
    &'g mut ResMut<'w, MaterialCache>,
    &'g mut Param<'w, 's>,
);

impl<'w, 's, 'a> TryFrom<Idk<'w, 's, 'a>> for MaterialBased {
    type Error = ();
    fn try_from(value: Idk<'w, 's, 'a>) -> Result<Self, Self::Error> {
        let (entity, cache, param) = value;
        if let Ok(mut mesh_material) = param.mesh_materials.get_mut(entity) {
            let off = cache.ensure_muted(&mesh_material.0, &mut param.materials);
            let on = std::mem::replace(&mut mesh_material.0, off.clone());
            Ok(MaterialBased {
                entity: entity,
                on: on,
                off: off,
            })
        } else {
            Err(())
        }
    }
}

impl<'w, 's, 'a> TryFrom<Idk<'w, 's, 'a>> for VisibilityBased {
    type Error = ();
    fn try_from(value: Idk<'w, 's, 'a>) -> Result<Self, Self::Error> {
        Ok(VisibilityBased {
            powered: value.0,
            unpowered: None,
        })
    }
}

fn entity_recursive_generate<'w, 's, T: for<'g> TryFrom<Idk<'w, 's, 'g>>>(
    entity: Entity,
    swaps: &mut Vec<T>,
    param: &mut PowerRuneParam<'w, 's>,
) {
    if let Ok(v) = T::try_from((entity, &mut param.cache, &mut param.appearance)) {
        swaps.push(v);
    }
    let children = param.children;
    if let Ok(children) = children.get(entity).clone() {
        for child in children {
            entity_recursive_generate(*child, swaps, param);
        }
    }
}

fn create_controller<'w, 's, T: Controller + for<'a> TryFrom<Idk<'w, 's, 'a>>>(
    entities: Vec<Entity>,
    param: &mut PowerRuneParam<'w, 's>,
) -> Vec<Combined<T>> {
    let mut controllers = Vec::new();
    for entity in entities {
        let mut swaps = Vec::new();
        entity_recursive_generate(entity, &mut swaps, param);
        if swaps.is_empty() {
            continue;
        }
        controllers.push(Combined::<T>(swaps));
    }
    controllers
}

fn apply_target_visual(
    mode: &RuneMode,
    visual: &mut RuneVisual,
    state: &RuneState,
    param: &mut PowerRuneParam,
) {
    match state {
        RuneState::Inactive => {
            visual.target.set(false, &mut param.appearance);
            for swap in &mut visual.legging_segments {
                for swap in swap {
                    swap.set(false, &mut param.appearance);
                }
            }
            for swap in &mut visual.padding_segments {
                swap.set(false, &mut param.appearance);
            }
            for swap in &mut visual.progress_segments {
                swap.set(false, &mut param.appearance);
            }
        }
        RuneState::Highlighted => {
            visual.target.set(true, &mut param.appearance);
            for swap in &mut visual.padding_segments {
                swap.set(false, &mut param.appearance);
            }
            // 高亮状态：关闭所有灯效，只显示进度灯效
            for level_swaps in &mut visual.legging_segments {
                for swap in level_swaps {
                    swap.set(false, &mut param.appearance);
                }
            }

            if mode == &RuneMode::Large {
                for swap in &mut visual.progress_segments {
                    swap.set(true, &mut param.appearance);
                }
            }
        }
        RuneState::Completed => {
            visual.target.set(false, &mut param.appearance);
            // 根据模式设置灯效：小机关全亮，大机关亮第1级
            for swap in &mut visual.padding_segments {
                swap.set(true, &mut param.appearance);
            }
            for swap in &mut visual.progress_segments {
                swap.set(false, &mut param.appearance);
            }

            match mode {
                &RuneMode::Small => {
                    // 小机关：完成时所有灯臂全亮 LEGGING_1,2,3
                    for level_swaps in &mut visual.legging_segments {
                        for swap in level_swaps {
                            swap.set(true, &mut param.appearance);
                        }
                    }
                }
                &RuneMode::Large => {
                    // 大机关：完成时仅亮第1级灯效 LEGGING_1
                    if let Some(first_level) = visual.legging_segments.get_mut(0) {
                        for swap in first_level {
                            swap.set(true, &mut param.appearance);
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
    name_map: &mut HashMap<&str, Entity>,
    param: &mut PowerRuneParam,
) -> Vec<RuneData> {
    let mut targets = Vec::new();
    for target_idx in 1..=5 {
        let p = format!("FACE_{}_TARGET_{}", face_index, target_idx);
        let a = format!("{}_ACTIVE", p);
        let d = format!("{}_DISABLED", p);
        let prefix = p.as_str();
        let active_name = a.as_str();
        let disabled_name = d.as_str();

        let Some(&powered) = name_map.get(active_name) else {
            continue;
        };
        let Some(&unpowered) = name_map.get(disabled_name) else {
            continue;
        };

        let padding_segments = create_controller(
            extract_entities_by(name_map, |name| {
                name.starts_with(&format!("{}_PADDING", prefix))
            }),
            param,
        );
        let progress_segments = create_controller(
            extract_entities_by(name_map, |name| {
                name.starts_with(&format!("{}_LEGGING_PROGRESSING", prefix))
            }),
            param,
        );

        let collision_entity = if name_map.contains_key(&disabled_name) {
            unpowered
        } else {
            powered
        };

        let logical_index = targets.len();
        insert_recursively(
            &mut param.commands,
            collision_entity,
            &mut param.children,
            (
                RuneIndex(logical_index, face_entity),
                CollisionEventsEnabled,
            ),
        );

        let mut legging_segments = [vec![], vec![], vec![]];
        for legging_idx in 1..=3 {
            legging_segments[legging_idx - 1] = create_controller(
                extract_entities_by(name_map, |name| {
                    name.starts_with(&format!("{}_LEGGING_{}", prefix, legging_idx))
                        && !name.contains("PROGRESSING")
                }),
                param,
            )
        }

        targets.push(RuneData {
            visual: RuneVisual {
                target: VisibilityBased {
                    powered,
                    unpowered: Some(unpowered),
                },
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

#[derive(SystemParam)]
struct PowerRuneParam<'w, 's> {
    commands: Commands<'w, 's>,
    scene_spawner: Res<'w, SceneSpawner>,
    cache: ResMut<'w, MaterialCache>,

    power_query: Query<'w, 's, (), With<PowerRuneRoot>>,
    names: Query<'w, 's, &'static Name>,
    children: Query<'w, 's, &'static Children>,
    appearance: Param<'w, 's>,
}

fn setup_power_rune(events: On<SceneInstanceReady>, mut param: PowerRuneParam) {
    if !param.power_query.contains(events.entity) {
        return;
    }

    let names = param.names;
    let mut name_map = param
        .scene_spawner
        .iter_instance_entities(events.instance_id)
        .filter_map(|entity| names.get(entity).map(|n| (n.as_str(), entity)).ok())
        .fold(HashMap::new(), |mut m, (name, entity)| {
            m.insert(name, entity);
            m
        });

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

        let mut targets = build_targets(index, face_entity, &mut name_map, &mut param);
        for target in &mut targets {
            apply_target_visual(&mode, &mut target.visual, &RuneState::Inactive, &mut param);
        }

        if targets.is_empty() {
            continue;
        }

        param.commands.entity(face_entity).insert(PowerRune::new(
            if (index & 1) > 0 {
                RuneTeam::Red
            } else {
                RuneTeam::Blue
            },
            mode,
            VisibilityBased {
                powered: name_map
                    .remove(format!("FACE_{}_R_POWERED", index).as_str())
                    .unwrap(),
                unpowered: name_map.remove(format!("FACE_{}_R_UNPOWERED", index).as_str()),
            },
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
    mut runes: Query<&mut PowerRune>,
    targets: Query<&RuneIndex>,
    projectiles: Query<(), With<Projectile>>,
    mut param: PowerRuneParam,
) {
    let Ok(&RuneIndex(index, rune_ent)) = targets.get(event.collider2) else {
        return;
    };
    let other = event.collider1;
    if !projectiles.contains(other) {
        return;
    }
    if let Ok(mut rune) = runes.get_mut(rune_ent) {
        let mut rng = rand::thread_rng();
        let result = rune.on_target_hit(index, &mut rng);

        match rune.state {
            MechanismState::Inactive { .. } => {
                commands.trigger(RuneHit {
                    rune: rune_ent,
                    result,
                });
            }
            MechanismState::Activating(_) => {
                commands.trigger(RuneHit {
                    rune: rune_ent,
                    result,
                });
            }
            MechanismState::Activated { .. } => {
                for rune in &mut rune.targets {
                    for leg in &mut rune.visual.legging_segments {
                        for leg in leg {
                            leg.set(true, &mut param.appearance);
                        }
                    }
                    for swap in &mut rune.visual.progress_segments {
                        swap.set(false, &mut param.appearance);
                    }
                }
                if result.change_state {
                    commands.trigger(RuneActivated { rune: rune_ent });
                } else {
                    commands.trigger(RuneHit {
                        rune: rune_ent,
                        result,
                    });
                }
            }
            MechanismState::Failed { .. } => {
                commands.trigger(RuneHit {
                    rune: rune_ent,
                    result,
                });
            }
        }
    }
}

fn rune_activation_tick(time: Res<Time>, mut runes: Query<&mut PowerRune>) {
    let delta = time.delta();
    let mut rng = rand::thread_rng();
    for mut rune in &mut runes {
        let action = match &mut rune.state {
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
                RuneAction::StartActivating => rune.enter_activating(&mut rng),
                RuneAction::NewRound => {
                    if let Some(state) = rune.build_new_round(&mut rng) {
                        rune.state = MechanismState::Activating(state);
                    } else {
                        rune.enter_activated();
                    }
                }
                RuneAction::Failure => rune.enter_failed(),
                RuneAction::ResetToInactive => rune.enter_inactive(),
            }
        }
    }
}

fn rune_apply_visuals(mut runes: Query<&mut PowerRune>, mut param: PowerRuneParam) {
    for mut rune in &mut runes {
        rune.apply_shared_visual(&mut param);
        let s = rune.mode.clone();
        for target in &mut rune.targets {
            if target.state != target.applied_state {
                apply_target_visual(&s, &mut target.visual, &target.state, &mut param);
                target.applied_state = target.state.clone();
            }
        }
    }
}

fn rune_rotation_system(time: Res<Time>, mut runes: Query<(&mut Transform, &mut PowerRune)>) {
    let dt = time.delta_secs();
    for (mut transform, mut rune) in &mut runes {
        let mode = rune.mode.clone();
        // 只有在激活状态下大机关才使用变量旋转
        let speed = rune.rotation.current_speed(mode, dt);
        let angle = speed * dt;

        // 确保旋转方向正确：红方顺时针(正角)，蓝方逆时针(负角)
        transform.rotate_local_axis(rune.rotation.direction, angle);
    }
}

pub struct PowerRunePlugin;

impl bevy::app::Plugin for PowerRunePlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_resource::<MaterialCache>()
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
