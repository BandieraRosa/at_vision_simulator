use crate::ros2::plugin::MainCamera;
use bevy::anti_alias::fxaa::Fxaa;
use bevy::camera::Exposure;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::pbr::Atmosphere;
use bevy::post_process::motion_blur::MotionBlur;
use bevy::render::view::Hdr;
use bevy::{
    image::TextureFormatPixelInfo,
    prelude::*,
    render::{
        render_asset::RenderAssets, render_graph::{self, NodeRunError, RenderGraph, RenderGraphContext, RenderLabel}, render_resource::{
            Buffer, BufferDescriptor, BufferUsages, CommandEncoderDescriptor, Extent3d, MapMode,
            PollType, TexelCopyBufferInfo, TexelCopyBufferLayout, TextureFormat, TextureUsages,
        }, renderer::{RenderContext, RenderDevice, RenderQueue},
        Extract,
        Render,
        RenderApp,
        RenderSystems,
    },
};
use crossbeam_channel::{Receiver, Sender};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

#[derive(Resource, Deref)]
struct MainWorldReceiver(Receiver<Vec<u8>>);

#[derive(Resource, Deref)]
struct RenderWorldSender(Sender<Vec<u8>>);

pub struct ImageCopyPlugin;
impl Plugin for ImageCopyPlugin {
    fn build(&self, app: &mut App) {
        let (s, r) = crossbeam_channel::unbounded();

        let render_app = app
            .insert_resource(MainWorldReceiver(r))
            .sub_app_mut(RenderApp);

        let mut graph = render_app.world_mut().resource_mut::<RenderGraph>();
        graph.add_node(ImageCopy, ImageCopyDriver);
        graph.add_node_edge(bevy::render::graph::CameraDriverLabel, ImageCopy);

        render_app
            .insert_resource(RenderWorldSender(s))
            .add_systems(ExtractSchedule, image_copy_extract)
            .add_systems(
                Render,
                receive_image_from_buffer.after(RenderSystems::Render),
            );
    }
}

#[derive(Clone, Default, Resource, Deref, DerefMut)]
struct ImageCopiers(pub Vec<ImageCopier>);

#[derive(Clone, Component)]
struct ImageCopier {
    buffer: Buffer,
    enabled: Arc<AtomicBool>,
    src_image: Handle<Image>,
}

impl ImageCopier {
    pub fn new(
        src_image: Handle<Image>,
        size: Extent3d,
        render_device: &RenderDevice,
    ) -> ImageCopier {
        let padded_bytes_per_row = RenderDevice::align_copy_bytes_per_row(size.width as usize) * 4;

        let cpu_buffer = render_device.create_buffer(&BufferDescriptor {
            label: None,
            size: padded_bytes_per_row as u64 * size.height as u64,
            usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        ImageCopier {
            buffer: cpu_buffer,
            src_image,
            enabled: Arc::new(AtomicBool::new(true)),
        }
    }

    pub fn enabled(&self) -> bool {
        self.enabled.load(Ordering::Relaxed)
    }
}

fn image_copy_extract(mut commands: Commands, image_copiers: Extract<Query<&ImageCopier>>) {
    commands.insert_resource(ImageCopiers(
        image_copiers.iter().cloned().collect::<Vec<ImageCopier>>(),
    ));
}

#[derive(Debug, PartialEq, Eq, Clone, Hash, RenderLabel)]
struct ImageCopy;

#[derive(Default)]
struct ImageCopyDriver;

impl render_graph::Node for ImageCopyDriver {
    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let image_copiers = world.get_resource::<ImageCopiers>().unwrap();
        let gpu_images = world
            .get_resource::<RenderAssets<bevy::render::texture::GpuImage>>()
            .unwrap();

        for image_copier in image_copiers.iter() {
            if !image_copier.enabled() {
                continue;
            }

            let src_image = gpu_images.get(&image_copier.src_image).unwrap();
            let mut encoder = render_context
                .render_device()
                .create_command_encoder(&CommandEncoderDescriptor::default());
            let block_dimensions = src_image.texture_format.block_dimensions();
            let block_size = src_image.texture_format.block_copy_size(None).unwrap();
            let padded_bytes_per_row = RenderDevice::align_copy_bytes_per_row(
                (src_image.size.width as usize / block_dimensions.0 as usize) * block_size as usize,
            );

            encoder.copy_texture_to_buffer(
                src_image.texture.as_image_copy(),
                TexelCopyBufferInfo {
                    buffer: &image_copier.buffer,
                    layout: TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(
                            std::num::NonZero::<u32>::new(padded_bytes_per_row as u32)
                                .unwrap()
                                .into(),
                        ),
                        rows_per_image: None,
                    },
                },
                src_image.size,
            );
            let render_queue = world.get_resource::<RenderQueue>().unwrap();
            render_queue.submit(std::iter::once(encoder.finish()));
        }
        Ok(())
    }
}

fn receive_image_from_buffer(
    image_copiers: Res<ImageCopiers>,
    render_device: Res<RenderDevice>,
    sender: Res<RenderWorldSender>,
) {
    for image_copier in image_copiers.0.iter() {
        if !image_copier.enabled() {
            continue;
        }

        let buffer_slice = image_copier.buffer.slice(..);
        let (s, r) = crossbeam_channel::bounded(1);

        buffer_slice.map_async(MapMode::Read, move |r| match r {
            Ok(r) => s.send(r).expect("Failed to send map update"),
            Err(err) => panic!("Failed to map buffer {err}"),
        });

        render_device
            .poll(PollType::Wait)
            .expect("Failed to poll device for map async");
        r.recv().expect("Failed to receive the map_async message");

        let _ = sender.send(buffer_slice.get_mapped_range().to_vec());
        image_copier.buffer.unmap();
    }
}

pub struct RosCapturePlugin {
    pub width: u32,
    pub height: u32,
}

impl Plugin for RosCapturePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(ImageCopyPlugin);

        app.insert_resource(CaptureConfig {
            width: self.width,
            height: self.height,
        })
        .add_systems(Update, sync_camera);

        app.add_systems(Startup, setup_capture_scene);

        app.add_systems(PostUpdate, publish_ros_image);
    }
}

#[derive(Resource)]
struct CaptureConfig {
    width: u32,
    height: u32,
}

#[derive(Component, Deref, DerefMut)]
struct CpuImageTarget(Handle<Image>);

#[derive(Component)]
struct CaptureCamera;

fn setup_capture_scene(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    render_device: Res<RenderDevice>,
    config: Res<CaptureConfig>,
) {
    let size = Extent3d {
        width: config.width,
        height: config.height,
        ..Default::default()
    };
    let mut render_target_image =
        Image::new_target_texture(size.width, size.height, TextureFormat::bevy_default());
    render_target_image.texture_descriptor.usage |= TextureUsages::COPY_SRC;
    let render_target_handle = images.add(render_target_image);

    let cpu_image =
        Image::new_target_texture(size.width, size.height, TextureFormat::bevy_default());
    let cpu_image_handle = images.add(cpu_image);

    commands.spawn(ImageCopier::new(
        render_target_handle.clone(),
        size,
        &render_device,
    ));
    commands.spawn(CpuImageTarget(cpu_image_handle));

    commands.spawn((
        Camera3d::default(),
        Atmosphere::EARTH,
        Camera {
            target: render_target_handle.clone().into(),
            ..default()
        },
        MotionBlur {
            shutter_angle: 0.25,
            samples: 4,
        },
        Projection::Perspective(PerspectiveProjection {
            fov: std::f32::consts::PI / 180.0 * 45.0,
            near: 0.1,
            far: 500000000.0,
            ..default()
        }),
        AmbientLight {
            brightness: 4000.0,
            color: Color::WHITE,
            ..default()
        },
        Exposure::SUNLIGHT,
        Tonemapping::AcesFitted,
        Msaa::Off,
        Fxaa::default(),
        Hdr,
        CaptureCamera,
    ));
}

fn sync_camera(
    target: Single<&Transform, (With<MainCamera>, Without<CaptureCamera>)>,
    mut our: Single<&mut Transform, (With<CaptureCamera>, Without<MainCamera>)>,
) {
    our.translation = target.translation;
    our.scale = target.scale;
    our.rotation = target.rotation;
}

#[derive(Event)]
pub struct Captured {
    pub image: Image,
}

fn publish_ros_image(
    mut commands: Commands,
    cpu_image_targets: Query<&CpuImageTarget>,
    receiver: Res<MainWorldReceiver>,
    mut images: ResMut<Assets<Image>>,
) {
    let mut image_data = Vec::new();
    while let Ok(data) = receiver.try_recv() {
        image_data = data;
    }

    if image_data.is_empty() {
        return;
    }

    if let Some(target) = cpu_image_targets.iter().next() {
        let mut bevy_image = images.get_mut(target.id()).unwrap().clone();

        let row_bytes = bevy_image.width() as usize
            * bevy_image.texture_descriptor.format.pixel_size().unwrap();
        let aligned_row_bytes = RenderDevice::align_copy_bytes_per_row(row_bytes);

        let hei = bevy_image.height();
        let target_data = bevy_image.data.get_or_insert_with(Vec::new);

        if row_bytes == aligned_row_bytes {
            target_data.clone_from(&image_data);
        } else {
            target_data.clear();
            target_data.extend(
                image_data
                    .chunks(aligned_row_bytes)
                    .take(hei as usize)
                    .flat_map(|row| &row[..row_bytes.min(row.len())])
                    .cloned(),
            );
        }
        commands.trigger(Captured { image: bevy_image })
    }
}
