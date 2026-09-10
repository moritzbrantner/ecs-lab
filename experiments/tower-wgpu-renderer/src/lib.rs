#![cfg(target_arch = "wasm32")]

use std::{f32::consts::PI, mem};

use bytemuck::{Pod, Zeroable};
use js_sys::Float32Array;
use wasm_bindgen::prelude::*;
use web_sys::HtmlCanvasElement;

const MAX_BODIES: usize = 64;
const BOX_EDGE_COUNT: usize = 12;
const LINE_VERTICES_PER_BODY: usize = BOX_EDGE_COUNT * 2;
const SPHERE_STACKS: usize = 16;
const SPHERE_SEGMENTS: usize = 24;
const SPHERE_VERTEX_CAPACITY: usize = SPHERE_STACKS * SPHERE_SEGMENTS * 6;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth24Plus;
const EDGES: [(usize, usize); BOX_EDGE_COUNT] = [
    (0, 1),
    (0, 2),
    (0, 4),
    (1, 3),
    (1, 5),
    (2, 3),
    (2, 6),
    (3, 7),
    (4, 5),
    (4, 6),
    (5, 7),
    (6, 7),
];

const LINE_SHADER: &str = r#"
struct Camera {
    view_projection: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: Camera;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(input: VertexInput) -> VertexOutput {
    var output: VertexOutput;
    output.clip_position = camera.view_projection * vec4<f32>(input.position, 1.0);
    output.color = input.color;
    return output;
}

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    return input.color;
}
"#;

const SPHERE_SHADER: &str = r#"
struct Camera {
    view_projection: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: Camera;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec4<f32>,
};

@vertex
fn vs_main(input: VertexInput) -> VertexOutput {
    var output: VertexOutput;
    output.clip_position = camera.view_projection * vec4<f32>(input.position, 1.0);
    output.normal = input.normal;
    output.color = input.color;
    return output;
}

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    let light_direction = normalize(vec3<f32>(0.45, 0.8, 0.55));
    let diffuse = max(dot(normalize(input.normal), light_direction), 0.0);
    let light = 0.3 + 0.7 * diffuse;
    return vec4<f32>(input.color.rgb * light, input.color.a);
}
"#;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct CameraUniform {
    view_projection: [f32; 16],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct LineVertex {
    position: [f32; 3],
    color: [f32; 4],
}

impl LineVertex {
    const ATTRIBUTES: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4];

    fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBUTES,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct SphereVertex {
    position: [f32; 3],
    normal: [f32; 3],
    color: [f32; 4],
}

impl SphereVertex {
    const ATTRIBUTES: [wgpu::VertexAttribute; 3] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x4];

    fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBUTES,
        }
    }
}

#[wasm_bindgen]
pub struct TowerRenderer {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    line_buffer: wgpu::Buffer,
    sphere_buffer: wgpu::Buffer,
    line_pipeline: wgpu::RenderPipeline,
    sphere_pipeline: wgpu::RenderPipeline,
    depth_texture: wgpu::Texture,
}

#[wasm_bindgen]
impl TowerRenderer {
    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        vertices: Float32Array,
        width: u32,
        height: u32,
        yaw: f32,
        pitch: f32,
        radius: f32,
        target_x: f32,
        target_y: f32,
        target_z: f32,
        dark: bool,
    ) -> Result<(), JsValue> {
        let values = vertices.to_vec();
        if values.len() % 24 != 0 {
            return Err(JsValue::from_str(
                "tower renderer requires eight xyz vertices per body",
            ));
        }
        let body_count = values.len() / 24;
        if !(2..=MAX_BODIES).contains(&body_count) {
            return Err(JsValue::from_str(
                "tower renderer received an invalid body count",
            ));
        }
        if width == 0 || height == 0 {
            return Ok(());
        }
        if self.config.width != width || self.config.height != height {
            self.resize(width, height);
        }

        let camera = CameraUniform {
            view_projection: camera_view_projection(
                yaw,
                pitch,
                radius,
                [target_x, target_y, target_z],
                width as f32 / height as f32,
            ),
        };
        self.queue
            .write_buffer(&self.camera_buffer, 0, bytemuck::bytes_of(&camera));

        let line_vertices = pack_box_lines(&values, body_count, dark)?;
        let sphere_vertices = pack_projectile_sphere(&values, dark)?;
        self.queue
            .write_buffer(&self.line_buffer, 0, bytemuck::cast_slice(&line_vertices));
        self.queue.write_buffer(
            &self.sphere_buffer,
            0,
            bytemuck::cast_slice(&sphere_vertices),
        );

        let frame = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame)
            | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => frame,
            wgpu::CurrentSurfaceTexture::Outdated => {
                self.resize(width, height);
                return Ok(());
            }
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => {
                return Ok(());
            }
            wgpu::CurrentSurfaceTexture::Lost => {
                return Err(JsValue::from_str("tower wgpu surface was lost"));
            }
            wgpu::CurrentSurfaceTexture::Validation => {
                return Err(JsValue::from_str("tower wgpu surface validation failed"));
            }
        };
        let color_view = frame
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let depth_view = self
            .depth_texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let clear = if dark {
            wgpu::Color {
                r: 0.035,
                g: 0.04,
                b: 0.05,
                a: 1.0,
            }
        } else {
            wgpu::Color {
                r: 0.96,
                g: 0.96,
                b: 0.95,
                a: 1.0,
            }
        };
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("tower renderer encoder"),
            });
        {
            let color_attachments = [Some(wgpu::RenderPassColorAttachment {
                view: &color_view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(clear),
                    store: wgpu::StoreOp::Store,
                },
            })];
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("tower renderer pass"),
                color_attachments: &color_attachments,
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &self.camera_bind_group, &[]);
            pass.set_pipeline(&self.line_pipeline);
            pass.set_vertex_buffer(0, self.line_buffer.slice(..));
            pass.draw(
                0..u32::try_from(line_vertices.len()).unwrap_or_default(),
                0..1,
            );
            pass.set_pipeline(&self.sphere_pipeline);
            pass.set_vertex_buffer(0, self.sphere_buffer.slice(..));
            pass.draw(
                0..u32::try_from(sphere_vertices.len()).unwrap_or_default(),
                0..1,
            );
        }
        self.queue.submit(std::iter::once(encoder.finish()));
        self.queue.present(frame);
        Ok(())
    }
}

impl TowerRenderer {
    async fn new(canvas: HtmlCanvasElement) -> Result<Self, JsValue> {
        let width = canvas.width().max(1);
        let height = canvas.height().max(1);
        let instance = wgpu::Instance::default();
        let surface: wgpu::Surface<'static> = instance
            .create_surface(wgpu::SurfaceTarget::Canvas(canvas))
            .map_err(js_error)?;
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                ..Default::default()
            })
            .await
            .map_err(js_error)?;
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("ECS tower Rust/Wasm wgpu device"),
                ..Default::default()
            })
            .await
            .map_err(js_error)?;
        let mut config = surface
            .get_default_config(&adapter, width, height)
            .ok_or_else(|| {
                JsValue::from_str("wgpu surface has no supported default configuration")
            })?;
        config.desired_maximum_frame_latency = 2;
        surface.configure(&device, &config);

        let camera_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("tower camera uniform"),
            size: mem::size_of::<CameraUniform>() as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let camera_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("tower camera layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("tower camera bind group"),
            layout: &camera_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("tower renderer pipeline layout"),
            bind_group_layouts: &[Some(&camera_layout)],
            immediate_size: 0,
        });
        let line_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("tower line shader"),
            source: wgpu::ShaderSource::Wgsl(LINE_SHADER.into()),
        });
        let sphere_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("tower sphere shader"),
            source: wgpu::ShaderSource::Wgsl(SPHERE_SHADER.into()),
        });
        let line_targets = [Some(wgpu::ColorTargetState {
            format: config.format,
            blend: None,
            write_mask: wgpu::ColorWrites::ALL,
        })];
        let line_buffers = [Some(LineVertex::layout())];
        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("tower line pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &line_shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &line_buffers,
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &line_shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &line_targets,
            }),
            multiview_mask: None,
            cache: None,
        });
        let sphere_targets = [Some(wgpu::ColorTargetState {
            format: config.format,
            blend: None,
            write_mask: wgpu::ColorWrites::ALL,
        })];
        let sphere_buffers = [Some(SphereVertex::layout())];
        let sphere_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("tower sphere pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &sphere_shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &sphere_buffers,
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &sphere_shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &sphere_targets,
            }),
            multiview_mask: None,
            cache: None,
        });
        let line_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("tower line vertices"),
            size: (MAX_BODIES * LINE_VERTICES_PER_BODY * mem::size_of::<LineVertex>())
                as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let sphere_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("tower sphere vertices"),
            size: (SPHERE_VERTEX_CAPACITY * mem::size_of::<SphereVertex>()) as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let depth_texture = create_depth_texture(&device, width, height);

        Ok(Self {
            surface,
            device,
            queue,
            config,
            camera_buffer,
            camera_bind_group,
            line_buffer,
            sphere_buffer,
            line_pipeline,
            sphere_pipeline,
            depth_texture,
        })
    }

    fn resize(&mut self, width: u32, height: u32) {
        self.config.width = width.max(1);
        self.config.height = height.max(1);
        self.surface.configure(&self.device, &self.config);
        self.depth_texture =
            create_depth_texture(&self.device, self.config.width, self.config.height);
    }
}

#[wasm_bindgen]
pub async fn create_tower_renderer(canvas: HtmlCanvasElement) -> Result<TowerRenderer, JsValue> {
    TowerRenderer::new(canvas).await
}

fn pack_box_lines(
    values: &[f32],
    body_count: usize,
    dark: bool,
) -> Result<Vec<LineVertex>, JsValue> {
    let mut output = Vec::with_capacity((body_count - 1) * LINE_VERTICES_PER_BODY);
    for body in 0..body_count {
        if body == 1 {
            continue;
        }
        let color = if body == 0 {
            if dark {
                [0.62, 0.64, 0.67, 0.7]
            } else {
                [0.28, 0.3, 0.32, 0.7]
            }
        } else if dark {
            [0.88, 0.88, 0.86, 1.0]
        } else {
            [0.16, 0.16, 0.15, 1.0]
        };
        let base = body * 24;
        for (left, right) in EDGES {
            output.push(LineVertex {
                position: read_vertex(values, base, left)?,
                color,
            });
            output.push(LineVertex {
                position: read_vertex(values, base, right)?,
                color,
            });
        }
    }
    Ok(output)
}

fn pack_projectile_sphere(values: &[f32], dark: bool) -> Result<Vec<SphereVertex>, JsValue> {
    if values.len() < 48 {
        return Err(JsValue::from_str(
            "tower renderer is missing projectile vertices",
        ));
    }
    let base = 24;
    let mut center = [0.0_f32; 3];
    for vertex in 0..8 {
        let point = read_vertex(values, base, vertex)?;
        for axis in 0..3 {
            center[axis] += point[axis] / 8.0;
        }
    }
    let corner = read_vertex(values, base, 0)?;
    let diagonal = ((corner[0] - center[0]).powi(2)
        + (corner[1] - center[1]).powi(2)
        + (corner[2] - center[2]).powi(2))
    .sqrt();
    let radius = diagonal / 3.0_f32.sqrt();
    if !radius.is_finite() || radius <= 0.0 {
        return Err(JsValue::from_str("tower projectile radius is invalid"));
    }
    let color = if dark {
        [0.95, 0.68, 0.22, 1.0]
    } else {
        [0.52, 0.28, 0.03, 1.0]
    };
    let mut output = Vec::with_capacity(SPHERE_VERTEX_CAPACITY);
    for stack in 0..SPHERE_STACKS {
        let theta0 = PI * stack as f32 / SPHERE_STACKS as f32;
        let theta1 = PI * (stack + 1) as f32 / SPHERE_STACKS as f32;
        for segment in 0..SPHERE_SEGMENTS {
            let phi0 = 2.0 * PI * segment as f32 / SPHERE_SEGMENTS as f32;
            let phi1 = 2.0 * PI * (segment + 1) as f32 / SPHERE_SEGMENTS as f32;
            let n00 = sphere_normal(theta0, phi0);
            let n01 = sphere_normal(theta0, phi1);
            let n10 = sphere_normal(theta1, phi0);
            let n11 = sphere_normal(theta1, phi1);
            push_sphere_triangle(&mut output, center, radius, color, [n00, n11, n10]);
            push_sphere_triangle(&mut output, center, radius, color, [n00, n01, n11]);
        }
    }
    Ok(output)
}

fn push_sphere_triangle(
    output: &mut Vec<SphereVertex>,
    center: [f32; 3],
    radius: f32,
    color: [f32; 4],
    normals: [[f32; 3]; 3],
) {
    for normal in normals {
        output.push(SphereVertex {
            position: [
                center[0] + normal[0] * radius,
                center[1] + normal[1] * radius,
                center[2] + normal[2] * radius,
            ],
            normal,
            color,
        });
    }
}

fn sphere_normal(theta: f32, phi: f32) -> [f32; 3] {
    let sin_theta = theta.sin();
    [sin_theta * phi.cos(), theta.cos(), sin_theta * phi.sin()]
}

fn read_vertex(values: &[f32], body_base: usize, vertex: usize) -> Result<[f32; 3], JsValue> {
    let offset = body_base + vertex * 3;
    let Some(point) = values.get(offset..offset + 3) else {
        return Err(JsValue::from_str(
            "tower renderer vertex buffer is truncated",
        ));
    };
    Ok([point[0], point[1], point[2]])
}

fn create_depth_texture(device: &wgpu::Device, width: u32, height: u32) -> wgpu::Texture {
    device.create_texture(&wgpu::TextureDescriptor {
        label: Some("tower depth texture"),
        size: wgpu::Extent3d {
            width: width.max(1),
            height: height.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    })
}

fn camera_view_projection(
    yaw_degrees: f32,
    pitch_degrees: f32,
    radius: f32,
    target: [f32; 3],
    aspect: f32,
) -> [f32; 16] {
    let yaw = yaw_degrees.to_radians();
    let pitch = pitch_degrees.to_radians();
    let horizontal = radius * pitch.cos();
    let eye = [
        target[0] + horizontal * yaw.sin(),
        target[1] + radius * pitch.sin(),
        target[2] + horizontal * yaw.cos(),
    ];
    let view = look_at(eye, target, [0.0, 1.0, 0.0]);
    let projection = perspective(46.0_f32.to_radians(), aspect.max(0.01), 0.1, 180.0);
    multiply_mat4(projection, view)
}

fn look_at(eye: [f32; 3], target: [f32; 3], up: [f32; 3]) -> [f32; 16] {
    let z = normalize([eye[0] - target[0], eye[1] - target[1], eye[2] - target[2]]);
    let x = normalize(cross(up, z));
    let y = cross(z, x);
    [
        x[0],
        y[0],
        z[0],
        0.0,
        x[1],
        y[1],
        z[1],
        0.0,
        x[2],
        y[2],
        z[2],
        0.0,
        -dot(x, eye),
        -dot(y, eye),
        -dot(z, eye),
        1.0,
    ]
}

fn perspective(fov: f32, aspect: f32, near: f32, far: f32) -> [f32; 16] {
    let f = 1.0 / (fov / 2.0).tan();
    [
        f / aspect,
        0.0,
        0.0,
        0.0,
        0.0,
        f,
        0.0,
        0.0,
        0.0,
        0.0,
        far / (near - far),
        -1.0,
        0.0,
        0.0,
        (near * far) / (near - far),
        0.0,
    ]
}

fn multiply_mat4(left: [f32; 16], right: [f32; 16]) -> [f32; 16] {
    let mut output = [0.0_f32; 16];
    for column in 0..4 {
        for row in 0..4 {
            let mut value = 0.0;
            for index in 0..4 {
                value += left[index * 4 + row] * right[column * 4 + index];
            }
            output[column * 4 + row] = value;
        }
    }
    output
}

fn normalize(vector: [f32; 3]) -> [f32; 3] {
    let length = (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt();
    if length <= f32::EPSILON {
        return [0.0, 0.0, 0.0];
    }
    [vector[0] / length, vector[1] / length, vector[2] / length]
}

fn cross(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn dot(left: [f32; 3], right: [f32; 3]) -> f32 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn js_error(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&error.to_string())
}
