import { runWebGpuAabbPairs } from "../webgpu.js";

const BODY_NAMES = Object.freeze({
  0: "A",
  1: "B",
  2: "C",
  3: "Floor",
  4: "Ceiling",
  5: "Left wall",
  6: "Right wall",
  7: "Back wall",
  8: "Front wall",
});
const HIDDEN_CUTAWAY_WALLS = new Set([4, 6, 8]);
const MAX_BODIES = 64;
const TRANSITION_MS = 560;
const DEFAULT_CAMERA = Object.freeze({ yaw: 38, pitch: 28, radius: 62 });
const prefersReducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

const stage = document.querySelector("#physics-stage");
const webgpuCanvas = document.querySelector("#physics-webgpu-canvas");
const fallbackCanvas = document.querySelector("#physics-fallback-canvas");
const rendererStatus = document.querySelector("#renderer-status");
const frameInput = document.querySelector("#frame");
const frameLabel = document.querySelector("#frame-label");
const resetButton = document.querySelector("#reset-frame");
const stepButton = document.querySelector("#step-frame");
const runButton = document.querySelector("#run-frames");
const yawInput = document.querySelector("#camera-yaw");
const pitchInput = document.querySelector("#camera-pitch");
const resetCameraButton = document.querySelector("#reset-camera");
const webgpuEnabled = document.querySelector("#webgpu-enabled");
const runtimeStatus = document.querySelector("#runtime-status");
const collisionStatus = document.querySelector("#collision-status");
const webgpuStatus = document.querySelector("#webgpu-status");

if (
  !(stage instanceof HTMLElement) ||
  !(webgpuCanvas instanceof HTMLCanvasElement) ||
  !(fallbackCanvas instanceof HTMLCanvasElement) ||
  !(rendererStatus instanceof HTMLElement) ||
  !(frameInput instanceof HTMLInputElement) ||
  !(frameLabel instanceof HTMLElement) ||
  !(resetButton instanceof HTMLButtonElement) ||
  !(stepButton instanceof HTMLButtonElement) ||
  !(runButton instanceof HTMLButtonElement) ||
  !(yawInput instanceof HTMLInputElement) ||
  !(pitchInput instanceof HTMLInputElement) ||
  !(resetCameraButton instanceof HTMLButtonElement) ||
  !(webgpuEnabled instanceof HTMLInputElement) ||
  !(runtimeStatus instanceof HTMLElement) ||
  !(collisionStatus instanceof HTMLElement) ||
  !(webgpuStatus instanceof HTMLElement)
) {
  throw new Error("The 3D physics demo markup is incomplete.");
}

let wasmExports = null;
let frames = [];
let renderer = null;
let visualFrame = null;
let transition = null;
let animationHandle = 0;
let running = false;
let renderRevision = 0;

function bodyLabel(entity) {
  return BODY_NAMES[entity.id] ?? `Entity ${entity.id}`;
}

function currentStep() {
  return Number.parseInt(frameInput.value, 10);
}

function readRustFrame(step) {
  const bodyCount = wasmExports.physics_demo_body_count(step);
  if (!Number.isInteger(bodyCount) || bodyCount <= 0 || bodyCount > MAX_BODIES) {
    throw new Error(`Rust returned an invalid 3D body count: ${bodyCount}`);
  }

  const entities = Array.from({ length: bodyCount }, (_, bodyIndex) => ({
    id: wasmExports.physics_demo_entity_id(bodyIndex, step) >>> 0,
    x: wasmExports.physics_demo_position_x(bodyIndex, step),
    y: wasmExports.physics_demo_position_y(bodyIndex, step),
    z: wasmExports.physics_demo_position_z(bodyIndex, step),
    halfX: wasmExports.physics_demo_half_extent_x(bodyIndex, step),
    halfY: wasmExports.physics_demo_half_extent_y(bodyIndex, step),
    halfZ: wasmExports.physics_demo_half_extent_z(bodyIndex, step),
    fixed: wasmExports.physics_demo_is_fixed(bodyIndex, step) === 1,
    mass: wasmExports.physics_demo_mass_units(bodyIndex, step),
    restitution: wasmExports.physics_demo_restitution_milli(bodyIndex, step),
    friction: wasmExports.physics_demo_friction_milli(bodyIndex, step),
  }));

  const wordCount = wasmExports.physics_demo_pair_word_count(step);
  const pairWords = new Uint32Array(wordCount);
  for (let index = 0; index < wordCount; index += 1) {
    pairWords[index] = wasmExports.physics_demo_pair_word(index, step) >>> 0;
  }
  return { step, entities, pairWords };
}

function interpolateFrame(left, right, amount) {
  const entities = left.entities.map((entity, index) => {
    const target = right.entities[index];
    return {
      ...entity,
      x: lerp(entity.x, target.x, amount),
      y: lerp(entity.y, target.y, amount),
      z: lerp(entity.z, target.z, amount),
    };
  });
  return { step: lerp(left.step, right.step, amount), entities, pairWords: left.pairWords };
}

function lerp(left, right, amount) {
  return left + (right - left) * amount;
}

function pairContacts(frame) {
  const labels = [];
  let pair = 0;
  for (let left = 0; left < frame.entities.length; left += 1) {
    for (let right = left + 1; right < frame.entities.length; right += 1) {
      const word = frame.pairWords[Math.floor(pair / 32)] ?? 0;
      const mask = (1 << (pair % 32)) >>> 0;
      if ((word & mask) !== 0) {
        labels.push(`${bodyLabel(frame.entities[left])} ↔ ${bodyLabel(frame.entities[right])}`);
      }
      pair += 1;
    }
  }
  return labels;
}

function updateCollisionStatus(frame) {
  const contacts = pairContacts(frame);
  if (contacts.length === 0) {
    collisionStatus.textContent = `Rust 3D frame ${frame.step}: no AABB contact.`;
    return;
  }
  collisionStatus.textContent = `Rust 3D frame ${frame.step}: ${contacts.join(", ")}.`;
}

function visibleEntities(entities) {
  return entities.filter((entity) => !HIDDEN_CUTAWAY_WALLS.has(entity.id));
}

function cameraState() {
  return {
    yaw: Number(yawInput.value),
    pitch: Number(pitchInput.value),
    radius: DEFAULT_CAMERA.radius,
    target: [0, 9, 0],
  };
}

function renderVisual(frame) {
  visualFrame = frame;
  renderer?.render(visibleEntities(frame.entities), cameraState());
}

function setExactStep(step, { verifyGpu = false } = {}) {
  const bounded = Math.max(0, Math.min(frames.length - 1, step));
  frameInput.value = String(bounded);
  frameLabel.textContent = String(bounded);
  const frame = frames[bounded];
  renderVisual(frame);
  updateCollisionStatus(frame);
  if (verifyGpu) {
    void verifyWebGpu(frame, ++renderRevision);
  } else if (webgpuEnabled.checked) {
    webgpuStatus.textContent = "WebGPU verification is armed and runs when this step settles.";
  }
}

function stopAnimation() {
  transition = null;
  if (animationHandle !== 0) {
    window.cancelAnimationFrame(animationHandle);
    animationHandle = 0;
  }
}

function stopRun({ verifyGpu = false } = {}) {
  running = false;
  runButton.textContent = "Run";
  stopAnimation();
  if (verifyGpu) {
    setExactStep(currentStep(), { verifyGpu: true });
  }
}

function transitionTo(nextStep, { continueRun = false, verifyGpu = true } = {}) {
  stopAnimation();
  const fromStep = currentStep();
  const toStep = Math.max(0, Math.min(frames.length - 1, nextStep));
  if (fromStep === toStep || prefersReducedMotion) {
    setExactStep(toStep, { verifyGpu });
    if (continueRun) {
      continueRunning();
    }
    return;
  }

  transition = {
    fromStep,
    toStep,
    started: performance.now(),
    duration: TRANSITION_MS,
    continueRun,
    verifyGpu,
  };
  animationHandle = window.requestAnimationFrame(animate);
}

function animate(now) {
  if (!transition) {
    animationHandle = 0;
    return;
  }
  const elapsed = now - transition.started;
  const linear = Math.min(1, elapsed / transition.duration);
  const eased = linear * linear * (3 - 2 * linear);
  const frame = interpolateFrame(
    frames[transition.fromStep],
    frames[transition.toStep],
    eased,
  );
  renderVisual(frame);
  frameLabel.textContent = `${transition.fromStep} → ${transition.toStep}`;

  if (linear < 1) {
    animationHandle = window.requestAnimationFrame(animate);
    return;
  }

  const completed = transition;
  transition = null;
  animationHandle = 0;
  setExactStep(completed.toStep, {
    verifyGpu: completed.verifyGpu && !completed.continueRun,
  });
  if (completed.continueRun) {
    continueRunning();
  }
}

function continueRunning() {
  if (!running) {
    return;
  }
  const step = currentStep();
  if (step >= frames.length - 1) {
    running = false;
    runButton.textContent = "Run";
    setExactStep(step, { verifyGpu: true });
    return;
  }
  transitionTo(step + 1, { continueRun: true, verifyGpu: false });
}

function startRun() {
  if (running) {
    stopRun({ verifyGpu: true });
    return;
  }
  if (currentStep() >= frames.length - 1) {
    setExactStep(0);
  }
  running = true;
  runButton.textContent = "Pause";
  if (webgpuEnabled.checked) {
    webgpuStatus.textContent = "Animation is running; exact WebGPU verification resumes when it pauses.";
  }
  continueRunning();
}

function packAabbs(entities) {
  const values = new Float32Array(entities.length * 8);
  entities.forEach((entity, index) => {
    const offset = index * 8;
    values[offset] = entity.x - entity.halfX;
    values[offset + 1] = entity.y - entity.halfY;
    values[offset + 2] = entity.z - entity.halfZ;
    values[offset + 3] = 0;
    values[offset + 4] = entity.x + entity.halfX;
    values[offset + 5] = entity.y + entity.halfY;
    values[offset + 6] = entity.z + entity.halfZ;
    values[offset + 7] = 0;
  });
  return values;
}

function exactWordsMatch(left, right) {
  return left.length === right.length && left.every((word, index) => word === right[index]);
}

async function verifyWebGpu(frame, revision) {
  if (!("gpu" in navigator)) {
    webgpuStatus.textContent = "WebGPU is unavailable; Rust 3D physics and the fallback renderer remain usable.";
    return;
  }
  if (!webgpuEnabled.checked) {
    webgpuStatus.textContent = "WebGPU collision verification is off; Rust remains authoritative.";
    return;
  }

  webgpuStatus.textContent = `Verifying Rust 3D AABBs for step ${frame.step} with WebGPU…`;
  try {
    const measurement = await runWebGpuAabbPairs(packAabbs(frame.entities), frame.entities.length);
    if (revision !== renderRevision) {
      return;
    }
    if (!exactWordsMatch(frame.pairWords, measurement.bitset)) {
      webgpuStatus.textContent =
        "WebGPU 3D pair mismatch: GPU evidence was rejected and Rust remains authoritative.";
      return;
    }
    webgpuStatus.textContent = `WebGPU exact 3D pair parity · ${measurement.totalMs.toFixed(2)} ms including setup/readback.`;
  } catch (error) {
    if (revision !== renderRevision) {
      return;
    }
    webgpuStatus.textContent = `WebGPU verification unavailable: ${error instanceof Error ? error.message : String(error)}. Rust remains authoritative.`;
  }
}

function bindControls() {
  frameInput.addEventListener("input", () => {
    stopRun();
    setExactStep(Number(frameInput.value));
  });
  frameInput.addEventListener("change", () => {
    setExactStep(Number(frameInput.value), { verifyGpu: true });
  });
  resetButton.addEventListener("click", () => {
    stopRun();
    transitionTo(0, { verifyGpu: true });
  });
  stepButton.addEventListener("click", () => {
    stopRun();
    transitionTo(currentStep() + 1, { verifyGpu: true });
  });
  runButton.addEventListener("click", startRun);
  yawInput.addEventListener("input", () => visualFrame && renderVisual(visualFrame));
  pitchInput.addEventListener("input", () => visualFrame && renderVisual(visualFrame));
  resetCameraButton.addEventListener("click", () => {
    yawInput.value = String(DEFAULT_CAMERA.yaw);
    pitchInput.value = String(DEFAULT_CAMERA.pitch);
    if (visualFrame) {
      renderVisual(visualFrame);
    }
  });
  webgpuEnabled.addEventListener("change", () => {
    if (running) {
      webgpuStatus.textContent = "Animation is running; exact verification resumes when it pauses.";
      return;
    }
    setExactStep(currentStep(), { verifyGpu: true });
  });
  window.addEventListener("resize", () => {
    if (visualFrame) {
      renderVisual(visualFrame);
    }
  });
}

async function loadWasm() {
  const response = await fetch("../pkg/ecs_web_demo.wasm");
  if (!response.ok) {
    throw new Error(`Wasm request failed with HTTP ${response.status}`);
  }
  const bytes = await response.arrayBuffer();
  const { instance } = await WebAssembly.instantiate(bytes, {});
  const requiredExports = [
    "physics_demo_max_steps",
    "physics_demo_body_count",
    "physics_demo_entity_id",
    "physics_demo_position_x",
    "physics_demo_position_y",
    "physics_demo_position_z",
    "physics_demo_half_extent_x",
    "physics_demo_half_extent_y",
    "physics_demo_half_extent_z",
    "physics_demo_is_fixed",
    "physics_demo_mass_units",
    "physics_demo_restitution_milli",
    "physics_demo_friction_milli",
    "physics_demo_pair_word_count",
    "physics_demo_pair_word",
  ];
  for (const name of requiredExports) {
    if (typeof instance.exports[name] !== "function") {
      throw new Error(`Wasm export ${name} is missing`);
    }
  }
  return instance.exports;
}

function cubeVertices() {
  const faces = [
    { normal: [0, 0, 1], corners: [[-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]] },
    { normal: [0, 0, -1], corners: [[1, -1, -1], [-1, -1, -1], [-1, 1, -1], [1, 1, -1]] },
    { normal: [1, 0, 0], corners: [[1, -1, 1], [1, -1, -1], [1, 1, -1], [1, 1, 1]] },
    { normal: [-1, 0, 0], corners: [[-1, -1, -1], [-1, -1, 1], [-1, 1, 1], [-1, 1, -1]] },
    { normal: [0, 1, 0], corners: [[-1, 1, 1], [1, 1, 1], [1, 1, -1], [-1, 1, -1]] },
    { normal: [0, -1, 0], corners: [[-1, -1, -1], [1, -1, -1], [1, -1, 1], [-1, -1, 1]] },
  ];
  const data = [];
  for (const face of faces) {
    for (const index of [0, 1, 2, 0, 2, 3]) {
      data.push(...face.corners[index], ...face.normal);
    }
  }
  return new Float32Array(data);
}

const RENDER_SHADER = /* wgsl */ `
struct Body {
  center: vec4<f32>,
  half: vec4<f32>,
  color: vec4<f32>,
}

struct VertexInput {
  @location(0) position: vec3<f32>,
  @location(1) normal: vec3<f32>,
}

struct VertexOutput {
  @builtin(position) clip_position: vec4<f32>,
  @location(0) normal: vec3<f32>,
  @location(1) color: vec4<f32>,
}

@group(0) @binding(0) var<uniform> view_projection: mat4x4<f32>;
@group(0) @binding(1) var<storage, read> bodies: array<Body>;

@vertex
fn vs_main(input: VertexInput, @builtin(instance_index) instance: u32) -> VertexOutput {
  let body = bodies[instance];
  let world = body.center.xyz + input.position * body.half.xyz;
  var output: VertexOutput;
  output.clip_position = view_projection * vec4<f32>(world, 1.0);
  output.normal = input.normal;
  output.color = body.color;
  return output;
}

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
  let light_direction = normalize(vec3<f32>(0.45, 0.8, 0.55));
  let light = 0.34 + 0.66 * max(dot(normalize(input.normal), light_direction), 0.0);
  return vec4<f32>(input.color.rgb * light, input.color.a);
}
`;

async function createWebGpuRenderer(canvas) {
  if (!("gpu" in navigator)) {
    return null;
  }
  const adapter = await navigator.gpu.requestAdapter({ powerPreference: "high-performance" });
  if (!adapter) {
    return null;
  }
  const device = await adapter.requestDevice();
  const context = canvas.getContext("webgpu");
  if (!context) {
    device.destroy();
    return null;
  }
  const format = navigator.gpu.getPreferredCanvasFormat();
  context.configure({ device, format, alphaMode: "opaque" });

  const vertices = cubeVertices();
  const vertexBuffer = device.createBuffer({
    size: vertices.byteLength,
    usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
  });
  device.queue.writeBuffer(vertexBuffer, 0, vertices);
  const uniformBuffer = device.createBuffer({
    size: 64,
    usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
  });
  const bodyBuffer = device.createBuffer({
    size: MAX_BODIES * 48,
    usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST,
  });
  const module = device.createShaderModule({ code: RENDER_SHADER });
  const compilation = await module.getCompilationInfo();
  if (compilation.messages.some((message) => message.type === "error")) {
    device.destroy();
    return null;
  }
  const pipeline = await device.createRenderPipelineAsync({
    layout: "auto",
    vertex: {
      module,
      entryPoint: "vs_main",
      buffers: [{
        arrayStride: 24,
        attributes: [
          { shaderLocation: 0, offset: 0, format: "float32x3" },
          { shaderLocation: 1, offset: 12, format: "float32x3" },
        ],
      }],
    },
    fragment: { module, entryPoint: "fs_main", targets: [{ format }] },
    primitive: { topology: "triangle-list", cullMode: "back" },
    depthStencil: { format: "depth24plus", depthWriteEnabled: true, depthCompare: "less" },
  });
  const bindGroup = device.createBindGroup({
    layout: pipeline.getBindGroupLayout(0),
    entries: [
      { binding: 0, resource: { buffer: uniformBuffer } },
      { binding: 1, resource: { buffer: bodyBuffer } },
    ],
  });
  let depthTexture = null;
  let depthWidth = 0;
  let depthHeight = 0;

  return {
    mode: "webgpu",
    render(entities, camera) {
      const { width, height } = resizeCanvas(canvas);
      if (width !== depthWidth || height !== depthHeight) {
        depthTexture?.destroy();
        depthTexture = device.createTexture({
          size: [width, height],
          format: "depth24plus",
          usage: GPUTextureUsage.RENDER_ATTACHMENT,
        });
        depthWidth = width;
        depthHeight = height;
      }

      const viewProjection = cameraViewProjection(camera, width / height);
      device.queue.writeBuffer(uniformBuffer, 0, viewProjection);
      const bodyData = new Float32Array(entities.length * 12);
      entities.forEach((entity, index) => {
        const offset = index * 12;
        bodyData.set([entity.x, entity.y, entity.z, 0], offset);
        bodyData.set([entity.halfX, entity.halfY, entity.halfZ, 0], offset + 4);
        bodyData.set(bodyColor(entity), offset + 8);
      });
      device.queue.writeBuffer(bodyBuffer, 0, bodyData);

      const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
      const encoder = device.createCommandEncoder();
      const pass = encoder.beginRenderPass({
        colorAttachments: [{
          view: context.getCurrentTexture().createView(),
          clearValue: dark ? { r: 0.035, g: 0.04, b: 0.05, a: 1 } : { r: 0.96, g: 0.96, b: 0.95, a: 1 },
          loadOp: "clear",
          storeOp: "store",
        }],
        depthStencilAttachment: {
          view: depthTexture.createView(),
          depthClearValue: 1,
          depthLoadOp: "clear",
          depthStoreOp: "store",
        },
      });
      pass.setPipeline(pipeline);
      pass.setBindGroup(0, bindGroup);
      pass.setVertexBuffer(0, vertexBuffer);
      pass.draw(36, entities.length);
      pass.end();
      device.queue.submit([encoder.finish()]);
    },
  };
}

function createFallbackRenderer(canvas) {
  const context = canvas.getContext("2d");
  if (!context) {
    throw new Error("The browser cannot create the fallback 3D canvas renderer.");
  }
  return {
    mode: "canvas",
    render(entities, camera) {
      const { width, height, ratio } = resizeCanvas(canvas);
      const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      context.clearRect(0, 0, width / ratio, height / ratio);
      context.fillStyle = dark ? "#0b0c0f" : "#f5f5f2";
      context.fillRect(0, 0, width / ratio, height / ratio);
      const matrix = cameraViewProjection(camera, (width / ratio) / (height / ratio));
      const ordered = [...entities].sort((left, right) => Number(left.fixed) - Number(right.fixed));
      for (const entity of ordered) {
        drawWireBox(context, entity, matrix, width / ratio, height / ratio, dark);
      }
    },
  };
}

function resizeCanvas(canvas) {
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.floor(canvas.clientWidth * ratio));
  const height = Math.max(1, Math.floor(canvas.clientHeight * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return { width, height, ratio };
}

function bodyColor(entity) {
  if (entity.fixed) {
    return [0.42, 0.44, 0.47, 1];
  }
  if (entity.id === 0) return [0.28, 0.62, 0.88, 1];
  if (entity.id === 1) return [0.87, 0.58, 0.24, 1];
  return [0.54, 0.72, 0.39, 1];
}

function drawWireBox(context, entity, matrix, width, height, dark) {
  const corners = [
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
  ].map(([x, y, z]) => projectPoint(
    [entity.x + x * entity.halfX, entity.y + y * entity.halfY, entity.z + z * entity.halfZ],
    matrix,
    width,
    height,
  ));
  const edges = [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];
  context.beginPath();
  for (const [left, right] of edges) {
    const a = corners[left];
    const b = corners[right];
    if (!a || !b) continue;
    context.moveTo(a[0], a[1]);
    context.lineTo(b[0], b[1]);
  }
  context.lineWidth = entity.fixed ? 1 : 2;
  context.strokeStyle = entity.fixed
    ? (dark ? "rgba(230,230,230,.28)" : "rgba(20,20,20,.28)")
    : (dark ? "#f2f2f2" : "#161616");
  context.stroke();

  if (!entity.fixed) {
    const center = projectPoint([entity.x, entity.y, entity.z], matrix, width, height);
    if (center) {
      context.fillStyle = dark ? "#f5f5f5" : "#111";
      context.font = "600 13px system-ui, sans-serif";
      context.fillText(bodyLabel(entity), center[0] + 5, center[1] - 5);
    }
  }
}

function projectPoint(point, matrix, width, height) {
  const [x, y, z] = point;
  const clipX = matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12];
  const clipY = matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13];
  const clipW = matrix[3] * x + matrix[7] * y + matrix[11] * z + matrix[15];
  if (clipW <= 0.0001) {
    return null;
  }
  const ndcX = clipX / clipW;
  const ndcY = clipY / clipW;
  return [(ndcX * 0.5 + 0.5) * width, (0.5 - ndcY * 0.5) * height];
}

function cameraViewProjection(camera, aspect) {
  const yaw = camera.yaw * (Math.PI / 180);
  const pitch = camera.pitch * (Math.PI / 180);
  const horizontal = camera.radius * Math.cos(pitch);
  const eye = [
    camera.target[0] + horizontal * Math.sin(yaw),
    camera.target[1] + camera.radius * Math.sin(pitch),
    camera.target[2] + horizontal * Math.cos(yaw),
  ];
  const view = lookAt(eye, camera.target, [0, 1, 0]);
  const projection = perspective(46 * (Math.PI / 180), aspect, 0.1, 180);
  return multiplyMat4(projection, view);
}

function lookAt(eye, target, up) {
  const z = normalize([eye[0] - target[0], eye[1] - target[1], eye[2] - target[2]]);
  const x = normalize(cross(up, z));
  const y = cross(z, x);
  return new Float32Array([
    x[0], y[0], z[0], 0,
    x[1], y[1], z[1], 0,
    x[2], y[2], z[2], 0,
    -dot(x, eye), -dot(y, eye), -dot(z, eye), 1,
  ]);
}

function perspective(fov, aspect, near, far) {
  const f = 1 / Math.tan(fov / 2);
  return new Float32Array([
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, far / (near - far), -1,
    0, 0, (near * far) / (near - far), 0,
  ]);
}

function multiplyMat4(left, right) {
  const output = new Float32Array(16);
  for (let column = 0; column < 4; column += 1) {
    for (let row = 0; row < 4; row += 1) {
      let value = 0;
      for (let index = 0; index < 4; index += 1) {
        value += left[index * 4 + row] * right[column * 4 + index];
      }
      output[column * 4 + row] = value;
    }
  }
  return output;
}

function normalize(vector) {
  const length = Math.hypot(...vector) || 1;
  return vector.map((value) => value / length);
}

function cross(left, right) {
  return [
    left[1] * right[2] - left[2] * right[1],
    left[2] * right[0] - left[0] * right[2],
    left[0] * right[1] - left[1] * right[0],
  ];
}

function dot(left, right) {
  return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

async function createRenderer() {
  try {
    const gpuRenderer = await createWebGpuRenderer(webgpuCanvas);
    if (gpuRenderer) {
      fallbackCanvas.hidden = true;
      webgpuCanvas.hidden = false;
      rendererStatus.textContent = "WebGPU 3D renderer";
      return gpuRenderer;
    }
  } catch {
    // Fall through to the deterministic visual fallback.
  }
  webgpuCanvas.hidden = true;
  fallbackCanvas.hidden = false;
  rendererStatus.textContent = "Canvas 3D fallback";
  return createFallbackRenderer(fallbackCanvas);
}

async function main() {
  bindControls();
  try {
    wasmExports = await loadWasm();
    const maxSteps = wasmExports.physics_demo_max_steps();
    if (!Number.isInteger(maxSteps) || maxSteps <= 0) {
      throw new Error("Rust returned an invalid 3D physics frame horizon");
    }
    frameInput.max = String(maxSteps);
    frames = Array.from({ length: maxSteps + 1 }, (_, step) => readRustFrame(step));
    renderer = await createRenderer();

    runtimeStatus.textContent =
      "Rust/Wasm ready. ecs-physics-3d owns X/Y/Z integration, six-sided room response, mass, restitution, two-axis friction, and exact 3D pair evidence.";
    if (!("gpu" in navigator)) {
      webgpuEnabled.disabled = true;
      webgpuStatus.textContent = "WebGPU is unavailable; Rust physics and the Canvas 3D fallback remain usable.";
    } else {
      webgpuStatus.textContent = "WebGPU is available for 3D rendering and optional exact pair verification.";
      if (new URLSearchParams(window.location.search).get("webgpu") === "1") {
        webgpuEnabled.checked = true;
      }
    }
    setExactStep(0, { verifyGpu: webgpuEnabled.checked });
  } catch (error) {
    runtimeStatus.textContent = `Could not start the 3D Rust physics demo: ${error instanceof Error ? error.message : String(error)}`;
    collisionStatus.textContent = "The 3D physics demo is unavailable until the Rust/Wasm module loads.";
    webgpuStatus.textContent = "WebGPU verification was not started.";
    for (const control of [frameInput, resetButton, stepButton, runButton, yawInput, pitchInput, resetCameraButton, webgpuEnabled]) {
      control.disabled = true;
    }
  }
}

void main();