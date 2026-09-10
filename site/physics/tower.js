import initTowerRenderer, { create_tower_renderer } from "../pkg/tower_wgpu_renderer.js";

const originalCanvas = document.querySelector("#tower-canvas");
if (!(originalCanvas instanceof HTMLCanvasElement)) {
  throw new Error("The trebuchet tower canvas is missing.");
}

const stage = document.createElement("div");
stage.id = "tower-stage";
stage.className = "physics-stage tower-stage";
stage.tabIndex = 0;
stage.setAttribute("aria-label", "Interactive 3D tower destruction scene. Drag to orbit, Shift-drag or right-drag to pan, and use the mouse wheel to zoom.");

const webgpuCanvas = document.createElement("canvas");
webgpuCanvas.id = "tower-wgpu-canvas";
webgpuCanvas.width = originalCanvas.width;
webgpuCanvas.height = originalCanvas.height;
webgpuCanvas.hidden = true;
webgpuCanvas.style.display = "none";

const fallbackCanvas = originalCanvas;
fallbackCanvas.id = "tower-fallback-canvas";
fallbackCanvas.classList.remove("dice-stage");

const cameraHud = document.createElement("div");
cameraHud.className = "camera-hud";
cameraHud.innerHTML = "<span>Drag · orbit</span><span>Shift/right drag · pan</span><span>Wheel · zoom</span>";

const resetCameraButton = document.createElement("button");
resetCameraButton.id = "tower-reset-camera";
resetCameraButton.className = "camera-reset";
resetCameraButton.type = "button";
resetCameraButton.textContent = "Reset camera";

const rendererStatus = document.createElement("p");
rendererStatus.id = "tower-renderer-status";
rendererStatus.className = "renderer-status";
rendererStatus.textContent = "Starting renderer…";

fallbackCanvas.replaceWith(stage);
stage.append(webgpuCanvas, fallbackCanvas, cameraHud, resetCameraButton, rendererStatus);

const frameInput = document.querySelector("#tower-frame");
const frameLabel = document.querySelector("#tower-frame-label");
const resetButton = document.querySelector("#tower-reset");
const stepButton = document.querySelector("#tower-step");
const runButton = document.querySelector("#tower-run");
const status = document.querySelector("#tower-status");

if (
  !(stage instanceof HTMLElement) ||
  !(webgpuCanvas instanceof HTMLCanvasElement) ||
  !(fallbackCanvas instanceof HTMLCanvasElement) ||
  !(resetCameraButton instanceof HTMLButtonElement) ||
  !(rendererStatus instanceof HTMLElement) ||
  !(frameInput instanceof HTMLInputElement) ||
  !(frameLabel instanceof HTMLElement) ||
  !(resetButton instanceof HTMLButtonElement) ||
  !(stepButton instanceof HTMLButtonElement) ||
  !(runButton instanceof HTMLButtonElement) ||
  !(status instanceof HTMLElement)
) {
  throw new Error("The trebuchet tower demo markup is incomplete.");
}

const EDGES = Object.freeze([
  [0, 1], [0, 2], [0, 4],
  [1, 3], [1, 5],
  [2, 3], [2, 6],
  [3, 7],
  [4, 5], [4, 6],
  [5, 7],
  [6, 7],
]);
const DEFAULT_CAMERA = Object.freeze({ yaw: 38, pitch: 22, radius: 49, target: [-3, 7, 0] });
const CAMERA_LIMITS = Object.freeze({ minPitch: -10, maxPitch: 78, minRadius: 18, maxRadius: 95 });
const prefersReducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

let wasm = null;
let renderer = null;
let rendererFallbackReason = "";
let maxStep = 0;
let bodyCount = 0;
let roles = [];
let running = false;
let animationHandle = 0;
let runStartedAt = 0;
let runStartFrame = 0;
let camera = cloneCamera(DEFAULT_CAMERA);
let cameraDrag = null;

function cloneCamera(value) {
  return { yaw: value.yaw, pitch: value.pitch, radius: value.radius, target: [...value.target] };
}

function currentFrame() {
  return Number.parseInt(frameInput.value, 10) || 0;
}

function clampFrame(value) {
  return Math.max(0, Math.min(maxStep, Math.trunc(value)));
}

function readFrame(step) {
  const bodies = Array.from({ length: bodyCount }, (_, body) => ({
    role: roles[body],
    vertices: Array.from({ length: 8 }, (_, vertex) => [
      wasm.physics_tower_demo_vertex_x(body, vertex, step),
      wasm.physics_tower_demo_vertex_y(body, vertex, step),
      wasm.physics_tower_demo_vertex_z(body, vertex, step),
    ]),
  }));
  return {
    step,
    bodies,
    spinningBodies: wasm.physics_tower_demo_spinning_bodies(step),
    contacts: wasm.physics_tower_demo_contacts(step),
    impulsiveContacts: wasm.physics_tower_demo_impulsive_contacts(step),
  };
}

function averagePoint(points) {
  const sum = points.reduce(
    (accumulator, point) => [
      accumulator[0] + point[0],
      accumulator[1] + point[1],
      accumulator[2] + point[2],
    ],
    [0, 0, 0],
  );
  return sum.map((value) => value / points.length);
}

function projectileRadius(body) {
  const center = averagePoint(body.vertices);
  const diagonal = Math.hypot(
    body.vertices[0][0] - center[0],
    body.vertices[0][1] - center[1],
    body.vertices[0][2] - center[2],
  );
  return diagonal / Math.sqrt(3);
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

function renderFrame(frame) {
  if (!renderer) return;
  try {
    renderer.render(frame, camera);
  } catch (error) {
    if (renderer.mode !== "rust-wasm-wgpu") throw error;
    stopRun();
    const message = error instanceof Error ? error.message : String(error);
    renderer = activateCanvasFallback(`WebGPU renderer failed: ${message}`);
    renderer.render(frame, camera);
  }
}

function renderCurrentCamera() {
  if (!wasm || !renderer) return;
  renderFrame(readFrame(currentFrame()));
}

function beginCameraDrag(event) {
  if (event.target instanceof HTMLButtonElement) return;
  if (event.button !== 0 && event.button !== 2) return;
  event.preventDefault();
  stage.focus({ preventScroll: true });
  stage.setPointerCapture(event.pointerId);
  cameraDrag = {
    pointerId: event.pointerId,
    x: event.clientX,
    y: event.clientY,
    mode: event.button === 2 || event.shiftKey ? "pan" : "orbit",
  };
  stage.classList.add("camera-dragging");
}

function moveCameraDrag(event) {
  if (!cameraDrag || cameraDrag.pointerId !== event.pointerId) return;
  event.preventDefault();
  const dx = event.clientX - cameraDrag.x;
  const dy = event.clientY - cameraDrag.y;
  cameraDrag.x = event.clientX;
  cameraDrag.y = event.clientY;
  if (cameraDrag.mode === "orbit") {
    camera.yaw += dx * 0.35;
    camera.pitch = Math.max(
      CAMERA_LIMITS.minPitch,
      Math.min(CAMERA_LIMITS.maxPitch, camera.pitch + dy * 0.28),
    );
  } else {
    panCamera(dx, dy);
  }
  renderCurrentCamera();
}

function endCameraDrag(event) {
  if (!cameraDrag || cameraDrag.pointerId !== event.pointerId) return;
  if (stage.hasPointerCapture(event.pointerId)) stage.releasePointerCapture(event.pointerId);
  cameraDrag = null;
  stage.classList.remove("camera-dragging");
}

function zoomCamera(delta) {
  camera.radius = Math.max(
    CAMERA_LIMITS.minRadius,
    Math.min(CAMERA_LIMITS.maxRadius, camera.radius * Math.exp(delta * 0.0012)),
  );
  renderCurrentCamera();
}

function panCamera(dx, dy) {
  const yaw = camera.yaw * Math.PI / 180;
  const right = [Math.cos(yaw), 0, -Math.sin(yaw)];
  const scale = camera.radius * 0.0019;
  camera.target[0] -= right[0] * dx * scale;
  camera.target[2] -= right[2] * dx * scale;
  camera.target[1] += dy * scale;
}

function resetCamera() {
  camera = cloneCamera(DEFAULT_CAMERA);
  renderCurrentCamera();
}

function bindCameraControls() {
  stage.addEventListener("pointerdown", beginCameraDrag);
  stage.addEventListener("pointermove", moveCameraDrag);
  stage.addEventListener("pointerup", endCameraDrag);
  stage.addEventListener("pointercancel", endCameraDrag);
  stage.addEventListener("contextmenu", (event) => event.preventDefault());
  stage.addEventListener("wheel", (event) => {
    event.preventDefault();
    zoomCamera(event.deltaY);
  }, { passive: false });
  stage.addEventListener("dblclick", resetCamera);
  stage.addEventListener("keydown", (event) => {
    const orbitStep = event.shiftKey ? 8 : 3;
    if (event.key === "ArrowLeft") camera.yaw -= orbitStep;
    else if (event.key === "ArrowRight") camera.yaw += orbitStep;
    else if (event.key === "ArrowUp") {
      camera.pitch = Math.max(CAMERA_LIMITS.minPitch, camera.pitch - orbitStep);
    } else if (event.key === "ArrowDown") {
      camera.pitch = Math.min(CAMERA_LIMITS.maxPitch, camera.pitch + orbitStep);
    } else if (event.key === "+" || event.key === "=") {
      zoomCamera(-90);
      event.preventDefault();
      return;
    } else if (event.key === "-" || event.key === "_") {
      zoomCamera(90);
      event.preventDefault();
      return;
    } else if (event.key.toLowerCase() === "r") {
      resetCamera();
      event.preventDefault();
      return;
    } else {
      return;
    }
    event.preventDefault();
    renderCurrentCamera();
  });
  resetCameraButton.addEventListener("click", resetCamera);
}

function flattenFrameVertices(frame) {
  const values = new Float32Array(frame.bodies.length * 8 * 3);
  let offset = 0;
  for (const body of frame.bodies) {
    for (const vertex of body.vertices) {
      values[offset] = vertex[0];
      values[offset + 1] = vertex[1];
      values[offset + 2] = vertex[2];
      offset += 3;
    }
  }
  return values;
}

function createRustWgpuRenderer(wgpuRenderer) {
  return {
    mode: "rust-wasm-wgpu",
    render(frame, cameraValue) {
      const { width, height } = resizeCanvas(webgpuCanvas);
      const started = performance.now();
      wgpuRenderer.render(
        flattenFrameVertices(frame),
        width,
        height,
        cameraValue.yaw,
        cameraValue.pitch,
        cameraValue.radius,
        cameraValue.target[0],
        cameraValue.target[1],
        cameraValue.target[2],
        window.matchMedia("(prefers-color-scheme: dark)").matches,
      );
      const elapsed = performance.now() - started;
      rendererStatus.textContent = `Rust/Wasm wgpu · CPU submit ${elapsed.toFixed(2)} ms`;
    },
  };
}

function createCanvasRenderer() {
  const context = fallbackCanvas.getContext("2d");
  if (!context) {
    throw new Error("The browser cannot create the tower Canvas fallback.");
  }
  return {
    mode: "canvas",
    render(frame, cameraValue) {
      const { width, height, ratio } = resizeCanvas(fallbackCanvas);
      const cssWidth = width / ratio;
      const cssHeight = height / ratio;
      const started = performance.now();
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
      context.fillStyle = dark ? "#0b0c0f" : "#f5f5f2";
      context.fillRect(0, 0, cssWidth, cssHeight);
      const matrix = cameraViewProjection(cameraValue, cssWidth / cssHeight);
      for (const body of frame.bodies) {
        if (body.role === 1) drawProjectileBall(context, body, matrix, cssWidth, cssHeight, dark);
        else drawWireBox(context, body, matrix, cssWidth, cssHeight, dark);
      }
      const elapsed = performance.now() - started;
      const reason = rendererFallbackReason ? ` · ${rendererFallbackReason}` : "";
      rendererStatus.textContent = `Canvas fallback · CPU draw ${elapsed.toFixed(2)} ms${reason}`;
    },
  };
}

function activateCanvasFallback(reason = "") {
  rendererFallbackReason = reason;
  webgpuCanvas.hidden = true;
  webgpuCanvas.style.display = "none";
  fallbackCanvas.hidden = false;
  fallbackCanvas.style.display = "block";
  return createCanvasRenderer();
}

function drawWireBox(context, body, matrix, width, height, dark) {
  const projected = body.vertices.map((point) => projectPoint(point, matrix, width, height));
  context.beginPath();
  for (const [left, right] of EDGES) {
    const from = projected[left];
    const to = projected[right];
    if (!from || !to) continue;
    context.moveTo(from[0], from[1]);
    context.lineTo(to[0], to[1]);
  }
  context.lineWidth = body.role === 0 ? 1 : 1.45;
  context.strokeStyle = body.role === 0
    ? (dark ? "rgba(245,245,245,.34)" : "rgba(20,20,20,.34)")
    : (dark ? "rgba(245,245,245,.78)" : "rgba(20,20,20,.72)");
  context.stroke();
}

function drawProjectileBall(context, body, matrix, width, height, dark) {
  const center = averagePoint(body.vertices);
  const radius = projectileRadius(body);
  const projectedCenter = projectPoint(center, matrix, width, height);
  if (!projectedCenter) return;
  const projectedAxes = [
    [center[0] + radius, center[1], center[2]],
    [center[0], center[1] + radius, center[2]],
    [center[0], center[1], center[2] + radius],
  ].map((point) => projectPoint(point, matrix, width, height)).filter(Boolean);
  const screenRadius = Math.max(
    1,
    ...projectedAxes.map((point) => Math.hypot(point[0] - projectedCenter[0], point[1] - projectedCenter[1])),
  );
  const gradient = context.createRadialGradient(
    projectedCenter[0] - screenRadius * 0.28,
    projectedCenter[1] - screenRadius * 0.32,
    screenRadius * 0.08,
    projectedCenter[0],
    projectedCenter[1],
    screenRadius,
  );
  if (dark) {
    gradient.addColorStop(0, "#ffe1a0");
    gradient.addColorStop(0.55, "#c98320");
    gradient.addColorStop(1, "#593000");
  } else {
    gradient.addColorStop(0, "#f3c66f");
    gradient.addColorStop(0.55, "#9b5b09");
    gradient.addColorStop(1, "#3d2100");
  }
  context.beginPath();
  context.arc(projectedCenter[0], projectedCenter[1], screenRadius, 0, Math.PI * 2);
  context.fillStyle = gradient;
  context.fill();
}

function projectPoint(point, matrix, width, height) {
  const [x, y, z] = point;
  const clipX = matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12];
  const clipY = matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13];
  const clipW = matrix[3] * x + matrix[7] * y + matrix[11] * z + matrix[15];
  if (clipW <= 0.0001) return null;
  const ndcX = clipX / clipW;
  const ndcY = clipY / clipW;
  return [(ndcX * 0.5 + 0.5) * width, (0.5 - ndcY * 0.5) * height];
}

function cameraViewProjection(cameraValue, aspect) {
  const yaw = cameraValue.yaw * Math.PI / 180;
  const pitch = cameraValue.pitch * Math.PI / 180;
  const horizontal = cameraValue.radius * Math.cos(pitch);
  const eye = [
    cameraValue.target[0] + horizontal * Math.sin(yaw),
    cameraValue.target[1] + cameraValue.radius * Math.sin(pitch),
    cameraValue.target[2] + horizontal * Math.cos(yaw),
  ];
  return multiplyMat4(
    perspective(46 * Math.PI / 180, aspect, 0.1, 180),
    lookAt(eye, cameraValue.target, [0, 1, 0]),
  );
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

function projectileCenter(frame) {
  const projectile = frame.bodies.find((body) => body.role === 1);
  return projectile ? averagePoint(projectile.vertices) : [0, 0, 0];
}

function updateStatus(frame) {
  const projectile = projectileCenter(frame);
  const event = frame.impulsiveContacts > 0
    ? `${frame.impulsiveContacts} impulsive ${frame.impulsiveContacts === 1 ? "contact" : "contacts"}`
    : `${frame.contacts} resting/touching contacts`;
  status.textContent = `Rust frame ${frame.step}/60 s · ball x ${projectile[0].toFixed(1)}, y ${projectile[1].toFixed(1)} · ${frame.spinningBodies} dynamic bodies spinning · ${event}. The renderer draws the projectile as a sphere from its Rust-owned center and proxy radius; collision response is still the current Rust OBB proxy in this slice.`;
}

function setFrame(step) {
  const bounded = clampFrame(step);
  frameInput.value = String(bounded);
  frameLabel.textContent = String(bounded);
  if (!wasm || !renderer) return;
  const frame = readFrame(bounded);
  renderFrame(frame);
  updateStatus(frame);
}

function stopRun() {
  running = false;
  runButton.textContent = "Run impact";
  if (animationHandle !== 0) {
    cancelAnimationFrame(animationHandle);
    animationHandle = 0;
  }
}

function animateRun(now) {
  if (!running) return;
  const elapsedSeconds = (now - runStartedAt) / 1000;
  const next = clampFrame(runStartFrame + Math.floor(elapsedSeconds * 60));
  setFrame(next);
  if (!running || next >= maxStep) {
    stopRun();
    return;
  }
  animationHandle = requestAnimationFrame(animateRun);
}

function toggleRun() {
  if (running) {
    stopRun();
    return;
  }
  if (prefersReducedMotion) {
    status.textContent = "Automatic playback is disabled by reduced-motion preference; use Step +1 or the frame slider.";
    return;
  }
  if (currentFrame() >= maxStep) setFrame(0);
  running = true;
  runButton.textContent = "Pause";
  runStartFrame = currentFrame();
  runStartedAt = performance.now();
  animationHandle = requestAnimationFrame(animateRun);
}

async function loadPhysicsWasm() {
  const response = await fetch("../pkg/ecs_web_demo.wasm");
  if (!response.ok) {
    throw new Error(`Wasm request failed with HTTP ${response.status}`);
  }
  const { instance } = await WebAssembly.instantiate(await response.arrayBuffer(), {});
  const required = [
    "physics_tower_demo_max_steps",
    "physics_tower_demo_body_count",
    "physics_tower_demo_body_role",
    "physics_tower_demo_vertex_x",
    "physics_tower_demo_vertex_y",
    "physics_tower_demo_vertex_z",
    "physics_tower_demo_spinning_bodies",
    "physics_tower_demo_contacts",
    "physics_tower_demo_impulsive_contacts",
  ];
  for (const name of required) {
    if (typeof instance.exports[name] !== "function") {
      throw new Error(`Wasm export ${name} is missing`);
    }
  }
  return instance.exports;
}

async function createRenderer() {
  if ("gpu" in navigator) {
    try {
      resizeCanvas(webgpuCanvas);
      await initTowerRenderer();
      const wgpuRenderer = await create_tower_renderer(webgpuCanvas);
      rendererFallbackReason = "";
      fallbackCanvas.hidden = true;
      fallbackCanvas.style.display = "none";
      webgpuCanvas.hidden = false;
      webgpuCanvas.style.display = "block";
      return createRustWgpuRenderer(wgpuRenderer);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      return activateCanvasFallback(`Rust/Wasm wgpu unavailable: ${message}`);
    }
  }
  return activateCanvasFallback("WebGPU unavailable in this browser");
}

frameInput.addEventListener("input", () => {
  stopRun();
  setFrame(Number(frameInput.value));
});
resetButton.addEventListener("click", () => {
  stopRun();
  setFrame(0);
});
stepButton.addEventListener("click", () => {
  stopRun();
  setFrame(currentFrame() + 1);
});
runButton.addEventListener("click", toggleRun);
window.addEventListener("resize", renderCurrentCamera);
bindCameraControls();

try {
  wasm = await loadPhysicsWasm();
  maxStep = wasm.physics_tower_demo_max_steps();
  bodyCount = wasm.physics_tower_demo_body_count();
  roles = Array.from({ length: bodyCount }, (_, index) => wasm.physics_tower_demo_body_role(index));
  frameInput.max = String(maxStep);
  renderer = await createRenderer();
  setFrame(0);
} catch (error) {
  status.textContent = `Trebuchet tower demo unavailable: ${error instanceof Error ? error.message : String(error)}`;
  rendererStatus.textContent = "Renderer unavailable";
  for (const control of [frameInput, resetButton, stepButton, runButton, resetCameraButton]) {
    control.disabled = true;
  }
}