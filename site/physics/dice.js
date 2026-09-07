const canvas = document.querySelector("#dice-canvas");
const frameInput = document.querySelector("#dice-frame");
const frameLabel = document.querySelector("#dice-frame-label");
const resetButton = document.querySelector("#dice-reset");
const stepButton = document.querySelector("#dice-step");
const runButton = document.querySelector("#dice-run");
const status = document.querySelector("#dice-status");

if (
  !(canvas instanceof HTMLCanvasElement) ||
  !(frameInput instanceof HTMLInputElement) ||
  !(frameLabel instanceof HTMLElement) ||
  !(resetButton instanceof HTMLButtonElement) ||
  !(stepButton instanceof HTMLButtonElement) ||
  !(runButton instanceof HTMLButtonElement) ||
  !(status instanceof HTMLElement)
) {
  throw new Error("The tumbling-dice demo markup is incomplete.");
}

const context = canvas.getContext("2d");
if (!context) {
  throw new Error("The browser cannot create the tumbling-dice canvas.");
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
const CAMERA = Object.freeze({ yaw: 34 * Math.PI / 180, pitch: 24 * Math.PI / 180 });
const prefersReducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

let wasm = null;
let maxStep = 0;
let running = false;
let animationHandle = 0;
let runStartedAt = 0;
let runStartFrame = 0;

function currentFrame() {
  return Number.parseInt(frameInput.value, 10) || 0;
}

function clampFrame(value) {
  return Math.max(0, Math.min(maxStep, Math.trunc(value)));
}

function readFrame(step) {
  const vertices = Array.from({ length: 8 }, (_, vertex) => [
    wasm.physics_dice_demo_vertex_x(vertex, step),
    wasm.physics_dice_demo_vertex_y(vertex, step),
    wasm.physics_dice_demo_vertex_z(vertex, step),
  ]);
  return {
    step,
    center: [
      wasm.physics_dice_demo_center_x(step),
      wasm.physics_dice_demo_center_y(step),
      wasm.physics_dice_demo_center_z(step),
    ],
    vertices,
    orientation: [
      wasm.physics_dice_demo_orientation_x(step),
      wasm.physics_dice_demo_orientation_y(step),
      wasm.physics_dice_demo_orientation_z(step),
      wasm.physics_dice_demo_orientation_w(step),
    ],
    angularVelocity: [
      wasm.physics_dice_demo_angular_velocity_x(step),
      wasm.physics_dice_demo_angular_velocity_y(step),
      wasm.physics_dice_demo_angular_velocity_z(step),
    ],
    contactVertices: wasm.physics_dice_demo_contact_vertices(step),
    normalImpulse: wasm.physics_dice_demo_normal_impulse(step),
  };
}

function cameraPoint([x, y, z]) {
  const cosYaw = Math.cos(CAMERA.yaw);
  const sinYaw = Math.sin(CAMERA.yaw);
  const yawX = x * cosYaw - z * sinYaw;
  const yawZ = x * sinYaw + z * cosYaw;
  const cosPitch = Math.cos(CAMERA.pitch);
  const sinPitch = Math.sin(CAMERA.pitch);
  return [yawX, y * cosPitch - yawZ * sinPitch, y * sinPitch + yawZ * cosPitch];
}

function resizeCanvas() {
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.floor(canvas.clientWidth * ratio));
  const height = Math.max(1, Math.floor(canvas.clientHeight * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return { width, height, ratio };
}

function sceneProjection(frame, width, height) {
  const points = frame.vertices.map(cameraPoint);
  const floorCorners = [
    [-7, 0, -7], [7, 0, -7], [7, 0, 7], [-7, 0, 7],
  ].map(cameraPoint);
  const all = [...points, ...floorCorners];
  const minimumX = Math.min(...all.map((point) => point[0]));
  const maximumX = Math.max(...all.map((point) => point[0]));
  const minimumY = Math.min(...all.map((point) => point[1]));
  const maximumY = Math.max(...all.map((point) => point[1]));
  const spanX = Math.max(1, maximumX - minimumX);
  const spanY = Math.max(1, maximumY - minimumY);
  const scale = Math.min(width * 0.72 / spanX, height * 0.72 / spanY);
  const centerX = (minimumX + maximumX) / 2;
  const centerY = (minimumY + maximumY) / 2;
  const project = (point) => [
    width / 2 + (point[0] - centerX) * scale,
    height / 2 - (point[1] - centerY) * scale,
  ];
  return {
    vertices: points.map(project),
    floor: floorCorners.map(project),
  };
}

function drawPath(points, close = false) {
  if (points.length === 0) return;
  context.beginPath();
  context.moveTo(points[0][0], points[0][1]);
  for (const point of points.slice(1)) {
    context.lineTo(point[0], point[1]);
  }
  if (close) context.closePath();
}

function render(frame) {
  const { width, height, ratio } = resizeCanvas();
  const cssWidth = width / ratio;
  const cssHeight = height / ratio;
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, cssWidth, cssHeight);

  const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
  context.fillStyle = dark ? "#0b0c0f" : "#f5f5f2";
  context.fillRect(0, 0, cssWidth, cssHeight);

  const projected = sceneProjection(frame, cssWidth, cssHeight);
  drawPath(projected.floor, true);
  context.fillStyle = dark ? "rgba(255,255,255,.035)" : "rgba(0,0,0,.035)";
  context.fill();
  context.strokeStyle = dark ? "rgba(245,245,245,.32)" : "rgba(20,20,20,.32)";
  context.lineWidth = 1;
  context.stroke();

  context.strokeStyle = dark ? "#f3f3f3" : "#151515";
  context.lineWidth = 2.2;
  context.beginPath();
  for (const [left, right] of EDGES) {
    const from = projected.vertices[left];
    const to = projected.vertices[right];
    context.moveTo(from[0], from[1]);
    context.lineTo(to[0], to[1]);
  }
  context.stroke();

  for (const point of projected.vertices) {
    context.beginPath();
    context.arc(point[0], point[1], 2.4, 0, Math.PI * 2);
    context.fillStyle = dark ? "#f3f3f3" : "#151515";
    context.fill();
  }
}

function updateStatus(frame) {
  const omega = frame.angularVelocity.map((value) => value.toFixed(3)).join(", ");
  const contact = frame.contactVertices > 0
    ? `${frame.contactVertices} contact ${frame.contactVertices === 1 ? "vertex" : "vertices"} · normal impulse ${frame.normalImpulse.toFixed(3)}`
    : "airborne";
  status.textContent = `Rust frame ${frame.step}/60 s · angular velocity [${omega}] rad/s · ${contact}. The wireframe vertices are exported directly from the Rust collision geometry.`;
}

function setFrame(step) {
  const bounded = clampFrame(step);
  frameInput.value = String(bounded);
  frameLabel.textContent = String(bounded);
  if (!wasm) return;
  const frame = readFrame(bounded);
  render(frame);
  updateStatus(frame);
}

function stopRun() {
  running = false;
  runButton.textContent = "Run";
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
  if (next >= maxStep) {
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

async function loadWasm() {
  const response = await fetch("../pkg/ecs_web_demo.wasm");
  if (!response.ok) {
    throw new Error(`Wasm request failed with HTTP ${response.status}`);
  }
  const { instance } = await WebAssembly.instantiate(await response.arrayBuffer(), {});
  const required = [
    "physics_dice_demo_max_steps",
    "physics_dice_demo_center_x",
    "physics_dice_demo_center_y",
    "physics_dice_demo_center_z",
    "physics_dice_demo_vertex_x",
    "physics_dice_demo_vertex_y",
    "physics_dice_demo_vertex_z",
    "physics_dice_demo_orientation_x",
    "physics_dice_demo_orientation_y",
    "physics_dice_demo_orientation_z",
    "physics_dice_demo_orientation_w",
    "physics_dice_demo_angular_velocity_x",
    "physics_dice_demo_angular_velocity_y",
    "physics_dice_demo_angular_velocity_z",
    "physics_dice_demo_contact_vertices",
    "physics_dice_demo_normal_impulse",
  ];
  for (const name of required) {
    if (typeof instance.exports[name] !== "function") {
      throw new Error(`Wasm export ${name} is missing`);
    }
  }
  return instance.exports;
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
window.addEventListener("resize", () => {
  if (wasm) setFrame(currentFrame());
});

try {
  wasm = await loadWasm();
  maxStep = wasm.physics_dice_demo_max_steps();
  frameInput.max = String(maxStep);
  setFrame(0);
} catch (error) {
  status.textContent = `Tumbling-dice demo unavailable: ${error instanceof Error ? error.message : String(error)}`;
  for (const control of [frameInput, resetButton, stepButton, runButton]) {
    control.disabled = true;
  }
}
