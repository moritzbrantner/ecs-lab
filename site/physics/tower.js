const canvas = document.querySelector("#tower-canvas");
const frameInput = document.querySelector("#tower-frame");
const frameLabel = document.querySelector("#tower-frame-label");
const resetButton = document.querySelector("#tower-reset");
const stepButton = document.querySelector("#tower-step");
const runButton = document.querySelector("#tower-run");
const status = document.querySelector("#tower-status");

if (
  !(canvas instanceof HTMLCanvasElement) ||
  !(frameInput instanceof HTMLInputElement) ||
  !(frameLabel instanceof HTMLElement) ||
  !(resetButton instanceof HTMLButtonElement) ||
  !(stepButton instanceof HTMLButtonElement) ||
  !(runButton instanceof HTMLButtonElement) ||
  !(status instanceof HTMLElement)
) {
  throw new Error("The trebuchet tower demo markup is incomplete.");
}

const context = canvas.getContext("2d");
if (!context) {
  throw new Error("The browser cannot create the trebuchet tower canvas.");
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
const CAMERA = Object.freeze({ yaw: 35 * Math.PI / 180, pitch: 18 * Math.PI / 180 });
const SCENE_GUIDES = Object.freeze([
  [-22, 0, -8], [-22, 0, 8], [12, 0, -8], [12, 0, 8],
  [-22, 12, -8], [-22, 12, 8], [12, 12, -8], [12, 12, 8],
]);
const prefersReducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

let wasm = null;
let maxStep = 0;
let bodyCount = 0;
let roles = [];
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

function projector(width, height) {
  const guidePoints = SCENE_GUIDES.map(cameraPoint);
  const minimumX = Math.min(...guidePoints.map((point) => point[0]));
  const maximumX = Math.max(...guidePoints.map((point) => point[0]));
  const minimumY = Math.min(...guidePoints.map((point) => point[1]));
  const maximumY = Math.max(...guidePoints.map((point) => point[1]));
  const spanX = Math.max(1, maximumX - minimumX);
  const spanY = Math.max(1, maximumY - minimumY);
  const scale = Math.min(width * 0.88 / spanX, height * 0.82 / spanY);
  const centerX = (minimumX + maximumX) / 2;
  const centerY = (minimumY + maximumY) / 2;
  return (point) => {
    const camera = cameraPoint(point);
    return [
      width / 2 + (camera[0] - centerX) * scale,
      height / 2 - (camera[1] - centerY) * scale,
      camera[2],
    ];
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

function floorTopFace(body) {
  const maximumY = Math.max(...body.vertices.map((vertex) => vertex[1]));
  return body.vertices
    .filter((vertex) => Math.abs(vertex[1] - maximumY) < 0.001)
    .sort((left, right) => Math.atan2(left[2], left[0]) - Math.atan2(right[2], right[0]));
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

  const project = projector(cssWidth, cssHeight);
  const floor = frame.bodies.find((body) => body.role === 0);
  if (floor) {
    const top = floorTopFace(floor).map(project);
    drawPath(top, true);
    context.fillStyle = dark ? "rgba(255,255,255,.035)" : "rgba(0,0,0,.035)";
    context.fill();
    context.strokeStyle = dark ? "rgba(245,245,245,.25)" : "rgba(20,20,20,.25)";
    context.lineWidth = 1;
    context.stroke();
  }

  const visibleBodies = frame.bodies
    .filter((body) => body.role !== 0)
    .map((body) => ({
      ...body,
      projected: body.vertices.map(project),
      depth: cameraPoint(averagePoint(body.vertices))[2],
    }))
    .sort((left, right) => left.depth - right.depth);

  for (const body of visibleBodies) {
    const projectile = body.role === 1;
    context.strokeStyle = projectile
      ? (dark ? "#f5c96a" : "#7b4c00")
      : (dark ? "rgba(245,245,245,.72)" : "rgba(20,20,20,.66)");
    context.lineWidth = projectile ? 3 : 1.45;
    context.beginPath();
    for (const [left, right] of EDGES) {
      const from = body.projected[left];
      const to = body.projected[right];
      context.moveTo(from[0], from[1]);
      context.lineTo(to[0], to[1]);
    }
    context.stroke();
  }
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
  status.textContent = `Rust frame ${frame.step}/60 s · projectile x ${projectile[0].toFixed(1)}, y ${projectile[1].toFixed(1)} · ${frame.spinningBodies} dynamic bodies spinning · ${event}. Every displayed cuboid edge comes from Rust-exported collision vertices.`;
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
  maxStep = wasm.physics_tower_demo_max_steps();
  bodyCount = wasm.physics_tower_demo_body_count();
  roles = Array.from({ length: bodyCount }, (_, index) => wasm.physics_tower_demo_body_role(index));
  frameInput.max = String(maxStep);
  setFrame(0);
} catch (error) {
  status.textContent = `Trebuchet tower demo unavailable: ${error instanceof Error ? error.message : String(error)}`;
  for (const control of [frameInput, resetButton, stepButton, runButton]) {
    control.disabled = true;
  }
}
