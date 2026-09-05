import { runWebGpuAabbPairs } from "../webgpu.js";

const WORLD = Object.freeze({ minX: -32, maxX: 32, minY: -4, maxY: 20 });
const FRAME_INTERVAL_MS = 420;
const BODY_NAMES = Object.freeze({ 0: "A", 1: "B", 2: "C" });

const stage = document.querySelector("#physics-stage");
const frameInput = document.querySelector("#frame");
const frameLabel = document.querySelector("#frame-label");
const resetButton = document.querySelector("#reset-frame");
const stepButton = document.querySelector("#step-frame");
const runButton = document.querySelector("#run-frames");
const webgpuEnabled = document.querySelector("#webgpu-enabled");
const runtimeStatus = document.querySelector("#runtime-status");
const collisionStatus = document.querySelector("#collision-status");
const webgpuStatus = document.querySelector("#webgpu-status");

let wasmExports = null;
let runTimer = null;
let renderRevision = 0;

function currentStep() {
  return Number.parseInt(frameInput.value, 10);
}

function bodyLabel(entity) {
  if (entity.fixed) {
    return "Floor";
  }
  return BODY_NAMES[entity.id] ?? `Entity ${entity.id}`;
}

function readRustFrame(step) {
  const bodyCount = wasmExports.physics_demo_body_count(step);
  if (!Number.isInteger(bodyCount) || bodyCount <= 0 || bodyCount > 64) {
    throw new Error(`Rust returned an invalid physics-demo body count: ${bodyCount}`);
  }

  const entities = Array.from({ length: bodyCount }, (_, bodyIndex) => ({
    id: wasmExports.physics_demo_entity_id(bodyIndex, step) >>> 0,
    frameX: wasmExports.physics_demo_position_x(bodyIndex, step),
    frameY: wasmExports.physics_demo_position_y(bodyIndex, step),
    halfX: wasmExports.physics_demo_half_extent_x(bodyIndex, step),
    halfY: wasmExports.physics_demo_half_extent_y(bodyIndex, step),
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

  return { entities, pairWords, step };
}

function pairContacts(entities, pairWords) {
  const bodyIndexes = new Set();
  const labels = [];
  let pair = 0;

  for (let left = 0; left < entities.length; left += 1) {
    for (let right = left + 1; right < entities.length; right += 1) {
      const word = pairWords[Math.floor(pair / 32)] ?? 0;
      const mask = (1 << (pair % 32)) >>> 0;
      if ((word & mask) !== 0) {
        bodyIndexes.add(left);
        bodyIndexes.add(right);
        labels.push(`${bodyLabel(entities[left])} ↔ ${bodyLabel(entities[right])}`);
      }
      pair += 1;
    }
  }

  return { bodyIndexes, labels };
}

function drawFrame(frame) {
  const contacts = pairContacts(frame.entities, frame.pairWords);
  stage.querySelectorAll(".physics-body").forEach((node) => node.remove());

  const spanX = WORLD.maxX - WORLD.minX;
  const spanY = WORLD.maxY - WORLD.minY;
  frame.entities.forEach((entity, bodyIndex) => {
    const body = document.createElement("div");
    const label = bodyLabel(entity);
    body.className = "physics-body";
    body.textContent = entity.fixed ? "" : label;
    body.dataset.contact = contacts.bodyIndexes.has(bodyIndex) ? "true" : "false";
    body.dataset.fixed = entity.fixed ? "true" : "false";
    body.title = entity.fixed
      ? "Fixed floor"
      : `${label}: position (${entity.frameX}, ${entity.frameY}), mass ${entity.mass}, restitution ${entity.restitution}, friction ${entity.friction}`;

    const width = ((entity.halfX * 2) / spanX) * 100;
    const height = ((entity.halfY * 2) / spanY) * 100;
    const left = ((entity.frameX - entity.halfX - WORLD.minX) / spanX) * 100;
    const bottom = ((entity.frameY - entity.halfY - WORLD.minY) / spanY) * 100;
    body.style.width = `${width}%`;
    body.style.height = `${height}%`;
    body.style.left = `${left}%`;
    body.style.bottom = `${bottom}%`;
    stage.append(body);
  });

  if (contacts.labels.length === 0) {
    collisionStatus.textContent = `Rust frame: no AABB contact at physics step ${frame.step}.`;
  } else {
    collisionStatus.textContent = `Rust frame: canonical contact ${contacts.labels.join(", ")}.`;
  }
}

function packAabbs(entities) {
  const values = new Float32Array(entities.length * 8);
  entities.forEach((entity, index) => {
    const offset = index * 8;
    values[offset] = entity.frameX - entity.halfX;
    values[offset + 1] = entity.frameY - entity.halfY;
    values[offset + 2] = -0.5;
    values[offset + 3] = 0;
    values[offset + 4] = entity.frameX + entity.halfX;
    values[offset + 5] = entity.frameY + entity.halfY;
    values[offset + 6] = 0.5;
    values[offset + 7] = 0;
  });
  return values;
}

function exactWordsMatch(left, right) {
  if (left.length !== right.length) {
    return false;
  }
  return left.every((word, index) => word === right[index]);
}

async function verifyWebGpu(frame, revision) {
  if (!("gpu" in navigator)) {
    webgpuStatus.textContent = "WebGPU is unavailable; the Rust/Wasm physics demo continues unchanged.";
    return;
  }
  if (!webgpuEnabled.checked) {
    webgpuStatus.textContent = "WebGPU verification is off. Rust remains the active physics path.";
    return;
  }

  webgpuStatus.textContent = "Running WebGPU broad-phase verification for this Rust frame…";
  try {
    const measurement = await runWebGpuAabbPairs(packAabbs(frame.entities), frame.entities.length);
    if (revision !== renderRevision) {
      return;
    }
    if (!exactWordsMatch(frame.pairWords, measurement.bitset)) {
      webgpuStatus.textContent =
        "WebGPU mismatch: GPU evidence was rejected and the Rust collision result remains authoritative.";
      return;
    }
    webgpuStatus.textContent = `WebGPU exact pair parity · ${measurement.totalMs.toFixed(2)} ms including setup and readback.`;
  } catch (error) {
    if (revision !== renderRevision) {
      return;
    }
    webgpuStatus.textContent = `WebGPU verification unavailable: ${error.message}. Rust remains authoritative.`;
  }
}

async function render({ verifyGpu = false } = {}) {
  if (!wasmExports) {
    return;
  }
  const revision = ++renderRevision;
  const step = currentStep();
  frameLabel.textContent = String(step);

  try {
    const frame = readRustFrame(step);
    drawFrame(frame);
    if (verifyGpu) {
      await verifyWebGpu(frame, revision);
    } else if (webgpuEnabled.checked) {
      webgpuStatus.textContent = "WebGPU is armed; verification runs when the current step settles.";
    } else {
      webgpuStatus.textContent = "WebGPU verification is off. Rust remains the active physics path.";
    }
  } catch (error) {
    runtimeStatus.textContent = `Physics demo error: ${error.message}`;
  }
}

function stopRun({ verifyGpu = false } = {}) {
  if (runTimer !== null) {
    window.clearInterval(runTimer);
    runTimer = null;
  }
  runButton.textContent = "Run";
  if (verifyGpu) {
    void render({ verifyGpu: true });
  }
}

function startRun() {
  if (runTimer !== null) {
    stopRun({ verifyGpu: true });
    return;
  }
  if (currentStep() >= Number(frameInput.max)) {
    frameInput.value = frameInput.min;
    void render();
  }

  runButton.textContent = "Pause";
  runTimer = window.setInterval(() => {
    const next = currentStep() + 1;
    frameInput.value = String(next);
    void render();
    if (next >= Number(frameInput.max)) {
      stopRun({ verifyGpu: true });
    }
  }, FRAME_INTERVAL_MS);
}

function bindControls() {
  frameInput.addEventListener("input", () => {
    stopRun();
    void render();
  });
  frameInput.addEventListener("change", () => void render({ verifyGpu: true }));
  resetButton.addEventListener("click", () => {
    stopRun();
    frameInput.value = frameInput.min;
    void render({ verifyGpu: true });
  });
  stepButton.addEventListener("click", () => {
    stopRun();
    frameInput.value = String(Math.min(Number(frameInput.max), currentStep() + 1));
    void render({ verifyGpu: true });
  });
  runButton.addEventListener("click", startRun);
  webgpuEnabled.addEventListener("change", () => void render({ verifyGpu: true }));
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
    "physics_demo_half_extent_x",
    "physics_demo_half_extent_y",
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

async function main() {
  bindControls();
  try {
    wasmExports = await loadWasm();
    const maxSteps = wasmExports.physics_demo_max_steps();
    if (!Number.isInteger(maxSteps) || maxSteps <= 0) {
      throw new Error("Rust returned an invalid physics-demo frame horizon");
    }
    frameInput.max = String(maxSteps);
    runtimeStatus.textContent =
      "Rust/Wasm ready. BouncingRoomScenario owns gravity, integration, fixed-body response, mass, restitution, friction, and canonical pair evidence.";

    if (!("gpu" in navigator)) {
      webgpuEnabled.disabled = true;
      webgpuStatus.textContent = "WebGPU is unavailable; the Rust/Wasm physics demo continues unchanged.";
    } else {
      webgpuStatus.textContent = "WebGPU is available. Enable verification to compare exact collision-pair evidence.";
      if (new URLSearchParams(window.location.search).get("webgpu") === "1") {
        webgpuEnabled.checked = true;
      }
    }
    await render({ verifyGpu: webgpuEnabled.checked });
  } catch (error) {
    runtimeStatus.textContent = `Could not load the Rust physics module: ${error.message}`;
    collisionStatus.textContent = "The physics demo is unavailable until the Wasm module loads.";
    webgpuStatus.textContent = "WebGPU verification was not started.";
    frameInput.disabled = true;
    resetButton.disabled = true;
    stepButton.disabled = true;
    runButton.disabled = true;
    webgpuEnabled.disabled = true;
  }
}

void main();
