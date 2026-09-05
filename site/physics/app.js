import { runWebGpuAabbPairs } from "../webgpu.js";

const WORLD = Object.freeze({ minX: -32, maxX: 32, minY: -16, maxY: 16 });
const FRAME_INTERVAL_MS = 420;

const PRESETS = Object.freeze({
  elastic: {
    copy:
      "Equal-mass, fully elastic bodies exchange momentum. The browser does not approximate the collision response; every displayed position comes back from the compiled Rust module.",
    legend: "A and B: mass 1 · restitution 1000 · friction 0",
    entities: [
      { id: 1, label: "A", x: -14, y: 0, vx: 4, vy: 0, half: 3, mass: 1, restitution: 1000, friction: 0 },
      { id: 2, label: "B", x: 14, y: 0, vx: -4, vy: 0, half: 3, mass: 1, restitution: 1000, friction: 0 },
    ],
  },
  mass: {
    copy:
      "A light fast body meets a heavier body. Rust applies the deterministic mass-weighted impulse and returns the resulting positions for each step.",
    legend: "A: mass 1 · B: mass 4 · both restitution 1000 · friction 0",
    entities: [
      { id: 1, label: "A", x: -18, y: 0, vx: 5, vy: 0, half: 3, mass: 1, restitution: 1000, friction: 0 },
      { id: 2, label: "B", x: 8, y: 0, vx: 0, vy: 0, half: 4, mass: 4, restitution: 1000, friction: 0 },
    ],
  },
  friction: {
    copy:
      "A glancing contact makes tangential velocity visible. High friction is resolved inside the Rust material solver rather than by browser animation code.",
    legend: "A: mass 1 · B: mass 3 · restitution 200 · friction 1000",
    entities: [
      { id: 1, label: "A", x: -12, y: -8, vx: 4, vy: 3, half: 3, mass: 1, restitution: 200, friction: 1000 },
      { id: 2, label: "B", x: 0, y: 0, vx: 0, vy: 0, half: 4, mass: 3, restitution: 200, friction: 1000 },
    ],
  },
  mixed: {
    copy:
      "Three bodies with deliberately different materials converge on the same area. Stable Rust ordering keeps the experiment deterministic even when several contacts compete.",
    legend: "A: bouncy · B: medium material, mass 3 · C: inelastic with maximum friction",
    entities: [
      { id: 1, label: "A", x: -18, y: 6, vx: 5, vy: -1, half: 3, mass: 1, restitution: 1000, friction: 0 },
      { id: 2, label: "B", x: 0, y: 0, vx: 0, vy: 0, half: 4, mass: 3, restitution: 500, friction: 500 },
      { id: 3, label: "C", x: 18, y: -6, vx: -5, vy: 1, half: 3, mass: 2, restitution: 0, friction: 1000 },
    ],
  },
});

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
const scenarioCopy = document.querySelector("#scenario-copy");
const scenarioLegend = document.querySelector("#scenario-legend");
const presetButtons = [...document.querySelectorAll("[data-preset]")];

let wasmExports = null;
let selectedPreset = "elastic";
let runTimer = null;
let renderRevision = 0;

function selectedScenario() {
  return PRESETS[selectedPreset];
}

function currentStep() {
  return Number.parseInt(frameInput.value, 10);
}

function syncScenario() {
  if (wasmExports.interactive_clear() !== 1) {
    throw new Error("Rust rejected the physics-world reset.");
  }

  for (const entity of selectedScenario().entities) {
    const accepted = wasmExports.interactive_push_entity(
      entity.id,
      entity.x,
      entity.y,
      entity.vx,
      entity.vy,
      entity.half,
      entity.mass,
      entity.restitution,
      entity.friction,
    );
    if (accepted !== 1) {
      throw new Error(`Rust rejected physics input for body ${entity.label}.`);
    }
  }
}

function readRustFrame(step) {
  syncScenario();
  const entities = selectedScenario().entities.map((entity, bodyIndex) => ({
    ...entity,
    frameX: wasmExports.interactive_position_x(bodyIndex, step),
    frameY: wasmExports.interactive_position_y(bodyIndex, step),
  }));
  const wordCount = wasmExports.interactive_pair_word_count(step);
  const pairWords = new Uint32Array(wordCount);
  for (let index = 0; index < wordCount; index += 1) {
    pairWords[index] = wasmExports.interactive_pair_word(index, step) >>> 0;
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
        labels.push(`${entities[left].label} ↔ ${entities[right].label}`);
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
    body.className = "physics-body";
    body.textContent = entity.label;
    body.dataset.contact = contacts.bodyIndexes.has(bodyIndex) ? "true" : "false";
    body.title = `${entity.label}: position (${entity.frameX}, ${entity.frameY}), mass ${entity.mass}, restitution ${entity.restitution}, friction ${entity.friction}`;

    const width = ((entity.half * 2) / spanX) * 100;
    const height = ((entity.half * 2) / spanY) * 100;
    const left = ((entity.frameX - entity.half - WORLD.minX) / spanX) * 100;
    const bottom = ((entity.frameY - entity.half - WORLD.minY) / spanY) * 100;
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
    values[offset] = entity.frameX - entity.half;
    values[offset + 1] = entity.frameY - entity.half;
    values[offset + 2] = -0.5;
    values[offset + 3] = 0;
    values[offset + 4] = entity.frameX + entity.half;
    values[offset + 5] = entity.frameY + entity.half;
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
    if (verifyGpu || webgpuEnabled.checked) {
      await verifyWebGpu(frame, revision);
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

function choosePreset(name) {
  stopRun();
  selectedPreset = name;
  const scenario = selectedScenario();
  scenarioCopy.textContent = scenario.copy;
  scenarioLegend.textContent = scenario.legend;
  presetButtons.forEach((button) => {
    button.classList.toggle("is-active", button.dataset.preset === name);
  });
  frameInput.value = frameInput.min;
  void render({ verifyGpu: true });
}

function bindControls() {
  presetButtons.forEach((button) => {
    button.addEventListener("click", () => choosePreset(button.dataset.preset));
  });
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
    "interactive_clear",
    "interactive_push_entity",
    "interactive_position_x",
    "interactive_position_y",
    "interactive_pair_word_count",
    "interactive_pair_word",
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
    runtimeStatus.textContent =
      "Rust/Wasm ready. Rust owns integration, collision response, mass, restitution, friction, and the canonical pair bitset.";

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
    presetButtons.forEach((button) => {
      button.disabled = true;
    });
  }
}

void main();
