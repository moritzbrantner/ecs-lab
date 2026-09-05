import { runWebGpuAabbPairs } from "./webgpu.js";

const WORLD = Object.freeze({ minX: -48, maxX: 48, minY: -28, maxY: 28 });
const MAX_ENTITIES = 24;
const INITIAL_ENTITIES = Object.freeze([
  { id: 1, x: -20, y: -8, vx: 3, vy: 2, half: 3 },
  { id: 2, x: 0, y: 8, vx: -2, vy: -1, half: 4 },
  { id: 3, x: 20, y: -6, vx: -3, vy: 1, half: 3 },
]);

const addEntityButton = document.querySelector("#add-entity");
const removeEntityButton = document.querySelector("#remove-entity");
const resetWorldButton = document.querySelector("#reset-world");
const stepTickButton = document.querySelector("#step-tick");
const ticksInput = document.querySelector("#ticks");
const ticksLabel = document.querySelector("#ticks-label");
const entityList = document.querySelector("#entity-list");
const world = document.querySelector("#world");
const inspectorTitle = document.querySelector("#inspector-title");
const selectedFramePosition = document.querySelector("#selected-frame-position");
const runtimeStatus = document.querySelector("#runtime-status");
const collisionStatus = document.querySelector("#collision-status");
const webgpuToggle = document.querySelector("#webgpu-enabled");
const webgpuStatus = document.querySelector("#webgpu-status");
const inspectorInputs = [...document.querySelectorAll("[data-field]")];

if (
  !(addEntityButton instanceof HTMLButtonElement) ||
  !(removeEntityButton instanceof HTMLButtonElement) ||
  !(resetWorldButton instanceof HTMLButtonElement) ||
  !(stepTickButton instanceof HTMLButtonElement) ||
  !(ticksInput instanceof HTMLInputElement) ||
  !(ticksLabel instanceof HTMLElement) ||
  !(entityList instanceof HTMLElement) ||
  !(world instanceof HTMLElement) ||
  !(inspectorTitle instanceof HTMLElement) ||
  !(selectedFramePosition instanceof HTMLElement) ||
  !(runtimeStatus instanceof HTMLElement) ||
  !(collisionStatus instanceof HTMLElement) ||
  !(webgpuToggle instanceof HTMLInputElement) ||
  !(webgpuStatus instanceof HTMLElement) ||
  inspectorInputs.some((input) => !(input instanceof HTMLInputElement))
) {
  throw new Error("ECS Lab demo markup is incomplete.");
}

let entities = cloneInitialEntities();
let selectedId = entities[0].id;
let nextId = Math.max(...entities.map((entity) => entity.id)) + 1;
let wasmExports;
let revision = 0;
let gpuRunToken = 0;

async function instantiateWasm(url) {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Unable to load WebAssembly (${response.status}).`);
  }

  if (WebAssembly.instantiateStreaming) {
    try {
      return await WebAssembly.instantiateStreaming(response.clone(), {});
    } catch {
      // Fall back when a static host does not attach the WebAssembly MIME type.
    }
  }

  return WebAssembly.instantiate(await response.arrayBuffer(), {});
}

function cloneInitialEntities() {
  return INITIAL_ENTITIES.map((entity) => ({ ...entity }));
}

function requiredWasmFunctions(exports, names) {
  return names.every((name) => typeof exports[name] === "function");
}

function selectedEntity() {
  return entities.find((entity) => entity.id === selectedId) ?? entities[0];
}

function ticks() {
  return Number(ticksInput.value);
}

function syncRustWorld() {
  if (Number(wasmExports.interactive_clear()) !== 1) {
    throw new Error("Rust could not reset the interactive ECS world.");
  }

  for (const entity of entities) {
    const accepted = Number(
      wasmExports.interactive_push_entity(
        entity.id,
        entity.x,
        entity.y,
        entity.vx,
        entity.vy,
        entity.half,
      ),
    );
    if (accepted !== 1) {
      throw new Error(`Rust rejected Entity ${entity.id} from the interactive world.`);
    }
  }

  if (Number(wasmExports.interactive_entity_count()) !== entities.length) {
    throw new Error("Rust and the browser disagree about the interactive entity set.");
  }
}

function evaluateFrame() {
  if (!wasmExports) {
    return [];
  }

  syncRustWorld();
  const frameTicks = ticks();
  return entities.map((entity, index) => ({
    ...entity,
    frameX: Number(wasmExports.interactive_position_x(index, frameTicks)),
    frameY: Number(wasmExports.interactive_position_y(index, frameTicks)),
  }));
}

function pairIndex(left, right, count) {
  return (left * (2 * count - left - 1)) / 2 + (right - left - 1);
}

function rustCollisionEvidence(frame) {
  const frameTicks = ticks();
  const wordCount = Number(wasmExports.interactive_pair_word_count(frameTicks));
  const pairWords = new Uint32Array(wordCount);
  for (let index = 0; index < wordCount; index += 1) {
    pairWords[index] = Number(wasmExports.interactive_pair_word(index, frameTicks)) >>> 0;
  }

  const contacts = new Set();
  const contactPairs = [];
  for (let left = 0; left < frame.length; left += 1) {
    for (let right = left + 1; right < frame.length; right += 1) {
      const index = pairIndex(left, right, frame.length);
      const word = pairWords[Math.floor(index / 32)] ?? 0;
      const overlaps = (word & (1 << (index % 32))) !== 0;
      if (!overlaps) {
        continue;
      }
      contacts.add(frame[left].id);
      contacts.add(frame[right].id);
      contactPairs.push([frame[left].id, frame[right].id]);
    }
  }

  return { pairWords, contacts, contactPairs };
}

function packedAabbs(frame) {
  const values = new Float32Array(frame.length * 8);
  frame.forEach((entity, index) => {
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

function renderEntityList() {
  entityList.replaceChildren();
  for (const entity of entities) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "entity-row";
    button.dataset.selected = String(entity.id === selectedId);
    button.innerHTML = `
      <span class="entity-row-title">Entity ${entity.id}</span>
      <span class="entity-row-components">Position · Velocity · Collider</span>
    `;
    button.addEventListener("click", () => {
      selectedId = entity.id;
      renderInspector();
      renderEntityList();
      renderWorldOnly();
    });
    entityList.append(button);
  }

  addEntityButton.disabled = entities.length >= MAX_ENTITIES;
}

function renderInspector() {
  const entity = selectedEntity();
  if (!entity) {
    return;
  }

  inspectorTitle.textContent = `Entity ${entity.id}`;
  for (const input of inspectorInputs) {
    const field = input.dataset.field;
    if (field && field in entity) {
      input.value = String(entity[field]);
    }
  }
  removeEntityButton.disabled = entities.length <= 1;
}

function percent(value, min, max) {
  return ((value - min) / (max - min)) * 100;
}

function renderWorld(frame, contacts) {
  for (const existing of [...world.querySelectorAll(".entity")]) {
    existing.remove();
  }

  for (const entity of frame) {
    const node = document.createElement("button");
    node.type = "button";
    node.className = "entity";
    node.dataset.selected = String(entity.id === selectedId);
    node.dataset.contact = String(contacts.has(entity.id));
    node.setAttribute(
      "aria-label",
      `Entity ${entity.id} at ${entity.frameX}, ${entity.frameY}; select for component editing`,
    );
    node.style.left = `${percent(entity.frameX, WORLD.minX, WORLD.maxX)}%`;
    node.style.bottom = `${percent(entity.frameY, WORLD.minY, WORLD.maxY)}%`;
    node.style.width = `${(entity.half * 2 * 100) / (WORLD.maxX - WORLD.minX)}%`;
    node.style.height = `${(entity.half * 2 * 100) / (WORLD.maxY - WORLD.minY)}%`;

    const speed = Math.hypot(entity.vx, entity.vy);
    const angle = Math.atan2(-entity.vy, entity.vx) * (180 / Math.PI);
    node.innerHTML = `<span>${entity.id}</span><i class="velocity-vector" aria-hidden="true"></i>`;
    const vector = node.querySelector(".velocity-vector");
    if (vector instanceof HTMLElement) {
      vector.style.width = `${Math.max(0, speed * 0.85)}rem`;
      vector.style.transform = `translateY(-50%) rotate(${angle}deg)`;
      vector.hidden = speed === 0;
    }

    node.addEventListener("click", () => {
      selectedId = entity.id;
      renderInspector();
      renderEntityList();
      renderWorldOnly();
    });
    world.append(node);
  }

  const selectedFrame = frame.find((entity) => entity.id === selectedId);
  selectedFramePosition.textContent = selectedFrame
    ? `Frame position after ${ticks()} ticks: (${selectedFrame.frameX}, ${selectedFrame.frameY})`
    : "Frame position: —";
}

function renderCollisionStatus(evidence) {
  if (evidence.contactPairs.length === 0) {
    collisionStatus.dataset.state = "ready";
    collisionStatus.textContent = "Rust broad phase: no collider pairs overlap in this frame.";
    return;
  }

  const firstPairs = evidence.contactPairs
    .slice(0, 3)
    .map(([left, right]) => `${left}↔${right}`)
    .join(", ");
  const suffix = evidence.contactPairs.length > 3 ? ", …" : "";
  collisionStatus.dataset.state = "verified";
  collisionStatus.textContent = `Rust broad phase: overlapping entities are highlighted (${firstPairs}${suffix}).`;
}

function renderWorldOnly() {
  if (!wasmExports) {
    return;
  }
  const frame = evaluateFrame();
  const evidence = rustCollisionEvidence(frame);
  renderWorld(frame, evidence.contacts);
  renderCollisionStatus(evidence);
}

function markWebGpuStale() {
  if (!webgpuToggle.checked) {
    return;
  }
  webgpuStatus.dataset.state = "ready";
  webgpuStatus.textContent = "World changed; release the control to verify the new frame with WebGPU.";
}

function render({ verifyGpu = false } = {}) {
  if (!wasmExports) {
    return;
  }

  revision += 1;
  const renderRevision = revision;
  ticksLabel.textContent = ticksInput.value;
  renderEntityList();
  renderInspector();

  const frame = evaluateFrame();
  const evidence = rustCollisionEvidence(frame);
  renderWorld(frame, evidence.contacts);
  renderCollisionStatus(evidence);

  if (webgpuToggle.checked && verifyGpu) {
    void verifyWebGpu(frame, evidence.pairWords, renderRevision);
  } else {
    markWebGpuStale();
  }
}

async function verifyWebGpu(frame, rustWords, renderRevision) {
  const token = ++gpuRunToken;
  webgpuStatus.dataset.state = "running";
  webgpuStatus.textContent = "WebGPU is recomputing this frame; timing stays hidden until parity succeeds.";

  try {
    const measurement = await runWebGpuAabbPairs(packedAabbs(frame), frame.length);
    if (token !== gpuRunToken || renderRevision !== revision || !webgpuToggle.checked) {
      return;
    }

    const exactParity =
      measurement.bitset.length === rustWords.length &&
      rustWords.every((word, index) => measurement.bitset[index] === word);
    if (!exactParity) {
      webgpuStatus.dataset.state = "error";
      webgpuStatus.textContent =
        "WebGPU pair output differs from Rust. GPU evidence is rejected for this frame.";
      return;
    }

    webgpuStatus.dataset.state = "verified";
    webgpuStatus.textContent = `WebGPU exact parity verified · ${measurement.totalMs.toFixed(2)} ms end-to-end.`;
  } catch (error) {
    if (token !== gpuRunToken || renderRevision !== revision) {
      return;
    }
    webgpuStatus.dataset.state = "error";
    webgpuStatus.textContent =
      error instanceof Error ? error.message : "The optional WebGPU verification failed.";
  }
}

function clampInteger(value, input) {
  const parsed = Number.parseInt(value, 10);
  const min = Number(input.min);
  const max = Number(input.max);
  if (!Number.isFinite(parsed)) {
    return undefined;
  }
  return Math.max(min, Math.min(max, parsed));
}

function updateSelectedFromInput(input, verifyGpu) {
  const entity = selectedEntity();
  const field = input.dataset.field;
  if (!entity || !field || !(field in entity)) {
    return;
  }

  const next = clampInteger(input.value, input);
  if (next === undefined) {
    input.value = String(entity[field]);
    return;
  }

  entity[field] = next;
  input.value = String(next);
  render({ verifyGpu });
}

function newEntity(id) {
  const slot = id - 1;
  const column = slot % 4;
  const row = Math.floor(slot / 4) % 3;
  const velocities = [-2, -1, 1, 2];
  return {
    id,
    x: -24 + column * 16,
    y: -8 + row * 8,
    vx: velocities[slot % velocities.length],
    vy: velocities[(slot + 1) % velocities.length],
    half: 2 + (slot % 3),
  };
}

addEntityButton.addEventListener("click", () => {
  if (entities.length >= MAX_ENTITIES) {
    return;
  }
  const entity = newEntity(nextId);
  nextId += 1;
  entities.push(entity);
  selectedId = entity.id;
  render({ verifyGpu: true });
});

removeEntityButton.addEventListener("click", () => {
  if (entities.length <= 1) {
    return;
  }
  const index = entities.findIndex((entity) => entity.id === selectedId);
  entities = entities.filter((entity) => entity.id !== selectedId);
  selectedId = entities[Math.min(Math.max(index, 0), entities.length - 1)].id;
  render({ verifyGpu: true });
});

resetWorldButton.addEventListener("click", () => {
  entities = cloneInitialEntities();
  selectedId = entities[0].id;
  nextId = Math.max(...entities.map((entity) => entity.id)) + 1;
  ticksInput.value = "4";
  render({ verifyGpu: true });
});

stepTickButton.addEventListener("click", () => {
  const next = Math.min(Number(ticksInput.max), ticks() + 1);
  ticksInput.value = String(next);
  render({ verifyGpu: true });
});

ticksInput.addEventListener("input", () => render());
ticksInput.addEventListener("change", () => render({ verifyGpu: true }));

for (const input of inspectorInputs) {
  input.addEventListener("input", () => updateSelectedFromInput(input, false));
  input.addEventListener("change", () => updateSelectedFromInput(input, true));
}

webgpuToggle.addEventListener("change", () => {
  gpuRunToken += 1;
  if (!webgpuToggle.checked) {
    webgpuStatus.dataset.state = "ready";
    webgpuStatus.textContent = "WebGPU verification is off; Rust remains the active reference path.";
    return;
  }
  render({ verifyGpu: true });
});

try {
  const { instance } = await instantiateWasm("./pkg/ecs_web_demo.wasm");
  wasmExports = instance.exports;
  const required = [
    "interactive_clear",
    "interactive_push_entity",
    "interactive_entity_count",
    "interactive_position_x",
    "interactive_position_y",
    "interactive_pair_word_count",
    "interactive_pair_word",
  ];
  if (!requiredWasmFunctions(wasmExports, required)) {
    throw new Error("WebAssembly module does not expose the interactive ECS demo functions.");
  }

  runtimeStatus.dataset.state = "ready";
  runtimeStatus.textContent =
    "Rust WebAssembly ready: ReferenceWorld owns the interactive frame and Rust physics owns the exact pair bitset.";

  if (!("gpu" in navigator)) {
    webgpuToggle.disabled = true;
    webgpuStatus.dataset.state = "ready";
    webgpuStatus.textContent =
      "WebGPU is unavailable in this browser; the interactive Rust path remains fully usable.";
  } else {
    webgpuStatus.dataset.state = "ready";
    webgpuStatus.textContent = "WebGPU is available. Enable it to verify the current world against Rust.";
  }

  render();
  if (new URLSearchParams(window.location.search).get("webgpu") === "1" && !webgpuToggle.disabled) {
    webgpuToggle.checked = true;
    render({ verifyGpu: true });
  }
} catch (error) {
  runtimeStatus.dataset.state = "error";
  runtimeStatus.textContent = error instanceof Error ? error.message : "Unable to start ECS WebAssembly demo.";
  collisionStatus.dataset.state = "error";
  collisionStatus.textContent = "Interactive collision evidence requires the Rust WebAssembly module.";
  webgpuToggle.disabled = true;
  webgpuStatus.dataset.state = "error";
  webgpuStatus.textContent = "WebGPU verification requires the Rust reference frame.";
}
