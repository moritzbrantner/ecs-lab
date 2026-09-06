const MAX_BODIES = 64;
const VALID_STRIDES = new Set([1, 2, 4, 8]);
const MATRIX_CELL_WIDTH = 14;
const MATRIX_CELL_HEIGHT = 8;
const MATRIX_LEFT = 54;
const MATRIX_TOP = 30;
const MATRIX_BOTTOM = 18;

const frameInput = document.querySelector("#frame");
const stepForwardButton = document.querySelector("#step-frame");
const stepBackButton = document.querySelector("#step-back");
const strideSelect = document.querySelector("#step-stride");
const matrixEnabled = document.querySelector("#time-matrix-enabled");
const matrixPanel = document.querySelector("#time-matrix-panel");
const matrixCanvas = document.querySelector("#time-matrix-canvas");
const matrixReadout = document.querySelector("#time-matrix-readout");

if (
  !(frameInput instanceof HTMLInputElement) ||
  !(stepForwardButton instanceof HTMLButtonElement) ||
  !(stepBackButton instanceof HTMLButtonElement) ||
  !(strideSelect instanceof HTMLSelectElement) ||
  !(matrixEnabled instanceof HTMLInputElement) ||
  !(matrixPanel instanceof HTMLElement) ||
  !(matrixCanvas instanceof HTMLCanvasElement) ||
  !(matrixReadout instanceof HTMLElement)
) {
  throw new Error("The temporal physics controls are incomplete.");
}

let matrixFrames = null;
let matrixLoad = null;
let matrixGeometry = null;
let selectedMatrixCell = null;

function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function currentStep() {
  return Number.parseInt(frameInput.value, 10);
}

function maxStep() {
  return Number.parseInt(frameInput.max, 10);
}

function stepStride() {
  const parsed = Number.parseInt(strideSelect.value, 10);
  return VALID_STRIDES.has(parsed) ? parsed : 1;
}

function setFrame(step) {
  const bounded = clamp(step, 0, maxStep());
  frameInput.value = String(bounded);
  frameInput.dispatchEvent(new Event("input", { bubbles: true }));
  frameInput.dispatchEvent(new Event("change", { bubbles: true }));
  if (matrixEnabled.checked) {
    renderTimeMatrix();
  }
}

function updateStepLabels() {
  const stride = stepStride();
  stepForwardButton.textContent = `Step +${stride}`;
  stepBackButton.textContent = `Step −${stride}`;
}

function captureForwardStep(event) {
  const stride = stepStride();
  if (stride === 1) {
    return;
  }
  event.preventDefault();
  event.stopImmediatePropagation();
  setFrame(currentStep() + stride);
}

async function loadMatrixWasm() {
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
    "physics_demo_is_fixed",
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

function pairContactIndexes(pairWords, bodyCount) {
  const contacts = new Set();
  let pair = 0;
  for (let left = 0; left < bodyCount; left += 1) {
    for (let right = left + 1; right < bodyCount; right += 1) {
      const word = pairWords[Math.floor(pair / 32)] ?? 0;
      const mask = (1 << (pair % 32)) >>> 0;
      if ((word & mask) !== 0) {
        contacts.add(left);
        contacts.add(right);
      }
      pair += 1;
    }
  }
  return contacts;
}

function readMatrixFrame(wasmExports, step) {
  const bodyCount = wasmExports.physics_demo_body_count(step);
  if (!Number.isInteger(bodyCount) || bodyCount <= 0 || bodyCount > MAX_BODIES) {
    throw new Error(`Rust returned an invalid body count at step ${step}: ${bodyCount}`);
  }

  const bodies = [];
  for (let bodyIndex = 0; bodyIndex < bodyCount; bodyIndex += 1) {
    if (wasmExports.physics_demo_is_fixed(bodyIndex, step) === 1) {
      continue;
    }
    bodies.push({
      bodyIndex,
      id: wasmExports.physics_demo_entity_id(bodyIndex, step) >>> 0,
      x: wasmExports.physics_demo_position_x(bodyIndex, step),
      y: wasmExports.physics_demo_position_y(bodyIndex, step),
      z: wasmExports.physics_demo_position_z(bodyIndex, step),
    });
  }

  const wordCount = wasmExports.physics_demo_pair_word_count(step);
  const pairWords = new Uint32Array(wordCount);
  for (let wordIndex = 0; wordIndex < wordCount; wordIndex += 1) {
    pairWords[wordIndex] = wasmExports.physics_demo_pair_word(wordIndex, step) >>> 0;
  }

  return {
    step,
    bodies,
    contacts: pairContactIndexes(pairWords, bodyCount),
  };
}

async function ensureMatrixFrames() {
  if (matrixFrames) {
    return matrixFrames;
  }
  if (matrixLoad) {
    return matrixLoad;
  }

  matrixReadout.textContent = "Loading the authoritative Rust timeline for the time matrix…";
  matrixLoad = (async () => {
    const wasmExports = await loadMatrixWasm();
    const maxSteps = wasmExports.physics_demo_max_steps();
    if (!Number.isInteger(maxSteps) || maxSteps < 1) {
      throw new Error("Rust returned an invalid physics timeline length.");
    }
    const loaded = Array.from(
      { length: maxSteps + 1 },
      (_, step) => readMatrixFrame(wasmExports, step),
    );
    const bodyIds = loaded[0].bodies.map((body) => body.id);
    for (const frame of loaded) {
      const ids = frame.bodies.map((body) => body.id);
      if (ids.length !== bodyIds.length || ids.some((id, index) => id !== bodyIds[index])) {
        throw new Error("Dynamic body ordering changed across the Rust timeline.");
      }
    }
    matrixFrames = loaded;
    return loaded;
  })();

  try {
    return await matrixLoad;
  } finally {
    matrixLoad = null;
  }
}

function sampledSteps(frameCount, stride) {
  const steps = [];
  for (let step = 0; step < frameCount; step += stride) {
    steps.push(step);
  }
  const finalStep = frameCount - 1;
  if (steps.at(-1) !== finalStep) {
    steps.push(finalStep);
  }
  return steps;
}

function matrixColors() {
  const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
  return dark
    ? {
        background: "#101216",
        grid: "rgba(255,255,255,.08)",
        text: "rgba(245,245,245,.84)",
        muted: "rgba(245,245,245,.48)",
        current: "#9dc7f2",
        contact: "#ffffff",
      }
    : {
        background: "#f7f7f5",
        grid: "rgba(0,0,0,.08)",
        text: "rgba(15,15,15,.84)",
        muted: "rgba(15,15,15,.48)",
        current: "#315f8e",
        contact: "#111111",
      };
}

function heightColor(y) {
  const amount = clamp((y + 1) / 22, 0, 1);
  const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
  if (dark) {
    const red = Math.round(36 + amount * 88);
    const green = Math.round(58 + amount * 118);
    const blue = Math.round(82 + amount * 145);
    return `rgb(${red} ${green} ${blue})`;
  }
  const red = Math.round(222 - amount * 115);
  const green = Math.round(232 - amount * 105);
  const blue = Math.round(242 - amount * 72);
  return `rgb(${red} ${green} ${blue})`;
}

function buildMatrixGeometry() {
  if (!matrixFrames) {
    return null;
  }
  const steps = sampledSteps(matrixFrames.length, stepStride());
  const bodyCount = matrixFrames[0].bodies.length;
  return {
    steps,
    bodyCount,
    width: MATRIX_LEFT + bodyCount * MATRIX_CELL_WIDTH + 10,
    height: MATRIX_TOP + steps.length * MATRIX_CELL_HEIGHT + MATRIX_BOTTOM,
  };
}

function renderTimeMatrix() {
  if (!matrixEnabled.checked || !matrixFrames) {
    return;
  }
  const context = matrixCanvas.getContext("2d");
  if (!context) {
    matrixReadout.textContent = "This browser cannot draw the time matrix canvas.";
    return;
  }

  const geometry = buildMatrixGeometry();
  if (!geometry) {
    return;
  }
  matrixGeometry = geometry;
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  matrixCanvas.width = Math.ceil(geometry.width * ratio);
  matrixCanvas.height = Math.ceil(geometry.height * ratio);
  matrixCanvas.style.width = `${geometry.width}px`;
  matrixCanvas.style.height = `${geometry.height}px`;
  context.setTransform(ratio, 0, 0, ratio, 0, 0);

  const colors = matrixColors();
  context.fillStyle = colors.background;
  context.fillRect(0, 0, geometry.width, geometry.height);
  context.font = "11px system-ui, sans-serif";
  context.textBaseline = "middle";

  const selectedStep = currentStep();
  geometry.steps.forEach((step, rowIndex) => {
    const frame = matrixFrames[step];
    const y = MATRIX_TOP + rowIndex * MATRIX_CELL_HEIGHT;
    const isCurrent = step === selectedStep;

    if (isCurrent) {
      context.fillStyle = colors.current;
      context.globalAlpha = 0.16;
      context.fillRect(MATRIX_LEFT - 4, y, geometry.bodyCount * MATRIX_CELL_WIDTH + 8, MATRIX_CELL_HEIGHT);
      context.globalAlpha = 1;
    }

    if (rowIndex % 4 === 0 || rowIndex === geometry.steps.length - 1 || isCurrent) {
      context.fillStyle = isCurrent ? colors.current : colors.muted;
      context.textAlign = "right";
      context.fillText(String(step), MATRIX_LEFT - 8, y + MATRIX_CELL_HEIGHT / 2);
    }

    frame.bodies.forEach((body, columnIndex) => {
      const x = MATRIX_LEFT + columnIndex * MATRIX_CELL_WIDTH;
      context.fillStyle = heightColor(body.y);
      context.fillRect(x + 1, y + 1, MATRIX_CELL_WIDTH - 2, MATRIX_CELL_HEIGHT - 2);
      if (frame.contacts.has(body.bodyIndex)) {
        context.fillStyle = colors.contact;
        context.fillRect(x + MATRIX_CELL_WIDTH - 4, y + 2, 2, 2);
      }
    });
  });

  context.strokeStyle = colors.grid;
  context.lineWidth = 1;
  for (let column = 0; column <= geometry.bodyCount; column += 4) {
    const x = MATRIX_LEFT + column * MATRIX_CELL_WIDTH + 0.5;
    context.beginPath();
    context.moveTo(x, MATRIX_TOP);
    context.lineTo(x, geometry.height - MATRIX_BOTTOM);
    context.stroke();
  }

  context.fillStyle = colors.text;
  context.textAlign = "center";
  for (let column = 0; column < geometry.bodyCount; column += 4) {
    const body = matrixFrames[0].bodies[column];
    const x = MATRIX_LEFT + column * MATRIX_CELL_WIDTH + MATRIX_CELL_WIDTH / 2;
    context.fillText(String(body.id + 1), x, 14);
  }
  context.save();
  context.translate(13, MATRIX_TOP + (geometry.steps.length * MATRIX_CELL_HEIGHT) / 2);
  context.rotate(-Math.PI / 2);
  context.fillText("time / Rust step", 0, 0);
  context.restore();
  context.fillText("dynamic body", MATRIX_LEFT + (geometry.bodyCount * MATRIX_CELL_WIDTH) / 2, geometry.height - 7);

  if (!selectedMatrixCell) {
    matrixReadout.textContent =
      `Time matrix: ${geometry.steps.length} sampled Rust frames × ${geometry.bodyCount} dynamic bodies. Cell brightness encodes Y height; the corner dot marks a contact.`;
  }
}

function matrixHit(event) {
  if (!matrixGeometry || !matrixFrames) {
    return null;
  }
  const rect = matrixCanvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  if (x < MATRIX_LEFT || y < MATRIX_TOP) {
    return null;
  }
  const column = Math.floor((x - MATRIX_LEFT) / MATRIX_CELL_WIDTH);
  const row = Math.floor((y - MATRIX_TOP) / MATRIX_CELL_HEIGHT);
  if (
    column < 0 ||
    column >= matrixGeometry.bodyCount ||
    row < 0 ||
    row >= matrixGeometry.steps.length
  ) {
    return null;
  }
  const step = matrixGeometry.steps[row];
  const frame = matrixFrames[step];
  const body = frame.bodies[column];
  return {
    row,
    column,
    step,
    body,
    contact: frame.contacts.has(body.bodyIndex),
  };
}

function updateMatrixReadout(hit) {
  selectedMatrixCell = hit;
  if (!hit) {
    renderTimeMatrix();
    return;
  }
  const { body, step, contact } = hit;
  matrixReadout.textContent =
    `Step ${step} · Body ${body.id + 1} · position (${body.x}, ${body.y}, ${body.z}) · ${contact ? "in contact" : "no contact"}. Click to jump the 3D scene to this time.`;
}

function bindTemporalControls() {
  updateStepLabels();
  stepForwardButton.addEventListener("click", captureForwardStep, true);
  stepBackButton.addEventListener("click", () => setFrame(currentStep() - stepStride()));
  strideSelect.addEventListener("change", () => {
    updateStepLabels();
    selectedMatrixCell = null;
    renderTimeMatrix();
  });

  matrixEnabled.addEventListener("change", async () => {
    matrixPanel.hidden = !matrixEnabled.checked;
    selectedMatrixCell = null;
    if (!matrixEnabled.checked) {
      return;
    }
    try {
      await ensureMatrixFrames();
      renderTimeMatrix();
    } catch (error) {
      matrixReadout.textContent =
        `Could not build the time matrix: ${error instanceof Error ? error.message : String(error)}`;
    }
  });

  frameInput.addEventListener("input", () => {
    selectedMatrixCell = null;
    renderTimeMatrix();
  });

  matrixCanvas.addEventListener("pointermove", (event) => updateMatrixReadout(matrixHit(event)));
  matrixCanvas.addEventListener("pointerleave", () => updateMatrixReadout(null));
  matrixCanvas.addEventListener("click", (event) => {
    const hit = matrixHit(event);
    if (!hit) {
      return;
    }
    setFrame(hit.step);
    updateMatrixReadout(hit);
  });
  matrixCanvas.addEventListener("keydown", (event) => {
    if (event.key === "ArrowUp") {
      event.preventDefault();
      setFrame(currentStep() - stepStride());
    } else if (event.key === "ArrowDown") {
      event.preventDefault();
      setFrame(currentStep() + stepStride());
    }
  });
  window.addEventListener("resize", renderTimeMatrix);
}

bindTemporalControls();
