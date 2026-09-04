import { runWebGpuAabbPairs } from "./webgpu.js";

const ticksInput = document.querySelector("#ticks");
const ticksLabel = document.querySelector("#ticks-label");
const startPosition = document.querySelector("#start-position");
const velocity = document.querySelector("#velocity");
const resultPosition = document.querySelector("#result-position");
const runtimeStatus = document.querySelector("#runtime-status");
const entity = document.querySelector("#entity");
const webgpuToggle = document.querySelector("#webgpu-enabled");
const webgpuStatus = document.querySelector("#webgpu-status");
const webgpuBodies = document.querySelector("#webgpu-bodies");
const cpuOverlaps = document.querySelector("#cpu-overlaps");
const gpuOverlaps = document.querySelector("#gpu-overlaps");
const gpuParity = document.querySelector("#gpu-parity");
const gpuTiming = document.querySelector("#gpu-timing");

if (
  !(ticksInput instanceof HTMLInputElement) ||
  !(ticksLabel instanceof HTMLElement) ||
  !(startPosition instanceof HTMLElement) ||
  !(velocity instanceof HTMLElement) ||
  !(resultPosition instanceof HTMLElement) ||
  !(runtimeStatus instanceof HTMLElement) ||
  !(entity instanceof HTMLElement) ||
  !(webgpuToggle instanceof HTMLInputElement) ||
  !(webgpuStatus instanceof HTMLElement) ||
  !(webgpuBodies instanceof HTMLElement) ||
  !(cpuOverlaps instanceof HTMLElement) ||
  !(gpuOverlaps instanceof HTMLElement) ||
  !(gpuParity instanceof HTMLElement) ||
  !(gpuTiming instanceof HTMLElement)
) {
  throw new Error("ECS Lab demo markup is incomplete.");
}

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

function positionEntity(x, y) {
  const left = Math.max(3, Math.min(97, 10 + (x / 80) * 82));
  const bottom = Math.max(4, Math.min(96, 10 + (y / 60) * 78));
  entity.style.left = `${left}%`;
  entity.style.bottom = `${bottom}%`;
}

function requiredWasmFunctions(exports, names) {
  return names.every((name) => typeof exports[name] === "function");
}

async function configureWebGpu(exports) {
  const required = [
    "webgpu_dynamic_body_count",
    "webgpu_frame_steps",
    "webgpu_body_count",
    "webgpu_cpu_overlap_count",
    "webgpu_pair_word_count",
    "webgpu_pair_word",
    "webgpu_aabb_value",
  ];
  if (!requiredWasmFunctions(exports, required)) {
    webgpuToggle.disabled = true;
    webgpuStatus.dataset.state = "error";
    webgpuStatus.textContent = "The WebAssembly module does not expose the WebGPU parity fixture.";
    return;
  }

  const dynamicCount = Number(exports.webgpu_dynamic_body_count());
  const frameSteps = Number(exports.webgpu_frame_steps());
  const bodyCount = Number(exports.webgpu_body_count());
  const wordCount = Number(exports.webgpu_pair_word_count());
  const rustOverlapCount = Number(exports.webgpu_cpu_overlap_count());
  if (
    !Number.isInteger(bodyCount) ||
    bodyCount <= 0 ||
    !Number.isInteger(wordCount) ||
    wordCount <= 0
  ) {
    webgpuToggle.disabled = true;
    webgpuStatus.dataset.state = "error";
    webgpuStatus.textContent = "Rust could not prepare the deterministic falling-box collision frame.";
    return;
  }

  webgpuBodies.textContent = `${bodyCount} (${dynamicCount} dynamic + 1 fixed), after ${frameSteps} steps`;
  cpuOverlaps.textContent = String(rustOverlapCount);

  if (!("gpu" in navigator)) {
    webgpuToggle.disabled = true;
    webgpuStatus.textContent =
      "WebGPU is unavailable here. The Rust/CPU path remains authoritative and the demo continues without GPU acceleration.";
    return;
  }

  webgpuStatus.dataset.state = "ready";
  webgpuStatus.textContent =
    "WebGPU is available but optional. Enable it to recompute the same AABB pair bitset on the GPU and compare every word with Rust.";

  const runParityCheck = async () => {
    if (!webgpuToggle.checked) {
      webgpuStatus.dataset.state = "ready";
      webgpuStatus.textContent = "WebGPU is disabled; Rust/CPU collision evidence remains authoritative.";
      gpuOverlaps.textContent = "—";
      gpuParity.textContent = "Not run";
      gpuTiming.textContent = "—";
      return;
    }

    webgpuToggle.disabled = true;
    webgpuStatus.dataset.state = "running";
    webgpuStatus.textContent = "Running the optional WebGPU AABB pass and checking exact pair parity…";
    gpuOverlaps.textContent = "Running…";
    gpuParity.textContent = "Checking…";
    gpuTiming.textContent = "Withheld until parity";

    try {
      const packedAabbs = new Float32Array(bodyCount * 8);
      for (let body = 0; body < bodyCount; body += 1) {
        for (let lane = 0; lane < 8; lane += 1) {
          const value = Number(exports.webgpu_aabb_value(body, lane));
          if (!Number.isFinite(value)) {
            throw new Error(`Rust returned an invalid AABB lane for body ${body}.`);
          }
          packedAabbs[body * 8 + lane] = value;
        }
      }

      const rustWords = new Uint32Array(wordCount);
      for (let index = 0; index < wordCount; index += 1) {
        rustWords[index] = Number(exports.webgpu_pair_word(index)) >>> 0;
      }

      const measurement = await runWebGpuAabbPairs(packedAabbs, bodyCount);
      const exactParity =
        measurement.bitset.length === rustWords.length &&
        rustWords.every((word, index) => measurement.bitset[index] === word);

      gpuOverlaps.textContent = String(measurement.overlaps);
      gpuParity.textContent = exactParity ? "Exact" : "Mismatch";
      if (!exactParity) {
        webgpuStatus.dataset.state = "error";
        webgpuStatus.textContent =
          "WebGPU returned a different pair bitset. Its timing is suppressed and Rust/CPU remains authoritative.";
        gpuTiming.textContent = "Suppressed";
        return;
      }

      webgpuStatus.dataset.state = "verified";
      webgpuStatus.textContent =
        "Exact CPU↔WebGPU pair parity verified. The timing below is now eligible as descriptive evidence.";
      gpuTiming.textContent = `${measurement.totalMs.toFixed(2)} ms end-to-end (${measurement.setupMs.toFixed(2)} ms setup + ${measurement.runMs.toFixed(2)} ms dispatch/readback)`;
    } catch (error) {
      webgpuStatus.dataset.state = "error";
      webgpuStatus.textContent =
        error instanceof Error ? error.message : "The optional WebGPU verification failed.";
      gpuOverlaps.textContent = "Unavailable";
      gpuParity.textContent = "Not verified";
      gpuTiming.textContent = "Suppressed";
    } finally {
      webgpuToggle.disabled = false;
    }
  };

  webgpuToggle.addEventListener("change", () => {
    void runParityCheck();
  });

  if (new URLSearchParams(window.location.search).get("webgpu") === "1") {
    webgpuToggle.checked = true;
    await runParityCheck();
  }
}

try {
  const { instance } = await instantiateWasm("./pkg/ecs_web_demo.wasm");
  const exports = instance.exports;
  const referenceExports = [
    "start_x",
    "start_y",
    "velocity_x",
    "velocity_y",
    "position_x_after",
    "position_y_after",
  ];

  if (!requiredWasmFunctions(exports, referenceExports)) {
    throw new Error("WebAssembly module does not expose the expected ECS demo functions.");
  }

  const sx = Number(exports.start_x());
  const sy = Number(exports.start_y());
  const vx = Number(exports.velocity_x());
  const vy = Number(exports.velocity_y());
  startPosition.textContent = `(${sx}, ${sy})`;
  velocity.textContent = `(${vx}, ${vy}) / tick`;

  const render = () => {
    const ticks = Number(ticksInput.value);
    const x = Number(exports.position_x_after(ticks));
    const y = Number(exports.position_y_after(ticks));
    ticksLabel.textContent = String(ticks);
    resultPosition.textContent = `(${x}, ${y})`;
    positionEntity(x, y);
  };

  ticksInput.addEventListener("input", render);
  runtimeStatus.dataset.state = "ready";
  runtimeStatus.textContent =
    "Running ecs-reference::ReferenceWorld as WebAssembly; JavaScript only visualizes the returned snapshot coordinates.";
  render();
  await configureWebGpu(exports);
} catch (error) {
  runtimeStatus.dataset.state = "error";
  runtimeStatus.textContent = error instanceof Error ? error.message : "Unable to start ECS WebAssembly demo.";
  resultPosition.textContent = "Unavailable";
  webgpuToggle.disabled = true;
  webgpuStatus.dataset.state = "error";
  webgpuStatus.textContent = "WebGPU verification requires the Rust WebAssembly fixture.";
}
