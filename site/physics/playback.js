const PLAYBACK_FPS = 60;
const FRAME_DURATION_MS = 1000 / PLAYBACK_FPS;
const prefersReducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

const frameInput = document.querySelector("#frame");
const runButton = document.querySelector("#run-frames");
const resetButton = document.querySelector("#reset-frame");
const stepForwardButton = document.querySelector("#step-frame");
const stepBackButton = document.querySelector("#step-back");
const runtimeStatus = document.querySelector("#runtime-status");

if (
  !(frameInput instanceof HTMLInputElement) ||
  !(runButton instanceof HTMLButtonElement) ||
  !(resetButton instanceof HTMLButtonElement) ||
  !(stepForwardButton instanceof HTMLButtonElement) ||
  !(stepBackButton instanceof HTMLButtonElement) ||
  !(runtimeStatus instanceof HTMLElement)
) {
  throw new Error("The 60 FPS physics playback controls are incomplete.");
}

let running = false;
let advancing = false;
let animationHandle = 0;
let startedAt = 0;
let originStep = 0;
let lastRenderedStep = 0;

function currentStep() {
  return Number.parseInt(frameInput.value, 10);
}

function maxStep() {
  return Number.parseInt(frameInput.max, 10);
}

function dispatchStep(step, settle = false) {
  advancing = true;
  try {
    frameInput.value = String(step);
    frameInput.dispatchEvent(new Event("input", { bubbles: true }));
    if (settle) {
      frameInput.dispatchEvent(new Event("change", { bubbles: true }));
    }
  } finally {
    advancing = false;
  }
}

function stopPlayback({ settle = false } = {}) {
  if (animationHandle !== 0) {
    window.cancelAnimationFrame(animationHandle);
    animationHandle = 0;
  }
  const wasRunning = running;
  running = false;
  runButton.textContent = "Run";
  if (wasRunning && settle) {
    dispatchStep(currentStep(), true);
  }
}

function finishPlayback() {
  running = false;
  animationHandle = 0;
  runButton.textContent = "Run";
  dispatchStep(maxStep(), true);
}

function tick(now) {
  if (!running) {
    animationHandle = 0;
    return;
  }

  const elapsed = Math.max(0, now - startedAt);
  const elapsedFrames = Math.floor(elapsed / FRAME_DURATION_MS);
  const targetStep = Math.min(maxStep(), originStep + elapsedFrames);

  if (targetStep !== lastRenderedStep) {
    lastRenderedStep = targetStep;
    dispatchStep(targetStep);
  }

  if (targetStep >= maxStep()) {
    finishPlayback();
    return;
  }

  animationHandle = window.requestAnimationFrame(tick);
}

function startPlayback() {
  if (!runtimeStatus.textContent.startsWith("Rust/Wasm ready.")) {
    return;
  }

  if (currentStep() >= maxStep()) {
    dispatchStep(0);
  }

  if (prefersReducedMotion) {
    dispatchStep(maxStep(), true);
    return;
  }

  running = true;
  runButton.textContent = "Pause";
  originStep = currentStep();
  lastRenderedStep = originStep;
  startedAt = performance.now();
  animationHandle = window.requestAnimationFrame(tick);
}

function togglePlayback(event) {
  event.preventDefault();
  event.stopImmediatePropagation();
  if (running) {
    stopPlayback({ settle: true });
  } else {
    startPlayback();
  }
}

runButton.addEventListener("click", togglePlayback, true);
frameInput.addEventListener("input", () => {
  if (!advancing) {
    stopPlayback();
  }
});
for (const button of [resetButton, stepForwardButton, stepBackButton]) {
  button.addEventListener("click", () => stopPlayback(), true);
}
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    stopPlayback({ settle: true });
  }
});
