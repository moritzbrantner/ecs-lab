const VALID_STRIDES = new Set([1, 2, 4, 8, 15, 30, 60]);

const frameInput = document.querySelector("#frame");
const stepForwardButton = document.querySelector("#step-frame");
const stepBackButton = document.querySelector("#step-back");
const strideSelect = document.querySelector("#step-stride");

if (
  !(frameInput instanceof HTMLInputElement) ||
  !(stepForwardButton instanceof HTMLButtonElement) ||
  !(stepBackButton instanceof HTMLButtonElement) ||
  !(strideSelect instanceof HTMLSelectElement)
) {
  throw new Error("The physics playback controls are incomplete.");
}

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
  frameInput.value = String(clamp(step, 0, maxStep()));
  frameInput.dispatchEvent(new Event("input", { bubbles: true }));
  frameInput.dispatchEvent(new Event("change", { bubbles: true }));
}

function updateStepLabels() {
  const stride = stepStride();
  stepForwardButton.textContent = `Step +${stride}`;
  stepBackButton.textContent = `Step −${stride}`;
}

function captureForwardStep(event) {
  event.preventDefault();
  event.stopImmediatePropagation();
  setFrame(currentStep() + stepStride());
}

function bindPlaybackControls() {
  updateStepLabels();
  stepForwardButton.addEventListener("click", captureForwardStep, true);
  stepBackButton.addEventListener("click", () => setFrame(currentStep() - stepStride()));
  strideSelect.addEventListener("change", updateStepLabels);
}

bindPlaybackControls();
