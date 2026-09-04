const ticksInput = document.querySelector("#ticks");
const ticksLabel = document.querySelector("#ticks-label");
const startPosition = document.querySelector("#start-position");
const velocity = document.querySelector("#velocity");
const resultPosition = document.querySelector("#result-position");
const runtimeStatus = document.querySelector("#runtime-status");
const entity = document.querySelector("#entity");

if (
  !(ticksInput instanceof HTMLInputElement) ||
  !(ticksLabel instanceof HTMLElement) ||
  !(startPosition instanceof HTMLElement) ||
  !(velocity instanceof HTMLElement) ||
  !(resultPosition instanceof HTMLElement) ||
  !(runtimeStatus instanceof HTMLElement) ||
  !(entity instanceof HTMLElement)
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

try {
  const { instance } = await instantiateWasm("./pkg/ecs_web_demo.wasm");
  const { start_x, start_y, velocity_x, velocity_y, position_x_after, position_y_after } =
    instance.exports;

  const exportsAreValid = [
    start_x,
    start_y,
    velocity_x,
    velocity_y,
    position_x_after,
    position_y_after,
  ].every((value) => typeof value === "function");

  if (!exportsAreValid) {
    throw new Error("WebAssembly module does not expose the expected ECS demo functions.");
  }

  const sx = Number(start_x());
  const sy = Number(start_y());
  const vx = Number(velocity_x());
  const vy = Number(velocity_y());
  startPosition.textContent = `(${sx}, ${sy})`;
  velocity.textContent = `(${vx}, ${vy}) / tick`;

  const render = () => {
    const ticks = Number(ticksInput.value);
    const x = Number(position_x_after(ticks));
    const y = Number(position_y_after(ticks));
    ticksLabel.textContent = String(ticks);
    resultPosition.textContent = `(${x}, ${y})`;
    positionEntity(x, y);
  };

  ticksInput.addEventListener("input", render);
  runtimeStatus.dataset.state = "ready";
  runtimeStatus.textContent =
    "Running ecs-reference::ReferenceWorld as WebAssembly; JavaScript only visualizes the returned snapshot coordinates.";
  render();
} catch (error) {
  runtimeStatus.dataset.state = "error";
  runtimeStatus.textContent = error instanceof Error ? error.message : "Unable to start ECS WebAssembly demo.";
  resultPosition.textContent = "Unavailable";
}
