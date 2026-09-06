const MATERIAL_NAMES = Object.freeze([
  "Elastic",
  "Bouncy",
  "Balanced",
  "Damped",
  "High-friction",
  "Dead-stop",
]);
const SAMPLE_STRIDE = 2;
const REST_Y = 1;

const status = document.querySelector("#material-lab-status");
const traces = document.querySelector("#material-traces");

if (!(status instanceof HTMLElement) || !(traces instanceof HTMLElement)) {
  throw new Error("The material-response lab markup is incomplete.");
}

function requiredExport(exports, name) {
  const value = exports[name];
  if (typeof value !== "function") {
    throw new Error(`Wasm export ${name} is missing`);
  }
  return value;
}

async function loadMaterialWasm() {
  const response = await fetch("../pkg/ecs_web_demo.wasm");
  if (!response.ok) {
    throw new Error(`Wasm request failed with HTTP ${response.status}`);
  }
  const bytes = await response.arrayBuffer();
  const { instance } = await WebAssembly.instantiate(bytes, {});
  const exports = instance.exports;
  return {
    count: requiredExport(exports, "physics_material_demo_count"),
    maxSteps: requiredExport(exports, "physics_material_demo_max_steps"),
    restitution: requiredExport(exports, "physics_material_demo_restitution_milli"),
    friction: requiredExport(exports, "physics_material_demo_friction_milli"),
    positionX: requiredExport(exports, "physics_material_demo_position_x"),
    positionY: requiredExport(exports, "physics_material_demo_position_y"),
  };
}

function sampleTrace(api, bodyIndex, maxSteps) {
  const initialX = api.positionX(bodyIndex, 0);
  const points = [];
  for (let step = 0; step <= maxSteps; step += SAMPLE_STRIDE) {
    points.push({
      step,
      x: api.positionX(bodyIndex, step) - initialX,
      y: api.positionY(bodyIndex, step),
    });
  }
  if (points.at(-1)?.step !== maxSteps) {
    points.push({
      step: maxSteps,
      x: api.positionX(bodyIndex, maxSteps) - initialX,
      y: api.positionY(bodyIndex, maxSteps),
    });
  }
  return points;
}

function reboundHeight(points) {
  const contactIndex = points.findIndex((point) => point.y <= REST_Y + 0.08);
  if (contactIndex < 0) {
    return null;
  }
  return Math.max(...points.slice(contactIndex).map((point) => point.y));
}

function createSvg(points, bounds) {
  const namespace = "http://www.w3.org/2000/svg";
  const svg = document.createElementNS(namespace, "svg");
  svg.classList.add("material-trace");
  svg.setAttribute("viewBox", "0 0 160 72");
  svg.setAttribute("role", "img");
  svg.setAttribute("aria-label", "Rust-computed vertical bounce and horizontal slide over ten seconds");

  const xRange = Math.max(1e-6, bounds.maxX - bounds.minX);
  const yRange = Math.max(1e-6, bounds.maxY - bounds.minY);
  const mapX = (value) => 8 + ((value - bounds.minX) / xRange) * 144;
  const mapY = (value) => 64 - ((value - bounds.minY) / yRange) * 56;

  const floor = document.createElementNS(namespace, "line");
  floor.setAttribute("x1", "8");
  floor.setAttribute("x2", "152");
  floor.setAttribute("y1", String(mapY(REST_Y)));
  floor.setAttribute("y2", String(mapY(REST_Y)));
  floor.classList.add("material-floor");
  svg.append(floor);

  const polyline = document.createElementNS(namespace, "polyline");
  polyline.setAttribute(
    "points",
    points.map((point) => `${mapX(point.x).toFixed(2)},${mapY(point.y).toFixed(2)}`).join(" "),
  );
  polyline.classList.add("material-path");
  svg.append(polyline);
  return svg;
}

function renderTraceCard(trace, bounds) {
  const article = document.createElement("article");
  article.className = "material-card";

  const heading = document.createElement("div");
  heading.className = "material-card-heading";
  const title = document.createElement("h4");
  title.textContent = trace.name;
  const coefficients = document.createElement("p");
  coefficients.textContent = `restitution ${(trace.restitution / 1000).toFixed(2)} · friction ${(trace.friction / 1000).toFixed(2)}`;
  heading.append(title, coefficients);

  const metrics = document.createElement("p");
  metrics.className = "material-metrics";
  const rebound = reboundHeight(trace.points);
  const slide = trace.points.at(-1)?.x ?? 0;
  metrics.textContent = rebound === null
    ? `No floor impact in sample · slide ${slide.toFixed(2)}`
    : `Post-impact peak ${rebound.toFixed(2)} · slide ${slide.toFixed(2)}`;

  article.append(heading, createSvg(trace.points, bounds), metrics);
  return article;
}

async function main() {
  try {
    const api = await loadMaterialWasm();
    const count = api.count();
    const maxSteps = api.maxSteps();
    if (!Number.isInteger(count) || count <= 0 || count > MATERIAL_NAMES.length) {
      throw new Error(`Rust returned invalid material fixture count ${count}`);
    }
    if (!Number.isInteger(maxSteps) || maxSteps <= 0) {
      throw new Error(`Rust returned invalid material fixture horizon ${maxSteps}`);
    }

    const materialTraces = Array.from({ length: count }, (_, bodyIndex) => ({
      name: MATERIAL_NAMES[bodyIndex] ?? `Material ${bodyIndex + 1}`,
      restitution: api.restitution(bodyIndex),
      friction: api.friction(bodyIndex),
      points: sampleTrace(api, bodyIndex, maxSteps),
    }));
    const allPoints = materialTraces.flatMap((trace) => trace.points);
    const bounds = {
      minX: Math.min(0, ...allPoints.map((point) => point.x)),
      maxX: Math.max(1, ...allPoints.map((point) => point.x)),
      minY: Math.min(0, ...allPoints.map((point) => point.y)),
      maxY: Math.max(1, ...allPoints.map((point) => point.y)),
    };

    traces.replaceChildren(...materialTraces.map((trace) => renderTraceCard(trace, bounds)));
    status.textContent =
      `${count} isolated bodies · identical mass, shape, start state, and 60 Hz gravity · only restitution and friction differ. Curves are authoritative Rust positions.`;
  } catch (error) {
    status.textContent = `Could not build the material-response comparison: ${error instanceof Error ? error.message : String(error)}`;
  }
}

void main();
