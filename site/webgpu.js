const WORKGROUP_SIZE = 8;

const SHADER = /* wgsl */ `
struct Aabb {
  min: vec4<f32>,
  max: vec4<f32>,
}

struct Params {
  object_count: u32,
  word_count: u32,
  _padding0: u32,
  _padding1: u32,
}

@group(0) @binding(0) var<storage, read> aabbs: array<Aabb>;
@group(0) @binding(1) var<storage, read_write> hits: array<atomic<u32>>;
@group(0) @binding(2) var<uniform> params: Params;

fn overlaps(left: Aabb, right: Aabb) -> bool {
  return left.min.x <= right.max.x && left.max.x >= right.min.x
    && left.min.y <= right.max.y && left.max.y >= right.min.y
    && left.min.z <= right.max.z && left.max.z >= right.min.z;
}

fn pair_index(left: u32, right: u32, count: u32) -> u32 {
  return (left * (2u * count - left - 1u)) / 2u + (right - left - 1u);
}

@compute @workgroup_size(8, 8, 1)
fn main(@builtin(global_invocation_id) id: vec3<u32>) {
  let left = id.x;
  let right = id.y;
  if (left >= params.object_count || right >= params.object_count || left >= right) {
    return;
  }

  if (overlaps(aabbs[left], aabbs[right])) {
    let index = pair_index(left, right, params.object_count);
    let word = index / 32u;
    if (word < params.word_count) {
      atomicOr(&hits[word], 1u << (index % 32u));
    }
  }
}
`;

export async function runWebGpuAabbPairs(aabbs, objectCount) {
  if (!(aabbs instanceof Float32Array)) {
    throw new Error("WebGPU AABB input must be a Float32Array.");
  }
  if (!("gpu" in navigator)) {
    throw new Error("WebGPU is not available in this browser.");
  }
  if (!Number.isInteger(objectCount) || objectCount < 0 || objectCount > 65_536) {
    throw new Error("The WebGPU pair encoding supports 0 to 65,536 bodies.");
  }
  if (aabbs.length !== objectCount * 8) {
    throw new Error(`Expected ${objectCount * 8} packed AABB floats, received ${aabbs.length}.`);
  }

  const possiblePairs = (objectCount * (objectCount - 1)) / 2;
  if (possiblePairs > 0xffff_ffff) {
    throw new Error("The WebGPU pair bitset exceeds the current u32 index range.");
  }
  const wordCount = Math.ceil(possiblePairs / 32);
  const outputBytes = Math.max(4, wordCount * Uint32Array.BYTES_PER_ELEMENT);
  const inputBytes = Math.max(32, aabbs.byteLength);

  const setupStarted = performance.now();
  const adapter = await navigator.gpu.requestAdapter({ powerPreference: "high-performance" });
  if (!adapter) {
    throw new Error("The browser exposes WebGPU, but no GPU adapter is available.");
  }
  const device = await adapter.requestDevice();
  const module = device.createShaderModule({ label: "ecs-lab AABB all-pairs", code: SHADER });
  const compilation = await module.getCompilationInfo();
  const errors = compilation.messages.filter((message) => message.type === "error");
  if (errors.length > 0) {
    device.destroy();
    throw new Error(
      `WebGPU shader compilation failed: ${errors.map((message) => message.message).join(" | ")}`,
    );
  }
  const pipeline = await device.createComputePipelineAsync({
    label: "ecs-lab AABB all-pairs",
    layout: "auto",
    compute: { module, entryPoint: "main" },
  });
  const setupMs = performance.now() - setupStarted;

  if (inputBytes > device.limits.maxStorageBufferBindingSize) {
    device.destroy();
    throw new Error("The AABB input exceeds this GPU's storage-buffer binding limit.");
  }
  if (outputBytes > device.limits.maxStorageBufferBindingSize) {
    device.destroy();
    throw new Error("The pair bitset exceeds this GPU's storage-buffer binding limit.");
  }

  const inputBuffer = device.createBuffer({
    label: "ecs-lab AABBs",
    size: inputBytes,
    usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST,
  });
  const outputBuffer = device.createBuffer({
    label: "ecs-lab overlap bitset",
    size: outputBytes,
    usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_SRC | GPUBufferUsage.COPY_DST,
  });
  const readbackBuffer = device.createBuffer({
    label: "ecs-lab overlap readback",
    size: outputBytes,
    usage: GPUBufferUsage.COPY_DST | GPUBufferUsage.MAP_READ,
  });
  const paramsBuffer = device.createBuffer({
    label: "ecs-lab WebGPU params",
    size: 16,
    usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
  });

  try {
    const params = new Uint32Array([objectCount, wordCount, 0, 0]);
    device.queue.writeBuffer(inputBuffer, 0, aabbs);
    device.queue.writeBuffer(paramsBuffer, 0, params);
    const bindGroup = device.createBindGroup({
      label: "ecs-lab AABB all-pairs",
      layout: pipeline.getBindGroupLayout(0),
      entries: [
        { binding: 0, resource: { buffer: inputBuffer } },
        { binding: 1, resource: { buffer: outputBuffer } },
        { binding: 2, resource: { buffer: paramsBuffer } },
      ],
    });

    const workgroups = Math.ceil(objectCount / WORKGROUP_SIZE);
    if (workgroups > device.limits.maxComputeWorkgroupsPerDimension) {
      throw new Error("The body count exceeds this GPU's compute-workgroup dimension limit.");
    }

    const runStarted = performance.now();
    const encoder = device.createCommandEncoder({ label: "ecs-lab AABB all-pairs" });
    encoder.clearBuffer(outputBuffer);
    const pass = encoder.beginComputePass();
    pass.setPipeline(pipeline);
    pass.setBindGroup(0, bindGroup);
    pass.dispatchWorkgroups(workgroups, workgroups, 1);
    pass.end();
    encoder.copyBufferToBuffer(outputBuffer, 0, readbackBuffer, 0, outputBytes);
    device.queue.submit([encoder.finish()]);
    await readbackBuffer.mapAsync(GPUMapMode.READ);
    const bitset = new Uint32Array(readbackBuffer.getMappedRange().slice(0, wordCount * 4));
    const runMs = performance.now() - runStarted;
    const overlaps = countSetBits(bitset);
    readbackBuffer.unmap();

    return { bitset, overlaps, setupMs, runMs, totalMs: setupMs + runMs };
  } finally {
    inputBuffer.destroy();
    outputBuffer.destroy();
    readbackBuffer.destroy();
    paramsBuffer.destroy();
    device.destroy();
  }
}

function countSetBits(values) {
  let count = 0;
  for (const value of values) {
    let remaining = value;
    while (remaining !== 0) {
      remaining &= remaining - 1;
      count += 1;
    }
  }
  return count;
}
