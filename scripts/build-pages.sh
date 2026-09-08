#!/usr/bin/env bash
set -euo pipefail

rustup target add wasm32-unknown-unknown
cargo test --locked -p ecs-web-demo
cargo build --locked --release -p ecs-web-demo --target wasm32-unknown-unknown

if ! command -v wasm-bindgen >/dev/null 2>&1 || ! wasm-bindgen --version | grep -q '0.2.127'; then
  cargo install wasm-bindgen-cli --version 0.2.127 --locked
fi
rm -rf target/tower-wgpu-renderer target/tower-wgpu-bindgen
CARGO_TARGET_DIR=target/tower-wgpu-renderer \
  cargo build \
  --manifest-path experiments/tower-wgpu-renderer/Cargo.toml \
  --locked \
  --release \
  --target wasm32-unknown-unknown
wasm-bindgen \
  target/tower-wgpu-renderer/wasm32-unknown-unknown/release/ecs_tower_wgpu_renderer.wasm \
  --target web \
  --out-dir target/tower-wgpu-bindgen \
  --out-name tower_wgpu_renderer

node --input-type=module --check < site/app.js
node --input-type=module --check < site/webgpu.js
node --input-type=module --check < site/physics/app.js
node --input-type=module --check < site/physics/temporal.js
node --input-type=module --check < site/physics/tower.js

rm -rf pages-dist
mkdir -p pages-dist/pkg
cp -R site/. pages-dist/
cp target/wasm32-unknown-unknown/release/ecs_web_demo.wasm pages-dist/pkg/ecs_web_demo.wasm
cp target/tower-wgpu-bindgen/tower_wgpu_renderer.js pages-dist/pkg/tower_wgpu_renderer.js
cp target/tower-wgpu-bindgen/tower_wgpu_renderer_bg.wasm pages-dist/pkg/tower_wgpu_renderer_bg.wasm

printf 'Pages artifact ready: %s\n' "$(du -sh pages-dist | cut -f1)"