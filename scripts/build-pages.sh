#!/usr/bin/env bash
set -euo pipefail

rustup target add wasm32-unknown-unknown
cargo test --locked -p ecs-web-demo
cargo build --locked --release -p ecs-web-demo --target wasm32-unknown-unknown

rm -rf pages-dist
mkdir -p pages-dist/pkg
cp -R site/. pages-dist/
cp target/wasm32-unknown-unknown/release/ecs_web_demo.wasm pages-dist/pkg/ecs_web_demo.wasm

printf 'Pages artifact ready: %s\n' "$(du -sh pages-dist | cut -f1)"
