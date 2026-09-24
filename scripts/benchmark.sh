#!/usr/bin/env bash
set -euo pipefail

mode="${1:-full}"

fingerprint="unverified"
if command -v coding-tooling >/dev/null 2>&1; then
  fingerprint="$(coding-tooling environment fingerprint --json | python3 -c 'import json,sys; print(json.load(sys.stdin)["data"]["fingerprint"])')"
fi

case "$mode" in
  smoke) runner_mode=benchmark-smoke ;;
  full) runner_mode=benchmark ;;
  storage)
    cargo run --locked --release -p ecs-runner --bin storage-benchmark -- --bench "$fingerprint"
    exit
    ;;
  *) printf 'usage: %s [smoke|full|storage]\n' "$0" >&2; exit 2 ;;
esac

cargo run --locked --release -p ecs-runner --bin ecs-runner -- "$runner_mode" "$fingerprint"
