#!/usr/bin/env bash
set -euo pipefail

mode="${1:-full}"
case "$mode" in
  smoke) runner_mode=benchmark-smoke ;;
  full) runner_mode=benchmark ;;
  *) printf 'usage: %s [smoke|full]\n' "$0" >&2; exit 2 ;;
esac

fingerprint="unverified"
if command -v coding-tooling >/dev/null 2>&1; then
  fingerprint="$(coding-tooling environment fingerprint --json | python3 -c 'import json,sys; print(json.load(sys.stdin)["data"]["fingerprint"])')"
fi

cargo run --locked --release -p ecs-runner -- "$runner_mode" "$fingerprint"
