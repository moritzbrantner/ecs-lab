#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

temporary="$(mktemp -d)"
trap 'rm -rf "$temporary"' EXIT
baseline=.performance/storage-ratchet.tsv
previous=()
if [[ -n "${BASE_SHA:-}" && ! "$BASE_SHA" =~ ^0+$ ]]; then
  # A bad/missing base commit is an error, not permission to skip the monotonic check.
  git cat-file -e "${BASE_SHA}^{commit}"
  if git cat-file -e "${BASE_SHA}:${baseline}" 2>/dev/null; then
    git show "${BASE_SHA}:${baseline}" > "$temporary/previous.tsv"
    previous=(--previous "$temporary/previous.tsv")
  else
    printf 'Introducing storage ratchet: no baseline exists at the base revision.\n'
  fi
fi

python3 -m unittest discover -s scripts -p 'test_storage_ratchet.py'
cargo run --locked --release -p ecs-runner --bin storage-benchmark | tee "$temporary/evidence.txt"
python3 scripts/storage_ratchet.py --baseline "$baseline" \
  --evidence "$temporary/evidence.txt" "${previous[@]}"
