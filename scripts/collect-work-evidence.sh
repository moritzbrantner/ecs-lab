#!/usr/bin/env bash
# Collector for the root `performance:work` capability.
#
# Runs the Cargo tests that measure the deterministic work counters for the
# operations declared in .performance/contract.json and prints one
# `coding-tooling/work-evidence/v1` document on stdout. Cargo output goes to
# stderr so stdout carries only the evidence. Each test proves snapshot
# equality across reference, sparse-set, cached-sparse and archetype storage
# before it writes `<operation>.json` into ECS_LAB_WORK_EVIDENCE_DIR. With that
# variable set the tests leave the bounds to `coding-tooling performance work`;
# without it, plain `cargo test` asserts the same bounds.
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
cd "$root"

contract=".performance/contract.json"

evidence_dir="$(mktemp -d)"
trap 'rm -rf "$evidence_dir"' EXIT
export ECS_LAB_WORK_EVIDENCE_DIR="$evidence_dir"

cargo test --locked --quiet -p ecs-runner --test work_complexity -- --exact \
  integrate_sparse_subset_scales_with_moving_entities \
  repeated_cached_query_scales_with_returned_rows \
  single_entity_component_churn_does_not_scale_with_population \
  full_snapshot_is_declared_linear_in_entities >&2

# Contract order; a missing fragment means a test did not run or failed.
operations=(
  integrate-sparse-subset
  repeat-cached-query
  add-remove-component
  full-snapshot
)
fragments=()
for operation in "${operations[@]}"; do
  fragment="$evidence_dir/$operation.json"
  if [[ ! -s "$fragment" ]]; then
    printf 'missing work evidence for %s\n' "$operation" >&2
    exit 1
  fi
  fragments+=("$(<"$fragment")")
done

# Hash the contract bytes as they are on disk now, after the tests ran.
if command -v sha256sum >/dev/null 2>&1; then
  contract_sha="$(sha256sum "$contract" | cut -d' ' -f1)"
else
  contract_sha="$(shasum -a 256 "$contract" | cut -d' ' -f1)"
fi
suite="$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["suite"])' "$contract")"

joined="$(IFS=,; printf '%s' "${fragments[*]}")"
printf '{"schemaVersion":"coding-tooling/work-evidence/v1","suite":"%s","contractSha256":"%s","operations":[%s]}\n' \
  "$suite" "$contract_sha" "$joined"
