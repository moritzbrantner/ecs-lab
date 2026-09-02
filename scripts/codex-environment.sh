#!/usr/bin/env bash
set -euo pipefail

mode="${1:-setup}"
if [[ "$mode" != "setup" && "$mode" != "maintenance" ]]; then
  printf 'usage: %s [setup|maintenance]\n' "$0" >&2
  exit 2
fi

root="$(git rev-parse --show-toplevel)"
config="$root/.repository-environment.toml"

if [[ ! -f "$config" ]]; then
  printf 'missing environment-v1 config: %s\n' "$config" >&2
  exit 2
fi

run_privileged() {
  if command -v sudo >/dev/null 2>&1; then sudo "$@"; else "$@"; fi
}

if [[ "$mode" == "setup" ]] && command -v apt-get >/dev/null 2>&1; then
  mapfile -t apt_packages < <(python3 - "$config" <<'PY'
import sys, tomllib
with open(sys.argv[1], 'rb') as handle:
    data = tomllib.load(handle)
for package in data.get('system', {}).get('apt', []):
    print(package)
PY
  )
  if (( ${#apt_packages[@]} )); then
    run_privileged apt-get update
    run_privileged apt-get install -y --no-install-recommends "${apt_packages[@]}"
  fi
fi

rust_toolchain="$(python3 - "$root/rust-toolchain.toml" <<'PY'
import pathlib, sys, tomllib
path = pathlib.Path(sys.argv[1])
print(tomllib.loads(path.read_text()).get('toolchain', {}).get('channel', '') if path.is_file() else '')
PY
)"
if ! [[ "$rust_toolchain" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  printf 'Rust toolchain must use an exact version, got %s\n' "$rust_toolchain" >&2
  exit 2
fi
if ! command -v rustup >/dev/null 2>&1; then
  printf '%s\n' 'rustup is required before repository setup' >&2
  exit 2
fi
rustup toolchain install "$rust_toolchain" --profile minimal
mapfile -t rust_components < <(python3 - "$root/rust-toolchain.toml" <<'PY'
import pathlib, sys, tomllib
path = pathlib.Path(sys.argv[1])
for component in tomllib.loads(path.read_text()).get('toolchain', {}).get('components', []):
    print(component)
PY
)
for component in "${rust_components[@]}"; do
  rustup component add --toolchain "$rust_toolchain" "$component"
done

mapfile -t environment_commands < <(python3 - "$config" "$mode" <<'PY'
import sys, tomllib
with open(sys.argv[1], 'rb') as handle:
    data = tomllib.load(handle)
for command in data.get(sys.argv[2], {}).get('commands', []):
    print(command)
PY
)
for command in "${environment_commands[@]}"; do
  (cd "$root" && bash -lc "$command")
done

observed_rust="$(cd "$root" && rustc --version | awk '{print $2}')"
if [[ "$observed_rust" != "$rust_toolchain" ]]; then
  printf 'Rust preflight mismatch: expected %s, got %s\n' "$rust_toolchain" "$observed_rust" >&2
  exit 1
fi
