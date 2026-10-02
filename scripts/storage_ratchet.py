#!/usr/bin/env python3
"""Fail-closed deterministic storage budgets; wall-clock output is never a budget."""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

FIELDS = (
    "operations", "integrated_entities", "live_entities", "integration_rows_scanned",
    "component_lookups", "structural_table_transitions", "query_cache_updates",
    "snapshot_slots_scanned", "snapshot_entities_materialized", "entity_index_entries",
    "component_index_slots", "query_index_slots", "query_rows",
    "peak_component_index_slots", "peak_query_index_slots",
)
EXACT = frozenset(("operations", "integrated_entities", "live_entities", "snapshot_entities_materialized"))
HEADER = ("scenario", "implementation", *FIELDS)
Rows = dict[tuple[str, str], dict[str, int]]


def numeric(value: str) -> int:
    if not re.fullmatch(r"0|[1-9][0-9]*", value) or int(value) > 2**64 - 1:
        raise ValueError(f"expected an unsigned 64-bit integer, got {value!r}")
    return int(value)


def add_row(rows: Rows, tokens: list[str]) -> None:
    if len(tokens) != len(HEADER):
        raise ValueError(f"expected {len(HEADER)} columns, got {len(tokens)}")
    key = (tokens[0], tokens[1])
    if not all(re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", part) for part in key):
        raise ValueError(f"invalid scenario/implementation: {key}")
    if key in rows:
        raise ValueError(f"duplicate row: {'/'.join(key)}")
    rows[key] = dict(zip(FIELDS, map(numeric, tokens[2:]), strict=True))


def parse_baseline(text: str) -> Rows:
    lines = [line.split() for line in text.splitlines() if line.strip()]
    if not lines or tuple(lines[0]) != HEADER:
        raise ValueError("missing or incompatible storage-ratchet header")
    rows: Rows = {}
    for tokens in lines[1:]:
        add_row(rows, tokens)
    if not rows:
        raise ValueError("empty storage-ratchet baseline")
    return rows


def parse_evidence(text: str) -> Rows:
    rows: Rows = {}
    for line in text.splitlines():
        if not line.strip() or line.startswith("storage_timing "):
            continue
        if not line.startswith("storage_work "):
            raise ValueError(f"unexpected evidence line: {line[:120]}")
        values: dict[str, str] = {}
        for token in line.split()[1:]:
            field, separator, value = token.partition("=")
            if not separator or field in values:
                raise ValueError(f"malformed or duplicate evidence field: {token}")
            values[field] = value
        if set(values) != set(HEADER):
            raise ValueError("missing or unknown evidence fields")
        add_row(rows, [values[field] for field in HEADER])
    if not rows:
        raise ValueError("no deterministic storage evidence")
    return rows


def compare(candidate: Rows, baseline: Rows, *, allow_additions: bool = False) -> None:
    missing = baseline.keys() - candidate.keys()
    extra = candidate.keys() - baseline.keys()
    if missing or (extra and not allow_additions):
        raise ValueError(f"row coverage changed: missing={sorted(missing)}, extra={sorted(extra)}")
    failures = []
    for key, limits in baseline.items():
        for field, limit in limits.items():
            actual = candidate[key][field]
            invalid = actual != limit if field in EXACT else actual > limit
            if invalid:
                relation = "==" if field in EXACT else "<="
                failures.append(f"{'/'.join(key)} {field}: {actual}, required {relation} {limit}")
    if failures:
        raise ValueError("\n".join(failures))


def render(rows: Rows) -> str:
    lines = ["\t".join(HEADER)]
    for key, values in sorted(rows.items()):
        lines.append("\t".join((*key, *(str(values[field]) for field in FIELDS))))
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--previous", type=Path, help="Base-revision budget: reject weakening/removal")
    parser.add_argument("--tighten", action="store_true", help="Explicitly lower passing budgets to observed work")
    args = parser.parse_args()
    try:
        baseline = parse_baseline(args.baseline.read_text(encoding="utf-8"))
        if args.previous:
            previous = parse_baseline(args.previous.read_text(encoding="utf-8"))
            compare(baseline, previous, allow_additions=True)
        evidence = parse_evidence(args.evidence.read_text(encoding="utf-8"))
        compare(evidence, baseline)
        if args.tighten:
            # Comparison above forbids learning a regression, missing case, or weaker fixture.
            temporary = args.baseline.with_suffix(args.baseline.suffix + ".tmp")
            temporary.write_text(render(evidence), encoding="utf-8")
            temporary.replace(args.baseline)
        print(f"Storage ratchet passed: {len(evidence)} rows; timing excluded.")
        return 0
    except (OSError, ValueError) as error:
        print(f"Storage ratchet failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
