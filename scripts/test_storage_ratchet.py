"""Contract mutations prove the gate rejects incomplete and regressing evidence."""
import copy
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import storage_ratchet as ratchet


class StorageRatchetTests(unittest.TestCase):
    def setUp(self):
        self.rows = {("dense", "cached-sparse"): dict.fromkeys(ratchet.FIELDS, 4)}
        self.rows[("dense", "cached-sparse")]["component_lookups"] = 0

    def evidence(self, rows=None):
        rows = self.rows if rows is None else rows
        return "\n".join(
            "storage_work " + " ".join(f"{key}={value}" for key, value in
                zip(ratchet.HEADER, (*identity, *(values[field] for field in ratchet.FIELDS)), strict=True))
            for identity, values in rows.items()
        ) + "\n"

    def test_baseline_and_evidence_round_trip(self):
        self.assertEqual(ratchet.parse_baseline(ratchet.render(self.rows)), self.rows)
        self.assertEqual(ratchet.parse_evidence(self.evidence()), self.rows)

    def test_each_regression_is_rejected_including_zero_budgets(self):
        for field in ratchet.FIELDS:
            with self.subTest(field=field):
                candidate = copy.deepcopy(self.rows)
                candidate[("dense", "cached-sparse")][field] += 1
                with self.assertRaisesRegex(ValueError, field):
                    ratchet.compare(candidate, self.rows)

    def test_useful_work_and_fixture_size_cannot_shrink(self):
        for field in ratchet.EXACT:
            candidate = copy.deepcopy(self.rows)
            candidate[("dense", "cached-sparse")][field] -= 1
            with self.assertRaisesRegex(ValueError, field):
                ratchet.compare(candidate, self.rows)

    def test_induced_work_can_improve(self):
        candidate = copy.deepcopy(self.rows)
        candidate[("dense", "cached-sparse")]["query_cache_updates"] -= 1
        ratchet.compare(candidate, self.rows)

    def test_missing_extra_and_duplicate_rows_fail(self):
        with self.assertRaises(ValueError):
            ratchet.compare({}, self.rows)
        extra = {**self.rows, ("new", "reference"): dict.fromkeys(ratchet.FIELDS, 1)}
        with self.assertRaises(ValueError):
            ratchet.compare(extra, self.rows)
        with self.assertRaises(ValueError):
            ratchet.parse_evidence(self.evidence() * 2)
        with self.assertRaises(ValueError):
            ratchet.parse_baseline(ratchet.render(self.rows) + ratchet.render(self.rows).splitlines()[1])

    def test_previous_baseline_allows_additions_not_removals_or_increases(self):
        added = {**self.rows, ("new", "reference"): dict.fromkeys(ratchet.FIELDS, 1)}
        ratchet.compare(added, self.rows, allow_additions=True)
        with self.assertRaises(ValueError):
            ratchet.compare(self.rows, added, allow_additions=True)
        weakened = copy.deepcopy(self.rows)
        weakened[("dense", "cached-sparse")]["query_rows"] += 1
        with self.assertRaises(ValueError):
            ratchet.compare(weakened, self.rows, allow_additions=True)

    def test_bad_evidence_fails_closed(self):
        valid = self.evidence()
        malformed = [
            "", "storage_timing median_ns=1\n", "unexpected output\n",
            valid.replace("component_lookups=0", "component_lookups=-1"),
            valid.replace("component_lookups=0", "component_lookups=NaN"),
            valid.replace("component_lookups=0", "component_lookups=1.5"),
            valid.replace("component_lookups=0", "component_lookups=true"),
            valid.replace("component_lookups=0", "component_lookups=18446744073709551616"),
            valid.replace("component_lookups=0", "component_lookups=00"),
            valid.replace("component_lookups=0", "unknown=0"),
            valid.replace("component_lookups=0", "component_lookups=0 component_lookups=0"),
            valid.replace("component_lookups=0", ""),
        ]
        for value in malformed:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    ratchet.parse_evidence(value)

    def test_bad_baselines_fail_closed(self):
        valid = ratchet.render(self.rows)
        for value in ["", "\t".join(ratchet.HEADER), valid.replace("operations", "unknown"),
                      valid.replace("\t4", "\t-1", 1), valid + "broken\n"]:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    ratchet.parse_baseline(value)

    def test_timing_has_no_effect(self):
        parsed = ratchet.parse_evidence(self.evidence() + "storage_timing median_ns=9999999999999\n")
        ratchet.compare(parsed, self.rows)

    def test_cli_tightens_and_never_learns_regressions(self):
        with tempfile.TemporaryDirectory() as directory:
            baseline = Path(directory) / "baseline.tsv"
            evidence = Path(directory) / "evidence.txt"
            baseline.write_text(ratchet.render(self.rows), encoding="utf-8")
            improved = copy.deepcopy(self.rows)
            improved[("dense", "cached-sparse")]["query_rows"] = 2
            evidence.write_text(self.evidence(improved), encoding="utf-8")
            command = [sys.executable, str(Path(ratchet.__file__)), "--baseline", str(baseline),
                       "--evidence", str(evidence), "--tighten"]
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            saved = baseline.read_text(encoding="utf-8")
            self.assertEqual(ratchet.parse_baseline(saved), improved)
            evidence.write_text(self.evidence(), encoding="utf-8")
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            self.assertEqual(result.returncode, 1)
            self.assertIn("query_rows", result.stderr)
            self.assertEqual(baseline.read_text(encoding="utf-8"), saved)


if __name__ == "__main__":
    unittest.main()
