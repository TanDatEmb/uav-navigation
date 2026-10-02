from __future__ import annotations

import json
import csv
from pathlib import Path
import tempfile
import unittest

from tools.refactor import baseline_summary


class BaselineSummaryTests(unittest.TestCase):
    def write_session(self, root: Path, *, policy: dict | None = None,
                      report: dict | None = None, metadata: dict | None = None) -> Path:
        session = root / "external-mode-check-fixture"
        session.mkdir()
        (session / "metadata.json").write_text(
            json.dumps(metadata or {"run_id": "fixture-1", "nav_build_sha": "nav"}),
            encoding="utf-8",
        )
        scenario = {"scenario": {"requested_cruise_speed_mps": 5.0}}
        if policy is not None:
            scenario["backup_evidence_experiment"] = policy
        (session / "scenario.json").write_text(json.dumps(scenario), encoding="utf-8")
        (session / "report.json").write_text(
            json.dumps(report or {"runtime_verdict": "PASS"}), encoding="utf-8"
        )
        (session / "monitor.json").write_text("{}", encoding="utf-8")
        (session / "runtime.json").write_text("{}", encoding="utf-8")
        return session

    def test_policy_comes_from_effective_backup_policy(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = self.write_session(
                Path(directory),
                policy={"name": "raycasting_on_backup_unknown", "backup_allow_unknown": True},
            )
            row = baseline_summary.summarize_session(session, matrix="M1", run_idx=1)

        self.assertEqual(row["policy"], "FAST")
        self.assertEqual(row["backup_evidence_experiment"], "raycasting_on_backup_unknown")

    def test_missing_policy_is_unclassified_and_not_evaluable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = self.write_session(Path(directory), report={"runtime_verdict": "PASS"})
            row = baseline_summary.summarize_session(session, matrix="M2", run_idx=1)

        self.assertEqual(row["policy"], "UNCLASSIFIED")
        self.assertEqual(row["classification_status"], "NOT_EVALUABLE")

    def test_missing_adr_metrics_are_not_measured(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = self.write_session(Path(directory))
            row = baseline_summary.summarize_session(session, matrix="M1", run_idx=1)

        self.assertEqual(row["adr_m3_mapping_update_us"], "NOT_MEASURED")
        self.assertEqual(row["adr_m4_solve_us"], "NOT_MEASURED")
        self.assertEqual(row["adr_m6_certify_us"], "NOT_MEASURED")

    def test_assessment_status_overrides_runner_pass_for_classification(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = self.write_session(
                Path(directory),
                policy={"name": "raycasting_on_backup_unknown", "backup_allow_unknown": True},
                report={
                    "runtime_verdict": "PASS",
                    "evaluation": {"assessment_status": "NOT_EVALUABLE"},
                },
            )
            row = baseline_summary.summarize_session(session, matrix="M1", run_idx=1)

        self.assertEqual(row["runtime_verdict"], "PASS")
        self.assertEqual(row["classification_status"], "NOT_EVALUABLE")

    def test_percentile_summary_preserves_empty_as_not_measured(self) -> None:
        self.assertEqual(
            baseline_summary.distribution([1.0, 2.0, 3.0, 4.0]),
            {"n": 4, "p50": 3.0, "p95": 4.0, "p99": 4.0, "max": 4.0},
        )
        self.assertEqual(baseline_summary.distribution([]), "NOT_MEASURED")

    def test_distribution_has_one_row_per_group_and_metric(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = self.write_session(root, policy={"name": "raycasting_on_backup_strict", "backup_allow_unknown": False})
            second_root = root / "second"
            second_root.mkdir()
            second = self.write_session(second_root, policy={"name": "raycasting_on_backup_strict", "backup_allow_unknown": False})
            rows = [
                baseline_summary.summarize_session(first, matrix="M1", run_idx=1),
                baseline_summary.summarize_session(second, matrix="M1", run_idx=2),
            ]
            _, distribution_path = baseline_summary.write_summary(rows, root / "out")
            with distribution_path.open(newline="", encoding="utf-8") as stream:
                output = list(csv.DictReader(stream))

        self.assertEqual(len(output), len(baseline_summary.METRIC_ALIASES))

    def test_nav_sha_comes_from_manifest_source_and_cause_from_triage(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = self.write_session(
                Path(directory),
                policy={"name": "raycasting_on_backup_strict", "backup_allow_unknown": False},
                metadata={
                    "build_provenance": {
                        "manifest": {"source": {"git_head": "manifest-sha", "git_dirty": False}}
                    }
                },
            )
            row = baseline_summary.summarize_session(session, matrix="M1", run_idx=1)
        self.assertEqual(row["nav_build_sha"], "manifest-sha")
        self.assertEqual(row["cause"], "NO_SOLVE_FAILURE")
        self.assertFalse(row["provenance_dirty"])

    def test_dirty_provenance_is_not_evaluable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = self.write_session(
                Path(directory),
                policy={"name": "raycasting_on_backup_strict", "backup_allow_unknown": False},
                metadata={
                    "build_provenance": {
                        "manifest": {"source": {"git_head": "dirty-sha", "git_dirty": True}}
                    }
                },
            )
            row = baseline_summary.summarize_session(session, matrix="M1", run_idx=1)
        self.assertEqual(row["classification_status"], "NOT_EVALUABLE")
        self.assertTrue(row["provenance_dirty"])

    def test_exports_terminal_outcome_and_infrastructure_status(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = self.write_session(Path(directory))
            (session / "scenario.json").write_text(
                json.dumps({
                    "outcome": "PAUSED_SAFETY_STOP",
                    "events": [{"name": "terminal_outcome_assigned", "outcome": "PAUSED_SAFETY_STOP"}],
                }),
                encoding="utf-8",
            )
            (session / "report.json").write_text(
                json.dumps({
                    "runtime_verdict": "BLOCKED",
                    "infrastructure": {
                        "classification": "INFRASTRUCTURE_INVALID",
                        "valid": False,
                        "reasons": ["simulation clock lease gap"],
                    },
                }),
                encoding="utf-8",
            )
            row = baseline_summary.summarize_session(session, matrix="M1", run_idx=1)

        self.assertEqual(row["terminal_outcome"], "PAUSED_SAFETY_STOP")
        self.assertEqual(row["infrastructure_classification"], "INFRASTRUCTURE_INVALID")
        self.assertTrue(row["infrastructure_invalid"])

    def test_writes_pooled_distribution_and_per_cell_table(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = self.write_session(
                root,
                policy={"name": "raycasting_on_backup_strict", "backup_allow_unknown": False},
                metadata={"build_provenance": {"manifest": {"source": {"git_head": "sha", "git_dirty": False}}}},
            )
            (first / "samples.jsonl").write_text(
                '{"mapping_total_update_us": 10}\n{"mapping_total_update_us": 30}\n',
                encoding="utf-8",
            )
            second_root = root / "second"
            second_root.mkdir()
            second = self.write_session(
                second_root,
                policy={"name": "raycasting_on_backup_strict", "backup_allow_unknown": False},
                metadata={"build_provenance": {"manifest": {"source": {"git_head": "sha", "git_dirty": False}}}},
            )
            (second / "samples.jsonl").write_text(
                '{"mapping_total_update_us": 50}\n',
                encoding="utf-8",
            )
            for session, outcome in ((first, "PAUSED_SAFETY_STOP"), (second, "COMPLETE")):
                (session / "scenario.json").write_text(json.dumps({"outcome": outcome}), encoding="utf-8")
            rows = [
                baseline_summary.summarize_session(first, matrix="M1", run_idx=1),
                baseline_summary.summarize_session(second, matrix="M1", run_idx=2),
            ]
            baseline_summary.write_summary(rows, root / "out")
            with (root / "out" / "baseline_pooled_distribution.csv").open(newline="", encoding="utf-8") as stream:
                pooled = list(csv.DictReader(stream))
            with (root / "out" / "baseline_by_cell.csv").open(newline="", encoding="utf-8") as stream:
                cells = list(csv.DictReader(stream))

        mapping = next(row for row in pooled if row["metric"] == "adr_m3_mapping_update_us")
        self.assertEqual(mapping["n"], "3")
        self.assertEqual(mapping["p50"], "30.0")
        self.assertEqual(len(cells), 1)
        self.assertEqual(cells[0]["paused_safety_stop"], "1")
        self.assertEqual(cells[0]["paused_rate_pct"], "50.0")


if __name__ == "__main__":
    unittest.main()
