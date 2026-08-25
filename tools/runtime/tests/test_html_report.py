import json
import sys
import tempfile
import unittest
from pathlib import Path

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

from flight_review_report import _evaluation, _timing_rows
from html_report import _planning_continuity, _samples, _trajectory_smoothness


class HtmlReportSmoothnessTest(unittest.TestCase):
    def test_overlapping_plans_are_compared_at_handover_time(self) -> None:
        # Each plan is three seconds long but is replaced after 0.2 seconds.
        # The old implementation compared the old plan's terminal velocity to
        # the new plan's first velocity and reported a false discontinuity.
        records = [
            {
                "_publish_time_s": 10.0,
                "duration_s": 3.0,
                "position_points": [[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
                "velocity_points": [[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            },
            {
                "_publish_time_s": 10.2,
                "duration_s": 3.0,
                "position_points": [[0.2, 0.0, 0.0], [3.2, 0.0, 0.0]],
                "velocity_points": [[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            },
        ]

        metrics, series = _trajectory_smoothness(records)

        self.assertTrue(metrics["timeline_time_anchored"])
        self.assertEqual(metrics["handover_sample_count"], 1)
        self.assertAlmostEqual(metrics["boundary_velocity_jump_mps"]["maximum"], 0.0)
        self.assertAlmostEqual(metrics["boundary_position_jump_m"]["maximum"], 0.0)
        self.assertEqual(len(series["speed"]), 2)
        self.assertEqual(series["speed"][1][0], 10.2)


class HtmlReportEvaluationTest(unittest.TestCase):
    def test_complete_observation_is_passed_from_explicit_evidence(self) -> None:
        result = _evaluation(
            {"verdict": "PASS"},
            {"outcome": "COMPLETE"},
            {
                "expected_outcome": "complete",
                "mission_complete_observed": True,
                "waypoint_acceptance_complete": True,
            },
            {"state": "TRACKING", "navigation_valid": True},
            {
                "estimator_initialized": True,
                "local_position_valid": True,
                "local_velocity_valid": True,
            },
            0.22,
            0.75,
            0.0,
        )

        self.assertEqual(result["overall"], "PASS")
        self.assertTrue(all(status == "PASS" for status in result["gates"].values()))

    def test_temporary_bypass_can_never_render_as_certification_pass(self) -> None:
        result = _evaluation(
            {
                "verdict": "PASS",
                "experimental_bypasses": {
                    "bypass_id": "TB-001",
                    "certification_status": "uncertified_experiment",
                },
            },
            {"outcome": "COMPLETE"},
            {
                "expected_outcome": "complete",
                "mission_complete_observed": True,
                "waypoint_acceptance_complete": True,
            },
            {"state": "TRACKING", "navigation_valid": True},
            {
                "estimator_initialized": True,
                "local_position_valid": True,
                "local_velocity_valid": True,
            },
            0.22,
            0.75,
            0.0,
        )
        self.assertEqual(result["gates"]["temporary_bypass"], "FAIL")
        self.assertEqual(result["overall"], "FAIL")

    def test_unmeasured_evidence_is_not_scored_as_false_or_zero(self) -> None:
        result = _evaluation(
            {"verdict": "FAIL"},
            {"outcome": "UNKNOWN"},
            {
                "expected_outcome": "complete",
                "mission_complete_observed": None,
                "waypoint_acceptance_complete": None,
            },
            {"state": "TRACKING", "navigation_valid": True},
            {
                "estimator_initialized": True,
                "local_position_valid": True,
                "local_velocity_valid": True,
            },
            None,
            None,
            None,
        )

        self.assertEqual(result["overall"], "FAIL")
        self.assertEqual(result["gates"]["mission"], "N/A")
        self.assertEqual(result["gates"]["cross_track"], "N/A")
        self.assertEqual(result["gates"]["collision"], "N/A")

    def test_unavailable_waypoint_evidence_is_not_a_failed_gate(self) -> None:
        result = _evaluation(
            {"verdict": "FAIL"},
            {"outcome": "UNKNOWN"},
            {
                "expected_outcome": "complete",
                "mission_complete_observed": None,
                "waypoint_acceptance_complete": False,
                "waypoint_acceptance_indices": [],
                "reasons": ["waypoint acceptance evidence is unavailable"],
            },
            {"state": "TRACKING", "navigation_valid": True},
            {
                "estimator_initialized": True,
                "local_position_valid": True,
                "local_velocity_valid": True,
            },
            0.1,
            0.75,
            0.0,
        )

        self.assertEqual(result["gates"]["waypoint"], "N/A")
        self.assertEqual(result["overall"], "FAIL")

    def test_runtime_contract_verdict_cannot_be_ignored_by_passing_display_gates(self) -> None:
        result = _evaluation(
            {"verdict": "FAIL"},
            {"outcome": "COMPLETE"},
            {
                "expected_outcome": "complete",
                "mission_complete_observed": True,
                "waypoint_acceptance_complete": True,
            },
            {"state": "TRACKING", "navigation_valid": True},
            {
                "estimator_initialized": True,
                "local_position_valid": True,
                "local_velocity_valid": True,
            },
            0.1,
            0.75,
            0.0,
        )
        self.assertEqual(result["gates"]["runtime_contract"], "FAIL")
        self.assertEqual(result["overall"], "FAIL")

    def test_current_super_navigation_diagnostics_are_parsed_without_fake_planner_fields(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            (session / "samples.jsonl").write_text(
                json.dumps({
                    "kind": "sample",
                    "stream": "mapping_diagnostics",
                    "timestamp_ns": 1_000_000_000,
                    "payload": {
                        "stamp_ns": 1_000_000_000,
                        "statuses": [{
                            "name": "super_navigation/super_planner",
                            "values": {"cycle_count": 4},
                        }],
                    },
                }) + "\n",
                encoding="utf-8",
            )

            _, planning = _samples(session)

        self.assertEqual(len(planning), 1)
        self.assertEqual(planning[0]["cycle_count"], 4)
        self.assertNotIn("planning_total_us", planning[0])

    def test_timing_table_omits_empty_distributions(self) -> None:
        rows = _timing_rows({
            "lio": {"map_maintenance": {"maximum_maintenance_us": 5740.0}},
            "planning": {
                "planning_total_us": {
                    "sample_count": 0,
                    "mean": None,
                    "p50": None,
                    "p95": None,
                    "p99": None,
                    "max": None,
                },
            },
        })

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["component"], "LIO")
        self.assertEqual(rows[0]["max"], 5740.0)

    def test_empty_planner_stream_does_not_become_zero_safety_stop_ratio(self) -> None:
        self.assertIsNone(_planning_continuity([])["safety_stop_ratio"])
        self.assertIsNone(_planning_continuity([{"cycle_count": 4}])["safety_stop_ratio"])
        self.assertEqual(
            _planning_continuity([{"safety_plan_kind": "nominal"}])["safety_stop_ratio"],
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
