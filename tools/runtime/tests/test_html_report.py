import json
import sys
import tempfile
import unittest
from pathlib import Path

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

from flight_review_report import _evaluation, _safety_stop_status, _timing_rows, line_chart
from html_report import _planning_continuity, _runtime_observability, _safety_timeline, _samples, _trajectory_smoothness


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

    def test_observability_does_not_join_different_trajectory_generations(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            rows = []
            for stamp, generation, position in ((1_000_000_000, 4, [0.0, 0.0, 3.0]),
                                                 (1_100_000_000, 4, [0.1, 0.0, 3.0]),
                                                 (1_200_000_000, 5, [0.2, 0.0, 3.0])):
                rows.append(json.dumps({
                    "kind": "sample",
                    "stream": "pva_command",
                    "timestamp_ns": stamp,
                    "payload": {
                        "stamp_ns": stamp,
                        "position": position,
                        "velocity": [1.0, 0.0, 0.0],
                        "acceleration": [0.0, 0.0, 0.0],
                        "trajectory_id": stamp,
                        "trajectory_generation": generation,
                        "trajectory_flag": 1,
                        "trajectory_status": 1,
                    },
                }))
            scenario_rows = [
                json.dumps({"kind": "pva_command", "sim_time_ns": stamp, "payload": json.loads(row)["payload"]})
                for row, stamp in zip(rows, (1_000_000_000, 1_100_000_000, 1_200_000_000))
            ]
            (session / "scenario.jsonl").write_text("\n".join(scenario_rows) + "\n", encoding="utf-8")
            observed = _runtime_observability(session)

        paths = observed["trajectory_paths"]
        self.assertEqual(len(paths), 2)
        self.assertEqual({item["trajectory_generation"] for item in paths}, {4, 5})
        self.assertEqual(sorted(item["count"] for item in paths), [1, 2])

    def test_legacy_commands_stay_one_explicit_waypoint_role_trace(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            rows = []
            for stamp, position in ((1_000_000_000, [0.0, 0.0, 3.0]),
                                    (1_100_000_000, [0.1, 0.0, 3.0])):
                rows.append(json.dumps({
                    "kind": "sample",
                    "stream": "pva_command",
                    "timestamp_ns": stamp,
                    "payload": {
                        "stamp_ns": stamp,
                        "position": position,
                        "velocity": [1.0, 0.0, 0.0],
                        "trajectory_id": stamp,
                        "trajectory_flag": 1,
                        "trajectory_status": 1,
                    },
                }))
            (session / "samples.jsonl").write_text("\n".join(rows) + "\n", encoding="utf-8")
            (session / "scenario.jsonl").write_text(
                "\n".join(
                    json.dumps({
                        "kind": "pva_command",
                        "sim_time_ns": stamp,
                        "payload": json.loads(row)["payload"],
                    })
                    for stamp, row in zip((1_000_000_000, 1_100_000_000), rows)
                ) + "\n",
                encoding="utf-8",
            )
            observed = _runtime_observability(session)

        paths = observed["trajectory_paths"]
        self.assertEqual(len(paths), 1)
        self.assertEqual(paths[0]["trajectory_identity"], "legacy_waypoint_role")
        self.assertEqual(paths[0]["count"], 2)

    def test_legacy_trace_keeps_waypoint_and_role_boundaries(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            rows = []
            current_waypoint = None
            for stamp, waypoint, flag in (
                (1_000_000_000, 0, 1), (1_100_000_000, 1, 1),
                (1_200_000_000, 1, 2),
            ):
                if waypoint != current_waypoint:
                    rows.append(json.dumps({
                        "kind": "goal", "sim_time_ns": stamp,
                        "payload": {"mission_id": "m", "waypoint_index": waypoint},
                    }))
                    current_waypoint = waypoint
                payload = {
                    "stamp_ns": stamp,
                    "position": [float(waypoint), float(flag), 3.0],
                    "velocity": [1.0, 0.0, 0.0],
                    "trajectory_id": stamp,
                    "trajectory_flag": flag,
                    "trajectory_status": 1,
                }
                rows.append(json.dumps({"kind": "pva_command", "sim_time_ns": stamp, "payload": payload}))
            (session / "scenario.jsonl").write_text("\n".join(rows) + "\n", encoding="utf-8")
            observed = _runtime_observability(session)

        self.assertEqual(len(observed["trajectory_paths"]), 3)

    def test_safety_timeline_recovers_boundary_from_older_event_log(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            events = [
                {"kind": "navigation_mode_status", "sim_time_ns": 62_716_000_000,
                 "payload": {"state_name": "PAUSED", "reason_name": "SAFETY_STOP"}},
                {"kind": "event", "sim_time_ns": 62_732_000_000,
                 "payload": {"name": "px4_hold_handover_requested", "trigger": "safety_stop"}},
            ]
            (session / "scenario.jsonl").write_text(
                "\n".join(json.dumps(item) for item in events) + "\n", encoding="utf-8"
            )
            timeline = _safety_timeline(session)

        self.assertTrue(timeline["safety_stop_observed"])
        self.assertEqual(timeline["safety_stop_reason_name"], "SAFETY_STOP")
        self.assertAlmostEqual(timeline["safety_to_handover_ms"], 16.0)


class HtmlReportEvaluationTest(unittest.TestCase):
    def test_safety_stop_is_observed_not_an_acceptance_pass(self) -> None:
        self.assertEqual(_safety_stop_status(True), "OBSERVE")
        self.assertEqual(_safety_stop_status(False), "N/A")

    def test_line_chart_has_toggleable_legend_for_every_visible_trace(self) -> None:
        chart = line_chart(
            "Velocity components",
            [
                {"label": "measured", "points": [(0.0, 0.0), (1.0, 1.0)], "color": "#1f6feb"},
                {"label": "PVA command", "points": [(0.0, 0.0), (1.0, 0.8)], "color": "#c0392b"},
            ],
            "velocity (m/s)",
        )
        self.assertIn("chart-legend-toggle", chart)
        self.assertIn("measured", chart)
        self.assertIn("PVA command", chart)
        self.assertIn('aria-pressed="true"', chart)

    def test_line_chart_wraps_a_fifth_legend_entry_inside_the_svg(self) -> None:
        chart = line_chart(
            "Velocity ENU",
            [
                {"label": f"trace-{index}", "points": [(0.0, 0.0), (1.0, float(index))], "color": "#1f6feb"}
                for index in range(5)
            ],
            "velocity (m/s)",
        )
        self.assertIn('viewBox="0 0 980 322"', chart)
        self.assertEqual(chart.count('class="chart-legend-toggle"'), 5)
        self.assertIn('y1="44"', chart)

    def test_px4_ned_vectors_are_normalized_to_display_enu(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            rows = []
            for stream in ("external_odometry", "px4_odometry"):
                rows.append(json.dumps({
                    "kind": "sample",
                    "stream": stream,
                    "timestamp_ns": 1_000_000_000,
                    "payload": {
                        "timestamp_us": 1_000_000,
                        "pose_frame": 1,
                        "velocity_frame": 1,
                        "position": [1.0, 2.0, 3.0],
                        "velocity": [4.0, 5.0, 6.0],
                    },
                }))
            (session / "samples.jsonl").write_text("\n".join(rows) + "\n", encoding="utf-8")
            observed = _runtime_observability(session)

        for stream in ("external_odometry", "px4_odometry"):
            self.assertEqual(observed["streams"][stream]["position_series"][0]["x"], 2.0)
            self.assertEqual(observed["streams"][stream]["position_series"][0]["y"], 1.0)
            self.assertEqual(observed["streams"][stream]["position_series"][0]["z"], -3.0)
            self.assertEqual(observed["streams"][stream]["velocity_series"][0]["vx"], 5.0)
            self.assertEqual(observed["streams"][stream]["velocity_series"][0]["vy"], 4.0)
            self.assertEqual(observed["streams"][stream]["velocity_series"][0]["vz"], -6.0)
        self.assertEqual(observed["coordinate_contract"]["display_frame"], "ENU (x=east, y=north, z=up)")

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

    def test_observation_complete_is_not_an_acceptance_pass(self) -> None:
        result = _evaluation(
            {"verdict": "OBSERVATION_COMPLETE"},
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
        self.assertEqual(result["gates"]["runtime_contract"], "N/A")
        self.assertEqual(result["overall"], "INCOMPLETE")

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
