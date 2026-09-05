import json
import csv
from pathlib import Path
import sys
import tempfile
import unittest

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

import analyze_e5_tracking_root_cause as e5


class E5TrackingRootCauseTest(unittest.TestCase):
    def test_frame_name_without_declared_convention_is_unknown(self):
        self.assertIsNone(e5.frame_kind("lio_odom"))
        self.assertEqual(e5.frame_kind("lio_odom", "ENU"), "ENU")

    def test_exact_metadata_sample_does_not_inspect_next_reset(self):
        series = [(1_000, (1, 0)), (2_000, (2, 0))]
        self.assertEqual(e5.metadata_at(series, 1_000, 1_000), (1, 0))
        self.assertIsNone(e5.metadata_at(series, 1_500, 1_000))
        self.assertFalse(e5.continuity_continuous(series))

    def test_source_timestamp_rollback_is_retained_as_an_error(self):
        records = [
            {"stream": "local_position", "payload": {"timestamp_us": 20, "x_ned_m": 0, "y_ned_m": 0, "z_ned_m": 0}},
            {"stream": "local_position", "payload": {"timestamp_us": 10, "x_ned_m": 0, "y_ned_m": 0, "z_ned_m": 0}},
        ]
        self.assertIn("source_timestamp_regression", e5.stream_series(records, "local_position")["_errors"])

    def test_malformed_timestamps_and_mode_rows_are_ignored_without_crash(self):
        malformed = [{"stream": "local_position", "payload": {"timestamp_us": "bad", "x_ned_m": 0}}]
        self.assertNotIn("position", e5.stream_series(malformed, "local_position"))
        self.assertEqual(e5.mode_at([{"timestamp_ns": "bad", "external_mode_state_name": "TRACK"}], 10), e5.MISSING)

    def test_malformed_metadata_shapes_are_insufficient_without_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "metadata.json").write_text(json.dumps({
                "experiment_id": "malformed", "run_id": root.name,
                "environment": "not-an-object", "mission_planning": ["bad"]}), encoding="utf-8")
            (root / "scenario_config.yaml").write_text("[]\n", encoding="utf-8")
            (root / "samples.jsonl").write_text("", encoding="utf-8")
            (root / "scenario.jsonl").write_text("", encoding="utf-8")
            result = e5.analyze(root)
            self.assertEqual(result["evidence_status"], "INSUFFICIENT_EVIDENCE")
            self.assertTrue(result["evidence_status_reasons"])

    def test_missing_scope_and_malformed_trace_are_insufficient(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "metadata.json").write_text(json.dumps({"experiment_id": "synthetic", "run_id": root.name}), encoding="utf-8")
            (root / "scenario_config.yaml").write_text("scenario: {}\nruntime:\n  thresholds: {}\n", encoding="utf-8")
            (root / "samples.jsonl").write_text("not-json\n", encoding="utf-8")
            (root / "scenario.jsonl").write_text("", encoding="utf-8")
            result = e5.analyze(root)
            self.assertEqual(result["evidence_status"], "INSUFFICIENT_EVIDENCE")
            self.assertTrue(any("malformed JSON" in reason for reason in result["evidence_status_reasons"]))
            self.assertIsNone(e5.retained_tracking_threshold(root))
            self.assertEqual(result["T_cross_ns"], e5.MISSING)
            self.assertTrue(all(item["status"] == "INSUFFICIENT_EVIDENCE" for item in result["h8"].values()))

    def test_conflicting_retained_thresholds_are_not_promoted(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "metadata.json").write_text(json.dumps({"tracking_certificate_threshold_m": 0.25}), encoding="utf-8")
            value, provenance, errors = e5.retained_tracking_threshold_info(
                root, [{"payload": {"retained_tracking_limit_m": 0.30}}], [])
            self.assertIsNone(value)
            self.assertEqual(len(provenance), 2)
            self.assertTrue(any("conflicting retained tracking thresholds" in error for error in errors))

    def test_crossing_requires_threshold_on_each_aligned_command_row(self):
        rows = [
            {"timestamp_ns": 1_000, "aligned_LIO_tracking_error_m": 0.1,
             "retained_tracking_limit_m": 0.25},
            {"timestamp_ns": 2_000, "aligned_LIO_tracking_error_m": 0.3,
             "retained_tracking_limit_m": e5.MISSING},
        ]
        self.assertIsNone(e5.threshold_cross(rows, 0.25))
        rows[1]["retained_tracking_limit_m"] = 0.25
        self.assertEqual(e5.threshold_cross(rows, 0.25), 1_750)

    def test_derived_temporal_csv_is_not_threshold_authority(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "e5_temporal_alignment.csv").write_text(
                "retained_tracking_limit_m\n0.25\n", encoding="utf-8")
            value, provenance, errors = e5.retained_tracking_threshold_info(root, [], [])
            self.assertIsNone(value)
            self.assertEqual(provenance, [])
            self.assertEqual(errors, [])

    def test_command_boundary_hypothesis_keeps_own_evidence_without_effective_setpoint(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            metadata = {
                "experiment_id": "synthetic-boundary", "run_id": root.name,
                "requested_cruise_speed_mps": 1.0, "environment": {"map_profile": "synthetic"},
                "common_source_clock": "ros_sim_ns",
                "frame_conventions": {"propagated_odometry": {
                    "frame_id": "lio_odom", "child_frame_id": "base_link",
                    "position_convention": "ENU", "velocity_convention": "BODY_FLU"}},
                "generation_boundary_continuity": {
                    "max_delta_position_m": 0.1, "max_delta_velocity_mps": 0.1,
                    "max_delta_acceleration_mps2": 0.1, "max_delta_jerk_mps3": 0.1},
                "px4_to_ros_mapping": {"status": "VALID", "source_clock": "px4_boot_us",
                    "target_clock": "ros_sim_ns", "timestamp_relation": "numeric_identity",
                    "scale_to_ros_ns": 1.0, "offset_ns": 0.0},
            }
            (root / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
            (root / "scenario_config.yaml").write_text(
                "scenario:\n  map_profile: synthetic\n  map_scene: test\n  route: route-a\n  expected_max_velocity_mps: 1.0\nruntime:\n  thresholds:\n    maximum_synchronization_tolerance_ms: 20\n", encoding="utf-8")
            (root / "resolved_mission.yaml").write_text("mission:\n  id: route-a\n", encoding="utf-8")
            pva = []
            samples = []
            for stamp, generation, x in ((1_000_000_000, 1, 0.0), (1_010_000_000, 2, 1.0)):
                pva.append({"kind": "pva_command", "sim_time_ns": stamp, "payload": {
                    "trajectory_generation": generation, "analytic_sample_role": 0,
                    "position": [x, 0.0, 0.0], "velocity": [1.0, 0.0, 0.0],
                    "acceleration": [0.0, 0.0, 0.0], "jerk": [0.0, 0.0, 0.0]}})
                samples.extend([
                    {"stream": "propagated_odometry", "payload": {"stamp_ns": stamp,
                        "source_clock": "ros_sim_ns", "frame_id": "lio_odom", "child_frame_id": "base_link",
                        "frame_convention": "ENU", "child_frame_convention": "BODY_FLU",
                        "position": [0.0, 0.0, 0.0], "linear_velocity": [1.0, 0.0, 0.0],
                        "q_xyzw": [0.0, 0.0, 0.0, 1.0], "localization_epoch": 1}},
                    {"stream": "local_position", "payload": {"timestamp_us": stamp // 1000,
                        "source_clock": "px4_boot_us", "x_ned_m": 0.0, "y_ned_m": 0.0, "z_ned_m": 0.0,
                        "vx_ned_m_s": 1.0, "vy_ned_m_s": 0.0, "vz_ned_m_s": 0.0}},
                ])
            (root / "scenario.jsonl").write_text("\n".join(json.dumps(x) for x in pva) + "\n", encoding="utf-8")
            (root / "samples.jsonl").write_text("\n".join(json.dumps(x) for x in samples) + "\n", encoding="utf-8")
            (root / "navigation_mode_status.csv").write_text(
                "timestamp_ns,external_mode_state_name\n1000000000,TRACK_TRAJECTORY\n1010000000,TRACK_TRAJECTORY\n",
                encoding="utf-8")
            samples.append({"stream": "mapping_diagnostics", "timestamp_ns": 1_010_000_000,
                            "payload": {"statuses": [{"name": "navigation_runtime/planner",
                            "message": "DECISION_TRACE", "values": {
                                "candidate_result": 1, "emergency_authorization_reason": 1}}]}})
            (root / "samples.jsonl").write_text("\n".join(json.dumps(x) for x in samples) + "\n", encoding="utf-8")
            (root / "e5_temporal_alignment.csv").write_text(
                "time_aligned_anchor_error_m,raw_anchor_error_m,tracking_certificate_exceeded\n"
                "999,999,true\n", encoding="utf-8")
            result = e5.analyze(root)
            self.assertEqual(result["evidence_status"], "INSUFFICIENT_EVIDENCE")
            self.assertEqual(result["h8"]["H8a_command_discontinuity"]["status"], "INCONCLUSIVE")
            self.assertEqual(result["h8"]["H8e_command_setpoint_interruption"]["status"], "INCONCLUSIVE")
            self.assertEqual(result["h8"]["H8c_px4_control_reshaping"]["status"], "INSUFFICIENT_EVIDENCE")
            failure_event = next(item for item in result["event_rows"] if item["event"] == "planner_failure_return")
            self.assertEqual(failure_event["aligned_LIO_tracking_error_m"], 1.0)
            self.assertEqual(failure_event.get("tracking_certificate_exceeded", e5.MISSING), e5.MISSING)

    def test_lio_continuity_failure_invalidates_all_required_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            metadata = {
                "experiment_id": "continuity", "run_id": root.name,
                "requested_cruise_speed_mps": 1.0, "environment": {"map_profile": "synthetic"},
                "common_source_clock": "ros_sim_ns",
                "frame_conventions": {"propagated_odometry": {
                    "frame_id": "lio_odom", "child_frame_id": "base_link",
                    "position_convention": "ENU", "velocity_convention": "BODY_FLU"}},
                "px4_to_ros_mapping": {"status": "VALID", "source_clock": "px4_boot_us",
                    "target_clock": "ros_sim_ns", "timestamp_relation": "numeric_identity",
                    "scale_to_ros_ns": 1.0, "offset_ns": 0.0},
            }
            (root / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
            (root / "scenario_config.yaml").write_text(
                "scenario:\n  map_profile: synthetic\n  map_scene: test\n  route: route-a\n  expected_max_velocity_mps: 1.0\n"
                "runtime:\n  thresholds:\n    maximum_synchronization_tolerance_ms: 20\n", encoding="utf-8")
            pva = []
            samples = []
            for stamp, epoch in ((1_000_000_000, 1), (1_010_000_000, 2)):
                pva.append({"kind": "pva_command", "sim_time_ns": stamp, "payload": {
                    "trajectory_generation": 1, "analytic_sample_role": 0,
                    "position": [0.0, 0.0, 0.0], "velocity": [1.0, 0.0, 0.0],
                    "acceleration": [0.0, 0.0, 0.0], "jerk": [0.0, 0.0, 0.0]}})
                samples.append({"stream": "propagated_odometry", "payload": {
                    "stamp_ns": stamp, "source_clock": "ros_sim_ns", "frame_id": "lio_odom",
                    "child_frame_id": "base_link", "frame_convention": "ENU",
                    "child_frame_convention": "BODY_FLU", "position": [0.0, 0.0, 0.0],
                    "linear_velocity": [1.0, 0.0, 0.0], "q_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "localization_epoch": epoch}})
            (root / "scenario.jsonl").write_text("\n".join(json.dumps(x) for x in pva) + "\n", encoding="utf-8")
            (root / "samples.jsonl").write_text("\n".join(json.dumps(x) for x in samples) + "\n", encoding="utf-8")
            result = e5.analyze(root)
            self.assertTrue(any("LIO epoch/reset identity is discontinuous" in x for x in result["evidence_status_reasons"]))
            with (root / "e5_tracking_root_cause.csv").open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertTrue(rows)
            self.assertTrue(all(row["LIO_evidence_valid"] == "False" for row in rows))
            self.assertTrue(all(row["required_evidence_valid"] == "False" for row in rows))

    def test_control_uses_its_own_synchronization_tolerance(self):
        def write_artifact(root, tolerance_ms, with_samples):
            metadata = {
                "experiment_id": root.name, "run_id": root.name,
                "requested_cruise_speed_mps": 1.0, "environment": {"map_profile": "synthetic"},
                "common_source_clock": "ros_sim_ns",
                "frame_conventions": {"propagated_odometry": {
                    "frame_id": "lio_odom", "child_frame_id": "base_link",
                    "position_convention": "ENU", "velocity_convention": "BODY_FLU"}},
                "px4_to_ros_mapping": {"status": "VALID", "source_clock": "px4_boot_us",
                    "target_clock": "ros_sim_ns", "timestamp_relation": "numeric_identity",
                    "scale_to_ros_ns": 1.0, "offset_ns": 0.0},
            }
            (root / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
            (root / "scenario_config.yaml").write_text(
                f"scenario:\n  map_profile: synthetic\n  map_scene: test\n  route: route-a\n  expected_max_velocity_mps: 1.0\n"
                f"runtime:\n  thresholds:\n    maximum_synchronization_tolerance_ms: {tolerance_ms}\n", encoding="utf-8")
            (root / "resolved_mission.yaml").write_text("mission:\n  id: route-a\n", encoding="utf-8")
            pva = [{"kind": "pva_command", "sim_time_ns": 1_000_000_000, "payload": {
                "trajectory_generation": 1, "analytic_sample_role": 0,
                "position": [0.0, 0.0, 0.0], "velocity": [1.0, 0.0, 0.0],
                "acceleration": [0.0, 0.0, 0.0], "jerk": [0.0, 0.0, 0.0]}}]
            (root / "scenario.jsonl").write_text(json.dumps(pva[0]) + "\n", encoding="utf-8")
            samples = []
            if with_samples:
                samples.append({"stream": "propagated_odometry", "payload": {
                    "stamp_ns": 1_000_000_000, "source_clock": "ros_sim_ns",
                    "frame_id": "lio_odom", "child_frame_id": "base_link",
                    "frame_convention": "ENU", "child_frame_convention": "BODY_FLU",
                    "position": [0.0, 0.0, 0.0], "linear_velocity": [1.0, 0.0, 0.0],
                    "q_xyzw": [0.0, 0.0, 0.0, 1.0], "localization_epoch": 1}})
                for stamp_us in (999_500, 1_000_500):
                    samples.append({"stream": "local_position", "payload": {
                        "timestamp_us": stamp_us, "source_clock": "px4_boot_us",
                        "x_ned_m": 0.0, "y_ned_m": 0.0, "z_ned_m": 0.0,
                        "vx_ned_m_s": 1.0, "vy_ned_m_s": 0.0, "vz_ned_m_s": 0.0}})
            (root / "samples.jsonl").write_text("\n".join(json.dumps(x) for x in samples) + ("\n" if samples else ""), encoding="utf-8")

        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            selected = parent / "selected"
            control = parent / "control"
            selected.mkdir(); control.mkdir()
            write_artifact(selected, 20.0, False)
            write_artifact(control, 0.5, True)
            result = e5.analyze(selected, control)
            self.assertIsNotNone(result["control"])
            self.assertEqual(result["control"]["scope_status"], "INSUFFICIENT_EVIDENCE")
            self.assertTrue(any("same-time frame/clock/continuity witness" in error for error in result["control"]["scope_errors"]))
            self.assertEqual(result["control"]["tracking_error"], e5.MISSING)

            # A same-time witness with the wrong retained frame contract must
            # remain unusable even when the selected artifact has a wider gap.
            write_artifact(control, 20.0, True)
            control_metadata = json.loads((control / "metadata.json").read_text(encoding="utf-8"))
            control_metadata["frame_conventions"]["propagated_odometry"]["frame_id"] = "wrong_frame"
            (control / "metadata.json").write_text(json.dumps(control_metadata), encoding="utf-8")
            mismatched = e5.analyze(selected, control)["control"]
            self.assertEqual(mismatched["scope_status"], "INSUFFICIENT_EVIDENCE")
            self.assertTrue(any("frame/clock/continuity witness" in error for error in mismatched["scope_errors"]))

            # A later invalid LIO row cannot be hidden after a valid first
            # calibration witness; complete control coverage is required.
            control_metadata["frame_conventions"]["propagated_odometry"]["frame_id"] = "lio_odom"
            (control / "metadata.json").write_text(json.dumps(control_metadata), encoding="utf-8")
            control_scenario = json.loads((control / "scenario.jsonl").read_text(encoding="utf-8").splitlines()[0])
            control_scenario["sim_time_ns"] = 1_001_000_000
            control_scenario["payload"]["trajectory_generation"] = 2
            with (control / "scenario.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(control_scenario) + "\n")
            control_samples = [json.loads(line) for line in (control / "samples.jsonl").read_text(encoding="utf-8").splitlines()]
            control_samples.extend([
                {"stream": "propagated_odometry", "payload": {
                    "stamp_ns": 1_001_000_000, "source_clock": "ros_sim_ns",
                    "frame_id": "lio_odom", "child_frame_id": "base_link",
                    "frame_convention": "ENU", "child_frame_convention": "BODY_FLU",
                    "position": [0.0, 0.0, 0.0], "linear_velocity": [1.0, 0.0, 0.0],
                    "q_xyzw": [0.0, 0.0, 0.0, 2.0], "localization_epoch": 1}},
                {"stream": "local_position", "payload": {
                    "timestamp_us": 1_001_000, "source_clock": "px4_boot_us",
                    "x_ned_m": 0.0, "y_ned_m": 0.0, "z_ned_m": 0.0,
                    "vx_ned_m_s": 1.0, "vy_ned_m_s": 0.0, "vz_ned_m_s": 0.0}},
            ])
            (control / "samples.jsonl").write_text("\n".join(json.dumps(x) for x in control_samples) + "\n", encoding="utf-8")
            invalid_rows = e5.analyze(selected, control)["control"]
            self.assertEqual(invalid_rows["scope_status"], "INSUFFICIENT_EVIDENCE")
            self.assertTrue(any("one or more rows lack LIO evidence" in error for error in invalid_rows["scope_errors"]))
            self.assertEqual(invalid_rows["tracking_error"], e5.MISSING)


if __name__ == "__main__":
    unittest.main()
