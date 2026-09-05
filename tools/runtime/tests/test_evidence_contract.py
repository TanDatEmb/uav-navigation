import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

import analyze_closed_loop_characterization as characterization
import analyze_e5_tracking_root_cause as e5
import closed_loop_characterization as recorder


def odom(stamp=1_000_000_000, position=(0.0, 0.0, 0.0), velocity=(1.0, 0.0, 0.0),
         frame="x500_mid360/odom", child="base_link", q=(0.0, 0.0, 0.7071067812, 0.7071067812), epoch=1):
    pose = SimpleNamespace(position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
                           orientation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))
    twist = SimpleNamespace(linear=SimpleNamespace(x=velocity[0], y=velocity[1], z=velocity[2]))
    nested = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000), frame_id=frame),
                             child_frame_id=child, pose=SimpleNamespace(pose=pose), twist=SimpleNamespace(twist=twist), localization_epoch=epoch)
    return nested


def lio_odom(**kwargs):
    kwargs.setdefault("frame", "lio_odom")
    kwargs.setdefault("child", "base_link")
    return odom(**kwargs)


class EvidenceContractTest(unittest.TestCase):
    @staticmethod
    def evidence_summary(mapping=None):
        return {"evidence_contract": {
            "maximum_synchronization_tolerance_ms": 20.0,
            "px4_to_ros_mapping": mapping or {"status": "MISSING"},
            "frame_conventions": {
                "ground_truth": {"position_frame_id": "x500_mid360/odom", "position_convention": "ENU",
                                  "velocity_frame_id": "base_link", "velocity_convention": "BODY_FLU",
                                  "continuity": "ros_sim_session"},
                "lio": {"position_frame_id": "lio_odom", "position_convention": "ENU",
                         "velocity_frame_id": "base_link", "velocity_convention": "BODY_FLU",
                         "continuity": "producer_epoch_or_reset"},
            },
        }}

    def test_recorder_preserves_frame_quaternion_and_epoch_metadata(self):
        message = odom()
        message.sequence = 7
        payload = recorder.Characterization._odom(message)
        self.assertEqual(payload["source_stamp_ns"], 1_000_000_000)
        self.assertEqual(payload["frame_id"], "x500_mid360/odom")
        self.assertEqual(payload["child_frame_id"], "base_link")
        self.assertEqual(payload["q_xyzw"][2], message.pose.pose.orientation.z)
        self.assertEqual(payload["epoch"], 1)
        self.assertEqual(payload["sequence"], 7)

    def test_yaw_90_body_flu_velocity_is_rotated_before_enu_to_ned(self):
        payload = recorder.Characterization._odom(odom(velocity=(1.0, 0.0, 0.0)))
        normalized, reason = characterization._aligned_odom(payload, 1_000_000_000, 20_000_000,
                                                            {"position_frame_id": "x500_mid360/odom", "position_convention": "ENU",
                                                             "velocity_frame_id": "base_link", "velocity_convention": "BODY_FLU"})
        self.assertEqual(reason, "aligned")
        self.assertAlmostEqual(normalized["velocity_ned"][0], 1.0, places=8)
        self.assertAlmostEqual(normalized["velocity_ned"][1], 0.0, places=8)

    def test_e5_stream_normalization_uses_header_convention_and_rejects_bad_quaternion(self):
        records = [{"stream": "propagated_odometry", "payload": {"stamp_ns": 1_000_000_000, "frame_id": "world", "child_frame_id": "base_link", "frame_convention": "ENU", "child_frame_convention": "BODY_FLU", "source_clock": "ros_sim_ns", "position": [0, 0, 0], "linear_velocity": [1, 0, 0], "q_xyzw": [0, 0, 0.7071067812, 0.7071067812], "localization_epoch": 1}}]
        series = e5.stream_series(records, "propagated_odometry")
        self.assertAlmostEqual(series["velocity"][0][1][0], 1.0, places=8)
        self.assertAlmostEqual(series["velocity"][0][1][1], 0.0, places=8)
        records[0]["payload"]["q_xyzw"] = [0, 0, 0, 2]
        self.assertIsNone(e5.stream_series(records, "propagated_odometry")["velocity"][0][1])

    def test_e5_alignment_rejects_large_source_gap_and_epoch_rollback(self):
        value, source, age, method = e5.interp([(1_000_000_000, [0.0, 0.0, 0.0]), (1_030_000_000, [1.0, 0.0, 0.0])], 1_015_000_000)
        self.assertIsNone(value)
        self.assertEqual(method, e5.MISSING)
        self.assertFalse(e5.epoch_continuous([(1, 1), (2, 0)]))

    def test_characterization_interpolates_asynchronous_acceleration_in_same_epoch(self):
        first = recorder.Characterization._odom(odom(stamp=1_000_000_000, position=(0, 0, 0), velocity=(0, 0, 0), child="world"))
        second = recorder.Characterization._odom(odom(stamp=1_010_000_000, position=(0, 0.01, 0), velocity=(0, 2, 0), child="world"))
        samples = [{"ground_truth": first, "lio": first}, {"ground_truth": second, "lio": second}]
        state, method = characterization._interpolated_odom(samples, "ground_truth", 1_005_000_000, 20_000_000,
                                                            {"position_frame_id": "x500_mid360/odom", "position_convention": "ENU",
                                                             "velocity_frame_id": "world", "velocity_convention": "ENU"})
        self.assertEqual(method, "interpolated")
        self.assertAlmostEqual(state["position_ned"][0], 0.005, places=8)
        self.assertAlmostEqual(state["velocity_ned"][0], 1.0, places=8)

    def test_equivalent_truth_has_zero_residual_after_body_rotation(self):
        gt = recorder.Characterization._odom(odom(velocity=(1.0, 0.0, 0.0)))
        lio = recorder.Characterization._odom(lio_odom(velocity=(1.0, 0.0, 0.0)))
        sample = {"timestamp_ns": 1_000_000_000, "reference_position_ned": [0.0, 0.0, 0.0], "reference_velocity_ned": [1.0, 0.0, 0.0], "reference_acceleration_ned": [0.0, 0.0, 0.0], "reference_jerk_ned": [0.0, 0.0, 0.0], "ground_truth": gt, "lio": lio, "px4_state": {"position_ned": [0.0, 0.0, 0.0], "receive_stamp_ns": 1_000_000_000}, "status": {"failsafe": False}}
        rows, _ = characterization._analyze_rows(self.evidence_summary(), [sample])
        # GT/LIO are source-time aligned and comparable without PX4 clock
        # provenance.  Receipt time alone must not unlock PX4 comparisons.
        self.assertTrue(rows[0]["lio_gt_pair_valid"])
        self.assertFalse(rows[0]["px4_source_valid"])
        self.assertFalse(rows[0]["px4_gt_pair_valid"])
        self.assertIsNone(rows[0]["gt_tracking_error_m"])
        self.assertAlmostEqual(rows[0]["lio_velocity_error_gt_mps"], 0.0, places=8)

    def test_stale_missing_nonfinite_and_epoch_mismatch_are_insufficient(self):
        cases = [
            ({"source_stamp_ns": 1, "position": [0, 0, 0], "velocity": [0, 0, 0], "frame_id": "world", "child_frame_id": "world", "epoch": 1}, "source_timestamp_stale_or_mismatched"),
            ({}, "missing_source_timestamp"),
            ({"source_stamp_ns": 1_000_000_000, "position": [float("nan"), 0, 0], "velocity": [0, 0, 0], "frame_id": "world", "child_frame_id": "world", "epoch": 1}, "missing_or_unknown_position_frame"),
        ]
        for payload, expected in cases:
            _, reason = characterization._aligned_odom(payload, 1_000_000_000, 20_000_000)
            self.assertEqual(reason, expected)
        gt = recorder.Characterization._odom(odom(epoch=1)); lio = recorder.Characterization._odom(lio_odom(epoch=2))
        second_lio = recorder.Characterization._odom(lio_odom(stamp=1_010_000_000, epoch=1))
        samples = [{"timestamp_ns": 1_000_000_000, "reference_position_ned": [0, 0, 0], "reference_velocity_ned": [1, 0, 0], "ground_truth": gt, "lio": lio, "px4_state": {"position_ned": [0, 0, 0], "receive_stamp_ns": 1_000_000_000}}, {"timestamp_ns": 1_010_000_000, "reference_position_ned": [0, 0, 0], "reference_velocity_ned": [1, 0, 0], "ground_truth": gt | {"source_stamp_ns": 1_010_000_000}, "lio": second_lio, "px4_state": {"position_ned": [0, 0, 0], "receive_stamp_ns": 1_010_000_000}}]
        rows, _ = characterization._analyze_rows(self.evidence_summary(), samples)
        self.assertTrue(rows[0]["lio_gt_pair_valid"])
        self.assertFalse(rows[0]["px4_gt_pair_valid"])
        self.assertFalse(rows[1]["alignment_valid"])
        self.assertEqual(rows[1]["alignment_reason"], "epoch_reset_discontinuity")

    def test_fresh_receipt_stale_or_unknown_px4_source_never_calibrates(self):
        gt = recorder.Characterization._odom(odom(velocity=(1.0, 0.0, 0.0)))
        lio = recorder.Characterization._odom(lio_odom(velocity=(1.0, 0.0, 0.0)))
        sample = {
            "timestamp_ns": 1_000_000_000,
            "reference_position_ned": [0.0, 0.0, 0.0],
            "reference_velocity_ned": [1.0, 0.0, 0.0],
            "ground_truth": gt,
            "lio": lio,
            "px4_state": {
                "position_ned": [3.0, 0.0, 0.0],
                "velocity_ned": [0.0, 0.0, 0.0],
                "receive_stamp_ns": 1_000_000_000,
                "source_stamp_ns": 900_000_000,
                "source_clock": "px4_boot_us",
            },
        }
        summary = self.evidence_summary({"status": "VALID", "source_clock": "ros_sim_ns"})
        rows, initial = characterization._analyze_rows(summary, [sample])
        self.assertFalse(rows[0]["px4_source_valid"])
        self.assertEqual(rows[0]["px4_source_reason"], "unknown_or_mismatched_px4_source_clock")
        self.assertFalse(rows[0]["px4_gt_pair_valid"])
        self.assertIsNone(initial["gt_to_px4_offset_ned"])

        stale = dict(sample)
        stale["px4_state"] = dict(sample["px4_state"], source_clock="ros_sim_ns")
        rows, initial = characterization._analyze_rows(summary, [stale])
        self.assertFalse(rows[0]["px4_source_valid"])
        self.assertEqual(rows[0]["px4_source_reason"], "stale_or_unmapped_px4_source_timestamp")
        self.assertIsNone(initial["gt_to_px4_offset_ned"])

    def test_failsafe_with_missing_alignment_is_unusable(self):
        row = {
            "segment_id": "arc-0", "segment_kind": "arc", "alignment_valid": False,
            "failsafe": True, "px4_xy_valid": "NOT_RECORDED", "lio_navigation_valid": "NOT_RECORDED",
            "gt_tracking_error_m": None,
        }
        reports = characterization._segment_reports([row], {})
        self.assertEqual(reports[0]["quality"], "UNUSABLE")

        observed_safe_but_missing_health = dict(row, failsafe=False, alignment_valid=True,
                                                 gt_tracking_error_m=0.0)
        self.assertEqual(characterization._segment_reports([observed_safe_but_missing_health], {})[0]["quality"],
                         "INSUFFICIENT_EVIDENCE")

    def test_validated_px4_mapping_allows_good_when_no_evidence_is_missing(self):
        gt = recorder.Characterization._odom(odom(velocity=(1.0, 0.0, 0.0)))
        lio = recorder.Characterization._odom(lio_odom(velocity=(1.0, 0.0, 0.0)))
        sample = {
            "timestamp_ns": 1_000_000_000, "segment_id": "arc-0", "segment_kind": "arc",
            "reference_position_ned": [0.0, 0.0, 0.0], "reference_velocity_ned": [1.0, 0.0, 0.0],
            "reference_acceleration_ned": [0.0, 0.0, 0.0], "ground_truth": gt, "lio": lio,
            "px4_state": {"position_ned": [0.0, 0.0, 0.0], "velocity_ned": [1.0, 0.0, 0.0], "source_stamp_ns": 1_000_000_000,
                          "source_clock": "ros_sim_ns", "reset_metadata": {"xy_reset_counter": 0}, "xy_valid": True, "z_valid": True,
                          "v_xy_valid": True, "v_z_valid": True},
            "px4_effective_setpoint": {"position_ned": [0.0, 0.0, 0.0], "velocity_ned": [1.0, 0.0, 0.0],
                                        "acceleration_ned": [0.0, 0.0, 0.0], "source_stamp_ns": 1_000_000_000,
                                        "source_clock": "ros_sim_ns"},
            "status": {"failsafe": False},
            "lio_diagnostics": {"status": [{"values": {"navigation_valid": "true", "status": "OK"}}]},
        }
        summary = self.evidence_summary({"status": "VALID", "source_clock": "ros_sim_ns"})
        rows, _ = characterization._analyze_rows(summary, [sample])
        reports = characterization._segment_reports(rows, summary)
        self.assertTrue(rows[0]["px4_source_valid"])
        self.assertEqual(reports[0]["quality"], "GOOD")

    def test_gt_px4_pair_does_not_crash_when_lio_is_missing(self):
        gt = recorder.Characterization._odom(odom(velocity=(1.0, 0.0, 0.0)))
        sample = {
            "timestamp_ns": 1_000_000_000, "reference_position_ned": [0.0, 0.0, 0.0],
            "reference_velocity_ned": [1.0, 0.0, 0.0], "ground_truth": gt,
            "px4_state": {"position_ned": [0.0, 0.0, 0.0], "velocity_ned": [1.0, 0.0, 0.0],
                          "source_stamp_ns": 1_000_000_000, "source_clock": "ros_sim_ns", "reset_metadata": {"xy_reset_counter": 0}},
        }
        summary = self.evidence_summary({"status": "VALID", "source_clock": "ros_sim_ns"})
        rows, _ = characterization._analyze_rows(summary, [sample])
        self.assertTrue(rows[0]["gt_pair_valid"])
        self.assertAlmostEqual(rows[0]["gt_tracking_error_m"], 0.0, places=8)
        self.assertIsNone(rows[0]["px4_lio_position_residual_m"])

    def test_gt_command_metric_survives_lio_and_px4_dropout_after_calibration(self):
        gt0 = recorder.Characterization._odom(odom(stamp=1_000_000_000, velocity=(1.0, 0.0, 0.0)))
        gt1 = recorder.Characterization._odom(odom(stamp=1_010_000_000, position=(0.01, 0.0, 0.0), velocity=(1.0, 0.0, 0.0)))
        px4 = {"position_ned": [0.0, 0.0, 0.0], "velocity_ned": [1.0, 0.0, 0.0],
               "source_stamp_ns": 1_000_000_000, "source_clock": "ros_sim_ns",
               "reset_metadata": {"xy_reset_counter": 0}}
        samples = [
            {"timestamp_ns": 1_000_000_000, "segment_id": "arc-0", "segment_kind": "arc", "reference_position_ned": [0.0, 0.0, 0.0],
             "reference_velocity_ned": [1.0, 0.0, 0.0], "ground_truth": gt0,
             "lio": recorder.Characterization._odom(lio_odom(stamp=1_000_000_000)), "px4_state": px4},
            {"timestamp_ns": 1_010_000_000, "segment_id": "arc-0", "segment_kind": "arc", "reference_position_ned": [0.0, 0.01, 0.0],
             "reference_velocity_ned": [1.0, 0.0, 0.0], "ground_truth": gt1},
        ]
        summary = self.evidence_summary({"status": "VALID", "source_clock": "ros_sim_ns"})
        rows, _ = characterization._analyze_rows(summary, samples)
        self.assertTrue(rows[0]["gt_pair_valid"])
        self.assertTrue(rows[1]["gt_pair_valid"])
        self.assertAlmostEqual(rows[1]["gt_tracking_error_m"], 0.0, places=8)
        report = characterization._segment_reports(rows, summary)[0]
        self.assertLess(report["required_evidence_coverage"], 1.0)
        self.assertEqual(report["quality"], "INSUFFICIENT_EVIDENCE")
        reasons = characterization._evidence_status_reasons(rows, [report])
        self.assertTrue(any(reason.startswith("segment arc-0: missing ") for reason in reasons))
        self.assertTrue(any("PX4 source clock/time" in reason for reason in reasons))

    def test_epoch_change_does_not_resume_good_after_one_invalid_row(self):
        summary = self.evidence_summary({"status": "VALID", "source_clock": "ros_sim_ns"})
        samples = []
        for index, epoch in enumerate((1, 2, 2)):
            stamp = 1_000_000_000 + index * 10_000_000
            samples.append({"timestamp_ns": stamp, "segment_id": "arc-0", "segment_kind": "arc",
                            "reference_position_ned": [0.0, 0.0, 0.0], "reference_velocity_ned": [1.0, 0.0, 0.0],
                            "ground_truth": recorder.Characterization._odom(odom(stamp=stamp, epoch=epoch)),
                            "lio": recorder.Characterization._odom(lio_odom(stamp=stamp, epoch=epoch)),
                            "px4_state": {"position_ned": [0.0, 0.0, 0.0], "velocity_ned": [1.0, 0.0, 0.0],
                          "source_stamp_ns": stamp, "source_clock": "ros_sim_ns", "reset_metadata": {"xy_reset_counter": 0}},
                            "status": {"failsafe": False}})
        rows, _ = characterization._analyze_rows(summary, samples)
        self.assertTrue(rows[0]["gt_pair_valid"])
        self.assertFalse(rows[1]["gt_pair_valid"])
        self.assertFalse(rows[2]["gt_pair_valid"])
        self.assertIsNone(rows[2]["gt_tracking_error_m"])
        self.assertEqual(characterization._segment_reports(rows, summary)[0]["quality"], "INSUFFICIENT_EVIDENCE")

    def test_retained_sitl_clock_witness_requires_actual_scope_and_parameter(self):
        def make_run(log_text, world="closed_loop_characterization"):
            directory = Path(tempfile.mkdtemp())
            (directory / "logs").mkdir()
            (directory / "metadata.json").write_text(json.dumps({
                "environment": {"map_profile": world},
                "build_provenance": {"status": "VALID"},
            }), encoding="utf-8")
            (directory / "scenario.json").write_text("{}", encoding="utf-8")
            (directory / "scenario.jsonl").write_text("", encoding="utf-8")
            (directory / "logs" / "px4_gazebo.log").write_text(log_text, encoding="utf-8")
            return directory

        valid_log = "\n".join((
            "World          : closed_loop_characterization",
            "Existing model : x500_mid360",
            "PX4 UXRCE_DDS_SYNCT: 0 (simulation clock authority)",
            "INFO  [gz_bridge] world: closed_loop_characterization, model: x500_mid360",
            "x + UXRCE_DDS_SYNCT [990,1877] : 0",
        )) + "\n"
        run = make_run(valid_log)
        summary, _ = characterization._read_samples(run)
        self.assertEqual(summary["clock_witness"]["status"], "VALID")
        self.assertEqual(summary["evidence_contract"]["px4_to_ros_mapping"]["source_clock"], "px4_dds_ns")
        self.assertEqual(summary["clock_witness"]["parameter_lines"], [5])

        for bad_log, reason in (
            (valid_log.replace(" : 0\n", " : 1\n"), "uxrce_dds_synct_not_zero"),
            (valid_log.replace(" : 0\n", " : 1\n", 1) + "x + UXRCE_DDS_SYNCT [990,1877] : 0\n", "conflicting_actual_uxrce_dds_synct_parameter"),
            (valid_log.replace("closed_loop_characterization", "occlusion"), "conflicting_or_wrong_sitl_scope"),
        ):
            bad = make_run(bad_log)
            summary, _ = characterization._read_samples(bad)
            self.assertEqual(summary["clock_witness"]["status"], "NOT_RECORDED")
            self.assertEqual(summary["clock_witness"]["reason"], reason)
            self.assertEqual(summary["evidence_contract"]["px4_to_ros_mapping"], characterization.MISSING)

    def test_actual_recorder_px4_dds_ns_shape_uses_explicit_numeric_identity(self):
        gt = recorder.Characterization._odom(odom(velocity=(1.0, 0.0, 0.0)))
        sample = {"timestamp_ns": 1_000_000_000, "reference_position_ned": [0.0, 0.0, 0.0],
                  "reference_velocity_ned": [1.0, 0.0, 0.0], "ground_truth": gt,
                  "px4_state": {"position_ned": [0.0, 0.0, 0.0], "velocity_ned": [1.0, 0.0, 0.0],
                                "source_stamp_ns": 1_000_000_000, "source_clock": "px4_dds_ns",
                                "reset_metadata": {"xy_reset_counter": 0}, "xy_valid": True,
                                "z_valid": True, "v_xy_valid": True, "v_z_valid": True}}
        mapping = {"status": "VALID", "source_clock": "px4_dds_ns",
                   "timestamp_relation": "numeric_identity", "scale_to_ros_ns": 1.0,
                   "offset_ns": 0.0}
        rows, _ = characterization._analyze_rows(self.evidence_summary(mapping), [sample])
        self.assertTrue(rows[0]["px4_source_valid"])
        self.assertTrue(rows[0]["px4_position_pair_valid"])
        self.assertTrue(rows[0]["px4_velocity_pair_valid"])

    def test_controller_acceleration_uses_velocity_validity_not_position_validity(self):
        gt = recorder.Characterization._odom(odom(velocity=(1.0, 0.0, 0.0)))
        sample = {
            "timestamp_ns": 1_000_000_000, "segment_id": "arc-0", "segment_kind": "arc",
            "reference_position_ned": [0.0, 0.0, 0.0], "reference_velocity_ned": [1.0, 0.0, 0.0],
            "reference_acceleration_ned": [0.5, 0.0, 0.0], "ground_truth": gt,
            "px4_state": {"position_ned": [0.0, 0.0, 0.0], "velocity_ned": [1.0, 0.0, 0.0],
                          "source_stamp_ns": 1_000_000_000, "source_clock": "ros_sim_ns",
                          "reset_metadata": {"xy_reset_counter": 0}, "xy_valid": False, "z_valid": False,
                          "v_xy_valid": True, "v_z_valid": True},
            "px4_effective_setpoint": {"position_ned": [0.0, 0.0, 0.0], "velocity_ned": [1.0, 0.0, 0.0],
                                        "acceleration_ned": [0.0, 0.0, 0.0], "source_stamp_ns": 1_000_000_000,
                                        "source_clock": "ros_sim_ns"},
        }
        summary = self.evidence_summary({"status": "VALID", "source_clock": "ros_sim_ns"})
        rows, _ = characterization._analyze_rows(summary, [sample])
        self.assertFalse(rows[0]["px4_position_pair_valid"])
        self.assertTrue(rows[0]["px4_velocity_pair_valid"])
        self.assertFalse(rows[0]["controller_position_pair_valid"])
        self.assertTrue(rows[0]["controller_acceleration_pair_valid"])
        self.assertIsNone(rows[0]["px4_effective_tracking_error_m"])
        self.assertIsNotNone(rows[0]["delta_a_px4_controller_mps2"])

    def test_read_samples_malformed_summary_or_payload_is_structured(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "scenario.json").write_text("{", encoding="utf-8")
            (root / "scenario.jsonl").write_text(json.dumps({"kind": "sample", "payload": []}) + "\nnot-json\n", encoding="utf-8")
            summary, rows = characterization._read_samples(root)
            self.assertEqual(rows, [])
            self.assertIn("malformed_or_missing_scenario_summary", summary["read_errors"])
            self.assertIn("malformed_sample_payload", summary["read_errors"])
            self.assertIn("malformed_trace_row", summary["read_errors"])

    def test_e5_empty_scope_is_insufficient_and_does_not_claim_causality(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "samples.jsonl").write_text("", encoding="utf-8")
            (root / "scenario.jsonl").write_text("", encoding="utf-8")
            result = e5.analyze(root)
            self.assertEqual(result["evidence_status"], "INSUFFICIENT_EVIDENCE")
            self.assertTrue(all(item["status"] == "INSUFFICIENT_EVIDENCE" for item in result["h8"].values()))


if __name__ == "__main__":
    unittest.main()
