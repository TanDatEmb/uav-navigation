from pathlib import Path
import json
import sys
import tempfile
import unittest

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

from evaluation import (
    build_execution_segments,
    build_evidence_contract,
    evaluate_motion_quality,
    evaluate_session,
    evaluate_timing,
    load_evaluation_inputs,
    reduce_lifecycle,
)


def pva(stamp_ns, position, velocity, *, generation=1, role=0, frame="world"):
    return {
        "source_stamp_ns": stamp_ns,
        "time_basis": "source_stamp",
        "source_clock": "ros_time",
        "session_id": "session-a",
        "position": list(position),
        "velocity": list(velocity),
        "acceleration": [0.0, 0.0, 0.0],
        "jerk": [0.0, 0.0, 0.0],
        "frame_id": frame,
        "bundle_generation": generation,
        "trajectory_flag": role,
        "analytic_sample_role": role,
        "localization_epoch": 1,
        "goal_epoch": 1,
        "request_id": 1,
        "sample_id": stamp_ns,
        "executable": True,
    }


def truth(stamp_ns, position, velocity, *, frame="world", epoch=1):
    return {
        "source_stamp_ns": stamp_ns,
        "time_basis": "source_stamp",
        "source_clock": "ros_time",
        "position": list(position),
        "linear_velocity": list(velocity),
        "frame_id": frame,
        "localization_epoch": epoch,
    }


def inputs(commands, truth_rows, **scenario_overrides):
    scenario = {
        "mission_complete_observed": True,
        "outcome": "COMPLETE",
        "duration_s": 4.0,
        "mission_waypoint_count": 2,
        "waypoint_acceptance_events": [
            {"waypoint_accepted": True, "accepted_waypoint_index": 0},
            {"waypoint_accepted": True, "accepted_waypoint_index": 1},
        ],
        "collision_count": 0,
        "minimum_collision_clearance_m": 1.0,
        "truth_frame_witness": {
            "valid": True,
            "source_frame": "world",
            "target_frame": "world",
            "point_semantics": "base_position",
            "localization_epoch": 1,
            "validity_scope": "localization_epoch",
            "provenance": "synthetic_independent_ground_truth",
            "T_L_G": {
                "translation": [0.0, 0.0, 0.0],
                "rotation_matrix": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
            },
        },
        **scenario_overrides,
    }
    lifecycle = [{"phase": phase} for phase in (
        "request", "authorize", "export", "activate", "publish")]
    return {
        "scenario": scenario,
        "metadata": {},
        "config": {},
        "pva": commands,
        "streams": {"ground_truth_odometry": truth_rows},
        "planner_trace": [],
        "waypoints": [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)],
        "scenario_events": [
            {"kind": "navigation_mode_status", "payload": {
                "waypoint_accepted": True, "accepted_waypoint_index": 0,
            }}
        ],
        "lifecycle": lifecycle,
        "completeness_reasons": [],
        "capture_integrity_valid": True,
        "reference_lineage_valid": True,
        "tracking_coverage_policy": {
            "min_coverage_ratio": 0.75,
            "max_uncovered_interval_s": 0.30,
            "max_pairing_gap_s": 1.0,
        },
        "tracking_acceptance_policy": {
            "position_error_p95_max_m": 0.10,
            "position_error_max_m": 0.175,
            "velocity_error_p95_max_mps": 0.20,
            "velocity_error_max_mps": 0.35,
            "provenance": "synthetic_test_policy",
        },
        "evaluation_window": {
            "start_ns": min((item["source_stamp_ns"] for item in commands), default=1_000_000_000),
            "end_ns": max((item["source_stamp_ns"] for item in commands), default=2_000_000_000),
        },
    }


def complete_writer_stats(categories):
    total = sum(categories.values())
    return {
        "capture_complete": True,
        "submitted_records": total,
        "accepted_records": total,
        "written_records": total,
        "pending_records": 0,
        "dropped_records": 0,
        "snapshot_rejected_records": 0,
        "queue_full_drop_records": 0,
        "closed_rejected_records": 0,
        "serialization_error_count": 0,
        "write_error_count": 0,
        "records_by_category": {
            name: {
                "submitted_records": count,
                "accepted_records": count,
                "written_records": count,
                "dropped_records": 0,
            }
            for name, count in categories.items()
        },
    }


class EvaluationTest(unittest.TestCase):
    @staticmethod
    def _lifecycle(*, request=1, bundle=4, cycle=9, sample=11, disposition=None):
        common = {
            "runtime_instance_id": "runtime-a",
            "session_id": "session-a",
            "localization_epoch": 1,
            "goal_epoch": 2,
            "request_id": request,
            "causal_planning_cycle_id": cycle,
            "bundle_generation": bundle,
            "sample_id": sample,
            "world_generation": 5,
            "world_revision": 6,
            "world_observation_stamp_ns": 7,
        }
        values = []
        for phase, default in (
            ("request", "PUBLISHED"), ("authorize", "AUTHORIZED"),
            ("export", "EXPORTED"), ("activate", "ACTIVATED"),
            ("publish", "OBSERVED"),
        ):
            item = dict(common, phase=phase, disposition=disposition or default)
            if phase == "authorize":
                item.update({
                    "authorization_boundary": "execution_timeline_publish_if_current",
                    "authorization_steady_ns": 8,
                })
            if phase == "publish":
                item.update({
                    "adapter_trace_sequence": 21,
                    "setpoint_kind": "tracking",
                    "attribution": "px4_input_trace",
                })
            values.append(item)
        return values

    def test_valid_detour_is_guidance_deviation_not_mission_failure(self):
        commands = [
            pva(1_000_000_000, (0.0, 2.0, 0.0), (1.0, 0.0, 0.0)),
            pva(2_000_000_000, (1.0, 2.0, 0.0), (1.0, 0.0, 0.0)),
        ]
        result = evaluate_session(inputs(
            commands,
            [truth(1_000_000_000, (0.0, 2.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (1.0, 2.0, 0.0), (1.0, 0.0, 0.0))],
        ))
        self.assertEqual(result["dimensions"]["mission"]["status"], "PASS")
        self.assertEqual(result["metrics"]["mission_guidance_deviation_xy_m"]["p95"], 2.0)
        self.assertEqual(result["metrics"]["mission_guidance_deviation_xy_m"]["qualification_role"], "descriptive_only")

    def test_stop_go_detects_a_real_command_pause(self):
        stamps = [1_000_000_000, 1_100_000_000, 1_300_000_000, 1_500_000_000, 1_700_000_000]
        speeds = [1.0, 0.0, 0.0, 0.0, 1.0]
        commands = [pva(stamp, (index, 0.0, 0.0), (speed, 0.0, 0.0))
                    for index, (stamp, speed) in enumerate(zip(stamps, speeds))]
        motion = evaluate_motion_quality(inputs(commands, []))
        self.assertEqual(len(motion["stop_go"]["events"]), 1)
        self.assertAlmostEqual(motion["stop_go"]["events"][0]["duration_s"], 0.6)
        self.assertEqual(motion["completion_time_s"]["value"], 4.0)

    def test_role_switches_and_command_variation_report_chattering(self):
        commands = [
            pva(1_000_000_000 + index * 100_000_000, (index, 0.0, 0.0),
                (1.0 if index % 2 == 0 else -1.0, 0.0, 0.0), role=index % 2,
                generation=index + 1)
            for index in range(4)
        ]
        chattering = evaluate_motion_quality(inputs(commands, []))["chattering"]
        self.assertEqual(chattering["role_switch_count"], 3)
        self.assertGreater(chattering["command_velocity_total_variation_mps"], 0.0)

    def test_stitching_uses_planner_residuals_and_preserves_units(self):
        data = inputs([], [])
        data["planner_trace"] = [{
            "record_key": [1, 2], "cycle_id": 1, "bundle_id": 2,
            "splice_position_residual_m": 0.2,
            "splice_velocity_residual_mps": 0.3,
            "splice_acceleration_residual_mps2": 0.4,
            "splice_jerk_residual_mps3": 0.5,
        }]
        stitching = evaluate_motion_quality(data)["stitching"]["planned_splice_residual"]
        self.assertEqual(stitching["splice_position_residual_m"]["status"], "AVAILABLE")
        self.assertEqual(stitching["splice_velocity_residual_mps"]["unit"], "m/s")
        self.assertEqual(stitching["splice_jerk_residual_mps3"]["unit"], "m/s3")

    def test_timing_uses_steady_clock_and_explicit_px4_duration(self):
        data = inputs(
            [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             pva(1_100_000_000, (0.1, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [],
        )
        data["pva"][0]["observer_record_steady_ns"] = 10_000_000_000
        data["pva"][1]["observer_record_steady_ns"] = 10_010_000_000
        data["px4_input_trace"] = [{
            "trace_timestamp_ns": 1_000_000_000,
            "setpoint_update_duration_ns": 2_000_000,
        }]
        timing = evaluate_session(data)["metrics"]["timing"]
        self.assertEqual(timing["pva_observer_interarrival_ms"]["p95"], 10.0)
        self.assertEqual(timing["px4_setpoint_update_duration_ms"]["p50"], 2.0)

    @staticmethod
    def state_use_trace(**overrides):
        values = {
            "state_input_present": "true", "state_sequence": "42",
            "state_localization_epoch": "7", "localization_epoch": "7",
            "state_source_stamp_ros_ns": "1000000000",
            "state_callback_enter_ros_ns": "1010000000",
            "state_callback_enter_steady_ns": "10000000000",
            "state_lock_requested_steady_ns": "10001000000",
            "state_lock_acquired_steady_ns": "10011000000",
            "state_receive_steady_ns": "10012000000",
            "state_snapshot_ros_ns": "1020000000",
            "state_snapshot_steady_ns": "10022000000",
            "update_start_ros_ns": "1030000000",
            "update_start_steady_ns": "10025000000",
            "update_end_steady_ns": "10027000000",
        }
        values.update(overrides)
        return {"trace_values": values}

    def test_state_use_separates_mutex_wait_from_validation_and_source_age(self):
        trace = self.state_use_trace()
        timing = evaluate_timing({"px4_input_trace": [trace, trace]})["px4_state_use"]
        self.assertEqual(timing["usable_record_count"], 2)
        self.assertEqual(timing["distinct_state_count"], 1)
        expected = {
            "callback_validation_ms": 1.0, "receiver_mutex_wait_ms": 10.0,
            "post_lock_to_receive_ms": 1.0, "receive_to_snapshot_ms": 10.0,
            "snapshot_to_update_ms": 3.0, "px4_update_ms": 2.0,
            "source_age_at_callback_ms": 10.0, "source_age_at_update_ms": 30.0,
        }
        for metric, value in expected.items():
            self.assertEqual(timing["metrics"][metric]["p50"], value, metric)

    def test_state_use_legacy_missing_fields_are_not_zero_or_observer_time(self):
        trace = self.state_use_trace(state_receive_steady_ns="NOT_RECORDED")
        timing = evaluate_timing({"px4_input_trace": [
            {"trace_timestamp_ns": 100, "observer_record_steady_ns": 200}, trace,
        ]})["px4_state_use"]
        self.assertEqual(timing["status"], "NOT_EVALUABLE")
        self.assertEqual(timing["record_count"], 2)
        self.assertEqual(timing["missing_record_count"], 2)
        self.assertIsNone(timing["metrics"]["receiver_mutex_wait_ms"]["maximum"])

    def test_state_use_invalid_timestamps_keep_the_denominator(self):
        for field, value in (
            ("state_sequence", True),
            ("state_receive_steady_ns", "1.5"),
            ("state_receive_steady_ns", "0"),
            ("state_lock_acquired_steady_ns", "9999999999"),
            ("state_snapshot_steady_ns", str(1 << 63)),
        ):
            with self.subTest(field=field, value=value):
                timing = evaluate_timing({"px4_input_trace": [
                    self.state_use_trace(**{field: value}),
                ]})["px4_state_use"]
                self.assertEqual(timing["invalid_record_count"], 1)
                self.assertEqual(timing["usable_record_count"], 0)
                self.assertEqual(timing["record_count"], 1)

    def test_state_use_ros_pause_and_backward_jump_do_not_change_steady_durations(self):
        for update_ros, expected_age, backward_count in (
            ("1020000000", 20.0, 0), ("990000000", -10.0, 1),
        ):
            with self.subTest(update_ros=update_ros):
                timing = evaluate_timing({"px4_input_trace": [
                    self.state_use_trace(update_start_ros_ns=update_ros),
                ]})["px4_state_use"]
                self.assertEqual(timing["metrics"]["source_age_at_update_ms"]["p50"], expected_age)
                self.assertEqual(timing["metrics"]["receiver_mutex_wait_ms"]["p50"], 10.0)
                self.assertEqual(timing["ros_backward_interval_count"], backward_count)
                self.assertEqual(timing["negative_source_age_at_update_count"], backward_count)

    def test_state_use_uint64_identity_is_not_rounded_through_float(self):
        records = [self.state_use_trace(state_sequence=str((1 << 53) + index))
                   for index in (0, 1)]
        timing = evaluate_timing({"px4_input_trace": records})["px4_state_use"]
        self.assertEqual(timing["distinct_state_count"], 2)

    def test_state_use_epoch_mismatch_is_visible_not_a_freshness_verdict(self):
        timing = evaluate_timing({"px4_input_trace": [
            self.state_use_trace(localization_epoch="8"),
        ]})["px4_state_use"]
        self.assertEqual(timing["command_state_epoch_mismatch_count"], 1)
        self.assertEqual(timing["status"], "AVAILABLE")
        self.assertNotIn("PASS", timing.values())

    def test_internal_lio_consistency_without_independent_truth_is_not_tracking_pass(self):
        commands = [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))]
        data = inputs(commands, [], truth_frame_witness={"valid": False})
        data["streams"]["propagated_odometry"] = [
            truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), frame="lio_odom")
        ]
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE")
        self.assertIn("tracking.position_error_m source incomplete", result["dimensions"]["evidence"]["reasons"])

    def test_active_command_is_used_when_candidate_snapshot_differs(self):
        active = [
            pva(1_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            pva(2_000_000_000, (2.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ]
        data = inputs(
            active,
            [truth(1_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (2.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
        )
        data["candidate_final"] = {"position": [99.0, 0.0, 0.0]}
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "PASS")
        self.assertEqual(result["metrics"]["tracking.navigation_reference_vs_truth"]["p95"], 0.0)

    def test_tracking_error_above_explicit_policy_is_a_failure(self):
        commands = [
            pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ]
        data = inputs(commands, [
            truth(1_000_000_000, (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
            truth(2_000_000_000, (2.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        ])
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "FAIL")
        self.assertIn(
            "TRACKING_POSITION_ERROR_P95_MAX_M_EXCEEDED",
            result["dimensions"]["tracking"]["reasons"],
        )

    def test_tracking_without_acceptance_policy_is_not_evaluable(self):
        commands = [
            pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ]
        data = inputs(commands, [
            truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ])
        data.pop("tracking_acceptance_policy")
        result = evaluate_session(data)
        self.assertEqual(
            result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE"
        )
        self.assertIn(
            "TRACKING_ACCEPTANCE_POLICY_UNAVAILABLE", result["blocking_reasons"]
        )

    def test_requested_speed_outside_c0_is_ineligible(self):
        result = evaluate_session(inputs([], [], requested_cruise_speed_mps=8.0))
        self.assertFalse(result["qualification_eligible"])
        self.assertEqual(result["scope"]["requested_speed_mps"], 8.0)
        self.assertIn("SPEED_OUTSIDE_C0_SCOPE", result["qualification_reasons"])

    def test_optimistic_outcome_without_completion_event_is_not_mission_pass(self):
        data = inputs([], [])
        data["scenario"]["mission_complete_observed"] = False
        data["scenario"]["outcome"] = "COMPLETE"
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["mission"]["status"], "NOT_EVALUABLE")

    def test_epoch_change_splits_segment_and_missing_lifecycle_is_incomplete(self):
        first = pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        second = pva(1_100_000_000, (0.1, 0.0, 0.0), (1.0, 0.0, 0.0))
        second["localization_epoch"] = 2
        data = inputs([first, second], [
            truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            truth(1_100_000_000, (0.1, 0.0, 0.0), (1.0, 0.0, 0.0), epoch=2),
        ])
        data["completeness_reasons"] = ["lifecycle phase activate is missing", "clock reset witness is missing"]
        data["lifecycle"] = []
        segments = build_execution_segments(data)["reference_navigation"]
        self.assertEqual(len(segments), 2)
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["evidence"]["status"], "NOT_EVALUABLE")

    def test_contract_explains_missing_source_instead_of_zero(self):
        contract = build_evidence_contract(inputs([], []))
        tracking = next(item for item in contract["metrics"] if item["metric"] == "tracking.position_error_m")
        self.assertEqual(tracking["status"], "NOT_EVALUABLE")
        self.assertIn("source_stamp_ns", tracking["sources"][0]["missing_fields"])

    def test_contract_rejects_malformed_source_values_fail_closed(self):
        cases = (
            ("source_stamp_ns", "1000000000", "SOURCE_TIME_INVALID"),
            ("source_clock", "unknown", "SOURCE_CLOCK_UNVERIFIED"),
            ("frame_id", 7, "FRAME_INVALID"),
            ("position", [0.0, float("inf"), 0.0], "NONFINITE"),
            ("request_id", 0, "IDENTITY_INVALID"),
            ("sample_id", "sample-1", "IDENTITY_INVALID"),
        )
        for field, value, reason in cases:
            with self.subTest(field=field):
                data = inputs(
                    [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
                    [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
                )
                data["pva"][0][field] = value
                contract = build_evidence_contract(data)
                metric = next(item for item in contract["metrics"] if item["metric"] == "tracking.position_error_m")
                source = metric["sources"][0]
                self.assertEqual(metric["status"], "NOT_EVALUABLE")
                self.assertIn(field, source["invalid_fields"])
                self.assertIn(reason, source["invalid_reasons"])
                self.assertIn("tracking.position_error_m source incomplete", contract["qualification_missing"])

    def test_contract_does_not_infer_missing_source_clock_or_identity(self):
        data = inputs(
            [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
        )
        data["pva"][0].pop("source_clock")
        data["pva"][0].pop("session_id")
        contract = build_evidence_contract(data)
        metric = next(item for item in contract["metrics"] if item["metric"] == "tracking.position_error_m")
        source = metric["sources"][0]
        self.assertEqual(metric["status"], "NOT_EVALUABLE")
        self.assertIn("source_clock", source["missing_fields"])
        self.assertIn("session_id", source["missing_fields"])
        self.assertIn("tracking.position_error_m source incomplete", contract["qualification_missing"])

    def test_legacy_observer_time_is_diagnostic_only_for_tracking(self):
        data = inputs(
            [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
        )
        for row in data["pva"] + data["streams"]["ground_truth_odometry"]:
            row["time_basis"] = "observer_sim_time_legacy"
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE")
        self.assertIn("OBSERVER_TIME_ONLY", result["blocking_reasons"])

    def test_valid_boolean_without_frame_witness_content_is_not_tracking_evidence(self):
        data = inputs(
            [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            truth_frame_witness={"valid": True},
        )
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE")
        self.assertIn("FRAME_WITNESS_FIELDS_MISSING", result["blocking_reasons"])

    def test_invalid_rotation_witness_is_rejected_without_repair(self):
        data = inputs(
            [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
        )
        data["scenario"]["truth_frame_witness"]["T_L_G"]["rotation_matrix"][0][0] = -1.0
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE")
        self.assertIn("FRAME_TRANSFORM_ROTATION_INVALID", result["blocking_reasons"])

    def test_same_frame_witness_cannot_hide_nonidentity_transform(self):
        data = inputs(
            [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
        )
        data["scenario"]["truth_frame_witness"]["T_L_G"]["translation"] = [
            1.0, 0.0, 0.0,
        ]
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE")
        self.assertIn("FRAME_WITNESS_SAME_FRAME_NONIDENTITY", result["blocking_reasons"])

    def test_non_identity_truth_frame_transform_is_applied_to_tracking(self):
        commands = [
            pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), frame="lio"),
            pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), frame="lio"),
        ]
        truth_rows = [
            truth(1_000_000_000, (10.0, 0.0, 0.0), (1.0, 0.0, 0.0), frame="gazebo"),
            truth(2_000_000_000, (11.0, 0.0, 0.0), (1.0, 0.0, 0.0), frame="gazebo"),
        ]
        data = inputs(commands, truth_rows)
        data["scenario"]["truth_frame_witness"] = {
            "valid": True,
            "source_frame": "lio",
            "target_frame": "gazebo",
            "point_semantics": "base_position",
            "localization_epoch": 1,
            "validity_scope": "localization_epoch",
            "provenance": "synthetic_non_identity_transform",
            "T_L_G": {
                "translation": [-10.0, 0.0, 0.0],
                "rotation_matrix": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
            },
        }
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "PASS")
        self.assertEqual(
            result["metrics"]["tracking.navigation_reference_vs_truth"]["maximum"],
            0.0,
        )

    def test_mixed_measured_frames_cannot_share_one_frame_witness(self):
        data = inputs(
            [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), frame="other")],
        )
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE")
        self.assertIn("FRAME_WITNESS_MISMATCH", result["blocking_reasons"])

    def test_partial_coverage_keeps_diagnostic_value_but_not_qualification(self):
        commands = [
            pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            pva(3_000_000_000, (2.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ]
        data = inputs(commands, [
            truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ])
        data["evaluation_window"] = {"start_ns": 1_000_000_000, "end_ns": 3_000_000_000}
        metric = evaluate_session(data)["metrics"]["tracking.navigation_reference_vs_truth"]
        self.assertEqual(metric["status"], "AVAILABLE")
        self.assertAlmostEqual(metric["coverage_ratio"], 0.5)
        self.assertEqual(metric["qualification_checks"]["coverage_sufficient"], False)

    def test_zero_minimum_coverage_policy_is_invalid(self):
        data = inputs(
            [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
        )
        data["tracking_coverage_policy"]["min_coverage_ratio"] = 0.0
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE")
        self.assertIn("TRACKING_COVERAGE_POLICY_INVALID", result["blocking_reasons"])

    def test_source_timestamp_regression_is_not_hidden_by_metric_sorting(self):
        data = inputs(
            [pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
        )
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE")
        self.assertIn("SOURCE_TIMESTAMP_REGRESSION", result["blocking_reasons"])

    def test_missing_source_clock_is_not_synthesized(self):
        data = inputs(
            [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
        )
        for row in data["streams"]["ground_truth_odometry"]:
            row.pop("source_clock")
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE")
        self.assertIn("CLOCK_RELATION_UNVERIFIED", result["blocking_reasons"])

    def test_unmapped_source_clock_domains_cannot_be_compared(self):
        data = inputs(
            [pva(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             pva(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
            [truth(1_000_000_000, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
             truth(2_000_000_000, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))],
        )
        for row in data["streams"]["ground_truth_odometry"]:
            row["source_clock"] = "independent_clock"
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["tracking"]["status"], "NOT_EVALUABLE")
        self.assertIn("CLOCK_RELATION_UNVERIFIED", result["blocking_reasons"])

    def test_capture_false_with_zero_counters_is_not_complete(self):
        data = inputs([], [])
        data["capture_integrity_valid"] = False
        result = evaluate_session(data)
        self.assertEqual(result["evidence_status"], "INCOMPLETE")
        self.assertIn("CAPTURE_NOT_FINALIZED", result["blocking_reasons"])

    def test_missing_capture_witness_defaults_fail_closed(self):
        data = inputs([], [])
        data.pop("capture_integrity_valid")
        result = evaluate_session(data)
        self.assertEqual(result["evidence_status"], "INCOMPLETE")
        self.assertFalse(result["qualification_eligible"])
        self.assertIn("CAPTURE_NOT_FINALIZED", result["blocking_reasons"])

    def test_loaded_complete_writer_stats_are_not_marked_unfinalized(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            scenario_writer_stats = complete_writer_stats({"event": 1})
            monitor_writer_stats = complete_writer_stats({})
            (session / "scenario.json").write_text(json.dumps({
                "terminal_outcome": "COMPLETE",
                "evidence_writer": scenario_writer_stats,
            }) + "\n", encoding="utf-8")
            (session / "monitor.json").write_text(json.dumps({
                "evidence_writer": monitor_writer_stats,
            }) + "\n", encoding="utf-8")
            (session / "scenario.jsonl").write_text(json.dumps({
                "kind": "event",
                "payload": {"name": "scenario_finished", "terminal_marker": True},
            }) + "\n", encoding="utf-8")
            (session / "samples.jsonl").write_text("", encoding="utf-8")
            loaded = load_evaluation_inputs(session, {})
            self.assertTrue(loaded["capture_integrity_valid"])
            self.assertNotIn("SCENARIO_CAPTURE_NOT_FINALIZED", loaded["completeness_reasons"])
            self.assertNotIn("MONITOR_CAPTURE_NOT_FINALIZED", loaded["completeness_reasons"])
            evaluated = evaluate_session(loaded)
            self.assertNotIn("CAPTURE_NOT_FINALIZED", evaluated["blocking_reasons"])

    def test_missing_or_malformed_writer_counters_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            malformed = {"capture_complete": True, "submitted_records": "bogus"}
            (session / "scenario.json").write_text(json.dumps({
                "terminal_outcome": "COMPLETE", "evidence_writer": malformed,
            }) + "\n", encoding="utf-8")
            (session / "monitor.json").write_text(json.dumps({
                "evidence_writer": malformed,
            }) + "\n", encoding="utf-8")
            (session / "scenario.jsonl").write_text(json.dumps({
                "kind": "event", "payload": {"name": "scenario_finished"},
            }) + "\n", encoding="utf-8")
            (session / "samples.jsonl").write_text("", encoding="utf-8")
            loaded = load_evaluation_inputs(session, {})
            self.assertFalse(loaded["capture_integrity_valid"])
            self.assertIn(
                "SCENARIO_WRITER_COUNTER_INVALID", loaded["completeness_reasons"]
            )

    def test_malformed_jsonl_and_pva_are_incomplete_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            scenario_writer = complete_writer_stats({"pva_command": 1, "event": 1})
            monitor_writer = complete_writer_stats({})
            (session / "scenario.json").write_text(json.dumps({
                "terminal_outcome": "COMPLETE", "evidence_writer": scenario_writer,
            }) + "\n", encoding="utf-8")
            (session / "monitor.json").write_text(json.dumps({
                "evidence_writer": monitor_writer,
            }) + "\n", encoding="utf-8")
            (session / "scenario.jsonl").write_text(
                '{"kind":"pva_command","payload":{}}\n{bad json\n',
                encoding="utf-8",
            )
            (session / "samples.jsonl").write_text("", encoding="utf-8")
            loaded = load_evaluation_inputs(session, {})
            self.assertIn("PVA_NORMALIZATION_REJECTED", loaded["completeness_reasons"])
            self.assertTrue(any(
                reason.startswith("MALFORMED_JSONL:scenario:")
                for reason in loaded["completeness_reasons"]
            ))
            self.assertFalse(loaded["capture_integrity_valid"])

    def test_px4_trace_drop_and_publish_error_make_capture_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            scenario_writer_stats = complete_writer_stats({
                "px4_input_trace": 1, "event": 1,
            })
            monitor_writer_stats = complete_writer_stats({})
            (session / "scenario.json").write_text(json.dumps({
                "terminal_outcome": "COMPLETE",
                "evidence_writer": scenario_writer_stats,
            }) + "\n", encoding="utf-8")
            (session / "monitor.json").write_text(json.dumps({
                "evidence_writer": monitor_writer_stats,
            }) + "\n", encoding="utf-8")
            events = [
                {"kind": "px4_input_trace", "payload": {
                    "trace_sequence": 1,
                    "trace_drop_count": 2,
                    "trace_publish_error_count": 1,
                }},
                {"kind": "event", "payload": {
                    "name": "scenario_finished", "terminal_marker": True,
                }},
            ]
            (session / "scenario.jsonl").write_text(
                "\n".join(json.dumps(item) for item in events) + "\n",
                encoding="utf-8",
            )
            (session / "samples.jsonl").write_text("", encoding="utf-8")
            loaded = load_evaluation_inputs(session, {})
            self.assertIn("PX4_INPUT_TRACE_QUEUE_DROP", loaded["completeness_reasons"])
            self.assertIn("PX4_INPUT_TRACE_PUBLISH_ERROR", loaded["completeness_reasons"])
            self.assertFalse(loaded["capture_integrity_valid"])

    def test_px4_trace_sequence_gap_is_explicitly_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            scenario_writer_stats = complete_writer_stats({
                "px4_input_trace": 2, "event": 1,
            })
            monitor_writer_stats = complete_writer_stats({})
            summary = {
                "terminal_outcome": "COMPLETE",
                "evidence_writer": scenario_writer_stats,
            }
            (session / "scenario.json").write_text(
                json.dumps(summary) + "\n", encoding="utf-8"
            )
            (session / "monitor.json").write_text(json.dumps({
                "evidence_writer": monitor_writer_stats,
            }) + "\n", encoding="utf-8")
            events = [
                {"kind": "px4_input_trace", "payload": {"trace_sequence": sequence}}
                for sequence in (10, 12)
            ]
            events.append({"kind": "event", "payload": {
                "name": "scenario_finished", "terminal_marker": True,
            }})
            (session / "scenario.jsonl").write_text(
                "\n".join(json.dumps(item) for item in events) + "\n",
                encoding="utf-8",
            )
            (session / "samples.jsonl").write_text("", encoding="utf-8")
            loaded = load_evaluation_inputs(session, {})
            self.assertIn("PX4_INPUT_TRACE_SEQUENCE_GAP", loaded["completeness_reasons"])
            self.assertFalse(loaded["capture_integrity_valid"])

    def test_rejected_required_samples_keep_their_recorder_denominator(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            scenario_writer_stats = complete_writer_stats({"event": 1})
            monitor_writer_stats = complete_writer_stats({
                "sample:ground_truth_odometry": 1,
            })
            (session / "scenario.json").write_text(json.dumps({
                "terminal_outcome": "COMPLETE",
                "evidence_writer": scenario_writer_stats,
            }) + "\n", encoding="utf-8")
            (session / "monitor.json").write_text(json.dumps({
                "evidence_writer": monitor_writer_stats,
            }) + "\n", encoding="utf-8")
            (session / "scenario.jsonl").write_text(json.dumps({
                "kind": "event",
                "payload": {"name": "scenario_finished", "terminal_marker": True},
            }) + "\n", encoding="utf-8")
            (session / "samples.jsonl").write_text(json.dumps({
                "kind": "sample",
                "stream": "ground_truth_odometry",
                "accepted_by_monitor": False,
                "timestamp_ns": 0,
                "payload": {},
            }) + "\n", encoding="utf-8")
            loaded = load_evaluation_inputs(session, {})
            counters = loaded["monitor_sample_counts"]["ground_truth_odometry"]
            self.assertEqual(counters["recorded_rows"], 1)
            self.assertEqual(counters["monitor_rejected_rows"], 1)
            self.assertEqual(counters["normalized_rows"], 0)
            self.assertIn(
                "MONITOR_REJECTED_REQUIRED_SAMPLE:ground_truth_odometry",
                loaded["completeness_reasons"],
            )

    def test_known_collision_is_not_erased_by_incomplete_evidence(self):
        data = inputs([], [], collision_count=1)
        data["capture_integrity_valid"] = False
        result = evaluate_session(data)
        self.assertEqual(result["dimensions"]["safety"]["status"], "FAIL")
        self.assertEqual(result["assessment_status"], "FAIL")
        self.assertEqual(result["evidence_status"], "INCOMPLETE")
        self.assertFalse(result["qualification_eligible"])

    def test_lifecycle_reducer_does_not_join_request_a_to_request_b(self):
        events = self._lifecycle(request=1)
        events[1:] = [dict(item, request_id=2, bundle_owner_request_id=2) for item in events[1:]]
        reduced = reduce_lifecycle(events)
        self.assertEqual(reduced["valid_transaction_count"], 0)
        self.assertIn("LIFECYCLE_PHASE_INCOMPLETE:request", reduced["reasons"])

    def test_reordered_causal_events_reduce_to_one_valid_transaction(self):
        events = list(reversed(self._lifecycle()))
        reduced = reduce_lifecycle(events)
        self.assertEqual(reduced["status"], "VALID")
        self.assertEqual(reduced["valid_transaction_count"], 1)
        self.assertTrue(reduced["order_independent"])

    def test_later_revalidation_cycle_does_not_relabel_bundle_owner(self):
        events = self._lifecycle(bundle=4, cycle=9, sample=11)
        for event in events:
            if event["phase"] in {"authorize", "publish"}:
                event["causal_planning_cycle_id"] = 14
                # Legacy captures incorrectly copied the validation cycle into
                # the owner field. The unique export witness is authoritative.
                event["bundle_owner_cycle_id"] = 14
        reduced = reduce_lifecycle(events)
        self.assertEqual(reduced["status"], "VALID")
        self.assertEqual(reduced["valid_transaction_count"], 1)
        transaction = reduced["transactions"][0]
        self.assertEqual(
            transaction["identity"]["causal_planning_cycle_id"], 9
        )
        self.assertEqual(
            transaction["events"]["authorize"]["causal_planning_cycle_id"], 14
        )
        self.assertEqual(
            transaction["events"]["authorize"]["bundle_owner_cycle_id"], 9
        )

    def test_bundle_owner_is_not_guessed_without_export_witness(self):
        events = [
            event for event in self._lifecycle(bundle=4, cycle=9, sample=11)
            if event["phase"] != "export"
        ]
        for event in events:
            if event["phase"] in {"authorize", "publish"}:
                event["causal_planning_cycle_id"] = 14
                event.pop("bundle_owner_cycle_id", None)
        reduced = reduce_lifecycle(events)
        self.assertEqual(reduced["valid_transaction_count"], 0)
        self.assertIn("BUNDLE_OWNER_MISSING_EXPORT", reduced["reasons"])

    def test_cycleless_request_cannot_be_attached_to_a_later_cycle(self):
        events = self._lifecycle()
        events[0]["causal_planning_cycle_id"] = None
        reduced = reduce_lifecycle(events)
        self.assertEqual(reduced["status"], "INCOMPLETE")
        self.assertIn("LIFECYCLE_PHASE_INCOMPLETE:request", reduced["reasons"])

    def test_incomplete_transaction_is_not_hidden_by_a_valid_transaction(self):
        valid = self._lifecycle(request=1, bundle=4, cycle=9, sample=11)
        incomplete = self._lifecycle(request=2, bundle=5, cycle=10, sample=12)[:-1]
        reduced = reduce_lifecycle(valid + incomplete)
        self.assertEqual(reduced["valid_transaction_count"], 1)
        self.assertEqual(reduced["status"], "INCOMPLETE")
        self.assertTrue(reduced["unresolved"])

    def test_multiple_adapter_updates_keep_each_trace_sequence(self):
        events = self._lifecycle(sample=11)
        second = dict(next(item for item in events if item["phase"] == "publish"), sample_id=12, adapter_trace_sequence=22)
        reduced = reduce_lifecycle(events + [second])
        transaction = reduced["transactions"][0]
        self.assertEqual(
            [item["adapter_trace_sequence"] for item in transaction["events_all"]["publish"]],
            [21, 22],
        )

    def test_each_published_sample_requires_its_own_authorization_witness(self):
        events = self._lifecycle(sample=11)
        second_authorize = dict(
            next(item for item in events if item["phase"] == "authorize"),
            sample_id=12,
        )
        second_publish = dict(
            next(item for item in events if item["phase"] == "publish"),
            sample_id=12,
            adapter_trace_sequence=22,
        )
        reduced = reduce_lifecycle(events + [second_authorize, second_publish])
        self.assertEqual(reduced["status"], "VALID")
        self.assertEqual(
            sorted(item[2] for item in reduced["valid_reference_ids"]),
            [11, 12],
        )

    def test_receiver_shape_check_cannot_replace_producer_authorization(self):
        events = self._lifecycle()
        authorize = next(item for item in events if item["phase"] == "authorize")
        authorize.pop("authorization_boundary")
        reduced = reduce_lifecycle(events)
        self.assertEqual(reduced["valid_transaction_count"], 0)
        self.assertIn("COMMAND_AUTHORIZATION_LINEAGE_MISSING", reduced["reasons"])

    def test_publish_world_must_match_the_authorized_command_world(self):
        events = self._lifecycle()
        publish = next(item for item in events if item["phase"] == "publish")
        publish["world_revision"] += 1
        reduced = reduce_lifecycle(events)
        self.assertEqual(reduced["valid_transaction_count"], 0)
        self.assertIn("COMMAND_WORLD_IDENTITY_MISMATCH", reduced["reasons"])

    def test_px4_trace_is_a_publish_observation_without_synthetic_lifecycle_event(self):
        events = [item for item in self._lifecycle() if item["phase"] != "publish"]
        px4_trace = dict(
            self._lifecycle()[4],
            attribution="px4_input_trace",
        )
        px4_trace.pop("disposition")
        reduced = reduce_lifecycle(events, [px4_trace])
        self.assertEqual(reduced["status"], "VALID")
        self.assertEqual(reduced["valid_reference_ids"], [[1, 4, 11]])

    def test_failed_activation_is_not_active_reference(self):
        events = self._lifecycle()
        events[3]["disposition"] = "FAILED"
        reduced = reduce_lifecycle(events)
        self.assertEqual(reduced["valid_transaction_count"], 0)
        self.assertIn("LIFECYCLE_ACTIVATE_NOT_SUCCESSFUL", reduced["reasons"])

    def test_rejected_request_has_terminal_transaction_without_fake_phases(self):
        events = self._lifecycle(disposition="REJECTED")[:2]
        events[0]["disposition"] = "PUBLISHED"
        reduced = reduce_lifecycle(events)
        self.assertEqual(reduced["transactions"][0]["status"], "VALID_REJECT")
        self.assertEqual(reduced["valid_transaction_count"], 0)

    def test_conflicting_dispositions_are_not_resolved_by_last_value(self):
        events = self._lifecycle()
        duplicate = dict(next(item for item in events if item["phase"] == "export"), disposition="REJECTED")
        reduced = reduce_lifecycle(events + [duplicate])
        self.assertEqual(reduced["status"], "CONFLICTING")
        self.assertIn("CONFLICTING_EVIDENCE", reduced["reasons"])

    def test_duplicate_authorization_world_conflict_is_order_independent(self):
        events = self._lifecycle()
        authorize = next(item for item in events if item["phase"] == "authorize")
        contradictory = dict(authorize, world_revision=99)
        forward = reduce_lifecycle(events + [contradictory])
        reverse = reduce_lifecycle([contradictory] + events)
        self.assertEqual(forward["status"], "CONFLICTING")
        self.assertEqual(reverse["status"], "CONFLICTING")
        self.assertEqual(forward["valid_transaction_count"], 0)
        self.assertEqual(reverse["valid_transaction_count"], 0)


if __name__ == "__main__":
    unittest.main()
