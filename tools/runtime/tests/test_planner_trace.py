import sys
from pathlib import Path
import unittest

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

from planner_trace import (
    collect_planner_trace_records,
    normalize_planner_trace_record,
    planner_trace_summary,
)


class PlannerTraceTest(unittest.TestCase):
    def test_missing_ids_are_partial_and_never_inferred(self) -> None:
        record = normalize_planner_trace_record(
            {"horizon_end_arc_m": 12.0, "request_id": 99}, source="test"
        )
        self.assertIsNotNone(record)
        self.assertFalse(record["complete"])
        self.assertIsNone(record["planning_cycle_id"])
        self.assertIsNone(record["bundle_id"])

    def test_ros_trace_keeps_explicit_bundle_fields(self) -> None:
        records = collect_planner_trace_records(
            {
                "planner_trace_records": [
                    {
                        "planning_cycle_id": 7,
                        "bundle_id": 11,
                        "route_id": 4,
                        "horizon_endpoint": [10.0, 2.0, 3.0],
                        "selected_branch": 0,
                        "splice_position_residual_m": 0.01,
                    }
                ]
            }
        )
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["record_key"], [7, 11])
        self.assertEqual(records[0]["horizon_endpoint"], [10.0, 2.0, 3.0])
        self.assertEqual(records[0]["selected_branch"], 0)
        self.assertEqual(records[0]["route_id"], 4)

    def test_diagnostic_json_vectors_preserve_exact_execution_state(self) -> None:
        record = normalize_planner_trace_record(
            {
                "planning_cycle_id": "8",
                "bundle_id": "12",
                "planning_state_position": "[1.25,-2.5,3.75]",
                "planning_state_velocity": "[4.0,5.0,-6.0]",
                "commit_observed_this_cycle": "1",
                "candidate_start_position": "[1.5,-2.0,3.8]",
                "candidate_start_velocity": "[3.9,4.8,-5.9]",
                "candidate_start_acceleration": "[0.1,0.2,0.3]",
                "candidate_start_jerk": "[0.01,0.02,0.03]",
                "commit_previous_generation": "11",
                "splice_jerk_residual_mps3": "0.04",
                "splice_yaw_residual_rad": "-0.05",
                "splice_yaw_rate_residual_radps": "0.06",
            },
            source="diagnostics",
        )
        self.assertIsNotNone(record)
        self.assertEqual(record["planning_state_position"], [1.25, -2.5, 3.75])
        self.assertEqual(record["planning_state_velocity"], [4.0, 5.0, -6.0])
        self.assertTrue(record["commit_observed_this_cycle"])
        self.assertEqual(record["candidate_start_position"], [1.5, -2.0, 3.8])
        self.assertEqual(record["candidate_start_acceleration"], [0.1, 0.2, 0.3])
        self.assertEqual(record["candidate_start_jerk"], [0.01, 0.02, 0.03])
        self.assertEqual(record["commit_previous_generation"], 11)
        self.assertEqual(record["splice_jerk_residual_mps3"], 0.04)
        self.assertEqual(record["splice_yaw_residual_rad"], -0.05)
        self.assertEqual(record["splice_yaw_rate_residual_radps"], 0.06)

    def test_malformed_diagnostic_vector_is_fail_closed(self) -> None:
        record = normalize_planner_trace_record(
            {
                "planning_cycle_id": "9",
                "bundle_id": "13",
                "planning_state_position": "[1.0,NaN,3.0]",
                "commit_observed_this_cycle": "0",
            },
            source="diagnostics",
        )
        self.assertIsNotNone(record)
        self.assertIsNone(record["planning_state_position"])
        self.assertFalse(record["commit_observed_this_cycle"])
        self.assertIsNone(record["candidate_start_position"])

    def test_diagnostics_without_pair_do_not_create_fake_bundle_records(self) -> None:
        records = collect_planner_trace_records(
            samples=[
                {
                    "kind": "sample",
                    "payload": {
                        "statuses": [
                            {
                                "name": "navigation_planning/planner",
                                "values": {"horizon_arc_m": 20.0, "request_id": 3},
                            }
                        ]
                    },
                }
            ]
        )
        self.assertEqual(records, [])

    def test_planner_decision_trace_preserves_stage_and_deadline_fields(self) -> None:
        records = collect_planner_trace_records(
            samples=[
                {
                    "kind": "sample",
                    "t": 12.5,
                    "payload": {
                        "statuses": [
                            {
                                "name": "navigation_runtime/planner",
                                "values": {
                                    "planning_cycle_id": "42",
                                    "bundle_id": "17",
                                    "solve_generation": "23",
                                    "pinned_world_generation": "2",
                                    "pinned_world_revision": "91",
                                    "pinned_world_stamp_ns": "123456789",
                                    "candidate_result": "0",
                                    "replan_code": "-3",
                                    "planning_outcome": "5",
                                    "planning_failure_stage": "6",
                                    "planning_failure_reason": "16",
                                    "commit_decision": "4",
                                    "solve_stage": "5",
                                    "solve_stage_name": "backup",
                                    "planning_latency_ms": "8.25",
                                    "planning_total_us": "8250",
                                    "route_yaw_source": "0",
                                    "route_yaw_target_rad": "1.25",
                                    "route_yaw_lookahead_m": "7.5",
                                    "route_yaw_progress_arc_m": "18.0",
                                    "yaw_rate_limit_rad_s": "1.0",
                                    "yaw_acceleration_limit_rad_s2": "0.3",
                                    "candidate_maximum_yaw_rate_rad_s": "0.8",
                                    "candidate_maximum_yaw_acceleration_rad_s2": "0.25",
                                    "exp_frontend_us": "1200",
                                    "exp_opt_us": "34000",
                                    "backup_frontend_us": "900",
                                    "backup_opt_us": "5600",
                                    "backup_certificate_attempted": "1",
                                    "backup_switch_candidate_count": "4",
                                    "backup_feasible_seed_count": "3",
                                    "backup_visibility_hull_pass_count": "1",
                                    "backup_aligned_sfc_built_count": "2",
                                    "backup_aligned_hull_pass_count": "1",
                                    "backup_known_free_check_count": "2",
                                    "backup_known_free_pass_count": "1",
                                    "backup_certificate_selected": "0",
                                    "backup_last_reject_stage": "6",
                                    "backup_last_known_free_failure_code": "10",
                                    "backup_last_known_free_cell_state": "2",
                                    "backup_last_known_free_blocked_role": "1",
                                    "backup_last_known_free_first_blocked_time_s": "0.25",
                                    "backup_last_known_free_blocked_position": "[1.0,2.0,3.0]",
                                    "backup_last_seed_switch_time_s": "0.40",
                                    "backup_last_seed_duration_s": "1.25",
                                    "backup_last_seed_initial_velocity_mps": "4.0",
                                    "backup_last_seed_max_velocity_mps": "4.1",
                                    "backup_last_seed_max_acceleration_mps2": "2.1",
                                    "backup_last_seed_max_jerk_mps3": "4.0",
                                    "backup_last_seed_endpoint": "[2.0,3.0,3.0]",
                                    "exp_diagnostics_valid": "1",
                                    "exp_used_certified_seed": "1",
                                    "exp_certified_seed_failure_stage": "0",
                                    "exp_corridor_seed_build_failure_stage": "0",
                                    "exp_corridor_seed_retry_attempt_count": "2",
                                    "exp_corridor_seed_retry_build_valid_count": "1",
                                    "exp_corridor_seed_retry_last_certificate_stage": "0",
                                    "exp_corridor_seed_selected_mode": "2",
                                    "exp_corridor_seed_selected_max_duration_scale": "1.75",
                                    "exp_lbfgs_attempt_count": "3",
                                    "exp_lbfgs_evaluation_count": "173",
                                    "exp_lbfgs_first_attempt_evaluation_count": "91",
                                    "exp_lbfgs_last_attempt_evaluation_count": "42",
                                    "exp_retry_count": "2",
                                    "exp_retry_violation_mask": "5",
                                    "exp_retry_stop_reason": "5",
                                    "exp_lbfgs_first_return_code": "0",
                                    "exp_lbfgs_last_return_code": "-1",
                                    "exp_lbfgs_cancelled": "0",
                                    "exp_initial_normalized_dynamic_violation": "1.25",
                                    "exp_best_normalized_dynamic_violation": "1.01",
                                    "exp_final_normalized_dynamic_violation": "1.01",
                                    "maximum_velocity_mps": "4.95",
                                    "maximum_acceleration_mps2": "2.04",
                                    "maximum_jerk_mps3": "4.40",
                                    "exp_certified_seed_maximum_velocity_mps": "7.5",
                                    "exp_certified_seed_maximum_acceleration_mps2": "8.0",
                                    "exp_certified_seed_maximum_jerk_mps3": "42.0",
                                    "exp_initial_duration_s": "4.25",
                                    "exp_initial_minimum_piece_duration_s": "0.01",
                                    "exp_initial_maximum_piece_duration_s": "1.75",
                                    "exp_final_duration_s": "4.75",
                                    "exp_retry_duration_lower_bound_min_s": "1.25",
                                    "exp_retry_duration_lower_bound_max_s": "2.50",
                                    "exp_retry_free_duration_seed_min_s": "0.0625",
                                    "exp_retry_free_duration_seed_max_s": "0.125",
                                    "guide_path_length_m": "18.5",
                                    "guide_duration_s": "4.0",
                                    "required_lookahead_m": "7.2",
                                    "certified_lookahead_m": "6.0",
                                    "lookahead_complete": "0",
                                    "exp_retry_budget_remaining_us": "42000",
                                    "exp_nonfinite_evaluation_count": "2",
                                    "exp_first_nonfinite_stage": "5",
                                    "exp_first_nonfinite_value_mask": "9",
                                    "exp_first_nonfinite_attempt": "2",
                                    "exp_first_nonfinite_iteration": "4",
                                    "exp_first_nonfinite_min_duration_s": "0.031",
                                    "exp_first_nonfinite_max_duration_s": "1.24",
                                    "exp_first_nonfinite_cost": "inf",
                                    "exp_first_nonfinite_gradient_norm": "nan",
                                    "solve_deadline_exceeded": "0",
                                    "command_available": "1",
                                    "planner_failure_latched": "0",
                                },
                            }
                        ]
                    },
                }
            ]
        )
        self.assertEqual(len(records), 1)
        self.assertTrue(records[0]["complete"])
        self.assertEqual(records[0]["solve_generation"], 23)
        self.assertEqual(records[0]["pinned_world_generation"], 2)
        self.assertEqual(records[0]["pinned_world_revision"], 91)
        self.assertEqual(records[0]["pinned_world_stamp_ns"], 123456789)
        self.assertEqual(records[0]["solve_stage_name"], "backup")
        self.assertEqual(records[0]["replan_code"], "-3")
        self.assertEqual(records[0]["planning_outcome"], 5)
        self.assertEqual(records[0]["planning_failure_stage"], 6)
        self.assertEqual(records[0]["planning_failure_reason"], 16)
        self.assertEqual(records[0]["commit_decision"], 4)
        self.assertFalse(records[0]["solve_deadline_exceeded"])
        self.assertTrue(records[0]["command_available"])
        self.assertEqual(records[0]["planning_total_us"], 8250.0)
        self.assertEqual(records[0]["route_yaw_source"], 0)
        self.assertEqual(records[0]["route_yaw_target_rad"], 1.25)
        self.assertEqual(records[0]["route_yaw_lookahead_m"], 7.5)
        self.assertEqual(records[0]["route_yaw_progress_arc_m"], 18.0)
        self.assertEqual(records[0]["yaw_rate_limit_rad_s"], 1.0)
        self.assertEqual(records[0]["yaw_acceleration_limit_rad_s2"], 0.3)
        self.assertEqual(records[0]["candidate_maximum_yaw_rate_rad_s"], 0.8)
        self.assertEqual(
            records[0]["candidate_maximum_yaw_acceleration_rad_s2"], 0.25
        )
        self.assertEqual(records[0]["exp_frontend_us"], 1200.0)
        self.assertEqual(records[0]["exp_opt_us"], 34000.0)
        self.assertEqual(records[0]["backup_frontend_us"], 900.0)
        self.assertEqual(records[0]["backup_opt_us"], 5600.0)
        self.assertTrue(records[0]["backup_certificate_attempted"])
        self.assertEqual(records[0]["backup_switch_candidate_count"], 4)
        self.assertEqual(records[0]["backup_feasible_seed_count"], 3)
        self.assertEqual(records[0]["backup_visibility_hull_pass_count"], 1)
        self.assertEqual(records[0]["backup_aligned_sfc_built_count"], 2)
        self.assertEqual(records[0]["backup_aligned_hull_pass_count"], 1)
        self.assertEqual(records[0]["backup_known_free_check_count"], 2)
        self.assertEqual(records[0]["backup_known_free_pass_count"], 1)
        self.assertFalse(records[0]["backup_certificate_selected"])
        self.assertEqual(records[0]["backup_last_reject_stage"], 6)
        self.assertEqual(records[0]["backup_last_known_free_failure_code"], 10)
        self.assertEqual(records[0]["backup_last_known_free_blocked_position"], [1.0, 2.0, 3.0])
        self.assertEqual(records[0]["backup_last_seed_duration_s"], 1.25)
        self.assertEqual(records[0]["backup_last_seed_max_jerk_mps3"], 4.0)
        self.assertTrue(records[0]["exp_diagnostics_valid"])
        self.assertTrue(records[0]["exp_used_certified_seed"])
        self.assertEqual(records[0]["exp_certified_seed_failure_stage"], 0)
        self.assertEqual(records[0]["exp_corridor_seed_build_failure_stage"], 0)
        self.assertEqual(records[0]["exp_corridor_seed_retry_attempt_count"], 2)
        self.assertEqual(records[0]["exp_corridor_seed_retry_build_valid_count"], 1)
        self.assertEqual(
            records[0]["exp_corridor_seed_retry_last_certificate_stage"], 0
        )
        self.assertEqual(records[0]["exp_corridor_seed_selected_mode"], 2)
        self.assertEqual(
            records[0]["exp_corridor_seed_selected_max_duration_scale"], 1.75
        )
        self.assertEqual(records[0]["exp_lbfgs_attempt_count"], 3)
        self.assertEqual(records[0]["exp_retry_count"], 2)
        self.assertEqual(records[0]["exp_lbfgs_evaluation_count"], 173)
        self.assertEqual(records[0]["exp_lbfgs_first_attempt_evaluation_count"], 91)
        self.assertEqual(records[0]["exp_lbfgs_last_attempt_evaluation_count"], 42)
        self.assertEqual(records[0]["exp_retry_violation_mask"], 5)
        self.assertEqual(records[0]["exp_retry_stop_reason"], 5)
        self.assertEqual(records[0]["exp_lbfgs_first_return_code"], 0)
        self.assertEqual(records[0]["exp_lbfgs_last_return_code"], -1)
        self.assertFalse(records[0]["exp_lbfgs_cancelled"])
        self.assertEqual(records[0]["exp_best_normalized_dynamic_violation"], 1.01)
        self.assertEqual(records[0]["maximum_velocity_mps"], 4.95)
        self.assertEqual(records[0]["maximum_acceleration_mps2"], 2.04)
        self.assertEqual(records[0]["maximum_jerk_mps3"], 4.40)
        self.assertEqual(
            records[0]["exp_certified_seed_maximum_velocity_mps"], 7.5
        )
        self.assertEqual(
            records[0]["exp_certified_seed_maximum_acceleration_mps2"], 8.0
        )
        self.assertEqual(
            records[0]["exp_certified_seed_maximum_jerk_mps3"], 42.0
        )
        self.assertEqual(records[0]["exp_initial_duration_s"], 4.25)
        self.assertEqual(records[0]["exp_initial_minimum_piece_duration_s"], 0.01)
        self.assertEqual(records[0]["exp_initial_maximum_piece_duration_s"], 1.75)
        self.assertEqual(records[0]["exp_final_duration_s"], 4.75)
        self.assertEqual(records[0]["exp_retry_duration_lower_bound_min_s"], 1.25)
        self.assertEqual(records[0]["exp_retry_duration_lower_bound_max_s"], 2.5)
        self.assertEqual(records[0]["exp_retry_free_duration_seed_min_s"], 0.0625)
        self.assertEqual(records[0]["exp_retry_free_duration_seed_max_s"], 0.125)
        self.assertEqual(records[0]["guide_path_length_m"], 18.5)
        self.assertEqual(records[0]["guide_duration_s"], 4.0)
        self.assertEqual(records[0]["required_lookahead_m"], 7.2)
        self.assertEqual(records[0]["certified_lookahead_m"], 6.0)
        self.assertFalse(records[0]["lookahead_complete"])
        self.assertEqual(records[0]["exp_retry_budget_remaining_us"], 42000.0)
        self.assertEqual(records[0]["exp_nonfinite_evaluation_count"], 2)
        self.assertEqual(records[0]["exp_first_nonfinite_stage"], 5)
        self.assertEqual(records[0]["exp_first_nonfinite_value_mask"], 9)
        self.assertEqual(records[0]["exp_first_nonfinite_attempt"], 2)
        self.assertEqual(records[0]["exp_first_nonfinite_iteration"], 4)
        self.assertEqual(records[0]["exp_first_nonfinite_min_duration_s"], 0.031)
        self.assertEqual(records[0]["exp_first_nonfinite_max_duration_s"], 1.24)
        self.assertIsNone(records[0]["exp_first_nonfinite_cost"])
        self.assertIsNone(records[0]["exp_first_nonfinite_gradient_norm"])

    def test_summary_counts_explicit_certified_seed_outcomes(self) -> None:
        records = [
            normalize_planner_trace_record(
                {
                    "planning_cycle_id": index,
                    "bundle_id": index,
                    "exp_used_certified_seed": used,
                    "exp_certified_seed_failure_stage": stage,
                    "exp_corridor_seed_build_failure_stage": build_stage,
                    "exp_corridor_seed_retry_attempt_count": retry_attempts,
                    "exp_corridor_seed_retry_build_valid_count": retry_valid,
                    "exp_corridor_seed_retry_last_certificate_stage": retry_stage,
                    "exp_corridor_seed_selected_mode": selected_mode,
                },
                source="test",
            )
            for index, (
                used,
                stage,
                build_stage,
                retry_attempts,
                retry_valid,
                retry_stage,
                selected_mode,
            ) in enumerate(
                (
                    (1, 0, 0, 2, 2, 0, 2),
                    (0, 5, 0, 3, 1, 5, 0),
                    (0, 5, 3, 0, 0, 0, 0),
                    (0, 2, 0, 1, 0, 0, 0),
                ),
                start=1,
            )
        ]
        summary = planner_trace_summary([record for record in records if record])
        self.assertEqual(summary["exp_certified_seed_used_count"], 1)
        self.assertEqual(
            summary["exp_certified_seed_failure_stage_counts"],
            {"0": 1, "2": 1, "5": 2},
        )
        self.assertEqual(
            summary["exp_corridor_seed_build_failure_stage_counts"],
            {"0": 3, "3": 1},
        )
        self.assertEqual(summary["exp_corridor_seed_retry_attempt_total"], 6)
        self.assertEqual(summary["exp_corridor_seed_retry_build_valid_total"], 3)
        self.assertEqual(
            summary["exp_corridor_seed_retry_last_certificate_stage_counts"],
            {"0": 1, "5": 1},
        )
        self.assertEqual(
            summary["exp_corridor_seed_selected_mode_counts"],
            {"0": 3, "2": 1},
        )


if __name__ == "__main__":
    unittest.main()
