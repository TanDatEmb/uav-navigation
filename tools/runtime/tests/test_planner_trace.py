import sys
from pathlib import Path
import unittest

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

from planner_trace import collect_planner_trace_records, normalize_planner_trace_record


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
                                    "commit_decision": "4",
                                    "solve_stage": "5",
                                    "solve_stage_name": "backup",
                                    "planning_latency_ms": "8.25",
                                    "planning_total_us": "8250",
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
                                    "exp_lbfgs_attempt_count": "3",
                                    "exp_retry_count": "2",
                                    "exp_retry_violation_mask": "5",
                                    "exp_retry_stop_reason": "5",
                                    "exp_lbfgs_first_return_code": "0",
                                    "exp_lbfgs_last_return_code": "-1",
                                    "exp_lbfgs_cancelled": "0",
                                    "exp_initial_normalized_dynamic_violation": "1.25",
                                    "exp_best_normalized_dynamic_violation": "1.01",
                                    "exp_final_normalized_dynamic_violation": "1.01",
                                    "exp_initial_duration_s": "4.25",
                                    "exp_final_duration_s": "4.75",
                                    "exp_retry_duration_lower_bound_min_s": "1.25",
                                    "exp_retry_duration_lower_bound_max_s": "2.50",
                                    "exp_retry_free_duration_seed_min_s": "0.0625",
                                    "exp_retry_free_duration_seed_max_s": "0.125",
                                    "guide_path_length_m": "18.5",
                                    "guide_duration_s": "4.0",
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
        self.assertEqual(records[0]["commit_decision"], 4)
        self.assertFalse(records[0]["solve_deadline_exceeded"])
        self.assertTrue(records[0]["command_available"])
        self.assertEqual(records[0]["planning_total_us"], 8250.0)
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
        self.assertEqual(records[0]["exp_lbfgs_attempt_count"], 3)
        self.assertEqual(records[0]["exp_retry_count"], 2)
        self.assertEqual(records[0]["exp_retry_violation_mask"], 5)
        self.assertEqual(records[0]["exp_retry_stop_reason"], 5)
        self.assertEqual(records[0]["exp_lbfgs_first_return_code"], 0)
        self.assertEqual(records[0]["exp_lbfgs_last_return_code"], -1)
        self.assertFalse(records[0]["exp_lbfgs_cancelled"])
        self.assertEqual(records[0]["exp_best_normalized_dynamic_violation"], 1.01)
        self.assertEqual(records[0]["exp_initial_duration_s"], 4.25)
        self.assertEqual(records[0]["exp_final_duration_s"], 4.75)
        self.assertEqual(records[0]["exp_retry_duration_lower_bound_min_s"], 1.25)
        self.assertEqual(records[0]["exp_retry_duration_lower_bound_max_s"], 2.5)
        self.assertEqual(records[0]["exp_retry_free_duration_seed_min_s"], 0.0625)
        self.assertEqual(records[0]["exp_retry_free_duration_seed_max_s"], 0.125)
        self.assertEqual(records[0]["guide_path_length_m"], 18.5)
        self.assertEqual(records[0]["guide_duration_s"], 4.0)
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


if __name__ == "__main__":
    unittest.main()
