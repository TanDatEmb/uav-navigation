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

    def test_super_decision_trace_preserves_stage_and_deadline_fields(self) -> None:
        records = collect_planner_trace_records(
            samples=[
                {
                    "kind": "sample",
                    "t": 12.5,
                    "payload": {
                        "statuses": [
                            {
                                "name": "super_navigation/super_planner",
                                "values": {
                                    "planning_cycle_id": "42",
                                    "bundle_id": "17",
                                    "solve_generation": "23",
                                    "pinned_world_generation": "2",
                                    "pinned_world_revision": "91",
                                    "pinned_world_stamp_ns": "123456789",
                                    "candidate_result": "0",
                                    "replan_code": "-3",
                                    "solve_stage": "5",
                                    "solve_stage_name": "backup",
                                    "planning_latency_ms": "8.25",
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
        self.assertFalse(records[0]["solve_deadline_exceeded"])
        self.assertTrue(records[0]["command_available"])


if __name__ == "__main__":
    unittest.main()
