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


if __name__ == "__main__":
    unittest.main()
