from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from external_mode_scenario import ExternalModeScenario
from evaluation import reduce_lifecycle


class DirectCommitLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.scenario = ExternalModeScenario.__new__(ExternalModeScenario)
        self.events = []
        self.scenario._record_lifecycle = lambda phase, disposition, **fields: (
            self.events.append((phase, disposition, fields)))
        self.values = {
            "desired_request_id": "3", "desired_epoch": "4",
            "planning_localization_epoch": "7", "candidate_generation": "11",
            "planning_cycle_id": "29", "runtime_admission_attempted": "1",
            "runtime_admission_succeeded": "1",
            "runtime_admission_disposition": "IMMEDIATELY_COMMITTED",
        }

    def test_exact_direct_commit_is_activation_evidence(self):
        self.scenario._record_planner_trace_lifecycle(self.values, 123)
        self.assertEqual([event[0] for event in self.events],
                         ["request", "export", "activate"])
        activation = self.events[-1][2]
        self.assertEqual(activation["bundle_owner_cycle_id"], 29)
        self.assertEqual(activation["bundle_generation"], 11)
        self.assertEqual(activation["disposition_source"],
                         "navigation_runtime/commit_planner_candidate")

    def test_lifecycle_event_does_not_inherit_previous_command_identity(self):
        recorder = ExternalModeScenario.__new__(ExternalModeScenario)
        recorded = []
        recorder._record = lambda kind, payload: recorded.append((kind, payload))
        recorder.runtime_instance_id = "runtime-a"
        recorder.session_id = "session-a"
        recorder.latest_pva_command = {
            "request_id": 4, "goal_epoch": 8, "bundle_generation": 19,
            "sample_id": 40, "causal_planning_cycle_id": 12,
        }
        recorder.latest_goal = {"request_id": 5}
        recorder._record_lifecycle(
            "request", "PUBLISHED", request_id=6,
            request_boundary="goal_publisher",
        )
        event = recorded[0][1]
        self.assertEqual(event["request_id"], 6)
        for stale_field in ("goal_epoch", "bundle_generation", "sample_id",
                            "causal_planning_cycle_id"):
            self.assertNotIn(stale_field, event)

    def test_staged_candidate_does_not_fabricate_activation(self):
        self.values["runtime_admission_disposition"] = "STAGED_PENDING_ACTIVATION"
        self.scenario._record_planner_trace_lifecycle(self.values, 123)
        self.assertEqual([event[0] for event in self.events], ["request", "export"])

    def test_failed_candidate_does_not_fabricate_activation(self):
        self.values["runtime_admission_succeeded"] = "0"
        self.scenario._record_planner_trace_lifecycle(self.values, 123)
        self.assertEqual([event[0] for event in self.events], ["request", "export"])

    def test_direct_commit_witness_closes_exact_reference_lineage(self):
        self.scenario._record_planner_trace_lifecycle(self.values, 123)
        common = {"runtime_instance_id": "runtime-a", "session_id": "session-a",
                  "localization_epoch": 7, "goal_epoch": 4, "request_id": 3,
                  "bundle_generation": 11, "sample_id": 1}
        events = [dict(common, phase=phase, disposition=disposition, **fields)
                  for phase, disposition, fields in self.events]
        events.extend([
            dict(common, phase="authorize", disposition="AUTHORIZED",
                 authorization_boundary="execution_timeline_publish_if_current",
                 authorization_steady_ns=7, world_generation=2,
                 world_revision=4, world_observation_stamp_ns=123),
            dict(common, phase="publish", disposition="OBSERVED",
                 attribution="px4_input_trace", adapter_trace_sequence=9,
                 setpoint_kind="tracking", world_generation=2,
                 world_revision=4, world_observation_stamp_ns=123),
        ])
        reduced = reduce_lifecycle(events)
        self.assertEqual(reduced["status"], "VALID")
        self.assertIn([3, 11, 1], reduced["valid_reference_ids"])


if __name__ == "__main__":
    unittest.main()
