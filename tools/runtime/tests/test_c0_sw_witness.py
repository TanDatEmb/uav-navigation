"""Exact producer witnesses for the separately scoped C0-SW assessment."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluation import (  # noqa: E402
    C0_SW_POLICY_PROVENANCE, C0_SW_POLICY_VERSION,
    evaluate_software_qualification, reduce_lifecycle,
)


def writer_stats(count: int) -> dict:
    return {
        "capture_complete": True, "submitted_records": count,
        "accepted_records": count, "written_records": count,
        "pending_records": 0, "dropped_records": 0,
        "snapshot_rejected_records": 0, "queue_full_drop_records": 0,
        "closed_rejected_records": 0, "serialization_error_count": 0,
        "write_error_count": 0,
        "records_by_category": ({"lifecycle": {
            "submitted_records": count, "accepted_records": count,
            "written_records": count, "dropped_records": 0,
        }} if count else {}),
    }


def command_events() -> list[dict]:
    common = {
        "runtime_instance_id": "core-a", "session_id": "run-a",
        "localization_epoch": 7, "goal_epoch": 4, "request_id": 3,
        "bundle_generation": 11, "sample_id": 22,
        "world_generation": 2, "world_revision": 5,
        "world_observation_stamp_ns": 123,
        "bundle_owner_request_id": 3, "bundle_owner_cycle_id": 29,
        "bundle_owner_attribution": "producer_declared",
    }
    events = [
        dict(common, phase="request", disposition="PUBLISHED",
             causal_planning_cycle_id=29),
        dict(common, phase="result", disposition="OBSERVED",
             planner_status=0, planner_disposition=0,
             runtime_admission_attempted=True),
        dict(common, phase="export", disposition="EXPORTED",
             causal_planning_cycle_id=29),
        dict(common, phase="activate", disposition="ACTIVATED"),
        dict(common, phase="authorize", disposition="AUTHORIZED",
             authorization_boundary="execution_timeline_publish_if_current",
             authorization_steady_ns=99, causal_planning_cycle_id=40),
        dict(common, phase="publish", disposition="OBSERVED",
             attribution="px4_input_trace", adapter_trace_sequence=8,
             setpoint_kind="tracking", bundle_owner_cycle_id=None,
             bundle_owner_attribution=None),
    ]
    return events


def software_inputs(events: list[dict] | None = None) -> dict:
    events = command_events() if events is None else events
    reduction = reduce_lifecycle(events)
    return {
        "metadata": {
            "qualification_scope": "C0_SW",
            "qualification_policy_version": C0_SW_POLICY_VERSION,
            "qualification_policy_provenance": C0_SW_POLICY_PROVENANCE,
            "requested_cruise_speed_mps": 1.5,
            "tracking_experiment": {"mode": "off"},
            "runtime_configuration": {
                name: {"effective": {
                    "mode": "off", "enabled": False,
                    "suppress_braking": False,
                    "suppress_estimator_health_response": False,
                }} for name in ("mapping", "external_mode")
            } | {"planner_fault_injection": {"effective": {"once": False}}},
        },
        "scenario": {
            "outcome": "COMPLETE", "mission_complete_observed": True,
            "waypoint_acceptance_events": [{
                "waypoint_accepted": True, "accepted_waypoint_index": 0,
            }],
        },
        "waypoints": [(1.0, 0.0, 0.0)],
        "lifecycle": events,
        "lifecycle_reduction": reduction,
        "scenario_events": [{"kind": "lifecycle"}],
        "writer": {"scenario": writer_stats(1), "monitor": writer_stats(0)},
        "pva": [{
            "runtime_instance_id": "core-a", "session_id": "run-a",
            "localization_epoch": 7, "goal_epoch": 4, "request_id": 3,
            "bundle_generation": 11, "sample_id": 22,
            "world_generation": 2, "world_revision": 5,
            "world_observation_stamp_ns": 123, "executable": True,
        }],
    }


class C0SoftwareWitnessTest(unittest.TestCase):
    def test_exact_command_lifecycle_is_eligible_independently_of_flight_policy(self):
        data = software_inputs()
        result = evaluate_software_qualification(data)
        self.assertTrue(result["software_qualification_eligible"])
        self.assertEqual(result["required_reference_missing"], 0)
        self.assertEqual(result["axes"]["FLIGHT_PERFORMANCE"], "DEFERRED_C0_IFP")

    def test_missing_producer_owner_is_not_healed_from_export(self):
        events = command_events()
        for event in events:
            if event["phase"] == "authorize":
                event["bundle_owner_cycle_id"] = None
                event["bundle_owner_attribution"] = None
        result = evaluate_software_qualification(software_inputs(events))
        self.assertFalse(result["software_qualification_eligible"])
        self.assertEqual(result["required_reference_missing"], 1)

    def test_wrong_request_generation_localization_and_world_reject(self):
        for field, value in (
            ("request_id", 4), ("bundle_generation", 12),
            ("localization_epoch", 8), ("world_revision", 6),
        ):
            with self.subTest(field=field):
                data = software_inputs()
                data["pva"][0][field] = value
                self.assertEqual(
                    evaluate_software_qualification(data)["required_reference_missing"], 1)

    def test_missing_scope_and_writer_drop_are_not_eligible(self):
        data = software_inputs()
        data["metadata"].pop("qualification_policy_provenance")
        self.assertFalse(evaluate_software_qualification(data)["software_qualification_eligible"])

    def test_same_sample_with_conflicting_world_is_not_eligible(self):
        data = software_inputs()
        duplicate = dict(data["pva"][0], world_revision=6)
        data["pva"].append(duplicate)
        result = evaluate_software_qualification(data)
        self.assertEqual(result["required_reference_conflicting"], 1)
        self.assertFalse(result["software_qualification_eligible"])

    def test_safety_stop_label_alone_cannot_prove_gate_decision(self):
        data = software_inputs()
        data["scenario"].update(
            outcome="PAUSED_SAFETY_STOP", mission_complete_observed=False,
            mode_status_reason_name="SAFETY_STOP", px4_hold_observed=True,
        )
        result = evaluate_software_qualification(data)
        self.assertEqual(result["axes"]["PRODUCT_LOGIC"], "NOT_EVALUABLE")
        self.assertEqual(result["axes"]["FLIGHT_PERFORMANCE"], "DEFERRED_C0_IFP")
        self.assertFalse(result["software_qualification_eligible"])
        data = software_inputs()
        writer = data["writer"]["scenario"]
        writer["submitted_records"] += 1
        writer["dropped_records"] = 1
        writer["records_by_category"]["lifecycle"]["submitted_records"] += 1
        writer["records_by_category"]["lifecycle"]["dropped_records"] = 1
        self.assertFalse(evaluate_software_qualification(data)["software_qualification_eligible"])

    def test_exact_retained_outcome_resolves_no_new_bundle(self):
        common = {
            "runtime_instance_id": "core-a", "session_id": "run-a",
            "localization_epoch": 7, "goal_epoch": 5, "request_id": 4,
            "bundle_owner_request_id": 4, "bundle_owner_cycle_id": 30,
        }
        events = [
            dict(common, phase="request", disposition="PUBLISHED",
                 causal_planning_cycle_id=30),
            dict(common, phase="result", disposition="OBSERVED",
                 planner_disposition=4, runtime_admission_attempted=False),
            dict(common, phase="retained", disposition="OBSERVED",
                 disposition_code=8, owner_snapshot_current=1,
                 callback_request_current=1, after_command_available=1,
                 after_failure_latched=0),
        ]
        transaction = reduce_lifecycle(events)["transactions"][0]
        self.assertEqual(transaction["evidence_outcome"], "INTENTIONALLY_ABSENT")
        self.assertEqual(transaction["terminal_outcome"], "RETAINED_INCUMBENT")
        events[-1]["owner_snapshot_current"] = 0
        self.assertEqual(reduce_lifecycle(events)["transactions"][0][
            "evidence_outcome"], "MISSING_EVIDENCE")

    def test_heading_rebind_has_its_own_origin_not_a_planning_cycle(self):
        events = command_events()
        base = {key: value for key, value in events[-1].items()
                if key in ("runtime_instance_id", "session_id", "localization_epoch",
                           "goal_epoch", "request_id", "bundle_generation", "sample_id",
                           "world_generation", "world_revision",
                           "world_observation_stamp_ns")}
        base.update(bundle_generation=15, sample_id=23, bundle_source=2)
        heading = [
            dict(base, phase="heading_admitted", disposition="ADMITTED",
                 parent_bundle_generation=11,
                 bundle_owner_attribution="producer_declared"),
            dict(base, phase="activate", disposition="ACTIVATED",
                 bundle_owner_attribution="producer_declared"),
            dict(base, phase="authorize", disposition="AUTHORIZED",
                 bundle_owner_attribution="producer_declared",
                 authorization_boundary="execution_timeline_publish_if_current",
                 authorization_steady_ns=101),
            dict(base, phase="publish", disposition="OBSERVED",
                 bundle_source=None, attribution="px4_input_trace",
                 adapter_trace_sequence=9, setpoint_kind="tracking"),
        ]
        reduction = reduce_lifecycle(heading)
        self.assertEqual(len(reduction["unresolved"]), 0)
        self.assertEqual(reduction["transactions"][0]["identity"]["producer_kind"],
                         "HEADING_REBIND")
        self.assertEqual(len(reduction["producer_owned_reference_ids"]), 1)

    def test_emergency_commit_witness_is_required_for_emergency_reference(self):
        base = {key: value for key, value in command_events()[-1].items()
                if key in ("runtime_instance_id", "session_id", "localization_epoch",
                           "goal_epoch", "request_id", "bundle_generation", "sample_id",
                           "world_generation", "world_revision",
                           "world_observation_stamp_ns")}
        base.update(bundle_generation=16, sample_id=24, bundle_source=3)
        events = [
            dict(base, phase="retained", disposition="OBSERVED",
                 disposition_code=5, after_bundle_generation=16,
                 after_command_available=1, after_failure_latched=0),
            dict(base, phase="authorize", disposition="AUTHORIZED",
                 bundle_owner_attribution="producer_declared",
                 authorization_boundary="execution_timeline_publish_if_current",
                 authorization_steady_ns=102),
            dict(base, phase="publish", disposition="OBSERVED", bundle_source=None,
                 attribution="px4_input_trace", adapter_trace_sequence=10,
                 setpoint_kind="tracking"),
        ]
        reduction = reduce_lifecycle(events)
        self.assertEqual(len(reduction["unresolved"]), 0)
        self.assertEqual(reduction["transactions"][0]["identity"]["producer_kind"],
                         "EMERGENCY_BRAKE")
        self.assertEqual(len(reduction["producer_owned_reference_ids"]), 1)
        events[0]["after_failure_latched"] = 1
        self.assertEqual(len(reduce_lifecycle(events)["producer_owned_reference_ids"]), 0)

    def test_pending_supersession_requires_exact_owner_transition(self):
        base = {key: value for key, value in command_events()[0].items()
                if key in ("runtime_instance_id", "session_id", "localization_epoch",
                           "goal_epoch", "request_id", "bundle_generation",
                           "bundle_owner_cycle_id")}
        events = [
            dict(base, phase="request", disposition="PUBLISHED",
                 causal_planning_cycle_id=29),
            dict(base, phase="result", disposition="OBSERVED",
                 planner_status=0, runtime_admission_attempted=True),
            dict(base, phase="export", disposition="EXPORTED",
                 causal_planning_cycle_id=29),
            dict(base, phase="supersede", disposition="SUPERSEDED",
                 replacement_bundle_generation=12,
                 previous_snapshot_version=4, current_snapshot_version=5),
        ]
        transaction = reduce_lifecycle(events)["transactions"][0]
        self.assertEqual(transaction["terminal_outcome"], "SUPERSEDED_PENDING")
        self.assertEqual(transaction["evidence_outcome"], "SUPERSEDED")
        events[-1]["current_snapshot_version"] = 4
        self.assertEqual(reduce_lifecycle(events)["transactions"][0][
            "evidence_outcome"], "MISSING_EVIDENCE")
        events[-1].update(current_snapshot_version=5,
                          replacement_bundle_generation=0,
                          admission_goal_epoch=5)
        self.assertEqual(reduce_lifecycle(events)["transactions"][0][
            "terminal_outcome"], "SUPERSEDED_PENDING")
        events[-1]["admission_goal_epoch"] = 4
        self.assertEqual(reduce_lifecycle(events)["transactions"][0][
            "evidence_outcome"], "MISSING_EVIDENCE")

    def test_retry_from_rest_requires_producer_outcome(self):
        base = {key: value for key, value in command_events()[0].items()
                if key in ("runtime_instance_id", "session_id", "localization_epoch",
                           "goal_epoch", "request_id", "bundle_owner_cycle_id")}
        events = [
            dict(base, phase="request", disposition="PUBLISHED",
                 causal_planning_cycle_id=29),
            dict(base, phase="result", disposition="OBSERVED",
                 planner_disposition=2, runtime_admission_attempted=False),
            dict(base, phase="recovery_retry", disposition="RETRY_SCHEDULED",
                 identity_current=True, timeout=False, after_failed=False),
        ]
        transaction = reduce_lifecycle(events)["transactions"][0]
        self.assertEqual(transaction["terminal_outcome"], "RECOVERY_RETRY_SCHEDULED")
        self.assertEqual(transaction["evidence_outcome"], "INTENTIONALLY_ABSENT")
        events[-1]["identity_current"] = False
        self.assertEqual(reduce_lifecycle(events)["transactions"][0][
            "evidence_outcome"], "MISSING_EVIDENCE")

    def test_emergency_commit_closes_its_exact_causal_planning_cycle(self):
        base = {key: value for key, value in command_events()[0].items()
                if key in ("runtime_instance_id", "session_id", "localization_epoch",
                           "goal_epoch", "request_id", "bundle_owner_cycle_id")}
        events = [
            dict(base, phase="request", disposition="PUBLISHED",
                 causal_planning_cycle_id=29),
            dict(base, phase="result", disposition="OBSERVED",
                 planner_status=4, planner_disposition=4,
                 runtime_admission_attempted=False),
            dict(base, phase="retained", disposition="OBSERVED", purpose=0,
                 planning_cycle_id=29, disposition_code=5,
                 after_bundle_generation=12, callback_request_current=1,
                 after_command_available=1, after_failure_latched=0),
        ]
        transactions = reduce_lifecycle(events)["transactions"]
        cycle = next(item for item in transactions
                     if item["identity"]["producer_kind"] == "PLANNING_CYCLE")
        self.assertEqual(cycle["terminal_outcome"], "EMERGENCY_COMMITTED")
        events[-1]["callback_request_current"] = 0
        transactions = reduce_lifecycle(events)["transactions"]
        cycle = next(item for item in transactions
                     if item["identity"]["producer_kind"] == "PLANNING_CYCLE")
        self.assertEqual(cycle["evidence_outcome"], "MISSING_EVIDENCE")

    def test_terminal_monitor_is_its_own_producer_transaction(self):
        base = {key: value for key, value in command_events()[0].items()
                if key in ("runtime_instance_id", "session_id", "localization_epoch",
                           "goal_epoch", "request_id")}
        monitor = dict(
            base, phase="retained", disposition="OBSERVED", purpose=2,
            planning_cycle_id=90, bundle_owner_cycle_id=90,
            producer_event_sequence=6, captured_bundle_generation=11,
            after_bundle_generation=11, disposition_code=8,
            state_ingress_sequence=10, final_state_source_ros_ns=101,
            final_state_receive_steady_ns=102,
            owner_snapshot_current=1, callback_request_current=1,
            monitor_window_current=1, after_command_available=1,
            after_failure_latched=0, final_witness_age_bounded=1,
            final_body_known_free=1, final_anchor_valid=1,
            final_bridge_usable=0, final_freshness_reason=0,
        )
        transaction = reduce_lifecycle([monitor])["transactions"][0]
        self.assertEqual(transaction["identity"]["producer_kind"], "TERMINAL_MONITOR")
        self.assertEqual(transaction["terminal_outcome"], "CERTIFIED_COMMAND_PRESERVED")
        monitor["monitor_window_current"] = 0
        self.assertEqual(reduce_lifecycle([monitor])["transactions"][0][
            "evidence_outcome"], "MISSING_EVIDENCE")
        self.assertEqual(len(reduce_lifecycle([monitor])["unresolved"]), 1)

    def test_exact_typed_adapter_rejection_explains_undelivered_sample(self):
        data = software_inputs()
        # The transaction has an earlier delivered sample. A later exact Core
        # authorization exists, but the adapter rejects it before PX4 publish.
        data["lifecycle"].append(dict(
            data["lifecycle"][-2], sample_id=23, authorization_steady_ns=100))
        data["lifecycle_reduction"] = reduce_lifecycle(data["lifecycle"])
        data["scenario_events"].append({"kind": "command_rejection", "payload": {
            "command_present": True, "stage": 3, "reason_code": 1,
            "disposition": 4, "mode_activation_id": 9,
            "localization_epoch": 7, "goal_epoch": 4, "request_id": 3,
            "bundle_generation": 11, "sample_id": 23,
        }})
        data["pva"].append(dict(data["pva"][0], sample_id=23,
                                mode_activation_id=9))
        result = evaluate_software_qualification(data)
        self.assertEqual(result["required_reference_missing"], 0)
        self.assertTrue(result["software_qualification_eligible"])
        data["scenario_events"][-1]["payload"]["sample_id"] = 24
        self.assertEqual(evaluate_software_qualification(data)[
            "required_reference_missing"], 1)


if __name__ == "__main__":
    unittest.main()
