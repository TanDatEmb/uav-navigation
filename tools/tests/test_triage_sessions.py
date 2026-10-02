from __future__ import annotations

import json
import re
import tempfile
import unittest
from pathlib import Path

from tools.refactor import triage_sessions as T

REPO = Path(__file__).resolve().parents[2]


def trace(sim_ns: int, *, solve_generation: int, outcome: int, stage: int = 0, reason: int = 0,
          replan_code: int = 3, commit_decision: int = 1, tcert: str = "0", emergency: str = "0",
          anchor: str = "nan", limit: str = "0.25") -> dict:
    values = {
        "solve_generation": str(solve_generation), "planning_cycle_id": str(solve_generation),
        "planning_outcome": str(outcome), "planning_failure_stage": str(stage),
        "planning_failure_reason": str(reason), "replan_code": str(replan_code),
        "commit_decision": str(commit_decision), "solve_deadline_exceeded": "0",
        "planning_latency_ms": "30.0", "anchor_error_m": anchor,
        "projected_anchor_error_m": anchor, "retained_tracking_limit_m": limit,
        "tracking_certificate_exceeded": tcert, "emergency_candidate_commit_result": emergency,
        "time_to_backup_start_s": "0.5", "pinned_world_revision": "1",
        "certificate_world_revision": "1", "planner_disposition": "0",
        "runtime_admission_attempted": "0", "execution_boundary_rejection": "0",
        "planning_start_mode": "0",
    }
    return {"record": {"kind": "planner_trace", "sim_time_ns": sim_ns,
                       "payload": {"source_stamp_ns": sim_ns, "values": values}}}


ENVELOPE_LINE = (
    "[ERROR] [1790863598.675555609] [px4_navigation_external_mode]: planner backend tracking "
    "envelope exceeded: longitudinal=0.893/0.750 m reverse=0.000/0.750 m lateral=0.000/0.750 m "
    "measured_enu=[22.457,-0.071,3.106] command_enu=[21.570,-0.062,3.003] "
    "measured_velocity_body_frame=[-0.149,0.082,-0.150] command_velocity_enu=[0.000,-0.000,-0.000] "
    "command_acceleration_enu=[-0.036,0.002,0.026] command_jerk_enu=[4.411,-0.261,-3.198] "
    "odom_header_age_ms=4.000 odom_receive_age_ms=4.000 message_id=404 generation=6 "
    "role=EMERGENCY trajectory_time=0.768000 s status=2 stamp=27.219999999 previous_message_id=403")


def write_session(root: Path, records: list[dict], *, mapping_warns: list[str] = (),
                  adapter_lines: list[str] = (), report: dict | None = None,
                  extra_timeline: list[dict] = ()) -> Path:
    session = root / "external-mode-check-fixture"
    (session / "logs").mkdir(parents=True)
    with (session / "planning_timeline.jsonl").open("w", encoding="utf-8") as handle:
        for item in list(records) + list(extra_timeline):
            handle.write(json.dumps(item) + "\n")
    (session / "report.json").write_text(json.dumps(report or {
        "runtime_verdict": "BLOCKED",
        "mission_outcome": {"scenario": {"outcome": "PAUSED_SAFETY_STOP"}, "acceptance": {}},
    }), encoding="utf-8")
    (session / "scenario.json").write_text("{}", encoding="utf-8")
    (session / "metadata.json").write_text("{}", encoding="utf-8")
    (session / "execution_timeline.jsonl").write_text("", encoding="utf-8")
    (session / "logs" / "mapping.log").write_text("\n".join(mapping_warns), encoding="utf-8")
    (session / "logs" / "navigation_runtime_node_1_1.log").write_text("", encoding="utf-8")
    (session / "logs" / "px4_navigation_external_mode_node_1_1.log").write_text(
        "\n".join(adapter_lines), encoding="utf-8")
    return session


def rejection(sim_ns: int, disposition: int) -> dict:
    return {"record": {"kind": "command_rejection", "sim_time_ns": sim_ns,
                       "payload": {"stage": 7, "reason_code": 1, "disposition": disposition,
                                   "tracking_longitudinal_error_m": 0.893,
                                   "tracking_lateral_error_m": 0.0}}}


class TriageSessionTests(unittest.TestCase):
    def test_final_streak_and_dynamics_initiator(self) -> None:
        records = [
            trace(1_000_000_000, solve_generation=1, outcome=0),
            trace(2_000_000_000, solve_generation=2, outcome=5, stage=6, reason=6, replan_code=-6,
                  commit_decision=0, tcert="1", emergency="1", anchor="0.4"),
            rejection(2_700_000_000, T.DISPOSITION_SAFETY_STOP),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            session = write_session(Path(tmp), records, adapter_lines=[ENVELOPE_LINE])
            chain = T.triage_session(session)
        self.assertEqual(chain["initiator"], "SOLVE_DYNAMICS")
        self.assertEqual(chain["failing_streak_len"], 1)
        self.assertEqual(chain["first_failure_stage_reason"], "nominal_seed/nominal_dynamics")
        self.assertEqual(chain["adapter_first_error"]["kind"], "ADAPTER_TRACKING_ENVELOPE")
        self.assertAlmostEqual(chain["adapter_first_error"]["longitudinal_m"], 0.893)
        self.assertEqual(chain["emergency_count"], 1)
        self.assertTrue(chain["emergency_events"][0]["followed_by_stop_within_3s"])
        self.assertEqual(chain["stop_sim_s"], 2.7)

    def test_world_changed_label_alone_is_not_world_advance(self) -> None:
        # rc -7 maps to commit_recertification/world_changed for every cause (replan_contract.hpp).
        records = [
            trace(1_000_000_000, solve_generation=1, outcome=0),
            trace(2_000_000_000, solve_generation=2, outcome=5, stage=13, reason=11,
                  replan_code=-7, commit_decision=6),
            rejection(2_500_000_000, T.DISPOSITION_SAFETY_STOP),
        ]
        warns = ["[navigation_runtime_node-1] [planner WARN]  -- [planner] command rejected by MAIN "
                 "route-regression certificate: maximum=1.1 tolerance=0.5"]
        with tempfile.TemporaryDirectory() as tmp:
            chain = T.triage_session(write_session(Path(tmp), records, mapping_warns=warns))
        self.assertEqual(chain["initiator"], "CERT_ROUTE_REGRESSION")
        self.assertEqual(chain["commit_world_advanced_count"], 0)

    def test_commit_gate_world_advanced_is_detected_from_commit_decision(self) -> None:
        records = [
            trace(1_000_000_000, solve_generation=1, outcome=0),
            trace(2_000_000_000, solve_generation=2, outcome=5, stage=13, reason=11,
                  replan_code=-7, commit_decision=T.WORLD_ADVANCED),
        ]
        warns = ["[planner WARN]  -- [planner] command authorization rejected with reason 3"]
        with tempfile.TemporaryDirectory() as tmp:
            chain = T.triage_session(write_session(Path(tmp), records, mapping_warns=warns))
        self.assertEqual(chain["initiator"], "COMMIT_GATE_WORLD_ADVANCED")
        self.assertEqual(chain["commit_world_advanced_count"], 1)
        self.assertEqual(chain["reject_subtype_hist"], {"COMMIT_GATE_REJECTED": 1})

    def test_reject_subtype_is_unmatched_when_counts_differ(self) -> None:
        records = [trace(1_000_000_000, solve_generation=1, outcome=5, stage=13, reason=11,
                         replan_code=-7, commit_decision=6)]
        with tempfile.TemporaryDirectory() as tmp:
            chain = T.triage_session(write_session(Path(tmp), records, mapping_warns=[]))
        self.assertEqual(chain["reject_subtype_hist"], {"UNMATCHED": 1})
        self.assertEqual(chain["initiator"], "CERT_REJECTED_OTHER")

    def test_startup_invalid_request_is_not_a_failing_solve(self) -> None:
        records = [
            trace(1_000_000_000, solve_generation=0, outcome=8, stage=1, reason=15, replan_code=0,
                  commit_decision=0),
            trace(2_000_000_000, solve_generation=1, outcome=0),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            chain = T.triage_session(write_session(Path(tmp), records))
        self.assertEqual(chain["failure_count"], 0)
        self.assertEqual(chain["initiator"], "NO_SOLVE_FAILURE")

    def test_no_execution_authority_is_split_at_the_stop(self) -> None:
        def tx(outcome: str, stamp_ns: int) -> dict:
            return {"terminal_outcome": outcome,
                    "events": {"authorize": {"source_stamp_ns": stamp_ns}}}

        report = {
            "runtime_verdict": "BLOCKED",
            "mission_outcome": {"scenario": {"outcome": "PAUSED_SAFETY_STOP"}, "acceptance": {}},
            "evaluation": {"lifecycle_reduction": {"transactions": [
                tx("NO_EXECUTION_AUTHORITY", 3_000_000_000),
                tx("NO_EXECUTION_AUTHORITY", 4_000_000_000),
                tx("SUPERSEDED", 1_000_000_000)]}},
        }
        records = [trace(1_000_000_000, solve_generation=1, outcome=0),
                   rejection(2_500_000_000, T.DISPOSITION_SAFETY_STOP)]
        with tempfile.TemporaryDirectory() as tmp:
            chain = T.triage_session(write_session(Path(tmp), records, report=report))
        lifecycle = chain["lifecycle"]
        self.assertEqual(lifecycle["pre_stop_by_terminal_outcome"], {"SUPERSEDED": 1})
        self.assertEqual(lifecycle["post_stop_by_terminal_outcome"], {"NO_EXECUTION_AUTHORITY": 2})
        self.assertEqual(lifecycle["first_listed_terminal_outcome"], "NO_EXECUTION_AUTHORITY")

    def test_vehicle_ahead_of_command_sign(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            session = write_session(Path(tmp), [trace(1_000_000_000, solve_generation=1, outcome=0)],
                                    adapter_lines=[ENVELOPE_LINE])
            sample = {"record": {"kind": "sample", "stream": "ground_truth_odometry",
                                 "payload": {"stamp_ns": 2_000_000_000,
                                             "linear_velocity": [4.0, 0.0, 0.0],
                                             "position": [0.0, 0.0, 3.0]}}}
            (session / "execution_timeline.jsonl").write_text(json.dumps(sample) + "\n",
                                                              encoding="utf-8")
            adapter = T.adapter_first_error(session)
            ahead = T._ahead_of_command(session, adapter, 3.0)
        self.assertAlmostEqual(ahead, 0.887, places=3)

    def test_command_stale_is_reported_with_runtime_error_as_cause(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            session = write_session(
                Path(tmp), [trace(1_000_000_000, solve_generation=1, outcome=0)],
                adapter_lines=["[ERROR] [100.0] [px4_navigation_external_mode]: planner backend PVA "
                               "command stale; safety hold then handover to PX4 Hold"])
            (session / "logs" / "navigation_runtime_node_1_1.log").write_text(
                "[ERROR] [99.0] [navigation_runtime_node]: execution boundary rejected planned "
                "STOPPED_HOLD endpoint known_free=1 execution_support_valid=1 near_execution=0 "
                "anchor_error_m=0.753 anchor_limit_m=0.750\n", encoding="utf-8")
            chain = T.triage_session(session)
        self.assertEqual(chain["adapter_first_error"]["kind"], "ADAPTER_COMMAND_STALE")
        self.assertEqual(chain["runtime_first_error"]["kind"], "RUNTIME_STOPPED_HOLD_ENDPOINT_REJECTED")
        self.assertAlmostEqual(chain["runtime_first_error"]["anchor_error_m"], 0.753)

    def test_aggregate_counts(self) -> None:
        records = [trace(1_000_000_000, solve_generation=1, outcome=0),
                   trace(2_000_000_000, solve_generation=2, outcome=5, stage=8, reason=8,
                         replan_code=-8, commit_decision=0, tcert="1", emergency="1"),
                   rejection(2_700_000_000, T.DISPOSITION_SAFETY_STOP)]
        with tempfile.TemporaryDirectory() as tmp:
            chain = T.triage_session(write_session(Path(tmp), records, adapter_lines=[ENVELOPE_LINE]))
        summary = T.aggregate([chain])
        self.assertEqual(summary["paused_safety_stop_sessions"], 1)
        self.assertEqual(summary["initiator_of_paused_safety_stop"], {"SOLVE_KNOWN_FREE": 1})
        self.assertEqual(summary["terminal_of_paused_safety_stop"], {"ADAPTER_TRACKING_ENVELOPE": 1})
        self.assertEqual(summary["sessions_with_world_advanced_in_final_streak"], 0)


def _snake(name: str) -> str:
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def _enum_body(text: str, name: str) -> list[str]:
    match = re.search(r"enum class " + name + r"[^{]*\{(.*?)\};", text, re.S)
    if not match:
        raise AssertionError(name)
    body = re.sub(r"//[^\n]*", "", match.group(1))
    return [item.strip() for item in body.split(",") if item.strip()]


class EnumAlignmentTests(unittest.TestCase):
    """The lookup tables are copies; fail loudly if the product headers move."""

    def test_stage_reason_outcome_tables_match_planning_outcome_header(self) -> None:
        header = REPO / "src/planning/navigation_planning/include/navigation_planning/planning_outcome.hpp"
        if not header.exists():
            self.skipTest("product header not present")
        text = header.read_text(encoding="utf-8")
        self.assertEqual([_snake(x[1:]) for x in _enum_body(text, "PlanningFailureStage")], T.STAGE_NAMES)
        self.assertEqual([_snake(x[1:]) for x in _enum_body(text, "PlanningFailureReason")], T.REASON_NAMES)
        self.assertEqual([x[1:] for x in _enum_body(text, "CompletePlanningOutcome")], T.OUTCOME_NAMES)

    def test_world_commit_decision_table_matches_header(self) -> None:
        header = REPO / "src/mapping/navigation_world_model/include/navigation_world_model/world_commit_authorizer.hpp"
        if not header.exists():
            self.skipTest("product header not present")
        text = header.read_text(encoding="utf-8")
        self.assertEqual(_enum_body(text, "WorldCommitDecision"), T.WORLD_COMMIT_NAMES)
        self.assertEqual(T.WORLD_COMMIT_NAMES[T.WORLD_ADVANCED], "kWorldAdvanced")


if __name__ == "__main__":
    unittest.main()
