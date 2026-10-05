import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import report


REPO_ROOT = Path(__file__).resolve().parents[3]
RUNTIME_ROOT = Path(__file__).resolve().parents[1]
STREAM_NAMES = (
    "simulation_clock", "imu", "lidar", "corrected_odometry",
    "propagated_odometry", "registered_scan", "ground_truth_odometry",
    "external_odometry", "px4_odometry", "vehicle_status", "local_position",
    "estimator_status_flags", "px4_local_position_setpoint",
    "px4_attitude_setpoint", "px4_thrust_setpoint", "px4_actuator_motors",
)
NAVIGATION_START_NS = 10_000_000_000
OBSERVATION_END_NS = 60_000_000_000


class ReportHandoverTests(unittest.TestCase):
    def _report(
        self,
        *,
        outcome: str,
        stale_ns: int,
        expected_outcome: str = "complete",
        handover_ns: int | None = None,
    ) -> dict:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            streams = {
                name: {"received": 100, "mean_rate_hz": 50.0}
                for name in STREAM_NAMES
            }
            for name in ("external_odometry", "propagated_odometry"):
                streams[name].update({
                    "stale_event_count": 1,
                    "stale_event_times_ns": [stale_ns],
                })
            (session / "monitor.json").write_text(json.dumps({
                "streams": streams,
                "diagnostics": {"state": "TRACKING"},
            }), encoding="utf-8")
            (session / "runtime.json").write_text(json.dumps({
                "navigation_start_wall_ns": NAVIGATION_START_NS,
                "observation_finished_wall_ns": OBSERVATION_END_NS,
            }), encoding="utf-8")
            scenario = {
                "outcome": outcome,
                "expected_outcome": expected_outcome,
            }
            if handover_ns is not None:
                scenario["terminal_handover_wall_ns"] = handover_ns
            (session / "scenario.json").write_text(
                json.dumps(scenario), encoding="utf-8"
            )
            rows = []
            for name in ("external_odometry", "propagated_odometry"):
                rows.extend((
                    {
                        "kind": "sample", "stream": name,
                        "arrival_wall_ns": stale_ns - 1_000_000,
                        "timestamp_ns": stale_ns - 3_000_000_000,
                    },
                    {
                        "kind": "sample", "stream": name,
                        "arrival_wall_ns": stale_ns + 1_000_000,
                        "timestamp_ns": stale_ns,
                    },
                ))
            (session / "samples.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows),
                encoding="utf-8",
            )
            return report._sim_report(
                session,
                {"runtime": {"thresholds": {"stale_after_s": 0.5}}},
                json.loads((session / "monitor.json").read_text(encoding="utf-8")),
                REPO_ROOT,
                None,
                "external-mode",
            )

    def test_complete_mid_mission_stall_remains_a_violation(self) -> None:
        result = self._report(
            outcome="COMPLETE", stale_ns=20_000_000_000, handover_ns=30_000_000_000
        )
        for name in ("external_odometry", "propagated_odometry"):
            self.assertEqual(result["streams"][name]["source_stale_event_count"], 1)
            self.assertIn(f"{name} timestamp/freshness/validity violation", result["reasons"])

    def test_complete_stale_event_after_handover_is_exempt(self) -> None:
        result = self._report(
            outcome="COMPLETE", stale_ns=40_000_000_000, handover_ns=30_000_000_000
        )
        for name in ("external_odometry", "propagated_odometry"):
            self.assertEqual(result["streams"][name]["source_stale_event_count"], 0)
            self.assertNotIn(f"{name} timestamp/freshness/validity violation", result["reasons"])

    def test_complete_without_handover_marker_does_not_exempt(self) -> None:
        result = self._report(outcome="COMPLETE", stale_ns=40_000_000_000)
        for name in ("external_odometry", "propagated_odometry"):
            self.assertEqual(result["streams"][name]["source_stale_event_count"], 1)
            self.assertIn(f"{name} timestamp/freshness/validity violation", result["reasons"])

    def test_fail_closed_safety_stop_uses_the_same_bounded_exemption(self) -> None:
        exempted = self._report(
            outcome="PAUSED_SAFETY_STOP",
            expected_outcome="fail_closed",
            stale_ns=40_000_000_000,
            handover_ns=30_000_000_000,
        )
        retained = self._report(
            outcome="PAUSED_SAFETY_STOP",
            expected_outcome="fail_closed",
            stale_ns=20_000_000_000,
            handover_ns=30_000_000_000,
        )
        missing = self._report(
            outcome="PAUSED_SAFETY_STOP",
            expected_outcome="fail_closed",
            stale_ns=40_000_000_000,
        )
        for name in ("external_odometry", "propagated_odometry"):
            self.assertEqual(exempted["streams"][name]["source_stale_event_count"], 0)
            self.assertEqual(retained["streams"][name]["source_stale_event_count"], 1)
            self.assertEqual(missing["streams"][name]["source_stale_event_count"], 1)

    def test_r01_repro_complete_no_longer_zeros_mid_mission_stale_events(self) -> None:
        for outcome in ("", "COMPLETE"):
            result = self._report(outcome=outcome, stale_ns=20_000_000_000)
            self.assertEqual(
                result["streams"]["external_odometry"]["source_stale_event_count"], 1
            )
            self.assertTrue(any("odometry" in reason for reason in result["reasons"]))

    def test_handover_exemption_is_fail_closed_for_missing_or_malformed_marker(self) -> None:
        self.assertEqual(
            report.handover_stale_exemption([10, 20], None), []
        )
        self.assertEqual(
            report.handover_stale_exemption([10, 20], "not-a-timestamp"), []
        )
        self.assertEqual(
            report.handover_stale_exemption([10, 20], 15.5), []
        )
        self.assertEqual(
            report.handover_stale_exemption([10, "not-a-timestamp"], 15), []
        )
        self.assertEqual(
            report.handover_stale_exemption([10, 20, 30], 20), [20, 30]
        )

    def test_scenario_records_first_terminal_handover_wall_timestamp(self) -> None:
        path = RUNTIME_ROOT / "external_mode_scenario.py"
        spec = importlib.util.spec_from_file_location("external_mode_scenario_h1", path)
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        scenario = object.__new__(module.ExternalModeScenario)
        scenario.terminal_outcome = None
        scenario.terminal_handover_wall_ns = None
        scenario.events = []
        scenario.interactive_handover = True
        scenario.handover_waiting = False
        records = []
        scenario._record = lambda kind, payload: records.append((kind, payload))

        scenario._finish_or_wait("COMPLETE")

        marker = scenario.terminal_handover_wall_ns
        self.assertIsInstance(marker, int)
        self.assertGreater(marker, 0)
        self.assertEqual(scenario.events[0]["name"], "terminal_outcome_assigned")
        self.assertEqual(scenario.events[0]["terminal_handover_wall_ns"], marker)
        self.assertEqual(records[0][1]["terminal_handover_wall_ns"], marker)


if __name__ == "__main__":
    unittest.main()
