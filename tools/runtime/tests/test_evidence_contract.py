from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

import closed_loop_characterization as recorder
from evidence_contract import build_evidence_contract


def odom(stamp=1_000_000_000, position=(0.0, 0.0, 0.0), velocity=(1.0, 0.0, 0.0),
         frame="x500_mid360/odom", child="base_link", q=(0.0, 0.0, 0.7071067812, 0.7071067812), epoch=1):
    pose = SimpleNamespace(position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
                           orientation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))
    twist = SimpleNamespace(linear=SimpleNamespace(x=velocity[0], y=velocity[1], z=velocity[2]))
    nested = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000), frame_id=frame),
                             child_frame_id=child, pose=SimpleNamespace(pose=pose), twist=SimpleNamespace(twist=twist), localization_epoch=epoch)
    return nested


class EvidenceContractTest(unittest.TestCase):
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


class MetricSourceValidationTest(unittest.TestCase):
    @staticmethod
    def _inputs():
        return {
            "scenario": {
                "mission_complete_observed": True,
                "outcome": "COMPLETE",
                "collision_count": 0,
                "minimum_collision_clearance_m": 1.0,
            },
            "scenario_events": [{
                "kind": "navigation_mode_status",
                "payload": {"waypoint_accepted": True, "accepted_waypoint_index": 0},
            }],
            "pva": [{
                "source_stamp_ns": 1_000_000_000,
                "time_basis": "source_stamp",
                "source_clock": "ros_time",
                "session_id": "session-a",
                "request_id": 1,
                "bundle_generation": 1,
                "sample_id": 1,
                "position": [0.0, 0.0, 0.0],
                "velocity": [1.0, 0.0, 0.0],
                "frame_id": "world",
                "trajectory_flag": 0,
            }],
            "streams": {"ground_truth_odometry": [{
                "source_stamp_ns": 1_000_000_000,
                "time_basis": "source_stamp",
                "source_clock": "ros_time",
                "position": [0.0, 0.0, 0.0],
                "linear_velocity": [1.0, 0.0, 0.0],
                "frame_id": "world",
            }]},
        }

    @staticmethod
    def _metric(contract, name):
        return next(item for item in contract["metrics"] if item["metric"] == name)

    def test_malformed_source_values_are_not_presence_only(self):
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
                data = self._inputs()
                data["pva"][0][field] = value
                contract = build_evidence_contract(data)
                metric = self._metric(contract, "tracking.position_error_m")
                source = metric["sources"][0]
                self.assertEqual(metric["status"], "NOT_EVALUABLE")
                self.assertIn(field, source["invalid_fields"])
                self.assertIn(reason, source["invalid_reasons"])
                self.assertIn(
                    "tracking.position_error_m source incomplete",
                    contract["qualification_missing"],
                )

    def test_missing_source_clock_and_identity_are_not_inferred(self):
        data = self._inputs()
        data["pva"][0].pop("source_clock")
        data["pva"][0].pop("session_id")
        source = self._metric(
            build_evidence_contract(data), "tracking.position_error_m"
        )["sources"][0]
        self.assertEqual(source["status"], "NOT_EVALUABLE")
        self.assertIn("source_clock", source["missing_fields"])
        self.assertIn("session_id", source["missing_fields"])

    def test_non_executable_terminal_marker_is_not_command_evidence(self):
        data = self._inputs()
        terminal = dict(data["pva"][0])
        terminal.update({
            "source_stamp_ns": 2_000_000_000,
            "sample_id": 2,
            "bundle_generation": 0,
            "executable": False,
        })
        data["pva"].append(terminal)

        contract = build_evidence_contract(data)
        source = self._metric(contract, "tracking.position_error_m")["sources"][0]
        self.assertEqual(source["row_count"], 1)
        self.assertEqual(source["status"], "AVAILABLE")
        self.assertEqual(source["invalid_rows"], [])

    def test_reaction_witness_contract_requires_typed_trigger_and_onset(self):
        data = self._inputs()
        data["reaction_witnesses"] = [{
            "trigger_event_id": "emergency-1",
            "attitude_onset_event_id": "attitude-onset-1",
            "producer": "px4_reaction_witness",
            "predicate_id": "vehicle_attitude_tilt_onset_v1",
            "trigger_kind": "measured_emergency_brake",
            "attitude_onset_kind": "vehicle_attitude_tilt_onset",
            "attitude_onset_observed": True,
            "trigger_source_stamp_ns": 1_000_000_000,
            "attitude_onset_source_stamp_ns": 1_150_000_000,
            "time_basis": "source_stamp",
            "source_clock": "ros_time",
            "achieved_acceleration_mps2": 8.5,
            "achieved_jerk_mps3": 7.5,
        }]
        metric = self._metric(
            build_evidence_contract(data), "control.u4_reaction_witness"
        )
        self.assertEqual(metric["status"], "AVAILABLE")
        self.assertEqual(metric["sources"][0]["invalid_fields"], [])

    def test_attitude_only_rows_do_not_satisfy_reaction_contract(self):
        data = self._inputs()
        data["reaction_witnesses"] = [{
            "source_stamp_ns": 1_150_000_000,
            "tilt_rad": 0.1,
        }]
        metric = self._metric(
            build_evidence_contract(data), "control.u4_reaction_witness"
        )
        self.assertEqual(metric["status"], "NOT_EVALUABLE")
        self.assertIn("trigger_event_id", metric["sources"][0]["missing_fields"])


if __name__ == "__main__":
    unittest.main()
