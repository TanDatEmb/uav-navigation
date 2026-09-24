from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from command_diagnostics import join_execution_diagnostics


def command(sample=7):
    return {"kind": "pva_command", "payload": {
        "mode_activation_id": 2, "localization_epoch": 3, "goal_epoch": 4,
        "mission_id": "m", "waypoint_index": 1, "request_id": 5,
        "bundle_generation": 6, "sample_id": sample, "stamp_ns": 100,
        "position": [1.0, 2.0, 3.0], "role": 0,
    }}


def diagnostic(sample=7, *, stamp=100, value=0.25):
    return {"kind": "execution_diagnostics", "payload": {
        "mode_activation_id": 2, "localization_epoch": 3, "goal_epoch": 4,
        "mission_id": "m", "waypoint_index": 1, "request_id": 5,
        "bundle_generation": 6, "sample_id": sample,
        "header_stamp_ns": stamp, "anchor_error_m": value,
    }}


class CommandDiagnosticsJoinTest(unittest.TestCase):
    def test_delayed_and_duplicated_diagnostic_do_not_change_control_record(self):
        cmd = command()
        events = [cmd, diagnostic(), diagnostic()]
        events[1]["payload"]["position"] = [99.0, 99.0, 99.0]
        events[2]["payload"]["position"] = [99.0, 99.0, 99.0]
        self.assertEqual(join_execution_diagnostics(events), [])
        self.assertEqual(cmd["payload"]["anchor_error_m"], 0.25)
        self.assertEqual(cmd["payload"]["position"], [1.0, 2.0, 3.0])
        self.assertFalse(cmd["payload"]["execution_diagnostic_missing"])

    def test_missing_diagnostic_keeps_command_and_marks_evidence_gap(self):
        cmd = command()
        self.assertEqual(join_execution_diagnostics([cmd]), [])
        self.assertEqual(cmd["payload"]["position"], [1.0, 2.0, 3.0])
        self.assertTrue(cmd["payload"]["execution_diagnostic_missing"])

    def test_wrong_identity_cannot_attach_evidence(self):
        cmd = command()
        self.assertEqual(join_execution_diagnostics([diagnostic(sample=8), cmd]), [])
        self.assertTrue(cmd["payload"]["execution_diagnostic_missing"])

    def test_conflict_or_timestamp_mismatch_is_not_attached(self):
        cmd = command()
        issues = join_execution_diagnostics([cmd, diagnostic(), diagnostic(value=0.3)])
        self.assertIn("EXECUTION_DIAGNOSTIC_CONFLICT", issues)
        self.assertTrue(cmd["payload"]["execution_diagnostic_missing"])
        cmd = command()
        issues = join_execution_diagnostics([cmd, diagnostic(stamp=101)])
        self.assertIn("EXECUTION_DIAGNOSTIC_STAMP_MISMATCH", issues)
        self.assertTrue(cmd["payload"]["execution_diagnostic_missing"])


if __name__ == "__main__":
    unittest.main()
