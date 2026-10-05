import json
import tempfile
import unittest
from pathlib import Path

from tools.runtime.wave5_u9_distribution import (
    extract_session,
    nearest_rank,
    summarize,
)


class Wave5U9DistributionTest(unittest.TestCase):
    def test_nearest_rank_matches_five_run_matrix(self):
        self.assertEqual(nearest_rank([5, 1, 3, 2, 4], 0.50), 3.0)
        self.assertEqual(nearest_rank([5, 1, 3, 2, 4], 0.95), 5.0)
        self.assertIsNone(nearest_rank([], 0.95))

    def test_extracts_final_finite_seed_and_summarizes_group(self):
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary) / "external-mode-check-20261004T000000-1"
            session.mkdir()
            (session / "metadata.json").write_text(
                json.dumps(
                    {
                        "scenario_identity": {"requested": {"scene": "sanity_open"}},
                        "requested_cruise_speed_mps": 3.0,
                    }
                ),
                encoding="utf-8",
            )
            (session / "scenario.json").write_text(
                json.dumps(
                    {
                        "mission_complete_observed": False,
                        "outcome": "PAUSED_SAFETY_STOP",
                        "collision_count": 2,
                    }
                ),
                encoding="utf-8",
            )
            values = {
                "backup_last_seed_duration_s": "nan",
                "backup_last_seed_attempts_used": "1",
            }
            final_values = {
                "backup_last_seed_duration_s": "2.0",
                "backup_last_seed_attempts_used": "4",
                "backup_last_seed_maximum_attempts": "24",
                "backup_last_seed_steady_cruise": "0",
                "backup_last_seed_failure": "0",
                "backup_last_seed_entry_acceleration_mps2": "1.0",
                "backup_last_seed_entry_jerk_mps3": "2.0",
                "backup_last_seed_initial_velocity_mps": "3.0",
            }
            records = [
                {"record": {"payload": {"values": values}}},
                {"record": {"payload": {"values": final_values}}},
            ]
            (session / "planning_timeline.jsonl").write_text(
                "\n".join(json.dumps(record) for record in records) + "\n",
                encoding="utf-8",
            )
            witness = extract_session(session)
            self.assertIsNotNone(witness)
            self.assertEqual(witness["duration_s"], 2.0)
            self.assertEqual(witness["attempts_used"], 4)
            summary = summarize([witness])
            self.assertEqual(summary["session_count"], 1)
            self.assertEqual(summary["groups"][0]["duration_s_p95"], 2.0)
            self.assertEqual(summary["groups"][0]["initial_velocity_mps_p95"], 3.0)
            self.assertEqual(summary["groups"][0]["collision_count_total"], 2)

    def test_incomplete_session_is_not_measurement(self):
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary) / "external-mode-check-20261004T000000-2"
            session.mkdir()
            (session / "metadata.json").write_text(
                json.dumps(
                    {
                        "scenario_identity": {"requested": {"scene": "sanity_open"}},
                        "requested_cruise_speed_mps": 3.0,
                    }
                ),
                encoding="utf-8",
            )
            (session / "planning_timeline.jsonl").write_text("", encoding="utf-8")
            self.assertIsNone(extract_session(session))


if __name__ == "__main__":
    unittest.main()
