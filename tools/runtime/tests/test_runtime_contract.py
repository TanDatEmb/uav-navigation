from pathlib import Path
import sys
import tempfile
import time
import unittest

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

from monitor import StreamStats
import report


class RuntimeContractTest(unittest.TestCase):
    def test_first_sample_does_not_create_stale_event(self) -> None:
        stats = StreamStats("external_odometry", "/fmu/in/vehicle_visual_odometry", stale_after_s=0.1)
        start = 1_000_000_000
        stats.update(start, start)
        stats.check_stale(start + 200_000_000)
        self.assertEqual(stats.as_dict()["stale_event_count"], 0)

    def test_stream_stats_records_rates_and_regressions(self) -> None:
        stats = StreamStats("imu", "/lidar/imu", expected_hz=200.0, stale_after_s=0.1)
        start = 1_000_000_000
        stats.update(start, start, frame_id="livox_imu_frame")
        stats.update(start + 5_000_000, start + 5_000_000, frame_id="livox_imu_frame")
        stats.update(start + 4_000_000, start + 6_000_000, frame_id="livox_imu_frame")
        stats.check_stale(start + 200_000_000)
        snapshot = stats.as_dict()
        self.assertEqual(snapshot["received"], 3)
        self.assertEqual(snapshot["timestamp_regression_count"], 1)
        self.assertEqual(snapshot["stale_event_count"], 1)
        self.assertEqual(snapshot["frame_ids"], ["livox_imu_frame"])

    def test_residual_report_declares_missing_pre_fusion_data(self) -> None:
        result = report._residuals([], 20.0)
        self.assertEqual(result["pre_fusion"], "NOT_AVAILABLE")
        self.assertEqual(result["fusion_enabled"], "OBSERVED_ONLY")
        self.assertTrue(result["circular_comparison"])

    def test_residual_report_aligns_lio_frd_to_px4_world(self) -> None:
        yaw_quarter_turn = [0.7071067811865476, 0.0, 0.0, 0.7071067811865476]

        def sample(stream: str, stamp_us: int, payload: dict[str, object]) -> dict[str, object]:
            return {"kind": "sample", "stream": stream, "payload": payload, "timestamp_ns": stamp_us * 1000}

        samples = [
            sample(
                "external_odometry",
                1_000,
                {"timestamp_sample_us": 1_000, "position": [1.0, 2.0, 3.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0], "velocity": [1.0, 0.0, 0.0]},
            ),
            sample(
                "external_odometry",
                1_020,
                {"timestamp_sample_us": 1_020, "position": [2.0, 2.0, 3.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0], "velocity": [1.0, 0.0, 0.0]},
            ),
            sample(
                "px4_odometry",
                2_000,
                {"timestamp_sample_us": 2_000, "position": [10.0, 20.0, 30.0], "q_wxyz": yaw_quarter_turn, "velocity": [0.0, 1.0, 0.0], "velocity_frame": 1},
            ),
            sample(
                "px4_odometry",
                2_020,
                {"timestamp_sample_us": 2_020, "position": [10.0, 21.0, 30.0], "q_wxyz": yaw_quarter_turn, "velocity": [0.0, 1.0, 0.0], "velocity_frame": 1},
            ),
        ]
        result = report._residuals(samples, 20.0)
        self.assertEqual(result["source"], "lio/external_odometry_input vs px4/estimator_odometry")
        self.assertEqual(result["matched_sample_count"], 2)
        self.assertAlmostEqual(result["position"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["velocity"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["attitude"]["maximum"], 0.0, places=9)

    def test_report_verdict_set_is_closed(self) -> None:
        self.assertEqual(report.VERDICTS, {"PASS", "FAIL", "BLOCKED", "NOT_RUN", "OBSERVATION_COMPLETE"})

    def test_replay_tail_stale_events_are_ignored(self) -> None:
        row = {"stale_event_count": 3, "stale_event_times_ns": [700_000_000, 950_000_000, 1_100_000_000]}
        runtime = {"replay_finished_wall_ns": 1_000_000_000, "replay_tail_grace_s": 0.1}
        self.assertEqual(report._active_stale_count(row, runtime), 1)


if __name__ == "__main__":
    unittest.main()
