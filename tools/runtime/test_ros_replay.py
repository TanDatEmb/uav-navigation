#!/usr/bin/env python3

import unittest
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_replay import acceptance_failures, diagnostics_values, drained


class DrainTest(unittest.TestCase):
    def test_parses_jazzy_byte_encoded_diagnostic_level(self) -> None:
        message = SimpleNamespace(status=[
            SimpleNamespace(
                name="fast_lio/transport",
                level=b"\x01",
                message="lag",
                values=[SimpleNamespace(key="imu_drop_count", value="0")],
            )
        ])
        self.assertEqual(
            diagnostics_values(message),
            {"level": 1, "message": "lag", "imu_drop_count": 0},
        )

    def test_requires_zero_queues_and_converged_counters(self) -> None:
        state = {
            "current_input_queue_depth": 0,
            "current_imu_queue_depth": 0,
            "current_lidar_queue_depth": 0,
            "received_imu_count": 10,
            "processed_imu_count": 10,
            "received_lidar_count": 2,
            "processed_lidar_count": 2,
        }
        self.assertTrue(drained(state))
        state["processed_lidar_count"] = 1
        self.assertFalse(drained(state))

    def test_overflow_and_drop_fail_acceptance(self) -> None:
        state = {
            "current_input_queue_depth": 0,
            "current_imu_queue_depth": 0,
            "current_lidar_queue_depth": 0,
            "received_imu_count": 10,
            "processed_imu_count": 10,
            "received_lidar_count": 2,
            "processed_lidar_count": 2,
            "overflow_detected": True,
            "imu_drop_count": 1,
            "lidar_drop_count": 0,
        }
        self.assertEqual(
            acceptance_failures(state),
            ["input queue overflow detected", "IMU messages dropped"],
        )

    def test_processing_lag_limit_fails_acceptance(self) -> None:
        state = {
            "current_input_queue_depth": 0,
            "current_imu_queue_depth": 0,
            "current_lidar_queue_depth": 0,
            "received_imu_count": 1,
            "processed_imu_count": 1,
            "received_lidar_count": 1,
            "processed_lidar_count": 1,
            "processing_lag_exceeded": True,
        }
        self.assertEqual(
            acceptance_failures(state), ["maximum processing lag exceeded"]
        )


if __name__ == "__main__":
    unittest.main()
