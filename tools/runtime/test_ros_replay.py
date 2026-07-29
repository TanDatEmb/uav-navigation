#!/usr/bin/env python3

import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_replay import acceptance_failures, drained


class DrainTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
