import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from observer_core import EventLifecycle, ObserverState, StreamTracker, classify
from sim_observer import diagnostic_level


class ObserverCoreTest(unittest.TestCase):
    def test_diagnostic_level_int_and_bytes(self):
        self.assertEqual(diagnostic_level(2), 2)
        self.assertEqual(diagnostic_level(b"\x01"), 1)
        self.assertEqual(diagnostic_level(b""), 0)

    def test_zero_stamp_not_regression_and_regression_detected(self):
        stream = StreamTracker("/x", "T")
        stream.update(1, 10)
        stream.update(2, 0)
        stream.update(3, 9)
        self.assertEqual(stream.zero_stamp_count, 1)
        self.assertEqual(stream.stamp_regressions, 1)

    def test_short_window_recovers(self):
        stream = StreamTracker("/x", "T")
        for value in (0, 1, 2, 20, 20.1, 20.2):
            stream.update(value, int(value*1e9))
        self.assertGreater(stream.rate(20.2, 5), 9)
        self.assertLess(stream.rate(20.2, 30), 1)

    def test_fault_classifications(self):
        base = {"gazebo": {"raw_scan_sequence": 1}, "streams": {}, "diagnostics": {}, "system": {}}
        current = {"gazebo": {"clock_sequence": 2, "raw_scan_sequence": 1},
                   "streams": {"imu": {"message_count": 2}},
                   "diagnostics": {}, "system": {}}
        state, _ = classify(current, base)
        self.assertEqual(state, ObserverState.LIDAR_SOURCE_STALL)
        base_conversion = {"gazebo": {"raw_scan_sequence": 1, "pointcloud_sequence": 1},
                           "streams": {}, "diagnostics": {}, "system": {}}
        current = {"gazebo": {"raw_scan_sequence": 2, "pointcloud_sequence": 1},
                   "streams": {}, "diagnostics": {}, "system": {}}
        self.assertEqual(classify(current, base_conversion)[0], ObserverState.GAZEBO_POINT_CONVERSION_STALL)
        current = {"gazebo": {"pointcloud_sequence": 2},
                   "streams": {"lidar": {"message_count": 1}}, "diagnostics": {}, "system": {}}
        base_bridge = {"gazebo": {"pointcloud_sequence": 1},
                       "streams": {"lidar": {"message_count": 1}},
                       "diagnostics": {}, "system": {}}
        self.assertEqual(classify(current, base_bridge)[0], ObserverState.BRIDGE_STALL)

    def test_invalid_rejection_processing_and_output(self):
        old = {"gazebo": {}, "streams": {"lidar": {"message_count": 1},
               "odometry": {"message_count": 1}}, "diagnostics": {
               "accepted_imu_count": 1, "accepted_lidar_count": 1,
               "rejected_lidar_count": 0, "synchronized_group_count": 1,
               "correction_success_count": 1}, "system": {}}
        invalid = {**old, "pointcloud": {"finite_ratio": .01},
                   "finite_error_threshold": .1,
                   "streams": {**old["streams"], "lidar": {"message_count": 2}}}
        self.assertEqual(classify(invalid, old)[0], ObserverState.INVALID_POINTCLOUD)
        rejection = {**invalid, "pointcloud": {"finite_ratio": 1},
                     "diagnostics": {**old["diagnostics"], "rejected_lidar_count": 1}}
        self.assertEqual(classify(rejection, old)[0], ObserverState.FAST_LIO_INPUT_REJECTION)
        processing = {**rejection, "diagnostics": {**old["diagnostics"],
                      "accepted_imu_count": 2, "accepted_lidar_count": 2}}
        self.assertEqual(classify(processing, old)[0], ObserverState.FAST_LIO_PROCESSING_STALL)
        output = {**old, "diagnostics": {**old["diagnostics"], "correction_success_count": 2}}
        self.assertEqual(classify(output, old)[0], ObserverState.OUTPUT_STALL)

    def test_event_cooldown_and_recovery(self):
        events = EventLifecycle(30, 10)
        self.assertEqual(events.update("X", 0), [("X_ENTER", "entered")])
        self.assertEqual(events.update("X", 5), [])
        self.assertEqual(events.update("X", 31), [("X_PERSIST", "persists")])
        self.assertEqual(events.update(None, 32), [("X_RECOVERED", "recovered")])


if __name__ == "__main__":
    unittest.main()
