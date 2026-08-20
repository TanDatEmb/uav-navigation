import sys
import unittest
from pathlib import Path

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

from html_report import _trajectory_smoothness


class HtmlReportSmoothnessTest(unittest.TestCase):
    def test_overlapping_plans_are_compared_at_handover_time(self) -> None:
        # Each plan is three seconds long but is replaced after 0.2 seconds.
        # The old implementation compared the old plan's terminal velocity to
        # the new plan's first velocity and reported a false discontinuity.
        records = [
            {
                "_publish_time_s": 10.0,
                "duration_s": 3.0,
                "position_points": [[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
                "velocity_points": [[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            },
            {
                "_publish_time_s": 10.2,
                "duration_s": 3.0,
                "position_points": [[0.2, 0.0, 0.0], [3.2, 0.0, 0.0]],
                "velocity_points": [[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            },
        ]

        metrics, series = _trajectory_smoothness(records)

        self.assertTrue(metrics["timeline_time_anchored"])
        self.assertEqual(metrics["handover_sample_count"], 1)
        self.assertAlmostEqual(metrics["boundary_velocity_jump_mps"]["maximum"], 0.0)
        self.assertAlmostEqual(metrics["boundary_position_jump_m"]["maximum"], 0.0)
        self.assertEqual(len(series["speed"]), 2)
        self.assertEqual(series["speed"][1][0], 10.2)


if __name__ == "__main__":
    unittest.main()
