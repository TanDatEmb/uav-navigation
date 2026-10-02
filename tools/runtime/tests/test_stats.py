import sys
import unittest
from pathlib import Path

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

from stats import percentile


class StatsTest(unittest.TestCase):
    def test_percentile_keeps_acceptance_nearest_index_definition(self):
        self.assertEqual(percentile([0.1] * 10 + [5.0], 0.95), 5.0)
        self.assertEqual(percentile([1.0, 2.0], 0.5), 1.0)
        self.assertEqual(percentile([1.0, 2.0, 3.0, 4.0], 0.5), 3.0)


if __name__ == "__main__":
    unittest.main()
