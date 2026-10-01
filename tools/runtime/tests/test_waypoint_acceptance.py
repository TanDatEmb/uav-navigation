import unittest

from tools.runtime.waypoint_acceptance import parse_waypoint_acceptance


class WaypointAcceptanceTests(unittest.TestCase):
    def test_acceptance_parser_fails_closed_for_missing_false_zero_and_true(self):
        self.assertEqual(parse_waypoint_acceptance({"accepted_waypoint_index": 0}), ("NOT_EVALUABLE", None))
        self.assertEqual(parse_waypoint_acceptance({"waypoint_accepted": False, "accepted_waypoint_index": 0}), ("REJECTED", None))
        self.assertEqual(parse_waypoint_acceptance({"waypoint_accepted": True, "accepted_waypoint_index": 0}), ("ACCEPTED", 0))
        self.assertEqual(parse_waypoint_acceptance({"waypoint_accepted": True}), ("NOT_EVALUABLE", None))


if __name__ == "__main__":
    unittest.main()
