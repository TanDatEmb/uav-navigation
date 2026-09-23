import unittest

from tools.audit_observability.normalize import (attach_first_use, command_key,
    correlate_commands, final_drop_count, mission_hash, sequence_gaps)


def event(kind, seq, **kwargs):
    row = dict(event_type=kind, diagnostic_sequence=seq, producer_id=1,
               process_incarnation=123, localization_epoch=2, goal_epoch=3,
               mission_hash=4, waypoint_index=5, request_id=6,
               bundle_generation=7, sample_id=8, ros_now_ns=100,
               ros_return_ns=110)
    row.update(kwargs)
    return row


class NormalizationTest(unittest.TestCase):
    def test_key_includes_request_and_mission(self):
        a = event(2, 1)
        self.assertNotEqual(command_key(a), command_key(event(2, 2, request_id=7)))
        self.assertNotEqual(command_key(a), command_key(event(2, 2, mission_hash=9)))

    def test_mission_hash_matches_cpp_fnv(self):
        self.assertEqual(mission_hash(""), 14695981039346656037)
        self.assertEqual(mission_hash("a"), 12638187200555641996)

    def test_sequence_gap_is_not_interpolated(self):
        self.assertEqual(sequence_gaps([event(1, 1), event(1, 3)]), [
            dict(producer_id=1, process_incarnation=123, first_missing=2, last_missing=2)])

    def test_restart_has_separate_sequence(self):
        self.assertEqual(sequence_gaps([event(1, 1), event(1, 1, process_incarnation=456)]), [])

    def test_drop_counter_is_cumulative(self):
        self.assertEqual(final_drop_count([event(7, 1, dropped_count=2),
                                           event(7, 2, dropped_count=3)]), 3)

    def test_no_latency_without_clock_proof(self):
        pair = correlate_commands([event(2, 1), event(3, 2, ros_now_ns=120)])[0]
        self.assertEqual(pair["status"], "PAIRED_IDENTITY_ONLY")
        self.assertNotIn("ros_publish_enter_to_receive_ns", pair)

    def test_duplicate_key_is_not_pair(self):
        pair = correlate_commands([event(2, 1), event(2, 2),
                                   event(3, 3, ros_now_ns=120)])[0]
        self.assertEqual(pair["status"], "AMBIGUOUS_OR_UNPAIRED")

    def test_ros_pair_needs_explicit_proof(self):
        pair = correlate_commands([event(2, 1), event(3, 2, ros_now_ns=120)],
                                  "shared_sim_clock")[0]
        self.assertEqual(pair["ros_publish_enter_to_receive_ns"], 20)
        self.assertEqual(pair["ros_publish_return_to_receive_ns"], 10)

    def test_first_use_is_exact_key_after_receive(self):
        pair = correlate_commands([event(2, 1), event(3, 2, ros_now_ns=120)],
                                  "shared_sim_clock")
        use = dict(zip(("localization_epoch", "goal_epoch", "mission_hash",
                        "waypoint_index", "request_id", "bundle_generation", "sample_id"),
                       command_key(event(2, 1))))
        use.update(update_start_ros_ns=130, setpoint_boundary="tracking",
                   trace_sequence=5, trace_drop_count=0)
        result = attach_first_use(pair, [use], "shared_sim_clock")[0]
        self.assertEqual(result["ros_receive_to_first_use_ns"], 10)


if __name__ == "__main__":
    unittest.main()
