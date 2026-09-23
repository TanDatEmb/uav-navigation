"""Audit-only policy split; does not assert product equivalence."""
import unittest


class Brake:
    def __init__(self):
        self.state = "tracking"

    def event(self, event, certified=False, continuous=False, valid_lease=False):
        if event == "uncertainty" and self.state == "tracking":
            self.state = "recoverable"
        elif event == "commit_stop" and self.state in ("tracking", "recoverable"):
            self.state = "committed"
        elif event == "nominal" and self.state == "recoverable" and certified and continuous and valid_lease:
            self.state = "tracking"
        elif event == "measured_stop" and self.state == "committed":
            self.state = "stopped"
        elif event == "nominal" and self.state == "stopped" and certified and valid_lease:
            self.state = "tracking"


class BrakingModel(unittest.TestCase):
    def test_source_transitions_differ_without_product_reachability(self):
        # Abstract the two exact local source transitions. The onTrajectory
        # API is not invoked by this repository's product adapter call graph.
        adapter_state = "Braking"
        runtime_state = "TrackBackup"
        measured_stop = False
        non_stop_success = True
        if adapter_state == "Braking" and non_stop_success:
            adapter_state = "ExecutingWaypoint"
        if runtime_state == "TrackBackup" and not measured_stop:
            runtime_state = "TrackBackup"
        self.assertEqual((adapter_state, runtime_state), ("ExecutingWaypoint", "TrackBackup"))

    def test_recoverable_requires_all_witnesses(self):
        for certified in (False, True):
            for continuous in (False, True):
                for valid_lease in (False, True):
                    m = Brake()
                    m.event("uncertainty")
                    m.event("nominal", certified, continuous, valid_lease)
                    self.assertEqual(m.state == "tracking", certified and continuous and valid_lease)

    def test_committed_is_one_way_until_stop(self):
        m = Brake()
        for event in ("uncertainty", "commit_stop", "nominal", "new_world", "nominal"):
            m.event(event, True, True, True)
        self.assertEqual(m.state, "committed")
        m.event("measured_stop")
        self.assertEqual(m.state, "stopped")

    def test_role_not_lifecycle(self):
        m = Brake()
        m.event("uncertainty")
        self.assertEqual(m.state, "recoverable")
        # The abstract sample role does not transition the global lifecycle.
        role = "BACKUP"
        self.assertEqual(role, "BACKUP")
        self.assertEqual(m.state, "recoverable")


if __name__ == "__main__":
    unittest.main()
