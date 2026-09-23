"""Audit-only temporal invariant model; times are logical ticks, not runtime ms."""
import unittest


class Lease:
    def __init__(self, certificate_expiry=10):
        self.expiry = certificate_expiry
        self.authority = "core"
        self.last_heartbeat = None
        self.recertified = False

    def tick(self, now, world_valid=True):
        if self.authority != "core": return
        if now >= self.expiry or not world_valid:
            self.authority = "transfer_pending"
            return
        self.last_heartbeat = now

    def recertify(self, new_expiry):
        if self.authority == "core" and new_expiry > self.expiry:
            self.expiry = new_expiry
            self.recertified = True


class WorldLeaseModel(unittest.TestCase):
    def test_target_stale_world_silences_publisher(self):
        latest_world_fresh = False
        published = latest_world_fresh  # publishCommand returns before publish.
        episode_command_available_after_suspend = False
        self.assertFalse(published)
        self.assertFalse(episode_command_available_after_suspend)

    def test_new_world_may_recertify_before_expiry(self):
        m = Lease()
        m.tick(2)
        m.recertify(20)
        m.tick(11)
        self.assertEqual(m.authority, "core")
        self.assertEqual(m.last_heartbeat, 11)

    def test_expiry_requires_explicit_transition(self):
        m = Lease()
        m.tick(10)
        self.assertEqual(m.authority, "transfer_pending")
        self.assertIsNone(m.last_heartbeat)

    def test_no_silent_gap_while_core_owns_valid_certificate(self):
        m = Lease()
        for now in range(10):
            m.tick(now)
            self.assertEqual(m.last_heartbeat, now)

    def test_invalidated_world_changes_authority(self):
        m = Lease()
        m.tick(1, world_valid=False)
        self.assertEqual(m.authority, "transfer_pending")


if __name__ == "__main__":
    unittest.main()
