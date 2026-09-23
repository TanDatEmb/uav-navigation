"""Audit-only event-order model; status freshness is explicit."""
import itertools
import unittest


class Hold:
    def __init__(self):
        self.requested = False
        self.ack = None
        self.completed = None
        self.status = None
        self.deactivated = None
        self.deadline = 100

    def event(self, event):
        if event == "request": self.requested = True
        elif event == "ack": self.ack = "accepted"
        elif event == "reject": self.ack = "rejected"
        elif event == "complete": self.completed = "success"
        elif event == "deactivated": self.deactivated = "other"
        elif event == "shutdown": self.deactivated = "shutdown"
        elif event == "operator": self.deactivated = "operator"
        elif event == "failsafe": self.deactivated = "failsafe"
        elif event == "status": self.status = ("AUTO_LOITER", 101, "released")
        elif event == "stale_status": self.status = ("AUTO_LOITER", 99, "released")
        elif event == "status_still_owned": self.status = ("AUTO_LOITER", 101, "owned")

    def confirmed(self):
        # Hypothetical approved witness: fresh post-request mode plus released executor.
        # The firmware contract for this tuple remains a specification gap.
        return self.requested and self.deactivated is None and self.status == ("AUTO_LOITER", 101, "released")

    def retry_due(self):
        return self.requested and not self.confirmed() and self.deactivated is None


class HoldModel(unittest.TestCase):
    def test_target_callback_counterexample(self):
        # TARGET onPx4HoldHandoverCompleted(Success) clears pending/inflight;
        # checkHoldHandover retries only when pending and not confirmed.
        pending, in_flight, confirmed = True, True, False
        result = "Success"
        if result in ("Success", "Deactivated"):
            pending, in_flight = False, False
        self.assertFalse(confirmed)
        self.assertFalse(pending and not confirmed and not in_flight)

    def test_target_inflight_without_callback_or_status_cannot_retry(self):
        pending, in_flight, confirmed = True, True, False
        self.assertFalse(pending and not confirmed and not in_flight)

    def test_status_and_completion_orders(self):
        for order in itertools.permutations(("ack", "status", "complete")):
            h = Hold()
            h.event("request")
            for event in order: h.event(event)
            self.assertTrue(h.confirmed(), order)

    def test_completion_without_status_keeps_retry(self):
        h = Hold()
        for event in ("request", "ack", "complete"): h.event(event)
        self.assertFalse(h.confirmed())
        self.assertTrue(h.retry_due())

    def test_stale_status_is_not_authority(self):
        h = Hold()
        for event in ("stale_status", "request", "ack"): h.event(event)
        self.assertFalse(h.confirmed())

    def test_status_without_executor_release_is_not_authority(self):
        h = Hold()
        for event in ("request", "ack", "status_still_owned"):
            h.event(event)
        self.assertFalse(h.confirmed())

    def test_takeover_not_hold_success(self):
        for cause in ("operator", "failsafe", "deactivated", "shutdown"):
            h = Hold()
            for event in ("request", "ack", cause): h.event(event)
            self.assertFalse(h.confirmed())
            self.assertFalse(h.retry_due())
            self.assertEqual(h.deactivated, cause if cause != "deactivated" else "other")

    def test_duplicate_status_does_not_change_result(self):
        h = Hold()
        for event in ("request", "ack", "status", "status"):
            h.event(event)
        self.assertTrue(h.confirmed())

    def test_status_before_completion_after_request(self):
        h = Hold()
        for event in ("request", "status", "complete"):
            h.event(event)
        self.assertTrue(h.confirmed())

    def test_rejected_not_confirmed(self):
        h = Hold()
        for event in ("request", "reject"): h.event(event)
        self.assertTrue(h.retry_due())


if __name__ == "__main__":
    unittest.main()
