#!/usr/bin/env python3
import unittest

from tools.refactor.check_trace_corpus import validate_corpus


def decision_record(site_id="RT-001"):
    return {
        "schema_version": "p4-0.v1",
        "corpus": "K2",
        "site_id": site_id,
        "event": {"kind": "CommandTick", "payload": {"now_ns": 1000}},
        "pre_state": {"phase": "TrackingMain", "epoch": 7},
        "trigger": {"source": "timer", "payload": {"now_ns": 1000}},
        "predicates": [{"name": "commandAnchorRecoveryDue", "args": {"age_ns": 1}, "result": False}],
        "effects": [{"type": "NO_OP"}],
        "post_state": {"phase": "TrackingMain", "epoch": 7},
        "dropped": 0,
    }


class TraceCorpusValidationTest(unittest.TestCase):
    def test_complete_decision_record_covers_expected_site(self):
        report = validate_corpus([decision_record()], {"RT-001"}, {})
        self.assertTrue(report.ok, report.errors)
        self.assertEqual(report.covered_sites, {"RT-001"})

    def test_missing_decision_field_is_rejected(self):
        record = decision_record()
        del record["pre_state"]
        report = validate_corpus([record], {"RT-001"}, {})
        self.assertFalse(report.ok)
        self.assertIn("record[0].pre_state: missing", report.errors)

    def test_ring_buffer_drop_is_rejected(self):
        record = decision_record()
        record["dropped"] = 1
        report = validate_corpus([record], {"RT-001"}, {})
        self.assertFalse(report.ok)
        self.assertIn("record[0].dropped: 1", report.errors)

    def test_unreached_site_requires_reason(self):
        report = validate_corpus([], {"RT-001"}, {"RT-001": "not reached by this fixture"})
        self.assertTrue(report.ok, report.errors)


if __name__ == "__main__":
    unittest.main()
