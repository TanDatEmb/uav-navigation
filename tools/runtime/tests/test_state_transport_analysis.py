import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from state_transport_analysis import analyze_session, classify_gap


class StateTransportAnalysisTest(unittest.TestCase):
    def test_sim_clock_stall_is_not_called_dds_delay(self):
        result = classify_gap(
            source_period_ms=4.0, accepted_gap_ms=208.583,
            producer_gap_ms=210.0, publish_to_callback_ms=1.0,
            mutex_wait_ms=0.01, rejected_between=0,
            clock_gap_ms=1150.256, clock_source_delta_ms=4.0)
        self.assertEqual(result, "SIM_TIME_STALL_OR_SLOWDOWN")

    def test_independent_causes_remain_separate(self):
        base = dict(source_period_ms=20.0, accepted_gap_ms=180.0,
                    producer_gap_ms=20.0, publish_to_callback_ms=2.0,
                    mutex_wait_ms=0.01, rejected_between=0,
                    clock_gap_ms=None, clock_source_delta_ms=None)
        self.assertEqual(classify_gap(**(base | {"rejected_between": 3})),
                         "ADAPTER_SEMANTIC_REJECTION")
        self.assertEqual(classify_gap(**(base | {"mutex_wait_ms": 155.0})),
                         "ADAPTER_MUTEX_CONTENTION")
        self.assertEqual(classify_gap(**(base | {"producer_gap_ms": 175.0})),
                         "PRODUCER_PUBLICATION_GAP")
        self.assertEqual(classify_gap(**(base | {"publish_to_callback_ms": 155.0})),
                         "DDS_OR_EXECUTOR_DELIVERY_DELAY")
        self.assertEqual(classify_gap(**base), "UNKNOWN")

    def test_rejected_callback_does_not_advance_accepted_receive(self):
        with tempfile.TemporaryDirectory() as temp:
            session = Path(temp)
            rows = [
                {"stream": "odometry_producer_trace", "payload": {
                    "localization_epoch": 1, "sequence": n,
                    "source_stamp_ros_ns": n * 20_000_000,
                    "publisher_publish_call_steady_ns": n * 20_000_000,
                }} for n in (1, 2, 3)
            ] + [
                {"stream": "odometry_adapter_ingress_trace", "payload": {
                    "localization_epoch": 1, "sequence": n,
                    "disposition": disposition,
                    "source_stamp_ros_ns": n * 20_000_000,
                    "callback_enter_steady_ns": steady,
                    "lock_requested_steady_ns": steady,
                    "lock_acquired_steady_ns": steady + 1000,
                    "accepted_receive_steady_ns": steady + 2000 if disposition == 1 else 0,
                }} for n, disposition, steady in (
                    (1, 1, 20_001_000), (2, 8, 40_001_000),
                    (3, 1, 220_001_000))
            ]
            (session / "samples.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            (session / "monitor.json").write_text(json.dumps({"streams": {}}), encoding="utf-8")
            result = analyze_session(session)
            self.assertEqual(result["callback_count"], 3)
            self.assertEqual(result["accepted_count"], 2)
            self.assertEqual(result["rejected_count"], 1)
            self.assertEqual(result["dispositions"]["SOURCE_TIMESTAMP_NON_INCREASING"], 1)
            self.assertEqual(result["maximum_accepted_receive_gap_ms"], 200.0)
            self.assertEqual(result["tails_over_100_ms"][0]["rejected_between"], 1)
            self.assertEqual(result["tails_over_100_ms"][0]["class"],
                             "ADAPTER_SEMANTIC_REJECTION")


if __name__ == "__main__":
    unittest.main()
