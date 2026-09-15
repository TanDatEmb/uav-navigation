import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

from evidence_writer import EvidenceWriter, snapshot_record


class EvidenceWriterTest(unittest.TestCase):
    def test_new_writer_replaces_a_preexisting_session_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            path.write_text('{"kind":"stale"}\n', encoding="utf-8")
            writer = EvidenceWriter(path, batch_size=1, flush_interval_s=0.01)
            self.assertTrue(writer.enqueue({"kind": "current"}))
            stats = writer.close()

            rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            self.assertTrue(stats.capture_complete)
            self.assertEqual([row["kind"] for row in rows], ["current"])

    def test_full_queue_drops_without_blocking_and_marks_capture_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            writer = EvidenceWriter(path, capacity=1, batch_size=1, flush_interval_s=0.01)
            # Hold the consumer after it takes its first batch so the producer
            # can deterministically exercise the non-blocking full path.
            original_write_batch = writer._write_batch
            consumer_started = threading.Event()

            def delayed_write_batch(stream, batch):
                consumer_started.set()
                time.sleep(0.05)
                original_write_batch(stream, batch)

            writer._write_batch = delayed_write_batch
            self.assertTrue(writer.enqueue({"index": 0}))
            self.assertTrue(consumer_started.wait(timeout=1.0))
            accepted = [writer.enqueue({"index": index}) for index in range(1, 32)]
            stats = writer.close()
            self.assertTrue(any(accepted))
            self.assertGreater(stats.dropped_records, 0)
            self.assertGreater(stats.queue_full_drop_records, 0)
            self.assertFalse(stats.capture_complete)
            rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual([row["index"] for row in rows], [0, 1])

    def test_snapshot_rejection_is_evidence_failure_not_producer_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = EvidenceWriter(Path(directory) / "events.jsonl")
            self.assertFalse(writer.enqueue({"bad": object()}))
            stats = writer.close()
            self.assertEqual(stats.snapshot_rejected_records, 1)
            self.assertEqual(stats.queue_full_drop_records, 0)
            self.assertEqual(stats.submitted_records, 1)
            self.assertEqual(stats.dropped_records, 1)
            self.assertEqual(stats.records_by_category["__unknown__"]["dropped_records"], 1)
            self.assertFalse(stats.capture_complete)

    def test_nested_mutation_after_enqueue_keeps_owned_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            writer = EvidenceWriter(path, batch_size=1, flush_interval_s=0.01)
            payload = {"nested": {"value": 1}, "values": [1], "tuple": (2, 3)}
            self.assertTrue(writer.enqueue(payload))
            payload["nested"]["value"] = 2
            payload["values"].append(2)
            payload["values"].clear()
            payload["tuple"] = (99,)
            stats = writer.close()
            self.assertTrue(stats.capture_complete)
            row = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(row["nested"], {"value": 1})
            self.assertEqual(row["values"], [1])
            self.assertEqual(row["tuple"], [2, 3])

    def test_shared_nested_reference_is_snapshotted_once_at_enqueue(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            writer = EvidenceWriter(path, batch_size=1, flush_interval_s=0.01)
            shared = {"value": 7}
            record = {"left": shared, "right": shared}
            self.assertTrue(writer.enqueue(record))
            shared["value"] = 8
            writer.close()
            row = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(row["left"], {"value": 7})
            self.assertEqual(row["right"], {"value": 7})

    def test_source_mutation_while_consumer_is_delayed_cannot_change_artifact(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            writer = EvidenceWriter(path, batch_size=1, flush_interval_s=0.01)
            started = threading.Event()
            release = threading.Event()
            original_write_batch = writer._write_batch

            def delayed_write_batch(stream, batch):
                started.set()
                release.wait(timeout=1.0)
                original_write_batch(stream, batch)

            writer._write_batch = delayed_write_batch
            source = {"nested": {"value": 1}, "values": [1]}
            self.assertTrue(writer.enqueue(source))
            self.assertTrue(started.wait(timeout=1.0))
            source["nested"]["value"] = 2
            source["values"].append(2)
            release.set()
            stats = writer.close()
            self.assertTrue(stats.capture_complete)
            row = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(row["nested"]["value"], 1)
            self.assertEqual(row["values"], [1])

    def test_serialization_error_has_its_own_accounting(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = EvidenceWriter(Path(directory) / "events.jsonl")
            with patch("evidence_writer.json.dumps", side_effect=TypeError("writer serialization")):
                self.assertTrue(writer.enqueue({"value": 1}))
                stats = writer.close()
            self.assertEqual(stats.serialization_error_count, 1)
            self.assertEqual(stats.snapshot_rejected_records, 0)
            self.assertEqual(stats.queue_full_drop_records, 0)
            self.assertFalse(stats.capture_complete)

    def test_category_counters_preserve_per_stream_denominators(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            writer = EvidenceWriter(path)
            self.assertTrue(writer.enqueue({"kind": "sample", "stream": "pva", "value": 1}))
            self.assertTrue(writer.enqueue({"kind": "lifecycle", "payload": {"phase": "request"}}))
            stats = writer.close()
            self.assertEqual(stats.submitted_records, 2)
            self.assertEqual(stats.accepted_records, 2)
            self.assertEqual(stats.written_records, 2)
            self.assertEqual(stats.dropped_records, 0)
            self.assertEqual(stats.records_by_category["sample:pva"], {
                "submitted_records": 1,
                "accepted_records": 1,
                "written_records": 1,
                "dropped_records": 0,
            })
            self.assertEqual(stats.records_by_category["lifecycle"]["written_records"], 1)

    def test_enqueue_after_close_is_counted_and_invalidates_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = EvidenceWriter(Path(directory) / "events.jsonl")
            self.assertTrue(writer.enqueue({"kind": "event", "value": 1}))
            self.assertTrue(writer.close().capture_complete)
            self.assertFalse(writer.enqueue({"kind": "event", "value": 2}))
            stats = writer.stats()
            self.assertEqual(stats.submitted_records, 2)
            self.assertEqual(stats.accepted_records, 1)
            self.assertEqual(stats.dropped_records, 1)
            self.assertEqual(stats.closed_rejected_records, 1)
            self.assertFalse(stats.capture_complete)

    def test_snapshot_record_rejects_nonfinite_and_unsupported_values(self):
        for record in ({"value": float("nan")}, {"value": object()}):
            with self.assertRaises(ValueError):
                snapshot_record(record)


if __name__ == "__main__":
    unittest.main()
