"""Bounded, best-effort JSONL evidence writer.

The writer intentionally owns no ROS objects and has no control semantics.  A
callback may enqueue a small already-built record and continue immediately;
serialization, file I/O and flushing happen on a low-priority helper thread.
Queue overflow is evidence loss, never back-pressure for the producer.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import queue
import threading
import time
from typing import Any


# Evidence records are normalized, small envelopes. Raw ROS messages and
# point clouds must be summarized before they reach this queue.
MAX_SNAPSHOT_DEPTH = 8
MAX_SNAPSHOT_BYTES = 256 * 1024
MAX_COUNTER_CATEGORIES = 128
MAX_COUNTER_CATEGORY_LENGTH = 96


class SnapshotError(ValueError):
    """The producer did not hand the writer a supported evidence envelope."""


def _snapshot_value(value: Any, *, depth: int, budget: list[int]) -> Any:
    if depth > MAX_SNAPSHOT_DEPTH:
        raise SnapshotError("snapshot_depth_exceeded")
    if value is None or isinstance(value, (bool, int, str)):
        cost = len(value.encode("utf-8")) if isinstance(value, str) else 8
        budget[0] -= cost
        if budget[0] < 0:
            raise SnapshotError("snapshot_size_exceeded")
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise SnapshotError("nonfinite_number")
        budget[0] -= 8
        if budget[0] < 0:
            raise SnapshotError("snapshot_size_exceeded")
        return value
    if isinstance(value, dict):
        result: dict[str, Any] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise SnapshotError("mapping_key_not_string")
            budget[0] -= len(key.encode("utf-8")) + 4
            if budget[0] < 0:
                raise SnapshotError("snapshot_size_exceeded")
            result[key] = _snapshot_value(item, depth=depth + 1, budget=budget)
        return result
    if isinstance(value, (list, tuple)):
        result = []
        for item in value:
            budget[0] -= 1
            if budget[0] < 0:
                raise SnapshotError("snapshot_size_exceeded")
            result.append(_snapshot_value(item, depth=depth + 1, budget=budget))
        return result
    raise SnapshotError(f"unsupported_type:{type(value).__name__}")


def snapshot_record(record: dict[str, Any]) -> dict[str, Any]:
    """Create the writer-owned snapshot before returning from ``enqueue``."""
    if not isinstance(record, dict):
        raise SnapshotError("record_not_mapping")
    return _snapshot_value(record, depth=0, budget=[MAX_SNAPSHOT_BYTES])


def _record_category(record: dict[str, Any]) -> str:
    """Return a bounded attribution key for recorder accounting."""
    kind = record.get("kind") if isinstance(record, dict) else None
    stream = record.get("stream") if isinstance(record, dict) else None
    if isinstance(kind, str) and kind:
        category = f"{kind}:{stream}" if kind == "sample" and isinstance(stream, str) and stream else kind
    else:
        category = "__unknown__"
    return category[:MAX_COUNTER_CATEGORY_LENGTH]


@dataclass(frozen=True)
class EvidenceWriterStats:
    submitted_records: int
    accepted_records: int
    written_records: int
    pending_records: int
    dropped_records: int
    snapshot_rejected_records: int
    queue_full_drop_records: int
    closed_rejected_records: int
    serialization_error_count: int
    write_error_count: int
    records_by_category: dict[str, dict[str, int]]
    capture_complete: bool
    error: str | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "submitted_records": self.submitted_records,
            "accepted_records": self.accepted_records,
            "written_records": self.written_records,
            "pending_records": self.pending_records,
            "dropped_records": self.dropped_records,
            "snapshot_rejected_records": self.snapshot_rejected_records,
            "queue_full_drop_records": self.queue_full_drop_records,
            "closed_rejected_records": self.closed_rejected_records,
            "serialization_error_count": self.serialization_error_count,
            "write_error_count": self.write_error_count,
            "records_by_category": self.records_by_category,
            "capture_complete": self.capture_complete,
            "error": self.error,
        }


class EvidenceWriter:
    """Write JSON objects to a bounded JSONL queue without blocking producers."""

    def __init__(
        self,
        path: Path,
        *,
        capacity: int = 4096,
        batch_size: int = 128,
        flush_interval_s: float = 0.05,
    ) -> None:
        if capacity <= 0:
            raise ValueError("evidence writer capacity must be positive")
        if batch_size <= 0:
            raise ValueError("evidence writer batch_size must be positive")
        if not flush_interval_s > 0.0:
            raise ValueError("evidence writer flush_interval_s must be positive")
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.capacity = int(capacity)
        self.batch_size = int(batch_size)
        self.flush_interval_s = float(flush_interval_s)
        self._queue: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=self.capacity)
        self._lock = threading.Lock()
        self._sequence = 0
        self._submitted = 0
        self._accepted = 0
        self._written = 0
        self._dropped = 0
        self._snapshot_rejected = 0
        self._queue_full_drops = 0
        self._closed_rejected = 0
        self._serialization_errors = 0
        self._write_errors = 0
        self._records_by_category: dict[str, dict[str, int]] = {}
        self._error: str | None = None
        self._closing = False
        self._complete = False
        self._thread = threading.Thread(
            target=self._run,
            name=f"evidence-writer-{self.path.name}",
            daemon=True,
        )
        self._thread.start()

    def _bump_category_locked(self, category: str, field: str) -> None:
        if category not in self._records_by_category:
            if len(self._records_by_category) >= MAX_COUNTER_CATEGORIES:
                category = "__other__"
            self._records_by_category.setdefault(category, {
                "submitted_records": 0,
                "accepted_records": 0,
                "written_records": 0,
                "dropped_records": 0,
            })
        self._records_by_category[category][field] += 1

    def enqueue(
        self,
        record: dict[str, Any],
        *,
        observer_record_steady_ns: int | None = None,
    ) -> bool:
        """Try to enqueue one complete envelope and return whether it was accepted."""
        category = _record_category(record)
        try:
            snapshot = snapshot_record(record)
        except Exception as error:
            # Telemetry must never throw back into the product callback.
            with self._lock:
                self._submitted += 1
                self._snapshot_rejected += 1
                self._dropped += 1
                self._error = f"snapshot_rejected:{error}"
                self._complete = False
                self._bump_category_locked(category, "submitted_records")
                self._bump_category_locked(category, "dropped_records")
            return False
        with self._lock:
            self._submitted += 1
            self._bump_category_locked(category, "submitted_records")
            if self._closing:
                self._closed_rejected += 1
                self._dropped += 1
                self._error = self._error or "enqueue_rejected:writer_closed"
                self._complete = False
                self._bump_category_locked(category, "dropped_records")
                return False
            self._sequence += 1
            envelope = snapshot
            envelope.setdefault("record_sequence", self._sequence)
            envelope.setdefault(
                "observer_record_steady_ns",
                int(observer_record_steady_ns)
                if observer_record_steady_ns is not None
                else time.monotonic_ns(),
            )
            try:
                # Keep the close transition ordered after this non-blocking
                # enqueue.  This prevents a shutdown sentinel from overtaking
                # a producer that already owns an accepted record.
                self._queue.put_nowait(envelope)
            except queue.Full:
                self._queue_full_drops += 1
                self._dropped += 1
                self._error = self._error or "queue_full_drop"
                self._complete = False
                self._bump_category_locked(category, "dropped_records")
                return False
            self._accepted += 1
            self._bump_category_locked(category, "accepted_records")
        return True

    def _run(self) -> None:
        stream = None
        try:
            # One writer instance owns one session artifact. Reusing an output
            # path must start a new capture rather than splice two sessions.
            stream = self.path.open("w", encoding="utf-8")
            while True:
                try:
                    first = self._queue.get(timeout=self.flush_interval_s)
                except queue.Empty:
                    with self._lock:
                        if self._closing:
                            break
                    continue
                if first is None:
                    break
                batch = [first]
                while len(batch) < self.batch_size:
                    try:
                        item = self._queue.get_nowait()
                    except queue.Empty:
                        break
                    if item is None:
                        # Drain the records already acquired before shutdown.
                        self._write_batch(stream, batch)
                        return
                    batch.append(item)
                self._write_batch(stream, batch)
            while True:
                try:
                    item = self._queue.get_nowait()
                except queue.Empty:
                    break
                if item is not None:
                    self._write_batch(stream, [item])
        except (OSError, RuntimeError, ValueError) as error:
            with self._lock:
                self._write_errors += 1
                self._error = f"{type(error).__name__}: {error}"
        finally:
            if stream is not None:
                try:
                    stream.flush()
                    stream.close()
                except (OSError, ValueError) as error:
                    with self._lock:
                        self._write_errors += 1
                        self._error = f"{type(error).__name__}: {error}"
            with self._lock:
                self._complete = (
                    self._error is None
                    and self._dropped == 0
                    and self._accepted == self._written
                )

    def _write_batch(self, stream: Any, batch: list[dict[str, Any]]) -> None:
        lines: list[str] = []
        written_categories: list[str] = []
        for item in batch:
            try:
                lines.append(json.dumps(item, sort_keys=True, allow_nan=False) + "\n")
                written_categories.append(_record_category(item))
            except (TypeError, ValueError) as error:
                with self._lock:
                    self._serialization_errors += 1
                    self._error = f"{type(error).__name__}: {error}"
                continue
        if not lines:
            return
        try:
            stream.writelines(lines)
            stream.flush()
        except (OSError, ValueError) as error:
            with self._lock:
                self._write_errors += 1
                self._error = f"{type(error).__name__}: {error}"
            return
        with self._lock:
            self._written += len(lines)
            for category in written_categories:
                self._bump_category_locked(category, "written_records")

    def stats(self) -> EvidenceWriterStats:
        with self._lock:
            return EvidenceWriterStats(
                submitted_records=self._submitted,
                accepted_records=self._accepted,
                written_records=self._written,
                pending_records=max(0, self._accepted - self._written),
                dropped_records=self._dropped,
                snapshot_rejected_records=self._snapshot_rejected,
                queue_full_drop_records=self._queue_full_drops,
                closed_rejected_records=self._closed_rejected,
                serialization_error_count=self._serialization_errors,
                write_error_count=self._write_errors,
                records_by_category={
                    category: dict(counters)
                    for category, counters in self._records_by_category.items()
                },
                capture_complete=self._complete,
                error=self._error,
            )

    def close(self) -> EvidenceWriterStats:
        with self._lock:
            if not self._closing:
                self._closing = True
                try:
                    self._queue.put_nowait(None)
                except queue.Full:
                    # A sentinel must not displace evidence. The worker will
                    # drain naturally after producers stop. No record was
                    # dropped by the missing sentinel itself.
                    pass
        self._thread.join(timeout=max(1.0, self.flush_interval_s * 20.0))
        if self._thread.is_alive():
            with self._lock:
                self._write_errors += 1
                self._error = "writer thread did not shut down before timeout"
                self._complete = False
        return self.stats()
