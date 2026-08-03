"""ROS-independent accounting for P0.8 supervisor qualification metrics."""

from __future__ import annotations

from typing import Any


class StatusEventAccumulator:
    """Account for every supervisor status event in the measurement window."""

    COUNTERS = (
        "query_sequence",
        "query_success_count",
        "query_failure_count",
        "query_timeout_count",
        "query_generation_mismatch_count",
        "query_service_unavailable_count",
        "query_invalid_component_count",
        "query_stale_sequence_count",
        "reinitialization_request_sequence",
        "state_transition_count",
    )

    def __init__(self) -> None:
        self.start_ns: int | None = None
        self.end_ns: int | None = None
        self.baseline: dict[str, int] = {}
        self.last_status: Any | None = None
        self.status_count = 0
        self.comparison_valid_count = 0
        self.monitoring_available_count = 0
        self.new_comparison_sample_count = 0
        self.aligned_comparison_fresh_count = 0
        self.reason_histogram: dict[str, int] = {}
        self.health_histogram: dict[str, int] = {}
        self.invalid_since_ns: dict[str, int | None] = {
            "comparison_valid": None,
            "monitoring_available": None,
        }
        self.max_invalid_duration_ns = {
            "comparison_valid": 0,
            "monitoring_available": 0,
        }

    def start(self, start_ns: int, end_ns: int, baseline_status: Any | None) -> None:
        self.start_ns = start_ns
        self.end_ns = end_ns
        self.baseline = {
            name: int(getattr(baseline_status, name))
            for name in self.COUNTERS
            if baseline_status is not None and hasattr(baseline_status, name)
        }

    def _update_invalid_duration(self, name: str, valid: bool, event_ns: int) -> None:
        if valid:
            start = self.invalid_since_ns[name]
            if start is not None:
                self.max_invalid_duration_ns[name] = max(
                    self.max_invalid_duration_ns[name], max(0, event_ns - start))
                self.invalid_since_ns[name] = None
        elif self.invalid_since_ns[name] is None:
            self.invalid_since_ns[name] = event_ns

    def record(self, status: Any, event_ns: int) -> None:
        if self.start_ns is None or self.end_ns is None:
            return
        if event_ns < self.start_ns or event_ns > self.end_ns:
            return
        self.last_status = status
        self.status_count += 1
        self.comparison_valid_count += int(bool(status.comparison_valid))
        self.monitoring_available_count += int(bool(status.monitoring_available))
        self.new_comparison_sample_count += int(bool(status.new_comparison_sample))
        self.aligned_comparison_fresh_count += int(bool(status.aligned_comparison_fresh))
        self.reason_histogram[status.reason] = self.reason_histogram.get(status.reason, 0) + 1
        health = str(int(status.health))
        self.health_histogram[health] = self.health_histogram.get(health, 0) + 1
        self._update_invalid_duration("comparison_valid", bool(status.comparison_valid), event_ns)
        self._update_invalid_duration("monitoring_available", bool(status.monitoring_available), event_ns)

    def _close_invalid_windows(self, end_ns: int) -> None:
        for name, start in self.invalid_since_ns.items():
            if start is not None:
                self.max_invalid_duration_ns[name] = max(
                    self.max_invalid_duration_ns[name], max(0, end_ns - start))
                self.invalid_since_ns[name] = None

    def _counter_delta(self, name: str, status: Any | None) -> int | None:
        if status is None or name not in self.baseline:
            return None
        return max(0, int(getattr(status, name)) - self.baseline[name])

    def finish(self, end_ns: int) -> dict[str, Any]:
        self._close_invalid_windows(end_ns)
        status = self.last_status
        result: dict[str, Any] = {
            "status_event_count": self.status_count,
            "comparison_valid_count": self.comparison_valid_count,
            "monitoring_available_count": self.monitoring_available_count,
            "comparison_valid_ratio": (
                self.comparison_valid_count / self.status_count if self.status_count else None
            ),
            "monitoring_available_ratio": (
                self.monitoring_available_count / self.status_count if self.status_count else None
            ),
            "new_comparison_sample_count": self.new_comparison_sample_count,
            "aligned_comparison_fresh_count": self.aligned_comparison_fresh_count,
            "reason_histogram": self.reason_histogram,
            "health_histogram": self.health_histogram,
            "comparison_valid_false_max_duration_ms":
                self.max_invalid_duration_ns["comparison_valid"] / 1e6,
            "monitoring_available_false_max_duration_ms":
                self.max_invalid_duration_ns["monitoring_available"] / 1e6,
        }
        for name in self.COUNTERS:
            result[f"{name}_delta"] = self._counter_delta(name, status)
        return result
