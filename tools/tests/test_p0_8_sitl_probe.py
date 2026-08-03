from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "performance"))
from p0_8_probe_metrics import StatusEventAccumulator


def status(*, comparison_valid: bool, monitoring_available: bool,
           query_timeout_count: int = 2) -> SimpleNamespace:
    return SimpleNamespace(
        comparison_valid=comparison_valid,
        monitoring_available=monitoring_available,
        new_comparison_sample=True,
        aligned_comparison_fresh=comparison_valid,
        reason="HEALTHY" if comparison_valid else "ALIGNED_COMPARISON_STALE",
        health=1,
        query_sequence=100,
        query_success_count=98,
        query_failure_count=2,
        query_timeout_count=query_timeout_count,
        query_generation_mismatch_count=0,
        query_service_unavailable_count=0,
        query_invalid_component_count=0,
        query_stale_sequence_count=0,
        reinitialization_request_sequence=0,
        state_transition_count=1,
    )


class P08StatusEventAccumulatorTest(unittest.TestCase):
    def test_ratio_uses_all_status_events_not_one_hz_snapshots(self):
        accumulator = StatusEventAccumulator()
        accumulator.start(0, 10_000_000_000, status(
            comparison_valid=True, monitoring_available=True))
        for index in range(400):
            valid = index < 396
            accumulator.record(status(
                comparison_valid=valid, monitoring_available=valid), index * 20_000_000)

        result = accumulator.finish(10_000_000_000)
        self.assertEqual(result["status_event_count"], 400)
        self.assertAlmostEqual(result["comparison_valid_ratio"], 0.99)
        self.assertAlmostEqual(result["monitoring_available_ratio"], 0.99)

    def test_counter_delta_excludes_warmup_timeout(self):
        accumulator = StatusEventAccumulator()
        accumulator.start(1_000_000_000, 2_000_000_000, status(
            comparison_valid=True, monitoring_available=True, query_timeout_count=2))
        accumulator.record(status(
            comparison_valid=True, monitoring_available=True, query_timeout_count=2),
            1_100_000_000)
        result = accumulator.finish(2_000_000_000)
        self.assertEqual(result["query_timeout_count_delta"], 0)

    def test_invalid_window_duration_is_reported(self):
        accumulator = StatusEventAccumulator()
        accumulator.start(0, 1_000_000_000, status(
            comparison_valid=True, monitoring_available=True))
        accumulator.record(status(
            comparison_valid=False, monitoring_available=False), 100_000_000)
        accumulator.record(status(
            comparison_valid=True, monitoring_available=True), 300_000_000)
        result = accumulator.finish(1_000_000_000)
        self.assertEqual(result["comparison_valid_false_max_duration_ms"], 200.0)
        self.assertEqual(result["monitoring_available_false_max_duration_ms"], 200.0)


if __name__ == "__main__":
    unittest.main()
