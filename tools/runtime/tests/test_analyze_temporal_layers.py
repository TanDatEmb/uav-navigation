"""Causal attribution needs independent witnesses, not downstream gaps alone."""

from __future__ import annotations

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_temporal_layers import classify  # noqa: E402


def gap(ms: float, progress_ms: float = 4.0) -> dict[str, float]:
    return {"gap_ms": ms, "source_progress_ms": progress_ms}


class TemporalLayerClassificationTest(unittest.TestCase):
    def test_native_simulation_stall(self) -> None:
        result = classify({"gazebo_stats": gap(481), "gazebo_clock": gap(481)}, True)
        self.assertEqual(result[0], "GAZEBO_SIMULATION_STALL")

    def test_native_simulation_slowdown_with_measurable_source_progress(self) -> None:
        # Exact controlled-pause shape: stats progressed 44 ms across a
        # 404 ms receive interval.  The 360 ms deficit is the causal witness;
        # requiring near-zero progress would leave the real pause unresolved.
        result = classify({"gazebo_stats": gap(404, 44),
                           "gazebo_clock": gap(354, 4)}, True)
        self.assertEqual(result[0], "GAZEBO_SIMULATION_STALL")

    def test_short_source_deficit_is_not_called_simulation_stall(self) -> None:
        result = classify({"gazebo_stats": gap(210, 150),
                           "gazebo_clock": gap(210, 150)}, True)
        self.assertEqual(result[0], "UNRESOLVED")

    def test_observer_sched_stall_prevents_gazebo_attribution(self) -> None:
        result = classify({"gazebo_stats": gap(481), "gazebo_clock": gap(481),
                           "observer_loop": gap(481)}, True)
        self.assertEqual(result[0], "UNRESOLVED")

    def test_native_clock_progress_excludes_simulation_stall(self) -> None:
        result = classify({"gazebo_stats": gap(20), "gazebo_clock": gap(20),
                           "ros_clock": gap(481)}, True)
        self.assertEqual(result[0], "UNRESOLVED")

    def test_worker_stall_with_live_inputs(self) -> None:
        result = classify({"ros_clock": gap(4), "imu": gap(5),
                           "worker": gap(220), "publisher": gap(220)}, True)
        self.assertEqual(result[0], "FAST_LIO_WORKER_STALL")

    def test_native_imu_gap_brackets_but_does_not_invent_first_component(self) -> None:
        result = classify({"gazebo_clock": gap(4), "native_imu": gap(220),
                           "imu": gap(220)}, True, True)
        self.assertEqual(result[0], "UNRESOLVED")
        result = classify({"gazebo_clock": gap(4), "native_imu": gap(5),
                           "imu": gap(220)}, True, True)
        self.assertEqual(result[0], "UNRESOLVED")

    def test_downstream_gap_without_native_witness_is_unresolved(self) -> None:
        result = classify({"ros_clock": gap(481), "imu": gap(481),
                           "worker": gap(481), "publisher": gap(481),
                           "adapter_callback": gap(481),
                           "accepted_state": gap(481)}, False)
        self.assertEqual(result[0], "UNRESOLVED")

    def test_adapter_starvation_with_live_callbacks(self) -> None:
        result = classify({"adapter_callback": gap(20),
                           "accepted_state": gap(220)}, True)
        self.assertEqual(result[0], "ADAPTER_SEMANTIC_REJECTION_STARVATION")


if __name__ == "__main__":
    unittest.main()
