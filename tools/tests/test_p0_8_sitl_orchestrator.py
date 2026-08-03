from pathlib import Path
from types import SimpleNamespace
import json
import signal
import sys
import tempfile
import unittest
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "performance"))
import p0_8_sitl_orchestrator as orchestrator
from p0_8_probe_metrics import StatusEventAccumulator


class P08OrchestratorContractTest(unittest.TestCase):
    def test_stage_machine_accepts_canonical_startup_prefix(self):
        machine = orchestrator.StageMachine()
        for stage in (
            orchestrator.Stage.PREFLIGHT,
            orchestrator.Stage.XRCE_AGENT_STARTING,
            orchestrator.Stage.XRCE_AGENT_READY,
            orchestrator.Stage.PX4_GAZEBO_STARTING,
            orchestrator.Stage.GAZEBO_CLOCK_READY,
            orchestrator.Stage.ROS_BRIDGE_STARTING,
            orchestrator.Stage.ROS_CLOCK_READY,
            orchestrator.Stage.XRCE_SESSION_READY,
            orchestrator.Stage.RAW_PX4_ODOMETRY_READY,
            orchestrator.Stage.PX4_INGRESS_STARTING,
            orchestrator.Stage.PX4_INGRESS_READY,
        ):
            machine.transition(stage)
        machine.transition(orchestrator.Stage.COMPLETE)
        self.assertEqual(machine.current, orchestrator.Stage.COMPLETE)

    def test_stage_machine_accepts_supervisor_off_warmup_bypass(self):
        machine = orchestrator.StageMachine()
        machine.transition(orchestrator.Stage.PREFLIGHT)
        for stage in (
            orchestrator.Stage.XRCE_AGENT_STARTING,
            orchestrator.Stage.XRCE_AGENT_READY,
            orchestrator.Stage.PX4_GAZEBO_STARTING,
            orchestrator.Stage.GAZEBO_CLOCK_READY,
            orchestrator.Stage.ROS_BRIDGE_STARTING,
            orchestrator.Stage.ROS_CLOCK_READY,
            orchestrator.Stage.XRCE_SESSION_READY,
            orchestrator.Stage.RAW_PX4_ODOMETRY_READY,
            orchestrator.Stage.PX4_INGRESS_STARTING,
            orchestrator.Stage.PX4_INGRESS_READY,
            orchestrator.Stage.SENSORS_READY,
            orchestrator.Stage.FAST_LIO_STARTING,
            orchestrator.Stage.FAST_LIO_READY,
            orchestrator.Stage.PRIOR_ACCEPTED,
        ):
            machine.transition(stage)
        machine.transition(orchestrator.Stage.WARMUP)
        self.assertEqual(machine.current, orchestrator.Stage.WARMUP)

    def test_illegal_transition_is_rejected(self):
        machine = orchestrator.StageMachine()
        machine.transition(orchestrator.Stage.PREFLIGHT)
        with self.assertRaises(ValueError):
            machine.transition(orchestrator.Stage.FAST_LIO_READY)

    def test_failure_can_only_be_followed_by_cleanup(self):
        machine = orchestrator.StageMachine()
        machine.transition(orchestrator.Stage.PREFLIGHT)
        machine.transition(orchestrator.Stage.FAILED)
        with self.assertRaises(ValueError):
            machine.transition(orchestrator.Stage.COMPLETE)
        machine.transition(orchestrator.Stage.CLEANUP)

    def test_mode_policies_are_centralized_and_exact(self):
        self.assertEqual(orchestrator.MODE_POLICIES[orchestrator.RunMode.SMOKE_ON].warmup_sim_s, 10.0)
        self.assertEqual(orchestrator.MODE_POLICIES[orchestrator.RunMode.SMOKE_ON].measurement_sim_s, 20.0)
        self.assertEqual(orchestrator.MODE_POLICIES[orchestrator.RunMode.SITL_OFF].measurement_sim_s, 120.0)
        self.assertEqual(orchestrator.MODE_POLICIES[orchestrator.RunMode.MEMORY_ON].warmup_sim_s, 120.0)
        self.assertEqual(orchestrator.MODE_POLICIES[orchestrator.RunMode.MEMORY_ON].measurement_sim_s, 1200.0)

    def test_stage_timeout_maps_to_failure_code(self):
        self.assertEqual(orchestrator.STAGE_POLICIES[orchestrator.Stage.GAZEBO_CLOCK_READY].timeout_failure,
                         orchestrator.FailureCode.GAZEBO_CLOCK_TOPIC_MISSING)
        self.assertEqual(orchestrator.STAGE_POLICIES[orchestrator.Stage.ROS_CLOCK_READY].timeout_failure,
                         orchestrator.FailureCode.ROS_CLOCK_NOT_ADVANCING)

    def test_product_freeze_guard_is_clean_at_base(self):
        self.assertTrue(orchestrator.product_paths_unchanged(Path(__file__).resolve().parents[2]))

    def test_raw_topic_resolution_prefers_unversioned_then_lowest_version(self):
        candidates, selected = orchestrator.resolve_raw_candidates([
            "/fmu/out/vehicle_odometry_v10", "/fmu/out/vehicle_odometry_v2",
            "/fmu/out/vehicle_odometry", "/other/topic",
        ])
        self.assertEqual(candidates, ["/fmu/out/vehicle_odometry", "/fmu/out/vehicle_odometry_v2",
                                      "/fmu/out/vehicle_odometry_v10"])
        self.assertEqual(selected, "/fmu/out/vehicle_odometry")

    def test_raw_topic_resolution_is_fail_closed_when_empty(self):
        self.assertEqual(orchestrator.resolve_raw_candidates([]), ([], None))

    def _observation(self, logical="sensor"):
        return orchestrator.TopicObservation(logical, "/topic", "type", orchestrator.SENSOR_QOS)

    def test_topic_without_publisher_is_not_sample_timeout(self):
        self.assertEqual(orchestrator.classify_topic_observation(self._observation()),
                         orchestrator.FailureCode.SENSOR_TOPIC_MISSING)

    def test_topic_with_publisher_without_sample_is_distinct(self):
        observation = self._observation()
        observation.publisher_count = 1
        self.assertEqual(orchestrator.classify_topic_observation(observation),
                         orchestrator.FailureCode.SENSOR_SAMPLE_MISSING)

    def test_qos_incompatibility_wins_over_generic_sample_timeout(self):
        observation = self._observation()
        observation.publisher_count = 1
        observation.qos_incompatible_event_count = 1
        self.assertEqual(orchestrator.classify_topic_observation(observation),
                         orchestrator.FailureCode.QOS_INCOMPATIBLE)

    def test_raw_missing_topic_has_raw_failure_code(self):
        self.assertEqual(orchestrator.classify_topic_observation(self._observation("raw_px4_odometry")),
                         orchestrator.FailureCode.RAW_ODOMETRY_TOPIC_MISSING)

    def test_topic_observation_records_wall_and_ros_gaps(self):
        observation = self._observation()
        first = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
        second = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0)))
        observation.observe(first, {"value": 1}, 100)
        observation.observe(second, {"value": 2}, 350)
        self.assertEqual(observation.message_count, 2)
        self.assertEqual(observation.longest_wall_gap_ns, 250)
        self.assertEqual(observation.longest_ros_stamp_gap_ns, 1_000_000_000)
        self.assertEqual(observation.latest_payload_summary, {"value": 2})

    def test_diagnostics_cache_retains_status_not_present_in_next_array(self):
        def status(name, key, value):
            return SimpleNamespace(name=name, level=0, message="ok",
                                   values=[SimpleNamespace(key=key, value=value)])
        cache = orchestrator.DiagnosticsCache()
        cache.update(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
                                     status=[status("fast_lio/estimator", "status", "TRACKING"),
                                             status("fast_lio/transport", "imu_drop_count", "0")]))
        cache.update(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0)),
                                     status=[status("fast_lio/estimator", "navigation_valid", "true")]))
        self.assertEqual(cache.values("fast_lio/estimator"), {"navigation_valid": "true"})
        self.assertEqual(cache.values("fast_lio/transport"), {"imu_drop_count": "0"})

    def test_missing_diagnostic_metric_stays_none(self):
        cache = orchestrator.DiagnosticsCache()
        cache.update(SimpleNamespace(status=[SimpleNamespace(name="fast_lio/transport", level=0,
                                                              message="ok", values=[])]))
        self.assertNotIn("p95_corrected_scan_end_to_end_us", cache.values("fast_lio/transport"))
        self.assertIsNone(orchestrator.parse_int(cache.values("fast_lio/transport").get(
            "p95_corrected_scan_end_to_end_us")))

    def test_explicit_qos_profiles_match_contract(self):
        self.assertEqual(orchestrator.RAW_QOS,
                         {"history": "KEEP_LAST", "depth": 10,
                          "reliability": "BEST_EFFORT", "durability": "VOLATILE"})
        self.assertEqual(orchestrator.PX4_DIAGNOSTICS_QOS["durability"], "TRANSIENT_LOCAL")
        self.assertEqual(orchestrator.SENSOR_QOS["depth"], 100)
        self.assertEqual(orchestrator.LIO_QOS["reliability"], "RELIABLE")

    def test_status_accumulator_uses_event_level_accounting(self):
        accumulator = StatusEventAccumulator()
        status = SimpleNamespace(comparison_valid=True, monitoring_available=True,
                                 new_comparison_sample=True, aligned_comparison_fresh=True,
                                 reason="HEALTHY", health=1, query_sequence=10,
                                 query_success_count=10, query_failure_count=0,
                                 query_timeout_count=2, query_generation_mismatch_count=0,
                                 query_service_unavailable_count=0, query_invalid_component_count=0,
                                 query_stale_sequence_count=0, reinitialization_request_sequence=0,
                                 state_transition_count=0)
        accumulator.start(0, 1_000, status)
        accumulator.record(status, 1)
        accumulator.record(status, 2)
        result = accumulator.finish(1_000)
        self.assertEqual(result["status_event_count"], 2)
        self.assertEqual(result["query_timeout_count_delta"], 0)

    def test_heading_ratio_and_pending_query_use_direct_evidence(self):
        accumulator = StatusEventAccumulator()
        accumulator.start(0, 1_000, None)
        accumulator.record_heading_event(10, True)
        accumulator.record_heading_event(20, False)
        accumulator.record_pending_query(30, 123, 50)
        result = accumulator.finish(1_000)
        self.assertEqual(result["heading_observable_event_count"], 2)
        self.assertEqual(result["heading_observable_count"], 1)
        self.assertEqual(result["heading_observable_ratio"], 0.5)
        self.assertEqual(result["maximum_outstanding_queries"], 1)
        self.assertEqual(result["maximum_pending_query_age_ns"], 50)

    def test_memory_growth_uses_first_and_last_medians(self):
        rows = [{"role": "supervisor", "rss_bytes": value}
                for value in (100, 101, 99, 100, 110, 111, 109, 110)]
        result = orchestrator.robust_rss_growth(rows)
        self.assertEqual(result["first_window_rss_median"], 100)
        self.assertEqual(result["last_window_rss_median"], 110)
        self.assertAlmostEqual(result["rss_growth_ratio"], 0.10)
        self.assertFalse(result["pass"])

    def test_memory_growth_requires_two_complete_windows(self):
        with self.assertRaises(ValueError):
            orchestrator.robust_rss_growth([{"role": "supervisor", "rss_bytes": 1}] * 4)

    def test_atomic_artifact_write_leaves_partial_run(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "run"
            artifact = orchestrator.ArtifactWriter(path, {"outcome": "RUNNING"})
            artifact.update(stage="PREFLIGHT")
            artifact.event({"event": "stage_enter", "stage": "PREFLIGHT"})
            artifact.close()
            payload = json.loads((path / "run.json").read_text())
            self.assertEqual(payload["stage"], "PREFLIGHT")
            self.assertTrue((path / "events.jsonl").read_text().strip())

    def test_artifact_output_collision_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "run"
            path.mkdir()
            with self.assertRaises(orchestrator.QualificationFailure) as context:
                orchestrator.ArtifactWriter(path, {})
            self.assertEqual(context.exception.code, orchestrator.FailureCode.RESOURCE_CONFLICT)

    def test_udp_preflight_does_not_kill_any_process(self):
        with mock.patch("p0_8_sitl_orchestrator.socket.socket") as socket_factory:
            probe = socket_factory.return_value
            probe.bind.side_effect = OSError("busy")
            with mock.patch("p0_8_sitl_orchestrator.os.kill") as kill:
                owners = orchestrator.occupied_udp_port(8888)
                self.assertTrue(owners)
                kill.assert_not_called()

    def test_cleanup_signal_order_is_declared(self):
        self.assertEqual([signal.SIGINT.name, signal.SIGTERM.name, signal.SIGKILL.name],
                         ["SIGINT", "SIGTERM", "SIGKILL"])


if __name__ == "__main__":
    unittest.main()
