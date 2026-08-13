from pathlib import Path
import sys
import tempfile
import time
import unittest

import yaml

RUNTIME = Path(__file__).resolve().parents[1]
ROOT = RUNTIME.parents[1]
sys.path.insert(0, str(RUNTIME))

from monitor import StreamStats
import report
import runner


class RuntimeContractTest(unittest.TestCase):
    def test_mapping_config_uses_canonical_product_contract(self) -> None:
        mapping = runner.load_config("mapping.yaml")["navigation_runtime"]["ros__parameters"]["mapping"]
        self.assertEqual(mapping["input"]["min_range_m"], 0.5)
        self.assertEqual(mapping["input"]["max_range_m"], 0.0)
        self.assertEqual(mapping["map"]["local_size_m"], [30.0, 30.0, 12.0])
        self.assertEqual(mapping["raycast"]["min_range_m"], 0.3)
        self.assertEqual(mapping["input_qos"]["reliability"], "best_effort")
        self.assertNotIn("rog", mapping)
        self.assertNotIn("qos", mapping)
        rviz = runner.RVIZ_CONFIG.read_text(encoding="utf-8")
        for topic in (
            "/navigation_mapping/visualization/occupied",
            "/navigation_mapping/visualization/inflated_occupied",
            "/navigation_mapping/visualization/unknown",
            "/navigation_mapping/visualization/frontier",
            "/navigation/visualization/planned_path",
        ):
            self.assertIn(topic, rviz)
        for obsolete in ("/rog_map/occ", "/rog_map/inf_occ", "/rog_map/unk", "/rog_map/frontier"):
            self.assertNotIn(obsolete, rviz)

    def test_mapping_profile_keeps_frontier_off_when_rviz_is_interactive(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            target = runner._mapping_params(
                session, ROOT / "config/runtime/mapping.yaml", interactive=True
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))[
                "navigation_runtime"
            ]["ros__parameters"]["mapping"]
            self.assertTrue(parameters["visualization"]["enabled"])
            self.assertTrue(parameters["visualization"]["publish_unknown"])
            self.assertFalse(parameters["visualization"]["publish_frontier"])
            self.assertFalse(parameters["visualization"]["publish_frontier"])

            debug_target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                interactive=True,
                frontier_debug=True,
            )
            debug_parameters = yaml.safe_load(debug_target.read_text(encoding="utf-8"))[
                "navigation_runtime"
            ]["ros__parameters"]["mapping"]
            self.assertTrue(debug_parameters["visualization"]["publish_frontier"])
            self.assertTrue(debug_parameters["visualization"]["publish_frontier"])

    def test_runtime_forces_legacy_rviz_environment_off(self) -> None:
        self.assertEqual(
            runner.NO_RVIZ_ENV,
            {
                "ENABLE_RVIZ": "0",
                "RVIZ_ENABLE": "0",
                "DISABLE_RVIZ": "1",
                "NAVIGATION_NO_RVIZ": "1",
            },
        )
        self.assertEqual(
            runner.RVIZ_ENV,
            {
                "ENABLE_RVIZ": "1",
                "RVIZ_ENABLE": "1",
                "DISABLE_RVIZ": "0",
                "NAVIGATION_NO_RVIZ": "0",
            },
        )

    def test_rviz_shell_explicitly_enables_visualizer_environment(self) -> None:
        command = runner._ros_shell(["rviz2"], enable_rviz=True)[-1]
        self.assertIn("export ENABLE_RVIZ=1 RVIZ_ENABLE=1 DISABLE_RVIZ=0 NAVIGATION_NO_RVIZ=0", command)

    def test_rviz_command_uses_sim_clock_without_legacy_topic_remap(self) -> None:
        command = runner._rviz_command(use_sim_time=True)
        self.assertIn("--ros-args", command)
        self.assertIn("-p", command)
        self.assertIn("use_sim_time:=true", command)
        self.assertNotIn("/livox/lidar:=/lidar/points", command)

    def test_static_sensor_tf_is_derived_from_each_estimator_extrinsic(self) -> None:
        expected = {
            "dataset.yaml": (0.019391, 0.000278, -0.080926),
            "sim.yaml": (0.011, 0.02329, -0.04412),
        }
        for config_name, expected_xyz in expected.items():
            xyz, rpy = runner._lidar_to_imu_launch_arguments(
                runner.load_config(config_name)
            )
            for actual, reference in zip(map(float, xyz.split()), expected_xyz):
                self.assertAlmostEqual(actual, reference, places=12)
            for value in map(float, rpy.split()):
                self.assertAlmostEqual(value, 0.0, places=12)

    def test_product_rviz_config_shows_only_published_odometry(self) -> None:
        config = runner.RVIZ_CONFIG.read_text(encoding="utf-8")
        self.assertIn("Fixed Frame: lio_odom", config)
        self.assertIn("Value: /lio/odometry_corrected", config)
        self.assertNotIn("/lio/registered_points", config)
        self.assertNotIn("/lio/local_map", config)
        self.assertIn("Class: rviz_default_plugins/Odometry", config)

    def test_stop_discovers_all_owned_runtime_sessions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in ("sim-old", "sim-new"):
                session = root / name
                session.mkdir()
                (session / "processes.json").write_text("{}", encoding="utf-8")
            (root / "latest").symlink_to("sim-new")
            self.assertEqual(
                [path.name for path in runner._runtime_session_paths(root)],
                ["sim-old", "sim-new"],
            )

    def test_simulation_config_is_lio_only_at_startup(self) -> None:
        config = runner.load_config("sim.yaml")["fast_lio"]["ros__parameters"]
        prior = config["initial_prior"]
        self.assertEqual(prior["source"], "zero")
        self.assertEqual(prior["source_frame"], "lio_odom")
        self.assertEqual(prior["source_frame_transform"], "same_frame")
        self.assertFalse(config["output"]["publish_registered_points"])
        local_map = config["mapping"]["local_map"]
        self.assertGreater(local_map["absolute_map_point_guard"], 0)
        propagated = config["propagated_odometry"]
        self.assertEqual(propagated["imu_history_duration_ns"], 1_000_000_000)
        self.assertEqual(propagated["maximum_correction_age_ns"], 250_000_000)

    def test_simulation_bridge_gates_are_explicit_profile_parameters(self) -> None:
        config = runner.load_config("sim.yaml")
        external = config["px4_external_odometry_bridge"]["ros__parameters"]
        self.assertEqual(external["external_odometry"]["maximum_age_ns"], 500_000_000)
        ingress = config["px4_odometry_bridge"]["ros__parameters"]
        self.assertTrue(ingress["simulation_clock"])
        self.assertEqual(ingress["reset"]["stable_samples_after_reset"], 3)
        launcher = (ROOT / "tools/simulation/run_px4_mid360.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("export PX4_PARAM_SIM_GZ_EN_BARO=1", launcher)

    def test_replay_and_simulation_preserve_propagation_recovery_headroom(self) -> None:
        for config_name in ("sim.yaml", "dataset.yaml"):
            propagated = runner.load_config(config_name)["fast_lio"]["ros__parameters"]["propagated_odometry"]
            self.assertEqual(propagated["maximum_correction_age_ns"], 250_000_000)
            self.assertGreater(
                propagated["imu_history_duration_ns"] - propagated["maximum_correction_age_ns"],
                0,
            )

    def test_estimator_status_flags_rate_matches_px4_diagnostic_publication(self) -> None:
        runtime = runner.load_config("common.yaml")["runtime"]
        status_flags = runtime["streams"]["estimator_status_flags"]
        self.assertEqual(status_flags["expected_hz"], 1.0)
        self.assertEqual(status_flags["stale_after_s"], 2.5)

    def test_external_odometry_bridge_accepts_only_lio_not_simulator_truth(self) -> None:
        source = (ROOT / "src/px4_interface/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn('constexpr char kLioPropagatedOdometryTopic[] = "/lio/odometry_propagated"', source)
        self.assertEqual(source.count("create_subscription<nav_msgs::msg::Odometry>"), 1)
        self.assertIn("kLioPropagatedOdometryTopic", source)
        self.assertNotIn("/sim/ground_truth/odometry", source)

    def test_runtime_never_depends_on_nonexistent_ev_aid_source_topics(self) -> None:
        for path in (
            RUNTIME / "monitor.py",
            RUNTIME / "report.py",
            ROOT / "config/runtime/common.yaml",
        ):
            source = path.read_text(encoding="utf-8")
            self.assertNotIn("estimator_aid_src_ev", source)
            self.assertNotIn("EstimatorAidSource", source)

    def test_lio_diagnostics_expose_map_guard_and_propagation_latency(self) -> None:
        source = (ROOT / "src/navigation_estimator/fast_lio_ros/src/ros_output_publisher.cpp").read_text(
            encoding="utf-8"
        )
        for key in (
            'keyValue("absolute_guard_triggered"',
            'keyValue("absolute_guard_recovery_failed"',
            'keyValue("map_maintenance_us"',
            'keyValue("measurement_model_us"',
            'keyValue("maximum_replay_runtime_us"',
            'keyValue("maximum_imu_batch_size"',
        ):
            self.assertIn(key, source)

    def test_first_sample_does_not_create_stale_event(self) -> None:
        stats = StreamStats("external_odometry", "/fmu/in/vehicle_visual_odometry", stale_after_s=0.1)
        start = 1_000_000_000
        stats.update(start, start)
        stats.check_stale(start + 200_000_000)
        self.assertEqual(stats.as_dict()["stale_event_count"], 0)

    def test_stream_stats_records_rates_and_regressions(self) -> None:
        stats = StreamStats("imu", "/lidar/imu", expected_hz=200.0, stale_after_s=0.1)
        start = 1_000_000_000
        stats.update(start, start, frame_id="livox_imu_frame")
        stats.update(start + 5_000_000, start + 5_000_000, frame_id="livox_imu_frame")
        stats.update(start + 4_000_000, start + 6_000_000, frame_id="livox_imu_frame")
        stats.check_stale(start + 200_000_000)
        snapshot = stats.as_dict()
        self.assertEqual(snapshot["received"], 3)
        self.assertEqual(snapshot["timestamp_regression_count"], 1)
        self.assertEqual(snapshot["stale_event_count"], 1)
        self.assertEqual(snapshot["frame_ids"], ["livox_imu_frame"])

    def test_residual_report_declares_missing_pre_fusion_data(self) -> None:
        result = report._residuals([], 20.0)
        self.assertEqual(result["pre_fusion"], "NOT_AVAILABLE")
        self.assertEqual(result["fusion_enabled"], "OBSERVED_ONLY")
        self.assertTrue(result["circular_comparison"])

    def test_residual_report_aligns_lio_frd_to_px4_world(self) -> None:
        yaw_quarter_turn = [0.7071067811865476, 0.0, 0.0, 0.7071067811865476]

        def sample(stream: str, stamp_us: int, payload: dict[str, object]) -> dict[str, object]:
            return {"kind": "sample", "stream": stream, "payload": payload, "timestamp_ns": stamp_us * 1000}

        samples = [
            sample(
                "external_odometry",
                1_000,
                {"timestamp_sample_us": 1_000, "position": [1.0, 2.0, 3.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0], "velocity": [1.0, 0.0, 0.0]},
            ),
            sample(
                "external_odometry",
                1_020,
                {"timestamp_sample_us": 1_020, "position": [2.0, 2.0, 3.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0], "velocity": [1.0, 0.0, 0.0]},
            ),
            sample(
                "px4_odometry",
                1_000,
                {"timestamp_sample_us": 1_000, "position": [10.0, 20.0, 30.0], "q_wxyz": yaw_quarter_turn, "velocity": [0.0, 1.0, 0.0], "velocity_frame": 1},
            ),
            sample(
                "px4_odometry",
                1_020,
                {"timestamp_sample_us": 1_020, "position": [10.0, 21.0, 30.0], "q_wxyz": yaw_quarter_turn, "velocity": [0.0, 1.0, 0.0], "velocity_frame": 1},
            ),
        ]
        result = report._residuals(samples, 20.0)
        self.assertEqual(result["source"], "lio/external_odometry_input vs px4/estimator_odometry")
        self.assertEqual(result["matched_sample_count"], 2)
        self.assertAlmostEqual(result["position"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["velocity"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["attitude"]["maximum"], 0.0, places=9)

    def test_residual_report_keeps_independent_stream_epochs_distinct(self) -> None:
        def sample(stream: str, stamp_us: int) -> dict[str, object]:
            return {
                "kind": "sample",
                "stream": stream,
                "payload": {
                    "timestamp_sample_us": stamp_us,
                    "position": [0.0, 0.0, 0.0],
                    "q_wxyz": [1.0, 0.0, 0.0, 0.0],
                },
                "timestamp_ns": stamp_us * 1000,
            }

        result = report._residuals([sample("external_odometry", 1_000), sample("px4_odometry", 2_000)], 0.5)
        self.assertEqual(result["matched_sample_count"], 0)
        self.assertEqual(result["initial_stream_epoch_offset_ms"], -1.0)

    def test_frame_contract_reports_ned_world_mapping_and_attitude(self) -> None:
        samples = [
            {
                "kind": "sample",
                "stream": "propagated_odometry",
                "payload": {
                    "stamp_ns": 1_000_000,
                    "position": [2.0, 3.0, 4.0],
                    "q_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "linear_velocity": [1.0, 2.0, 3.0],
                    "angular_velocity": [4.0, 5.0, 6.0],
                },
                "timestamp_ns": 1_000_000,
            },
            {
                "kind": "sample",
                "stream": "external_odometry",
                "payload": {
                    "timestamp_sample_us": 1_000,
                    "pose_frame": 1,
                    "velocity_frame": 1,
                    "position": [3.0, 2.0, -4.0],
                    # C_NED_FROM_ENU * C_FRD_FROM_FLU at identity attitude is
                    # a +90 deg yaw in the NED basis.
                    "q_wxyz": [0.7071067811865476, 0.0, 0.0, 0.7071067811865475],
                    "velocity": [2.0, 1.0, -3.0],
                    "angular_velocity": [4.0, -5.0, -6.0],
                },
                "timestamp_ns": 1_000_000,
            },
        ]
        result = report._frame_contract_residuals(samples, 1.0)
        self.assertEqual(result["world_transform"], "x_px4_ned=y_lio_enu; y_px4_ned=x_lio_enu; z_px4_ned=-z_lio_enu")
        self.assertEqual(result["matched_sample_count"], 1)
        self.assertAlmostEqual(result["position"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["velocity"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["angular_velocity"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["attitude"]["maximum"], 0.0, places=9)
        self.assertEqual(result["frame_contract_violation_count"], 0)

    def test_report_verdict_set_is_closed(self) -> None:
        self.assertEqual(report.VERDICTS, {"PASS", "FAIL", "BLOCKED", "NOT_RUN", "OBSERVATION_COMPLETE"})

    def test_replay_tail_stale_events_are_ignored(self) -> None:
        row = {"stale_event_count": 3, "stale_event_times_ns": [700_000_000, 950_000_000, 1_100_000_000]}
        runtime = {"replay_finished_wall_ns": 1_000_000_000, "replay_tail_grace_s": 0.1}
        self.assertEqual(report._active_stale_count(row, runtime), 1)

    def test_runtime_stale_accounting_ends_at_observation_boundary(self) -> None:
        row = {
            "stale_event_count": 3,
            "stale_event_times_ns": [700_000_000, 950_000_000, 1_010_000_000],
        }
        runtime = {
            "observation_finished_wall_ns": 1_000_000_000,
            "observation_tail_grace_s": 0.1,
        }
        self.assertEqual(report._active_stale_count(row, runtime), 2)

    def test_startup_stale_event_before_tracking_is_not_runtime_violation(self) -> None:
        row = {"stale_event_count": 2, "stale_event_times_ns": [100, 300]}
        samples = [
            {
                "kind": "sample",
                "stream": "diagnostics",
                "arrival_wall_ns": 200,
                "payload": {"values": {"state": "TRACKING"}},
            }
        ]
        self.assertEqual(report._active_stale_count(row, {}, samples), 1)

    def test_callback_stall_with_continuous_source_timestamps_is_not_source_stale(self) -> None:
        row = {"stale_event_count": 1, "stale_event_times_ns": [250_000_000]}
        samples = [
            {"stream": "imu", "arrival_wall_ns": 100_000_000, "timestamp_ns": 1_000_000_000},
            {"stream": "imu", "arrival_wall_ns": 300_000_000, "timestamp_ns": 1_008_000_000},
        ]
        config = {"runtime": {"streams": {"imu": {"stale_after_s": 0.1}}}}
        result = report._stale_classification("imu", row, config, {}, samples)
        self.assertEqual(result["active_callback_stall_count"], 1)
        self.assertEqual(result["observer_dispatch_stall_count"], 1)
        self.assertEqual(result["source_stale_event_count"], 0)
        self.assertEqual(result["maximum_observer_dispatch_source_gap_ms"], 8.0)

    def test_callback_stall_with_a_source_timestamp_gap_fails_closed(self) -> None:
        row = {"stale_event_count": 1, "stale_event_times_ns": [250_000_000]}
        samples = [
            {"stream": "external_odometry", "arrival_wall_ns": 100_000_000, "timestamp_ns": 1_000_000_000},
            {"stream": "external_odometry", "arrival_wall_ns": 300_000_000, "timestamp_ns": 1_300_000_000},
        ]
        config = {"runtime": {"streams": {"external_odometry": {"stale_after_s": 0.2}}}}
        result = report._stale_classification("external_odometry", row, config, {}, samples)
        self.assertEqual(result["active_callback_stall_count"], 1)
        self.assertEqual(result["observer_dispatch_stall_count"], 0)
        self.assertEqual(result["source_stale_event_count"], 1)

    def test_map_maintenance_summary_keeps_guard_evidence(self) -> None:
        samples = [
            {
                "stream": "diagnostics",
                "payload": {
                    "statuses": [
                        {
                            "name": "fast_lio/estimator",
                            "values": {
                                "absolute_guard_triggered": True,
                                "absolute_guard_recovery_failed": False,
                                "map_maintenance_us": 21606,
                            },
                        }
                    ]
                },
            }
        ]
        result = report._map_maintenance_summary(samples)
        self.assertEqual(result["absolute_guard_trigger_count"], 1)
        self.assertEqual(result["absolute_guard_recovery_failure_count"], 0)
        self.assertEqual(result["maximum_maintenance_us"], 21606.0)

    def test_mapping_and_planning_timing_reports_have_required_percentiles(self) -> None:
        samples = [
            {
                "stream": "diagnostics",
                "payload": {
                    "statuses": [
                        {
                            "name": "navigation_mapping/world_model",
                            "values": {
                                "ros_pointcloud_decode_us": value,
                                "mapping_filter_us": value + 1,
                                "planning_total_us": value + 2,
                            },
                        }
                    ]
                },
            }
            for value in (10, 20, 30)
        ]
        mapping = report._diagnostic_timing_summary(
            samples,
            "navigation_mapping/world_model",
            ("ros_pointcloud_decode_us", "mapping_filter_us"),
        )
        self.assertEqual(mapping["ros_pointcloud_decode_us"]["sample_count"], 3)
        self.assertEqual(mapping["ros_pointcloud_decode_us"]["p50"], 20.0)
        for key in ("mean", "p50", "p95", "p99", "max"):
            self.assertIn(key, mapping["mapping_filter_us"])
        planning = report._planning_timing_summary(samples)
        self.assertEqual(planning["planning_total_us"]["sample_count"], 0)


if __name__ == "__main__":
    unittest.main()
