import importlib.util
import hashlib
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
    def test_clean_path_list_excludes_build_and_install_trees(self) -> None:
        for name in ("build", "install"):
            self.assertNotIn(ROOT / name, runner.GENERATED_CLEAN_PATHS)
        for name in ("build-debug", "build-gprof", "install-debug", "log-debug"):
            self.assertIn(ROOT / name, runner.GENERATED_CLEAN_PATHS)

    def test_clean_preserves_incremental_build_and_install_trees(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            root = Path(temporary)
            build = root / "build"
            install = root / "install"
            stale_build = root / "build-debug"
            artifacts = root / ".artifacts"
            logs = root / "log"
            for directory in (build, install, stale_build, artifacts, logs):
                directory.mkdir()
                (directory / "sentinel").write_text("keep", encoding="utf-8")

            original_paths = runner.GENERATED_CLEAN_PATHS
            runner.GENERATED_CLEAN_PATHS = (artifacts, logs, stale_build)
            try:
                self.assertEqual(runner.clean(), 0)
            finally:
                runner.GENERATED_CLEAN_PATHS = original_paths

            self.assertTrue((build / "sentinel").is_file())
            self.assertTrue((install / "sentinel").is_file())
            self.assertFalse(stale_build.exists())
            self.assertFalse(artifacts.exists())
            self.assertFalse(logs.exists())

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

    def test_simulation_mapping_profile_preserves_collision_parameters(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            target = runner._mapping_params(
                session, ROOT / "config/runtime/mapping.yaml", simulation=True
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))[
                "navigation_runtime"
            ]["ros__parameters"]
            self.assertEqual(
                parameters["navigation"]["collision"],
                {"vehicle_radius_m": 0.32, "safety_margin_m": 0.05},
            )
            self.assertTrue(parameters["navigation"]["planner"]["allow_unknown_start"])
            self.assertFalse(parameters["navigation"]["planner"]["allow_nominal_unknown"])

            dual_target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                simulation=True,
                dual_planning=True,
            )
            dual_parameters = yaml.safe_load(dual_target.read_text(encoding="utf-8"))[
                "navigation_runtime"
            ]["ros__parameters"]
            self.assertTrue(dual_parameters["navigation"]["planner"]["allow_nominal_unknown"])
            self.assertEqual(
                dual_parameters["navigation"]["planner"]["nominal_commitment_horizon_s"],
                1.5,
            )

    def test_mission_planning_policy_is_applied_to_runtime_parameters(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            mission = ROOT / "config/runtime/missions/open.yaml"
            target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                simulation=True,
                dual_planning=True,
                mission_file=mission,
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))["navigation_runtime"]["ros__parameters"]
            self.assertEqual(parameters["navigation"]["replan_rate_hz"], 5.0)
            self.assertEqual(parameters["navigation"]["planner"]["max_velocity_mps"], 1.0)
            self.assertEqual(parameters["navigation"]["planner"]["max_acceleration_mps2"], 2.0)
            self.assertEqual(parameters["navigation"]["planner"]["max_deceleration_mps2"], 2.0)
            self.assertEqual(parameters["navigation"]["planner"]["max_jerk_mps3"], 6.0)

            external = runner._external_mode_params(
                session, ROOT / "config/runtime/external_mode.yaml", mission
            )
            external_parameters = yaml.safe_load(external.read_text(encoding="utf-8"))["px4_navigation_external_mode"]["ros__parameters"]
            tracker = external_parameters["navigation"]["velocity_tracker"]
            self.assertEqual(tracker["max_velocity_mps"], 1.0)
            self.assertEqual(tracker["max_acceleration_mps2"], 2.0)
            self.assertEqual(tracker["max_deceleration_mps2"], 2.0)

    def test_stress_profiles_have_explicit_mission_limits_and_worlds(self) -> None:
        profiles = {
            "corridor": 1.0,
            "speed": 2.0,
            "long_open": 1.5,
            "long_open_slow": 0.8,
            "long_featured": 1.5,
            "long_three_pillars": 1.4,
            "no_path": 1.0,
            "occlusion_featured": 1.0,
            "occlusion_degenerate": 1.0,
            "tunnel_irregular": 1.5,
            "tunnel_smooth": 1.5,
            "forest_clutter": 1.5,
        }
        for profile, expected_velocity in profiles.items():
            registry_entry = runner._map_registry().get(profile, {})
            world_name = {
                "speed": "open",
                "long_open_slow": "long_open",
            }.get(profile, registry_entry.get("world", profile) if isinstance(registry_entry, dict) else profile)
            world = ROOT / "src/uav_simulation/worlds" / f"{world_name}.sdf"
            mission_name = registry_entry.get("mission", profile) if isinstance(registry_entry, dict) else profile
            mission = ROOT / "config/runtime/missions" / f"{mission_name}.yaml"
            self.assertTrue(world.is_file(), profile)
            self.assertTrue(mission.is_file(), profile)
            mission_value = yaml.safe_load(mission.read_text(encoding="utf-8"))
            self.assertEqual(mission_value["mission"]["planning"]["unknown_policy"], "blocked")
            planning = runner._mission_planning(mission)
            self.assertEqual(planning["max_velocity_mps"], expected_velocity)
            self.assertGreater(planning["max_acceleration_mps2"], 0.0)

    def test_stress_profiles_have_ground_truth_collision_geometry(self) -> None:
        for profile in (
            "open", "speed", "long_open", "long_open_slow", "long_featured",
            "corridor", "pillar", "occlusion", "occlusion_featured", "occlusion_degenerate",
            "tunnel_irregular", "tunnel_smooth", "forest_clutter", "long_three_pillars", "no_path",
        ):
            obstacles = runner._collision_obstacles(profile)
            self.assertTrue(obstacles, profile)
            names = [obstacle["name"] for obstacle in obstacles]
            self.assertEqual(len(names), len(set(names)))
            for obstacle in obstacles:
                self.assertIn(obstacle["type"], {"box", "cylinder"})
                self.assertEqual(len(obstacle["center"]), 3)

    def test_map_registry_is_deterministic_and_truth_names_are_unique(self) -> None:
        registry = runner._map_registry()
        for profile in ("occlusion_featured", "occlusion_degenerate", "tunnel_irregular", "tunnel_smooth", "forest_clutter", "long_three_pillars", "no_path"):
            descriptor = registry[profile]
            self.assertIn("world", descriptor)
            self.assertIn("mission", descriptor)
            self.assertEqual(len(descriptor["collision_truth"]), len(set(descriptor["collision_truth"])))
            self.assertTrue((ROOT / "src/uav_simulation/worlds" / f"{descriptor['world']}.sdf").is_file())
            self.assertTrue((ROOT / "config/runtime/missions" / f"{descriptor['mission']}.yaml").is_file())
        self.assertEqual(registry["no_path"]["expected_outcome"], "fail_closed")

    def test_long_three_pillars_has_explicit_route_obstacle_contract(self) -> None:
        descriptor = runner._map_registry()["long_three_pillars"]
        self.assertEqual(
            descriptor["route_obstacles"],
            ["long_three_pillar_01", "long_three_pillar_02", "long_three_pillar_03"],
        )
        self.assertEqual(descriptor["route_segment_waypoints"], [0, 1])
        self.assertEqual(len(descriptor["collision_truth"]), 21)
        self.assertTrue(set(descriptor["route_obstacles"]).issubset(descriptor["collision_truth"]))

    def test_long_three_pillars_uses_bounded_receding_horizon(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                simulation=True,
                mission_file=ROOT / "config/runtime/missions/long_three_pillars.yaml",
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))["navigation_runtime"]["ros__parameters"]
            navigation = parameters["navigation"]
            mapping = parameters["mapping"]
            self.assertEqual(navigation["local_subgoal"]["max_distance_m"], 15.0)
            self.assertEqual(navigation["planning_horizon"]["minimum_distance_m"], 10.0)
            self.assertEqual(navigation["planning_horizon"]["maximum_distance_m"], 30.0)
            self.assertEqual(navigation["planning_horizon"]["preview_time_s"], 5.0)
            self.assertEqual(navigation["local_subgoal"]["switch_distance_m"], 0.8)
            self.assertEqual(navigation["collision"]["safety_margin_m"], 0.25)
            self.assertEqual(mapping["raycast"]["max_range_m"], 40.0)
            self.assertEqual(mapping["map"]["local_size_m"], [70.0, 40.0, 12.0])

    def test_canonical_scene_resolver_collapses_variants_without_new_make_profiles(self) -> None:
        self.assertEqual(
            runner._resolve_scene_profile("structured_obstacle", "positive", "nominal", None)[0],
            "occlusion_featured",
        )
        self.assertEqual(
            runner._resolve_scene_profile("structured_obstacle", "degenerate", "nominal", None)[0],
            "occlusion_degenerate",
        )
        self.assertEqual(
            runner._resolve_scene_profile("long_route", "positive", "slow", None)[0],
            "long_open_slow",
        )
        self.assertEqual(
            runner._resolve_scene_profile("planner_negative", "no_path", "nominal", None)[0],
            "no_path",
        )
        legacy, metadata = runner._resolve_scene_profile(None, "positive", "nominal", "corridor")
        self.assertEqual(legacy, "corridor")
        self.assertEqual(metadata["scene"], "legacy")

    def test_registry_seed_is_only_effective_for_stochastic_profiles(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            _, deterministic = runner._resolve_map_descriptor(session, "tunnel_smooth", 47)
            _, stochastic = runner._resolve_map_descriptor(session, "forest_clutter", 47)
            self.assertEqual(deterministic["seed"], 0)
            self.assertEqual(stochastic["seed"], 47)
            self.assertTrue((session.directory / "map_descriptor.json").is_file())
            resolved = session.directory / "resolved_forest_clutter.sdf"
            self.assertEqual(hashlib.sha256(resolved.read_bytes()).hexdigest(), stochastic["world_sha256"])

    def test_profile_variants_share_mission_limits(self) -> None:
        for left, right in (("occlusion_featured", "occlusion_degenerate"),
                            ("tunnel_irregular", "tunnel_smooth")):
            self.assertEqual(runner._map_registry()[left]["family"], runner._map_registry()[right]["family"])
            left_planning = runner._mission_planning(
                ROOT / "config/runtime/missions" / f"{runner._map_registry()[left]['mission']}.yaml")
            right_planning = runner._mission_planning(
                ROOT / "config/runtime/missions" / f"{runner._map_registry()[right]['mission']}.yaml")
            self.assertEqual(left_planning, right_planning)

    def test_no_path_mission_crosses_sealed_wall(self) -> None:
        mission = yaml.safe_load(
            (ROOT / "config/runtime/missions/no_path.yaml").read_text(encoding="utf-8")
        )["mission"]
        wall = next(
            obstacle for obstacle in runner._collision_obstacles("no_path")
            if obstacle["name"] == "sealed_wall"
        )
        self.assertGreater(float(mission["waypoints"][0]["position"][0]), wall["center"][0])
        self.assertLess(0.0, wall["center"][0])

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

    def test_external_mode_gui_launch_passes_static_mission_file(self) -> None:
        command = runner._external_mode_launch_command(
            Path("/tmp/external_mode_params.yaml"),
            Path("/tmp/mission.yaml"),
        )
        self.assertIn("use_sim_time:=true", command)
        self.assertIn("mission_file:=/tmp/mission.yaml", command)

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

    def test_simulation_profiles_separate_headless_and_interactive_manual_control(self) -> None:
        self.assertEqual(
            runner._px4_manual_control_mode(True),
            runner.PX4_HEADLESS_COM_RC_IN_MODE,
        )
        self.assertEqual(
            runner._px4_manual_control_mode(False),
            runner.PX4_INTERACTIVE_COM_RC_IN_MODE,
        )

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
        source = (ROOT / "src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn('constexpr char kLioPropagatedOdometryTopic[] = "/lio/odometry_propagated"', source)
        self.assertEqual(source.count("create_subscription<nav_msgs::msg::Odometry>"), 1)
        self.assertIn("kLioPropagatedOdometryTopic", source)
        self.assertNotIn("/sim/ground_truth/odometry", source)

    def test_external_mode_scenario_waits_for_registration_before_retrying_nav_state(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        scenario = object.__new__(module.ExternalModeScenario)
        scenario.config = {
            "activation_timeout_s": 30.0,
            "post_activation_s": 8.0,
            "disarm_timeout_s": 10.0,
            "exit_nav_state": 4,
            "trajectory_publish_period_s": 0.2,
            "command_retry_period_s": 1.0,
            "trajectory_source": "none",
        }
        scenario.wall_start = time.monotonic() - 2.0
        scenario.sim_start_ns = 1_000_000_000
        scenario.sim_now_ns = 1_000_000_000 + 5_000_000_000
        scenario.finished = False
        scenario.failure = ""
        scenario.trajectory_success_count = 1
        scenario.mode_entered = False
        scenario.external_mode_id = None
        scenario.armed_seen = False
        scenario.last_trajectory_ns = 0
        scenario.last_command_ns = {}
        scenario.VehicleCommand = type("VehicleCommand", (), {"VEHICLE_CMD_SET_NAV_STATE": 100001})
        scenario.command_pub = object()
        scenario._retry = lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("retry while external mode id is unknown"))
        scenario.finish = lambda *args, **kwargs: None

        scenario._tick()
        self.assertEqual(scenario.failure, "")

    def test_external_mode_scenario_planner_source_publishes_bounded_goal(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario_goal",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        class Message:
            def __init__(self) -> None:
                self.header = type("Header", (), {"frame_id": "", "stamp": type("Stamp", (), {})()})()
                self.mission_id = ""
                self.waypoint_index = 0
                self.request_id = 0
                self.target = type("Point", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
                self.acceptance_radius_m = 0.0

        class Publisher:
            def __init__(self) -> None:
                self.messages = []

            def publish(self, message: object) -> None:
                self.messages.append(message)

        scenario = object.__new__(module.ExternalModeScenario)
        scenario.config = {"planning_frame": "lio_odom", "goal_offset_m": [1.0, -0.5, 0.25]}
        scenario.latest_odom = {"x": 2.0, "y": 3.0, "z": 4.0}
        scenario.sim_now_ns = 2_500_000_000
        scenario.NavigationGoal = Message
        scenario.goal_pub = Publisher()
        scenario.goal_publish_count = 0
        scenario.goal_request_id = 0
        scenario.latest_goal = {}
        scenario.last_goal_ns = -10**18
        scenario._record = lambda *args, **kwargs: None
        scenario.finish = lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError("valid planner goal must not finish the scenario")
        )

        scenario._publish_planner_goal()

        self.assertEqual(scenario.goal_publish_count, 1)
        self.assertEqual(len(scenario.goal_pub.messages), 1)
        message = scenario.goal_pub.messages[0]
        self.assertEqual(message.header.frame_id, "lio_odom")
        self.assertEqual((message.target.x, message.target.y, message.target.z),
                         (3.0, 2.5, 4.25))
        self.assertEqual(message.request_id, 1)

    def test_external_mode_scenario_treats_executor_handover_as_exit(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        class DummyVehicleStatus:
            NAVIGATION_STATE_EXTERNAL1 = 23
            NAVIGATION_STATE_AUTO_LOITER = 4
            NAVIGATION_STATE_AUTO_RTL = 20
            ARMING_STATE_ARMED = 2
            ARMING_STATE_DISARMED = 1

        scenario = object.__new__(module.ExternalModeScenario)
        scenario.external_mode_id = 23
        scenario.mode_entered = True
        scenario.mode_exit_observed = False
        scenario.exit_requested = True
        scenario.previous_nav_state = 23
        scenario.events = []
        scenario.VehicleStatus = DummyVehicleStatus
        scenario.latest_status = {}
        scenario.failsafe_seen = False
        scenario.armed_seen = False
        scenario.unexpected_rtl = False
        scenario._record = lambda *args, **kwargs: None
        scenario._status(type(
            "Message",
            (),
            {
                "nav_state": 23,
                "arming_state": 2,
                "can_set_nav_states_mask": 0,
                "failsafe": False,
                "pre_flight_checks_pass": True,
                "executor_in_charge": 0,
                "timestamp": 1,
            },
        )())
        self.assertTrue(scenario.mode_exit_observed)

    def test_external_mode_scenario_records_position_control_handover_without_commands(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario_handover",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        scenario = object.__new__(module.ExternalModeScenario)
        scenario.execution = "mission"
        scenario.sim_now_ns = 12_000_000_000
        scenario.events = []
        scenario._record = lambda *args, **kwargs: None

        scenario._record_handover_request(
            "unexpected_external_mode_exit", {"nav_state": 18})
        scenario._record_handover_request("second_request")

        self.assertEqual(len(scenario.events), 2)
        self.assertEqual(scenario.events[0]["name"], "position_control_handover_requested")
        self.assertEqual(scenario.events[0]["detail"], {"nav_state": 18})

    def test_mission_arms_and_takes_off_before_external_mode_activation(self) -> None:
        source = (ROOT / "tools/runtime/external_mode_scenario.py").read_text(
            encoding="utf-8")
        mission_tick = source.index("if mission_mode:")
        arm = source.index("if not self.armed_seen:", mission_tick)
        takeoff = source.index("if not self.takeoff_requested:", arm)
        activate = source.index('if not getattr(self, "mode_active", False)', takeoff)
        self.assertLess(arm, takeoff)
        self.assertLess(takeoff, activate)
        self.assertIn("pre_activation_odometry_stable_s", source[arm:activate])

    def test_manual_takeoff_path_sends_neither_arm_nor_takeoff(self) -> None:
        source = (ROOT / "tools/runtime/external_mode_scenario.py").read_text(
            encoding="utf-8")
        mission_tick = source.index("if mission_mode:")
        arm = source.index("if not self.armed_seen:", mission_tick)
        takeoff = source.index("if not self.takeoff_requested:", arm)
        self.assertIn("if self.manual_takeoff:", source[arm:takeoff])
        self.assertIn("if not self.manual_takeoff:", source[takeoff:takeoff + 300])

    def test_runtime_never_depends_on_nonexistent_ev_aid_source_topics(self) -> None:
        for path in (
            RUNTIME / "monitor.py",
            RUNTIME / "report.py",
            ROOT / "config/runtime/common.yaml",
        ):
            source = path.read_text(encoding="utf-8")
            self.assertNotIn("estimator_aid_src_ev", source)
            self.assertNotIn("EstimatorAidSource", source)

    def test_navigation_replanning_is_correlated_to_goal_and_world_revision(self) -> None:
        source = (
            ROOT / "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("plan_skip_count_", source)
        self.assertIn("last_planned_request_id_", source)
        self.assertIn("last_planned_world_revision_", source)
        self.assertIn("world.revision() == last_planned_world_revision_", source)
        self.assertIn('keyValue("plan_skip_count"', source)

    def test_lio_diagnostics_expose_map_guard_and_propagation_latency(self) -> None:
        source = (ROOT / "src/estimation/fast_lio_ros/src/ros_output_publisher.cpp").read_text(
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

    def test_simulation_stream_discards_wall_epoch_without_regression(self) -> None:
        stats = StreamStats(
            "px4_odometry",
            "/fmu/out/vehicle_odometry",
            stale_after_s=0.1,
            timestamp_upper_bound_ns=1_000_000_000_000_000,
        )
        stats.update(1_787_022_423_986_327_000, 1_000_000_000)
        stats.update(4_000_000_000, 1_100_000_000)
        stats.update(4_016_000_000, 1_116_000_000)
        snapshot = stats.as_dict()
        self.assertEqual(snapshot["received"], 2)
        self.assertEqual(snapshot["timestamp_regression_count"], 0)
        self.assertEqual(snapshot["timestamp_epoch_discard_count"], 1)

    def test_report_ignores_monitor_discarded_samples_for_matching(self) -> None:
        def sample(stream: str, stamp_ns: int, accepted: bool = True) -> dict[str, object]:
            return {
                "kind": "sample",
                "stream": stream,
                "timestamp_ns": stamp_ns,
                "accepted_by_monitor": accepted,
                "payload": {"timestamp_sample_us": stamp_ns // 1000},
            }

        samples = [
            sample("external_odometry", 4_000_000_000),
            sample("px4_odometry", 1_787_022_423_986_327_000, False),
            sample("px4_odometry", 4_000_000_000),
        ]
        self.assertEqual(len(report._series(samples, "px4_odometry")), 1)

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

    def test_planning_execution_summary_exposes_replan_skips_and_fallbacks(self) -> None:
        snapshot = {
            "latest": {
                "planning_diagnostics": {
                    "statuses": [
                        {
                            "name": "navigation_planning/planner",
                            "values": {
                                "plan_count": "10",
                                "plan_skip_count": "4",
                                "success_count": "9",
                                "failure_count": "1",
                                "safety_fallback_count": "2",
                                "safety_route_selected_count": "1",
                                "safety_stop_selected_count": "1",
                                "nominal_plan_count": "3",
                                "nominal_selected_count": "1",
                                "dual_verification_failure_count": "0",
                                "verification_failure_count": "0",
                                "local_subgoal_selected_count": "0",
                                "local_subgoal_failure_count": "0",
                                "trajectory_revalidation_count": "6",
                                "trajectory_revalidation_failure_count": "1",
                                "trajectory_reuse_count": "5",
                                "full_replan_count": "2",
                            },
                        }
                    ]
                }
            }
        }
        self.assertEqual(
            report._planning_execution_summary(snapshot),
            {
                "plan_count": 10,
                "plan_skip_count": 4,
                "success_count": 9,
                "failure_count": 1,
                "safety_fallback_count": 2,
                "safety_route_selected_count": 1,
                "safety_stop_selected_count": 1,
                "nominal_plan_count": 3,
                "nominal_selected_count": 1,
                "dual_verification_failure_count": 0,
                "verification_failure_count": 0,
                "local_subgoal_selected_count": 0,
                "local_subgoal_failure_count": 0,
                "trajectory_revalidation_count": 6,
                "trajectory_revalidation_failure_count": 1,
                "trajectory_reuse_count": 5,
                "full_replan_count": 2,
            },
        )


if __name__ == "__main__":
    unittest.main()
