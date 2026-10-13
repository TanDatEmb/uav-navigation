"""The S1 estimation launch file (S1b Task 11; D22, D33/O19): four required arguments without defaults, the four
processes, use_sim_time everywhere, and every process ends the launch when it exits."""
import importlib.util
import pathlib

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

LAUNCH_FILE = pathlib.Path(__file__).resolve().parents[1] / "launch" / "s1_estimation.launch.py"
REQUIRED = {"lio_config", "lio_node_config", "bridge_config", "events_dir"}
PROCESSES = {("ros_gz_bridge", "parameter_bridge"), ("uav_simulation", "gz_lidar_bridge"),
             ("uavnav_lio", "lio_node"), ("uavnav_px4_bridge", "px4_bridge_node")}


def description():
    spec = importlib.util.spec_from_file_location("s1_estimation_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_required_arguments_have_no_default():
    arguments = [e for e in description().entities if isinstance(e, DeclareLaunchArgument)]
    assert {a.name for a in arguments} == REQUIRED
    assert all(a.default_value is None for a in arguments)


def test_starts_the_four_processes_with_sim_time_and_shutdown_on_exit():
    context = LaunchContext()
    context.launch_configurations.update({"lio_config": "/c/lio.yaml", "lio_node_config": "/c/node.yaml",
                                          "bridge_config": "/c/bridge.yaml", "events_dir": "/run"})
    nodes = [e for e in description().entities if isinstance(e, Node)]
    assert {(n.node_package, n.node_executable) for n in nodes} == PROCESSES
    parameters = {}
    for node in nodes:
        evaluated = {}
        for p in evaluate_parameters(context, node._Node__parameters):  # launch_ros keeps them unevaluated here
            evaluated.update(p)
        parameters[node.node_executable] = evaluated
        assert evaluated.get("use_sim_time") is True, node.node_executable
        exits = node._ExecuteLocal__on_exit  # launch: the on_exit actions of the process
        assert exits and any(isinstance(a, Shutdown) for a in exits), node.node_executable
    assert parameters["lio_node"] == {"config_file": "/c/lio.yaml", "node_config_file": "/c/node.yaml",
                                      "events_jsonl": "/run/events_lio.jsonl", "use_sim_time": True}
    assert parameters["px4_bridge_node"] == {"config_file": "/c/bridge.yaml",
                                             "events_jsonl": "/run/events_px4_bridge.jsonl", "use_sim_time": True}
    assert parameters["parameter_bridge"]["config_file"].endswith("/bridge/px4_mid360_control_bridge.yaml")
