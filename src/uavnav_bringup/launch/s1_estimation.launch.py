"""The S1 estimation stack for SITL (S1b Task 11; SYSTEM_DESIGN §6.1, §7.2; D14, D22; D33 O19, O24).

Starts, every one with use_sim_time true (sensor and PX4 time are /clock time in SITL, D29):
  - ros_gz_bridge parameter_bridge with uav_simulation's px4_mid360_control_bridge.yaml: /clock, /lidar/imu and
    the evaluation-only /sim/ground_truth/odometry;
  - uav_simulation gz_lidar_bridge: /lidar/points;
  - uavnav_lio lio_node: config_file, node_config_file, events_jsonl = <events_dir>/events_lio.jsonl;
  - uavnav_px4_bridge px4_bridge_node: config_file, events_jsonl = <events_dir>/events_px4_bridge.jsonl.

Four required arguments with no defaults (D22: tunables have one source, the config files):
  lio_config        config/lio/sim.yaml       (kLioSpecs)
  lio_node_config   config/lio/node_sim.yaml  (kLioNodeSpecs)
  bridge_config     config/px4_bridge/sim.yaml (kAlignmentSpecs)
  events_dir        an existing directory for the two JSONL event logs
A node refuses to start on a missing or invalid file (one FATAL line), and the exit of any process ends the
launch, so a supervisor (tools/uavnav/sitl_hover.sh) sees it. PX4 SITL, Gazebo and MicroXRCEAgent are not started
here.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

REQUIRED_ARGUMENTS = (
    ("lio_config", "LIO estimator parameters (config/lio/sim.yaml)"),
    ("lio_node_config", "lio node parameters: the base_link <- LiDAR mount (config/lio/node_sim.yaml)"),
    ("bridge_config", "PX4 bridge alignment parameters (config/px4_bridge/sim.yaml)"),
    ("events_dir", "existing directory for events_lio.jsonl and events_px4_bridge.jsonl"),
)


def _node(package, executable, parameters, name=None):
    return Node(package=package, executable=executable, name=name, output="screen",
                parameters=[dict(parameters, use_sim_time=True)],
                on_exit=[Shutdown(reason=f"{package}/{executable} exited")])


def generate_launch_description():
    sim_share = get_package_share_directory("uav_simulation")
    events_dir = LaunchConfiguration("events_dir")
    return LaunchDescription(
        [DeclareLaunchArgument(name, description=text) for name, text in REQUIRED_ARGUMENTS] + [
            _node("ros_gz_bridge", "parameter_bridge",
                  {"config_file": os.path.join(sim_share, "bridge", "px4_mid360_control_bridge.yaml")},
                  name="px4_mid360_control_bridge"),
            _node("uav_simulation", "gz_lidar_bridge", {}),
            _node("uavnav_lio", "lio_node",
                  {"config_file": LaunchConfiguration("lio_config"),
                   "node_config_file": LaunchConfiguration("lio_node_config"),
                   "events_jsonl": PathJoinSubstitution([events_dir, "events_lio.jsonl"])}),
            _node("uavnav_px4_bridge", "px4_bridge_node",
                  {"config_file": LaunchConfiguration("bridge_config"),
                   "events_jsonl": PathJoinSubstitution([events_dir, "events_px4_bridge.jsonl"])}),
        ])
