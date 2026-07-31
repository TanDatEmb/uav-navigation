from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_estimator = LaunchConfiguration("start_estimator")
    bridge_config = LaunchConfiguration("bridge_config")
    estimator_config = LaunchConfiguration("estimator_config")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_estimator", default_value="true"),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("uav_simulation"), "bridge", "px4_mid360_bridge.yaml"
            ]),
        ),
        DeclareLaunchArgument(
            "estimator_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("fast_lio_ros"), "config", "mid360-px4-sim.yaml"
            ]),
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="px4_mid360_bridge", output="screen",
            parameters=[{"config_file": bridge_config, "use_sim_time": use_sim_time}],
        ),
        Node(
            condition=IfCondition(start_estimator),
            package="fast_lio_ros", executable="fast_lio_node",
            name="fast_lio", output="screen",
            parameters=[estimator_config, {"use_sim_time": use_sim_time}],
        ),
    ])
