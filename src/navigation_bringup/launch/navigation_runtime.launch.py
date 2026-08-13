from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Navigation runtime runs as a separate ROS 2
    # process from fast_lio_node. It must never be included inside
    # fast_lio.launch.py's process/container; compose the two launch files
    # from a parent launch description instead.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                description="Path to the navigation_runtime parameter YAML.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="navigation_runtime",
                executable="navigation_runtime_node",
                name="navigation_runtime",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
