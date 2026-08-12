from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # P1 architectural requirement: navigation_mapping runs as a separate ROS 2
    # process from fast_lio_node. It must never be included inside
    # fast_lio.launch.py's process/container; compose the two launch files
    # from a parent launch description instead.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                description="Path to the navigation_mapping_node parameter YAML.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="navigation_mapping",
                executable="navigation_mapping_node",
                name="navigation_mapping_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
