from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                description="Path to px4_navigation_external_mode parameters.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="px4_navigation_external_mode",
                executable="px4_navigation_external_mode_node",
                name="px4_navigation_external_mode",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
