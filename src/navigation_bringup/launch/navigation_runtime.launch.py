from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("navigation_runtime"), "config", "navigation_runtime.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("mission_file", default_value=""),
            Node(
                package="navigation_runtime",
                executable="navigation_runtime_node",
                name="navigation_runtime_node",
                output="screen",
                    parameters=[
                        LaunchConfiguration("config_file"),
                        {
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "navigation_runtime.mission_file": LaunchConfiguration("mission_file"),
                        },
                ],
            ),
        ]
    )
