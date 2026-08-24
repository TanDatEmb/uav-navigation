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
                    [FindPackageShare("navigation_runtime"), "config", "super_navigation.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("mission_file", default_value=""),
            Node(
                package="navigation_runtime",
                executable="super_navigation_node",
                name="super_navigation_node",
                output="screen",
                    parameters=[
                        LaunchConfiguration("config_file"),
                        {
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "super_navigation.mission_file": LaunchConfiguration("mission_file"),
                        },
                ],
            ),
        ]
    )
