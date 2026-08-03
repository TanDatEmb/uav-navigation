from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("odometry_supervisor"),
        "config",
        "odometry_supervisor.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="odometry_supervisor",
            executable="odometry_supervisor_node",
            name="odometry_supervisor",
            output="screen",
            parameters=[LaunchConfiguration("config_file"), {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        )
    ])
