from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("fast_lio_ros")) / "config" / "fast_lio_real.yaml"
    return LaunchDescription([
        Node(package="fast_lio_ros", executable="fast_lio_node", name="fast_lio",
             output="screen", parameters=[str(config)])
    ])
