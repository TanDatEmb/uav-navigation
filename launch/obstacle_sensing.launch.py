"""
Launch file for the obstacle sensing stack (Phase 1).

Brings up only the obstacle_distance_publisher_node, which converts a
PointCloud2 stream (NED frame) into a 72-bin ObstacleDistance message
for PX4 Collision Prevention.

Future phases will add voxmap_manager_node, collision_avoidance_node, etc.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Config file path.
    config_file = os.path.join(
        get_package_share_directory('px4_navigation'),
        'config',
        'obstacle_distance_publisher.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        # Obstacle distance publisher — converts PointCloud2 to PX4 ObstacleDistance.
        Node(
            package='px4_navigation',
            executable='obstacle_distance_publisher_node',
            name='obstacle_distance_publisher',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            output='screen',
            remappings=[
                ('/livox_processed_ned', '/livox_processed_ned'),
                ('/fmu/out/vehicle_odometry', '/fmu/out/vehicle_odometry'),
                ('/fmu/in/obstacle_distance', '/fmu/in/obstacle_distance'),
            ]),
    ])