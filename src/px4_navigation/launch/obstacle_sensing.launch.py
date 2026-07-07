"""
Launch file for the obstacle sensing stack (Phase 1).

Brings up the livox_mid360_processor_node, which converts a Livox MID-360
style PointCloud2 stream into a 2.5D spherical grid and then publishes:
  - /fmu/in/obstacle_distance      : 72-bin message for PX4 Collision Prevention
  - /livox/grid_2d5/markers        : RViz debug MarkerArray
  - /livox/grid_2d5/min_distance   : RViz debug PointCloud2

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
        'livox_mid360_processor.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_frame = LaunchConfiguration('cloud_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'cloud_frame',
            default_value='sensor',
            description='Coordinate frame of the input point cloud: sensor (body FRD) or ned'),

        # Livox MID-360 processor — converts PointCloud2 to PX4 ObstacleDistance.
        Node(
            package='px4_navigation',
            executable='livox_mid360_processor_node',
            name='livox_mid360_processor',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    'cloud_frame': cloud_frame,
                }
            ],
            output='screen',
            remappings=[
                ('/lidar_360/points', '/lidar_360/points'),
                ('/fmu/out/vehicle_odometry', '/fmu/out/vehicle_odometry'),
                ('/fmu/in/obstacle_distance', '/fmu/in/obstacle_distance'),
                ('/livox/grid_2d5/markers', '/livox/grid_2d5/markers'),
                ('/livox/grid_2d5/min_distance', '/livox/grid_2d5/min_distance'),
            ]),
    ])
