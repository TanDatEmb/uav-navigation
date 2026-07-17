"""
Launch file for the voxel map manager node.

Builds accumulated global occupancy and a radius-bounded local view from a
point cloud plus synchronized odometry.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('px4_mapping'),
        'config',
        'defaults.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        Node(
            package='px4_mapping',
            executable='global_mapper',
            name='global_mapper',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                }
            ],
            output='screen',
            remappings=[
                ('/lio/cloud_registered', '/lio/cloud_registered'),
                ('/mapping/occupancy/global', '/mapping/occupancy/global'),
                ('/mapping/occupancy/local', '/mapping/occupancy/local'),
                ('/lio/odometry', '/lio/odometry'),
                ('/fmu/out/vehicle_odometry', '/fmu/out/vehicle_odometry'),
                ('/fmu/out/vehicle_status', '/fmu/out/vehicle_status'),
                ('/fmu/out/vehicle_local_position', '/fmu/out/vehicle_local_position'),
            ]),
    ])
