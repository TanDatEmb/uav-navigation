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
    input_source = LaunchConfiguration('input_source')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'input_source',
            default_value='px4_full',
            description='Cloud source: lio_world, px4_only, px4_full, localization_deskew'),

        Node(
            package='px4_mapping',
            executable='global_mapper',
            name='global_mapper',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    'input_source': input_source,
                }
            ],
            output='screen',
            remappings=[
                ('/world/cloud', '/world/cloud'),
                ('/mapping/global', '/mapping/global'),
                ('/mapping/local', '/mapping/local'),
                ('/localization/odometry', '/localization/odometry'),
                ('/fmu/out/vehicle_odometry', '/fmu/out/vehicle_odometry'),
                ('/fmu/out/vehicle_status', '/fmu/out/vehicle_status'),
                ('/fmu/out/vehicle_local_position', '/fmu/out/vehicle_local_position'),
            ]),
    ])
