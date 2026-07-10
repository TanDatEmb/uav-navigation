"""
Launch file for the voxel map manager node.

Builds a sparse global occupancy map in the PX4 map_ned frame from a
NED point cloud and odometry sources.
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
            description='Cloud source: lio_world, px4_only, px4_full, fast_lio2_deskew'),

        Node(
            package='px4_mapping',
            executable='voxmap_manager_node',
            name='voxel_map',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    'input_source': input_source,
                }
            ],
            output='screen',
            remappings=[
                ('/livox/world/cloud', '/livox/world/cloud'),
                ('/livox/map/global', '/livox/map/global'),
                ('/livox/l1/odometry', '/livox/l1/odometry'),
                ('/fmu/out/vehicle_odometry', '/fmu/out/vehicle_odometry'),
                ('/fmu/out/vehicle_status', '/fmu/out/vehicle_status'),
                ('/fmu/out/vehicle_local_position', '/fmu/out/vehicle_local_position'),
            ]),
    ])
