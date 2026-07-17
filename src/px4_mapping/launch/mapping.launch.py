"""
Launch file for the mapping pipeline.

Brings up:
    - lio_px4_bridge : FAST-LIO /lio/odometry -> /fmu/in/vehicle_visual_odometry
    - global_mapper  : /lio/cloud_registered -> occupancy maps
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_px4_mapping = get_package_share_directory('px4_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time')
    input_source = LaunchConfiguration('input_source')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'input_source',
            default_value='lio_world',
            description='Cloud source for global_mapper node'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_px4_mapping, 'launch', 'lio_px4_bridge.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_px4_mapping, 'launch', 'global_mapper.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'input_source': input_source,
            }.items()
        ),
    ])
