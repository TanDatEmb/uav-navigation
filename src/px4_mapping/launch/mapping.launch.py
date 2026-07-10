"""
Launch file for the full mapping pipeline (Phase 1).

Brings up:
    - fast_lio2_node : lidar + px4 odom -> /livox_processed + /odometry
    - ned_transform_node : camera_init/ENU cloud -> map_ned cloud
    - voxmap_manager_node : map_ned cloud -> sparse global voxel map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_px4_mapping = get_package_share_directory('px4_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_visual_odometry = LaunchConfiguration('publish_visual_odometry_to_px4')
    input_source = LaunchConfiguration('input_source')
    enable_fast_lio2 = LaunchConfiguration('enable_fast_lio2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'publish_visual_odometry_to_px4',
            default_value='false',
            description='Publish external vision odometry to PX4'),

        DeclareLaunchArgument(
            'input_source',
            default_value='lio_world',
            description='Cloud source for voxmap_manager_node'),

        DeclareLaunchArgument(
            'enable_fast_lio2',
            default_value='true',
            description='Run FAST-LIO2 adapter to provide /livox_processed and /odometry'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_px4_mapping, 'launch', 'fast_lio2.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(enable_fast_lio2)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_px4_mapping, 'launch', 'ned_transform.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'publish_visual_odometry_to_px4': publish_visual_odometry,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_px4_mapping, 'launch', 'voxmap_manager.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'input_source': input_source,
            }.items()
        ),
    ])
