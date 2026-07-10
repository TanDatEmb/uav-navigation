"""
Launch file for the full mapping pipeline (Phase 1).

Brings up:
    - lidar_odometry : lidar + px4 odom -> /localization/cloud + /localization/odometry
    - localization_bridge : camera_init/ENU cloud -> /world/cloud
    - global_mapper : world cloud -> sparse global voxel map
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
    enable_lidar_odometry = LaunchConfiguration('enable_lidar_odometry')

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
            default_value='px4_full',
            description='Cloud source for global_mapper node'),

        DeclareLaunchArgument(
            'enable_lidar_odometry',
            default_value='true',
            description='Run lidar_odometry to provide /localization/cloud and /localization/odometry'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_px4_mapping, 'launch', 'lidar_odometry.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(enable_lidar_odometry)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_px4_mapping, 'launch', 'localization_bridge.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'publish_visual_odometry_to_px4': publish_visual_odometry,
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
