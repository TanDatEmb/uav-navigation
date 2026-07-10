"""
Launch file for the lidar_odometry node.

This node publishes Layer-1 contract topics:
    - /localization/cloud (camera_init frame)
    - /localization/odometry (nav_msgs/Odometry in camera_init frame)
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
            executable='lidar_odometry',
            name='lidar_odometry',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                }
            ],
            output='screen',
            remappings=[
                ('/lidar_360/points', '/lidar_360/points'),
                ('/fmu/out/vehicle_odometry', '/fmu/out/vehicle_odometry'),
                ('/localization/cloud', '/localization/cloud'),
                ('/localization/odometry', '/localization/odometry'),
            ]),
    ])
