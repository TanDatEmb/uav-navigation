"""
Launch file for the NED transform node.

Transforms FAST-LIO2 point clouds from the camera_init/ENU frame to the
PX4 map_ned frame. Optionally publishes external vision odometry to PX4.
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
    publish_visual_odometry = LaunchConfiguration('publish_visual_odometry_to_px4')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'publish_visual_odometry_to_px4',
            default_value='false',
            description='Publish external vision odometry to PX4'),

        Node(
            package='px4_mapping',
            executable='ned_transform_node',
            name='ned_transform',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    'publish_visual_odometry_to_px4': publish_visual_odometry,
                }
            ],
            output='screen',
            remappings=[
                ('/livox_processed', '/livox_processed'),
                ('/livox_processed_ned', '/livox_processed_ned'),
                ('/odometry', '/odometry'),
                ('/fmu/out/vehicle_odometry', '/fmu/out/vehicle_odometry'),
                ('/fmu/in/vehicle_visual_odometry', '/fmu/in/vehicle_visual_odometry'),
            ]),
    ])
