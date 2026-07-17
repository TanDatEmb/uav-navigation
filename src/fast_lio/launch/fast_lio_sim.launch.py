#!/usr/bin/env python3
"""Launch FAST-LIO with the canonical ROS 2 parameter file.

The caller owns the sensor bridge. The workspace SITL orchestrator maps Gazebo
Harmonic MID-360 data to the launch arguments used here.

Frames:
    - ``lio_world``: gravity-aligned, Z-up LIO world with arbitrary initial yaw
    - ``mid360_imu``: FAST-LIO FLU body state
    - registered cloud output: ``lio_world``

This launch file does not publish a transform between ``lio_world`` and
``map_ned``. PX4 message conversion belongs to ``px4_mapping``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package path
    pkg_fast_lio = FindPackageShare('fast_lio')

    # Launch arguments
    config_file = LaunchConfiguration('config_file')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_fast_lio,
            'config',
            'fast_lio.params.yaml'
        ]),
        description='Path to FAST-LIO configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/sim/livox/mid360/imu',
        description='IMU topic for Gazebo MID-360 simulation'
    )

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/sim/livox/mid360/points',
        description='LiDAR topic for Gazebo MID-360 simulation snapshot'
    )

    # FAST-LIO2 node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fast_lio_node',
        name='fast_lio',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            # Input topics: node defaults are real-hardware names; remap to sim.
            ('/livox/mid360/imu', LaunchConfiguration('imu_topic')),
            ('/livox/mid360/points', LaunchConfiguration('lidar_topic')),
            # Output topics
            ('/cloud_registered', '/lio/cloud_registered'),
            ('/odometry', '/lio/odometry'),
            ('/path', '/lio/path'),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        imu_topic_arg,
        lidar_topic_arg,
        fast_lio_node,
    ])
