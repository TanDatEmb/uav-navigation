#!/usr/bin/env python3
"""
FAST-LIO2 Debug Bag Recording Launch

Records all relevant topics for offline debugging and analysis.
Run this alongside FAST-LIO2 to capture data for post-flight analysis.

Usage:
    ros2 launch fast_lio record_debug_bag.launch.py bag_path:=$HOME/bags/debug_session

Topics recorded:
    - IMU raw data
    - LiDAR point clouds
    - FAST-LIO2 odometry
    - PX4 vehicle states
    - Transforms
    - Debug markers
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import datetime


def generate_launch_description():
    # Launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Base path for bag file. If empty, uses ~/bags/fastlio_debug_YYYYMMDD_HHMMSS'
    )

    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='',
        description='Bag file name. If empty, auto-generated with timestamp'
    )

    record_imu_arg = DeclareLaunchArgument(
        'record_imu',
        default_value='true',
        description='Record IMU topics'
    )

    record_lidar_arg = DeclareLaunchArgument(
        'record_lidar',
        default_value='true',
        description='Record LiDAR topics'
    )

    record_px4_arg = DeclareLaunchArgument(
        'record_px4',
        default_value='true',
        description='Record PX4 vehicle topics'
    )

    record_tf_arg = DeclareLaunchArgument(
        'record_tf',
        default_value='true',
        description='Record TF transforms'
    )

    record_lio_arg = DeclareLaunchArgument(
        'record_lio',
        default_value='true',
        description='Record FAST-LIO2 output topics'
    )

    max_bag_size_arg = DeclareLaunchArgument(
        'max_bag_size',
        default_value='0',
        description='Max bag size in bytes (0 = unlimited)'
    )

    max_bag_duration_arg = DeclareLaunchArgument(
        'max_bag_duration',
        default_value='0',
        description='Max bag duration in seconds (0 = unlimited)'
    )

    compression_arg = DeclareLaunchArgument(
        'compression',
        default_value='zstd',
        description='Compression mode: none, zstd, or lz4'
    )

    def create_rosbag_process(context):
        bag_path = LaunchConfiguration('bag_path').perform(context)
        bag_name = LaunchConfiguration('bag_name').perform(context)

        # Auto-generate bag name if not provided
        if not bag_name:
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            bag_name = f'fastlio_debug_{timestamp}'

        # Auto-generate path if not provided
        if not bag_path:
            bag_path = f'/home/letandat/bags'

        full_bag_path = f'{bag_path}/{bag_name}'

        # Build topic list
        topics = []

        if LaunchConfiguration('record_imu').perform(context) == 'true':
            topics.extend([
                '/livox/imu',
                '/imu/out',
                '/fmu/out/vehicle_imu',
            ])

        if LaunchConfiguration('record_lidar').perform(context) == 'true':
            topics.extend([
                '/livox/lidar',
                '/livox/lidar/pointcloud',
                '/fmu/out/vehicle_distance_sensor',
            ])

        if LaunchConfiguration('record_lio').perform(context) == 'true':
            topics.extend([
                '/lio/odometry',
                '/lio/path',
                '/lio/cloud_registered',
                '/lio/eskf_debug',
                '/lio/mapping_update_time',
            ])

        if LaunchConfiguration('record_px4').perform(context) == 'true':
            topics.extend([
                '/fmu/out/vehicle_local_position',
                '/fmu/out/vehicle_attitude',
                '/fmu/out/vehicle_odometry',
                '/fmu/in/vehicle_visual_odometry',
                '/fmu/out/vehicle_status',
                '/fmu/out/vehicle_control_mode',
            ])

        if LaunchConfiguration('record_tf').perform(context) == 'true':
            topics.extend([
                '/tf',
                '/tf_static',
            ])

        # Additional debug topics
        topics.extend([
            '/rosout',
            '/parameter_events',
        ])

        # Remove duplicates
        topics = list(dict.fromkeys(topics))

        # Build rosbag2 record command
        compression = LaunchConfiguration('compression').perform(context)
        max_size = LaunchConfiguration('max_bag_size').perform(context)
        max_duration = LaunchConfiguration('max_bag_duration').perform(context)

        cmd = [
            'ros2', 'bag', 'record',
            '-o', full_bag_path,
            '--storage', 'mcap',
        ]

        if compression != 'none':
            cmd.extend(['--compression-mode', 'file', '--compression-format', compression])

        if max_size != '0':
            cmd.extend(['--max-bag-size', max_size])

        if max_duration != '0':
            cmd.extend(['--max-bag-duration', max_duration])

        cmd.extend(topics)

        print(f"Recording bag to: {full_bag_path}")
        print(f"Topics: {len(topics)} topics")

        return [ExecuteProcess(
            cmd=cmd,
            output='screen',
            shell=False,
        )]

    record_action = OpaqueFunction(function=create_rosbag_process)

    return LaunchDescription([
        bag_path_arg,
        bag_name_arg,
        record_imu_arg,
        record_lidar_arg,
        record_px4_arg,
        record_tf_arg,
        record_lio_arg,
        max_bag_size_arg,
        max_bag_duration_arg,
        compression_arg,
        record_action,
    ])
