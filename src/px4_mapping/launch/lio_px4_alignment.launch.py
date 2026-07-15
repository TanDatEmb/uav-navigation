#!/usr/bin/env python3
"""
LIO-PX4 Alignment Bridge Launch File

Bridge FAST-LIO2 odometry to PX4 external odometry.
Transforms from LIO world (ENU-like) to PX4 NED frame.

Usage:
    ros2 launch px4_mapping lio_px4_alignment.launch.py

Topics:
    Input:  /lio/odometry (nav_msgs/Odometry, ENU frame)
    Output: /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry, NED frame)

Frame Conventions:
    - LIO: lio_world (gravity-aligned, Z-up, ENU-like)
    - PX4: map (X-north, Y-east, Z-down, NED)

    Transform: [x, y, z] → [x, -y, -z]
               [roll, pitch, yaw] → [roll, -pitch, -yaw]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    lio_topic_arg = DeclareLaunchArgument(
        'lio_topic',
        default_value='/lio/odometry',
        description='FAST-LIO2 odometry topic'
    )

    px4_topic_arg = DeclareLaunchArgument(
        'px4_topic',
        default_value='/fmu/in/vehicle_visual_odometry',
        description='PX4 external odometry input topic'
    )

    lio_topic = LaunchConfiguration('lio_topic')
    px4_topic = LaunchConfiguration('px4_topic')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # LIO-PX4 alignment node
    lio_px4_alignment_node = Node(
        package='px4_mapping',
        executable='lio_px4_alignment',
        name='lio_px4_alignment',
        output='screen',
        parameters=[{
            'lio_topic': lio_topic,
            'px4_topic': px4_topic,
            'lio_frame_id': 'lio_world',
            'px4_frame_id': 'map',
            'use_tf_lookup': False,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        lio_topic_arg,
        px4_topic_arg,
        use_sim_time_arg,
        lio_px4_alignment_node,
    ])
