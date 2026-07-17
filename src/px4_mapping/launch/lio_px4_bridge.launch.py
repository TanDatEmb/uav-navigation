#!/usr/bin/env python3
"""Launch the unified LIO-to-PX4 bridge.

Estimates a fixed T_map_ned_lio_world transform from PX4 odometry, converts
FAST-LIO ENU/FLU poses to PX4 NED/FRD, and publishes external vision
odometry to PX4.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lio_topic_arg = DeclareLaunchArgument(
        'lio_topic',
        default_value='/lio/odometry',
        description='FAST-LIO odometry topic (ENU world / FLU body)'
    )

    px4_odom_topic_arg = DeclareLaunchArgument(
        'px4_odom_topic',
        default_value='/fmu/out/vehicle_odometry',
        description='PX4 vehicle odometry topic (NED world / FRD body)'
    )

    px4_topic_arg = DeclareLaunchArgument(
        'px4_topic',
        default_value='/fmu/in/vehicle_visual_odometry',
        description='PX4 external odometry input topic'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    align_arg = DeclareLaunchArgument(
        'align_to_px4',
        default_value='true',
        description='Estimate T_map_ned_lio_world from PX4 odometry at startup'
    )

    alignment_mode_arg = DeclareLaunchArgument(
        'alignment_mode',
        default_value='translation_only',
        description='Alignment mode: translation_only, yaw_translation, full_6dof'
    )

    quality_arg = DeclareLaunchArgument(
        'visual_odom_quality',
        default_value='100',
        description='PX4 VehicleOdometry quality in [1, 100]'
    )

    lio_px4_bridge_node = Node(
        package='px4_mapping',
        executable='lio_px4_bridge',
        name='lio_px4_bridge',
        output='screen',
        parameters=[{
            'lio_topic': LaunchConfiguration('lio_topic'),
            'px4_odom_topic': LaunchConfiguration('px4_odom_topic'),
            'px4_topic': LaunchConfiguration('px4_topic'),
            'align_to_px4': LaunchConfiguration('align_to_px4'),
            'alignment_mode': LaunchConfiguration('alignment_mode'),
            'visual_odom_quality': ParameterValue(
                LaunchConfiguration('visual_odom_quality'), value_type=int),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        lio_topic_arg,
        px4_odom_topic_arg,
        px4_topic_arg,
        use_sim_time_arg,
        align_arg,
        alignment_mode_arg,
        quality_arg,
        lio_px4_bridge_node,
    ])
