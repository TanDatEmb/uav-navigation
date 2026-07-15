#!/usr/bin/env python3
"""Launch the LIO-to-PX4 coordinate-basis and timestamp converter.

Input pose must already satisfy ENU-world/FLU-body semantics. The node converts
that representation to PX4 NED/FRD fields and converts ROS sample/publication
time to PX4 boot time through Timesync.

The historical executable name contains ``alignment``, but the node does not
estimate translation or yaw between an arbitrary ``lio_world`` origin and the
PX4 EKF origin. That alignment must be established separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    quality_arg = DeclareLaunchArgument(
        'visual_odom_quality',
        default_value='100',
        description='PX4 VehicleOdometry quality in [1, 100]'
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
            'visual_odom_quality': ParameterValue(
                LaunchConfiguration('visual_odom_quality'), value_type=int),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        lio_topic_arg,
        px4_topic_arg,
        use_sim_time_arg,
        quality_arg,
        lio_px4_alignment_node,
    ])
