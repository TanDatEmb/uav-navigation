#!/usr/bin/env python3
"""
Full Debug Session Launch

Launches complete FAST-LIO2 pipeline with rosbag recording for debugging.

Usage:
    ros2 launch fast_lio full_debug_session.launch.py
    ros2 launch fast_lio full_debug_session.launch.py with_px4:=true with_livox_adapter:=true

Components:
    1. FAST-LIO2 node
    2. LIO-PX4 alignment bridge (optional)
    3. Rosbag recorder
    4. RViz (optional)

Bag saved to: ~/bags/fastlio_debug_YYYYMMDD_HHMMSS/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Arguments
    with_px4_arg = DeclareLaunchArgument(
        'with_px4',
        default_value='false',
        description='Include LIO-PX4 alignment bridge'
    )

    with_livox_adapter_arg = DeclareLaunchArgument(
        'with_livox_adapter',
        default_value='false',
        description='Include Livox sim adapter'
    )

    with_rviz_arg = DeclareLaunchArgument(
        'with_rviz',
        default_value='true',
        description='Launch RViz'
    )

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Custom bag path (default: ~/bags/)'
    )

    bag_compression_arg = DeclareLaunchArgument(
        'bag_compression',
        default_value='zstd',
        description='Bag compression: none, zstd, lz4'
    )

    # Get conditions
    with_px4 = LaunchConfiguration('with_px4')
    with_livox_adapter = LaunchConfiguration('with_livox_adapter')
    with_rviz = LaunchConfiguration('with_rviz')

    # 1. FAST-LIO2
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'lio.launch.py'
            ])
        ]),
        launch_arguments={'profile': 'sim'}.items(),
    )

    # 2. Livox sim adapter (optional)
    livox_adapter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_sim_adapter'),
                'launch',
                'livox_sim_adapter.launch.py'
            ])
        ]),
        condition=IfCondition(with_livox_adapter),
    )

    # 3. LIO-PX4 bridge (optional)
    lio_px4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('px4_mapping'),
                'launch',
                'lio_px4_bridge.launch.py'
            ])
        ]),
        condition=IfCondition(with_px4),
    )

    # 4. Rosbag recorder
    rosbag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'record_debug_bag.launch.py'
            ])
        ]),
        launch_arguments=[
            ('bag_path', LaunchConfiguration('bag_path')),
            ('compression', LaunchConfiguration('bag_compression')),
            ('record_px4', with_px4),
        ],
    )

    # 5. RViz
    rviz_config = PathJoinSubstitution([
        FindPackageShare('fast_lio'),
        'config',
        'uav_navigation.rviz'
    ])

    rviz_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(with_rviz),
    )

    return LaunchDescription([
        # Arguments
        with_px4_arg,
        with_livox_adapter_arg,
        with_rviz_arg,
        bag_path_arg,
        bag_compression_arg,

        # Launch components
        fast_lio_launch,
        livox_adapter_launch,
        lio_px4_launch,
        rosbag_launch,
        rviz_node,
    ])
