#!/usr/bin/env python3
"""
PX4 + FAST-LIO2 Integration Launch File

Complete pipeline:
    Gazebo -> livox_sim_adapter -> FAST-LIO2 -> PX4 EKF2

Usage:
    ros2 launch fast_lio px4_fast_lio.launch.py

TF Tree:
    map (NED world from PX4)
    └── lio_world (gravity-aligned, FAST-LIO)
        └── mid360_imu (IMU frame from LIO)
            └── mid360_lidar (LiDAR frame)

PX4 External Odometry:
    FAST-LIO2 publishes /lio/odometry -> bridge to /fmu/in/odometry (NED frame)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_fast_lio = FindPackageShare('fast_lio')

    # Launch arguments
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    publish_tf = LaunchConfiguration('publish_tf', default='true')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_fast_lio, 'config', 'fast_lio.params.yaml'
        ]),
        description='FAST-LIO configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms'
    )

    # FAST-LIO2 node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fast_lio_node',
        name='fast_lio',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # Match Gazebo + livox_sim_adapter output
            ('/livox/imu', '/imu/out'),
            ('/livox/lidar', '/livox/lidar/pointcloud'),
            # LIO output
            ('/cloud_registered', '/lio/cloud_registered'),
            ('/odometry', '/lio/odometry'),
            ('/path', '/lio/path'),
        ],
    )

    # LIO -> PX4 odometry bridge
    # Converts LIO world (ENU-like) to PX4 world (NED)
    # This will be implemented in a separate node (lio_px4_alignment)
    # For now, placeholder: use external odometry bridge
    """
    lio_px4_bridge = Node(
        package='px4_mapping',
        executable='lio_px4_alignment',
        name='lio_px4_alignment',
        parameters=[{
            'lio_topic': '/lio/odometry',
            'px4_topic': '/fmu/in/odometry',
            'lio_world_frame': 'lio_world',
            'px4_world_frame': 'map',  # PX4 NED frame
            'body_frame': 'mid360_imu',
        }],
    )
    """

    # TF Publishers
    # Static transform: map (NED) -> lio_world (ENU-like)
    # Depends on alignment between PX4 origin and FAST-LIO origin
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='px4_to_lio_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'lio_world'],
        condition={'IfCondition': publish_tf},
    )

    # RViz2 configuration
    rviz_config = PathJoinSubstitution([
        pkg_fast_lio, 'config', 'uav_navigation.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition={'IfCondition': 'false'},  # Disabled by default
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        publish_tf_arg,
        fast_lio_node,
        static_tf_node,
        # lio_px4_bridge,  # Enable when implemented
        # rviz_node,       # Enable when rviz config is ready
    ])
