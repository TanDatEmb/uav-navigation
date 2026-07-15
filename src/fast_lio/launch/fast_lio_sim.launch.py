#!/usr/bin/env python3
"""
FAST-LIO2 Launch File for Gazebo + PX4 SITL

Prerequisites:
    - PX4 SITL running with Mid360 LiDAR model
    - Gazebo simulation running
    - livox_sim_adapter (or similar) converting Gazebo point cloud to ROS2

Usage:
    ros2 launch fast_lio fast_lio_sim.launch.py

Frame Convention:
    - world_frame: "lio_world" - gravity-aligned FAST-LIO world
    - body_frame: "mid360_imu" - IMU frame from PX4
    - lidar_frame: "mid360_lidar" - LiDAR optical center
    - map_frame: "lio_map" - registered point cloud frame

Configuration:
    Edit config/fast_lio.yaml for algorithm parameters
"""

import os
import ament_index_python
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
    # Static config path for the node's YAML loader (resolved at launch time)
    config_path_str = os.path.join(
        ament_index_python.get_package_share_directory('fast_lio'),
        'config', 'fast_lio_config.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/out',
        description='IMU topic for simulation'
    )

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar/pointcloud',
        description='LiDAR topic for simulation'
    )

    # FAST-LIO2 node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fast_lio_node',
        name='fast_lio',
        output='screen',
        parameters=[config_file,
                      {'use_sim_time': LaunchConfiguration('use_sim_time'),
                       'config_path': config_path_str}],
        remappings=[
            # Input topics (adjust if needed)
            ('/livox/imu', LaunchConfiguration('imu_topic')),
            ('/livox/lidar', LaunchConfiguration('lidar_topic')),
            # Output topics
            ('/cloud_registered', '/lio/cloud_registered'),
            ('/odometry', '/lio/odometry'),
            ('/path', '/lio/path'),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # Static transform: map -> lio_world (identity at startup)
    # This allows visualization in RViz
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lio_world_to_map',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'lio_world'],
    )

    # Optional: Point cloud processor for visualization
    # Downsamples registered cloud for RViz
    voxel_grid_node = Node(
        package='pcl_ros',
        executable='voxel_grid_node',
        name='voxel_grid',
        parameters=[{
            'leaf_size': 0.05,
            'filter_field_name': '',
            'filter_limit_min': -1000.0,
            'filter_limit_max': 1000.0,
        }],
        remappings=[
            ('/input', '/lio/cloud_registered'),
            ('/output', '/lio/cloud_registered_voxel'),
        ],
        condition={'IfCondition': 'false'},  # Disabled by default
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        imu_topic_arg,
        lidar_topic_arg,
        fast_lio_node,
        static_tf_node,
        # voxel_grid_node,  # Enable if needed
    ])
