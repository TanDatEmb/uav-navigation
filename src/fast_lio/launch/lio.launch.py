#!/usr/bin/env python3
"""Launch FAST-LIO2 with runtime-selectable input profiles.

The caller selects a sensor profile via the ``profile`` launch argument; the
common estimator parameters are always loaded from ``common.yaml``. No
rebuild is required to switch between simulation and hardware input formats.

Available profiles:
    - ``sim``: Gazebo MID-360 GPU-LiDAR XYZI snapshot.
    - ``mid360_pointcloud2``: Livox ROS Driver 2 PointCloud2 (xfer_format=0).
    - ``mid360_custom``: Livox ROS Driver 2 CustomMsg (xfer_format=1).

Frames:
    - ``lio_world``: gravity-aligned, Z-up LIO world with arbitrary initial yaw
    - ``mid360_imu``: FAST-LIO FLU body state
    - registered cloud output: ``lio_world``

This launch file does not publish a transform between ``lio_world`` and
``map_ned``. PX4 message conversion belongs to ``px4_mapping``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_fast_lio = FindPackageShare('fast_lio')

    profile = LaunchConfiguration('profile')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Whitelist: profile name -> YAML file name
    PROFILE_WHITELIST = {
        'sim': 'simulation',
        'mid360_custom': 'mid360_custom',
        'mid360_pointcloud2': 'mid360_pointcloud2',
    }

    def launch_with_profile(context, *args, **kwargs):
        """Select profile YAML based on launch argument (whitelist only)."""
        profile_name = context.perform_substitution(profile)

        if profile_name not in PROFILE_WHITELIST:
            raise ValueError(
                f"Invalid profile '{profile_name}'. "
                f"Must be one of: {', '.join(PROFILE_WHITELIST.keys())}"
            )

        yaml_name = PROFILE_WHITELIST[profile_name]

        common_config = PathJoinSubstitution([
            pkg_fast_lio, 'config', 'common.yaml'
        ])
        profile_config = PathJoinSubstitution([
            pkg_fast_lio, 'config', f'{yaml_name}.yaml'
        ])

        return [
            Node(
                package='fast_lio',
                executable='fast_lio_node',
                name='fast_lio',
                output='screen',
                parameters=[
                    common_config,
                    profile_config,
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('/cloud_registered', '/lio/cloud_registered'),
                    ('/odometry', '/lio/odometry'),
                    ('/path', '/lio/path'),
                ],
                arguments=['--ros-args', '--log-level', 'info'],
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='sim',
            description='Input profile: sim, mid360_pointcloud2, or mid360_custom'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),

        OpaqueFunction(function=launch_with_profile),
    ])
