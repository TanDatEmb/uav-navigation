from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = [FindPackageShare("fast_lio_ros"), "/config/mid360_real.yaml"]
    frames_launch = [
        FindPackageShare("uav_description"),
        "/launch/publish_sensor_frames.launch.py",
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the fast_lio_ros parameter YAML.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "publish_sensor_frames",
                default_value="true",
                description=(
                    "Publish the canonical static base_link -> livox_frame -> "
                    "livox_imu_frame tree."
                ),
            ),
            DeclareLaunchArgument("livox_mount_xyz"),
            DeclareLaunchArgument("livox_mount_rpy"),
            DeclareLaunchArgument(
                "livox_lidar_to_imu_xyz", default_value="0.011 0.02329 -0.04412"
            ),
            DeclareLaunchArgument("livox_lidar_to_imu_rpy", default_value="0 0 0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(frames_launch),
                condition=IfCondition(LaunchConfiguration("publish_sensor_frames")),
                launch_arguments={
                    "livox_mount_xyz": LaunchConfiguration("livox_mount_xyz"),
                    "livox_mount_rpy": LaunchConfiguration("livox_mount_rpy"),
                    "livox_lidar_to_imu_xyz": LaunchConfiguration(
                        "livox_lidar_to_imu_xyz"
                    ),
                    "livox_lidar_to_imu_rpy": LaunchConfiguration(
                        "livox_lidar_to_imu_rpy"
                    ),
                }.items(),
            ),
            Node(
                package="fast_lio_ros",
                executable="fast_lio_node",
                name="fast_lio",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
