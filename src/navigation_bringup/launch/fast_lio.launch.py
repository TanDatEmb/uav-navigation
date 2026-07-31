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
            DeclareLaunchArgument("publish_sensor_frames", default_value="true"),
            DeclareLaunchArgument("imu_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("imu_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("lidar_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("lidar_rpy", default_value="0 0 0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(frames_launch),
                condition=IfCondition(LaunchConfiguration("publish_sensor_frames")),
                launch_arguments={
                    "imu_xyz": LaunchConfiguration("imu_xyz"),
                    "imu_rpy": LaunchConfiguration("imu_rpy"),
                    "lidar_xyz": LaunchConfiguration("lidar_xyz"),
                    "lidar_rpy": LaunchConfiguration("lidar_rpy"),
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
