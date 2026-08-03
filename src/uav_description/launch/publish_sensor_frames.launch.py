from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = [FindPackageShare("uav_description"), "/urdf/uav_sensor_frames.urdf.xacro"]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "livox_mount_xyz",
                description="Required base_link to livox_frame translation.",
            ),
            DeclareLaunchArgument(
                "livox_mount_rpy",
                description="Required base_link to livox_frame rotation.",
            ),
            DeclareLaunchArgument(
                "livox_lidar_to_imu_xyz",
                default_value="0.011 0.02329 -0.04412",
            ),
            DeclareLaunchArgument("livox_lidar_to_imu_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "robot_description": ParameterValue(
                            Command(
                                [
                                "xacro ",
                                *xacro_file,
                                " livox_mount_xyz:=\"",
                                LaunchConfiguration("livox_mount_xyz"),
                                "\" livox_mount_rpy:=\"",
                                LaunchConfiguration("livox_mount_rpy"),
                                "\" livox_lidar_to_imu_xyz:=\"",
                                LaunchConfiguration("livox_lidar_to_imu_xyz"),
                                "\" livox_lidar_to_imu_rpy:=\"",
                                LaunchConfiguration("livox_lidar_to_imu_rpy"),
                                    "\"",
                                ]
                            ),
                            value_type=str,
                        )
                    }
                ],
            ),
        ]
    )
