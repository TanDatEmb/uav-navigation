from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = [FindPackageShare("uav_description"), "/urdf/uav_sensor_frames.urdf.xacro"]
    return LaunchDescription(
        [
            DeclareLaunchArgument("imu_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("imu_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("lidar_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("lidar_rpy", default_value="0 0 0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                *xacro_file,
                                " imu_xyz:=",
                                LaunchConfiguration("imu_xyz"),
                                " imu_rpy:=",
                                LaunchConfiguration("imu_rpy"),
                                " lidar_xyz:=",
                                LaunchConfiguration("lidar_xyz"),
                                " lidar_rpy:=",
                                LaunchConfiguration("lidar_rpy"),
                            ]
                        )
                    }
                ],
            ),
        ]
    )
