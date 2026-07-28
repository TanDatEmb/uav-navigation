from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution(
        [FindPackageShare("uav_simulation"), "worlds", "m1_registration_test.sdf"]
    )
    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={"gz_args": ["-r ", world]}.items(),
            )
        ]
    )
