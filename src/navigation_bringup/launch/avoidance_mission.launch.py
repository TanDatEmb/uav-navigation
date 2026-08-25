from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("navigation_bringup")
    config_file = LaunchConfiguration("config_file")
    mission_file = LaunchConfiguration("mission_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                description=(
                    "ROS parameter file containing navigation_runtime_node and "
                    "px4_navigation_external_mode sections."
                ),
            ),
            DeclareLaunchArgument(
                "mission_file",
                description="Mission YAML shared by planner backend and Avoidance Mission.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "navigation_runtime.launch.py"]
                    )
                ),
                launch_arguments={
                    "config_file": config_file,
                    "mission_file": mission_file,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "px4_external_mode.launch.py"]
                    )
                ),
                launch_arguments={
                    "config_file": config_file,
                    "mission_file": mission_file,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
