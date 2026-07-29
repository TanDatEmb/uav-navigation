from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution(
        [FindPackageShare("uav_simulation"), "worlds", "mid360_harness.sdf"]
    )
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("uav_simulation"), "bridge", "m1_sensor_bridge.yaml"]
    )
    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    frames_launch = PathJoinSubstitution(
        [FindPackageShare("uav_description"), "launch", "publish_sensor_frames.launch.py"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_estimator", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "estimator_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("fast_lio_ros"), "config", "mid360-sim.yaml"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={"gz_args": ["-r ", world]}.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="m1_sensor_bridge",
                parameters=[{"config_file": bridge_config, "use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(frames_launch),
                launch_arguments={"imu_xyz": "0 0 0", "lidar_xyz": "0 0 0.15"}.items(),
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("start_estimator")),
                package="fast_lio_ros",
                executable="fast_lio_node",
                name="fast_lio",
                output="screen",
                parameters=[LaunchConfiguration("estimator_config"), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
