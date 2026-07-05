from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        # Mapping node placeholder.
        # Node(
        #     package='px4_mapping',
        #     executable='mapping_node',
        #     name='mapping_node',
        #     parameters=[config, {'use_sim_time': use_sim_time}],
        #     output='screen'),

        # Navigation node placeholder.
        # Node(
        #     package='px4_navigation',
        #     executable='navigation_node',
        #     name='navigation_node',
        #     parameters=[config, {'use_sim_time': use_sim_time}],
        #     output='screen'),

        # PX4 bridge node placeholder.
        # Node(
        #     package='px4_ros_com',
        #     executable='px4_bridge_node',
        #     name='px4_bridge_node',
        #     parameters=[config, {'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])
