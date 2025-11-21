from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('my_robot_pkg'), 'config', 'explore_config.yaml']
                ),
                description='YAML file with explorer parameters',
            ),
            Node(
                package='my_robot_pkg',
                executable='my_robot_explore_node',
                name='frontier_explorer',
                output='screen',
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', 'info'],
            ),
        ]
    )
