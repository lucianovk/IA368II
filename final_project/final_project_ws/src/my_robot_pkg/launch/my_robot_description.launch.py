from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = LaunchConfiguration('urdf')
    default_urdf = PathJoinSubstitution(
        [
            FindPackageShare('my_robot_pkg'),
            'urdf',
            'my_robot.urdf',
        ]
    )
    default_rviz = PathJoinSubstitution(
        [
            FindPackageShare('my_robot_pkg'),
            'rviz',
            'my_robot.rviz',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'urdf',
                default_value=default_urdf,
                description='Path to the URDF file.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
                description='Launch RViz with the default configuration.',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[
                    {
                        'robot_description': ParameterValue(
                            Command(['cat ', urdf_file]),
                            value_type=str,
                        ),
                    }
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    default_rviz,
                    '--ros-args',
                    '--log-level',
                    'rviz2:=WARN',
                ],
                condition=IfCondition(LaunchConfiguration('start_rviz')),
            ),
        ]
    )
