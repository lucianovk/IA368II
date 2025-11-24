"""
Convenience launch file that brings up robot_state_publisher for my_robot_pkg and
optionally starts RViz with the default display configuration.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = LaunchConfiguration('urdf')
    def _resolve_rviz_path():
        src_pkg_dir = None
        current_dir = Path(__file__).resolve().parent
        for ancestor in current_dir.parents:
            candidate = ancestor / 'src' / 'my_robot_pkg'
            if candidate.exists():
                src_pkg_dir = candidate
                break

        if src_pkg_dir:
            rviz_src = src_pkg_dir / 'rviz' / 'my_robot.rviz'
            if rviz_src.exists():
                return TextSubstitution(text=str(rviz_src))

        return PathJoinSubstitution(
            [
                FindPackageShare('my_robot_pkg'),
                'rviz',
                'my_robot.rviz',
            ]
        )

    rviz_path = _resolve_rviz_path()
    default_urdf = PathJoinSubstitution(
        [
            FindPackageShare('my_robot_pkg'),
            'urdf',
            'my_robot.urdf',
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
                            # Use xacro/cat to resolve the URDF path at launch time
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
                    rviz_path,
                    '--ros-args',
                    '--log-level',
                    'rviz2:=WARN',
                ],
                condition=IfCondition(LaunchConfiguration('start_rviz')),
            ),
        ]
    )
