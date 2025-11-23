from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_start_delay = LaunchConfiguration('slam_start_delay')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_slam_start_delay = DeclareLaunchArgument(
        'slam_start_delay',
        default_value='1.0',
        description='Delay (seconds) before starting slam_toolbox to allow TF broadcasters to come up',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time for all nodes',
    )

    rf2o_params_file = PathJoinSubstitution(
        [
            FindPackageShare('my_robot_pkg'),
            'config',
            'rf2o_laser_odometry_config.yaml',
        ]
    )
    slam_params_file = PathJoinSubstitution(
        [
            FindPackageShare('my_robot_pkg'),
            'config',
            'slam_map_params.yaml',
        ]
    )
    log_args = ['--ros-args', '--log-level', 'warn']

    return LaunchDescription(
        [
            declare_slam_start_delay,
            declare_use_sim_time,
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_footprint_to_base_link',
                arguments=[
                    '--x',
                    '0.0',
                    '--y',
                    '0.0',
                    '--z',
                    '0.082',
                    '--qx',
                    '0.0',
                    '--qy',
                    '0.0',
                    '--qz',
                    '0.0',
                    '--qw',
                    '1.0',
                    '--frame-id',
                    'base_footprint',
                    '--child-frame-id',
                    'base_link',
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_laser_link',
                arguments=[
                    '--x',
                    '0.0',
                    '--y',
                    '0.0',
                    '--z',
                    '0.075',
                    '--qx',
                    '0.0',
                    '--qy',
                    '0.0',
                    '--qz',
                    '0.0',
                    '--qw',
                    '1.0',
                    '--frame-id',
                    'base_link',
                    '--child-frame-id',
                    'laser_link',
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='my_robot_pkg',
                executable='my_robot_sim_control_node',
                name='my_robot_sim_control',
                output='screen',
                arguments=log_args,
                # Keep use_sim_time false here so /clock is driven by wall time.
                parameters=[{'use_sim_time': False}],
            ),
            # Node(
            #     package='my_robot_pkg',
            #     executable='my_robot_odometry_node',
            #     name='my_robot_odometry_node',
            #     output='screen',
            #     arguments=log_args,
            #     parameters=[{'use_sim_time': use_sim_time}],
            # ),
            Node(
                package='my_robot_pkg',
                executable='my_robot_scan_node',
                name='my_robot_scan_node',
                output='screen',
                arguments=log_args,
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='my_robot_pkg',
                executable='my_robot_cmd_vel_node',
                name='my_robot_cmd_vel',
                output='screen',
                arguments=log_args,
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='my_robot_pkg',
                executable='my_robot_vision_node',
                name='my_robot_vision_node',
                output='screen',
                arguments=log_args,
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[rf2o_params_file, {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', 'error'],
            ),
            TimerAction(
                period=LaunchConfiguration('slam_start_delay'),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare('my_robot_pkg'),
                                    'launch',
                                    'slam_online_async.launch.py',
                                ]
                            )
                        ),
                        launch_arguments={
                            'slam_params_file': slam_params_file,
                            'use_sim_time': use_sim_time,
                        }.items(),
                    )
                ],
            ),
        ]
    )
