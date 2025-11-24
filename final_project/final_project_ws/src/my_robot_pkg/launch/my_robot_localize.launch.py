"""
Localization bringup: starts the simulated sensors, RF2O odometry, semantic stack,
and a delayed slam_toolbox instance for pose-graph localization.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the localization stack plus semantics/perception helpers."""
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
            'slam_loc_params.yaml',
        ]
    )
    def _resolve_pose_graph_base():
        src_pkg_dir = None
        current_dir = Path(__file__).resolve().parent
        for ancestor in current_dir.parents:
            candidate = ancestor / 'src' / 'my_robot_pkg'
            if candidate.exists():
                src_pkg_dir = candidate
                break

        if src_pkg_dir:
            pose_graphs_dir = src_pkg_dir / 'pose_graphs'
            pose_graphs_dir.mkdir(parents=True, exist_ok=True)
            base = pose_graphs_dir / 'my_robot_pose_graph'
            return TextSubstitution(text=str(base))
        fallback = PathJoinSubstitution(
            [
                FindPackageShare('my_robot_pkg'),
                'pose_graphs',
                'my_robot_pose_graph',
            ]
        )
        fallback_dir = Path(__file__).resolve().parent / 'pose_graphs'
        fallback_dir.mkdir(parents=True, exist_ok=True)
        return fallback

    pose_graph_path = _resolve_pose_graph_base()
    slam_params_with_posegraph = RewrittenYaml(
        source_file=slam_params_file,
        root_key='slam_toolbox',
        param_rewrites={'map_file_name': pose_graph_path},
        convert_types=True,
    )
    log_args = ['--ros-args', '--log-level', 'warn']

    return LaunchDescription(
        [
            declare_slam_start_delay,
            declare_use_sim_time,
            # The robot publishes odometry in base_link, so we hand out static transforms
            # upfront to guarantee TF consumers have a sane tree before sensors start.
            # Bridge TF from base_footprint up to laser_link to match the simulated kinematics
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
            # Core teleop/perception stack so localization can run autonomously.
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
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[rf2o_params_file, {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', 'error'],
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
                package='my_robot_pkg',
                executable='my_robot_topologic_segmentation_node',
                name='my_robot_topologic_segmentation_node',
                output='screen',
                arguments=log_args,
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='my_robot_pkg',
                executable='my_robot_detection_node',
                name='my_robot_detection_node',
                output='screen',
                arguments=log_args,
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='my_robot_pkg',
                executable='my_robot_semantic_segmentation_node',
                name='my_robot_semantic_segmentation_node',
                output='screen',
                arguments=log_args,
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='my_robot_pkg',
                executable='my_robot_explorer_node',
                name='my_robot_explorer_node',
                output='screen',
                arguments=log_args,
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            TimerAction(
                period=LaunchConfiguration('slam_start_delay'),
                actions=[
                    # Defer slam_toolbox until the TF tree (base_link -> odom) is live.
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
                            'slam_params_file': slam_params_with_posegraph,
                            'use_sim_time': use_sim_time,
                            'log_level': 'warn',
                        }.items(),
                    )
                ],
            ),
        ]
    )
