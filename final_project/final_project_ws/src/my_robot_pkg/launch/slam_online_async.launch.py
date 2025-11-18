import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import yaml


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    log_level = LaunchConfiguration('log_level')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description=(
            'Automatically startup the slamtoolbox. '
            'Ignored when use_lifecycle_manager is true.'
        ),
    )
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager',
        default_value='false',
        description='Enable bond connection during node activation',
    )
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock',
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('slam_toolbox'),
            'config',
            'mapper_params_online_async.yaml',
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )
    declare_log_level_argument = DeclareLaunchArgument(
        'log_level',
        default_value='',
        description=(
            'Optional RCUTILS log level override; leave empty to use '
            'the log_level value from the provided slam params file.'
        ),
    )

    def _configure_log_level(context, *args, **kwargs) -> List[SetLaunchConfiguration]:
        level_override = log_level.perform(context)
        if level_override:
            return []
        params_file_path = slam_params_file.perform(context)
        resolved_level = 'info'
        if os.path.exists(params_file_path):
            with open(params_file_path, 'r') as stream:
                yaml_data = yaml.safe_load(stream) or {}
            resolved_level = (
                yaml_data.get('slam_toolbox', {})
                .get('ros__parameters', {})
                .get('log_level', resolved_level)
            )
        return [SetLaunchConfiguration('log_level', str(resolved_level).lower())]

    set_log_level_from_params = OpaqueFunction(function=_configure_log_level)

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
            {
                'use_lifecycle_manager': use_lifecycle_manager,
                'use_sim_time': use_sim_time,
            },
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        arguments=['--ros-args', '--log-level', log_level],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] Slamtoolbox node is activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(
                            start_async_slam_toolbox_node
                        ),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_log_level_argument)
    ld.add_action(set_log_level_from_params)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
