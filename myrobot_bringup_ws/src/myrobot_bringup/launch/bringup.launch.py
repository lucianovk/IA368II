"""
bringup.launch.py
-----------------
Starts all nodes and the Nav2 lifecycle manager. The manager configures and
activates nodes; behaviour_server controls a *separate* operational mode.

Tip: This example is didactic. Replace the stub logic inside each node with
real robot logic while keeping the same topic/service contracts.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('myrobot_bringup'),
        'config',
        'lifecycle_manager.yaml'
    )

    return LaunchDescription([
        Node(package='myrobot_bringup', executable='myrobot_server', name='myrobot', output='screen'),
        Node(package='myrobot_bringup', executable='behaviour_server', name='behaviour_server', output='screen'),
        Node(package='myrobot_bringup', executable='cleaning_server', name='cleaning_server', output='screen'),
        Node(package='myrobot_bringup', executable='autodocking_server', name='autodocking_server', output='screen'),
        Node(package='myrobot_bringup', executable='charging_server', name='charging_server', output='screen'),
        Node(package='myrobot_bringup', executable='charged_server', name='charged_server', output='screen'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[cfg],
        ),
    ])
