from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['my_robot', 'my_robot.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=[
        'setuptools',
        'coppeliasim-zmqremoteapi-client',
        'numpy',
        'tf-transformations',
        'scipy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='adrolab',
    maintainer_email='adrolab.unicamp@gmail.com',
    description='Custom nodes for the IA368 final project robot.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_scan_node = my_robot.my_robot_scan_node:main',
            'my_robot_odometry_node = my_robot.my_robot_odometry_node:main',
            'my_robot_sim_control_node = my_robot.my_robot_sim_control_node:main',
            'my_robot_cmd_vel_node = my_robot.my_robot_cmd_vel_node:main',
            'my_robot_keyboard_node = my_robot.my_robot_keyboard_node:main',
            'my_robot_explorer_node = my_robot.my_robot_explorer_node:main',
            'my_robot_explorer_node_2 = my_robot.my_robot_explorer_node_2:main',
            'my_robot_vision_node = my_robot.my_robot_vision_node:main',
            'my_robot_yolo_key_node = my_robot.my_robot_yolo_key_node:main',
            'my_robot_room_segmentation_node = my_robot.my_robot_room_segmentation_node:main',
            
        ],
    },
)
