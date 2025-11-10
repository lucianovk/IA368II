from setuptools import setup

package_name = 'myrobot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/lifecycle_manager.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Dual-state lifecycle example for a vacuum robot (didactic)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'behaviour_server = myrobot_bringup.behaviour_server:main',
            'myrobot_server = myrobot_bringup.myrobot_server:main',
            'cleaning_server = myrobot_bringup.cleaning_server:main',
            'autodocking_server = myrobot_bringup.autodocking_server:main',
            'charging_server = myrobot_bringup.charging_server:main',
            'charged_server = myrobot_bringup.charged_server:main',
        ],
    },
)
