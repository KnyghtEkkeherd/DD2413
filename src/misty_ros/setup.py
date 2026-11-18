from glob import glob

from setuptools import find_packages, setup

package_name = 'misty_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='ROS 2 bridge for Misty II camera and teleop',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_node = misty_ros.camera_node:main',
            'teleop_node = misty_ros.teleop_node:main',
            'wizard_node = misty_ros.wizard_node:main',
            'keyboard_teleop = misty_ros.keyboard_teleop:main',
            'gamepad_teleop = misty_ros.gamepad_teleop:main',
            'sensors_node = misty_ros.sensors_node:main',
            'slam_node = misty_ros.slam_node:main',
            'nav_node = misty_ros.nav_node:main',
            'nav_sim = misty_ros.nav_sim_node:main',
            'gz_adapter = misty_ros.gz_adapter_node:main',
            'wheel_odom = misty_ros.wheel_odom_node:main',
            'marker_localizer = misty_ros.marker_localizer_node:main',
            'mission_node = misty_ros.mission_node:main',
            'teleop_relay = misty_ros.teleop_relay_node:main',
            'study_node = misty_ros.study_node:main',
        ],
    },
)
