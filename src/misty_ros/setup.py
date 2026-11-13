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
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='ROS 2 bridge for Misty II camera and teleop',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_node = misty_ros.camera_node:main',
            'teleop_node = misty_ros.teleop_node:main',
            'wizard_auto = misty_ros.wizard_node:main',
            'wizard_manual = misty_ros.wizard_manual_node:main',
        ],
    },
)
