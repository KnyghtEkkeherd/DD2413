"""Autonomous marker navigation: sensor bridge + EKF-SLAM + navigator.

    ros2 launch misty_ros nav.launch.py misty_ip:=192.168.1.237   # real robot
    ros2 launch misty_ros nav.launch.py sim:=true                 # simulator only

Usually started from wizard.launch.py with nav:=true, which also runs the
teleop node (executes /cmd_vel on Misty), the wizard (reactions) and Foxglove.
With sim:=true the simulated Misty replaces the sensor bridge and the teleop
node. Settings: config/misty_params.yaml (misty_sensors, misty_slam, misty_nav).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('misty_ros'))
    params = LaunchConfiguration('params_file')
    sim = LaunchConfiguration('sim')
    return LaunchDescription([
        DeclareLaunchArgument('misty_ip', default_value=EnvironmentVariable('MISTY_IP', default_value='')),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'misty_params.yaml')),
        DeclareLaunchArgument('sim', default_value='false', description='simulated Misty instead of the robot'),
        Node(package='misty_ros', executable='sensors_node', name='misty_sensors', output='screen',
             condition=UnlessCondition(sim),
             parameters=[params, {'misty_ip': LaunchConfiguration('misty_ip')}]),
        Node(package='misty_ros', executable='nav_sim', name='misty_nav_sim', output='screen',
             condition=IfCondition(sim), parameters=[params]),
        Node(package='misty_ros', executable='slam_node', name='misty_slam', output='screen',
             parameters=[params]),
        Node(package='misty_ros', executable='nav_node', name='misty_nav', output='screen',
             parameters=[params]),
    ])
