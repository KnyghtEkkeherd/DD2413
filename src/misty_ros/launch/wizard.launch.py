"""Full study setup: camera + teleop + foxglove_bridge + wizard state machine.

    ros2 launch misty_ros wizard.launch.py misty_ip:=192.168.1.237 \
        participant_id:=P01 condition:=BREAKDOWN

participant_id and condition can also be set/changed between sessions in
Foxglove's Parameters panel (/misty_wizard); they are read when START is pressed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('misty_ros')
    ip = LaunchConfiguration('misty_ip')
    return LaunchDescription([
        DeclareLaunchArgument('misty_ip', default_value=EnvironmentVariable('MISTY_IP', default_value='')),
        DeclareLaunchArgument('participant_id', default_value=''),
        DeclareLaunchArgument('condition', default_value='', description='CONTROL or BREAKDOWN'),
        DeclareLaunchArgument('study_config', default_value=os.path.join(share, 'config', 'study.yaml')),
        DeclareLaunchArgument('speech_bank', default_value=os.path.join(share, 'config', 'speech_bank.yaml')),
        DeclareLaunchArgument('session_dir', default_value=os.path.expanduser('~/misty_ws/sessions')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'misty_foxglove.launch.py')),
            launch_arguments={'misty_ip': ip}.items()),
        Node(package='misty_ros', executable='wizard_node', name='misty_wizard', output='screen',
             parameters=[{
                 'misty_ip': ip,
                 'participant_id': LaunchConfiguration('participant_id'),
                 'condition': LaunchConfiguration('condition'),
                 'study_config': LaunchConfiguration('study_config'),
                 'speech_bank': LaunchConfiguration('speech_bank'),
                 'session_dir': LaunchConfiguration('session_dir'),
             }]),
    ])
