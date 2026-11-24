"""Full study setup: camera + teleop + foxglove_bridge + wizard reaction menu.

    ros2 launch misty_ros wizard.launch.py misty_ip:=192.168.1.237 \
        participant_id:=P01 condition:=BREAKDOWN

The wizard triggers every reaction from Foxglove (no state machine).

participant_id and condition can also be set/changed between sessions in
Foxglove's Parameters panel (/misty_wizard); they are read when START is pressed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('misty_ros')
    ip = LaunchConfiguration('misty_ip')
    return LaunchDescription([
        DeclareLaunchArgument('misty_ip', default_value=EnvironmentVariable('MISTY_IP', default_value='')),
        DeclareLaunchArgument('participant_id', default_value=''),
        DeclareLaunchArgument('condition', default_value='', description='CONTROL or BREAKDOWN'),
        DeclareLaunchArgument('study_config', default_value=os.path.join(share, 'config', 'study.yaml')),
        DeclareLaunchArgument('reactions', default_value=os.path.join(share, 'config', 'reactions.json')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'misty_params.yaml')),
        DeclareLaunchArgument('session_dir', default_value=os.path.expanduser('~/misty_ws/sessions')),
        DeclareLaunchArgument('nav', default_value='false', description='also run autonomous marker navigation'),
        DeclareLaunchArgument('nav_stack', default_value='nav2',
                              description='nav2 (Nav2 + robot_localization, nav2.launch.py) or custom (nav.launch.py)'),
        DeclareLaunchArgument('markers', default_value='aruco_opencv',
                              description='nav2 stack: aruco_opencv (camera stream) or json (Misty on-board detector)'),
        DeclareLaunchArgument('depth', default_value='false', description='nav2 stack: depth camera -> /scan'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'nav2.launch.py')),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('nav'), "' == 'true' and '",
                                                    LaunchConfiguration('nav_stack'), "' == 'nav2'"])),
            launch_arguments={'misty_ip': ip, 'params_file': LaunchConfiguration('params_file'),
                              'markers': LaunchConfiguration('markers'),
                              'depth': LaunchConfiguration('depth'), 'mux': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'nav.launch.py')),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('nav'), "' == 'true' and '",
                                                    LaunchConfiguration('nav_stack'), "' == 'custom'"])),
            launch_arguments={'misty_ip': ip, 'params_file': LaunchConfiguration('params_file')}.items()),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('face_tracking_dry_run', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'misty_foxglove.launch.py')),
            launch_arguments={'misty_ip': ip,
                              'params_file': LaunchConfiguration('params_file'),
                              'camera': LaunchConfiguration('camera')}.items()),
        Node(package='misty_ros', executable='face_tracker', name='misty_face_tracker', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'misty_ip': ip,
                 'dry_run': ParameterValue(LaunchConfiguration('face_tracking_dry_run'), value_type=bool),
             }]),
        Node(package='misty_ros', executable='wizard_node', name='misty_wizard', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'misty_ip': ip,
                 'participant_id': LaunchConfiguration('participant_id'),
                 'condition': LaunchConfiguration('condition'),
                 'study_config': LaunchConfiguration('study_config'),
                 'reactions': LaunchConfiguration('reactions'),
                 'session_dir': LaunchConfiguration('session_dir'),
             }]),
    ])
