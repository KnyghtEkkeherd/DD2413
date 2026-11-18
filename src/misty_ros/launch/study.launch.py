"""The whole study in one launch: navigation + wizard + study state machine + controller.

    ros2 launch misty_ros study.launch.py misty_ip:=192.168.1.237 participant_id:=P01 condition:=BREAKDOWN
    ros2 launch misty_ros study.launch.py sim:=2d          # dry run: 2D simulator + fake Misty, no robot

Starts
  misty_foxglove.launch.py   camera, teleop (drives Misty), Xbox controller, keyboard-free
                             teleop relay, twist_mux, foxglove_bridge
  wizard_node                speech / sounds / emotions / gestures / session CSV
  study_node                 study state machine (config/study.yaml)
  nav2.launch.py             Nav2 + robot_localization + marker detection + mission
  (sim:=2d) nav_sim + scripts/fake_misty.py instead of the robot

Who drives (twist_mux): the wizard's controller / Foxglove DRIVE panel always win,
then the study node, then Nav2. STOP pauses the study and Nav2; RESUME continues.

participant_id / condition can also be set in Foxglove (Parameters -> /misty_wizard)
before pressing START; the study starts at START and ends at END (or its END state).
study.yaml and reactions.json are read from src/ (edits apply at the next START).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)
    share = Path(get_package_share_directory('misty_ros'))
    ws = share.parents[3]
    sim = arg('sim')
    ip = arg('misty_ip') or ('127.0.0.1:8099' if sim != 'none' else '')
    if not ip:
        raise RuntimeError('misty_ip is required (or set MISTY_IP), or use sim:=2d for a dry run')
    params = arg('params_file')
    src_cfg = ws / 'src' / 'misty_ros' / 'config'          # editable without rebuilding
    cfg = lambda name: str(src_cfg / name if (src_cfg / name).is_file() else share / 'config' / name)
    study_cfg = arg('study_config') or cfg('study.yaml')
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'misty_foxglove.launch.py')),
            launch_arguments={'misty_ip': ip, 'params_file': params, 'robot': 'true' if sim == 'none' else 'false',
                              'gamepad_local': arg('gamepad_local')}.items()),
        Node(package='misty_ros', executable='wizard_node', name='misty_wizard', output='screen',
             parameters=[params, {'misty_ip': ip, 'participant_id': arg('participant_id'),
                                  'condition': arg('condition'), 'study_config': study_cfg,
                                  'reactions': arg('reactions') or cfg('reactions.json'),
                                  'session_dir': arg('session_dir')}]),
        Node(package='misty_ros', executable='study_node', name='misty_study', output='screen',
             parameters=[{'study_config': study_cfg}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'nav2.launch.py')),
            launch_arguments={'misty_ip': ip, 'params_file': params, 'sim': sim, 'markers': arg('markers'),
                              'depth': arg('depth'), 'mux': 'false'}.items()),
    ]
    if sim != 'none':
        actions.append(ExecuteProcess(cmd=['python3', str(ws / 'scripts' / 'fake_misty.py'), ip.rsplit(':', 1)[1]],
                                      name='fake_misty', output='log'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('misty_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('misty_ip', default_value=EnvironmentVariable('MISTY_IP', default_value='')),
        DeclareLaunchArgument('participant_id', default_value=''),
        DeclareLaunchArgument('condition', default_value='', description='CONTROL or BREAKDOWN'),
        DeclareLaunchArgument('sim', default_value='none', description='none (robot) | 2d (dry run, no robot)'),
        DeclareLaunchArgument('markers', default_value='',
                              description='aruco_opencv (camera stream) | json (Misty on-board detector)'),
        DeclareLaunchArgument('depth', default_value='false', description='depth camera -> Nav2 obstacle scan'),
        DeclareLaunchArgument('gamepad_local', default_value='false',
                              description='controller plugged into this machine (else scripts/gamepad_sender.py)'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'misty_params.yaml')),
        DeclareLaunchArgument('study_config', default_value=''),
        DeclareLaunchArgument('reactions', default_value=''),
        DeclareLaunchArgument('session_dir', default_value=str(Path.home() / 'misty_ws' / 'sessions')),
        OpaqueFunction(function=setup),
    ])
