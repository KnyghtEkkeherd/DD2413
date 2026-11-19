"""Start Misty camera, teleop, controller nodes and foxglove_bridge.

    ros2 launch misty_ros misty_foxglove.launch.py misty_ip:=192.168.1.42

Speeds, gains and camera settings come from config/misty_params.yaml
(params_file:=... to use another file). Then in Foxglove: Open connection ->
Foxglove WebSocket -> ws://<this-host>:8765.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('misty_ros'))
    ws = share.parents[3]            # <ws>/install/misty_ros/share/misty_ros -> <ws>
    args = [
        DeclareLaunchArgument('misty_ip', default_value=EnvironmentVariable('MISTY_IP', default_value=''),
                              description="Misty's IP address (defaults to $MISTY_IP)"),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Start camera image polling (disable if face detection competes)'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'misty_params.yaml'),
                              description='Speeds / gains / camera settings'),
        DeclareLaunchArgument('port', default_value='8765', description='Foxglove WebSocket port'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='Bridge bind address; use 127.0.0.1 to allow only SSH-tunnelled access'),
        DeclareLaunchArgument('gamepad', default_value='true',
                              description='Listen for an Xbox controller from scripts/gamepad_sender.py'),
        DeclareLaunchArgument('gamepad_port', default_value='9876'),
        DeclareLaunchArgument('gamepad_local', default_value='false',
                              description='Controller plugged into THIS machine: also run the sender here'),
        DeclareLaunchArgument('gamepad_python', default_value=str(ws / 'misty_venv' / 'bin' / 'python'),
                              description='Python with pygame-ce for the local sender'),
    ]
    ip = LaunchConfiguration('misty_ip')
    params = LaunchConfiguration('params_file')

    camera = Node(package='misty_ros', executable='camera_node', name='misty_camera', output='screen',
                  condition=IfCondition(LaunchConfiguration('camera')),
                  parameters=[params, {'misty_ip': ip}])
    teleop = Node(package='misty_ros', executable='teleop_node', name='misty_teleop', output='screen',
                  parameters=[params, {'misty_ip': ip}])
    gamepad = Node(package='misty_ros', executable='gamepad_teleop', name='misty_gamepad_teleop', output='screen',
                   condition=IfCondition(LaunchConfiguration('gamepad')),
                   parameters=[params, {'port': ParameterValue(LaunchConfiguration('gamepad_port'), value_type=int)}])
    bridge = Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge', output='screen',
                  parameters=[{'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                               'address': LaunchConfiguration('address')}])
    gamepad_sender = ExecuteProcess(
        cmd=[LaunchConfiguration('gamepad_python'), str(ws / 'scripts' / 'gamepad_sender.py'),
             '127.0.0.1', '--port', LaunchConfiguration('gamepad_port')],
        name='gamepad_sender', output='screen', additional_env={'SDL_VIDEODRIVER': 'dummy'},
        condition=IfCondition(LaunchConfiguration('gamepad_local')))

    return LaunchDescription(args + [camera, teleop, bridge, gamepad, gamepad_sender])
