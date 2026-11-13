"""Start Misty camera + teleop nodes and foxglove_bridge.

    ros2 launch misty_ros misty_foxglove.launch.py misty_ip:=192.168.1.42

Then in Foxglove: Open connection -> Foxglove WebSocket -> ws://<this-host>:8765
(or tunnel it: ssh -L 8765:localhost:8765 <this-host>, then ws://localhost:8765).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('misty_ip', default_value=EnvironmentVariable('MISTY_IP', default_value=''),
                              description="Misty's IP address (defaults to $MISTY_IP)"),
        DeclareLaunchArgument('port', default_value='8765', description='Foxglove WebSocket port'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='Bridge bind address; use 127.0.0.1 to allow only SSH-tunnelled access'),
        DeclareLaunchArgument('fps', default_value='2.0', description='Camera polling rate'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('linear_scale', default_value='30.0', description='% speed per Twist unit'),
        DeclareLaunchArgument('angular_scale', default_value='40.0', description='% turn rate per Twist unit'),
        DeclareLaunchArgument('head_step_deg', default_value='10.0', description='Head move (deg) per head teleop message'),
        DeclareLaunchArgument('head_velocity', default_value='100.0', description='Head speed, % of max'),
    ]
    ip = LaunchConfiguration('misty_ip')

    def num(name, t):
        return ParameterValue(LaunchConfiguration(name), value_type=t)

    camera = Node(
        package='misty_ros', executable='camera_node', name='misty_camera', output='screen',
        parameters=[{'misty_ip': ip, 'fps': num('fps', float),
                     'width': num('width', int), 'height': num('height', int)}])
    teleop = Node(
        package='misty_ros', executable='teleop_node', name='misty_teleop', output='screen',
        parameters=[{'misty_ip': ip, 'linear_scale': num('linear_scale', float),
                     'angular_scale': num('angular_scale', float),
                     'head_step_deg': num('head_step_deg', float),
                     'head_velocity': num('head_velocity', float)}])
    bridge = Node(
        package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge', output='screen',
        parameters=[{'port': num('port', int), 'address': LaunchConfiguration('address')}])

    return LaunchDescription(args + [camera, teleop, bridge])
