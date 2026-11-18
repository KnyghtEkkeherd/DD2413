"""Misty in Gazebo: rendered camera + real ArUco detection -> SLAM -> navigation.

    ros2 launch misty_ros gazebo.launch.py                 # headless (over SSH), Foxglove on :8765
    ros2 launch misty_ros gazebo.launch.py gui:=true       # with the Gazebo window (needs a display)
    ros2 launch misty_ros gazebo.launch.py render_engine:=ogre   # if ogre2 fails on the VM's GPU

Runs: gz sim + ros_gz_bridge + gz_adapter (stands in for Misty: sensors_node
and teleop_node) + slam_node + nav_node + foxglove_bridge. The world (markers,
obstacles, walls) is `world` under misty_gz in misty_params.yaml; without it,
nav_sim's default world. Marker size/dictionary come from misty_sensors.
Drive manually with anything that publishes /cmd_vel (Foxglove DRIVE panel,
gamepad, keyboard_teleop) while nav is idle, or press NAV GO.
"""
import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

BRIDGE = [
    '/sim/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
    '/sim/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
    '/sim/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
    '/sim/tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    '/sim/camera@sensor_msgs/msg/Image[gz.msgs.Image',
    '/sim/depth@sensor_msgs/msg/Image[gz.msgs.Image',
    '/sim/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
]


def section(cfg, name):
    return (cfg.get(name) or {}).get('ros__parameters') or {}


def setup(context):
    from misty_ros.gz_world import generate
    from misty_ros.nav_sim_node import DEFAULT_WORLD

    params = LaunchConfiguration('params_file').perform(context)
    cfg = yaml.safe_load(open(params)) or {}
    gz, sensors, slam = section(cfg, 'misty_gz'), section(cfg, 'misty_sensors'), section(cfg, 'misty_slam')
    world = gz.get('world', DEFAULT_WORLD)
    world = json.loads(world) if isinstance(world, str) else world
    depth = LaunchConfiguration('depth').perform(context).lower() == 'true'
    sdf = generate(world, LaunchConfiguration('out_dir').perform(context),
                   marker_size_m=sensors.get('marker_size_m'),
                   aruco_dictionary=sensors.get('aruco_dictionary'),
                   camera_hfov_deg=sensors.get('camera_hfov_deg'),
                   camera_offset_x_m=slam.get('camera_offset_x_m'),
                   wheel_base_m=slam.get('wheel_base_m'),
                   depth_enabled=depth,
                   render_engine=LaunchConfiguration('render_engine').perform(context))

    cmd = ['gz', 'sim', '-r', str(sdf)]
    if LaunchConfiguration('gui').perform(context).lower() != 'true':
        cmd[3:3] = ['-s', '--headless-rendering']
    adapter_params = {'world': json.dumps(world)}
    for k in ('marker_size_m', 'aruco_dictionary', 'camera_hfov_deg'):
        if k in sensors:
            adapter_params[k] = sensors[k]
    teleop = section(cfg, 'misty_teleop')
    for k in ('linear_scale', 'angular_scale', 'drive_time_ms'):
        if k in teleop:
            adapter_params[k] = teleop[k]
    return [
        ExecuteProcess(cmd=cmd, name='gz_sim', output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
             arguments=BRIDGE if depth else [b for b in BRIDGE if not b.startswith('/sim/depth')]),
        Node(package='misty_ros', executable='gz_adapter', name='misty_gz_adapter', output='screen',
             parameters=[params, adapter_params]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('misty_ros'))
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'misty_params.yaml')),
        DeclareLaunchArgument('gui', default_value='false', description='show the Gazebo window'),
        DeclareLaunchArgument('render_engine', default_value='ogre2', description='ogre2 or ogre'),
        DeclareLaunchArgument('depth', default_value='true', description='simulate the depth camera'),
        DeclareLaunchArgument('nav', default_value='true', description='run slam_node + nav_node'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('out_dir', default_value=str(Path.home() / '.cache' / 'misty_gz')),
        OpaqueFunction(function=setup),
        Node(package='misty_ros', executable='slam_node', name='misty_slam', output='screen',
             condition=IfCondition(LaunchConfiguration('nav')), parameters=[params]),
        Node(package='misty_ros', executable='nav_node', name='misty_nav', output='screen',
             condition=IfCondition(LaunchConfiguration('nav')), parameters=[params]),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge', output='screen',
             condition=IfCondition(LaunchConfiguration('foxglove')),
             parameters=[{'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}]),
    ])
