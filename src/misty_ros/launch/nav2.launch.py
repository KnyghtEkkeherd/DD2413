"""Autonomous marker navigation with standard ROS 2 packages (Nav2 + robot_localization).

    ros2 launch misty_ros wizard.launch.py misty_ip:=<ip> nav:=true      # real robot (uses this file)
    ros2 launch misty_ros nav2.launch.py sim:=2d                         # 2D simulator, no robot
    ros2 launch misty_ros nav2.launch.py sim:=gazebo                     # Gazebo (where it installs)

    robot_state_publisher     urdf/misty.urdf: base_link, camera, ToF and depth frames
    wheel_odom                /misty/encoders -> /misty/wheel_odom
    ekf_odom / ekf_map        robot_localization (config/localization.yaml)
    aruco_tracker             aruco_opencv on the camera stream (markers:=aruco_opencv)
    marker_localizer          marker sightings + marker_map -> pose fixes for ekf_map
    depthimage_to_laserscan   depth camera -> /scan (depth:=true)
    Nav2                      planner, controller (Regulated Pure Pursuit), behaviours,
                              BT navigator, velocity smoother, collision monitor
                              (config/nav2_params.yaml); output /nav/cmd_vel in m/s
    mission (misty_nav)       NAV START / NAV GO / ... -> NavigateToPose goals, reactions

markers:=aruco_opencv (default with a camera stream) or markers:=json (Misty's
on-board detector via sensors_node, or the 2D simulator's detections).
The real robot's sensors_node / camera / teleop are started by wizard.launch.py.
"""
import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# aruco_dictionary (OpenCV enum) -> aruco_opencv marker_dict name
DICTS = ['4X4_50', '4X4_100', '4X4_250', '4X4_1000', '5X5_50', '5X5_100', '5X5_250', '5X5_1000',
         '6X6_50', '6X6_100', '6X6_250', '6X6_1000', '7X7_50', '7X7_100', '7X7_250', '7X7_1000',
         'ARUCO_ORIGINAL']
NAV2_NODES = [  # (package, executable, remappings) as in nav2_bringup/navigation_launch.py
    ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_smoother', 'smoother_server', []),
    ('nav2_planner', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_collision_monitor', 'collision_monitor', []),
    ('nav2_bt_navigator', 'bt_navigator', []),
]


def section(cfg, name):
    return (cfg.get(name) or {}).get('ros__parameters') or {}


def setup(context):
    from misty_ros.nav_sim_node import DEFAULT_WORLD
    arg = lambda n: LaunchConfiguration(n).perform(context)
    share = Path(get_package_share_directory('misty_ros'))
    params = arg('params_file')
    cfg = yaml.safe_load(open(params)) or {}
    sensors = section(cfg, 'misty_sensors')
    sim, depth = arg('sim'), arg('depth').lower() == 'true'
    markers = arg('markers') or ('json' if sim == '2d' else 'aruco_opencv')

    # marker map: measured positions (real robot), or the simulated world's markers
    marker_map = section(cfg, '/**').get('marker_map', '{}')
    if sim in ('2d', 'gazebo'):
        world = section(cfg, 'misty_gz').get('world', DEFAULT_WORLD) if sim == 'gazebo' else \
            section(cfg, 'misty_nav_sim').get('world', DEFAULT_WORLD)
        world = json.loads(world) if isinstance(world, str) else world
        marker_map = json.dumps(world['markers'])
    shared = {'marker_map': marker_map if isinstance(marker_map, str) else json.dumps(marker_map)}

    loc = str(share / 'config' / 'localization.yaml')
    nav2 = str(share / 'config' / 'nav2_params.yaml')
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             parameters=[{'robot_description': (share / 'urdf' / 'misty.urdf').read_text()}]),
        Node(package='misty_ros', executable='wheel_odom', name='misty_wheel_odom', output='screen',
             parameters=[params]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_odom', output='screen',
             parameters=[loc], remappings=[('odometry/filtered', 'odom')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_map', output='screen',
             parameters=[loc], remappings=[('odometry/filtered', 'odometry/global')]),
        Node(package='misty_ros', executable='marker_localizer', name='misty_marker_localizer', output='screen',
             parameters=[params, shared, {'marker_source': markers}]),
        Node(package='misty_ros', executable='mission_node', name='misty_nav', output='screen',
             parameters=[params, shared]),
    ]
    if markers == 'aruco_opencv':
        actions.append(Node(
            package='aruco_opencv', executable='aruco_tracker_autostart', name='aruco_tracker', output='screen',
            parameters=[loc, {'marker_dict': DICTS[int(sensors.get('aruco_dictionary', 0))],
                              'marker_size': float(sensors.get('marker_size_m', 0.15))}]))
    if depth:
        actions.append(Node(
            package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan', parameters=[loc],
            remappings=[('depth', 'misty/depth/image'), ('depth_camera_info', 'misty/depth/camera_info'),
                        ('scan', 'scan')]))
    cm_sources = {'observation_sources': ['tof', 'scan'] if depth else ['tof']}
    for pkg, exe, remaps in NAV2_NODES:
        extra = [cm_sources] if exe == 'collision_monitor' else []
        actions.append(Node(package=pkg, executable=exe, name=exe, output='screen',
                            parameters=[nav2] + extra, remappings=remaps))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_manager_navigation', output='screen', parameters=[nav2]))

    if arg('mux').lower() == 'true':           # standalone: who drives (wizard > Nav2), see twist_mux.yaml
        actions += [
            Node(package='misty_ros', executable='teleop_relay', name='misty_teleop_relay', parameters=[params]),
            Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
                 parameters=[str(share / 'config' / 'twist_mux.yaml')],
                 remappings=[('cmd_vel_out', 'misty/cmd_vel')])]
    if sim == 'none':
        actions.append(Node(
            package='misty_ros', executable='sensors_node', name='misty_sensors', output='screen',
            parameters=[params, {'misty_ip': arg('misty_ip'), 'depth_enabled': depth,
                                 # json: Misty's on-board detector; aruco_opencv runs in the VM instead
                                 'aruco_source': 'robot' if markers == 'json' else 'none'}]))
    elif sim == '2d':
        actions.append(Node(package='misty_ros', executable='nav_sim', name='misty_nav_sim', output='screen',
                            parameters=[params]))
    elif sim == 'gazebo':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'gazebo.launch.py')),
            launch_arguments={'params_file': params, 'nav': 'false', 'foxglove': 'false',
                              'depth': arg('depth')}.items()))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('misty_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('misty_ip', default_value=EnvironmentVariable('MISTY_IP', default_value='')),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'misty_params.yaml')),
        DeclareLaunchArgument('sim', default_value='none', description='none (real robot) | 2d | gazebo'),
        DeclareLaunchArgument('markers', default_value='',
                              description='aruco_opencv | json (default: json for sim:=2d, else aruco_opencv)'),
        DeclareLaunchArgument('depth', default_value='false', description='depth camera -> /scan'),
        DeclareLaunchArgument('mux', default_value='true',
                              description='start teleop_relay + twist_mux (false when misty_foxglove.launch.py runs)'),
        OpaqueFunction(function=setup),
    ])
