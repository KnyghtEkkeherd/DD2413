"""Generate a Gazebo world for Misty from the same world JSON as nav_sim.

    {"markers": {"id": [x, y, facing_deg]}, "obstacles": [[x, y, radius]],
     "walls": [xmin, ymin, xmax, ymax]}

Writes <out_dir>/misty_world.sdf plus one textured quad mesh per ArUco marker
(OBJ with explicit UVs, so the marker is never mirrored). The robot starts at
the origin facing +x, like nav_sim. Its dimensions roughly follow Misty II;
the sensor poses match the defaults in misty_params.yaml (camera 6 cm ahead of
the wheel axle, wheel base 0.11 m).

    python3 -m misty_ros.gz_world /tmp/misty_gz      # standalone
"""
import json
import math
import sys
from pathlib import Path

DEFAULTS = {
    'marker_size_m': 0.15,
    'marker_height_m': 0.30,      # marker centre above the floor
    'aruco_dictionary': 0,
    'camera_hfov_deg': 70.0,
    'camera_width': 640,
    'camera_height': 480,
    'camera_rate': 10.0,
    'camera_height_m': 0.30,
    'camera_offset_x_m': 0.06,
    'depth_enabled': True,
    'depth_hfov_deg': 60.0,
    'depth_width': 160,
    'depth_height': 120,
    'depth_rate': 5.0,
    'tof_angles_deg': [0.0, 25.0, -25.0],
    'tof_max_m': 2.0,
    'wheel_base_m': 0.11,
    'wheel_radius_m': 0.04,
    'wall_height_m': 0.6,
    'obstacle_height_m': 0.5,
    'physics_step_s': 0.004,
}


def _marker_texture(path, marker_id, dictionary, px=400, border_frac=0.25):
    """ArUco marker with a white quiet zone; returns the image side / marker side ratio."""
    import cv2
    import numpy as np
    d = cv2.aruco.getPredefinedDictionary(int(dictionary))
    img = cv2.aruco.generateImageMarker(d, int(marker_id), px)
    b = int(px * border_frac)
    img = np.pad(img, b, constant_values=255)
    cv2.imwrite(str(path), img)
    return img.shape[0] / px


def _marker_mesh(out, marker_id, side):
    """Quad facing +x, image right -> +y (as seen from in front), image top -> +z."""
    h = side / 2
    (out / f'aruco_{marker_id}.mtl').write_text(
        f'newmtl aruco_{marker_id}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nillum 1\n'
        f'map_Kd aruco_{marker_id}.png\n')
    (out / f'aruco_{marker_id}.obj').write_text(
        f'mtllib aruco_{marker_id}.mtl\n'
        f'v 0 {-h} {-h}\nv 0 {h} {-h}\nv 0 {h} {h}\nv 0 {-h} {h}\n'
        'vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 1 0 0\n'
        f'usemtl aruco_{marker_id}\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n')


def _box(name, x, y, z, sx, sy, sz, yaw=0.0, color='0.8 0.8 0.8 1'):
    return f'''
    <model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link></model>'''


def _inertial(m, sx, sy, sz):
    return (f'<inertial><mass>{m}</mass><inertia><ixx>{m * (sy * sy + sz * sz) / 12}</ixx>'
            f'<iyy>{m * (sx * sx + sz * sz) / 12}</iyy><izz>{m * (sx * sx + sy * sy) / 12}</izz>'
            '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')


def _robot(c):
    r, wb = c['wheel_radius_m'], c['wheel_base_m']
    cam_x, cam_z = c['camera_offset_x_m'], c['camera_height_m']
    tof = c['tof_angles_deg']
    tof_min, tof_max = math.radians(min(tof)), math.radians(max(tof))
    no_friction = '<surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface>'
    wheel = lambda side, y: f'''
      <link name="{side}_wheel"><pose>0 {y} {r} {-math.pi / 2} 0 0</pose>
        {_inertial(0.1, 2 * r, 2 * r, 0.03)}
        <collision name="c"><geometry><cylinder><radius>{r}</radius><length>0.03</length></cylinder></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><cylinder><radius>{r}</radius><length>0.03</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      </link>
      <joint name="{side}_wheel_joint" type="revolute"><parent>base_link</parent><child>{side}_wheel</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>'''
    depth = f'''
        <sensor name="depth" type="depth_camera"><pose>{cam_x} 0 {cam_z} 0 0 0</pose>
          <topic>/sim/depth</topic><update_rate>{c['depth_rate']}</update_rate>
          <camera><horizontal_fov>{math.radians(c['depth_hfov_deg'])}</horizontal_fov>
            <image><width>{c['depth_width']}</width><height>{c['depth_height']}</height><format>R_FLOAT32</format></image>
            <clip><near>0.1</near><far>6.0</far></clip></camera>
        </sensor>''' if c['depth_enabled'] else ''
    return f'''
    <model name="misty"><pose>0 0 0.005 0 0 0</pose>
      <link name="base_link">
        {_inertial(2.7, 0.22, 0.18, 0.3)}
        <collision name="body"><pose>0 0 0.12 0 0 0</pose><geometry><box><size>0.22 0.17 0.14</size></box></geometry></collision>
        <collision name="caster_front"><pose>0.08 0 0.02 0 0 0</pose><geometry><sphere><radius>0.02</radius></sphere></geometry>{no_friction}</collision>
        <collision name="caster_back"><pose>-0.08 0 0.02 0 0 0</pose><geometry><sphere><radius>0.02</radius></sphere></geometry>{no_friction}</collision>
        <visual name="body"><pose>0 0 0.12 0 0 0</pose><geometry><box><size>0.22 0.17 0.14</size></box></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
        <visual name="head"><pose>{cam_x - 0.075} 0 {cam_z} 0 0 0</pose><geometry><sphere><radius>0.07</radius></sphere></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
        <sensor name="rgb" type="camera"><pose>{cam_x} 0 {cam_z} 0 0 0</pose>
          <topic>/sim/camera</topic><update_rate>{c['camera_rate']}</update_rate>
          <camera><horizontal_fov>{math.radians(c['camera_hfov_deg'])}</horizontal_fov>
            <image><width>{c['camera_width']}</width><height>{c['camera_height']}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>20</far></clip>
            <noise><type>gaussian</type><mean>0</mean><stddev>0.007</stddev></noise></camera>
        </sensor>{depth}
        <sensor name="tof" type="gpu_lidar"><pose>0.115 0 0.06 0 0 0</pose>
          <topic>/sim/tof</topic><update_rate>10</update_rate>
          <lidar><scan><horizontal><samples>{len(tof)}</samples><resolution>1</resolution>
            <min_angle>{tof_min}</min_angle><max_angle>{tof_max}</max_angle></horizontal></scan>
            <range><min>0.01</min><max>{c['tof_max_m']}</max><resolution>0.001</resolution></range>
            <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise></lidar>
        </sensor>
        <sensor name="imu" type="imu"><topic>/sim/imu</topic><update_rate>50</update_rate>
          <imu><angular_velocity><z><noise type="gaussian"><mean>0</mean><stddev>0.005</stddev>
            <bias_mean>0.002</bias_mean></noise></z></angular_velocity></imu>
        </sensor>
      </link>
      {wheel('left', wb / 2)}
      {wheel('right', -wb / 2)}
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>
        <wheel_separation>{wb}</wheel_separation><wheel_radius>{r}</wheel_radius>
        <topic>/sim/cmd_vel</topic><odom_topic>/sim/odom</odom_topic>
        <max_linear_acceleration>1.5</max_linear_acceleration>
        <max_angular_acceleration>6.0</max_angular_acceleration>
      </plugin>
      <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
        <topic>/sim/joint_states</topic>
        <joint_name>left_wheel_joint</joint_name><joint_name>right_wheel_joint</joint_name>
      </plugin>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <odom_topic>/sim/ground_truth</odom_topic><odom_frame>map</odom_frame>
        <robot_base_frame>base_link</robot_base_frame><dimensions>2</dimensions><odom_publish_frequency>20</odom_publish_frequency>
      </plugin>
    </model>'''


def generate(world, out_dir, **overrides):
    c = dict(DEFAULTS)
    c.update({k: v for k, v in overrides.items() if v is not None})
    out = Path(out_dir).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    parts = []
    s, hz = c['marker_size_m'], c['marker_height_m']
    for mid, (mx, my, facing) in world['markers'].items():
        ratio = _marker_texture(out / f'aruco_{mid}.png', mid, c['aruco_dictionary'])
        _marker_mesh(out, mid, s * ratio)
        yaw = math.radians(facing)
        board = s * ratio + 0.04
        parts.append(f'''
    <model name="aruco_{mid}"><static>true</static><pose>{mx} {my} 0 0 0 {yaw}</pose>
      <link name="link">
        <visual name="marker"><pose>0.006 0 {hz} 0 0 0</pose>
          <geometry><mesh><uri>file://{out / f"aruco_{mid}.obj"}</uri></mesh></geometry></visual>
        <visual name="board"><pose>-0.005 0 {hz} 0 0 0</pose><geometry><box><size>0.015 {board} {board}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
        <visual name="post"><pose>-0.01 0 {(hz - board / 2) / 2} 0 0 0</pose>
          <geometry><box><size>0.02 0.02 {hz - board / 2}</size></box></geometry></visual>
        <collision name="c"><pose>-0.005 0 {hz / 2} 0 0 0</pose><geometry><box><size>0.03 0.04 {hz}</size></box></geometry></collision>
      </link></model>''')
    for k, (ox, oy, r) in enumerate(world.get('obstacles', [])):
        h = c['obstacle_height_m']
        parts.append(f'''
    <model name="obstacle_{k}"><static>true</static><pose>{ox} {oy} {h / 2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material><ambient>0.8 0.3 0.2 1</ambient><diffuse>0.8 0.3 0.2 1</diffuse></material></visual>
      </link></model>''')
    if world.get('walls'):
        x0, y0, x1, y1 = world['walls']
        h, t = c['wall_height_m'], 0.05
        cx, cy, lx, ly = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0 + 2 * t, y1 - y0
        parts += [_box('wall_s', cx, y0 - t / 2, h / 2, lx, t, h), _box('wall_n', cx, y1 + t / 2, h / 2, lx, t, h),
                  _box('wall_w', x0 - t / 2, cy, h / 2, t, ly, h), _box('wall_e', x1 + t / 2, cy, h / 2, t, ly, h)]
    sdf = f'''<?xml version="1.0"?>
<!-- generated by misty_ros.gz_world; edit the world JSON, not this file -->
<sdf version="1.9">
  <world name="misty_world">
    <physics name="sim" type="ignored"><max_step_size>{c['physics_step_s']}</max_step_size>
      <real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>{c.get('render_engine', 'ogre2')}</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.8 0.85 0.9 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 0.2 -0.9</direction></light>
    <model name="floor"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.55 0.5 0.45 1</ambient><diffuse>0.55 0.5 0.45 1</diffuse></material></visual>
      </link></model>{''.join(parts)}{_robot(c)}
  </world>
</sdf>
'''
    path = out / 'misty_world.sdf'
    path.write_text(sdf)
    return path


if __name__ == '__main__':
    from misty_ros.nav_sim_node import DEFAULT_WORLD
    print(generate(json.loads(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_WORLD,
                   sys.argv[1] if len(sys.argv) > 1 else '/tmp/misty_gz'))
