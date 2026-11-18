"""Simulated Misty for testing SLAM and navigation without the robot.

Replaces sensors_node + teleop_node: listens to /misty/cmd_vel (twist_mux output,
m/s; same limits and drive_time behaviour as the teleop node) and publishes the same sensor topics
as sensors_node, with noise:

    /misty/encoders  wheel distances with slip
    /misty/imu       yaw with noise and slow drift
    /misty/tof       center / left / right time-of-flight rays (obstacles + walls)
    /misty/markers   ArUco detections in the camera frame: only markers inside the
                     camera's field of view, in range, not occluded, and seen from
                     the front (within max_view_angle_deg of the marker's normal)
    /sim/true_pose   ground truth, for checking SLAM
    /sim/world       MarkerArray of markers, obstacles and walls for Foxglove

World (parameter `world`, JSON): {"markers": {"id": [x, y, facing_deg]},
"obstacles": [[x, y, radius]], "walls": [xmin, ymin, xmax, ymax]};
the robot starts at the origin facing +x.
"""
import json
import math
import random
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, Range
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from misty_ros.markers import marker_in_map, mat_to_quat, world_markers

DEFAULT_WORLD = {
    "markers": {"1": [1.2, 0.0, 180], "2": [3.5, 1.5, 180], "3": [1.5, -2.0, 90]},
    "obstacles": [[2.2, 0.85, 0.25]],
    "walls": [-1.0, -3.0, 5.0, 3.0],
}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return math.inf
    t = -b - math.sqrt(disc)
    return t if t > 0 else math.inf


def ray_walls(ox, oy, dx, dy, w):
    xmin, ymin, xmax, ymax = w
    best = math.inf
    for wall, d, o in ((xmin, dx, ox), (xmax, dx, ox)):
        if abs(d) > 1e-9:
            t = (wall - o) / d
            if t > 0:
                best = min(best, t)
    for wall, d, o in ((ymin, dy, oy), (ymax, dy, oy)):
        if abs(d) > 1e-9:
            t = (wall - o) / d
            if t > 0:
                best = min(best, t)
    return best


class NavSim(Node):
    def __init__(self):
        super().__init__('misty_nav_sim')
        P = self.declare_parameter
        P('world', json.dumps(DEFAULT_WORLD))
        P('seed', 7)
        P('linear_scale', 30.0)          # like teleop_node: Twist 1.0 -> 30 %
        P('angular_scale', 70.0)
        P('drive_time_ms', 600)
        P('max_linear_mps', 0.6)         # Misty speed at 100 %
        P('max_angular_dps', 120.0)      # Misty turn rate at 100 %
        P('wheel_base_m', 0.11)
        P('slip', 0.05)                  # wheel distance noise (fraction)
        P('imu_noise_deg', 0.5)
        P('imu_drift_dps', 0.2)
        P('camera_hfov_deg', 70.0)
        P('camera_offset_x_m', 0.06)
        P('marker_max_range_m', 4.0)
        P('max_view_angle_deg', 70.0)
        P('marker_noise_m', 0.03)
        P('marker_rate', 4.0)
        P('marker_latency_s', 0.15)
        P('tof_max_m', 1.5)
        P('tof_angles_deg', [0.0, 25.0, -25.0])     # center, left, right
        P('robot_radius_m', 0.12)
        P('rate', 50.0)
        P('camera_height_m', 0.30)       # for marker orientation (as aruco_opencv reports it)
        P('marker_height_m', 0.30)
        P('marker_yaw_noise_deg', 2.0)

        self.world = json.loads(self.get_parameter('world').value)
        random.seed(int(self.get_parameter('seed').value))
        self.x = self.y = self.th = 0.0
        self.enc = [0.0, 0.0]
        self.imu_bias = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_until = 0.0
        self.collisions = 0
        self.in_collision = False
        self.pending_markers = []

        self.pub_enc = self.create_publisher(JointState, 'misty/encoders', 20)
        self.pub_imu = self.create_publisher(Imu, 'misty/imu', 20)
        self.pub_tof = self.create_publisher(Range, 'misty/tof', 20)
        self.pub_markers = self.create_publisher(String, 'misty/markers', 10)
        self.pub_true = self.create_publisher(PoseStamped, 'sim/true_pose', 10)
        self.pub_world = self.create_publisher(MarkerArray, 'sim/world', 1)
        self.pub_stats = self.create_publisher(String, 'sim/stats', 1)
        self.create_subscription(TwistStamped, 'misty/cmd_vel', self.on_mux_cmd, 10)   # twist_mux output
        self.last_w = 0.0

        self.dt = 1.0 / float(self.get_parameter('rate').value)
        self.last = time.monotonic()
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.publish_sensors)
        self.create_timer(1.0 / float(self.get_parameter('marker_rate').value), self.detect_markers)
        self.create_timer(1.0, self.publish_world)
        self.get_logger().info(f'Simulated Misty: {len(self.world["markers"])} markers, '
                               f'{len(self.world.get("obstacles", []))} obstacles')

    def p(self, name):
        return self.get_parameter(name).value

    def on_mux_cmd(self, msg):
        """twist_mux output in m/s, rad/s (like teleop_node: limited to Misty at 100 %)."""
        v = max(-1.0, min(1.0, msg.twist.linear.x / float(self.p('max_linear_mps'))))
        w = max(-1.0, min(1.0, math.degrees(msg.twist.angular.z) / float(self.p('max_angular_dps'))))
        self.set_cmd(v * 100.0, w * 100.0)

    def set_cmd(self, lin, ang):
        if lin == 0.0 and ang == 0.0:
            self.cmd, self.cmd_until = (0.0, 0.0), 0.0
        else:
            self.cmd = (lin / 100.0 * float(self.p('max_linear_mps')),
                        math.radians(ang / 100.0 * float(self.p('max_angular_dps'))))
            self.cmd_until = time.monotonic() + int(self.p('drive_time_ms')) / 1000.0

    def blocked(self, x, y):
        rr = float(self.p('robot_radius_m'))
        for ox, oy, r in self.world.get('obstacles', []):
            if math.hypot(x - ox, y - oy) < r + rr:
                return True
        xmin, ymin, xmax, ymax = self.world['walls']
        return not (xmin + rr < x < xmax - rr and ymin + rr < y < ymax - rr)

    def step(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        v, w = self.cmd if now < self.cmd_until else (0.0, 0.0)
        ds, dth = v * dt, w * dt
        nx = self.x + ds * math.cos(self.th + dth / 2)
        ny = self.y + ds * math.sin(self.th + dth / 2)
        if ds != 0.0 and self.blocked(nx, ny):
            if not self.in_collision:
                self.collisions += 1
                self.get_logger().warning(f'COLLISION at ({self.x:.2f}, {self.y:.2f})')
            self.in_collision = True
            ds = 0.0                                   # wheels slip in place
        else:
            self.in_collision = False
            self.x, self.y = nx, ny
        self.th = wrap(self.th + dth)
        self.last_w = dth / dt if dt > 0 else 0.0
        wb, slip = float(self.p('wheel_base_m')), float(self.p('slip'))
        dl, dr = ds - dth * wb / 2, ds + dth * wb / 2
        self.enc[0] += dl * (1 + random.gauss(0, slip))
        self.enc[1] += dr * (1 + random.gauss(0, slip))
        self.imu_bias += math.radians(float(self.p('imu_drift_dps'))) * dt * random.choice((-1, 1))

    def publish_sensors(self):
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name, js.position = ['left_wheel', 'right_wheel'], list(self.enc)
        self.pub_enc.publish(js)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'base_link'
        yaw = self.th + self.imu_bias + math.radians(random.gauss(0, float(self.p('imu_noise_deg'))))
        imu.orientation.z, imu.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        var = math.radians(float(self.p('imu_noise_deg'))) ** 2 + 1e-6
        imu.orientation_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
        imu.angular_velocity.z = self.last_w + random.gauss(0, 0.01)
        imu.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        imu.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(imu)
        for name, a in zip(('center', 'left', 'right'), self.p('tof_angles_deg')):
            ang = self.th + math.radians(a)
            dx, dy = math.cos(ang), math.sin(ang)
            d = ray_walls(self.x, self.y, dx, dy, self.world['walls'])
            for ox, oy, r in self.world.get('obstacles', []):
                d = min(d, ray_circle(self.x, self.y, dx, dy, ox, oy, r))
            d -= float(self.p('robot_radius_m'))           # sensors sit on the robot's front
            rg = Range()
            rg.header.stamp, rg.header.frame_id = stamp, f'tof_{name}'
            rg.radiation_type, rg.field_of_view = Range.INFRARED, math.radians(25.0)
            rg.min_range, rg.max_range = 0.01, float(self.p('tof_max_m'))
            rg.range = max(0.01, d + random.gauss(0, 0.01)) if d < rg.max_range else rg.max_range
            self.pub_tof.publish(rg)
        t = PoseStamped()
        t.header.stamp, t.header.frame_id = stamp, 'map'
        t.pose.position.x, t.pose.position.y = self.x, self.y
        t.pose.orientation.z, t.pose.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.pub_true.publish(t)
        self.pub_stats.publish(String(data=json.dumps({'collisions': self.collisions})))

    def detect_markers(self):
        now = time.monotonic()
        # deliver detections captured `marker_latency_s` ago (camera + detector latency)
        ready = [d for d in self.pending_markers if d[0] <= now]
        self.pending_markers = [d for d in self.pending_markers if d[0] > now]
        for _, dets in ready:
            self.pub_markers.publish(String(data=json.dumps({'stamp': time.time(), 'source': 'sim',
                                                             'detections': dets})))
        cam = float(self.p('camera_offset_x_m'))
        cx, cy = self.x + cam * math.cos(self.th), self.y + cam * math.sin(self.th)
        dets = []
        for mid, (mx, my, facing) in self.world['markers'].items():
            dx, dy = mx - cx, my - cy
            rng = math.hypot(dx, dy)
            bearing = wrap(math.atan2(dy, dx) - self.th)
            if rng > float(self.p('marker_max_range_m')) or abs(bearing) > math.radians(self.p('camera_hfov_deg')) / 2:
                continue
            normal = math.radians(facing)
            view = wrap(math.atan2(-dy, -dx) - normal)          # angle between marker normal and camera
            if abs(view) > math.radians(float(self.p('max_view_angle_deg'))):
                continue
            if any(ray_circle(cx, cy, dx / rng, dy / rng, ox, oy, r) < rng
                   for ox, oy, r in self.world.get('obstacles', [])):
                continue                                         # occluded
            n = float(self.p('marker_noise_m')) * (0.5 + rng / 2)
            xr = rng * math.cos(bearing) + random.gauss(0, n)
            yr = rng * math.sin(bearing) + random.gauss(0, n)
            det = {'id': int(mid), 'x': -yr, 'y': float(self.p('camera_height_m')) - float(self.p('marker_height_m')),
                   'z': xr}                                      # camera optical frame
            # marker orientation in the camera frame, as aruco_opencv reports it
            th = self.th + math.radians(random.gauss(0, float(self.p('marker_yaw_noise_deg'))))
            cam_R = marker_in_map(0, 0, math.degrees(th) + 180.0, 0)[:3, :3]   # optical: z fwd, x right, y down
            cam_R = cam_R @ np.diag([1.0, -1.0, -1.0])
            qx, qy, qz, qw = mat_to_quat(cam_R.T @ marker_in_map(mx, my, facing, 0)[:3, :3])
            det.update(qx=qx, qy=qy, qz=qz, qw=qw)
            dets.append(det)
        self.pending_markers.append((now + float(self.p('marker_latency_s')), dets))

    def publish_world(self):
        self.pub_world.publish(world_markers(self.world, self.get_clock().now().to_msg()))

def main():
    rclpy.init()
    node = None
    try:
        node = NavSim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
