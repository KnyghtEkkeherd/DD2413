"""Simulated Misty for testing SLAM and navigation without the robot.

Replaces sensors_node + teleop_node: listens to /cmd_vel (same scaling and
drive_time behaviour as the teleop node) and publishes the same sensor topics
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

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, Range
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

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


def world_markers(world, stamp):
    """Markers, obstacles and walls as a MarkerArray for Foxglove."""
    arr = MarkerArray()
    k = 0
    for mid, (mx, my, facing) in world['markers'].items():
        m = Marker()
        m.header.stamp, m.header.frame_id = stamp, 'map'
        m.ns, m.id, m.type = 'aruco', k, Marker.CUBE
        m.pose.position.x, m.pose.position.y, m.pose.position.z = mx, my, 0.2
        m.pose.orientation.z, m.pose.orientation.w = math.sin(math.radians(facing) / 2), math.cos(math.radians(facing) / 2)
        m.scale.x, m.scale.y, m.scale.z = 0.02, 0.15, 0.15
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        arr.markers.append(m)
        k += 1
    for ox, oy, r in world.get('obstacles', []):
        m = Marker()
        m.header.stamp, m.header.frame_id = stamp, 'map'
        m.ns, m.id, m.type = 'obstacles', k, Marker.CYLINDER
        m.pose.position.x, m.pose.position.y, m.pose.position.z = ox, oy, 0.2
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 2 * r
        m.scale.z = 0.4
        m.color.r, m.color.a = 0.9, 0.8
        arr.markers.append(m)
        k += 1
    return arr


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
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)

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

    def on_cmd(self, msg):
        lin = max(-100.0, min(100.0, msg.linear.x * float(self.p('linear_scale'))))
        ang = max(-100.0, min(100.0, msg.angular.z * float(self.p('angular_scale'))))
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
        self.pub_imu.publish(imu)
        for name, a in zip(('center', 'left', 'right'), self.p('tof_angles_deg')):
            ang = self.th + math.radians(a)
            dx, dy = math.cos(ang), math.sin(ang)
            d = ray_walls(self.x, self.y, dx, dy, self.world['walls'])
            for ox, oy, r in self.world.get('obstacles', []):
                d = min(d, ray_circle(self.x, self.y, dx, dy, ox, oy, r))
            d -= float(self.p('robot_radius_m'))           # sensors sit on the robot's front
            if d <= float(self.p('tof_max_m')):
                rg = Range()
                rg.header.stamp, rg.header.frame_id = stamp, f'tof_{name}'
                rg.range = max(0.01, d + random.gauss(0, 0.01))
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
            dets.append({'id': int(mid), 'x': -yr, 'y': 0.0, 'z': xr})   # camera frame
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
