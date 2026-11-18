"""EKF-SLAM for Misty with ArUco markers as landmarks.

State: robot pose (x, y, theta) in the "map" frame (origin = where Misty
started, x forward, y left) plus the 2D position of every marker seen so far.

  Prediction  wheel dead reckoning from /misty/encoders (distance travelled);
              heading change from the IMU yaw (/misty/imu) when available,
              otherwise from the wheel difference.
  Update      every marker in /misty/markers is a range-bearing measurement.
              Unknown markers are added to the map; known ones correct both the
              robot pose and the marker position. Detections that disagree
              too much with the map (Mahalanobis gate) are ignored.
  Prior map   optional: known_markers (JSON {"id": [x, y]}) pins markers to
              fixed positions, turning SLAM into localisation for them.

Assumes the head (camera) is centred (yaw 0) while navigating.

Publishes /slam/pose (PoseWithCovarianceStamped), /slam/odom_pose (dead
reckoning only, for comparison), /slam/path, /slam/landmarks (JSON) and
/slam/landmark_markers (MarkerArray for Foxglove's 3D panel), all in "map".
Reset with /wizard/event NAV_RESET or SLAM_RESET.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class EkfSlam:
    """Plain EKF-SLAM with range-bearing landmarks (testable without ROS)."""

    def __init__(self, gate=9.21):
        self.mu = np.zeros(3)
        self.P = np.diag([1e-6, 1e-6, 1e-6])
        self.index = {}                    # marker id -> state index of its x
        self.gate = gate

    def predict(self, ds, dth, var_ds, var_dth):
        x, y, th = self.mu[:3]
        mid = th + dth / 2
        self.mu[0] = x + ds * math.cos(mid)
        self.mu[1] = y + ds * math.sin(mid)
        self.mu[2] = wrap(th + dth)
        n = len(self.mu)
        G = np.eye(n)
        G[0, 2] = -ds * math.sin(mid)
        G[1, 2] = ds * math.cos(mid)
        V = np.zeros((n, 2))
        V[0, 0], V[0, 1] = math.cos(mid), -ds * math.sin(mid) / 2
        V[1, 0], V[1, 1] = math.sin(mid), ds * math.cos(mid) / 2
        V[2, 1] = 1.0
        self.P = G @ self.P @ G.T + V @ np.diag([var_ds, var_dth]) @ V.T

    def add_landmark(self, mid, lx, ly, cov):
        i = len(self.mu)
        self.index[mid] = i
        self.mu = np.append(self.mu, [lx, ly])
        n = len(self.mu)
        P = np.zeros((n, n))
        P[:i, :i] = self.P
        P[i:, i:] = cov
        self.P = P
        return i

    def add_observed_landmark(self, mid, r, b, R):
        """Initialise a new landmark from a range-bearing observation."""
        x, y, th = self.mu[:3]
        lx, ly = x + r * math.cos(th + b), y + r * math.sin(th + b)
        i = len(self.mu)
        self.index[mid] = i
        self.mu = np.append(self.mu, [lx, ly])
        n = len(self.mu)
        # Jacobians of the landmark w.r.t. robot pose and the measurement
        Gr = np.array([[1, 0, -r * math.sin(th + b)], [0, 1, r * math.cos(th + b)]])
        Gz = np.array([[math.cos(th + b), -r * math.sin(th + b)], [math.sin(th + b), r * math.cos(th + b)]])
        P = np.zeros((n, n))
        P[:i, :i] = self.P
        P[i:, :3] = Gr @ self.P[:3, :3]
        P[i:, 3:i] = Gr @ self.P[:3, 3:i]
        P[:i, i:] = P[i:, :i].T
        P[i:, i:] = Gr @ self.P[:3, :3] @ Gr.T + Gz @ R @ Gz.T
        self.P = P

    def update(self, mid, r, b, R):
        """Range-bearing update. Returns 'new', 'ok' or 'rejected'."""
        if mid not in self.index:
            self.add_observed_landmark(mid, r, b, R)
            return 'new'
        i = self.index[mid]
        x, y, th = self.mu[:3]
        dx, dy = self.mu[i] - x, self.mu[i + 1] - y
        q = dx * dx + dy * dy
        sq = math.sqrt(q)
        zhat = np.array([sq, wrap(math.atan2(dy, dx) - th)])
        n = len(self.mu)
        H = np.zeros((2, n))
        H[:, :3] = [[-dx / sq, -dy / sq, 0], [dy / q, -dx / q, -1]]
        H[:, i:i + 2] = [[dx / sq, dy / sq], [-dy / q, dx / q]]
        S = H @ self.P @ H.T + R
        nu = np.array([r - zhat[0], wrap(b - zhat[1])])
        if float(nu @ np.linalg.solve(S, nu)) > self.gate:
            return 'rejected'
        K = self.P @ H.T @ np.linalg.inv(S)
        self.mu = self.mu + K @ nu
        self.mu[2] = wrap(self.mu[2])
        I_KH = np.eye(n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T      # Joseph form: stays symmetric PSD
        return 'ok'

    def landmarks(self):
        return {mid: (self.mu[i], self.mu[i + 1], math.sqrt(max(self.P[i, i], self.P[i + 1, i + 1])))
                for mid, i in self.index.items()}


class SlamNode(Node):
    def __init__(self):
        super().__init__('misty_slam')
        self.declare_parameter('wheel_base_m', 0.11)       # distance between the wheels / tracks
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('imu_yaw_sign', 1.0)        # -1 if Misty's yaw grows clockwise
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('odom_noise_dist', 0.05)    # std dev, fraction of distance travelled
        self.declare_parameter('odom_noise_turn', 0.05)    # std dev, fraction of the turn
        self.declare_parameter('odom_noise_turn_per_m', 0.05)  # rad of heading noise per metre driven
        self.declare_parameter('obs_noise_range_frac', 0.06)   # range std dev = a + frac * range
        self.declare_parameter('obs_noise_range_min', 0.04)
        self.declare_parameter('obs_noise_bearing_deg', 3.0)
        self.declare_parameter('max_obs_range_m', 4.0)
        self.declare_parameter('camera_offset_x_m', 0.06)  # camera ahead of the wheel axle
        self.declare_parameter('camera_offset_y_m', 0.0)
        self.declare_parameter('known_markers', '{}')      # JSON {"id": [x, y]} in map frame
        self.declare_parameter('known_marker_std_m', 0.02)
        self.declare_parameter('gate', 9.21)               # chi2, 2 dof, 99 %
        self.declare_parameter('path_every_m', 0.05)

        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, 'slam/pose', 10)
        self.pub_odom = self.create_publisher(PoseStamped, 'slam/odom_pose', 10)
        self.pub_path = self.create_publisher(Path, 'slam/path', 2)
        self.pub_lm = self.create_publisher(String, 'slam/landmarks', 5)
        self.pub_lm_viz = self.create_publisher(MarkerArray, 'slam/landmark_markers', 2)
        self.create_subscription(JointState, 'misty/encoders', self.on_encoders, 50)
        self.create_subscription(Imu, 'misty/imu', self.on_imu, 50)
        self.create_subscription(String, 'misty/markers', self.on_markers, 20)
        self.create_subscription(String, 'wizard/event', self.on_event, 10)
        self.create_timer(0.5, self.publish_landmarks)
        self.reset()
        self.get_logger().info('EKF-SLAM ready (landmarks = ArUco markers)')

    def p(self, name):
        return self.get_parameter(name).value

    def reset(self):
        self.ekf = EkfSlam(gate=float(self.p('gate')))
        self.last_enc = None
        self.imu_yaw = None
        self.imu_time = None
        self.last_imu_yaw_used = None
        self.odom = [0.0, 0.0, 0.0]
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.stats = {'ok': 0, 'new': 0, 'rejected': 0}
        std = float(self.p('known_marker_std_m'))
        try:
            known = json.loads(self.p('known_markers') or '{}')
        except ValueError as e:
            self.get_logger().error(f'known_markers is not valid JSON: {e}')
            known = {}
        for mid, (x, y) in known.items():
            self.ekf.add_landmark(int(mid), float(x), float(y), np.eye(2) * std * std)
        if known:
            self.get_logger().info(f'Prior map: markers {sorted(int(k) for k in known)}')

    def on_event(self, msg: String):
        if msg.data.strip().upper() in ('NAV_RESET', 'SLAM_RESET'):
            self.reset()
            self.get_logger().info('SLAM reset')

    def on_imu(self, msg: Imu):
        self.imu_yaw = float(self.p('imu_yaw_sign')) * yaw_from_quat(msg.orientation)
        self.imu_time = self.get_clock().now()

    def imu_fresh(self):
        return (self.p('use_imu_yaw') and self.imu_time is not None and
                (self.get_clock().now() - self.imu_time).nanoseconds / 1e9 < self.p('imu_timeout_s'))

    def on_encoders(self, msg: JointState):
        if len(msg.position) < 2:
            return
        left, right = float(msg.position[0]), float(msg.position[1])
        if self.last_enc is None:
            self.last_enc = (left, right)
            self.last_imu_yaw_used = self.imu_yaw
            return
        dl, dr = left - self.last_enc[0], right - self.last_enc[1]
        self.last_enc = (left, right)
        if abs(dl) > 1.0 or abs(dr) > 1.0:        # encoder reset / glitch: skip
            self.get_logger().warning(f'encoder jump ignored ({dl:.2f}, {dr:.2f} m)')
            return
        ds = (dl + dr) / 2.0
        if self.imu_fresh() and self.last_imu_yaw_used is not None:
            dth = wrap(self.imu_yaw - self.last_imu_yaw_used)
        else:
            dth = (dr - dl) / float(self.p('wheel_base_m'))
        if self.imu_yaw is not None:
            self.last_imu_yaw_used = self.imu_yaw
        if ds == 0.0 and dth == 0.0:
            return
        var_ds = (float(self.p('odom_noise_dist')) * ds) ** 2 + 1e-8
        var_dth = ((float(self.p('odom_noise_turn')) * dth) ** 2 +
                   (float(self.p('odom_noise_turn_per_m')) * ds) ** 2 + 1e-8)
        self.ekf.predict(ds, dth, var_ds, var_dth)
        # dead-reckoning-only pose for comparison
        o = self.odom
        o[0] += ds * math.cos(o[2] + dth / 2)
        o[1] += ds * math.sin(o[2] + dth / 2)
        o[2] = wrap(o[2] + dth)
        self.publish_pose()

    def on_markers(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        changed = False
        for d in data.get('detections', []):
            # camera frame (x right, y down, z forward) -> robot frame (x forward, y left)
            xr = float(self.p('camera_offset_x_m')) + float(d['z'])
            yr = float(self.p('camera_offset_y_m')) - float(d['x'])
            r = math.hypot(xr, yr)
            if r <= 0.05 or r > float(self.p('max_obs_range_m')) or float(d['z']) <= 0:
                continue
            b = math.atan2(yr, xr)
            sr = float(self.p('obs_noise_range_min')) + float(self.p('obs_noise_range_frac')) * r
            sb = math.radians(float(self.p('obs_noise_bearing_deg')))
            result = self.ekf.update(int(d['id']), r, b, np.diag([sr * sr, sb * sb]))
            self.stats[result] += 1
            if result == 'new':
                self.get_logger().info(f'New landmark: marker {d["id"]}')
            changed = changed or result != 'rejected'
        if changed:
            self.publish_pose()

    def publish_pose(self):
        now = self.get_clock().now().to_msg()
        x, y, th = self.ekf.mu[:3]
        msg = PoseWithCovarianceStamped()
        msg.header.stamp, msg.header.frame_id = now, 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        cov = [0.0] * 36
        P = self.ekf.P
        cov[0], cov[1], cov[6], cov[7] = P[0, 0], P[0, 1], P[1, 0], P[1, 1]
        cov[5], cov[30], cov[35] = P[0, 2], P[2, 0], P[2, 2]
        cov[11], cov[31] = P[1, 2], P[2, 1]
        msg.pose.covariance = [float(c) for c in cov]
        self.pub_pose.publish(msg)

        o = PoseStamped()
        o.header.stamp, o.header.frame_id = now, 'map'
        o.pose.position.x, o.pose.position.y = self.odom[0], self.odom[1]
        o.pose.orientation.z, o.pose.orientation.w = math.sin(self.odom[2] / 2), math.cos(self.odom[2] / 2)
        self.pub_odom.publish(o)

        last = self.path.poses[-1].pose.position if self.path.poses else None
        if last is None or math.hypot(last.x - x, last.y - y) >= float(self.p('path_every_m')):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            self.path.poses.append(ps)
            self.path.poses = self.path.poses[-2000:]
            self.path.header.stamp = now
            self.pub_path.publish(self.path)

    def publish_landmarks(self):
        lms = self.ekf.landmarks()
        self.pub_lm.publish(String(data=json.dumps(
            {'landmarks': {str(k): [round(v[0], 3), round(v[1], 3), round(v[2], 3)] for k, v in lms.items()},
             'updates': self.stats})))
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for k, (x, y, s) in lms.items():
            m = Marker()
            m.header.stamp, m.header.frame_id = now, 'map'
            m.ns, m.id, m.type, m.action = 'landmarks', int(k), Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.15
            m.pose.orientation.w = 1.0
            d = max(0.08, 2 * float(s))                     # diameter shows the uncertainty
            m.scale.x, m.scale.y, m.scale.z = d, d, 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.6, 1.0, 0.8
            t = Marker()
            t.header = m.header
            t.ns, t.id, t.type, t.action = 'labels', int(k), Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(x), float(y), 0.45
            t.pose.orientation.w = 1.0
            t.scale.z = 0.15
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f'marker {k}'
            arr.markers += [m, t]
        self.pub_lm_viz.publish(arr)


def main():
    rclpy.init()
    node = None
    try:
        node = SlamNode()
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
