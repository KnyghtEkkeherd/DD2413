"""ArUco markers -> Misty's pose in the map (for robot_localization), learning unknown markers.

Marker positions don't have to be measured. The localizer builds the marker map
while Misty drives, as long as each next marker can be seen from near the
previous one (turning on the spot if needed):

  * the first marker seen fixes the map: it's placed using Misty's current pose
    estimate, so nothing jumps (map origin = where the map EKF started);
  * a new marker is placed from Misty's pose estimate (map EKF, /odometry/global) plus
    the detection. Every further sighting refines it, weighted by how certain it is
    (robot pose covariance + distance), so the close sightings while driving up to
    it dominate. It is LEARNING: usable as a navigation goal, NOT used to localize
    (that would just confirm the estimate it came from);
  * after min_sightings sightings, at least one closer than fix_range_m, it becomes
    FIXED: frozen, and from then on every sighting of it corrects Misty's pose.

Markers listed in marker_map ({"id": [x, y, facing_deg(, height)]}, map frame) are
FIXED from the start. Learned markers are saved to map_file and loaded at the next
start (same room); MAP_RESET on /wizard/event forgets them.

    T_map_base = T_map_marker * inverse(T_camera_marker) * inverse(T_base_camera)

T_base_camera comes from TF (urdf/misty.urdf). Detections:
    marker_source: aruco_opencv  aruco_opencv_msgs/ArucoDetection on /aruco_detections
    marker_source: json          /misty/markers (Misty's on-board detector, simulators):
                                 with qx..qw a full pose, else position only
A marker's heading is trusted up to heading_max_range_m and only if the solved
marker faces the camera (OpenCV's IPPE sometimes returns the flipped solution);
otherwise a sighting gives position only.

Publishes (PoseWithCovarianceStamped, frame map; uncertainty grows with distance):
    /localization/marker_pose     x, y, yaw   -> ekf_map pose0
    /localization/marker_pose_xy  x, y        -> ekf_map pose1
    /localization/markers         String JSON (latched): {"id": {"x", "y", "facing", "state", "n"}}
                                  -> mission_node (navigation goals)
    /localization/marker_map      MarkerArray for Foxglove (green = fixed, orange = learning)
"""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from misty_ros.markers import marker_in_map, quat_to_mat, transform


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Learned:
    """Weighted average of a marker's position / facing / height over its sightings."""

    def __init__(self):
        self.w = self.sx = self.sy = self.sz = 0.0
        self.wf = self.fc = self.fs = 0.0
        self.n, self.best_range = 0, math.inf

    def add(self, x, y, z, w, facing, wf, rng):
        self.w += w
        self.sx, self.sy, self.sz = self.sx + w * x, self.sy + w * y, self.sz + w * z
        if wf > 0:
            self.wf += wf
            self.fc, self.fs = self.fc + wf * math.cos(facing), self.fs + wf * math.sin(facing)
        self.n += 1
        self.best_range = min(self.best_range, rng)

    def pose(self):
        return self.sx / self.w, self.sy / self.w, self.sz / self.w, math.atan2(self.fs, self.fc)


class MarkerLocalizer(Node):
    def __init__(self):
        super().__init__('misty_marker_localizer')
        P = self.declare_parameter
        P('marker_map', '{}')               # measured markers (optional): FIXED from the start
        P('map_file', str(Path.home() / '.ros' / 'misty_marker_map.json'))   # '' = don't save / load
        P('marker_height_m', 0.30)          # default centre height when the map has none
        P('marker_source', 'aruco_opencv')  # aruco_opencv | json
        P('map_frame', 'map')
        P('base_frame', 'base_link')
        P('camera_frame', 'camera_optical_frame')   # for json detections (no header)
        P('max_range_m', 3.5)               # ignore markers further than this
        P('heading_max_range_m', 2.0)       # further: single-marker heading is unreliable -> position only
        P('min_facing', 0.2)                # marker normal must point at the camera (cos)
        P('xy_std', [0.03, 0.04])           # sighting noise, metres: a + b * distance
        P('yaw_std_deg', [3.0, 4.0])        # degrees: a + b * distance
        P('min_sightings', 5)               # LEARNING -> FIXED after this many sightings ...
        P('fix_range_m', 1.5)               # ... at least one closer than this
        P('head_tolerance_deg', 5.0)        # camera is in the head: ignore sightings while it's turned

        self.fixed = {}                     # id -> 4x4 T_map_marker (used for localization)
        self.learning = {}                  # id -> Learned
        self.robot_cov = None               # (var_x, var_y, var_yaw) of the map EKF
        self.load()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, 'localization/marker_pose', 10)
        self.pub_xy = self.create_publisher(PoseWithCovarianceStamped, 'localization/marker_pose_xy', 10)
        self.pub_markers = self.create_publisher(String, 'localization/markers', latched)
        self.pub_viz = self.create_publisher(MarkerArray, 'localization/marker_map', 1)
        self.create_subscription(Odometry, 'odometry/global', self.on_global, 10)
        self.create_subscription(String, 'wizard/event', self.on_event, 10)
        self.create_subscription(Vector3, 'misty/head_state', self.on_head, 10)
        self.head = (0.0, 0.0, 0.0)          # (time, pitch, yaw) measured, degrees
        source = self.get_parameter('marker_source').value
        if source == 'aruco_opencv':
            from aruco_opencv_msgs.msg import ArucoDetection
            self.create_subscription(ArucoDetection, 'aruco_detections', self.on_aruco, 10)
        else:
            self.create_subscription(String, 'misty/markers', self.on_json, 10)
        self.create_timer(1.0, self.publish_viz)
        self.publish_markers()
        self.get_logger().info(f'Marker localizer ({source}): {len(self.fixed)} fixed markers '
                               f'{sorted(self.fixed)}; unknown markers are learned while driving')

    def p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------- map storage
    def load(self):
        entries = {}
        f = self.p('map_file')
        if f and Path(f).is_file():
            try:
                entries.update(json.loads(Path(f).read_text()))
                self.get_logger().info(f'Loaded learned markers {sorted(entries)} from {f} (MAP_RESET forgets them)')
            except ValueError as e:
                self.get_logger().error(f'{f}: {e}')
        entries.update(json.loads(self.p('marker_map') or '{}'))        # measured positions win
        for mid, v in entries.items():
            h = v[3] if len(v) > 3 else float(self.p('marker_height_m'))
            self.fixed[int(mid)] = marker_in_map(v[0], v[1], v[2], h)

    def save(self):
        f = self.p('map_file')
        if not f:
            return
        measured = {int(k) for k in json.loads(self.p('marker_map') or '{}')}
        out = {str(mid): [round(float(T[0, 3]), 3), round(float(T[1, 3]), 3),
                          round(math.degrees(math.atan2(T[1, 2], T[0, 2])), 1), round(float(T[2, 3]), 3)]
               for mid, T in self.fixed.items() if mid not in measured}
        Path(f).parent.mkdir(parents=True, exist_ok=True)
        Path(f).write_text(json.dumps(out, indent=1) + '\n')

    def on_event(self, msg):
        if msg.data.strip().upper() == 'MAP_RESET':
            measured = {int(k) for k in json.loads(self.p('marker_map') or '{}')}
            self.fixed = {k: v for k, v in self.fixed.items() if k in measured}
            self.learning = {}
            self.save()
            self.publish_markers()
            self.get_logger().info('MAP_RESET: learned markers forgotten')

    def on_global(self, msg):
        c = msg.pose.covariance
        self.robot_cov = (c[0], c[7], c[35])

    # ---------------------------------------------------------------- inputs
    def base_from_camera(self, frame):
        try:
            t = self.tf.lookup_transform(self.p('base_frame'), frame, Time())   # static mount
        except Exception as e:
            self.get_logger().warning(f'no TF {self.p("base_frame")} -> {frame}: {e}', throttle_duration_sec=5.0)
            return None
        q, p = t.transform.rotation, t.transform.translation
        return transform(quat_to_mat(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])

    def on_aruco(self, msg):
        T_base_cam = self.base_from_camera(msg.header.frame_id)
        if T_base_cam is None:
            return
        for m in msg.markers:
            q, p = m.pose.orientation, m.pose.position
            self.sighting(int(m.marker_id), T_base_cam, transform(quat_to_mat(q.x, q.y, q.z, q.w), [p.x, p.y, p.z]),
                          True, msg.header.stamp)

    def on_json(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        sec = float(data.get('stamp', 0.0))
        stamp = Time(nanoseconds=int(sec * 1e9)).to_msg() if sec > 0 else self.get_clock().now().to_msg()
        T_base_cam = self.base_from_camera(self.p('camera_frame'))
        if T_base_cam is None:
            return
        for d in data.get('detections', []):
            t = [float(d['x']), float(d['y']), float(d['z'])]
            has_rot = all(k in d for k in ('qx', 'qy', 'qz', 'qw'))
            R = quat_to_mat(d['qx'], d['qy'], d['qz'], d['qw']) if has_rot else np.eye(3)
            self.sighting(int(d['id']), T_base_cam, transform(R, t), has_rot, stamp)

    def on_head(self, msg):
        self.head = (time.monotonic(), float(msg.x), float(msg.z))

    def head_turned(self):
        """The URDF's camera pose assumes a centred head; a turned head would give wrong poses."""
        t, pitch, yaw = self.head
        tol = float(self.p('head_tolerance_deg'))
        return time.monotonic() - t < 3.0 and (abs(yaw) > tol or abs(pitch) > tol)

    def sighting(self, mid, T_base_cam, T_cam_marker, has_rot, stamp):
        if self.head_turned():
            self.get_logger().info('head turned: ignoring marker sightings until it is straight again',
                                   throttle_duration_sec=10.0)
            return
        rng = float(np.linalg.norm(T_cam_marker[:3, 3]))
        if rng > float(self.p('max_range_m')):
            return
        # trust the marker's orientation? (close enough, and the solution faces the camera)
        facing = -float(T_cam_marker[:3, 2] @ T_cam_marker[:3, 3]) / max(rng, 1e-6)
        full = has_rot and facing >= float(self.p('min_facing')) and rng <= float(self.p('heading_max_range_m'))
        if mid in self.fixed:
            self.localize(mid, T_base_cam, T_cam_marker, full, rng, stamp)
        else:
            self.learn(mid, T_base_cam, T_cam_marker, full, rng)

    # ------------------------------------------------------------ localization
    def stds(self, rng):
        a, b = self.p('xy_std')
        c, d = self.p('yaw_std_deg')
        return a + b * rng, math.radians(c + d * rng)

    def map_yaw(self):
        try:
            t = self.tf.lookup_transform(self.p('map_frame'), self.p('base_frame'), Time())
        except Exception:
            return None
        q = t.transform.rotation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def localize(self, mid, T_base_cam, T_cam_marker, full, rng, stamp):
        sxy, syaw = self.stds(rng)
        if full:
            T_map_base = self.fixed[mid] @ np.linalg.inv(T_cam_marker) @ np.linalg.inv(T_base_cam)
            yaw = math.atan2(T_map_base[1, 0], T_map_base[0, 0])
            self.publish(self.pub_pose, T_map_base[0, 3], T_map_base[1, 3], yaw, sxy, syaw, stamp)
            return
        yaw = self.map_yaw()                 # position only: heading from the current estimate
        if yaw is None:
            return
        p_base = T_base_cam @ np.append(T_cam_marker[:3, 3], 1.0)
        m = self.fixed[mid][:3, 3]
        c, s = math.cos(yaw), math.sin(yaw)
        x = m[0] - (c * p_base[0] - s * p_base[1])
        y = m[1] - (s * p_base[0] + c * p_base[1])
        self.publish(self.pub_xy, x, y, yaw, sxy * 1.5, math.pi, stamp)

    # ---------------------------------------------------------------- learning
    def learn(self, mid, T_base_cam, T_cam_marker, full, rng):
        try:
            t = self.tf.lookup_transform(self.p('map_frame'), self.p('base_frame'), Time())
        except Exception:
            self.get_logger().info('waiting for the map -> base_link pose before learning markers',
                                   throttle_duration_sec=10.0)
            return
        q, tr = t.transform.rotation, t.transform.translation
        T_map_base = transform(quat_to_mat(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])
        T_map_cam = T_map_base @ T_base_cam
        p_map = T_map_cam @ np.append(T_cam_marker[:3, 3], 1.0)
        if full:                              # marker normal (z) in the map
            z_map = T_map_cam[:3, :3] @ T_cam_marker[:3, 2]
            facing = math.atan2(z_map[1], z_map[0])
        else:                                 # seen from the front: it roughly faces the camera
            facing = math.atan2(T_map_cam[1, 3] - p_map[1], T_map_cam[0, 3] - p_map[0])
        first = not self.fixed
        # uncertainty of this sighting: robot pose + detection noise (both grow with distance)
        sxy, syaw = self.stds(rng)
        vx, vy, vyaw = self.robot_cov if (self.robot_cov and not first) else (0.0, 0.0, 0.0)
        var = sxy ** 2 + (vx + vy) / 2 + (rng ** 2) * min(vyaw, 1.0)
        L = self.learning.setdefault(mid, Learned())
        L.add(p_map[0], p_map[1], p_map[2], 1.0 / var, facing,
              (1.0 if full else 0.15) / (syaw ** 2 + min(vyaw, 1.0)), rng)
        if L.n == 1:
            self.get_logger().info(f'new marker {mid} at ~({p_map[0]:.2f}, {p_map[1]:.2f}), {rng:.1f} m away: learning')
        if first and L.n >= 3:                # the very first marker: fix it at once (defines the map)
            self.fix(mid, 'first marker: defines the map')
        elif L.n >= int(self.p('min_sightings')) and L.best_range <= float(self.p('fix_range_m')):
            self.fix(mid, f'{L.n} sightings, closest {L.best_range:.2f} m')
        else:
            self.publish_markers()

    def fix(self, mid, why):
        x, y, z, facing = self.learning.pop(mid).pose()
        self.fixed[mid] = marker_in_map(x, y, math.degrees(facing), z)
        self.get_logger().info(f'marker {mid} FIXED at ({x:.2f}, {y:.2f}) facing {math.degrees(facing):.0f} deg '
                               f'({why}); now used for localization')
        self.save()
        self.publish_markers()

    # ----------------------------------------------------------------- outputs
    def entries(self):
        out = {}
        for mid, T in self.fixed.items():
            out[str(mid)] = {'x': float(T[0, 3]), 'y': float(T[1, 3]),
                             'facing': math.degrees(math.atan2(T[1, 2], T[0, 2])), 'state': 'fixed', 'n': 0}
        for mid, L in self.learning.items():
            x, y, _, f = L.pose()
            out[str(mid)] = {'x': x, 'y': y, 'facing': math.degrees(f), 'state': 'learning', 'n': L.n}
        return out

    def publish_markers(self):
        self.pub_markers.publish(String(data=json.dumps(self.entries())))

    def publish_viz(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for k, (mid, e) in enumerate(sorted(self.entries().items())):
            for j, kind in enumerate((Marker.CUBE, Marker.TEXT_VIEW_FACING)):
                m = Marker()
                m.header.stamp, m.header.frame_id = stamp, self.p('map_frame')
                m.ns, m.id, m.type = 'markers', 2 * k + j, kind
                m.pose.position.x, m.pose.position.y = e['x'], e['y']
                f = math.radians(e['facing'])
                m.pose.orientation.z, m.pose.orientation.w = math.sin(f / 2), math.cos(f / 2)
                fixed = e['state'] == 'fixed'
                m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.8, 0.2, 1.0) if fixed else (1.0, 0.6, 0.0, 1.0)
                if kind == Marker.CUBE:
                    m.pose.position.z = 0.3
                    m.scale.x, m.scale.y, m.scale.z = 0.03, 0.2, 0.2
                else:
                    m.pose.position.z = 0.55
                    m.scale.z = 0.15
                    m.text = f'{mid}' + ('' if fixed else f' (learning, {e["n"]})')
                arr.markers.append(m)
        self.pub_viz.publish(arr)

    def publish(self, pub, x, y, yaw, sxy, syaw, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp, msg.header.frame_id = stamp, self.p('map_frame')
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = sxy * sxy
        cov[14] = cov[21] = cov[28] = 1e-3
        cov[35] = syaw * syaw
        msg.pose.covariance = cov
        pub.publish(msg)


def main():
    rclpy.init()
    node = MarkerLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
