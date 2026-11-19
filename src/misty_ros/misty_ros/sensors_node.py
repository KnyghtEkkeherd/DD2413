"""Bridge Misty's sensor event streams to ROS for SLAM and navigation.

Subscribes to Misty's event websocket (ws://<ip>/pubsub) and publishes:

    /misty/encoders   sensor_msgs/JointState   left/right wheel distance [m] and speed [m/s]
    /misty/imu        sensor_msgs/Imu          orientation from Misty's yaw/pitch/roll
    /misty/tof        sensor_msgs/Range        one message per time-of-flight sensor;
                                               header.frame_id = tof_<position> (e.g. tof_center)
    /misty/markers    std_msgs/String (JSON)   ArUco detections in the CAMERA frame
                                               (x right, y down, z forward, metres):
                                               {"stamp": t, "source": "robot"|"opencv",
                                                "detections": [{"id": 3, "x": .., "y": .., "z": ..}]}
    /misty/depth_scan sensor_msgs/LaserScan    optional: obstacle scan from the depth camera

ArUco markers come from Misty's on-board detector (aruco_source: robot), or,
as a fallback, from OpenCV running on the /misty/camera/image/compressed
frames (aruco_source: opencv; needs approximate camera intrinsics).

Misty's event field names are parsed case-insensitively and with several
known spellings, because they vary between firmware versions.
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, JointState, LaserScan, Range
from std_msgs.msg import String

from misty_ros.sdk import make_robot
from misty_ros.markers import mat_to_quat
from misty_ros.vision import ArucoDetector, camera_info, depth_to_scan

try:
    import websocket
except ImportError:
    websocket = None


def lower_keys(d):
    return {str(k).lower(): v for k, v in d.items()} if isinstance(d, dict) else {}


def first(d, *keys, default=None):
    for k in keys:
        if k in d and d[k] is not None:
            return d[k]
    return default


def quat_from_ypr(yaw, pitch, roll):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def parse_xyz(v):
    """Accept [x, y, z], {"x":..,"y":..,"z":..} or a JSON string of either."""
    if isinstance(v, str):
        try:
            v = json.loads(v)
        except ValueError:
            return None
    if isinstance(v, (list, tuple)) and len(v) >= 3:
        return [float(v[0]), float(v[1]), float(v[2])]
    if isinstance(v, dict):
        d = lower_keys(v)
        if all(k in d for k in 'xyz'):
            return [float(d['x']), float(d['y']), float(d['z'])]
    return None


class MistySensors(Node):
    def __init__(self):
        super().__init__('misty_sensors')
        self.declare_parameter('misty_ip', '')
        self.declare_parameter('request_timeout', 2.0)
        self.declare_parameter('debounce_ms', 100)          # Misty event rate limit
        self.declare_parameter('encoder_scale', 0.001)      # Misty encoder units -> metres (mm)
        self.declare_parameter('aruco_source', 'robot')     # robot | opencv
        self.declare_parameter('aruco_dictionary', 0)       # OpenCV enum: 0 = DICT_4X4_50
        self.declare_parameter('marker_size_m', 0.15)       # printed marker side length
        self.declare_parameter('robot_tag_scale', 1.0)      # on-board tag translation -> metres
        # OpenCV fallback intrinsics: fx/fy/cx/cy = 0 -> derived from camera_hfov_deg
        self.declare_parameter('camera_hfov_deg', 70.0)
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)
        # optional depth camera obstacle scan
        self.declare_parameter('depth_enabled', False)
        self.declare_parameter('depth_endpoint', 'cameras/depth')
        self.declare_parameter('depth_rate', 2.0)
        self.declare_parameter('depth_hfov_deg', 60.0)
        self.declare_parameter('depth_scale', 0.001)        # depth units -> metres (mm)
        self.declare_parameter('depth_band', [0.35, 0.65])  # image rows used (fraction of height)
        self.declare_parameter('depth_frame_id', 'camera_depth_optical_frame')
        self.declare_parameter('imu_yaw_sign', 1.0)         # -1 if Misty's yaw grows clockwise (ROS: counter-clockwise)
        self.declare_parameter('imu_yaw_std_deg', 1.0)      # orientation noise for robot_localization

        self.ip = self.get_parameter('misty_ip').value
        if not self.ip:
            raise RuntimeError('Parameter misty_ip is required')
        self.misty = make_robot(self.ip, float(self.get_parameter('request_timeout').value))
        self.source = self.get_parameter('aruco_source').value

        self.pub_enc = self.create_publisher(JointState, 'misty/encoders', 20)
        self.pub_imu = self.create_publisher(Imu, 'misty/imu', 20)
        self.pub_tof = self.create_publisher(Range, 'misty/tof', 20)
        self.pub_markers = self.create_publisher(String, 'misty/markers', 10)
        self.pub_depth = self.create_publisher(LaserScan, 'misty/depth_scan', 5)
        self.pub_head = self.create_publisher(Vector3, 'misty/head_state', 10)   # measured head (deg)
        self.head = [0.0, 0.0, 0.0]                                           # pitch, roll, yaw
        self.pub_depth_img = self.create_publisher(Image, 'misty/depth/image', 2)          # for
        self.pub_depth_info = self.create_publisher(CameraInfo, 'misty/depth/camera_info', 2)  # depthimage_to_laserscan
        self.stop_flag = threading.Event()
        self.counts = {}

        if self.source == 'opencv':
            self.setup_opencv()
            self.create_subscription(CompressedImage, 'misty/camera/image/compressed', self.on_image, 2)
        if self.get_parameter('depth_enabled').value:
            threading.Thread(target=self.depth_loop, daemon=True).start()
        if websocket is None:
            self.get_logger().error('python3-websocket missing: no encoder / IMU / ToF / on-board ArUco data')
        else:
            threading.Thread(target=self.pubsub_loop, daemon=True).start()
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'Sensor bridge for Misty {self.ip} (ArUco source: {self.source})')

    def report(self):
        if self.counts:
            self.get_logger().info('events in last 10 s: ' + ', '.join(f'{k}={v}' for k, v in self.counts.items()))
        else:
            self.get_logger().warning('no sensor events from Misty in the last 10 s', throttle_duration_sec=30.0)
        self.counts = {}

    # ---------------------------------------------------------- event socket
    def pubsub_loop(self):
        debounce = int(self.get_parameter('debounce_ms').value)
        subs = {'nav_encoders': 'DriveEncoders', 'nav_imu': 'IMU', 'nav_tof': 'TimeOfFlight',
                'nav_actuator': 'ActuatorPosition'}
        if self.source == 'robot':
            subs['nav_artag'] = 'ArTagDetection'
        warned = False
        while not self.stop_flag.is_set():
            ws = None
            try:
                ws = websocket.create_connection(f'ws://{self.ip}/pubsub', timeout=5)
                if self.source == 'robot':
                    self.start_robot_detector()      # once per (re)connection
                for name, typ in subs.items():
                    ws.send(json.dumps({'Operation': 'subscribe', 'Type': typ, 'DebounceMs': debounce,
                                        'EventName': name, 'Message': ''}))
                ws.settimeout(1.0)
                warned = False
                self.get_logger().info(f'Subscribed to Misty events: {", ".join(subs.values())}')
                while not self.stop_flag.is_set():
                    try:
                        self.on_event(ws.recv())
                    except websocket.WebSocketTimeoutException:
                        continue
            except Exception as e:
                if not warned:
                    self.get_logger().warning(f'Misty event websocket unavailable ({e}); retrying')
                    warned = True
            finally:
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
            self.stop_flag.wait(3.0)

    def start_robot_detector(self):
        try:
            r = self.misty.start_ar_tag_detector(
                dictionary=int(self.get_parameter('aruco_dictionary').value),
                tagSizeMm=float(self.get_parameter('marker_size_m').value) * 1000.0)
            if not r.ok:
                self.get_logger().warning(f'start_ar_tag_detector: HTTP {r.status_code} {r.text[:200]}')
        except Exception as e:
            self.get_logger().warning(f'start_ar_tag_detector failed: {e}')

    def on_event(self, raw):
        try:
            data = json.loads(raw)
        except (TypeError, ValueError):
            return
        data = lower_keys(data)
        msg = data.get('message')
        if isinstance(msg, str):
            try:
                msg = json.loads(msg)
            except ValueError:
                return
        if not isinstance(msg, dict):
            return
        name = data.get('eventname')
        handler = {'nav_encoders': self.on_encoders, 'nav_imu': self.on_imu,
                   'nav_tof': self.on_tof, 'nav_artag': self.on_artag, 'nav_actuator': self.on_actuator}.get(name)
        if handler is not None:
            self.counts[name[4:]] = self.counts.get(name[4:], 0) + 1
            try:
                handler(lower_keys(msg))
            except (TypeError, ValueError, KeyError) as e:
                self.get_logger().warning(f'could not parse {name}: {e}: {str(msg)[:200]}',
                                          throttle_duration_sec=5.0)

    def on_encoders(self, m):
        """Misty resets leftDistance / rightDistance at every new drive command (Nav2 sends
        ~10 per second), so the distances can't be differenced. The track velocities are
        continuous: integrate them into ever-growing wheel distances instead."""
        sc = float(self.get_parameter('encoder_scale').value)
        vl = first(m, 'leftvelocity')
        vr = first(m, 'rightvelocity')
        if vl is None or vr is None:
            raise KeyError('leftVelocity/rightVelocity')
        vl, vr = float(vl) * sc, float(vr) * sc
        now = time.monotonic()
        last = getattr(self, 'enc_last', None)
        if last is not None:
            dt = min(now - last[0], 0.5)               # trapezoid; cap gaps (lost events)
            self.enc_pos[0] += (vl + last[1]) / 2 * dt
            self.enc_pos[1] += (vr + last[2]) / 2 * dt
        else:
            self.enc_pos = [0.0, 0.0]
        self.enc_last = (now, vl, vr)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['left_wheel', 'right_wheel']
        js.position = list(self.enc_pos)
        js.velocity = [vl, vr]
        self.pub_enc.publish(js)

    def on_imu(self, m):
        sign = float(self.get_parameter('imu_yaw_sign').value)
        yaw = sign * math.radians(float(first(m, 'yaw', default=0.0)))
        pitch = math.radians(float(first(m, 'pitch', default=0.0)))
        roll = math.radians(float(first(m, 'roll', default=0.0)))
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        q = quat_from_ypr(yaw, pitch, roll)
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q
        var = math.radians(float(self.get_parameter('imu_yaw_std_deg').value)) ** 2
        imu.orientation_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
        imu.angular_velocity.z = sign * math.radians(
            float(first(m, 'zangularvelocity', 'yawvelocity', default=0.0)))
        imu.angular_velocity_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
        imu.linear_acceleration_covariance[0] = -1.0          # not provided
        self.pub_imu.publish(imu)

    def on_tof(self, m):
        """Valid reading -> its range; invalid / out of range -> max_range ("nothing seen"):
        Nav2's range layer clears on it, and the collision monitor (which rejects +inf)
        keeps getting fresh data."""
        pos = str(first(m, 'sensorposition', 'position', default='unknown')).lower()
        if pos.startswith('down'):               # edge (cliff) sensors: not obstacles
            return
        status = first(m, 'status', default=0)
        dist = first(m, 'distanceinmeters', 'distance')
        valid = dist is not None and (status in (0, '0', None) or first(m, 'inrange', default=False))
        r = Range()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = f'tof_{pos}'
        r.radiation_type = Range.INFRARED
        r.field_of_view = math.radians(25.0)
        r.min_range, r.max_range = 0.01, 2.0
        r.range = float(dist) if valid else r.max_range     # max_range = nothing seen
        self.pub_tof.publish(r)

    def on_actuator(self, m):
        """ActuatorPosition: ahp / ahr / ahy = head pitch / roll / yaw (degrees, yaw + = left).
        Published on /misty/head_state (x = pitch, y = roll, z = yaw) for the camera direction."""
        idx = {'ahp': 0, 'ahr': 1, 'ahy': 2}.get(str(first(m, 'sensorid', default='')).lower())
        if idx is None or first(m, 'value') is None:
            return
        self.head[idx] = float(m['value'])
        self.pub_head.publish(Vector3(x=self.head[0], y=self.head[1], z=self.head[2]))

    def on_artag(self, m):
        """Misty's on-board detector: tagId + homogeneousMatrix (4x4, column-major, metres; marker
        pose in the camera frame, same convention as OpenCV / aruco_opencv). Older firmware may
        send a plain translation instead."""
        tag_id = first(m, 'tagid', 'id', 'artagid', 'markerid')
        hm = m.get('homogeneousmatrix')
        s = float(self.get_parameter('robot_tag_scale').value)
        if tag_id is not None and isinstance(hm, list) and len(hm) == 16:
            T = np.array(hm, dtype=float).reshape(4, 4).T           # column-major -> row-major
            qx, qy, qz, qw = mat_to_quat(T[:3, :3])
            x, y, z = (T[:3, 3] * s).tolist()
            self.publish_markers('robot', [{'id': int(tag_id), 'x': x, 'y': y, 'z': z,
                                            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}])
            return
        xyz = parse_xyz(first(m, 'translation', 'position', 'tvec', default=m))
        if tag_id is None or xyz is None:
            raise KeyError('tagId/homogeneousMatrix')
        self.publish_markers('robot', [{'id': int(tag_id), 'x': xyz[0] * s, 'y': xyz[1] * s, 'z': xyz[2] * s}])

    def publish_markers(self, source, detections):
        self.pub_markers.publish(String(data=json.dumps(
            {'stamp': time.time(), 'source': source, 'detections': detections})))

    # --------------------------------------------------------- OpenCV path
    def setup_opencv(self):
        g = lambda n: self.get_parameter(n).value
        self.aruco = ArucoDetector(g('aruco_dictionary'), g('marker_size_m'), g('camera_hfov_deg'),
                                   g('fx'), g('fy'), g('cx'), g('cy'))

    def on_image(self, msg: CompressedImage):
        img = self.aruco.decode(msg.data)
        if img is None:
            return
        dets = self.aruco.detect(img)
        self.counts['opencv_frames'] = self.counts.get('opencv_frames', 0) + 1
        self.publish_markers('opencv', dets)

    # ---------------------------------------------------------- depth scan
    def depth_loop(self):
        period = 1.0 / float(self.get_parameter('depth_rate').value)
        warned = False
        while not self.stop_flag.is_set():
            try:
                r = self.misty.get_request(self.get_parameter('depth_endpoint').value)
                r.raise_for_status()
                res = lower_keys(r.json().get('result') or {})
                w, h = int(res['width']), int(res['height'])
                img = np.array(res['image'], dtype=np.float64).reshape(h, w)
                self.publish_depth_scan(img)
                self.counts['depth'] = self.counts.get('depth', 0) + 1
                warned = False
            except Exception as e:
                if not warned:
                    self.get_logger().warning(f'depth data unavailable ({e}); retrying')
                    warned = True
            self.stop_flag.wait(period)

    def publish_depth_scan(self, img):
        stamp = self.get_clock().now().to_msg()
        depth = (img * float(self.get_parameter('depth_scale').value)).astype(np.float32)
        depth[~np.isfinite(depth) | (depth <= 0.0)] = np.nan
        frame = self.get_parameter('depth_frame_id').value
        msg = Image(height=depth.shape[0], width=depth.shape[1], encoding='32FC1', step=depth.shape[1] * 4,
                    data=depth.tobytes())
        msg.header.stamp, msg.header.frame_id = stamp, frame
        self.pub_depth_img.publish(msg)
        self.pub_depth_info.publish(camera_info(depth.shape[1], depth.shape[0],
                                                self.get_parameter('depth_hfov_deg').value, frame, stamp))
        self.pub_depth.publish(depth_to_scan(
            img * float(self.get_parameter('depth_scale').value), self.get_parameter('depth_band').value,
            self.get_parameter('depth_hfov_deg').value, self.get_clock().now().to_msg()))

    def shutdown(self):
        self.stop_flag.set()
        if self.source == 'robot':
            try:
                self.misty.post_request('artags/detection/stop')
            except Exception:
                pass


def main():
    rclpy.init()
    node = None
    try:
        node = MistySensors()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
