"""Gazebo Misty <-> the topics of the real robot.

Replaces sensors_node + teleop_node when running in Gazebo (gazebo.launch.py).
Everything downstream (slam_node, nav_node, Foxglove) runs unchanged.

    /misty/cmd_vel (twist_mux output, m/s; drive_time like teleop_node)
                                -> /sim/cmd_vel
    /sim/joint_states           -> /misty/encoders     wheel distances [m] (+ optional slip noise)
    /sim/imu                    -> /misty/imu
    /sim/tof (3-ray gpu_lidar)  -> /misty/tof          tof_right / tof_center / tof_left
    /sim/camera (raw RGB)       -> /misty/camera/image/compressed (JPEG, like the real camera node)
                                -> /misty/markers      the real OpenCV ArUco detector on the rendered image
    /sim/depth (32FC1)          -> /misty/depth_scan
    /sim/ground_truth           -> /sim/true_pose      for checking SLAM
    world JSON                  -> /sim/world          markers / obstacles for Foxglove
"""
import json
import math
import random
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, JointState, LaserScan, Range
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from misty_ros.markers import world_markers
from misty_ros.nav_sim_node import DEFAULT_WORLD
from misty_ros.vision import ArucoDetector, camera_info, depth_to_scan


class GzAdapter(Node):
    def __init__(self):
        super().__init__('misty_gz_adapter')
        P = self.declare_parameter
        P('world', json.dumps(DEFAULT_WORLD))
        P('linear_scale', 30.0)          # like teleop_node: Twist 1.0 -> 30 %
        P('angular_scale', 70.0)
        P('drive_time_ms', 600)
        P('max_linear_mps', 0.6)         # Misty speed at 100 %
        P('max_angular_dps', 120.0)      # Misty turn rate at 100 %
        P('command_latency_s', 0.03)     # extra delay like the HTTP round trip to Misty
        P('wheel_radius_m', 0.04)
        P('slip', 0.03)                  # extra wheel distance noise (fraction)
        P('aruco_dictionary', 0)
        P('marker_size_m', 0.15)
        P('camera_hfov_deg', 70.0)
        P('jpeg_quality', 70)
        P('detect_rate', 5.0)            # max ArUco detection rate [Hz]
        P('depth_hfov_deg', 60.0)
        P('depth_band', [0.35, 0.65])
        P('tof_names', ['right', 'center', 'left'])   # gpu_lidar samples, min angle first
        P('publish_markers', True)       # own OpenCV detections on /misty/markers (json source)

        self.world = json.loads(self.p('world'))
        self.aruco = ArucoDetector(self.p('aruco_dictionary'), self.p('marker_size_m'), self.p('camera_hfov_deg'))
        import cv2
        self.cv2 = cv2

        self.cmd = (0.0, 0.0)
        self.cmd_until = 0.0
        self.pending_cmds = []
        self.last_wheels = None
        self.enc = [0.0, 0.0]
        self.last_detect = 0.0
        self.counts = {}
        self.frame_lock = threading.Lock()
        self.frame = None

        self.pub_sim_cmd = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.pub_enc = self.create_publisher(JointState, 'misty/encoders', 20)
        self.pub_imu = self.create_publisher(Imu, 'misty/imu', 20)
        self.pub_tof = self.create_publisher(Range, 'misty/tof', 20)
        self.pub_markers = self.create_publisher(String, 'misty/markers', 10)
        self.pub_img = self.create_publisher(CompressedImage, 'misty/camera/image/compressed', 1)
        self.pub_info = self.create_publisher(CameraInfo, 'misty/camera/camera_info', 1)
        self.pub_depth_img = self.create_publisher(Image, 'misty/depth/image', 2)
        self.pub_depth_info = self.create_publisher(CameraInfo, 'misty/depth/camera_info', 2)
        self.pub_depth = self.create_publisher(LaserScan, 'misty/depth_scan', 5)
        self.pub_true = self.create_publisher(PoseStamped, 'sim/true_pose', 10)
        self.pub_world = self.create_publisher(MarkerArray, 'sim/world', 1)

        self.create_subscription(TwistStamped, 'misty/cmd_vel', self.on_mux_cmd, 10)   # twist_mux output
        self.create_subscription(JointState, '/sim/joint_states', self.on_joints, 50)
        self.create_subscription(Imu, '/sim/imu', self.on_imu, 50)
        self.create_subscription(LaserScan, '/sim/tof', self.on_tof, 10)
        self.create_subscription(Image, '/sim/camera', self.on_camera, 1)
        self.create_subscription(Image, '/sim/depth', self.on_depth, 1)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_truth, 10)

        self.create_timer(0.02, self.drive)
        self.create_timer(1.0, lambda: self.pub_world.publish(world_markers(self.world, self.get_clock().now().to_msg())))
        self.create_timer(10.0, self.report)
        threading.Thread(target=self.vision_loop, daemon=True).start()
        self.get_logger().info('Gazebo adapter up: waiting for /sim/* topics from ros_gz_bridge')

    def p(self, name):
        return self.get_parameter(name).value

    def count(self, k):
        self.counts[k] = self.counts.get(k, 0) + 1

    def report(self):
        missing = [k for k in ('joints', 'imu', 'camera') if not self.counts.get(k)]
        if missing:
            self.get_logger().warning(f'no {", ".join(missing)} from Gazebo in the last 10 s '
                                      '(is gz sim running and the bridge up?)')
        else:
            self.get_logger().info('last 10 s: ' + ', '.join(f'{k}={v}' for k, v in sorted(self.counts.items())))
        self.counts = {}

    # ------------------------------------------------------------- driving
    def on_mux_cmd(self, msg: TwistStamped):
        """twist_mux output (m/s, rad/s), limited like Misty at 100 %."""
        vmax, wmax = float(self.p('max_linear_mps')), math.radians(float(self.p('max_angular_dps')))
        v = max(-vmax, min(vmax, msg.twist.linear.x))
        w = max(-wmax, min(wmax, msg.twist.angular.z))
        self.pending_cmds.append((time.monotonic() + float(self.p('command_latency_s')), v, w))

    def drive(self):
        now = time.monotonic()
        while self.pending_cmds and self.pending_cmds[0][0] <= now:
            _, v, w = self.pending_cmds.pop(0)
            if v == 0.0 and w == 0.0:
                self.cmd, self.cmd_until = (0.0, 0.0), 0.0
            else:
                self.cmd, self.cmd_until = (v, w), now + int(self.p('drive_time_ms')) / 1000.0
        v, w = self.cmd if now < self.cmd_until else (0.0, 0.0)
        t = Twist()
        t.linear.x, t.angular.z = v, w
        self.pub_sim_cmd.publish(t)

    # ------------------------------------------------------------- sensors
    def on_joints(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        try:
            wheels = (pos['left_wheel_joint'], pos['right_wheel_joint'])
        except KeyError:
            return
        r, slip = float(self.p('wheel_radius_m')), float(self.p('slip'))
        if self.last_wheels is not None:
            for i in range(2):
                d = (wheels[i] - self.last_wheels[i]) * r
                self.enc[i] += d * (1 + random.gauss(0, slip))
        self.last_wheels = wheels
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name, js.position = ['left_wheel', 'right_wheel'], list(self.enc)
        self.pub_enc.publish(js)
        self.count('joints')

    def on_imu(self, msg: Imu):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.orientation_covariance = [3e-4, 0.0, 0.0, 0.0, 3e-4, 0.0, 0.0, 0.0, 3e-4]
        msg.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        self.pub_imu.publish(msg)
        self.count('imu')

    def on_tof(self, msg: LaserScan):
        stamp = self.get_clock().now().to_msg()
        for name, d in zip(self.p('tof_names'), msg.ranges):
            r = Range()
            r.header.stamp, r.header.frame_id = stamp, f'tof_{name}'
            r.radiation_type = Range.INFRARED
            r.field_of_view = math.radians(25.0)
            r.min_range, r.max_range = msg.range_min, msg.range_max
            r.range = float(d) if math.isfinite(d) and msg.range_min <= d < msg.range_max else math.inf
            self.pub_tof.publish(r)

    def on_truth(self, msg: Odometry):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose = msg.pose.pose
        self.pub_true.publish(ps)

    def on_depth(self, msg: Image):
        if msg.encoding != '32FC1':
            self.get_logger().warning(f'depth encoding {msg.encoding} unsupported', throttle_duration_sec=30.0)
            return
        img = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp, msg.header.frame_id = stamp, 'camera_depth_optical_frame'
        self.pub_depth_img.publish(msg)
        self.pub_depth_info.publish(camera_info(msg.width, msg.height, self.p('depth_hfov_deg'),
                                                'camera_depth_optical_frame', stamp))
        self.pub_depth.publish(depth_to_scan(img, self.p('depth_band'), self.p('depth_hfov_deg'),
                                             self.get_clock().now().to_msg()))
        self.count('depth')

    def on_camera(self, msg: Image):
        with self.frame_lock:                    # keep only the newest frame; the vision thread
            self.frame = msg                     # drops anything it can't keep up with
        self.count('camera')

    def vision_loop(self):
        cv2 = self.cv2
        while rclpy.ok():
            with self.frame_lock:
                msg, self.frame = self.frame, None
            if msg is None:
                time.sleep(0.01)
                continue
            ch = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
            img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
            img = img.reshape(msg.height, msg.width, ch) if ch == 3 else img.reshape(msg.height, msg.width)
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img
            ok, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, int(self.p('jpeg_quality'))])
            if ok:
                out = CompressedImage()
                out.header.stamp = self.get_clock().now().to_msg()
                out.header.frame_id = 'camera_optical_frame'
                out.format = 'jpeg'
                out.data = jpg.tobytes()
                self.pub_img.publish(out)
                self.pub_info.publish(camera_info(msg.width, msg.height, self.p('camera_hfov_deg'),
                                                  'camera_optical_frame', out.header.stamp))
            now = time.monotonic()
            if self.p('publish_markers') and now - self.last_detect >= 1.0 / float(self.p('detect_rate')):
                self.last_detect = now
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
                dets = self.aruco.detect(gray)
                self.pub_markers.publish(String(data=json.dumps(
                    {'stamp': time.time(), 'source': 'gazebo', 'detections': dets})))
                self.count('detections' if dets else 'no_marker_frames')

    def shutdown(self):
        self.pub_sim_cmd.publish(Twist())


def main():
    rclpy.init()
    node = None
    try:
        node = GzAdapter()
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
