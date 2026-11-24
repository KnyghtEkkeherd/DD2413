"""Camera feed with overlays for the wizard (Foxglove).

/misty/camera/image/compressed  -> /misty/camera/overlay/compressed (JPEG), with
  * every ArUco marker in view: outline + id + distance (OpenCV detection on the
    frame itself; distance from Misty's own detector on /misty/markers when fresh,
    else OpenCV's estimate). The current navigation target is green, others yellow.
  * the current phase: study state (/study/state) and navigation status (/nav/status).
The raw image topic is left untouched (navigation / detection use it). Frames are
processed at most max_rate per second in a worker thread (newest frame only).
"""
import json
import math
import re
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from misty_ros.vision import ArucoDetector


class CameraOverlay(Node):
    def __init__(self):
        super().__init__('misty_camera_overlay')
        P = self.declare_parameter
        P('aruco_dictionary', 0)                # same as misty_sensors
        P('marker_size_m', 0.15)
        P('camera_hfov_deg', 70.0)              # only for the OpenCV distance fallback
        P('max_rate', 8.0)                      # overlay frames per second, at most
        P('jpeg_quality', 70)
        import cv2
        self.cv2 = cv2
        self.aruco = ArucoDetector(int(self.p('aruco_dictionary')), float(self.p('marker_size_m')),
                                   float(self.p('camera_hfov_deg')))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(CompressedImage, 'misty/camera/overlay/compressed', 1)
        self.create_subscription(CompressedImage, 'misty/camera/image/compressed', self.on_image, 1)
        self.create_subscription(String, 'misty/markers', self.on_markers, 10)
        self.create_subscription(String, 'study/state', lambda m: self.text.__setitem__('study', m.data), latched)
        self.create_subscription(String, 'nav/status', lambda m: self.text.__setitem__('nav', m.data), latched)
        self.text = {'study': '', 'nav': ''}
        self.robot_dist = {}                    # id -> (time, distance from Misty's detector)
        self.lock = threading.Lock()
        self.frame = None
        self.stop = threading.Event()
        threading.Thread(target=self.loop, daemon=True).start()

    def p(self, name):
        return self.get_parameter(name).value

    def on_image(self, msg):
        with self.lock:
            self.frame = msg                    # keep only the newest frame

    def on_markers(self, msg):
        try:
            now = time.monotonic()
            for d in json.loads(msg.data).get('detections', []):
                self.robot_dist[int(d['id'])] = (now, math.hypot(float(d['x']), float(d['z'])))
        except (ValueError, KeyError, TypeError):
            pass

    def target(self):
        m = re.search(r'marker (\d+)', self.text['nav'])
        return int(m.group(1)) if m and not self.text['nav'].startswith(('IDLE', 'DONE')) else None

    def loop(self):
        period = 1.0 / max(float(self.p('max_rate')), 0.5)
        while not self.stop.is_set():
            t0 = time.monotonic()
            with self.lock:
                msg, self.frame = self.frame, None
            if msg is not None:
                try:
                    self.publish(msg)
                except Exception as e:          # a bad frame must not stop the overlay
                    self.get_logger().warning(f'overlay: {e!r}', throttle_duration_sec=10.0)
            self.stop.wait(max(0.01, period - (time.monotonic() - t0)))

    def publish(self, msg):
        cv2 = self.cv2
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        h, w = img.shape[:2]
        scale = w / 480.0                       # text / line size relative to a 480 px wide frame
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco.detector.detectMarkers(gray)
        target, now = self.target(), time.monotonic()
        need_pose = ids is not None and any(
            not (int(i) in self.robot_dist and now - self.robot_dist[int(i)][0] < 1.5) for i in ids.flatten())
        poses = {d['id']: d for d in self.aruco.detect(gray)} if need_pose else {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                i = int(i)
                pts = c.reshape(4, 2).astype(np.int32)
                color = (0, 220, 0) if i == target else (0, 220, 255)       # BGR: green / yellow
                cv2.polylines(img, [pts], True, color, max(2, int(3 * scale)))
                rd = self.robot_dist.get(i)
                if rd and now - rd[0] < 1.5:
                    dist = f'{rd[1]:.2f} m'
                elif i in poses:
                    dist = f'~{math.hypot(poses[i]["x"], poses[i]["z"]):.2f} m'
                else:
                    dist = ''
                label = f'ID {i}  {dist}' + ('  TARGET' if i == target else '')
                x, y = int(pts[:, 0].min()), int(pts[:, 1].min()) - int(8 * scale)
                self.label(img, label, (x, max(int(20 * scale), y)), 0.6 * scale, color)
        # phase text (top): study state, then navigation status
        y = int(24 * scale)
        for key, color in (('study', (255, 255, 255)), ('nav', (180, 255, 180))):
            text = self.text[key]
            if text:
                for line in self.wrap(text, int(w / (11 * scale))):
                    self.label(img, line, (int(6 * scale), y), 0.55 * scale, color)
                    y += int(22 * scale)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(self.p('jpeg_quality'))])
        if ok:
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = jpg.tobytes()
            self.pub.publish(out)

    @staticmethod
    def wrap(text, width):
        words, lines, cur = text.split(), [], ''
        for wd in words:
            if len(cur) + len(wd) + 1 > max(width, 10) and cur:
                lines.append(cur)
                cur = wd
            else:
                cur = f'{cur} {wd}'.strip()
        return (lines + [cur])[:3]

    def label(self, img, text, org, size, color):
        """Text with a dark background box, readable on any image."""
        cv2 = self.cv2
        th = max(1, int(round(size * 2)))
        (tw, tht), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, th)
        x, y = org
        cv2.rectangle(img, (x - 3, y - tht - 4), (x + tw + 3, y + base + 2), (0, 0, 0), -1)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, th, cv2.LINE_AA)


def main():
    rclpy.init()
    node = CameraOverlay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
