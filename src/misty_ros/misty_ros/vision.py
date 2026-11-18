"""Camera processing shared by the real sensor bridge and the Gazebo simulator.

ArucoDetector: ArUco markers -> positions in the camera frame
(x right, y down, z forward, metres), the format of /misty/markers.
depth_to_scan: depth image (metres) -> LaserScan for obstacle avoidance.
"""
import math

import numpy as np
from sensor_msgs.msg import LaserScan

from misty_ros.markers import mat_to_quat


class ArucoDetector:
    def __init__(self, dictionary, marker_size_m, hfov_deg=70.0, fx=0.0, fy=0.0, cx=0.0, cy=0.0):
        import cv2
        self.cv2 = cv2
        d = cv2.aruco.getPredefinedDictionary(int(dictionary))
        self.detector = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        h = float(marker_size_m) / 2.0
        self.obj_pts = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)
        self.hfov_deg, self.fx, self.fy, self.cx, self.cy = float(hfov_deg), fx, fy, cx, cy

    def intrinsics(self, w, h):
        """fx/fy/cx/cy = 0 -> derived from the horizontal field of view."""
        fx = float(self.fx) or (w / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        fy = float(self.fy) or fx
        cx = float(self.cx) or w / 2.0
        cy = float(self.cy) or h / 2.0
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    def decode(self, jpeg_bytes):
        return self.cv2.imdecode(np.frombuffer(bytes(jpeg_bytes), np.uint8), self.cv2.IMREAD_GRAYSCALE)

    def detect(self, gray):
        """Grayscale image -> [{"id", "x", "y", "z", "qx", "qy", "qz", "qw"}]: marker pose in the
        camera frame (marker frame as aruco_opencv: z out of the face, x right, y up).

        SOLVEPNP_ITERATIVE, not IPPE_SQUARE: with OpenCV 4.10, IPPE returns a flipped
        (marker facing away) solution for many clean, near-symmetric corner sets.
        """
        cv2 = self.cv2
        corners, ids, _ = self.detector.detectMarkers(gray)
        dets = []
        if ids is not None:
            K = self.intrinsics(gray.shape[1], gray.shape[0])
            for c, i in zip(corners, ids.flatten()):
                ok, rvec, tvec = cv2.solvePnP(self.obj_pts.astype(np.float64), c.reshape(4, 2).astype(np.float64),
                                              K, None, flags=cv2.SOLVEPNP_ITERATIVE)
                if ok:
                    t = tvec.flatten()
                    qx, qy, qz, qw = mat_to_quat(cv2.Rodrigues(rvec)[0])
                    dets.append({'id': int(i), 'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2]),
                                 'qx': float(qx), 'qy': float(qy), 'qz': float(qz), 'qw': float(qw)})
        return dets


def camera_info(width, height, hfov_deg, frame_id, stamp=None, fx=0.0, fy=0.0, cx=0.0, cy=0.0):
    """sensor_msgs/CameraInfo for an undistorted pinhole camera (fx = 0: from the field of view).

    Put real values from `ros2 run camera_calibration cameracalibrator` in fx/fy/cx/cy
    for accurate marker distances.
    """
    from sensor_msgs.msg import CameraInfo
    fx = float(fx) or (width / 2.0) / math.tan(math.radians(float(hfov_deg)) / 2.0)
    fy, cx, cy = float(fy) or fx, float(cx) or width / 2.0, float(cy) or height / 2.0
    ci = CameraInfo()
    if stamp is not None:
        ci.header.stamp = stamp
    ci.header.frame_id = frame_id
    ci.width, ci.height = int(width), int(height)
    ci.distortion_model = 'plumb_bob'
    ci.d = [0.0] * 5
    ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return ci


def depth_to_scan(depth_m, band, hfov_deg, stamp, frame_id='camera'):
    """Nearest obstacle per image column in a horizontal band of rows -> LaserScan."""
    lo, hi = band
    h, w = depth_m.shape
    rows = depth_m[int(h * lo):max(int(h * hi), int(h * lo) + 1)].astype(np.float64)
    rows[~np.isfinite(rows) | (rows <= 0.05)] = np.inf
    ranges = rows.min(axis=0)[::-1]          # image left -> scan angle positive (left)
    fov = math.radians(float(hfov_deg))
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.header.frame_id = frame_id
    scan.angle_min, scan.angle_max = -fov / 2, fov / 2
    scan.angle_increment = fov / max(w - 1, 1)
    scan.range_min, scan.range_max = 0.05, 5.0
    scan.ranges = [float(x) for x in ranges]
    return scan
