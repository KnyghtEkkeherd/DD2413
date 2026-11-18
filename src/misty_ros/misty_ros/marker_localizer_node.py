"""ArUco sightings + measured marker map -> Misty's pose in the map, for robot_localization.

The marker map (parameter marker_map, JSON) gives each marker's position and
the direction its printed side faces:  {"id": [x, y, facing_deg]}  (metres, map
frame; optional 4th value: centre height). For every detected marker:

    T_map_base = T_map_marker * inverse(T_camera_marker) * inverse(T_base_camera)

T_base_camera comes from TF (urdf/misty.urdf), so the camera mount is
configured in one place. Detections come from

    marker_source: aruco_opencv   aruco_opencv_msgs/ArucoDetection on /aruco_detections
                                  (full 6D marker pose -> x, y and heading)
    marker_source: json           /misty/markers from Misty's on-board detector or the
                                  simulators; with qx..qw like aruco_opencv, else position
                                  only (heading then comes from the current estimate)

Publishes (PoseWithCovarianceStamped, frame map; uncertainty grows with distance):
    /localization/marker_pose     x, y, yaw   -> ekf_map pose0
    /localization/marker_pose_xy  x, y        -> ekf_map pose1 (position-only detections)
    /localization/marker_map      MarkerArray of the map, for Foxglove
"""
import json
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from misty_ros.markers import marker_in_map, quat_to_mat, transform, world_markers


class MarkerLocalizer(Node):
    def __init__(self):
        super().__init__('misty_marker_localizer')
        P = self.declare_parameter
        P('marker_map', '{}')
        P('marker_height_m', 0.30)          # default centre height when the map has none
        P('marker_source', 'aruco_opencv')  # aruco_opencv | json
        P('map_frame', 'map')
        P('base_frame', 'base_link')
        P('camera_frame', 'camera_optical_frame')   # for json detections (no header)
        P('max_range_m', 3.5)               # ignore markers further than this
        P('heading_max_range_m', 2.0)       # further: single-marker heading is unreliable -> position only
        P('min_facing', 0.2)                # marker normal must point at the camera (cos); else the PnP
                                            # solution is the flipped one -> position only
        P('xy_std', [0.03, 0.04])           # metres: a + b * distance
        P('yaw_std_deg', [3.0, 4.0])        # degrees: a + b * distance

        self.map = {}
        for mid, v in json.loads(self.get_parameter('marker_map').value or '{}').items():
            h = v[3] if len(v) > 3 else float(self.get_parameter('marker_height_m').value)
            self.map[int(mid)] = marker_in_map(v[0], v[1], v[2], h)
        if not self.map:
            self.get_logger().error('marker_map is empty: set the measured marker positions in misty_params.yaml')
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, 'localization/marker_pose', 10)
        self.pub_xy = self.create_publisher(PoseWithCovarianceStamped, 'localization/marker_pose_xy', 10)
        self.pub_map = self.create_publisher(MarkerArray, 'localization/marker_map', 1)
        world = {'markers': {k: v for k, v in json.loads(self.get_parameter('marker_map').value or '{}').items()}}
        self.create_timer(2.0, lambda: self.pub_map.publish(world_markers(world, self.get_clock().now().to_msg())))

        source = self.get_parameter('marker_source').value
        if source == 'aruco_opencv':
            from aruco_opencv_msgs.msg import ArucoDetection
            self.create_subscription(ArucoDetection, 'aruco_detections', self.on_aruco, 10)
        else:
            self.create_subscription(String, 'misty/markers', self.on_json, 10)
        self.unknown_warned = set()
        self.get_logger().info(f'Marker localizer: {len(self.map)} markers in the map, source {source}')

    def p(self, name):
        return self.get_parameter(name).value

    def base_from_camera(self, frame, stamp):
        try:
            t = self.tf.lookup_transform(self.p('base_frame'), frame, Time())   # static mount
        except Exception as e:
            self.get_logger().warning(f'no TF {self.p("base_frame")} -> {frame}: {e}', throttle_duration_sec=5.0)
            return None
        q, p = t.transform.rotation, t.transform.translation
        return transform(quat_to_mat(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])

    def known(self, mid):
        if mid in self.map:
            return True
        if mid not in self.unknown_warned:
            self.unknown_warned.add(mid)
            self.get_logger().warning(f'marker {mid} seen but not in marker_map: ignored')
        return False

    # ---------------------------------------------------------------- inputs
    def on_aruco(self, msg):
        T_base_cam = self.base_from_camera(msg.header.frame_id, msg.header.stamp)
        if T_base_cam is None:
            return
        for m in msg.markers:
            if self.known(m.marker_id):
                q, p = m.pose.orientation, m.pose.position
                self.full_pose(m.marker_id, T_base_cam,
                               transform(quat_to_mat(q.x, q.y, q.z, q.w), [p.x, p.y, p.z]), msg.header.stamp)

    def on_json(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        sec = float(data.get('stamp', 0.0))
        stamp = Time(nanoseconds=int(sec * 1e9)).to_msg() if sec > 0 else self.get_clock().now().to_msg()
        T_base_cam = self.base_from_camera(self.p('camera_frame'), stamp)
        if T_base_cam is None:
            return
        for d in data.get('detections', []):
            mid = int(d['id'])
            if not self.known(mid):
                continue
            t = [float(d['x']), float(d['y']), float(d['z'])]
            if all(k in d for k in ('qx', 'qy', 'qz', 'qw')):
                self.full_pose(mid, T_base_cam, transform(quat_to_mat(d['qx'], d['qy'], d['qz'], d['qw']), t), stamp)
            else:
                self.position_only(mid, T_base_cam, np.array(t + [1.0]), stamp)

    # --------------------------------------------------------------- outputs
    def stds(self, rng):
        a, b = self.p('xy_std')
        c, d = self.p('yaw_std_deg')
        return a + b * rng, math.radians(c + d * rng)

    def full_pose(self, mid, T_base_cam, T_cam_marker, stamp):
        rng = float(np.linalg.norm(T_cam_marker[:3, 3]))
        if rng > float(self.p('max_range_m')):
            return
        # cos(angle between the marker normal and the line of sight back to the camera)
        facing = -float(T_cam_marker[:3, 2] @ T_cam_marker[:3, 3]) / max(rng, 1e-6)
        if facing < float(self.p('min_facing')) or rng > float(self.p('heading_max_range_m')):
            self.position_only(mid, T_base_cam, np.append(T_cam_marker[:3, 3], 1.0), stamp)
            return
        T_map_base = self.map[mid] @ np.linalg.inv(T_cam_marker) @ np.linalg.inv(T_base_cam)
        yaw = math.atan2(T_map_base[1, 0], T_map_base[0, 0])
        sxy, syaw = self.stds(rng)
        self.publish(self.pub_pose, T_map_base[0, 3], T_map_base[1, 3], yaw, sxy, syaw, stamp)

    def position_only(self, mid, T_base_cam, p_cam, stamp):
        rng = float(np.linalg.norm(p_cam[:3]))
        if rng > float(self.p('max_range_m')):
            return
        try:                                  # heading from the current estimate
            t = self.tf.lookup_transform(self.p('map_frame'), self.p('base_frame'), Time())
        except Exception:
            self.get_logger().info('position-only marker: waiting for a first heading estimate (map -> base_link)',
                                   throttle_duration_sec=10.0)
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p_base = T_base_cam @ p_cam
        m = self.map[mid][:3, 3]
        c, s = math.cos(yaw), math.sin(yaw)
        x = m[0] - (c * p_base[0] - s * p_base[1])
        y = m[1] - (s * p_base[0] + c * p_base[1])
        sxy, _ = self.stds(rng)
        self.publish(self.pub_xy, x, y, yaw, sxy * 1.5, math.pi, stamp)

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
