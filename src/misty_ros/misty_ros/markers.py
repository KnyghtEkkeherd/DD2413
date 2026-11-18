"""Marker geometry shared by the localizer, the simulators and Foxglove drawing.

Marker map / world format: {"id": [x, y, facing_deg(, height)]}; facing is the
direction the printed side faces. ArUco marker frame (as OpenCV / aruco_opencv):
z out of the printed face, x to the viewer's right, y up.
"""
import math

import numpy as np
from visualization_msgs.msg import Marker, MarkerArray


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    """Rotation matrix -> (x, y, z, w) (Shepperd's method: stable for every rotation)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 2 * math.sqrt(1 + t)
        return (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0]
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return q[0], q[1], q[2], (R[k, j] - R[j, k]) / s


def transform(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def marker_in_map(x, y, facing_deg, height):
    """ArUco marker frame: z out of the printed face, x to the viewer's right, y up."""
    f = math.radians(facing_deg)
    z = np.array([math.cos(f), math.sin(f), 0.0])
    yv = np.array([0.0, 0.0, 1.0])
    return transform(np.column_stack([np.cross(yv, z), yv, z]), [x, y, height])


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
