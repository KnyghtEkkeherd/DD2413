"""Poll Misty's RGB camera and publish frames as sensor_msgs/CompressedImage.

Uses the SDK's take_picture REST call (GET /api/cameras/rgb, base64=true),
which returns a JPEG. Foxglove's Image panel displays CompressedImage directly,
and JPEG keeps bandwidth low over a remote connection.
"""
import base64

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from misty_ros.sdk import make_robot


class MistyCamera(Node):
    def __init__(self):
        super().__init__('misty_camera')
        self.declare_parameter('misty_ip', '')
        self.declare_parameter('fps', 2.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('frame_id', 'misty_camera')
        self.declare_parameter('request_timeout', 3.0)

        ip = self.get_parameter('misty_ip').value
        if not ip:
            raise RuntimeError('Parameter misty_ip is required (or set MISTY_IP for the launch file).')
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.frame_id = self.get_parameter('frame_id').value
        fps = float(self.get_parameter('fps').value)

        self.misty = make_robot(ip, float(self.get_parameter('request_timeout').value))
        self.pub = self.create_publisher(CompressedImage, 'misty/camera/image/compressed', 1)
        self._ensure_camera_service()
        self.timer = self.create_timer(1.0 / fps, self.grab)
        self.get_logger().info(
            f'Polling camera on Misty {ip} at {fps} fps, {self.width}x{self.height}')

    def _ensure_camera_service(self):
        try:
            r = self.misty.camera_service_enabled()
            if r.ok and r.json().get('result') is False:
                self.get_logger().warning('Camera service disabled on Misty; enabling it')
                self.misty.enable_camera_service()
        except Exception as e:  # robot unreachable: grab() will keep retrying
            self.get_logger().warning(f'Could not check camera service: {e}')

    def grab(self):
        try:
            r = self.misty.get_request('cameras/rgb', json={
                'base64': True, 'width': self.width, 'height': self.height})
            r.raise_for_status()
            data, fmt = self._decode(r)
        except Exception as e:
            self.get_logger().warning(f'take_picture failed: {e}', throttle_duration_sec=5.0)
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = fmt
        msg.data = data
        self.pub.publish(msg)

    @staticmethod
    def _decode(r):
        ctype = r.headers.get('Content-Type', '')
        if ctype.startswith('image/'):  # raw image bytes
            return r.content, ctype.split('/', 1)[1]
        body = r.json()
        result = body.get('result') or {}
        if body.get('status') not in (None, 'Success') or not result:
            raise RuntimeError(body.get('error') or body)
        result = {k.lower(): v for k, v in result.items()}
        b64 = result['base64']
        if b64.startswith('data:'):
            b64 = b64.split(',', 1)[1]
        fmt = (result.get('contenttype') or 'image/jpeg').split('/')[-1]
        return base64.b64decode(b64), fmt


def main():
    rclpy.init()
    node = None
    try:
        node = MistyCamera()
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
