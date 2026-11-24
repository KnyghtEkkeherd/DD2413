"""Publish Misty's RGB camera as sensor_msgs/CompressedImage (JPEG).

Modes (parameter `mode`):
  stream  (default) Misty's video streaming service (POST videostreaming/start,
          frames over a websocket at ws://<ip>:<stream_port>). Low latency.
          Frames may arrive as raw JPEG bytes, base64 text or JSON with a
          base64 field; all are accepted. If the stream can't be started or
          sends nothing usable, falls back to `poll` and retries the stream.
  rtsp    Misty's AV streaming service (POST avstreaming/start, Misty serves
          RTSP at rtsp://<ip>:<rtsp_port>), decoded with OpenCV/FFmpeg. Highest
          frame rate, but enabling AV streaming DISABLES Misty's camera
          service: no photos and no on-board ArUco detection, so navigation
          must use aruco_source: opencv.
  poll    single photos (GET cameras/rgb), `fps` per second. Slowest.

Only the newest frame is ever published (no queue), so a slow consumer never
sees old images. `rotate_deg` (0/90/180/270, clockwise) fixes a rotated stream; photos
(poll mode and the poll fallback) come upright and are never rotated.
"""
import base64
import json
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage

from misty_ros.sdk import make_robot
from misty_ros.vision import camera_info

try:
    import websocket
except ImportError:
    websocket = None

JPEG_SOI = b'\xff\xd8'


def extract_jpeg(payload):
    """Return JPEG bytes from a websocket message in any of the known shapes, or None."""
    if isinstance(payload, (bytes, bytearray)):
        payload = bytes(payload)
        if payload.startswith(JPEG_SOI):
            return payload
        i = payload.find(JPEG_SOI)          # small binary header before the JPEG
        if 0 < i < 64:
            return payload[i:]
        try:
            payload = payload.decode()
        except UnicodeDecodeError:
            return None
    if not isinstance(payload, str):
        return None
    text = payload.strip()
    if text.startswith('{'):
        try:
            obj = json.loads(text)
        except ValueError:
            return None
        if isinstance(obj, dict):
            obj = {k.lower(): v for k, v in obj.items()}
            inner = obj.get('result') if isinstance(obj.get('result'), dict) else obj
            inner = {k.lower(): v for k, v in inner.items()}
            for key in ('base64', 'image', 'data', 'frame'):
                if isinstance(inner.get(key), str):
                    return extract_jpeg(inner[key])
        return None
    if text.startswith('data:'):
        text = text.split(',', 1)[1]
    try:
        raw = base64.b64decode(text, validate=False)
    except (ValueError, TypeError):
        return None
    return raw if raw.startswith(JPEG_SOI) else None


class MistyCamera(Node):
    def __init__(self):
        super().__init__('misty_camera')
        P = self.declare_parameter
        P('misty_ip', '')
        P('mode', 'stream')                 # stream | rtsp | poll
        P('fps', 2.0)                       # poll mode
        P('poll_fallback', False)           # stream mode: take photos while the stream is down? Photos
                                            # starve Misty's own marker / face detection (navigation)
        P('stream_connect_s', 10.0)         # after starting the stream, keep trying to connect this long
        P('width', 640)
        P('height', 480)
        P('quality', 60)                    # stream: JPEG quality 1-100 (lower = less latency)
        P('stream_port', 5678)
        P('stream_rotation', 0)             # rotation applied by Misty (stream mode)
        P('rtsp_port', 1936)
        P('rtsp_fps', 15)
        P('rtsp_bitrate', 1500000)
        P('rotate_deg', 0)                  # rotation applied here (clockwise)
        P('jpeg_quality', 70)               # re-encoding quality (rtsp / rotate)
        P('frame_id', 'camera_optical_frame')   # must match the URDF (urdf/misty.urdf)
        P('camera_hfov_deg', 70.0)          # CameraInfo for aruco_opencv: field of view, or a
        P('fx', 0.0)                        # calibration (fx/fy/cx/cy from camera_calibration)
        P('fy', 0.0)
        P('cx', 0.0)
        P('cy', 0.0)
        P('request_timeout', 3.0)

        self.ip = self.get_parameter('misty_ip').value
        if not self.ip:
            raise RuntimeError('Parameter misty_ip is required (or set MISTY_IP for the launch file).')
        self.mode = self.get_parameter('mode').value
        self.host = self.ip.rsplit(':', 1)[0] if self.ip.count(':') == 1 else self.ip   # strip an HTTP port
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.rotate = int(self.get_parameter('rotate_deg').value) % 360
        self.misty = make_robot(self.ip, float(self.get_parameter('request_timeout').value))
        self.pub = self.create_publisher(CompressedImage, 'misty/camera/image/compressed', 1)
        self.pub_info = self.create_publisher(CameraInfo, 'misty/camera/camera_info', 1)
        w, h = (self.height, self.width) if self.rotate in (90, 270) else (self.width, self.height)
        g = lambda n: self.get_parameter(n).value
        self.info = camera_info(w, h, g('camera_hfov_deg'), self.frame_id, fx=g('fx'), fy=g('fy'),
                                cx=g('cx'), cy=g('cy'))
        self.stop_flag = threading.Event()
        self.frames = 0
        self.last_frame = 0.0
        self.polling = self.mode == 'poll'
        self.cv2 = None
        if self.rotate or self.mode == 'rtsp':
            import cv2
            import numpy as np
            self.cv2, self.np = cv2, np

        if self.mode == 'rtsp':
            threading.Thread(target=self.rtsp_loop, daemon=True).start()
        else:
            self.ensure_camera_service()
            if self.mode == 'stream':
                if websocket is None:
                    self.get_logger().error('python3-websocket missing: using poll mode')
                    self.polling = True
                else:
                    threading.Thread(target=self.stream_loop, daemon=True).start()
        self.create_timer(1.0 / float(self.get_parameter('fps').value), self.poll_tick)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'Camera on Misty {self.ip}: mode {self.mode}, {self.width}x{self.height}')

    def report(self):
        src = 'poll fallback' if (self.polling and self.mode != 'poll') else self.mode
        if self.frames:
            self.get_logger().info(f'{self.frames / 5.0:.1f} fps ({src})')
        self.frames = 0

    # ----------------------------------------------------------- publishing
    def publish_jpeg(self, data, rotated=False):
        if self.rotate and self.cv2 is not None and not rotated:
            img = self.cv2.imdecode(self.np.frombuffer(data, self.np.uint8), self.cv2.IMREAD_COLOR)
            if img is None:
                return
            img = self.cv2.rotate(img, {90: self.cv2.ROTATE_90_CLOCKWISE, 180: self.cv2.ROTATE_180,
                                        270: self.cv2.ROTATE_90_COUNTERCLOCKWISE}[self.rotate])
            ok, enc = self.cv2.imencode('.jpg', img, [self.cv2.IMWRITE_JPEG_QUALITY,
                                                      int(self.get_parameter('jpeg_quality').value)])
            if not ok:
                return
            data = enc.tobytes()
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = data
        self.pub.publish(msg)
        self.info.header = msg.header
        self.pub_info.publish(self.info)
        self.frames += 1
        self.last_frame = time.monotonic()

    # ----------------------------------------------------------- poll mode
    def ensure_camera_service(self):
        try:
            r = self.misty.camera_service_enabled()
            if r.ok and r.json().get('result') is False:
                self.get_logger().warning('Camera service disabled on Misty; enabling it')
                self.misty.enable_camera_service()
        except Exception as e:
            self.get_logger().warning(f'Could not check camera service: {e}')

    def poll_tick(self):
        if not self.polling:
            return
        try:
            r = self.misty.get_request('cameras/rgb', json={
                'base64': True, 'width': self.width, 'height': self.height})
            r.raise_for_status()
            ctype = r.headers.get('Content-Type', '')
            data = r.content if ctype.startswith('image/') else extract_jpeg(r.text)
            if data is None:
                body = r.json()
                raise RuntimeError(body.get('error') or str(body)[:200])
        except Exception as e:
            self.get_logger().warning(f'take_picture failed: {e}', throttle_duration_sec=5.0)
            return
        self.publish_jpeg(data, rotated=True)       # photos come upright: only stream frames need rotate_deg

    # ---------------------------------------------------------- stream mode
    def stream_loop(self):
        port = int(self.get_parameter('stream_port').value)
        warned_format = False
        while not self.stop_flag.is_set():
            ws = None
            try:
                r = self.misty.start_video_streaming(
                    port=port, rotation=int(self.get_parameter('stream_rotation').value),
                    width=self.width, height=self.height,
                    quality=int(self.get_parameter('quality').value), overlay=False)
                if not r.ok:
                    raise RuntimeError(f'videostreaming/start: HTTP {r.status_code} {r.text[:200]}')
                ws = self.connect_stream(port)       # Misty needs a moment to open the stream
                ws.settimeout(3.0)
                self.get_logger().info(f'Video stream connected (ws://{self.host}:{port})')
                bad = 0
                while not self.stop_flag.is_set():
                    frame = extract_jpeg(ws.recv())
                    if frame is None:
                        bad += 1
                        if not warned_format and bad >= 5:
                            warned_format = True
                            self.get_logger().warning('Video stream frames are not JPEG in a known '
                                                      'format; using poll fallback. Send this log to fix.')
                        if bad >= 5:
                            raise RuntimeError('unrecognised stream frame format')
                        continue
                    bad = 0
                    self.stream_warned = False
                    if self.polling:
                        self.polling = False
                        self.get_logger().info('Video stream frames arriving: polling stopped')
                    self.publish_jpeg(frame)
            except Exception as e:
                fallback = bool(self.get_parameter('poll_fallback').value)
                if not self.polling and not getattr(self, 'stream_warned', False):
                    self.get_logger().warning(f'Video stream unavailable ({e}); '
                                              + ('falling back to photos, ' if fallback else 'no image until it is back (poll_fallback: false), ')
                                              + 'retrying')
                    self.stream_warned = True
                self.polling = fallback
            finally:
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
            self.stop_flag.wait(5.0)

    def connect_stream(self, port):
        deadline = time.monotonic() + float(self.get_parameter('stream_connect_s').value)
        while True:
            try:
                return websocket.create_connection(f'ws://{self.host}:{port}', timeout=5)
            except Exception:
                if time.monotonic() > deadline or self.stop_flag.is_set():
                    raise
                time.sleep(0.5)

    # ------------------------------------------------------------ rtsp mode
    def rtsp_loop(self):
        cv2 = self.cv2
        port = int(self.get_parameter('rtsp_port').value)
        url = f'rtsp://{self.host}:{port}'
        while not self.stop_flag.is_set():
            cap = None
            try:
                self.misty.post_request('services/avstreaming/enable')
                r = self.misty.start_av_streaming(
                    url=f'rtspd:{port}', width=self.width, height=self.height,
                    frameRate=int(self.get_parameter('rtsp_fps').value),
                    videoBitRate=int(self.get_parameter('rtsp_bitrate').value))
                if not r.ok:
                    raise RuntimeError(f'avstreaming/start: HTTP {r.status_code} {r.text[:200]}')
                time.sleep(1.0)
                # low-latency FFmpeg options: TCP transport (no packet loss), no input buffering
                import os
                os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp|fflags;nobuffer|flags;low_delay'
                cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not cap.isOpened():
                    raise RuntimeError(f'cannot open {url}')
                self.get_logger().info(f'RTSP stream connected ({url})')
                q = int(self.get_parameter('jpeg_quality').value)
                while not self.stop_flag.is_set():
                    ok, img = cap.read()
                    if not ok:
                        raise RuntimeError('RTSP stream ended')
                    if self.rotate:
                        img = cv2.rotate(img, {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                                               270: cv2.ROTATE_90_COUNTERCLOCKWISE}[self.rotate])
                    ok, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
                    if ok:
                        self.publish_jpeg(enc.tobytes(), rotated=True)
            except Exception as e:
                self.get_logger().warning(f'RTSP stream unavailable ({e}); retrying', throttle_duration_sec=10.0)
            finally:
                if cap is not None:
                    cap.release()
            self.stop_flag.wait(3.0)

    def shutdown(self):
        self.stop_flag.set()
        try:
            if self.mode == 'stream':
                self.misty.post_request('videostreaming/stop')
            elif self.mode == 'rtsp':
                self.misty.post_request('avstreaming/stop')
        except Exception:
            pass


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
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
