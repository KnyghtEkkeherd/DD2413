"""Optional, default-OFF Misty II face-directed head tracking for the wizard.

Foxglove /wizard/event String values:
  FACE_TRACK_ON     Subscribe to Misty's built-in FaceRecognition event and
                    start anonymous face detection; nudge the existing head
                    teleop node toward detected faces.
  FACE_TRACK_OFF    Stop tracking and, if started here, face detection.

STOP, HEAD_CENTER, LOOK_BACK and END also turn tracking off so the wizard's
head actions take precedence. No images, identities, or biometric profiles
are stored. The feature is *face tracking*, not enrolled-face identification.

Offline test: launch with -p dry_run:=true, then publish synthetic bearings to
/misty/face_tracking/test_face (geometry_msgs/Vector3: x=bearing, y=elevation).
This mode never contacts the robot; it only publishes /misty/head_cmd.
"""
import json
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from misty_ros.face_tracking_math import compute_head_command, parse_face_event

try:
    import requests
    import websocket  # Ubuntu: python3-websocket (websocket-client)
except ImportError:
    requests = None
    websocket = None

EVENT_NAME = 'misty_face_tracker'


class MistyFaceTracker(Node):
    def __init__(self):
        super().__init__('misty_face_tracker')
        self.declare_parameter('misty_ip', '')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('deadband_deg', 5.0)
        self.declare_parameter('gain', 0.55)
        self.declare_parameter('max_command', 0.60)
        self.declare_parameter('head_step_deg', 13.0)
        self.declare_parameter('event_debounce_ms', 150)
        self.declare_parameter('control_period_s', 0.10)
        self.declare_parameter('enable_settle_s', 1.5)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('max_face_age_s', 1.5)
        self.declare_parameter('request_timeout', 3.0)
        self.declare_parameter('reconnect_delay_s', 2.0)
        # mode: eyes = the head stays still, Misty's eyes look towards the face (eye images);
        #       head = the head turns towards the face (/misty/head_cmd)
        self.declare_parameter('mode', 'eyes')
        self.declare_parameter('start_enabled', True)        # track from launch on (before the first
                                                             # drive); the follower switches it off
                                                             # while driving and on at each marker
        self.declare_parameter('eyes_center', 'e_DefaultContent.jpg')
        self.declare_parameter('eyes_left', 'e_ContentLeft.jpg')     # face on Misty's left
        self.declare_parameter('eyes_right', 'e_ContentRight.jpg')   # face on Misty's right
        self.declare_parameter('eyes_threshold_deg', 8.0)    # face further off-centre: look that way
        self.declare_parameter('eyes_hysteresis_deg', 3.0)   # look back to centre below threshold - this
        self.declare_parameter('eyes_return_s', 2.0)         # face lost this long: eyes back to centre

        self.ip = str(self.get_parameter('misty_ip').value).strip()
        self.dry_run = bool(self.get_parameter('dry_run').value)
        if not self.dry_run and not self.ip:
            raise ValueError('misty_ip must be set unless dry_run:=true')
        if not self.dry_run and (websocket is None or requests is None):
            raise RuntimeError('Install python3-websocket and python3-requests')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            String, '/misty/face_tracking/status', latched)
        self.head_pub = self.create_publisher(Twist, '/misty/head_cmd', 10)
        self.create_subscription(String, '/wizard/event', self.on_event, 10)
        self.create_subscription(
            Vector3, '/misty/face_tracking/test_face', self.on_test_face, 10)

        self.lock = threading.Lock()
        self.enabled = False
        self.enabled_at = 0.0  # wait for wizard to aim the head before tracking
        self.generation = 0  # invalidate an old websocket session on OFF/ON
        self.connection_state = 'OFF'
        self.face = None  # (bearing, elevation, timestamp, sequence)
        self.face_seq = 0
        self.sent_seq = 0
        self.shutdown_flag = threading.Event()
        self.last_status = None
        self.last_nudge_time = 0.0
        self.eyes_side = None                # 'left' | 'center' | 'right' currently shown (None = untouched)
        self.eyes_busy = threading.Lock()
        if bool(self.param('start_enabled')):
            self.enabled, self.enabled_at, self.generation = True, time.monotonic(), 1
            self.connection_state = 'DRY RUN' if self.dry_run else 'connecting'
        self.create_timer(max(0.05, float(self.param('control_period_s'))), self.control_tick)
        self.create_timer(1.0, self.publish_status)
        self.publish_status()

        self.worker = None
        if not self.dry_run:
            self.worker = threading.Thread(target=self.websocket_loop, daemon=True)
            self.worker.start()
        self.get_logger().info('Face tracker ready (%s%s)' % (
            'ON from the start' if self.enabled else 'OFF until FACE_TRACK_ON',
            ', DRY RUN' if self.dry_run else ''))

    def param(self, key):
        return self.get_parameter(key).value

    def on_event(self, msg: String):
        event = msg.data.strip().split(':', 1)[0].strip().upper()
        if event == 'FACE_TRACK_ON':
            with self.lock:
                if not self.enabled:
                    self.generation += 1
                    self.enabled = True
                    self.enabled_at = time.monotonic()
                    self.face = None
                    self.connection_state = 'DRY RUN' if self.dry_run else 'connecting'
            self.publish_status()
        elif event in ('FACE_TRACK_OFF', 'STOP', 'HEAD_CENTER', 'LOOK_BACK', 'END'):
            with self.lock:
                if self.enabled:
                    self.enabled = False
                    self.generation += 1
                    self.face = None
                    self.connection_state = 'OFF'
            self.publish_status()

    def on_test_face(self, msg: Vector3):
        if not self.dry_run:
            return
        self.record_face(float(msg.x), float(msg.y))

    def record_face(self, bearing, elevation):
        with self.lock:
            if not self.enabled:
                return
            self.face_seq += 1
            self.face = (bearing, elevation, time.monotonic(), self.face_seq)

    def current_session(self, generation):
        with self.lock:
            return self.enabled and self.generation == generation

    def set_connection_state(self, generation, state):
        with self.lock:
            if self.enabled and self.generation == generation:
                self.connection_state = state

    def websocket_loop(self):
        """Non-ROS thread: keep all blocking HTTP/WebSocket calls off callbacks."""
        while not self.shutdown_flag.is_set():
            with self.lock:
                enabled, generation = self.enabled, self.generation
            if not enabled:
                self.shutdown_flag.wait(0.2)
                continue

            ws = None
            detection_started = False
            try:
                self.set_connection_state(generation, 'connecting')
                ws = websocket.create_connection(
                    f'ws://{self.ip}/pubsub', timeout=float(self.param('request_timeout')))
                ws.send(json.dumps({
                    'Operation': 'subscribe', 'Type': 'FaceRecognition',
                    'DebounceMs': max(50, int(self.param('event_debounce_ms'))),
                    'EventName': EVENT_NAME, 'Message': ''}))
                if not self.current_session(generation):
                    continue

                response = requests.post(
                    f'http://{self.ip}/api/faces/detection/start',
                    timeout=float(self.param('request_timeout')))
                response.raise_for_status()
                try:
                    result = response.json()
                    if isinstance(result, dict) and result.get('result') is False:
                        raise RuntimeError('Misty refused to start face detection')
                except ValueError:
                    pass
                detection_started = True
                self.set_connection_state(generation, 'connected')
                ws.settimeout(0.75)

                while self.current_session(generation) and not self.shutdown_flag.is_set():
                    try:
                        raw = ws.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    face = parse_face_event(raw, EVENT_NAME)
                    if face is not None and self.current_session(generation):
                        self.record_face(*face)
            except Exception as exc:
                if self.current_session(generation):
                    self.get_logger().warning(f'Face detection connection: {exc}')
                    self.set_connection_state(generation, f'error: {str(exc)[:70]}')
                    self.shutdown_flag.wait(float(self.param('reconnect_delay_s')))
            finally:
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
                if detection_started:
                    try:
                        requests.post(
                            f'http://{self.ip}/api/faces/detection/stop',
                            timeout=float(self.param('request_timeout'))).raise_for_status()
                    except requests.RequestException as exc:
                        self.get_logger().warning(f'Could not stop face detection: {exc}')
                if self.current_session(generation):
                    self.set_connection_state(generation, 'connecting')

    def control_tick(self):
        if str(self.param('mode')).lower() == 'eyes':
            return self.eyes_tick()
        with self.lock:
            if not self.enabled:
                return
            face, state = self.face, self.connection_state
            # Wizard is moving to the start pose; don't fight that command.
            if time.monotonic() - self.enabled_at < float(self.param('enable_settle_s')):
                return
            if not self.dry_run and state != 'connected':
                return
            if face is None or face[3] == self.sent_seq:
                return
            bearing, elevation, timestamp, sequence = face
            self.sent_seq = sequence  # never move twice for one face event
        if time.monotonic() - timestamp > float(self.param('max_face_age_s')):
            return
        linear, angular = compute_head_command(
            bearing, elevation,
            deadband_deg=float(self.param('deadband_deg')),
            gain=float(self.param('gain')),
            head_step_deg=float(self.param('head_step_deg')),
            max_command=float(self.param('max_command')),
            yaw_sign=float(self.param('yaw_sign')),
            pitch_sign=float(self.param('pitch_sign')))
        if linear or angular:
            with self.lock:
                if not self.enabled:
                    return
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.head_pub.publish(cmd)

    # ------------------------------------------------------------- eyes mode
    def eyes_tick(self):
        with self.lock:
            enabled, face, state = self.enabled, self.face, self.connection_state
        if not enabled:
            if self.eyes_side not in (None, 'center'):
                self.show_eyes('center')      # tracking switched off: look straight again
            return
        if not self.dry_run and state != 'connected':
            return
        now = time.monotonic()
        if face is None or now - face[2] > float(self.param('eyes_return_s')):
            if self.eyes_side not in (None, 'center'):
                self.show_eyes('center')
            return
        # yaw_sign * bearing > 0 = face on Misty's left (same convention as head mode)
        b = float(self.param('yaw_sign')) * face[0]
        th, hyst = float(self.param('eyes_threshold_deg')), float(self.param('eyes_hysteresis_deg'))
        side = self.eyes_side or 'center'
        if b > th:
            side = 'left'
        elif b < -th:
            side = 'right'
        elif abs(b) < th - hyst:
            side = 'center'
        if side != self.eyes_side:
            self.show_eyes(side)

    def show_eyes(self, side):
        self.eyes_side = side
        image = str(self.param(f'eyes_{side}'))
        if self.dry_run:
            self.get_logger().info(f'(dry run) eyes {side}: {image}')
            return

        def send():                           # HTTP off the ROS callback thread
            if not self.eyes_busy.acquire(blocking=False):
                return                        # one at a time; the next tick catches up
            try:
                requests.post(f'http://{self.ip}/api/images/display', json={'FileName': image, 'Alpha': 1},
                              timeout=float(self.param('request_timeout')))
            except Exception as exc:
                self.get_logger().warning(f'eyes {side}: {exc}', throttle_duration_sec=10.0)
            finally:
                self.eyes_busy.release()
        threading.Thread(target=send, daemon=True).start()

    def publish_status(self):
        with self.lock:
            enabled, state, face = self.enabled, self.connection_state, self.face
        if not enabled:
            status = 'OFF | head control is manual'
        elif state not in ('connected', 'DRY RUN'):
            status = f'ON | {state}'
        elif face is None or time.monotonic() - face[2] > float(self.param('max_face_age_s')):
            status = ('DRY RUN' if self.dry_run else 'ON') + ' | searching for face'
        else:
            status = ('DRY RUN' if self.dry_run else 'ON') + (
                f' | tracking face: bearing={face[0]:+.1f}°, elevation={face[1]:+.1f}°')
        if status != self.last_status:
            self.status_pub.publish(String(data=status))
            self.last_status = status

    def close(self):
        with self.lock:
            self.enabled = False
            self.generation += 1
        self.shutdown_flag.set()
        if self.worker is not None:
            self.worker.join(timeout=float(self.param('request_timeout')) + 2)


def main():
    rclpy.init()
    node = None
    try:
        node = MistyFaceTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
