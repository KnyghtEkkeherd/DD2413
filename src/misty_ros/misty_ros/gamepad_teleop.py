"""Xbox controller teleop for Misty.

Receives controller state as JSON over UDP from scripts/gamepad_sender.py
(running on the computer the controller is plugged into) and publishes the
same topics as the Foxglove Teleop panels:

    left stick    drive    -> /cmd_vel          (up = forward, left = turn left)
    right stick   head     -> /misty/head_cmd   (up = tilt up, left = pan left);
                                                 speed scales with stick deflection
    B             STOP driving, speech, sounds  -> /wizard/event STOP
                  (driving stays off until the left stick is released)
    A / Y         head centre                   -> /misty/head_goal (0, 0, 0)
    RT / LT       right / left arm: released = down, fully pressed = up,
                  in between = in between       -> /misty/arms_cmd
    LB / RB       slower / faster driving

Also republishes the raw state as sensor_msgs/Joy on /joy. If no packet
arrives for `timeout_s` (sender stopped, network drop), driving stops.
Needs the teleop node (and the wizard node for B / Y) running.
"""
import json
import socket

import rclpy
import math

from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String



def shape(v, deadzone):
    """Apply a deadzone and rescale so motion starts smoothly at its edge."""
    if abs(v) < deadzone:
        return 0.0
    return (abs(v) - deadzone) / (1.0 - deadzone) * (1 if v > 0 else -1)


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__('misty_gamepad_teleop')
        self.declare_parameter('port', 9876)
        self.declare_parameter('rate', 15.0)          # Hz for /cmd_vel and /misty/head_cmd
        self.declare_parameter('head_gain', 0.75)     # head Twist at full deflection (x head_step_deg)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('timeout_s', 0.5)
        self.declare_parameter('drive_turn_scale', 1.0)   # left stick X -> turn
        self.declare_parameter('speed_levels', [0.5, 0.75, 1.0, 1.5, 2.0, 3.0])  # LB / RB
        self.declare_parameter('start_speed_level', 2)
        self.declare_parameter('invert_drive_turn', False)
        self.declare_parameter('invert_head_tilt', False)
        self.declare_parameter('invert_head_pan', False)
        self.declare_parameter('arm_down_deg', 90.0)       # arm angle with the trigger released
        self.declare_parameter('arm_up_deg', -29.0)        # arm angle with the trigger fully pressed
        self.declare_parameter('arm_min_change_deg', 3.0)  # only send when an arm moves this much
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.head = self.create_publisher(Twist, 'misty/head_cmd', 10)
        self.joy = self.create_publisher(Joy, 'joy', 10)
        self.wizard = self.create_publisher(String, 'wizard/event', 10)
        self.head_goal = self.create_publisher(Vector3, 'misty/head_goal', 10)
        self.arms = self.create_publisher(Vector3, 'misty/arms_cmd', 10)
        self.arm_sent = None                    # (left, right) degrees last sent

        port = int(self.get_parameter('port').value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)

        self.state = None
        self.last_rx = None
        self.prev_buttons = {}
        self.speeds = [float(v) for v in self.get_parameter('speed_levels').value]
        self.speed_i = max(0, min(len(self.speeds) - 1, int(self.get_parameter('start_speed_level').value)))
        self.driving = False
        self.latched = False        # B pressed: no driving until stick released
        self.connected = False
        self.create_timer(0.01, self.receive)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.tick)
        self.get_logger().info(f'Waiting for controller packets on UDP port {port} '
                               '(run scripts/gamepad_sender.py on the computer with the controller)')

    def receive(self):
        latest = None
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                latest = json.loads(data)
            except ValueError:
                continue
            if not self.connected:
                self.connected = True
                self.arm_sent = None
                self.get_logger().info(f'Controller connected from {addr[0]}: {latest.get("name", "?")}')
        if latest is None:
            return
        self.state = latest
        self.last_rx = self.get_clock().now()
        self.publish_joy(latest)
        self.handle_buttons(latest)

    def publish_joy(self, s):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gamepad'
        msg.axes = [float(s.get(k, 0.0)) for k in ('lx', 'ly', 'rx', 'ry', 'lt', 'rt')]
        msg.buttons = [int(s.get(k, 0)) for k in ('a', 'b', 'x', 'y', 'lb', 'rb', 'back', 'start')]
        self.joy.publish(msg)

    def handle_buttons(self, s):
        def pressed(name):           # rising edge only
            return s.get(name) and not self.prev_buttons.get(name)
        if pressed('b'):
            self.cmd.publish(Twist())
            self.driving = False
            self.latched = True
            self.wizard.publish(String(data='STOP'))
            self.get_logger().info('B: STOP')
        if pressed('a') or pressed('y'):
            self.head_goal.publish(Vector3(x=0.0, y=0.0, z=0.0))
            self.get_logger().info('Head centre')
        if pressed('lb') or pressed('rb'):
            self.speed_i = max(0, min(len(self.speeds) - 1, self.speed_i + (1 if pressed('rb') else -1)))
            self.get_logger().info(f'Drive speed x{self.speeds[self.speed_i]}')
        self.prev_buttons = {k: s.get(k, 0) for k in ('a', 'b', 'x', 'y', 'lb', 'rb', 'back', 'start')}

    def tick(self):
        stale = (self.last_rx is None or
                 (self.get_clock().now() - self.last_rx).nanoseconds / 1e9 > self.get_parameter('timeout_s').value)
        if stale:
            if self.connected and self.last_rx is not None:
                self.get_logger().warning('Controller packets stopped: stopping')
                self.connected = False
            if self.driving:
                self.cmd.publish(Twist())
                self.driving = False
            return
        s, dz = self.state, float(self.get_parameter('deadzone').value)
        # SDL: stick up / left are negative, so invert for ROS (forward / left positive)
        fwd = -shape(float(s.get('ly', 0.0)), dz)
        turn = -shape(float(s.get('lx', 0.0)), dz) * float(self.get_parameter('drive_turn_scale').value)
        if self.get_parameter('invert_drive_turn').value:
            turn = -turn
        if self.latched:
            if fwd or turn:
                fwd = turn = 0.0               # still held after B: ignore
            else:
                self.latched = False           # stick released: driving allowed again
        if fwd or turn:
            t = Twist()
            t.linear.x = fwd * self.speeds[self.speed_i]
            t.angular.z = turn * self.speeds[self.speed_i]
            self.cmd.publish(t)
            self.driving = True
        elif self.driving:
            self.cmd.publish(Twist())          # stick released -> teleop node sends drive/stop
            self.driving = False
        self.update_arms(s)
        tilt = -shape(float(s.get('ry', 0.0)), dz)
        pan = -shape(float(s.get('rx', 0.0)), dz)
        if self.get_parameter('invert_head_tilt').value:
            tilt = -tilt
        if self.get_parameter('invert_head_pan').value:
            pan = -pan
        if tilt or pan:
            h = Twist()
            gain = float(self.get_parameter('head_gain').value)
            h.linear.x = tilt * gain           # teleop node: x head_step_deg per message
            h.angular.z = pan * gain
            self.head.publish(h)


    def update_arms(self, s):
        """Triggers (0 = released .. 1 = fully pressed) -> arm angles."""
        down = float(self.get_parameter('arm_down_deg').value)
        up = float(self.get_parameter('arm_up_deg').value)
        def angle(t):
            t = max(0.0, min(1.0, float(t)))
            return down + t * (up - down)
        target = (angle(s.get('lt', 0.0)), angle(s.get('rt', 0.0)))
        if self.arm_sent is None:              # just connected: don't move the arms yet
            self.arm_sent = target
            return
        step = float(self.get_parameter('arm_min_change_deg').value)
        changed = [abs(t - a) >= step or (t != a and t in (up, down)) for t, a in zip(target, self.arm_sent)]
        if not any(changed):
            return
        left = target[0] if changed[0] else math.nan
        right = target[1] if changed[1] else math.nan
        self.arms.publish(Vector3(x=left, y=right, z=0.0))
        self.arm_sent = tuple(t if c else a for t, c, a in zip(target, changed, self.arm_sent))


def main():
    rclpy.init()
    node = None
    try:
        node = GamepadTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.cmd.publish(Twist())
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
