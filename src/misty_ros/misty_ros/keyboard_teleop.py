"""Keyboard teleop for Misty: drive and aim the head from a terminal (e.g. over SSH).

    ros2 run misty_ros keyboard_teleop

Publishes the same topics as the Foxglove Teleop panels, so it needs the
teleop node running (wizard.launch.py / misty_foxglove.launch.py):
    /cmd_vel          geometry_msgs/Twist   driving
    /misty/head_cmd   geometry_msgs/Twist   head tilt / pan
    /wizard/event     std_msgs/String       HEAD_CENTER, STOP (needs the wizard node)

A terminal only reports key presses (and auto-repeat), not releases:
  - driving continues while the key repeats and stops `hold_s` after the last
    repeat (Misty needs continuous drive commands),
  - the head moves one step per key press / repeat (max `rate` per second), so
    it stops the moment the key is released.
Space stops immediately.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

HELP = r"""
Misty keyboard teleop                         (Ctrl+C to quit)
------------------------------------------------------------------
  Drive           W  forward      S  back
                  A  turn left    D  turn right
                  Q  fwd + left   E  fwd + right
  Head / camera   I or Up    tilt up       K or Down   tilt down
                  J or Left  pan left      L or Right  pan right
                  C  head centre
  Space           STOP driving, speech and sounds
  Z / X           slower / faster driving
------------------------------------------------------------------
"""

# key -> (linear.x, angular.z) for /cmd_vel
DRIVE = {'w': (1, 0), 's': (-1, 0), 'a': (0, 1), 'd': (0, -1), 'q': (1, 1), 'e': (1, -1)}
# key -> (linear.x = tilt up, angular.z = pan left) for /misty/head_cmd
HEAD = {'i': (1, 0), 'k': (-1, 0), 'j': (0, 1), 'l': (0, -1),
        '\x1b[A': (1, 0), '\x1b[B': (-1, 0), '\x1b[D': (0, 1), '\x1b[C': (0, -1)}
SPEEDS = [0.5, 0.75, 1.0, 1.5, 2.0, 3.0]   # multiplier on the teleop node's linear/angular scale


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('misty_keyboard_teleop')
        self.declare_parameter('rate', 10.0)     # Hz while a key is held
        self.declare_parameter('hold_s', 0.5)    # keep moving this long after the last key repeat
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.head = self.create_publisher(Twist, 'misty/head_cmd', 10)
        self.wizard = self.create_publisher(String, 'wizard/event', 10)
        self.speed_i = SPEEDS.index(1.0)
        self.drive_key = None
        self.drive_until = 0.0
        self.last_head = 0.0
        self.moving = False

    @staticmethod
    def twist(lin, ang, scale=1.0):
        t = Twist()
        t.linear.x = float(lin) * scale
        t.angular.z = float(ang) * scale
        return t

    def key(self, k):
        now = time.monotonic()
        hold = self.get_parameter('hold_s').value
        if k in DRIVE:
            if k != self.drive_key:
                self.drive_key = k
                self.publish_drive()          # react immediately on a new key
            self.drive_until = now + hold
        elif k in HEAD:
            # one step per key press / repeat, rate-limited; nothing after release
            if now - self.last_head >= 1.0 / self.get_parameter('rate').value:
                self.head.publish(self.twist(*HEAD[k]))
                self.last_head = now
        elif k == ' ':
            self.stop()
            self.wizard.publish(String(data='STOP'))
            self.status('STOP')
        elif k == 'c':
            self.wizard.publish(String(data='HEAD_CENTER'))
            self.status('head centre')
        elif k in ('z', 'x'):
            self.speed_i = max(0, min(len(SPEEDS) - 1, self.speed_i + (1 if k == 'x' else -1)))
            self.status(f'drive speed x{SPEEDS[self.speed_i]}')

    def publish_drive(self):
        lin, ang = DRIVE[self.drive_key]
        self.cmd.publish(self.twist(lin, ang, SPEEDS[self.speed_i]))
        self.moving = True

    def stop(self):
        self.drive_key = None
        self.drive_until = 0.0
        self.cmd.publish(Twist())
        self.moving = False

    def tick(self):
        """Called at `rate` Hz: repeat a held drive key, stop when it expires."""
        now = time.monotonic()
        if self.drive_key and now < self.drive_until:
            self.publish_drive()
        elif self.moving:
            self.drive_key = None
            self.cmd.publish(Twist())          # teleop node turns this into drive/stop
            self.moving = False

    def status(self, text):
        sys.stdout.write(f'\r\x1b[K  {text}')
        sys.stdout.flush()


def read_key(fd, timeout):
    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return None
    ch = os.read(fd, 1).decode(errors='ignore')
    if ch == '\x1b':                           # arrow keys: ESC [ A..D
        r, _, _ = select.select([fd], [], [], 0.01)
        if r:
            ch += os.read(fd, 2).decode(errors='ignore')
    return ch if ch.startswith('\x1b') else ch.lower()


def main():
    if not sys.stdin.isatty():
        sys.exit('keyboard_teleop needs an interactive terminal: run it with `ros2 run`, not in a launch file.')
    # Handle Ctrl+C ourselves so the final stop is still published.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    print(HELP)
    period = 1.0 / node.get_parameter('rate').value
    next_tick = time.monotonic()
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            k = read_key(fd, max(0.0, next_tick - time.monotonic()))
            if k == '\x03':                    # Ctrl+C in cbreak mode
                break
            if k:
                node.key(k)
            if time.monotonic() >= next_tick:
                node.tick()
                next_tick += period
                if next_tick < time.monotonic():
                    next_tick = time.monotonic() + period
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        node.stop()
        time.sleep(0.1)                        # let the stop message go out
        print('\nkeyboard teleop stopped')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
