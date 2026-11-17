"""Drive Misty and aim its head (camera) from geometry_msgs/Twist.

/cmd_vel        linear.x -> forward/back, angular.z -> turn.
                Sent as drive_time so Misty stops by itself after
                drive_time_ms if commands stop arriving (lost connection,
                released button). Velocities are % of max (-100..100).
/misty/head_cmd linear.x -> tilt up(+)/down(-), angular.z -> pan left(+)/right(-).
                Each message nudges the head by head_step_deg.
/misty/head_goal (geometry_msgs/Vector3, degrees: x=pitch, z=yaw)
                Move the head to this absolute position (e.g. 0,0,0 = centre).
/misty/arms_cmd (geometry_msgs/Vector3, degrees: x=left arm, y=right arm)
                Move the arms; NaN leaves that arm where it is.
/misty/head_position (geometry_msgs/Vector3, degrees: x=pitch, y=roll, z=yaw)
                Other nodes (the wizard) publish head moves they make here, so
                the next head_cmd nudges from where the head really is.

Low latency: drive, head and arms each have their own sender thread, so a slow
reply from Misty for one never delays the others, and each sender always sends
the NEWEST command (older ones that piled up while waiting are skipped).

These match the Foxglove Teleop panel's default button mapping, so two Teleop
panels (one per topic) work without reconfiguring the buttons.
"""
import math
import threading

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node

from misty_ros.sdk import make_robot

# Misty II head limits in degrees (pitch: negative = up)
PITCH_MIN, PITCH_MAX = -40.0, 26.0
YAW_MIN, YAW_MAX = -81.0, 81.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class LatestSender:
    """Background sender that only ever sends the most recent command.

    put() replaces any command still waiting (optionally merging with it), so
    commands never queue up behind a slow robot.
    """

    def __init__(self, name, send, merge=None):
        self.send, self.merge = send, merge
        self.cond = threading.Condition()
        self.item = None
        self.stopped = False
        threading.Thread(target=self.run, name=name, daemon=True).start()

    def put(self, item):
        with self.cond:
            if self.merge is not None and self.item is not None:
                item = self.merge(self.item, item)
            self.item = item
            self.cond.notify()

    def run(self):
        while True:
            with self.cond:
                while self.item is None and not self.stopped:
                    self.cond.wait()
                if self.stopped:
                    return
                item, self.item = self.item, None
            self.send(item)

    def close(self):
        with self.cond:
            self.stopped = True
            self.cond.notify()


class MistyTeleop(Node):
    def __init__(self):
        super().__init__('misty_teleop')
        self.declare_parameter('misty_ip', '')
        self.declare_parameter('linear_scale', 30.0)    # Twist 1.0 -> 30% speed
        self.declare_parameter('angular_scale', 70.0)
        self.declare_parameter('drive_time_ms', 600)
        self.declare_parameter('head_step_deg', 13.0)
        self.declare_parameter('head_velocity', 100.0)  # % of max head speed
        self.declare_parameter('arm_velocity', 80.0)    # % of max arm speed
        self.declare_parameter('request_timeout', 1.0)

        ip = self.get_parameter('misty_ip').value
        if not ip:
            raise RuntimeError('Parameter misty_ip is required (or set MISTY_IP for the launch file).')
        self.misty = make_robot(ip, float(self.get_parameter('request_timeout').value))
        self.moving = False
        self.pitch = 0.0
        self.yaw = 0.0

        self.drive_tx = LatestSender('drive', self.send_drive)
        self.head_tx = LatestSender('head', self.send_head)
        self.arms_tx = LatestSender('arms', self.send_arms, merge=self.merge_arms)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Twist, 'misty/head_cmd', self.on_head_cmd, 10)
        self.create_subscription(Vector3, 'misty/head_position', self.on_head_position, 10)
        self.create_subscription(Vector3, 'misty/head_goal', self.on_head_goal, 10)
        self.create_subscription(Vector3, 'misty/arms_cmd', self.on_arms_cmd, 10)
        self.get_logger().info(f'Teleop for Misty {ip}: /cmd_vel drives, /misty/head_cmd aims head')

    def p(self, name):
        return self.get_parameter(name).value

    def _call(self, what, fn, **kwargs):
        try:
            r = fn(**kwargs)
            if not r.ok:
                self.get_logger().warning(f'{what}: HTTP {r.status_code} {r.text[:200]}',
                                          throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().warning(f'{what} failed: {e}', throttle_duration_sec=2.0)

    # ------------------------------------------------------------ callbacks
    # Callbacks only compute the target and hand it to a sender: no network here.
    def on_cmd_vel(self, msg: Twist):
        lin = clamp(msg.linear.x * self.p('linear_scale'), -100.0, 100.0)
        ang = clamp(msg.angular.z * self.p('angular_scale'), -100.0, 100.0)
        if lin == 0.0 and ang == 0.0:
            if self.moving:
                self.drive_tx.put(('stop',))
                self.moving = False
            return
        self.drive_tx.put(('drive', lin, ang, int(self.p('drive_time_ms'))))
        self.moving = True

    def on_head_cmd(self, msg: Twist):
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            return
        step = self.p('head_step_deg')
        self.pitch = clamp(self.pitch - msg.linear.x * step, PITCH_MIN, PITCH_MAX)
        self.yaw = clamp(self.yaw + msg.angular.z * step, YAW_MIN, YAW_MAX)
        self.head_tx.put((self.pitch, self.yaw))

    def on_head_goal(self, msg: Vector3):
        self.pitch = clamp(msg.x, PITCH_MIN, PITCH_MAX)
        self.yaw = clamp(msg.z, YAW_MIN, YAW_MAX)
        self.head_tx.put((self.pitch, self.yaw))

    def on_head_position(self, msg: Vector3):
        self.pitch = clamp(msg.x, PITCH_MIN, PITCH_MAX)
        self.yaw = clamp(msg.z, YAW_MIN, YAW_MAX)

    def on_arms_cmd(self, msg: Vector3):
        if math.isnan(msg.x) and math.isnan(msg.y):
            return
        self.arms_tx.put((msg.x, msg.y))

    @staticmethod
    def merge_arms(old, new):
        # keep an arm's waiting target if the newer command doesn't move that arm
        return tuple(o if math.isnan(n) else n for o, n in zip(old, new))

    # -------------------------------------------------------------- senders
    def send_drive(self, cmd):
        if cmd[0] == 'stop':
            self._call('stop', self.misty.stop)
        else:
            _, lin, ang, ms = cmd
            self._call('drive_time', self.misty.drive_time,
                       linearVelocity=lin, angularVelocity=ang, timeMs=ms)

    def send_head(self, target):
        pitch, yaw = target
        self._call('move_head', self.misty.move_head, pitch=pitch, roll=0.0, yaw=yaw,
                   velocity=self.p('head_velocity'), units='degrees')

    def send_arms(self, target):
        left, right = target
        vel = self.p('arm_velocity')
        if math.isnan(left) or math.isnan(right):
            arm, pos = ('right', right) if math.isnan(left) else ('left', left)
            self._call('move_arm', self.misty.move_arm, arm=arm, position=float(pos),
                       velocity=vel, units='degrees')
        else:
            self._call('move_arms', self.misty.move_arms, leftArmPosition=float(left),
                       rightArmPosition=float(right), leftArmVelocity=vel, rightArmVelocity=vel,
                       units='degrees')

    def stop(self):
        for tx in (self.drive_tx, self.head_tx, self.arms_tx):
            tx.close()
        self._call('stop', self.misty.stop)


def main():
    rclpy.init()
    node = None
    try:
        node = MistyTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
