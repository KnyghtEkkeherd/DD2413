"""Drive Misty and aim its head (camera) from geometry_msgs/Twist.

/cmd_vel        linear.x -> forward/back, angular.z -> turn.
                Sent as drive_time so Misty stops by itself after
                drive_time_ms if commands stop arriving (lost connection,
                released button). Velocities are % of max (-100..100).
/misty/head_cmd linear.x -> tilt up(+)/down(-), angular.z -> pan left(+)/right(-).
                Each message nudges the head by head_step_deg.

These match the Foxglove Teleop panel's default button mapping, so two Teleop
panels (one per topic) work without reconfiguring the buttons.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from misty_ros.sdk import make_robot

# Misty II head limits in degrees (pitch: negative = up)
PITCH_MIN, PITCH_MAX = -40.0, 26.0
YAW_MIN, YAW_MAX = -81.0, 81.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MistyTeleop(Node):
    def __init__(self):
        super().__init__('misty_teleop')
        self.declare_parameter('misty_ip', '')
        self.declare_parameter('linear_scale', 30.0)    # Twist 1.0 -> 30% speed
        self.declare_parameter('angular_scale', 40.0)
        self.declare_parameter('drive_time_ms', 600)
        self.declare_parameter('head_step_deg', 4.0)
        self.declare_parameter('head_velocity', 60.0)   # % of max head speed
        self.declare_parameter('request_timeout', 1.0)

        ip = self.get_parameter('misty_ip').value
        if not ip:
            raise RuntimeError('Parameter misty_ip is required (or set MISTY_IP for the launch file).')
        self.misty = make_robot(ip, float(self.get_parameter('request_timeout').value))
        self.moving = False
        self.pitch = 0.0
        self.yaw = 0.0

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 1)
        self.create_subscription(Twist, 'misty/head_cmd', self.on_head_cmd, 1)
        self.get_logger().info(f'Teleop for Misty {ip}: /cmd_vel drives, /misty/head_cmd aims head')

    def p(self, name):
        return self.get_parameter(name).value

    def _call(self, what, fn, *args, **kwargs):
        try:
            r = fn(*args, **kwargs)
            if not r.ok:
                self.get_logger().warning(f'{what}: HTTP {r.status_code} {r.text[:200]}',
                                       throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().warning(f'{what} failed: {e}', throttle_duration_sec=2.0)

    def on_cmd_vel(self, msg: Twist):
        lin = clamp(msg.linear.x * self.p('linear_scale'), -100.0, 100.0)
        ang = clamp(msg.angular.z * self.p('angular_scale'), -100.0, 100.0)
        if lin == 0.0 and ang == 0.0:
            if self.moving:
                self._call('stop', self.misty.stop)
                self.moving = False
            return
        self._call('drive_time', self.misty.drive_time,
                   linearVelocity=lin, angularVelocity=ang,
                   timeMs=int(self.p('drive_time_ms')))
        self.moving = True

    def on_head_cmd(self, msg: Twist):
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            return
        step = self.p('head_step_deg')
        self.pitch = clamp(self.pitch - msg.linear.x * step, PITCH_MIN, PITCH_MAX)
        self.yaw = clamp(self.yaw + msg.angular.z * step, YAW_MIN, YAW_MAX)
        self._call('move_head', self.misty.move_head,
                   pitch=self.pitch, roll=0.0, yaw=self.yaw,
                   velocity=self.p('head_velocity'), units='degrees')

    def stop(self):
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
