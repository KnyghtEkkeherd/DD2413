"""Manual driving inputs (teleop units) -> twist_mux inputs (TwistStamped, m/s and rad/s).

The Foxglove DRIVE panel, the Xbox controller and the keyboard publish
geometry_msgs/Twist in "teleop units": 1.0 = linear_scale / angular_scale % of
Misty's top speed. twist_mux (config/twist_mux.yaml) takes TwistStamped in real
units, one topic per source so each gets its own priority:

    /cmd_vel          (Foxglove DRIVE panel, custom nav stack) -> /cmd_vel/foxglove
    /teleop/gamepad   (gamepad_teleop)                         -> /cmd_vel/gamepad
    /teleop/keyboard  (keyboard_teleop)                        -> /cmd_vel/keyboard

Conversion: v [m/s] = x * linear_scale / 100 * max_linear_mps (same for turning
with angular_scale and max_angular_dps), i.e. exactly the speed teleop_node used
to command for the same input. Parameters are shared (/** in misty_params.yaml).
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

ROUTES = {'cmd_vel': 'cmd_vel/foxglove', 'teleop/gamepad': 'cmd_vel/gamepad', 'teleop/keyboard': 'cmd_vel/keyboard'}


class TeleopRelay(Node):
    def __init__(self):
        super().__init__('misty_teleop_relay')
        self.declare_parameter('linear_scale', 30.0)      # teleop 1.0 -> 30 % of top speed
        self.declare_parameter('angular_scale', 70.0)
        self.declare_parameter('max_linear_mps', 0.6)     # Misty's speed at 100 %
        self.declare_parameter('max_angular_dps', 120.0)  # Misty's turn rate at 100 %
        for src, dst in ROUTES.items():
            pub = self.create_publisher(TwistStamped, dst, 10)
            self.create_subscription(Twist, src, lambda m, p=pub: self.relay(m, p), 10)

    def relay(self, msg: Twist, pub):
        g = lambda n: float(self.get_parameter(n).value)
        lin = max(-100.0, min(100.0, msg.linear.x * g('linear_scale')))
        ang = max(-100.0, min(100.0, msg.angular.z * g('angular_scale')))
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = lin / 100.0 * g('max_linear_mps')
        out.twist.angular.z = math.radians(ang / 100.0 * g('max_angular_dps'))
        pub.publish(out)


def main():
    rclpy.init()
    node = TeleopRelay()
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
