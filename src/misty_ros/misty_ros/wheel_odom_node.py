"""Misty wheel encoders -> nav_msgs/Odometry for robot_localization.

/misty/encoders (JointState: left/right wheel distance [m]) -> /misty/wheel_odom
(differential-drive dead reckoning, frame odom -> base_link). Publishes no TF:
robot_localization's EKF fuses this with the IMU and owns odom -> base_link.
The twist (speed, turn rate) is what the EKFs use; the covariances tell them
how much to trust it.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelOdom(Node):
    def __init__(self):
        super().__init__('misty_wheel_odom')
        self.declare_parameter('wheel_base_m', 0.11)      # distance between the wheels / tracks
        self.declare_parameter('linear_std', 0.02)        # m/s
        self.declare_parameter('angular_std', 0.1)        # rad/s (tracks skid when turning)
        self.x = self.y = self.th = 0.0
        self.last = None
        self.pub = self.create_publisher(Odometry, 'misty/wheel_odom', 20)
        self.create_subscription(JointState, 'misty/encoders', self.on_encoders, 50)

    def on_encoders(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if 'left_wheel' not in pos or 'right_wheel' not in pos:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        left, right = pos['left_wheel'], pos['right_wheel']
        if self.last is None:
            self.last = (t, left, right)
            return
        t0, l0, r0 = self.last
        self.last = (t, left, right)
        dt = t - t0
        if dt <= 0.0:
            return
        dl, dr = left - l0, right - r0
        ds = (dl + dr) / 2.0
        dth = (dr - dl) / float(self.get_parameter('wheel_base_m').value)
        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))

        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        o.twist.twist.linear.x = ds / dt
        o.twist.twist.angular.z = dth / dt
        lv = float(self.get_parameter('linear_std').value) ** 2
        av = float(self.get_parameter('angular_std').value) ** 2
        o.twist.covariance = [0.0] * 36
        o.twist.covariance[0], o.twist.covariance[7], o.twist.covariance[35] = lv, 1e-4, av
        o.twist.covariance[14] = o.twist.covariance[21] = o.twist.covariance[28] = 1e-3
        o.pose.covariance = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            o.pose.covariance[i] = 1.0                    # dead-reckoned pose drifts: not fused
        self.pub.publish(o)


def main():
    rclpy.init()
    node = WheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
