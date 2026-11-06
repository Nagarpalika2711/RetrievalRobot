#!/usr/bin/env python3
"""Differential-drive wheel odometry for the retrieval robot."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SimpleOdometry(Node):
    def __init__(self):
        super().__init__('simple_odometry')
        self.declare_parameter('wheel_radius', 0.1132)
        self.declare_parameter('wheel_separation', 0.76)
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.radius = float(self.get_parameter('wheel_radius').value)
        self.separation = float(self.get_parameter('wheel_separation').value)
        self.left_joint = str(self.get_parameter('left_joint').value)
        self.right_joint = str(self.get_parameter('right_joint').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        if self.radius <= 0 or self.separation <= 0:
            raise ValueError('wheel_radius and wheel_separation must be positive')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.previous_left = None
        self.previous_right = None
        self.previous_time = None
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 20)
        self.get_logger().info(
            f'Odometry started: left={self.left_joint}, right={self.right_joint}, '
            f'r={self.radius:.4f} m, L={self.separation:.4f} m')

    def joint_callback(self, msg):
        try:
            left_index = msg.name.index(self.left_joint)
            right_index = msg.name.index(self.right_joint)
        except ValueError:
            self.get_logger().warn('Wheel joints not found in /joint_states', throttle_duration_sec=5.0)
            return
        if len(msg.position) <= max(left_index, right_index):
            self.get_logger().warn('Wheel joint positions are missing', throttle_duration_sec=5.0)
            return

        left = float(msg.position[left_index])
        right = float(msg.position[right_index])
        if not (math.isfinite(left) and math.isfinite(right)):
            return
        stamp = msg.header.stamp
        # Some publishers leave JointState stamps empty; use the node clock then.
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        t = stamp.sec + stamp.nanosec * 1e-9

        if self.previous_time is None or t <= self.previous_time:
            # On a backwards simulation clock jump, reinitialize increments.
            if self.previous_time is not None and t < self.previous_time:
                self.get_logger().warn('Clock moved backwards; resetting wheel baseline')
            self.previous_left, self.previous_right, self.previous_time = left, right, t
            return

        dt = t - self.previous_time
        delta_left = self.radius * (left - self.previous_left)
        delta_right = self.radius * (right - self.previous_right)
        self.previous_left, self.previous_right, self.previous_time = left, right, t

        ds = (delta_left + delta_right) / 2.0
        dtheta = (delta_right - delta_left) / self.separation
        heading_mid = self.yaw + dtheta / 2.0
        self.x += ds * math.cos(heading_mid)
        self.y += ds * math.sin(heading_mid)
        self.yaw = math.atan2(math.sin(self.yaw + dtheta), math.cos(self.yaw + dtheta))
        qx, qy, qz, qw = yaw_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = ds / dt
        odom.twist.twist.angular.z = dtheta / dt
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
