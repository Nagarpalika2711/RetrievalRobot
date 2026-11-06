#!/usr/bin/env python3
"""Publish one startup goal after map, TF, and subscribers are ready."""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException


class StartupGoal(Node):
    def __init__(self):
        super().__init__('startup_goal')
        for key, value in [('goal_x', 2.5), ('goal_y', 0.5), ('goal_yaw', 0.0)]:
            self.declare_parameter(key, value)
        self.map_received = False
        self.sent = False
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map',
                                                self.map_callback, map_qos)
        self.pub = self.create_publisher(PoseStamped, '/get_task', 10)
        self.timer = self.create_timer(0.5, self.try_send)
        self.get_logger().info('Startup goal waiting for /map, map -> base_link, and both /get_task subscribers')

    def map_callback(self, msg):
        self.map_received = bool(msg.info.width and msg.info.height)

    def try_send(self):
        if self.sent or not self.map_received:
            return
        # Both planner and navigator subscribe to /get_task. Do not lose the initial task.
        if self.pub.get_subscription_count() < 2:
            return
        if self.get_clock().now().nanoseconds <= 0:
            return
        try:
            self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return

        x = float(self.get_parameter('goal_x').value)
        y = float(self.get_parameter('goal_y').value)
        heading = float(self.get_parameter('goal_yaw').value)
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(heading / 2)
        msg.pose.orientation.w = math.cos(heading / 2)
        self.pub.publish(msg)
        self.sent = True
        self.timer.cancel()
        self.get_logger().info(f'Published startup /get_task goal: x={x:.3f}, y={y:.3f}, yaw={heading:.3f} rad')


def main():
    rclpy.init()
    node = StartupGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
