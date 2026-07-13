#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf_transformations as tfs
from nav_msgs.msg import Odometry


class OdomSender(Node):

    def __init__(self):
        super().__init__('odom_sender')
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.counter = 0

        self.msg = Odometry()
        self.msg.header.frame_id = 'world'
        q = tfs.quaternion_from_euler(0, 0, 0, 'rzyx')
        self.msg.pose.pose.orientation.x = q[0]
        self.msg.pose.pose.orientation.y = q[1]
        self.msg.pose.pose.orientation.z = q[2]
        self.msg.pose.pose.orientation.w = q[3]

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.counter += 1
        self.msg.header.stamp = (self.get_clock().now() - Duration(seconds=0.2)).to_msg()
        self.pub.publish(self.msg)
        self.get_logger().info('Send %3d msg(s).' % self.counter)


def main(args=None):
    rclpy.init(args=args)
    node = OdomSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
