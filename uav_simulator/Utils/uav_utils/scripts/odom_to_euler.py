#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations as tfs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy
from geometry_msgs.msg import Vector3Stamped


class OdomToEuler(Node):

    def __init__(self):
        super().__init__('odom_to_euler')

        self.euler_pub = self.create_publisher(Vector3Stamped, '~/euler', 10)
        self.imu_euler_pub = self.create_publisher(Vector3Stamped, '~/imueuler', 10)
        self.ctrlout_pub = self.create_publisher(Vector3Stamped, '~/ctrlout', 10)

        self.create_subscription(Odometry, '~/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '~/imu', self.imu_callback, 10)
        self.create_subscription(Joy, '~/ctrlin', self.joy_callback, 10)

    def odom_callback(self, odom_msg):
        q = np.array([odom_msg.pose.pose.orientation.x,
                      odom_msg.pose.pose.orientation.y,
                      odom_msg.pose.pose.orientation.z,
                      odom_msg.pose.pose.orientation.w])
        e = tfs.euler_from_quaternion(q, 'rzyx')

        euler_msg = Vector3Stamped()
        euler_msg.header = odom_msg.header
        euler_msg.vector.z = e[0] * 180.0 / 3.14159
        euler_msg.vector.y = e[1] * 180.0 / 3.14159
        euler_msg.vector.x = e[2] * 180.0 / 3.14159
        self.euler_pub.publish(euler_msg)

    def imu_callback(self, imu_msg):
        q = np.array([imu_msg.orientation.x,
                      imu_msg.orientation.y,
                      imu_msg.orientation.z,
                      imu_msg.orientation.w])
        e = tfs.euler_from_quaternion(q, 'rzyx')

        euler_msg = Vector3Stamped()
        euler_msg.header = imu_msg.header
        euler_msg.vector.z = e[0] * 180.0 / 3.14159
        euler_msg.vector.y = e[1] * 180.0 / 3.14159
        euler_msg.vector.x = e[2] * 180.0 / 3.14159
        self.imu_euler_pub.publish(euler_msg)

    def joy_callback(self, joy_msg):
        out_msg = Vector3Stamped()
        out_msg.header = joy_msg.header
        out_msg.vector.z = -joy_msg.axes[3]
        out_msg.vector.y = joy_msg.axes[1]
        out_msg.vector.x = joy_msg.axes[0]
        self.ctrlout_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToEuler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
