#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations as tfs
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped


class OdometryConverter(object):

    def __init__(self, node, br, frame_id_in, frame_id_out, broadcast_tf,
                 body_frame_id, intermediate_frame_id, world_frame_id):
        self.node = node
        self.br = br
        self.frame_id_in = frame_id_in
        self.frame_id_out = frame_id_out
        self.broadcast_tf = broadcast_tf
        self.body_frame_id = body_frame_id
        self.intermediate_frame_id = intermediate_frame_id
        self.world_frame_id = world_frame_id
        self.out_odom_pub = None
        self.out_path_pub = None
        self.tf_pub_flag = True

        if self.broadcast_tf:
            node.get_logger().info(
                'ROSTopic: [%s]->[%s] TF: [%s]-[%s]-[%s]' %
                (self.frame_id_in, self.frame_id_out, self.body_frame_id,
                 self.intermediate_frame_id, self.world_frame_id))
        else:
            node.get_logger().info(
                'ROSTopic: [%s]->[%s] No TF' % (self.frame_id_in, self.frame_id_out))

        self.path = []

    def _send_transform(self, translation, rotation, stamp, child_frame_id, parent_frame_id):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        self.br.sendTransform(t)

    def in_odom_callback(self, in_odom_msg):
        q = np.array([in_odom_msg.pose.pose.orientation.x,
                      in_odom_msg.pose.pose.orientation.y,
                      in_odom_msg.pose.pose.orientation.z,
                      in_odom_msg.pose.pose.orientation.w])
        p = np.array([in_odom_msg.pose.pose.position.x,
                      in_odom_msg.pose.pose.position.y,
                      in_odom_msg.pose.pose.position.z])

        e = tfs.euler_from_quaternion(q, 'rzyx')
        wqb = tfs.quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
        wqc = tfs.quaternion_from_euler(e[0], 0.0, 0.0, 'rzyx')

        #### odom ####
        odom_msg = in_odom_msg
        assert in_odom_msg.header.frame_id == self.frame_id_in
        odom_msg.header.frame_id = self.frame_id_out
        odom_msg.child_frame_id = ""
        self.out_odom_pub.publish(odom_msg)

        #### tf ####
        if self.broadcast_tf and self.tf_pub_flag:
            self.tf_pub_flag = False
            if not self.frame_id_in == self.frame_id_out:
                self._send_transform((0.0, 0.0, 0.0),
                                      tfs.quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx'),
                                      odom_msg.header.stamp,
                                      self.frame_id_in,
                                      self.frame_id_out)

            if not self.world_frame_id == self.frame_id_out:
                self._send_transform((0.0, 0.0, 0.0),
                                      tfs.quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx'),
                                      odom_msg.header.stamp,
                                      self.world_frame_id,
                                      self.frame_id_out)

            self._send_transform((p[0], p[1], p[2]),
                                  wqb,
                                  odom_msg.header.stamp,
                                  self.body_frame_id,
                                  self.world_frame_id)

            self._send_transform((p[0], p[1], p[2]),
                                  wqc,
                                  odom_msg.header.stamp,
                                  self.intermediate_frame_id,
                                  self.world_frame_id)
        #### path ####
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.append(pose)

    def path_pub_callback(self):
        if self.path:
            path = Path()
            path.header = self.path[-1].header
            path.poses = self.path[-30000::1]
            self.out_path_pub.publish(path)

    def tf_pub_callback(self):
        self.tf_pub_flag = True


class TfAssist(Node):

    def __init__(self):
        super().__init__('tf_assist',
                          automatically_declare_parameters_from_overrides=True)

        self.br = TransformBroadcaster(self)
        self.converters = []

        index = 0
        while True:
            prefix = 'converter%d' % index
            params = self.get_parameters_by_prefix(prefix)
            if not params:
                if index == 0:
                    self.get_logger().warn('No "%s.*" parameters found; no converters created.' % prefix)
                else:
                    self.get_logger().info('prefix:"%s" not found. Generated %d converter(s).' % (prefix, index))
                break

            def p(name, default=None):
                return params[name].value if name in params else default

            frame_id_in = p('frame_id_in')
            frame_id_out = p('frame_id_out')
            broadcast_tf = p('broadcast_tf', False)
            body_frame_id = p('body_frame_id', 'body')
            intermediate_frame_id = p('intermediate_frame_id', 'intermediate')
            world_frame_id = p('world_frame_id', 'world')

            converter = OdometryConverter(
                self, self.br, frame_id_in, frame_id_out, broadcast_tf,
                body_frame_id, intermediate_frame_id, world_frame_id)

            converter.in_odom_sub = self.create_subscription(
                Odometry, '%s/in_odom' % prefix, converter.in_odom_callback, 10)
            converter.out_odom_pub = self.create_publisher(
                Odometry, '%s/out_odom' % prefix, 10)
            converter.out_path_pub = self.create_publisher(
                Path, '%s/out_path' % prefix, 10)

            converter.tf_pub_timer = self.create_timer(0.1, converter.tf_pub_callback)
            converter.path_pub_timer = self.create_timer(0.5, converter.path_pub_callback)

            self.converters.append(converter)
            index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TfAssist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
