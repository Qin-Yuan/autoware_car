#!/usr/bin/env python3

import csv
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped

class BaseLinkTFMap(Node):
    def __init__(self):
        super().__init__('baselink_tf_map_node')
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("Pub base_link to map tf2 ...")

    def odom_callback(self, msg):
        odom_tf = TransformStamped()
        odom_tf.header = msg.header
        odom_tf.header.frame_id = "map"
        odom_tf.child_frame_id = "base_link"
        odom_tf.transform.translation.x = msg.pose.pose.position.x
        odom_tf.transform.translation.y = msg.pose.pose.position.y
        odom_tf.transform.translation.z = msg.pose.pose.position.z
        odom_tf.transform.rotation.x = msg.pose.pose.orientation.x
        odom_tf.transform.rotation.y = msg.pose.pose.orientation.y
        odom_tf.transform.rotation.z = msg.pose.pose.orientation.z
        odom_tf.transform.rotation.w = msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(odom_tf)

def main(args=None):
    rclpy.init(args=args)
    baselink_tf_map_node = BaseLinkTFMap()
    rclpy.spin(baselink_tf_map_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
