#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class tf2_base_link_to_map(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("tf2 base_link to map .")

        self._br = TransformBroadcaster(self)
        self.base_link_to_map_tf = TransformStamped()
        self.odom_msg = Odometry()
        self.odom_sub = self.create_subscription(Odometry,"odom",self.odom_sub_callback,10)

    def odom_sub_callback(self,date):
        self.odom_msg = date
        # self.gnss_msg.header.stamp = self.get_clock().now().to_msg()
        self.base_link_to_map_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_to_map_tf.header.frame_id = 'map'
        self.base_link_to_map_tf.child_frame_id = 'base_link'
        self.base_link_to_map_tf.transform.translation.x = self.odom_msg.pose.pose.position.x
        self.base_link_to_map_tf.transform.translation.y = self.odom_msg.pose.pose.position.y
        self.base_link_to_map_tf.transform.translation.z = self.odom_msg.pose.pose.position.z
        self.base_link_to_map_tf.transform.rotation.x = self.odom_msg.pose.pose.orientation.x
        self.base_link_to_map_tf.transform.rotation.y = self.odom_msg.pose.pose.orientation.y
        self.base_link_to_map_tf.transform.rotation.z = self.odom_msg.pose.pose.orientation.z
        self.base_link_to_map_tf.transform.rotation.w = self.odom_msg.pose.pose.orientation.w
        self._br.sendTransform(self.base_link_to_map_tf)

def main(args=None):
    rclpy.init(args=args)			    
    tf2_base_link_to_map_node = tf2_base_link_to_map("tf2_base_link_to_map_node")    
    rclpy.spin(tf2_base_link_to_map_node)                 
    rclpy.shutdown()

main()