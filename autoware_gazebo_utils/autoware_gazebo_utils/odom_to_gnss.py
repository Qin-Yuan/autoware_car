#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
class odom_to_gnss(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("odom msg to gnss .")
        self.gnss_msg = PoseWithCovarianceStamped()        
        self.odom_msg = Odometry()
        self.odom_sub = self.create_subscription(Odometry,"odom",self.odom_sub_callback,10)
        self.gnss_sub = self.create_publisher(PoseWithCovarianceStamped,"/sensing/gnss/pose_with_covariance",10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def odom_sub_callback(self,date):
        self.odom_msg = date
        
    def timer_callback(self) :
        self.gnss_msg.header.frame_id = "gnss_link"
        self.gnss_msg.header.stamp = self.get_clock().now().to_msg()
        self.gnss_msg.pose.pose.position = self.odom_msg.pose.pose.position
        self.gnss_sub.publish(self.gnss_msg)
        
def main(args=None):
    rclpy.init(args=args)			    
    odom_to_gnss_node = odom_to_gnss("odom_to_gnss_node")    
    rclpy.spin(odom_to_gnss_node)                 
    rclpy.shutdown()

main()