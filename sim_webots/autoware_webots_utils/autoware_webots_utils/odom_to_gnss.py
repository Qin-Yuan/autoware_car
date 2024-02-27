#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Point32
class odom_to_gnss(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("pose msg to gnss .")
        self.gnss_msg = PoseWithCovarianceStamped()        
        self.pose_msg = Point32()
        self.map_pose_gnss_init =[0.0,0.0,0.0] 
        self.pose_sub = self.create_subscription(Point32,"/vehicle/pose/point32",self.pose_sub_callback,10)
        self.gnss_sub = self.create_publisher(PoseWithCovarianceStamped,"/sensing/gnss/pose_with_covariance",10)
        # self.timer = self.create_timer(0.01, self.timer_callback)
        
    def pose_sub_callback(self,date):
        self.pose_msg = date
        self.timer_callback()
        
    def timer_callback(self) :
        self.gnss_msg.header.frame_id = "gnss_link"
        self.gnss_msg.header.stamp = self.get_clock().now().to_msg()
        self.gnss_msg.pose.pose.position.x = self.pose_msg.x - self.map_pose_gnss_init[0]
        self.gnss_msg.pose.pose.position.y = self.pose_msg.y - self.map_pose_gnss_init[1]
        self.gnss_msg.pose.pose.position.z = self.pose_msg.z - self.map_pose_gnss_init[2]
        self.gnss_sub.publish(self.gnss_msg)
        
def main(args=None):
    rclpy.init(args=args)			    
    odom_to_gnss_node = odom_to_gnss("odom_to_gnss_node")    
    rclpy.spin(odom_to_gnss_node)                 
    rclpy.shutdown()

main()