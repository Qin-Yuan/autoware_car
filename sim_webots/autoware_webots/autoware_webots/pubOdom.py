#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,LaserScan
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDrive

class pubOdom(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("pose msg to gnss .")
        self.gps_msg = PointStamped()        
        self.imu_msg = Imu()
        self.ackermann_msgs = AckermannDrive()
        self.odom_msg = Odometry()
        self.create_subscription(PointStamped,"/sensing/gps/gps_raw",self.gps_sub_callback,10)
        self.create_subscription(Imu,"/sensing/imu/tamagawa/imu_raw",self.imu_sub_callback,10)
        self.create_subscription(AckermannDrive,"/cmd_ackermann",self.ackermann_sub_callback,10)

        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def gps_sub_callback(self,date):
        self.gps_msg = date
        
    def imu_sub_callback(self,date):
        self.imu_msg = date
        
    def ackermann_sub_callback(self,date):
        self.ackermann_msgs = date
        
    def timer_callback(self) :
        self.odom_msg.header.frame_id = "base_link"
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position = self.gps_msg.point
        self.odom_msg.pose.pose.orientation = self.imu_msg.orientation
        self.odom_msg.twist.twist.linear.x = self.ackermann_msgs.speed
        self.odom_msg.twist.twist.angular.z = self.ackermann_msgs.steering_angle
        self.pub_odom.publish(self.odom_msg)
        
def main(args=None):
    rclpy.init(args=args)			    
    pubOdom_node = pubOdom("pubOdom_node")    
    rclpy.spin(pubOdom_node)                 
    rclpy.shutdown()

main()