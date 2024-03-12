#!/usr/bin/env python3

import csv
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_pub_node')
        self.declare_parameter("file_path", "../config/path_data.csv")
        file_path = self.get_parameter("file_path").get_parameter_value().string_value
        # 最短距离
        self.min_distance = 2.0
        # pub path
        self.path_msg = Path()
        self.path_raw_msg = Path()
        self.next_goal = PoseStamped()
        self.path_pub = self.create_publisher(Path, '/global_path/update', 10)
        self.path_raw_pub = self.create_publisher(Path, '/global_path/raw', 10)
        self.read_csv_and_publish_path(file_path)
        self.create_timer(1.0, self.timer_callback)
        # 实时更新路径
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info("Pub path from csv file ...")

    def timer_callback(self):
        self.path_pub.publish(self.path_msg)
        self.path_raw_pub.publish(self.path_raw_msg)

    def read_csv_and_publish_path(self, file_path):
        self.path_msg.header.frame_id = 'map'  # Assuming the frame_id is 'map'
        with open(file_path, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            next(csvreader)
            for row in csvreader:
                x, y, theta = map(float, row)
                pose = PoseStamped()
                pose.pose.position = Point(x=x, y=y, z=0.0)
                self.path_msg.poses.append(pose)
        # 原始路径
        self.path_raw_msg = self.path_msg
        # 记录下一个路径点
        self.next_goal = self.path_msg.poses[0]

    def odom_callback(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        pose_goal_distance = math.sqrt((self.next_goal.pose.position.x-x)**2 + (self.next_goal.pose.position.y-y)**2)
        if pose_goal_distance < self.min_distance and len(self.path_msg.poses) > 1 :
            self.next_goal = self.path_msg.poses.pop(0)
            self.next_goal = self.path_msg.poses[0]
            self.get_logger().info("next goal ... ")
        
        # self.get_logger().info(f"{pose_goal_distance}")

def main(args=None):
    rclpy.init(args=args)
    path_pub_node = PathPublisher()
    rclpy.spin(path_pub_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
