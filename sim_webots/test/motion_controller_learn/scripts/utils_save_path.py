#!/usr/bin/env python3

import csv
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations   # sudo pip3 install transforms3d


class PathSaveNode(Node):
    def __init__(self):
        super().__init__('path_save_node')
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.data = None
        self.last_data = None
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Save path to csv file ...")

    def odom_callback(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        euler = tf_transformations.euler_from_quaternion(orientation)
        theta = euler[2]
        self.data = [round(x, 2), round(y, 2), round(theta, 2)]
    
    def timer_callback(self):
        if self.data != None and self.last_data != self.data:
            self.save_data()

    def save_data(self):
        with open('../config/path_data.csv', 'a+', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # 如果文件为空，则写入标题行
            if csvfile.tell() == 0:
                writer.writerow(['x', 'y', 'theta'])
            # 追加新数据
            writer.writerow(self.data)
            self.last_data = self.data

def main(args=None):
    rclpy.init(args=args)
    path_save_node = PathSaveNode()
    rclpy.spin(path_save_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
