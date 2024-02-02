#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import subprocess
import time
from ament_index_python.packages import get_package_share_directory

class AutowareWebotsDriverNode(Node):
    def __init__(self) :
        super().__init__('autoware_webots_run')
        self.get_logger().info("autoware webots driver running ...")
        self.robot_dir = get_package_share_directory("robot") + "/../../lib/robot/"
        while True :
            result = subprocess.run(f"cd {self.robot_dir} ; ./robot", shell=True, stdout=subprocess.PIPE, text=True)
            self.get_logger().warning("autoware webots driver reset")

def main(args=None):
    rclpy.init(args=args)
    autoware_webots_run = AutowareWebotsDriverNode()
    rclpy.spin(autoware_webots_run)
    autoware_webots_run.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    