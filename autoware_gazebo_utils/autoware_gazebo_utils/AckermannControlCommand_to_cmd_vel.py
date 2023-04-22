#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from geometry_msgs.msg import Twist

class AckermannControlCommand_to_cmd_vel(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("Ackermann Control Command to cmd_vel .")
        self.AckermannControlCommand_msg = AckermannControlCommand()
        self.Cmd_vel_msg = Twist()

        self.AckermannControlCommand_sub = self.create_subscription(AckermannControlCommand,"/control/command/control_cmd",self.AckermannControlCommand_sub_callback,10)
        self.cmd_vel_pub = self.create_publisher(Twist,"cmd_vel",10)

    def AckermannControlCommand_sub_callback(self,date):
        # self.AckermannControlCommand_msg = date
        self.AckermannControlCommand_msg.longitudinal.speed = date.longitudinal.speed
        self.AckermannControlCommand_msg.longitudinal.acceleration = date.longitudinal.acceleration
        # print(self.AckermannControlCommand_msg.longitudinal.speed," - ",self.AckermannControlCommand_msg.longitudinal.acceleration)
        self.Cmd_vel_msg.linear.x = self.AckermannControlCommand_msg.longitudinal.speed
        self.Cmd_vel_msg.angular.z = -self.AckermannControlCommand_msg.longitudinal.acceleration
        self.cmd_vel_pub.publish(self.Cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)			    
    AckermannControlCommand_to_cmd_vel_node = AckermannControlCommand_to_cmd_vel("AckermannControlCommand_to_cmd_vel_node")    
    rclpy.spin(AckermannControlCommand_to_cmd_vel_node)                 
    rclpy.shutdown()

main()