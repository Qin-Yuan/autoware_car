#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import ControlModeReport,GearReport,HazardLightsReport,SteeringReport,TurnIndicatorsReport,VelocityReport
class vehlicle_gazebo_pub(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("vehlicle gazebo pub .")
        self.odom_msg = Odometry()
        self.odom_sub = self.create_subscription(Odometry,"odom",self.odom_sub_callback,10)
        # publisher
        self.control_mode_msgs = ControlModeReport() ; self.control_mode_msgs.mode = 1
        self.control_mode_pub = self.create_publisher(ControlModeReport,"/vehicle/status/control_mode",10)

        self.gear_status_msgs = GearReport() ; self.gear_status_msgs.report = 22 ;self.gear_status_init_flag = False
        self.gear_status_pub = self.create_publisher(GearReport,"/vehicle/status/gear_status",10)

        self.hazard_lights_status_msgs = HazardLightsReport() ; self.hazard_lights_status_msgs.report = 1
        self.hazard_lights_status_pub = self.create_publisher(HazardLightsReport,"/vehicle/status/hazard_lights_status",10)

        self.steering_status_msgs = SteeringReport() ; self.steering_status_msgs.steering_tire_angle = 0.0
        self.steering_status_pub = self.create_publisher(SteeringReport,"/vehicle/status/steering_status",10)

        self.turn_indicators_status_msgs = TurnIndicatorsReport() ; self.turn_indicators_status_msgs.report = 1
        self.turn_indicators_status_pub = self.create_publisher(TurnIndicatorsReport,"/vehicle/status/turn_indicators_status",10)

        self.velocity_status_msgs = VelocityReport() ; self.velocity_status_msgs.header.frame_id = "base_link" ; 
        self.velocity_status_msgs.longitudinal_velocity = 0.0 ; 
        self.velocity_status_msgs.lateral_velocity = 0.0 ; 
        self.velocity_status_msgs.heading_rate = 5.326322207110934e-05 ; 
        self.velocity_status_pub = self.create_publisher(VelocityReport,"/vehicle/status/velocity_status",10)

    def odom_sub_callback(self,date):
        self.odom_msg = date
        if self.gear_status_init_flag == False :
            if abs(self.odom_msg.twist.twist.linear.x) > 0.01 or abs(self.odom_msg.twist.twist.angular.z) > 0.01 :
                self.odom_msg.twist.twist.linear.x = 0.0
                # print(self.odom_msg.twist.twist.linear.x , self.odom_msg.twist.twist.angular.z)
                self.gear_status_init_flag = True
                self.gear_status_msgs.report = 2
                # self.get_logger().warn("vel : %f %f"%(abs(self.odom_msg.twist.twist.linear.x),abs(self.odom_msg.twist.twist.angular.z)))
        
        if abs(self.odom_msg.twist.twist.linear.x) < 0.01 :
            self.odom_msg.twist.twist.linear.x = 0.0

        if abs(self.odom_msg.twist.twist.angular.z) < 0.01 :
            self.odom_msg.twist.twist.angular.z = 0.0
            
        self.velocity_status_msgs.longitudinal_velocity = self.odom_msg.twist.twist.linear.x
        self.velocity_status_msgs.lateral_velocity = self.odom_msg.twist.twist.linear.y
        self.velocity_status_msgs.heading_rate = self.odom_msg.twist.twist.angular.z

        self.control_mode_msgs.stamp = self.get_clock().now().to_msg()
        self.gear_status_msgs.stamp = self.get_clock().now().to_msg()
        self.hazard_lights_status_msgs.stamp = self.get_clock().now().to_msg()
        self.steering_status_msgs.stamp = self.get_clock().now().to_msg()
        self.turn_indicators_status_msgs.stamp = self.get_clock().now().to_msg()
        self.velocity_status_msgs.header.stamp = self.get_clock().now().to_msg()

        self.control_mode_pub.publish(self.control_mode_msgs)
        self.gear_status_pub.publish(self.gear_status_msgs)
        self.hazard_lights_status_pub.publish(self.hazard_lights_status_msgs)
        self.steering_status_pub.publish(self.steering_status_msgs)
        self.turn_indicators_status_pub.publish(self.turn_indicators_status_msgs)
        self.velocity_status_pub.publish(self.velocity_status_msgs)

def main(args=None):
    rclpy.init(args=args)			    
    vehlicle_gazebo_pub_node = vehlicle_gazebo_pub("vehlicle_gazebo_pub_node")    
    rclpy.spin(vehlicle_gazebo_pub_node)                 
    rclpy.shutdown()

main()