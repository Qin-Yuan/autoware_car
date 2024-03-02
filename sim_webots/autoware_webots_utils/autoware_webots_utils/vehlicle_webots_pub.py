#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import ControlModeReport,GearReport,HazardLightsReport,SteeringReport,TurnIndicatorsReport,VelocityReport
import math

class vehlicle_webots_pub(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("vehlicle webots pub .")
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
        self.velocity_status_msgs.heading_rate = self.angular2degree()
        self.steering_status_msgs.steering_tire_angle = self.angular2degree()

        self.control_mode_msgs.stamp = self.odom_msg.header.stamp
        self.gear_status_msgs.stamp = self.odom_msg.header.stamp
        self.hazard_lights_status_msgs.stamp = self.odom_msg.header.stamp
        self.steering_status_msgs.stamp = self.odom_msg.header.stamp
        self.turn_indicators_status_msgs.stamp = self.odom_msg.header.stamp
        self.velocity_status_msgs.header.stamp = self.odom_msg.header.stamp

        self.control_mode_pub.publish(self.control_mode_msgs)
        self.gear_status_pub.publish(self.gear_status_msgs)
        self.hazard_lights_status_pub.publish(self.hazard_lights_status_msgs)
        self.steering_status_pub.publish(self.steering_status_msgs)
        self.turn_indicators_status_pub.publish(self.turn_indicators_status_msgs)
        # self.velocity_status_pub.publish(self.velocity_status_msgs)
    
    def angular2degree(self):
        wheelbase = 0.86*2  # 小车轮距
        lenbase = 2.94    # 小车轴距
        inner_angle =  0.0
        # 内轮转角 = arctan(车子轴距 * 角速度 / 车子线速度)
        # 其中，车子轴距是车子前后轮的距离，角速度是车子绕垂直于地面的轴旋转的速率，车子线速度是车子沿着地面运动的速率。
        # if self.odom_msg.twist.twist.linear.x != 0 and self.odom_msg.twist.twist.angular.z != 0 :
        #     # inner_angle = (2 * self.odom_msg.twist.twist.linear.x) / (wheelbase * self.odom_msg.twist.twist.angular.z)
        #     inner_angle = math.atan(lenbase * self.odom_msg.twist.twist.angular.z / self.odom_msg.twist.twist.linear.x)
        inner_angle = self.odom_msg.twist.twist.angular.z
        return inner_angle


def main(args=None):
    rclpy.init(args=args)			    
    vehlicle_webots_pub_node = vehlicle_webots_pub("vehlicle_webots_pub_node")    
    rclpy.spin(vehlicle_webots_pub_node)                 
    rclpy.shutdown()

main()