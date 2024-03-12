#!/usr/bin/env python3

import sys
import time
import math
import threading
import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
import tf_transformations

''' 位置式 PID  控制器
'''
class PositionPID :
    def __init__(self, k=[1., 0., 0.], target=1.0, upper=1.0, lower=-1.0 ):
        self.kp, self.ki, self.kd = k
        self.e = 0      # error
        self.pre_e = 0  # previous error
        self.sum_e = 0  # sum of error
        self.target = target        # target
        self.upper_bound = upper    # upper bound of output
        self.lower_bound = lower    # lower bound of output
    
    def cal_output(self,  state) :
        self.e = self.target - state
        u = self.e * self.kp + self.sum_e * \
            self.ki + (self.e - self.pre_e) * self.kd
        # 限制
        if u < self.lower_bound:
            u = self.lower_bound
        elif u > self.upper_bound:
            u = self.upper_bound
        self.pre_e = self.e
        self.sum_e += self.e
        # print(self.sum_e)
        return u

''' PID vehicle path tracing function
'''
class VehiclePIDPathTrack(Node):
    def __init__(self):
        super().__init__("VehickePIDPathTrack")
        self._init_PositionPID()
        self._init_VehicleState()
        self.create_subscription(Path, "/global_path/raw", self.update_global_path_callback, 10)
        self.create_subscription(Odometry, "odom", self.update_odom_callback, 10)
        self.twist_pub = self.create_publisher(AckermannDrive, "/cmd_ackermann", 10) ; self.twist_msgs = AckermannDrive()
        self.create_timer(self.dt, self.timer_callback)
    
    def timer_callback(self) :
        # print("timer_callback")
        # print("positions: ", self.x, self.y, self.yaw)
        if self.init_vehicle_state_flag == True and self.new_global_path_flag == True and self.pid_controller_thread_flag == False:
            self.pid_controller_thread.start()
            self.pid_controller_thread_flag = True
        
        # plt show
        self.plt_PIDPathTrack() 
    
    def _init_PositionPID(self) :
        # 0.2 0.0 10.0+
        kp = 0.2
        ki = 0.0
        kd = 20.3
        self.pid_controller = PositionPID(k=[kp, ki, kd], target=0, upper=np.pi/5, lower=-np.pi/5)

        self.pid_controller_thread = threading.Thread(target=self.vehicle_PIDPathTrack_thread)
        self.pid_controller_thread_flag = False

    
    def _init_VehicleState(self) :
        self.dt = 0.02
        self.x = None
        self.y = None
        self.steer_angle = 0.0
        self.v = 5.0
        self._past_x = []
        self._past_y = []
        self.yaw = None
        self.init_vehicle_state_flag = False

        self.global_path = Path()
        self.refer_path = []
        self.path_goal_index = 0
        self.new_global_path_flag = False
    
    def update_odom_callback(self, msgs) :
        # update position
        # msgs = Odometry()
        self.x = msgs.pose.pose.position.x
        self.y = msgs.pose.pose.position.y
        # update yaw data
        orientation = [ msgs.pose.pose.orientation.x, msgs.pose.pose.orientation.y, 
                        msgs.pose.pose.orientation.z, msgs.pose.pose.orientation.w
                        ]
        euler =  tf_transformations.euler_from_quaternion(orientation)
        self.yaw = euler[2]
        # set self.init_vehicle_state_flag to True
        if self.init_vehicle_state_flag == False :
            self.init_vehicle_state_flag = True

    def update_global_path_callback(self, msgs) :
        if self.new_global_path_flag == False :
            self.new_global_path_flag = True
            self.global_path = msgs


    def vehicle_PIDPathTrack_thread(self) :
        path_count = len(self.global_path.poses)
        self.refer_path = np.zeros((path_count, 2))
        robot_state = np.zeros(2)
        # print(self.global_path.poses.pop(0))
        for i in range(path_count):
            # pose = PoseStamped()
            pose = self.global_path.poses.pop(0)
            self.refer_path[i, 0] = pose.pose.position.x
            self.refer_path[i, 1] = pose.pose.position.y
        refer_tree = KDTree(self.refer_path)  # reference trajectory
        distance = 1
        # print( self.path_goal_index, path_count , distance)
        while True:
            robot_state[0] = self.x
            robot_state[1] = self.y
            # 1-计算参考轨迹上距离机器人最近的点
            distance, self.path_goal_index = refer_tree.query(robot_state)
            # 2-计算alpha角
            alpha = math.atan2(self.refer_path[self.path_goal_index,1] - robot_state[1],
                                self.refer_path[self.path_goal_index,0] - robot_state[0])
            # 3-计算后轮中心与目标点之间的距离-前视距离 np.linalg.norm(self.refer_path[self.path_goal_index]-robot_state)
            l_d = distance
            # 4-计算后轮中心与目标点之间的夹角-theta_e
            theta_e = alpha - self.yaw
            # 5-计算横向跟踪误差
            e_y = l_d*math.sin(theta_e)
            # 6-控制器,横向控制误差对应控制前轮转角 steer_angle
            delta_f = self.pid_controller.cal_output(e_y)
            # 7-发布控制指令
            self.twist_publisher(0, delta_f)
            # print("positions: ", self.x, self.y, self.yaw)
            # print(distance, self.path_goal_index)
            time.sleep(self.dt)
            # d到达目标点停止
            if self.path_goal_index == path_count - 1 and distance < 0.5 :
                break
        self.twist_stop()

    def twist_publisher(self, a, delta_f) :
        self.v = self.v + a*self.dt
        self.steer_angle = delta_f
        self.twist_msgs.speed = self.v
        self.twist_msgs.steering_angle = self.steer_angle
        # print(self.v, self.steer_angle)
        self.twist_pub.publish(self.twist_msgs)

    def twist_stop(self):
        self.twist_msgs.speed = 0.0
        self.twist_msgs.steering_angle = 0.0
        self.twist_pub.publish(self.twist_msgs)
        self.get_logger().info("reach the target !")

    def plt_PIDPathTrack(self) :
        if len(self.refer_path) > 0 :
            plt.cla()
            # 1 - global_path show 
            plt.plot(self.refer_path[:, 0], self.refer_path[:, 1], '-.b', linewidth=1.0)
            # 2 - now goal show
            # print(self.path_goal_index)
            plt.plot(self.refer_path[self.path_goal_index, 0], self.refer_path[self.path_goal_index, 1], "*", color="r")
            # 3 - vehicle now show
            plt.plot(self.x, self.y, "go")
            # 4 - vehicle past path
            self._past_x.append(self.x)
            self._past_y.append(self.y)
            plt.plot(self._past_x, self._past_y, "-r", label="trajectory")
            # plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)
            

def main(args=None):
    rclpy.init(args=args)
    vehicle_pid_path_track_node = VehiclePIDPathTrack()
    rclpy.spin(vehicle_pid_path_track_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
