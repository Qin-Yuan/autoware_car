#!/usr/bin/env python3
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

''' LQR 控制器
'''
class LQRController :
    def __init__(self, N, EPS, Q, R, dt):
        self.N = N
        self.EPS = EPS      
        self.Q = Q 
        self.R = R  
        self.dt = dt       
    
    # lqr 输出
    def lqr_control(self, robot_state, refer_state, A, B) :
        x = robot_state[0:3] - refer_state[0:3]
        P = self.cal_Ricatti(A, B, self.Q, self.R)
        # 计算增益
        K = -np.linalg.pinv(self.R + B.T @ P @ B) @ B.T @ P @ A
        # 计算误差
        u = K @ x
        u_star = u      # u_star = [[v-ref_v,delta-ref_delta]] 
        # print(u_star,u_star[0,1])
        return u_star[0,1]
    
    def cal_Ricatti(self, A, B, Q, R):
        ''' 解代数里卡提方程
            Args:
                A (_type_): 状态矩阵A
                B (_type_): 状态矩阵B
                Q (_type_): Q为半正定的状态加权矩阵, 通常取为对角阵；Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零；
                R (_type_): R为正定的控制加权矩阵，R矩阵元素变大意味着希望控制输入能够尽可能小。
            Returns:
                P_: 迭代优化求解的 P
        '''
        # 设置迭代初始值
        Qf = Q
        P = Qf
        # 循环迭代
        for t in range(self.N):
            P_ = Q + A.T@P@A - A.T@P@B@np.linalg.pinv(R + B.T@P@B)@B.T@P@A
            if(abs(P_-P).max()<self.EPS):
                break
            P = P_
        return P_
    
''' LQR vehicle path tracing function
'''
class VehicleLQRPathTrack(Node):
    def __init__(self):
        super().__init__("VehickeLQRPathTrack")
        self._init_VehicleState()
        self._init_LQRController()
        self.create_subscription(Path, "/global_path/raw", self.update_global_path_callback, 10)
        self.create_subscription(Odometry, "odom", self.update_odom_callback, 10)
        self.twist_pub = self.create_publisher(AckermannDrive, "/cmd_ackermann", 10) ; self.twist_msgs = AckermannDrive()
        self.create_timer(self.dt, self.timer_callback)
    
    def timer_callback(self) :
        # print("timer_callback")
        # print("positions: ", self.x, self.y, self.yaw)
        if self.init_vehicle_state_flag == True and self.new_global_path_flag == True and self.lqr_controller_thread_flag == False:
            self.lqr_controller_thread.start()
            self.lqr_controller_thread_flag = True
        # plt show
        self.plt_LQRPathTrack() 
    
    def _init_LQRController(self) :
        lqr_N = 100             # 迭代范围
        lqr_EPS = 1e-4          # 迭代精度
        lqr_Q = np.eye(3)*3     # Q-半正定矩阵
        lqr_R = np.eye(2)*4.    # R-正定矩阵
        self.lqr_controller = LQRController(N=lqr_N, EPS=lqr_EPS, Q=lqr_Q, R=lqr_R, dt=self.dt)
        self.lqr_controller_thread = threading.Thread(target=self.vehicle_LQRPathTrack_thread)
        self.lqr_controller_thread_flag = False

    
    def _init_VehicleState(self) :
        self.max_steer_angle = 0.7  # [0,1]
        self.dt = 0.1        # 控制时间间隔，单位: s 秒
        self.L = 4.5         # 车辆轴距，单位: m 米
        self.x = None
        self.y = None
        self.steer_angle = 0.0
        self.v = 10.0
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

    def state_space(self, ref_delta, ref_yaw):
        """ 将模型离散化后的状态空间表达
            Args:
                ref_delta: 参考输入
                ref_yaw: 参考输入
            Returns:
                A: 
                B: 
        """
        A = np.matrix([
            [1.0, 0.0, -self.v*self.dt*math.sin(ref_yaw)],
            [0.0, 1.0, self.v*self.dt*math.cos(ref_yaw)],
            [0.0, 0.0, 1.0]])

        B = np.matrix([
            [self.dt*math.cos(ref_yaw), 0],
            [self.dt*math.sin(ref_yaw), 0],
            [self.dt*math.tan(ref_delta)/self.L, self.v*self.dt /
            (self.L*math.cos(ref_delta)*math.cos(ref_delta))]
        ])
        return A, B
    
    def ReferencePath(self):
        ''' 计算供 LQR 使用的 path
        '''
        path_count = len(self.global_path.poses)
        # refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k 
        self.refer_path = np.zeros((path_count, 4))
        for i in range(path_count):
            # pose = PoseStamped()
            pose = self.global_path.poses.pop(0)
            self.refer_path[i, 0] = pose.pose.position.x
            self.refer_path[i, 1] = pose.pose.position.y
        for i in range(path_count):
            if i == 0:
                dx = self.refer_path[i+1,0] - self.refer_path[i,0]
                dy = self.refer_path[i+1,1] - self.refer_path[i,1]
                ddx = self.refer_path[2,0] + self.refer_path[0,0] - 2*self.refer_path[1,0]
                ddy = self.refer_path[2,1] + self.refer_path[0,1] - 2*self.refer_path[1,1]
            elif i == (len(self.refer_path)-1):
                dx = self.refer_path[i,0] - self.refer_path[i-1,0]
                dy = self.refer_path[i,1] - self.refer_path[i-1,1]
                ddx = self.refer_path[i,0] + self.refer_path[i-2,0] - 2*self.refer_path[i-1,0]
                ddy = self.refer_path[i,1] + self.refer_path[i-2,1] - 2*self.refer_path[i-1,1]
            else:      
                dx = self.refer_path[i+1,0] - self.refer_path[i,0]
                dy = self.refer_path[i+1,1] - self.refer_path[i,1]
                ddx = self.refer_path[i+1,0] + self.refer_path[i-1,0] - 2*self.refer_path[i,0]
                ddy = self.refer_path[i+1,1] + self.refer_path[i-1,1] - 2*self.refer_path[i,1]
            self.refer_path[i,2]=math.atan2(dy,dx) # yaw
            # 计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
            # 参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
            self.refer_path[i,3]=(ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2)) # 曲率k计算
        
        return path_count
    
    def calc_track_error(self, x, y):
        """ 计算跟踪误差
            Args:
                x (_type_): 当前车辆的位置x
                y (_type_): 当前车辆的位置y
            Returns:
                _type_: _description_
        """
        # 寻找参考轨迹最近目标点
        d_x = [self.refer_path[i,0]-x for i in range(len(self.refer_path))] 
        d_y = [self.refer_path[i,1]-y for i in range(len(self.refer_path))] 
        # 距离
        d = [np.sqrt(d_x[i]**2+d_y[i]**2) for i in range(len(d_x))]             
        s = np.argmin(d) # 最近目标点索引
        # 参考偏航角 yaw
        yaw = self.refer_path[s, 2]
        # 参考曲率 r
        k = self.refer_path[s, 3]
        # 计算角度，即在参考yaw 左侧还是右侧
        angle = self.normalize_angle(yaw - math.atan2(d_y[s], d_x[s]))
        e = d[s]  # 误差
        if angle < 0:
            e *= -1
        return e, k, yaw, s
    
    def normalize_angle(self, angle):
        """
            Normalize an angle to [-pi, pi].
            :param angle: (float)
            :return: (float) Angle in radian in [-pi, pi]
            copied from https://atsushisakai.github.io/PythonRobotics/modules/path_tracking/stanley_control/stanley_control.html
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def vehicle_LQRPathTrack_thread(self) :
        robot_state = np.zeros(4)
        refer_state = np.zeros(4)
        # print(self.global_path.poses.pop(0))
        # 将 ros2 path 计算出：位置x, 位置y， 轨迹点的切线方向, 曲率k 
        path_count = self.ReferencePath()
        # refer_path = [sublist[:2] for sublist in self.refer_path]
        # refer_tree = KDTree(refer_path)  # reference trajectory
        # print( self.path_goal_index, path_count , distance)
        while True:
            start_time = time.time()
            # 1-更新机器人状态
            robot_state = [self.x, self.y, self.yaw, self.v]
            # 2-计算参考轨迹上距离机器人最近的点
            e, k, ref_yaw, self.path_goal_index = self.calc_track_error(robot_state[0], robot_state[1])
            # 3-索引最近目标点位姿
            refer_state = self.refer_path[self.path_goal_index, 0:3]
            # 4-计算参考前轮转角
            ref_delta = math.atan2(self.L*k, 1)
            # 5-计算A、B状态矩阵
            A, B = self.state_space(ref_delta, ref_yaw)
            # 6-lqr控制器输出值
            delta = self.lqr_controller.lqr_control(robot_state, refer_state, A, B)
            delta = delta + ref_delta
            delta = -delta
            # 限制前车轮最大转角
            if delta < -self.max_steer_angle :
                delta = -self.max_steer_angle
            elif delta > self.max_steer_angle :
                delta = self.max_steer_angle
            # 7-发布控制指令
            self.twist_publisher(0, delta)
            # print("positions: ", self.x, self.y, self.yaw)
            # print(distance, self.path_goal_index)
            # 计算距离终点的距离
            distance = math.sqrt((refer_state[0] - robot_state[0])**2 + (refer_state[1] - robot_state[1])**2)
            # 记录结束时间
            end_time = time.time()
            elapsed_time = end_time - start_time
            if elapsed_time < self.dt :
                time.sleep(self.dt-elapsed_time)
            else:
                self.get_logger().error(f"LQR controller dt is {self.dt}, but elapsed_time is {elapsed_time} .")
            # 到达终点停止
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

    def plt_LQRPathTrack(self) :
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
    vehicle_lqr_path_track_node = VehicleLQRPathTrack()
    rclpy.spin(vehicle_lqr_path_track_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
