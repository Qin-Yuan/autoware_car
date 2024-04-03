#!/usr/bin/env python3
import time
import math
import cvxpy
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

''' MPC 控制器
'''
class MPCController :
    def __init__(self, NX, NU, T, R, Rd, Q, Qf, MAX_VEL, MAX_STEER, dt, L):
        self.NX = NX
        self.NU = NU      
        self.T = T 
        self.R = R  
        self.Rd = Rd  
        self.Q = Q  
        self.Qf = Qf  
        self.MAX_VEL = MAX_VEL
        self.MAX_STEER = MAX_STEER
        self.dt = dt       
        self.L = L       

    def state_space(self, ref_v, ref_delta, ref_yaw):
        """ 将模型离散化后的状态空间表达
            Args:
                ref_delta: 参考的转角控制量
                ref_yaw: 参考的偏航角
            Returns:
                A: 参考状态矩阵
                B: 参考输入矩阵
        """
        A = np.matrix([
            [1.0, 0.0, -ref_v*self.dt*math.sin(ref_yaw)],
            [0.0, 1.0, ref_v*self.dt*math.cos(ref_yaw)],
            [0.0, 0.0, 1.0]])

        B = np.matrix([
            [self.dt*math.cos(ref_yaw), 0],
            [self.dt*math.sin(ref_yaw), 0],
            [self.dt*math.tan(ref_delta)/self.L, ref_v*self.dt /
            (self.L*math.cos(ref_delta)*math.cos(ref_delta))]
        ])
        C = np.eye(3)
        return A, B, C
    
    # mpc 输出
    def linear_mpc_control(self, xref, x0, delta_ref) :
        ''' 
            xref: vehicle 参考轨迹
            x0: vehicle 初始状态(x, y, z, steer, velocity)
            delta_ref: 参考输入(velocity,steer_angle)
            returns: 最优的控制量和最优状态
        '''
        x = cvxpy.Variable((self.NX, self.T+1))     # 状态，+1是初始初始状态
        u = cvxpy.Variable((self.NU, self.T))       # 输入
        cost = 0.0                                  # 代价函数
        constraints = []                            # 约束条件
        # 预测空间长度 T 内的代价函数和约束条件
        for t in range(self.T):
            # 第一个 cost 函数 , 当前输入偏差
            cost += cvxpy.quad_form(u[:, t] - delta_ref[:, t], self.R)
            # 第二个 cost 函数 , 除去初始状态，当前状态偏差
            if t != 0 :
                cost += cvxpy.quad_form(x[:, t] - xref[:, t], self.Q)
            # A: 参考状态矩阵 , B: 参考输入矩阵
            A, B, C = self.state_space(delta_ref[0, t], delta_ref[1, t], xref[2, t])
            # 约束条件 X(t+1) = Ax(t) + Bu(t)
            constraints += [x[:, t + 1] - xref[:, t + 1] == A @ 
                            (x[:, t] - xref[:, t]) + B @ (u[:, t] - delta_ref[:, t])]
        # 第三个 cost 函数 , 最后一个稳定时的代价
        cost += cvxpy.quad_form(x[:, self.T] - xref[:, self.T], self.Qf)
        # 约束条件
        constraints += [(x[:, 0]) == x0]
        constraints += [cvxpy.abs(u[0, :]) <= self.MAX_VEL]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]

        # 优化问题描述，最小化 cost 同时满足 constraints 约束
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        # 指定求解器，https://www.wuzao.com/document/cvxpy/install/index.html
        prob.solve(solver=cvxpy.ECOS, verbose=False)
        # 
        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            opt_x = self.get_nparray_from_matrix(x.value[0, :])
            opt_y = self.get_nparray_from_matrix(x.value[1, :])
            opt_yaw = self.get_nparray_from_matrix(x.value[2, :])
            opt_v = self.get_nparray_from_matrix(u.value[0, :])
            opt_delta = self.get_nparray_from_matrix(u.value[1, :])
        else:
            print("Error: Cannot solve mpc..")
            opt_v, opt_delta, opt_x, opt_y, opt_yaw = None, None, None, None, None,
        
        # 返回优化控制输出
        return opt_v, opt_delta, opt_x, opt_y, opt_yaw
    
    def get_nparray_from_matrix(self, x):
        return np.array(x).flatten()
    
''' MPC vehicle path tracing function
'''
class VehicleMPCPathTrack(Node):
    def __init__(self):
        super().__init__("VehickeMPCPathTrack")
        self._init_VehicleState()
        self._init_MPCController()
        self.create_subscription(Path, "/global_path/raw", self.update_global_path_callback, 10)
        self.create_subscription(Odometry, "odom", self.update_odom_callback, 10)
        self.twist_pub = self.create_publisher(AckermannDrive, "/cmd_ackermann", 10) ; self.twist_msgs = AckermannDrive()
        self.create_timer(self.dt, self.timer_callback)
    
    def timer_callback(self) :
        # print("timer_callback")
        # print("positions: ", self.x, self.y, self.yaw)
        if self.init_vehicle_state_flag == True and self.new_global_path_flag == True and self.mpc_controller_thread_flag == False:
            self.mpc_controller_thread.start()
            self.mpc_controller_thread_flag = True
        # plt show
        self.plt_MPCPathTrack() 
    
    def _init_MPCController(self) :
        self.mpc_NX = 3                     # 状态维度 x = x, y, yaw
        self.mpc_NU = 2                     # 输入维度 u = [v,delta]
        self.mpc_T = 8                      # MPC预测区间
        mpc_R = np.diag([0.1, 0.1])*10        # input cost matrix
        mpc_Rd = np.diag([0.1, 0.1])        # input difference cost matrix
        mpc_Q = np.diag([1, 1, 1])*2         # state cost matrix
        mpc_Qf = mpc_Q                      # state final matrix
        self.mpc_controller = MPCController(NX=self.mpc_NX, NU=self.mpc_NU, T=self.mpc_T, 
                                            R=mpc_R, Rd=mpc_Rd, Q=mpc_Q, 
                                            Qf=mpc_Qf, MAX_VEL=self.max_vel, MAX_STEER=self.max_steer_angle ,
                                            dt=self.dt, L=self.L)
        self.mpc_controller_thread = threading.Thread(target=self.vehicle_MPCPathTrack_thread)
        self.mpc_controller_thread_flag = False
    
    def _init_VehicleState(self) :
        self.max_steer_angle = 0.7      # [0,1]
        self.max_vel = 10.0
        self.dt = 0.3                   # 控制时间间隔，单位: s 秒
        self.L = 4.5                    # 车辆轴距，单位: m 米
        self.x = None
        self.y = None
        self.steer_angle = 0.0
        self.v = 2.0
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
    
    def ReferencePath(self):
        ''' 计算供 MPC 使用的 path
        '''
        path_count = len(self.global_path.poses)
        # refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k 
        self.refer_path = np.zeros((path_count, 4))
        # 使用差分的方式计算路径点的一阶导和二阶导，从而得到切线方向和曲率
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
    
    def calc_ref_trajectory(self, robot_state, dl=1.0) :
        """计算参考轨迹点，统一化变量数组，便于后面MPC优化使用
            参考自https://github.com/AtsushiSakai/PythonRobotics/blob/eb6d1cbe6fc90c7be9210bf153b3a04f177cc138/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py
        Args:
            robot_state (_type_): 车辆的状态(x,y,yaw,v)
            dl (float, optional): _description_. Defaults to 1.0.

        Returns:
            _type_: _description_
        """
        e, k, ref_yaw, ind = self.calc_track_error(robot_state[0], robot_state[1])
        xref = np.zeros((self.mpc_NX, self.mpc_T + 1))
        dref = np.zeros((self.mpc_NU, self.mpc_T))
        ncourse = len(self.refer_path)
        # 参考状态
        xref[0, 0] = self.refer_path[ind, 0]
        xref[1, 0] = self.refer_path[ind, 1]
        xref[2, 0] = self.refer_path[ind, 2]
        # 参考控制量
        ref_delta = math.atan2(self.L*k, 1)             # 计算参考前轮转角
        dref[0, :] = robot_state[3]                     # 参考速度，这里恒定
        dref[1, :] = ref_delta

        travel = 0.0

        for i in range(self.mpc_T + 1) :
            # ？？？
            travel += abs(robot_state[3]) * self.dt
            dind = int(round(travel / dl))
            if (ind + dind) < ncourse:
                xref[0, i] = self.refer_path[ind + dind, 0]
                xref[1, i] = self.refer_path[ind + dind, 1]
                xref[2, i] = self.refer_path[ind + dind, 2]
            else:
                xref[0, i] = self.refer_path[ncourse - 1, 0]
                xref[1, i] = self.refer_path[ncourse - 1, 1]
                xref[2, i] = self.refer_path[ncourse - 1, 2]
        return xref, ind, dref
    
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
    
    def vehicle_MPCPathTrack_thread(self) :
        robot_state = np.zeros(4)
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
            x0 = robot_state[0:3]
            # 2-计算参考状态、目标状态索引、参考动作
            xref, self.path_goal_index, dref = self.calc_ref_trajectory(robot_state)
            # 3-mpc控制器输出值
            opt_v, opt_delta, self.opt_x, self.opt_y, opt_yaw = self.mpc_controller.linear_mpc_control(xref, x0, dref)
            # 限制前车轮最大转角
            # print(opt_delta)
            if opt_delta[0] < -self.max_steer_angle :
                opt_delta[0] = -self.max_steer_angle
            elif opt_delta[0] > self.max_steer_angle :
                opt_delta[0] = self.max_steer_angle
            # 7-发布控制指令
            self.twist_publisher(0, -opt_delta[0])
            # print("positions: ", self.x, self.y, self.yaw)
            # print(distance, self.path_goal_index)
            # 计算距离终点的距离
            distance = math.sqrt((self.refer_path[self.path_goal_index][0] - robot_state[0])**2 + (self.refer_path[self.path_goal_index][1] - robot_state[1])**2)
            # 记录结束时间
            end_time = time.time()
            elapsed_time = end_time - start_time
            if elapsed_time < self.dt :
                time.sleep(self.dt-elapsed_time)
            else:
                self.get_logger().error(f"MPC controller dt is {self.dt}, but elapsed_time is {elapsed_time} .")
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

    def plt_MPCPathTrack(self) :
        if len(self.refer_path) > 0 :
            plt.cla()
            # 1 - global_path show 
            plt.plot(self.refer_path[:, 0], self.refer_path[:, 1], '-.b', linewidth=1.0)
            # 2 - now goal show
            # print(self.path_goal_index)
            plt.plot(self.refer_path[self.path_goal_index, 0], self.refer_path[self.path_goal_index, 1], "*", color="r")
            # 3 - vehicle now show
            plt.plot(self.x, self.y, "go")
            # 4 - mpc 预测的状态
            pred_x = [self.opt_x[i] for i in range(len(self.opt_x))]
            pred_y = [self.opt_y[i] for i in range(len(self.opt_y))]
            plt.plot(pred_x, pred_y, '-g', label="predicted trajectory")
            # 5 - vehicle past path
            self._past_x.append(self.x)
            self._past_y.append(self.y)
            plt.plot(self._past_x, self._past_y, "-r", label="trajectory")
            # plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    vehicle_mpc_path_track_node = VehicleMPCPathTrack()
    rclpy.spin(vehicle_mpc_path_track_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
