#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

''' 位置式 PID  控制器
'''
class PositionPIDControl :
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
    
