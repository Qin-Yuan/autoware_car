#include "RobotDriver.h"
#include <cmath>

RobotDriver::RobotDriver(CState& St): 
    state(St), 
    lL(St.lL), 
    lR(St.lR)
{
    rbt=state.rbt;
}

RobotDriver::~RobotDriver()= default;

void RobotDriver::task()
{
    // 1-差速底盘模型，解算轮子速度
    if ( RobotMode == 1 ) {
        float linear_command = state.rc.vx ;
        float angular_command = state.rc.wz ;
        double velocity_left =
            (linear_command - angular_command * wheel_separation / 2.0) / wheel_radius ;
        double velocity_right =
            (linear_command + angular_command * wheel_separation / 2.0) / wheel_radius ;
        // 赋值控制机器人轮子
        lL.wl_cmd_v = velocity_left ;
        lR.wl_cmd_v = velocity_right ;
    }
    // 2-阿克曼底盘模型，解算后轮速度和前轮转角
    else if ( RobotMode == 2 ) {
        float linear_command = state.rc.vx ;
        float angular_command = state.rc.wz ;
        // // 计算车辆的轨迹半径
        // float radius = linear_command / angular_command ;
        // // 计算车辆的车轮转角
        // float wheel_angle = atan(wheel_base / radius) ;
        // 计算车辆后轮的速度
        double wheel_velocity_ = linear_command / wheel_radius ;
        // 这里单独根据角速度直接控制轮子转角，没有运动学解算
        double wheel_angle_ = -angular_command / wheel_base ;
        if (abs(wheel_angle_) >= 1.0 ) {
            wheel_angle_ = abs(wheel_angle_) / wheel_angle_ ;
        }
        // 赋值控制机器人轮子
        lL.wl_cmd_v = wheel_velocity_ ;
        lL.wl_cmd_p = wheel_angle_ ;
        lR.wl_cmd_v = wheel_velocity_ ;
        lR.wl_cmd_p = wheel_angle_ ;
        // std::cout << wheel_angle_ << std::endl;
    }
    
    // 归零
    // state.rc.vx = 0.0 ;
    // state.rc.wz = 0.0 ;
}




