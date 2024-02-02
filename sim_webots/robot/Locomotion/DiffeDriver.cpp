#include "DiffeDriver.h"

DiffeDriver::DiffeDriver(CState& St): 
    state(St), 
    lL(St.lL), 
    lR(St.lR)
{
    rbt=state.rbt;
}

DiffeDriver::~DiffeDriver()= default;

void DiffeDriver::task()
{
    // lL.wl_cmd_v = (double)state.rc.vx ;
    // lR.wl_cmd_v = (double)state.rc.vx ;
    // 差速地盘模型，解算轮子速度
    float linear_command = state.rc.vx ;
    float angular_command = state.rc.wz ;
    double velocity_left =
        (linear_command - angular_command * wheel_separation / 2.0) / wheel_radius ;
    double velocity_right =
        (linear_command + angular_command * wheel_separation / 2.0) / wheel_radius ;
    // 赋值控制机器人轮子
    lL.wl_cmd_v = velocity_left ;
    lR.wl_cmd_v = velocity_right ;
    // 归零
    // state.rc.vx = 0.0 ;
    // state.rc.wz = 0.0 ;
}




