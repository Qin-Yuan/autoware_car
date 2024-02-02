#include "CState.h"
#include "DiffeDriver.h"
#include "Ros2Info.hpp"
#include <iostream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    /*== 创建机器人状态量，包含上位机、参数等信息 =================*/
    // std::cout << "flag 1 " << std::endl;
    CState state;
    // state.rbt->testRobot();
    /*== 移动和操作任务主循环 ================================*/
    DiffeDriver move_df(state);
    // CManu manu(state);
    auto node = std::make_shared<Ros2Info>(&state);
    RCLCPP_INFO(node->get_logger(), "ROBOT RUNNING ...");
    while(move_df.taskId!=STOPLOOP)
    // while(rclcpp::ok())
    {
        //-- 机器人状态交互 ------------------
        rclcpp::spin_some(node);
        state.rdRobot();
        //-- 安全性检查 ---------------------
        if(state.FailCheck()) break;

        //-- UDP下发报文命令解析 -------------
        //-- 机器人运动控制 ------------------
        move_df.task(); // 行走移动
        // manu.task(); // 抓取操作

        //-- 执行电机控制 --------------------
        state.wrRobot();
        
        //-- 时刻对齐 2ms/tick --------------
        state.nextTic(ITS);
    }
    RCLCPP_INFO(node->get_logger(), "END ...");
    rclcpp::shutdown();
	return 0;
}