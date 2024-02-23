//
// Created by liqd on 2022/5/12.
//
#ifndef PADROID_CSTATE_H
#define PADROID_CSTATE_H
#include "Config.h"
#include "CWheels.h"
#include "IRobot.h"
#include "Droid.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

// 机器人状态数据的更新、存储、收发
// 单位标准化, 长度m，时间s，角rad，质量kg
class CState {
public:
    CState();
    ~CState();

    IRobot* rbt = nullptr;  // 机器人接口

    //========= Robot状态信息=============================
    uint32_t sysTic = 0; // 单位ms，以ITS为刻度对齐
    RC_t rc{}; // 遥控器指令
    CWheels lL; // 左右轮当前值：当前速度、位置 ； 目标速度
    CWheels lR; //
    Vector3d wImu; // IMU angular velocity
    Vector3d aImu; // IMU linear acceleration

    //--------- IMU ground truth 可能涉及偏航的问题 ------------
    Vector3d pigt;    // position imu ground truth
    Vector3d vigt;    // velocity imu ground truth
    Quaterniond qigt; // IMU姿态4元数 ground truth
    Vector3d eImu; // IMU欧拉角度
    //-------- 以IMU为坐标原点的目标位置 ----------
    Vector3d rbp; // robot pose
    Vector3d rcp; // red cyclinder pose
    Vector3d ycp; // yellow cyclinder pose
    Vector3d gcp; // green cyclinder pose
    Vector3d tp;  // table pose
    Vector3d wgp; // water glass poss
    Vector3d btp; // Bedside table poss
    Vector3d ttp; // Tea table poss


    //雷达  
    int lidar_mode = 0 ;
    bool IsLidarAble = 0 ;  // 是否打开雷达
    bool IsCameraAble = 0 ; // 是否打开相机
    sensor_msgs::msg::LaserScan mScan;
    sensor_msgs::msg::PointCloud2 mPC2;
    // 摄像头数据
    sensor_msgs::msg::Image mImage;  // 图像信息
    void rdRobot();   // 机器人为参考系，CoM为原点
    void wrRobot();
    void nextTic(int ts=ITS);
    bool FailCheck();

    //TODO 参数系统外部配置
    //======= 参数设置信息 ==============================
    //在机械坐标系下b，坐标原点是左右s轴中心的中心，前x，右y，下z
    Vector3d CoMb; // CoM在机器人坐标系下的位置
    Vector3d IMUb; // IMU在机器人坐标系下的位置
    Vector3d ICoM; // 以CoM为中心的转动惯量，不存在正负问题

    void iniPara();

};
#endif //PADROID_CSTATE_H
