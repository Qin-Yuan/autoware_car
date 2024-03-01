#include "CState.h"

CState::CState()
{
    rc.vx=0; rc.vy=0; rc.wz=0;
    rbt = new Droid();
    rbt->iniRobot();     // 初始化
    lidar_mode = rbt->get_lidar_mode();    // 获取激光雷达类型 1-单线 2-多线
    IsLidarAble = rbt->get_IsLidarAble() ;
    IsCameraAble = rbt->get_IsCameraAble() ;
    rbt->syncRobotTic();
    rbt->nextTic(ITS);   // 向前迭代一步
    iniPara();
}
CState::~CState() {
    delete(rbt);
}
void CState::rdRobot()
{
    //--DOF位置/电流/速度, 陀螺仪,加速度计 , 雷达，图像
    if (IsLidarAble == 0) {
        ;
    } else if(IsLidarAble == 1) {
        if (lidar_mode == 1) {
            rbt->rdLidar(mScan) ;
        }
        else if(lidar_mode == 2) {
            rbt->rdLidar(mPC2);
        }
    }
    if (IsCameraAble == 0) {
        ;
    } else if(IsCameraAble == 1) {
        rbt->rdImage(mImage);
    }
    rbt->rdIMU(wImu, aImu, eImu);
    rbt->rdImuGndTruth(pigt, vigt, qigt);
    // rbt->rdTargetPose(rbp,rcp,ycp,gcp,tp,wgp,btp,ttp);
    rbt->rdWheels(lL, lR);
    rbt->rdRC(rc);
    // 读取仿真时间 , 除以 100 单位 秒
    sim_time = rbt->syncRobotTic() ;
    std::cout << sim_time << std::endl;
}

void CState::wrRobot()
{
    rbt->wrWheels(lL, lR);
}

void CState::nextTic(int ts)
{
    rbt->nextTic(ts);
    sysTic += ts;
}

bool CState::FailCheck()
{
    return false;
}
void CState::iniPara()
{
    CoMb << 0.005,   -0,    -0.291;  // CoM的位置
    IMUb << 0.0,     -0,    -0.111;  // IMU的位置;
}

