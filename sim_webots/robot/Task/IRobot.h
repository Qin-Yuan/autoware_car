#pragma once
#include "CWheels.h"
#include "mydef.h"
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>


// 机器人硬件访问接口，面向虚拟机器人和实际机器人
class IRobot
{
public:
    // 机器人基本接口
    virtual void iniRobot()=0;
    virtual void iniWheels()=0;
    virtual void resetRobot() = 0;
    virtual void testRobot() = 0;
    virtual uint32_t syncRobotTic() = 0;
    virtual void nextTic(int ts) = 0;
    // 上行数据接口
    virtual void rdLidar(sensor_msgs::msg::LaserScan &pc1) = 0;
    virtual void rdLidar(sensor_msgs::msg::PointCloud2 &pc2) = 0;
    virtual int  get_lidar_mode(void) = 0 ;
    virtual bool get_IsLidarAble(void) = 0 ;
    virtual bool get_IsCameraAble(void) = 0 ;
    virtual void rdImage(sensor_msgs::msg::Image &img) = 0;
    virtual void rdIMU(Vector3d& wImu, Vector3d& aImu, Vector3d& eImu) = 0;
    virtual void rdImuGndTruth(Vector3d& pigt, Vector3d& vigt, Quaterniond& qgt) = 0;
    virtual void rdTargetPose(Vector3d& rbp,Vector3d& rcp, Vector3d& ycp, Vector3d& gcp, Vector3d& tp, Vector3d& wgp, Vector3d& btp, Vector3d& ttp) = 0;
    virtual void rdWheels(CWheels& lL, CWheels& lR) = 0;
    virtual void rdRC(RC_t& rc) = 0;
    // 下行数据接口
    virtual void wrWheels(CWheels& lL, CWheels& lR) = 0;
    // 环境交互接口
	virtual ~IRobot() = default;
};

