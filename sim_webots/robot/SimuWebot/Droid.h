#pragma once
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Keyboard.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Lidar.hpp>

#include "IRobot.h"
#include "CWheels.h"
#include "Config.h"
#include "CState.h"

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <webots/Camera.hpp>

using namespace webots;

class Droid : public IRobot
{
public:
	Droid();
	~Droid() override;

	Supervisor* robot = nullptr;
	Node* robotNode = nullptr;
	Node* imuNode = nullptr;
    Keyboard* keyboard;
    bool IsLidarAble = 1 ;  // 是否打开雷达 0 关 1 开
    bool IsCameraAble = 1 ; // 是否打开相机
	int timeStep = 0;       // the time step of the current world.
	long robotTic = 0;      // 机器人控制时间刻度，单位ms
    // 前轮角度控制
    Motor* rwlr = nullptr;   // 前右侧车轮
	Motor* rwll = nullptr;   // 前左侧车轮
    PositionSensor* rwlrs = nullptr;  // 右侧车轮转向关节
    PositionSensor* rwlls = nullptr;  // 左侧车轮转向关节
    // 后轮速度控制
	Motor* wlr = nullptr;   // 右侧车轮
	Motor* wll = nullptr;   // 左侧车轮 
    PositionSensor* wlrs = nullptr;  // 右侧车轮转向关节
    PositionSensor* wlls = nullptr;  // 左侧车轮转向关节

	InertialUnit* imu = nullptr; // IMU 陀螺仪
	Gyro* gyro = nullptr;        // 陀螺仪
	Accelerometer* acc = nullptr;
    Lidar* lidar = nullptr;      // 1/16线激光雷达
    int lidar_mode = 0 ;         // 单/多线激光雷达标志位 1-单线  2-多线
    int get_lidar_mode(void) ;   // 返回激光雷达类型
    bool get_IsLidarAble(void) ; // 返回是否打开传感器
    bool get_IsCameraAble(void) ;
    sensor_msgs::msg::PointCloud2 mPC2;   // 多线
    sensor_msgs::msg::LaserScan mScan;    // 单线
    webots::Camera* camera;      // 摄像头节点
    sensor_msgs::msg::Image mImage;            // 图像信息
    sensor_msgs::msg::CameraInfo mCameraInfo;  // 摄像头信息, 当有节点订阅该话题时再发布

	//--------- interface ------------------------------------------------------------------------------------------------------
    void iniRobot() override;
    void iniWheels() override;
    void resetRobot() override;
    void testRobot() override;
    uint32_t syncRobotTic() override;
    void nextTic(int ts) override;
    // NPC
    bool NPCEnable = true;
    Node* NPC1 = nullptr;
    const double* NPC1Pose ;
    webots::Field *NPC1positionField = nullptr;
    
    Node* NPC2 = nullptr;
    const double* NPC2Pose ;
    webots::Field *NPC2positionField = nullptr;
    Node* NPC3 = nullptr;
    void initNpc() ;
    void RunNPC() ;

    // 上行数据接口
    void rdLidar(sensor_msgs::msg::LaserScan &pc1);
    
    void rdLidar(sensor_msgs::msg::PointCloud2 &pc2);

    void rdImage(sensor_msgs::msg::Image &img);
    void rdIMU(Vector3d& wImu, Vector3d& aImu, Vector3d& eImu) override;
    void rdImuGndTruth(Vector3d& pigt, Vector3d& vigt, Quaterniond& qigt) override;
    //                         机器人         红杯子         黄杯子          绿杯子         书桌           水杯          床头柜          茶几                   
    void rdTargetPose(Vector3d& rbp,Vector3d& rcp, Vector3d& ycp, Vector3d& gcp, Vector3d& tp, Vector3d& wgp, Vector3d& btp, Vector3d& ttp) override;
    void rdWheels(CWheels& lL, CWheels& lR) override;
    
    void rdRC(RC_t& rc) override;
    
    // 下行数据接口
    void wrWheels(CWheels& lL, CWheels& lR) override;

};

