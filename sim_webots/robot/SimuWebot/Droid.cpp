#include "Droid.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstring>

Droid::Droid()
{
	robot = new Supervisor() ;
    robotNode = robot->getFromDef("vehicle");
	imuNode = robot->getFromDef("inertialunit");
	timeStep = (int)robot->getBasicTimeStep();
    if (NPCEnable) initNpc();
    // std::cout << "tetse2" << std::endl ;
    iniRobot();
    keyboard = new Keyboard();
    keyboard ->enable(timeStep);
//	robotNode->getPosition();
//	robotNode->getCenterOfMass();
}
Droid::~Droid(){delete robot;}

void Droid::initNpc() {
    // NPC1 
    NPC1 = robot->getFromDef("vehicle_npc");
    NPC1positionField = NPC1->getField("translation");
    NPC1Pose = NPC1->getPosition() ;
    if (!NPC1) {
        std::cerr << "Failed to get pedestrian node." << std::endl;
    } else {
        std::cerr << "get pedestrian NPC1 node." << std::endl;
        // std::cerr << NPC1Pose[0] << NPC1Pose[1] << NPC1Pose[2] << std::endl;
    }

    // NPC2 
    NPC2 = robot->getFromDef("vehicle_npc1");
    NPC2positionField = NPC2->getField("translation");
    NPC2Pose = NPC2->getPosition() ;
    if (!NPC2) {
        std::cerr << "Failed to get pedestrian node." << std::endl;
    } else {
        std::cerr << "get pedestrian NPC2 node." << std::endl;
        // std::cerr << NPC2Pose[0] << NPC2Pose[1] << NPC2Pose[2] << std::endl;
    }
}

void Droid::RunNPC() {
    // NPC1 
    static double NPC1newPosition[3] = {NPC1Pose[0], NPC1Pose[1], NPC1Pose[2]};
    static double NPC1Deta = 0.03 ;
    if (NPC1newPosition[1] < NPC1Pose[1] ) {
        NPC1Deta = +abs(NPC1Deta) ;
    } else if (NPC1newPosition[1] > NPC1Pose[1] + 50.0) {
        NPC1Deta = -abs(NPC1Deta) ;
    }
    NPC1newPosition[1] += NPC1Deta ;
    NPC1newPosition[2] = 0.4 ;
    NPC1positionField->setSFVec3f(NPC1newPosition);

    // NPC2 
    static double NPC2newPosition[3] = {NPC2Pose[0], NPC2Pose[1], NPC2Pose[2]};
    static double NPC2Deta = 0.03 ;
    if (NPC2newPosition[0] < NPC2Pose[0]) {
        NPC2Deta = +abs(NPC2Deta) ;
    } else if (NPC2newPosition[0] > NPC2Pose[0] + 50.0) {
        NPC2Deta = -abs(NPC2Deta) ;
    }
    NPC2newPosition[0] += NPC2Deta ;
    NPC2newPosition[2] = 0.4 ;
    NPC2positionField->setSFVec3f(NPC2newPosition);
}

// 机器人基本接口
void Droid::iniRobot()
{
    resetRobot();
    imu = robot->getInertialUnit("inertialunit");
    imu->enable(timeStep);
    gyro = robot->getGyro("gyro");
    gyro->enable(timeStep);
    acc = robot->getAccelerometer("accelerometer");
    acc->enable(timeStep);
    lidar = robot->getLidar("Laser");
    camera = robot->getCamera("camera");
    // 是否打开摄像头
    if (IsCameraAble == 0) {
        camera->disable();
    } else if(IsCameraAble == 1) {
        camera->enable(timeStep);
        // 摄像头初始化 mImage
        mImage.header.frame_id = "traffic_light_left_camera/camera_link";
        mImage.height = camera->getHeight();
        mImage.width = camera->getWidth();
        mImage.is_bigendian = false;
        mImage.step = sizeof(unsigned char) * 4 * camera->getWidth();
        mImage.data.resize(4 * camera->getWidth() * camera->getHeight());
        mImage.encoding = sensor_msgs::image_encodings::BGRA8;
        // 摄像头初始化 cameraInfo
        mCameraInfo.header.frame_id = "traffic_light_left_camera/camera_link";
        mCameraInfo.height = camera->getHeight();
        mCameraInfo.width = camera->getWidth();
        mCameraInfo.distortion_model = "plumb_bob";
        // Convert FoV to focal length.
        // Reference: https://en.wikipedia.org/wiki/Focal_length#In_photography
        const double diagonal = sqrt(pow(camera->getWidth(), 2) + pow(camera->getHeight(), 2));
        const double focalLength =  0.5 * diagonal * (cos(0.5 * camera->getFov()) / sin(0.5 * camera->getFov()));

        mCameraInfo.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        mCameraInfo.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        mCameraInfo.k = {
            focalLength, 0.0, (double)camera->getWidth() / 2,
            0.0, focalLength, (double)camera->getHeight() / 2,
            0.0, 0.0, 1.0};
        mCameraInfo.p = {
            focalLength, 0.0, (double)camera->getWidth() / 2, 0.0,
            0.0, focalLength, (double)camera->getHeight() / 2, 0.0,
            0.0, 0.0, 1.0, 0.0};
    }
    // 是否打开激光雷达
    if (IsLidarAble == 0) {
        lidar->disable();
    } else if(IsLidarAble == 1) {
        // lidar = robot->getLidar("lidar"); 
        // 单线激光雷达
        if(lidar->getNumberOfLayers() == 1) {
            lidar->enable(timeStep);   // timeStep  timeStep
            lidar_mode = 1 ;
            const int resolution = lidar->getHorizontalResolution();
            mScan.header.frame_id = "laser_link";
            mScan.angle_increment = -lidar->getFov() / (resolution - 1);
            mScan.angle_min = lidar->getFov() / 2.0;
            mScan.angle_max = -lidar->getFov() / 2.0;
            mScan.time_increment = (double)lidar->getSamplingPeriod() / (1000.0 * resolution);
            mScan.scan_time = (double)lidar->getSamplingPeriod() / 1000.0;
            mScan.range_min = lidar->getMinRange();
            mScan.range_max = lidar->getMaxRange();
            mScan.ranges.resize(resolution);
        }
        // 多线激光雷达
        else if(lidar->getNumberOfLayers() > 1) {
            lidar->enable(timeStep);
            // lidar->enablePointCloud();
            lidar_mode = 2 ;
            mPC2.header.frame_id = "velodyne_top_base_link";
            mPC2.height = lidar->getNumberOfLayers();
            mPC2.width = lidar->getHorizontalResolution();
            // webots 激光点云默认包含  float x;  float y;  float z;  int layer_id;  float time; 因此为 20，这已经是最小了
            mPC2.point_step = 22;
            // 一行点云数据的字节数
            mPC2.row_step = mPC2.point_step * mPC2.width ;
            mPC2.is_dense = true;
            mPC2.fields.resize(6);
            mPC2.fields[0].name = "x";
            mPC2.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            mPC2.fields[0].count = 1;
            mPC2.fields[0].offset = 0;
            mPC2.fields[1].name = "y";
            mPC2.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            mPC2.fields[1].count = 1;
            mPC2.fields[1].offset = 4;
            mPC2.fields[2].name = "z";
            mPC2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            mPC2.fields[2].count = 1;
            mPC2.fields[2].offset = 8;
            mPC2.fields[3].name = "intensity";
            mPC2.fields[3].offset = 12;
            mPC2.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
            mPC2.fields[3].count = 1;
            mPC2.fields[4].name = "ring";
            mPC2.fields[4].offset = 16;
            mPC2.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
            mPC2.fields[4].count = 1;
            mPC2.fields[5].name = "time";
            mPC2.fields[5].offset = 18;
            mPC2.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
            mPC2.fields[5].count = 1;
            mPC2.is_bigendian = false;
            // 一帧点云的二进制大小
            mPC2.data.resize(mPC2.row_step * mPC2.height);
        }
    }
    iniWheels();
}
void Droid::iniWheels()
{   
    // 前驱动角度控制轮子
    rwll = robot->getMotor("left_steer") ;
    rwll->setAvailableTorque(10);
    rwll->setControlPID(30, 0, 0);
    rwll->setPosition(0.0);
    // rwll->setPosition(INFINITY) ;
    // rwll->setVelocity(1.0);
    rwlls = robot->getPositionSensor("left_steer_sensor");
    rwlls->enable(timeStep);

    rwlr = robot->getMotor("right_steer") ;
    rwlr->setAvailableTorque(10);
    rwlr->setControlPID(30, 0, 0);
    rwlr->setPosition(0.0);
    rwlrs = robot->getPositionSensor("right_steer_sensor");
    rwlrs->enable(timeStep);

    // 后驱动速度控制轮子
    wlr = robot->getMotor("right_rear_wheel");   // 右轮
    wlr->setPosition(INFINITY) ;
    wlr->setAcceleration(10);
    wlr->setAvailableTorque(10);
    wlr->setControlPID(30, 0, 0);
    wlrs = robot->getPositionSensor("right_rear_sensor");   // 右轮向关节
    wlrs->enable(timeStep);

    wll = robot->getMotor("left_rear_wheel");    // 左轮
    wll->setPosition(INFINITY) ;
    wll->setAcceleration(10);
    wll->setAvailableTorque(10);
    wll->setControlPID(30, 0, 0);
    wlls = robot->getPositionSensor("left_rear_sensor");    // 左轮向关节
    wlls->enable(timeStep);
}

void Droid::resetRobot()
{
    robot->simulationReset();
    robot->step(2);
    robot->simulationReset();
}

void Droid::testRobot()
{
    double speed = 1.0;
    wlr->setVelocity(speed);
    wll->setVelocity(speed);
    while (robot->step(timeStep) != 1000) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();
        // Process sensor data here.
        // Enter here functions to send actuator commands, like:
        //  motor->setPosition(10.0);
    }
    wlr->setVelocity(0);
    wll->setVelocity(0);
}
void Droid::nextTic(int ts)
{
    robotTic += ts;
    robot->step(ts);
}
uint32_t Droid::syncRobotTic()
{
    return robotTic;
}
// 上行数据接口
/*****          lidar
                              *******/
// 返回
int Droid::get_lidar_mode(void) {
    return lidar_mode ;
}
bool Droid::get_IsLidarAble(void) {
    return IsLidarAble ;
}
// 单线激光雷达
void Droid::rdLidar(sensor_msgs::msg::LaserScan &pc1)
{
    lidar->enable(timeStep);   // timeStep  timeStep
    auto rangeImage = lidar->getLayerRangeImage(0);
    if (rangeImage)
    {
        memcpy(mScan.ranges.data(), rangeImage, mScan.ranges.size() * sizeof(float));
        // mScan.header.stamp = mNode->get_clock()->now();
        pc1 = mScan ;
    }
}
// 多线激光雷达
void Droid::rdLidar(sensor_msgs::msg::PointCloud2 &pc2)
{   
    lidar->enablePointCloud();
    auto data = lidar->getPointCloud();
    if (data)
    {
        uint16_t point_count = mPC2.width * mPC2.height ;
        uint16_t error_point_count = 0 ;
        // 编辑所有点云
        for (size_t point_num = 0 ; point_num < point_count ; ++point_num) {
            // if (data[point_num].layer_id < 12) continue;
            // if (point_num == 2000) {
            //     std::cout << data[point_num].x << std::endl;
            // }
            float x = data[point_num].x ;
            float y = data[point_num].y ;
            float z = data[point_num].z ;
            float intensity = 89 ;
            // fields 属性写入 x y z i r t
            // 检查是否包含NaN值
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                // nanf("") 也会报错
                // x = 0.0 ;
                // y = 0.0 ;
                // z = -1.8 ;
                // intensity = 0.0 ;
                // std::cout << " 发现异常值 " << x << std::endl;
                error_point_count += 1 ;
                continue;
            }
            // if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(data[point_num].layer_id) || !std::isfinite(data[point_num].time)) {
            //     std::cout << " 发现异常值 " << x << std::endl;
            // }
            // x
            uint32_t offset = (point_num-error_point_count) * mPC2.point_step + mPC2.fields[0].offset;
            *reinterpret_cast<float*>(&mPC2.data[offset]) = x ;
            // y
            offset = (point_num-error_point_count) * mPC2.point_step + mPC2.fields[1].offset;
            *reinterpret_cast<float*>(&mPC2.data[offset]) = y ;
            // z
            offset = (point_num-error_point_count) * mPC2.point_step + mPC2.fields[2].offset;
            *reinterpret_cast<float*>(&mPC2.data[offset]) = z ;
            // i 点云强度给固定值
            offset = (point_num-error_point_count) * mPC2.point_step + mPC2.fields[3].offset;
            *reinterpret_cast<float*>(&mPC2.data[offset]) = intensity ;
            // r 从上到下： 0-layer_max
            offset = (point_num-error_point_count) * mPC2.point_step + mPC2.fields[4].offset;
            *reinterpret_cast<uint16_t*>(&mPC2.data[offset]) = data[point_num].layer_id ;
            // std::cout << data[point_num].layer_id << std::endl;
            // t
            offset = (point_num-error_point_count) * mPC2.point_step + mPC2.fields[5].offset;
            // *reinterpret_cast<float*>(&mPC2.data[offset]) = data[point_num].time ;
            *reinterpret_cast<float*>(&mPC2.data[offset]) = 0.0 ;
        }
        // 一帧点云的二进制大小
        // mPC2.data.resize(point_count-error_point_count);
        std::cout << point_count << " - " << mPC2.data.size() / mPC2.point_step << std::endl;
        // std::cout << mPC2.width * mPC2.height << " " << mPC2.fields.size() << " " << data[28789].layer_id << std::endl;
        pc2 = mPC2;
    }
}

/*****          camera
                              *******/

bool Droid::get_IsCameraAble(void) {
    return IsCameraAble ;
}
// 读取图像信息
void Droid::rdImage(sensor_msgs::msg::Image &img) {
    camera->enable(timeStep);
    auto image = camera->getImage();
    if(image) {
        memcpy(mImage.data.data(), image, mImage.data.size());
        img = mImage ;
    }
    // sleep(0.03);  // 直接加延时没用
}

/*****          rdIMU
                              *******/
void Droid::rdIMU(Vector3d& wImu, Vector3d& aImu, Vector3d& eImu)
{//采集数据标准IMU坐标系：x向前，y向右，z向下
    wImu[0] =  gyro->getValues()[0];
    wImu[1] = -gyro->getValues()[1];
    wImu[2] = -gyro->getValues()[2];
    aImu[0] =  acc->getValues()[0];
    aImu[1] = -acc->getValues()[1];
    aImu[2] = -acc->getValues()[2];
    // 融合后的角度坐标系，x向前，y向左，z向上
    eImu[0] =  imu->getRollPitchYaw()[0]; // roll
    eImu[1] =  imu->getRollPitchYaw()[1]; // pitch
    eImu[2] =  imu->getRollPitchYaw()[2]; // yaw

    // NPC
    if (NPCEnable) RunNPC();
}
void Droid::rdImuGndTruth(Vector3d& pigt, Vector3d& vigt, Quaterniond& qigt)
{
    const double* r;
    r = imuNode->getPosition();  pigt << r[0], r[1], r[2];
    r = imuNode->getOrientation();
    Matrix3d R;
    R << r[0], r[1], r[2],
            r[3], r[4], r[5],
            r[6], r[7], r[8];
    Matrix3d Rz;
    Rz <<  0,  -1,   0, // 纠正偏航的旋转矩阵
            1,   0,   0,
            0,   0,   1;
    qigt = Rz*R;
    float yaw = imu->getRollPitchYaw()[2]; // yaw
    float cyaw = cosf(yaw), syaw = sinf(yaw);
    R << cyaw,  syaw,   0, // 纠正偏航的旋转矩阵
            -syaw,  cyaw,   0,
            0,      0,    1;
    r = imuNode->getVelocity(); // 在世界坐标系下的速度
    vigt << (float)r[0], (float)r[1], (float)r[2];
    vigt = R * vigt;
}

void Droid::rdTargetPose(Vector3d& rbp,Vector3d& rcp, Vector3d& ycp, Vector3d& gcp, Vector3d& tp, Vector3d& wgp, Vector3d& btp, Vector3d& ttp)
{
    // Node* rbtNode = robot->getFromDef("m3k4a");
    // Node* cyc1Node = robot->getFromDef("redcyc");
    // Node* cyc2Node = robot->getFromDef("yellowcyc");
    // Node* cyc3Node = robot->getFromDef("greencyc");
    // Node* tableNode = robot->getFromDef("rtable");      // rtable glass  btable ttable
    // Node* waterglassNode = robot->getFromDef("glass");
    // Node* bedsidetableNode = robot->getFromDef("btable");
    // Node* teatableNode = robot->getFromDef("ttable");
    // Eigen::Vector3d rbtPose,cyc1Pose,cyc2Pose,cyc3Pose,tablePose,wgPose,btPose,ttPose;
    // for(uint8_t r = 0;r<3;r++)
    // {
    //     rbtPose(r) = *(rbtNode->getPosition()+r);
    //     cyc1Pose(r) = *(cyc1Node->getPosition()+r);
    //     cyc2Pose(r) = *(cyc2Node->getPosition()+r);
    //     cyc3Pose(r) = *(cyc3Node->getPosition()+r);
    //     tablePose(r) = *(tableNode->getPosition()+r);
    //     wgPose(r) = *(waterglassNode->getPosition()+r);
    //     btPose(r) = *(bedsidetableNode->getPosition()+r);
    //     ttPose(r) = *(teatableNode->getPosition()+r);
    // }
    // rbp = rbtPose;
    // rcp = cyc1Pose;
    // ycp = cyc2Pose;
    // gcp = cyc3Pose;
    // tp = tablePose;
    // wgp = wgPose;
    // btp = btPose;
    // ttp = ttPose;
    //cout<<"位置："<<tp<<endl;
}

void Droid::rdWheels(CWheels& lL, CWheels& lR)
{
    lR.wl_v = (float)wlr->getVelocity();
    lR.wl_p = (float)wlrs->getValue();

    lL.wl_v = (float)wll->getVelocity();
    lL.wl_p = (float)wlls->getValue();
    // std::cout << lR.wl_v << " : " << lR.wl_p << " - " << lL.wl_v << " : " << lL.wl_p << " - " << timeStep << std::endl;
}

void Droid::rdRC(RC_t& rc){
    // rc.vx = _bnd(rc.vx, -1.0f, 1.0f);
    // rc.vy = _bnd(rc.vy, -1.0f, 1.0f);
    // rc.wz = _bnd(rc.wz, -1.0f, 1.0f);
    // ?
    rc.vx = rc.vx ;
    rc.vy = rc.vy ;
    rc.wz = rc.wz ;
};

// 下行数据接口
void Droid::wrWheels(CWheels& lL, CWheels& lR)
{	
    wll->setPosition(INFINITY) ;
    wlr->setPosition(INFINITY) ;
    wll->setVelocity(lL.wl_cmd_v);
    wlr->setVelocity(lR.wl_cmd_v);

    rwll->setPosition(lL.wl_cmd_p) ;
    rwlr->setPosition(lR.wl_cmd_p) ;
    // rwll->setVelocity(INFINITY);
    // rwlr->setVelocity(INFINITY);
}


