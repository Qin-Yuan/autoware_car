#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_impl.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp> 
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "CState.h"

class Ros2Info : public rclcpp::Node{
public:
    Ros2Info(CState* state) : Node("vehicle"){
        ex_state = state;
        // 激光雷达
        // mLidarPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("PointCloud2", 1);
        // mScanPublisher = create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
        // mImagePublisher = create_publisher<sensor_msgs::msg::Image>("image_ray", rclcpp::SensorDataQoS().reliable());
        // rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable()
        mImuPublisher = create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/tamagawa/imu_raw_", 10);
        mPoint32Publisher = create_publisher<geometry_msgs::msg::Point32>("vehicle/pose/point32_", 10);
        mOdomtryPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        mClockPublisher = create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);

        targetPosePub = create_publisher<std_msgs::msg::Float32MultiArray>("webots/device/target_pose_", 10);

        /***********订阅者************************/

        mTwistSubscription = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&Ros2Info::onCmdVelReceived, this, std::placeholders::_1));
        
        mJointStateSubscription = create_subscription<sensor_msgs::msg::JointState>(
        "vehicle/pose/joint_states", 1, std::bind(&Ros2Info::onJointStatesReceived, this, std::placeholders::_1));

        mResetSimServer = create_service<std_srvs::srv::Empty>("/webots/reset_sim",  std::bind(&Ros2Info::onResetSimServerCallback, this, std::placeholders::_1, std::placeholders::_2));

        mPubTimer = create_wall_timer(std::chrono::milliseconds(ITS), std::bind(&Ros2Info::publishData, this));        
    };
    
    void publishData(void);
    void subscribeData(void);

private:
    RC_t rcInfo{};
    CState* ex_state;
    
    rclcpp::TimerBase::SharedPtr mPubTimer;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mLidarPublisher;   // 发布lidar消息 多线 点云
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr mScanPublisher;      // 发布lidar消息 单线 激光雷达
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePublisher;         // 发布image图像信息发布
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mImuPublisher;             // 发布IMU消息
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomtryPublisher;       // 发布odom消息
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr mPoint32Publisher;   // 发布位置消息
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetPosePub;  // 发布目标位置
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr mClockPublisher;       // 发布时钟信息

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mTwistSubscription;                  // 订阅Twist控制消息
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr mJointStateSubscription;          // 订阅关节状态消息
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr mMarkerArraySubscription; // 订阅骨骼跟踪消息

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mResetSimServer ;                              // 订阅重置仿真服务
    
    void publishLidarData(void);
    void publishImageDate(void);
    void publishImuData(void);
    void publishOdomData(void);
    void publishClockData(void);
    void publishPoint32Data(float x, float y, float z);
    void publishTargetPose();//发送以IMU为坐标原点，圆柱体以及茶几的坐标

    void onCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg);
    void onJointStatesReceived(const sensor_msgs::msg::JointState::SharedPtr msg);
    void onMarkerArrayReceived(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

    void onResetSimServerCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>  request,
                                        std::shared_ptr<std_srvs::srv::Empty::Response>  response);

    std::map<std::string, double> joint_map;

};