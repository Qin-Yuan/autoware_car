#include "Ros2Info.hpp"

using namespace std;


void Ros2Info::publishData(void){
    publishLidarData();
    publishImuData();
    publishImageDate();
    publishOdomData();
    publishClockData() ;
    publishPoint32Data(ex_state->pigt[0], ex_state->pigt[1], ex_state->pigt[2]);
    // publishTargetPose();
}

void Ros2Info::subscribeData(void){
    ex_state->rc.vx = rcInfo.vx;
    ex_state->rc.vy = rcInfo.vy;
    ex_state->rc.wz = rcInfo.wz;    
}

void Ros2Info::publishLidarData(void) {
    if (ex_state->IsLidarAble == 0) {
        ;
    } else if (ex_state->IsLidarAble == 1) {
        if (ex_state->lidar_mode == 1) {
            mScanPublisher = create_publisher<sensor_msgs::msg::LaserScan>("/sensing/lidar/top/scan_", 10);
            ex_state->mScan.header.stamp = this->get_clock()->now() ;
            mScanPublisher->publish(ex_state->mScan);
        }
        else if(ex_state->lidar_mode == 2) {
            mLidarPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/top/pointcloud_raw_", 10);
            ex_state->mPC2.header.stamp = this->get_clock()->now() ;
            mLidarPublisher->publish(ex_state->mPC2) ;
        }
    }
}

void Ros2Info::publishImageDate(void) {
    if (ex_state->IsCameraAble == 0) {
        ;
    } else if (ex_state->IsCameraAble == 1) {
        mImagePublisher = create_publisher<sensor_msgs::msg::Image>("/sensing/camera/traffic_light/image_raw_", rclcpp::SensorDataQoS().reliable());
        ex_state->mImage.header.stamp = this->get_clock()->now() ;
        mImagePublisher->publish(ex_state->mImage) ;
    }
}

void Ros2Info::publishClockData(void) {
    auto msg = rosgraph_msgs::msg::Clock() ;
    msg.clock = this->get_clock()->now();
    mClockPublisher->publish(msg) ;
}

void Ros2Info::publishImuData(void) {
    auto msg = sensor_msgs::msg::Imu();
    double angularVelocity[] = {0.0, 0.0, 0.0};
    double linearAcceleration[] = {0.0, 0.0, 0.0};
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "tamagawa/imu_link";
    msg.orientation.x =  ex_state->qigt.x();
    msg.orientation.y =  ex_state->qigt.y();
    msg.orientation.z =  ex_state->qigt.z();
    msg.orientation.w =  ex_state->qigt.w();
    msg.angular_velocity.x = ex_state->wImu[0];
    msg.angular_velocity.y = -ex_state->wImu[1];
    msg.angular_velocity.z = -ex_state->wImu[2];
    msg.linear_acceleration.x = ex_state->aImu[0];
    msg.linear_acceleration.y = -ex_state->aImu[1];
    msg.linear_acceleration.z = -ex_state->aImu[2];
    //printf("%6.3f %6.3f %6.3f %6.3f\n",msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);

    mImuPublisher->publish(msg);
}


void Ros2Info::publishOdomData(void) {
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->get_clock()->now() ;
    msg.header.frame_id = "base_link" ;
    // 机器人位置
    msg.pose.pose.position.x = ex_state->pigt[0] ;
    msg.pose.pose.position.y = ex_state->pigt[1] ;
    msg.pose.pose.position.z = ex_state->pigt[2] ;
    msg.pose.pose.orientation.x = ex_state->qigt.x() ;
    msg.pose.pose.orientation.y = ex_state->qigt.y() ;
    msg.pose.pose.orientation.z = ex_state->qigt.z() ;
    msg.pose.pose.orientation.w = ex_state->qigt.w() ;
    // 控制指令
    msg.twist.twist.linear.x = ex_state->rc.vx ;
    msg.twist.twist.linear.y = ex_state->rc.vy ;
    msg.twist.twist.linear.z = 0.0 ;
    msg.twist.twist.angular.x = 0.0 ;
    msg.twist.twist.angular.y = 0.0 ;
    msg.twist.twist.angular.z = ex_state->rc.wz ;
    mOdomtryPublisher->publish(msg);
}

void Ros2Info::publishPoint32Data(float x, float y, float z){
    auto msg = geometry_msgs::msg::Point32();
    msg.x = -(y-5.55169);
    msg.y = x+4.30827;
    // msg.z = z-0.11285;
    msg.z = 0 ;
    mPoint32Publisher->publish(msg);
    // printf("pos0:%6.3f %6.3f %6.3f\n",msg.x,msg.y,msg.z);
}
void Ros2Info::publishTargetPose()
{
    // auto msg = std_msgs::msg::Float32MultiArray();
    
    // Matrix3d Rz;
    // Rz <<   0,  -1,   0,  // 机器人与原始坐标系的旋转矩阵
    //         1,   0,   0,
    //         0,   0,   1;
    // Eigen::Vector3d map(-3.89005,5.33988, 0); // map坐标系原点
    // // ---------以base_like为参考坐标系----------
    // Eigen::Vector3d rcp,ycp,gcp,tp,wgp,btp,ttp;
    // rcp = Rz*(ex_state->rcp-ex_state->rbp);
    // ycp = Rz*(ex_state->ycp-ex_state->rbp);
    // gcp = Rz*(ex_state->gcp-ex_state->rbp);
    // tp = Rz*(ex_state->tp-ex_state->rbp);
    // wgp = Rz*(ex_state->wgp-ex_state->rbp);
    // btp = Rz*(ex_state->btp-ex_state->rbp);
    // ttp = Rz*(ex_state->ttp-ex_state->rbp);
    // // ---------以map为参考坐标系----------
    // Eigen::Vector3d rcp1,ycp1,gcp1,tp1,wgp1,btp1,ttp1;
    // rcp1 =  Rz*(ex_state->rcp-map);
    // ycp1 = Rz*(ex_state->ycp-map);
    // gcp1 = Rz*(ex_state->gcp-map);
    // tp1 = Rz*(ex_state->tp-map);
    // wgp1 = Rz*(ex_state->wgp-map);
    // btp1 = Rz*(ex_state->btp-map);
    // ttp1 = Rz*(ex_state->ttp-map);
    // // --------发布顺序位：红、黄、绿圆柱，书桌，水杯，床头柜，茶几的x、y、z--------
    // msg.data = {rcp1(0),rcp1(1),rcp1(2),ycp1(0),ycp1(1),ycp1(2),        // [0:3] [3:6]
    //             gcp1(0),gcp1(1),gcp1(2),tp1(0),tp1(1),tp1(2),           // [6:9] [9:12]
    //             wgp1(0),wgp1(1),wgp1(2),btp1(0),btp1(1),btp1(2),        // [12:15] [15:18]
    //             ttp1(0),ttp1(1),ttp1(2)};                               // [18:21]
    // targetPosePub->publish(msg);
    //printf("红色圆柱位置：%6.3f %6.3f %6.3f\n",rcp1(0),rcp1(1),rcp1(2));
}

void Ros2Info::onCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    ex_state->rc.vx = msg->linear.x;
    ex_state->rc.vy = msg->linear.y;
    ex_state->rc.wz = msg->angular.z;
    // RCLCPP_INFO(get_logger(), "cmd_vel: %5.2f, %5.2f, %5.2f", ex_state->rc.vx, ex_state->rc.vy, ex_state->rc.wz);
}

void Ros2Info::onResetSimServerCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>  request,
                                            std::shared_ptr<std_srvs::srv::Empty::Response>  response)
{
    ex_state->rbt->iniRobot() ;
    ex_state->rc.vx = 0.0 ;
    ex_state->rc.vy = 0.0 ;
    ex_state->rc.wz = 0.0 ;
    RCLCPP_INFO(get_logger(), "RESET SIM ROBOT ...");
    // ex_state->rbt->initRobot() ;
}

void Ros2Info::onJointStatesReceived(const sensor_msgs::msg::JointState::SharedPtr msg)
{
}