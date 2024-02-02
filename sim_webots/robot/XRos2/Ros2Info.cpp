#include "Ros2Info.hpp"

using namespace std;


void Ros2Info::publishData(void){
    publishLidarData();
    publishImuData();
    publishImageDate();
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
            mScanPublisher = create_publisher<sensor_msgs::msg::LaserScan>("/sensing/lidar/top/scan", 10);
            ex_state->mScan.header.stamp = this->get_clock()->now() ;
            mScanPublisher->publish(ex_state->mScan);
        }
        else if(ex_state->lidar_mode == 2) {
            mLidarPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/top/pointcloud_raw", 10);
            ex_state->mPC2.header.stamp = this->get_clock()->now() ;
            mLidarPublisher->publish(ex_state->mPC2) ;
        }
    }
}

void Ros2Info::publishImageDate(void) {
    if (ex_state->IsCameraAble == 0) {
        ;
    } else if (ex_state->IsCameraAble == 1) {
        mImagePublisher = create_publisher<sensor_msgs::msg::Image>("/sensing/camera/traffic_light/image_raw", rclcpp::SensorDataQoS().reliable());
        ex_state->mImage.header.stamp = this->get_clock()->now() ;
        mImagePublisher->publish(ex_state->mImage) ;
    }
}

void Ros2Info::publishImuData(void) {
    auto msg = sensor_msgs::msg::Imu();
    double angularVelocity[] = {0.0, 0.0, 0.0};
    double linearAcceleration[] = {0.0, 0.0, 0.0};
    msg.header.stamp = now();
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
    // ex_state->armL.jPt = msg->position[2]; // shoulder_left_y;
    // ex_state->armL.jRt = msg->position[0]; // shoulder_left_x;
    // ex_state->armL.jYt = msg->position[4]; // arm_left_1_z;
    // ex_state->armL.jEt = msg->position[6]; // elbow_left_y;
    // ex_state->armL.jyt = msg->position[8]; // arm_left_2_z;
    // ex_state->armL.jWt = msg->position[10]; // wrist_left_y;
    // ex_state->armL.jpt = msg->position[12]; // palm_left_x;
    // ex_state->armL.jft = msg->position[14]; // finger_left_x;

    // ex_state->armR.jPt = msg->position[3]; // shoulder_right_y;
    // ex_state->armR.jRt = msg->position[1]; // shoulder_right_x;
    // ex_state->armR.jYt = msg->position[5]; // arm_right_1_z;
    // ex_state->armR.jEt = msg->position[7]; // elbow_right_y;
    // ex_state->armR.jyt = msg->position[9]; // arm_right_2_z;
    // ex_state->armR.jWt = msg->position[11]; // wrist_right_y;
    // ex_state->armR.jpt = msg->position[13]; // palm_right_x;
    // ex_state->armR.jft = msg->position[15]; // finger_right_x;
    // for(uint32_t i = 0; i < 26; i ++)
    // {
    //     // RCLCPP_INFO(get_logger(), "idx: %d, name: %15s, pos: %f", i, msg->name[i], msg->position[i]);
    //     // std::cout << i << ":" <<msg->name[i]<<"->"<<msg->position[i]<<std::endl;
    //     joint_map[msg->name[i]] = msg->position[i];
    // }
    // ex_state->armL.jPt = joint_map["shoulder_left_y"]; // shoulder_left_y;
    // ex_state->armL.jRt = joint_map["shoulder_left_x"]; // shoulder_left_x;
    // ex_state->armL.jYt = joint_map["arm_left_1_z"]; // arm_left_1_z;
    // ex_state->armL.jEt = joint_map["elbow_left_y"]; // elbow_left_y;
    // ex_state->armL.jyt = joint_map["arm_left_2_z"]; // arm_left_2_z;
    // ex_state->armL.jWt = joint_map["wrist_left_y"]; // wrist_left_y;
    // ex_state->armL.jpt = joint_map["palm_left_x"]; // palm_left_x;
    // ex_state->armL.jft = joint_map["finger_left_x"]; // finger_left_x;

    // ex_state->armR.jPt = joint_map["shoulder_right_y"]; // shoulder_right_y;
    // ex_state->armR.jRt = joint_map["shoulder_right_x"]; // shoulder_right_x;
    // ex_state->armR.jYt = joint_map["arm_right_1_z"]; // arm_right_1_z;
    // ex_state->armR.jEt = joint_map["elbow_right_y"]; // elbow_right_y;
    // ex_state->armR.jyt = joint_map["arm_right_2_z"]; // arm_right_2_z;
    // ex_state->armR.jWt = joint_map["wrist_right_y"]; // wrist_right_y;
    // ex_state->armR.jpt = joint_map["palm_right_x"]; // palm_right_x;
    // ex_state->armR.jft = joint_map["finger_right_x"]; // finger_right_x;
}

// void Ros2Info::onMarkerArrayReceived(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
// {
//     // cout << "Marker Array:" << msg->markers[0].id <<endl;
// }

// ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
// ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
// ros2 run teleop_twist_keyboard teleop_twist_keyboard