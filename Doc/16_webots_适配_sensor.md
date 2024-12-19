### webots 适配 sensor

新建文件夹说明 ：

- **sim_webots** ：存放autoware_webots的仿真文件夹 
  - **autoware_webots_individual_params** ：车辆 tf2 参数配置文件，官方 sample_sensor_kit 和 awsim_sensor_kit 两个 
  - **autoware_webots_launch** ：autoware启动launch文件
  - **autoware_webots_utils** ：适配 sensor 、vehicle 和 webots 连接的一些功能节点 

基于webots仿真环境中运行autoware导航程序，区别于之前一小节在rviz2 中的导航程序，用于趋近于真实的仿真环境 ，这里主要参考官方的模拟器 awsim 发布的话题和订阅的话题有哪些 ，按照这些话题进行模拟发布或者订阅；查看官方的 awsim 仿真器启动的传感器数据话题有 ：

指令 ： **ros2 topic list -v -t** 

```clojure
Published topics:
 * /clock [rosgraph_msgs/msg/Clock] 1 publisher
 * /parameter_events [rcl_interfaces/msg/ParameterEvent] 1 publisher
 * /rosout [rcl_interfaces/msg/Log] 1 publisher
 * /sensing/camera/traffic_light/camera_info [sensor_msgs/msg/CameraInfo] 1 publisher
 * /sensing/camera/traffic_light/image_raw [sensor_msgs/msg/Image] 1 publisher
 * /sensing/gnss/pose [geometry_msgs/msg/PoseStamped] 1 publisher
 * /sensing/gnss/pose_with_covariance [geometry_msgs/msg/PoseWithCovarianceStamped] 1 publisher
 * /sensing/imu/tamagawa/imu_raw [sensor_msgs/msg/Imu] 1 publisher
 * /sensing/lidar/top/pointcloud_raw [sensor_msgs/msg/PointCloud2] 1 publisher
 * /sensing/lidar/top/pointcloud_raw_ex [sensor_msgs/msg/PointCloud2] 1 publisher
 * /vehicle/status/control_mode [autoware_auto_vehicle_msgs/msg/ControlModeReport] 1 publisher
 * /vehicle/status/gear_status [autoware_auto_vehicle_msgs/msg/GearReport] 1 publisher
 * /vehicle/status/hazard_lights_status [autoware_auto_vehicle_msgs/msg/HazardLightsReport] 1 publisher
 * /vehicle/status/steering_status [autoware_auto_vehicle_msgs/msg/SteeringReport] 1 publisher
 * /vehicle/status/turn_indicators_status [autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport] 1 publisher
 * /vehicle/status/velocity_status [autoware_auto_vehicle_msgs/msg/VelocityReport] 1 publisher

Subscribed topics:
 * /control/command/control_cmd [autoware_auto_control_msgs/msg/AckermannControlCommand] 1 subscriber
 * /control/command/emergency_cmd [tier4_vehicle_msgs/msg/VehicleEmergencyStamped] 1 subscriber
 * /control/command/gear_cmd [autoware_auto_vehicle_msgs/msg/GearCommand] 1 subscriber
 * /control/command/hazard_lights_cmd [autoware_auto_vehicle_msgs/msg/HazardLightsCommand] 1 subscriber
 * /control/command/turn_indicators_cmd [autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand] 1 subscriber
```



### 一、传感器数据

```clojure
 * /sensing/camera/traffic_light/camera_info [sensor_msgs/msg/CameraInfo] 1 publisher
 * /sensing/camera/traffic_light/image_raw [sensor_msgs/msg/Image] 1 publisher
 * /sensing/gnss/pose [geometry_msgs/msg/PoseStamped] 1 publisher
 * /sensing/gnss/pose_with_covariance [geometry_msgs/msg/PoseWithCovarianceStamped] 1 publisher
 * /sensing/imu/tamagawa/imu_raw [sensor_msgs/msg/Imu] 1 publisher
 * /sensing/lidar/top/pointcloud_raw [sensor_msgs/msg/PointCloud2] 1 publisher
 * /sensing/lidar/top/pointcloud_raw_ex [sensor_msgs/msg/PointCloud2] 1 publisher
```

包括激光雷达、IMU、相机、GNSS等通用传感器，下表是autoware.universe中默认的传感器话题和frame_id，可以安装已经订阅的方式配置 ：

|     sensor     |                  topic                  |                    type                     |             frame_id (适当做了修改)             | rate(hz) |
| :------------: | :-------------------------------------: | :-----------------------------------------: | :---------------------------------------------: | :------: |
|   imu(订阅)    |      /sensing/imu/tamagawa/imu_raw      |             sensor_msgs/msg/Imu             |                tamagawa/imu_link                |    60    |
|  Lidar (订阅)  |    /sensing/lidar/top/pointcloud_raw    |         sensor_msgs/msg/PointCloud2         | velodyne_top_base_link (gazebo仿真使用这个数据) |    6     |
|     Lidar      |  /sensing/lidar/top/pointcloud_raw_ex   |         sensor_msgs/msg/PointCloud2         |             velodyne_top_base_link              |          |
|      gnss      |           /sensing/gnss/pose            |        geometry_msgs/msg/PoseStamped        |                    gnss_link                    |          |
|  gnss（订阅）  |   /sensing/gnss/pose_with_covariance    | geometry_msgs/msg/PoseWithCovarianceStamped |                    gnss_link                    |    60    |
| camera（订阅） | /sensing/camera/traffic_light/image_raw |            sensor_msgs/msg/Image            |      traffic_light_left_camera/camera_link      |    6     |

其中 webots 也没有插件gnss，这里可以订阅gazebo发布odom数据再发布gnss话题，新建 odom_to_gnss.py 文件，写入以下内容 ：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
class odom_to_gnss(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("odom msg to gnss .")
        self.gnss_msg = PoseWithCovarianceStamped()        
        self.odom_msg = Odometry()
        self.map_pose_gnss_init =[-11.17,-0.12,0.0] 
        self.odom_sub = self.create_subscription(Odometry,"odom",self.odom_sub_callback,10)
        self.gnss_sub = self.create_publisher(PoseWithCovarianceStamped,"/sensing/gnss/pose_with_covariance",10)
        # self.timer = self.create_timer(0.01, self.timer_callback)
        
    def odom_sub_callback(self,date):
        self.odom_msg = date
        self.timer_callback()
        
    def timer_callback(self) :
        self.gnss_msg.header.frame_id = "gnss_link"
        self.gnss_msg.header.stamp = self.get_clock().now().to_msg()
        self.gnss_msg.pose.pose.position.x = self.odom_msg.pose.pose.position.x - self.map_pose_gnss_init[0]
        self.gnss_msg.pose.pose.position.y = self.odom_msg.pose.pose.position.y - self.map_pose_gnss_init[1]
        self.gnss_msg.pose.pose.position.z = self.odom_msg.pose.pose.position.z - self.map_pose_gnss_init[2]
        self.gnss_sub.publish(self.gnss_msg)
        
def main(args=None):
    rclpy.init(args=args)			    
    odom_to_gnss_node = odom_to_gnss("odom_to_gnss_node")    
    rclpy.spin(odom_to_gnss_node)                 
    rclpy.shutdown()

main()
```



### ERROR 

按照官方给的 https://autowarefoundation.github.io/autoware.universe/main/sensing/pointcloud_preprocessor/docs/dual-return-outlier-filter/#inner-workings-algorithms ，输入数据必须是 PointXYZIRADRT 类型的数据，包括 return_type 

但autowate官方说支持 PointXYZI 到 PointXYZIRADRT 的转化，具体可以参考 ：https://autowarefoundation.github.io/autoware-documentation/pr-332/design/autoware-architecture/sensing/data-types/point-cloud/

在直接将数据用于 autoware.launch.xml 输入并不能正常启动，在写这小节时已经实现了 autoware.localization 模块，NDT匹配只需要XYZ属性即可，具体参考 random_downsample_filter 降采样后的 pointclouds 数据用于NDT .

```sh
Not recommended for use as it is under development. Input data must be PointXYZIRADRT type data including return_type.
# 输入数据必须是 PointXYZIRADRT 类型的数据，包括 return_type
```



### 二、步骤

#### 1、启动 webots 仿真环境

加载车辆、环境、点云等相关信息 ：

```sh
ros2 launch autoware_webots autoware_car_launch.py
```

#### 2、新建 autoware_webots_utils 功能包

添加 AckermannControlCommand_to_cmd_vel.py 程序用于将autoware发布的运动控制指令转换成webots订阅的 cmd_vel 话题，目前没有用到，后面在autoware 自动驾驶 planning 决策规划输出的运动指令映射到控制 webots车辆模型时会用到 ：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from geometry_msgs.msg import Twist
import math

class AckermannControlCommand_to_cmd_vel(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("Ackermann Control Command to cmd_vel .")
        self.AckermannControlCommand_msg = AckermannControlCommand()
        self.Cmd_vel_msg = Twist()

        self.AckermannControlCommand_sub = self.create_subscription(AckermannControlCommand,"/control/command/control_cmd",self.AckermannControlCommand_sub_callback,10)
        self.cmd_vel_pub = self.create_publisher(Twist,"cmd_vel",10)

    def AckermannControlCommand_sub_callback(self,date):
        # 根据阿克曼公式计算角速度
        wheelbase = 0.86*2  # 小车轮距
        wheelD = 0.5     # 轮直径
        lenbase = 2.94    # 小车轴距
        # 角速度 = （ tan(内轮转角) * 车子线速度 ）/ 车子轴距
        self.AckermannControlCommand_msg.longitudinal.speed = date.longitudinal.speed
        self.AckermannControlCommand_msg.lateral.steering_tire_angle = date.lateral.steering_tire_angle
        # print(self.AckermannControlCommand_msg.longitudinal.speed," - ",self.AckermannControlCommand_msg.longitudinal.acceleration)
        self.Cmd_vel_msg.linear.x = self.AckermannControlCommand_msg.longitudinal.speed
        # self.Cmd_vel_msg.angular.z = self.AckermannControlCommand_msg.lateral.steering_tire_angle
        if self.AckermannControlCommand_msg.longitudinal.speed != 0 and self.AckermannControlCommand_msg.lateral.steering_tire_angle != 0 :
            self.Cmd_vel_msg.angular.z = math.tan(self.AckermannControlCommand_msg.lateral.steering_tire_angle) * self.AckermannControlCommand_msg.longitudinal.speed / lenbase
        else :
            self.Cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.Cmd_vel_msg)
    
    def degree2angular(self) :
        pass


def main(args=None):
    rclpy.init(args=args)			    
    AckermannControlCommand_to_cmd_vel_node = AckermannControlCommand_to_cmd_vel("AckermannControlCommand_to_cmd_vel_node")    
    rclpy.spin(AckermannControlCommand_to_cmd_vel_node)                 
    rclpy.shutdown()

main()

```

启动该节点 ：

```sh
ros2 run autoware_webots_utils AckermannControlCommand_to_cmd_vel
```

#### 3、sensor 参数适配 autoware_webots_individual_params

参考文件autoware/src/param/autoware_individual_params/individual_params ，将其提出来重命名为autoware_webots_individual_params ，这里的 awsim 和 sample 两个模型使用的话题、数据类型、tf2变换都是一致的，以 sample_sensor_kit 为例修改其中的参数适配webots中的车辆模型，主要是修改下面两个参数文件：

- sensors_calibration.yaml

base_link -> sensor_kit_base_link ，这是 传感器sensor 到 base_link 的一个总体变换，为了方便这个全设置为 0 不变换 :

```yaml
base_link:
  sensor_kit_base_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  velodyne_rear_base_link:
    x: -0.358
    y: 0.0
    z: 1.631
    roll: -0.02
    pitch: 0.7281317
    yaw: 3.141592

```

- sensor_kit_calibration.yaml

sensor -> sensor_kit_base_link ，上面将sensor_kit_base_link绑定在了base_link上，这里就类似做sensor到base_link的tf变换，参照webots和上面的表格，修改对应传感器link在webots车辆中的实际位置关系，设计以下几个 （**注意：不要修改link名称否则会报错**）：

```yaml
sensor_kit_base_link:
  traffic_light_left_camera/camera_link:
    x: 1.56
    y: 0.0
    z: 1.04
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  velodyne_top_base_link:
    x: 1.24
    y: 0.0
    z: 1.27
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  gnss_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  tamagawa/imu_link:
    x: 0.0
    y: 0.0
    z: 0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

#### 4、运行

具体运行需要结合下一节vehicle 车辆模型的launch启动文件