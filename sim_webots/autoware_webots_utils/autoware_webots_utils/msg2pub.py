import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField,Imu,Image
from sensor_msgs_py import point_cloud2
import numpy as np
import tf_transformations

class PointCloudSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber_publisher')
        self.point2 = PointCloud2()
        self.subscription_point = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw_',
            self.pointcloud_callback,
            10
        )
        self.publisher_point = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw',
            10
        )

        self.imu = Imu()
        self.init_yaw = None
        self.create_subscription(
            Imu,
            '/sensing/imu/tamagawa/imu_raw_',  
            self.imu_callback,
            10
        )
        self.publisher_imu = self.create_publisher(
            Imu,
            '/sensing/imu/tamagawa/imu_raw',
            10
        )

        self.subscription_image = self.create_subscription(
            Image,
            '/sensing/camera/traffic_light/image_raw_',  
            self.image_callback,
            10
        )
        self.publisher_image = self.create_publisher(
            Image,
            '/sensing/camera/traffic_light/image_raw',
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

    def pointcloud_callback(self, msg):
        # 获取点云数据
        self.point2 = msg
        # self.publisher_point.publish(self.point2)

    def imu_callback(self, msg):
        self.imu = msg
        # 矫正初始值
        # if self.init_yaw == None :
        #     self.init_yaw = tf_transformations.euler_from_quaternion([  msg.orientation.x, msg.orientation.y, 
        #                                                                 msg.orientation.z, msg.orientation.w])[2]
        #     return
        # euler_raw = tf_transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        # euler = [euler_raw[0], euler_raw[1], euler_raw[2]]
        # euler[2] -= self.init_yaw
        # # print(euler)
        # orientation = tf_transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        # self.imu.header = msg.header
        # # print(orientation)
        # self.imu.orientation.x = orientation[0]
        # self.imu.orientation.y = orientation[1]
        # self.imu.orientation.z = orientation[2]
        # self.imu.orientation.w = orientation[3]
        # self.imu.linear_acceleration.z = 9.8
        # self.publisher_imu.publish(msg)

    def image_callback(self, msg):
        self.publisher_image.publish(msg)

    def timer_callback(self):
        self.publisher_point.publish(self.point2)
        self.publisher_imu.publish(self.imu)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber_publisher = PointCloudSubscriberPublisher()
    rclpy.spin(pointcloud_subscriber_publisher)
    pointcloud_subscriber_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
