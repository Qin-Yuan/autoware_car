import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField,Imu
from sensor_msgs_py import point_cloud2
import numpy as np

class PointCloudSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber_publisher')
        self.point2 = PointCloud2()
        self.subscription_point = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw',
            self.pointcloud_callback,
            10
        )
        self.publisher_point = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw_new',
            10
        )

        self.subscription_imu = self.create_subscription(
            Imu,
            '/sensing/imu/tamagawa/imu_raw',  
            self.imu_callback,
            10
        )

        self.publisher_imu = self.create_publisher(
            Imu,
            '/sensing/imu/tamagawa/imu_raw_new',
            10
        )
        # self.timer = self.create_timer(0.1, self.timer_callback)

    def pointcloud_callback(self, msg):
        # 获取点云数据
        self.point2 = msg
        self.publisher_point.publish(self.point2)

    def imu_callback(self, msg):
        self.publisher_imu.publish(msg)

    def timer_callback(self):
        self.publisher_point.publish(self.point2)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber_publisher = PointCloudSubscriberPublisher()
    rclpy.spin(pointcloud_subscriber_publisher)
    pointcloud_subscriber_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
