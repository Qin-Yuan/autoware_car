import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField,Imu,Image
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class PointCloudSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber_publisher')
        self.point2 = PointCloud2()
        # self.subscription_point = self.create_subscription(
        #     PointCloud2,
        #     '/sensing/lidar/top/pointcloud_raw_',
        #     self.pointcloud_callback,
        #     10
        # )
        # self.publisher_point = self.create_publisher(
        #     PointCloud2,
        #     '/sensing/lidar/top/pointcloud_raw',
        #     10
        # )

        self.imu = Imu()
        self.subscription_imu = self.create_subscription(
            Imu,
            '/sensing/imu/tamagawa/imu_raw_bag',  
            self.imu_callback,
            10
        )
        self.publisher_imu = self.create_publisher(
            Imu,
            '/sensing/imu/tamagawa/imu_raw',
            10
        )

        # self.gnss = PoseWithCovarianceStamped()
        # self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     '/sensing/gnss/pose_with_covariance_',  
        #     self.gnss_callback,
        #     10
        # )
        # self.publisher_gnss = self.create_publisher(
        #     PoseWithCovarianceStamped,
        #     '/sensing/gnss/pose_with_covariance',
        #     10
        # )


        self.timer = self.create_timer(0.01, self.timer_callback)

    def pointcloud_callback(self, msg):
        # 获取点云数据
        self.point2 = msg
        # self.publisher_point.publish(self.point2)

    def imu_callback(self, msg):
        self.imu = msg

    def image_callback(self, msg):
        self.publisher_image.publish(msg)

    def gnss_callback(self, msg):
        self.gnss = msg
        

    def timer_callback(self):
        pass
        # self.point2.header.stamp = self.get_clock().now().to_msg()
        # self.publisher_point.publish(self.point2)

        # self.imu.header.stamp = self.get_clock().now().to_msg()
        # self.publisher_imu.publish(self.imu)

        # self.gnss.header.stamp = self.get_clock().now().to_msg()
        # self.publisher_gnss.publish(self.gnss)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber_publisher = PointCloudSubscriberPublisher()
    rclpy.spin(pointcloud_subscriber_publisher)
    pointcloud_subscriber_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
