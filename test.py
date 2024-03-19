import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.create_subscription(Imu,"/sensing/imu/tamagawa/imu_raw_",self.callbakck,10)
        self.imu = Imu()
        self.publisher_ = self.create_publisher(
            Imu,
            '/sensing/imu/tamagawa/imu_raw__',
            10)
        self.timer = self.create_timer(0.1, self.publish_imu)

    def callbakck(self,msg) :
        # self.imu = msg
        pass

    def publish_imu(self):
        # 设置IMU的时间戳
        self.imu.header.stamp = self.get_clock().now().to_msg()
        self.imu.header.frame_id = "tamagawa/imu_link"
        # self.imu.orientation.x
        # 发布IMU消息
        self.publisher_.publish(self.imu)
        self.get_logger().info('Published IMU data.')

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
