import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np
import pcl

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/vehicle/Velodyne_VLP_16/point_cloud',
            self.process_pointcloud,
            10)

        self.publisher = self.create_publisher(PointCloud2, 'processed_points2', 10)

        self.pointcloud_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1)
        ]

    def process_pointcloud(self, msg):
        # Convert PointCloud2 to numpy array
        pointcloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)

        # Calculate intensity
        intensity = np.linalg.norm(pointcloud[:, :3], axis=1)

        # Calculate time
        time = np.arange(len(pointcloud), dtype=np.float32)

        # Add intensity and time to the pointcloud array
        pointcloud = np.hstack((pointcloud, intensity.reshape(-1, 1), time.reshape(-1, 1)))

        # Convert pointcloud back to PointCloud2
        processed_msg = PointCloud2()
        processed_msg.header = msg.header
        processed_msg.fields = self.pointcloud_fields
        processed_msg.height = 1
        processed_msg.width = len(pointcloud)
        processed_msg.point_step = len(self.pointcloud_fields) * 4
        processed_msg.row_step = processed_msg.point_step * processed_msg.width
        processed_msg.is_dense = False
        processed_msg.data = pointcloud.astype(np.float32).tobytes()

        self.publisher.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)

    pointcloud_processor = PointCloudProcessor()

    rclpy.spin(pointcloud_processor)

    pointcloud_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
