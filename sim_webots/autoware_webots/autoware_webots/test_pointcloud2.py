import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import pcl


class PclProcessingNode(Node):

    def __init__(self):
        super().__init__('pcl_processing_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            'raw_pointcloud_topic',
            self.process_pointcloud_callback,
            10)
        self.publisher = self.create_publisher(
            PointCloud2,
            'processed_pointcloud_topic',
            10)

    def process_pointcloud_callback(self, pointcloud_msg):
        # Convert ROS pointcloud2 message to numpy array
        points = point_cloud2.read_points(pointcloud_msg)

        # Extract x,y,z coordinates
        point_cloud = np.array([list(pt) for pt in points])
        x = point_cloud[:, 0]
        y = point_cloud[:, 1]
        z = point_cloud[:, 2]

        # Calculate intensity and time
        intensity = np.sqrt(x**2 + y**2 + z**2)
        time = np.zeros_like(x)

        # Create new PCL point cloud and add fields
        pcl_point_cloud = pcl.PointCloud()
        pcl_point_cloud.from_array(point_cloud.astype(np.float32))
        pcl_intensity = pcl.io.PointCloudField()
        pcl_intensity.name = 'intensity'
        pcl_intensity.datatype = 7  # float32
        pcl_intensity.count = 1
        pcl_intensity.offset = 12  # Offset of intensity field in bytes
        pcl_point_cloud.fields.append(pcl_intensity)
        pcl_time = pcl.io.PointCloudField()
        pcl_time.name = 'time'
        pcl_time.datatype = 7  # float32
        pcl_time.count = 1
        pcl_time.offset = 16  # Offset of time field in bytes
        pcl_point_cloud.fields.append(pcl_time)

        # Set intensity and time values for each point in the point cloud
        for i in range(len(intensity)):
            pcl_point_cloud[i][3] = intensity[i]
            pcl_point_cloud[i][4] = time[i]

        # Convert PCL point cloud to ROS pointcloud2 message
        processed_pointcloud_msg = pcl_point_cloud.to_array()
        processed_pointcloud_msg.header = pointcloud_msg.header
        processed_pointcloud_msg.height = 1
        processed_pointcloud_msg.width = len(processed_pointcloud_msg) // (pcl_point_cloud.point_step // 4)
        processed_pointcloud_msg.is_bigendian = False
        processed_pointcloud_msg.point_step = pcl_point_cloud.point_step
        processed_pointcloud_msg.row_step = pcl_point_cloud.row_step
        processed_pointcloud_msg.is_dense = pcl_point_cloud.is_dense

        # Publish processed point cloud
        self.publisher.publish(processed_pointcloud_msg)


def main(args=None):
    rclpy.init(args=args)
    pcl_processing_node = PclProcessingNode()
    rclpy.spin(pcl_processing_node)
    pcl_processing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
