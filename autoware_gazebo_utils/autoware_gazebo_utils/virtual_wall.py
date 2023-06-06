# 全局代价地图中的禁行线，驱使小车运动规划时绕开禁行区
import numpy as np
import rclpy
import os
import yaml   # pip3 install pyyaml
import json
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

class VirtualWall(Node):
    def __init__(self):
        super().__init__('virtual_wall_node')
        self.get_json_yaml()
        self.init_virtual_cloud()

        for map_json_shapes in self.map_json_data["shapes"] :
            last_point = False
            for map_json_points in map_json_shapes["points"] :
                if last_point == False :
                    last_point = map_json_points
                else :
                    x1,y1 = self.calculate_map_xy(last_point[0],last_point[1])
                    x2,y2 = self.calculate_map_xy(map_json_points[0],map_json_points[1])
                    self.creat_virtual_cloud([x1,y1],[x2,y2])
                    last_point = map_json_points
                # print(map_json_points)

        # end - 生成 ros2 禁行区点云
        self.creat_ros2_virtuel_clouds()
        self.get_sub_counts = 0 
        self.publisher_ = self.create_publisher(PointCloud2, '/sensing/virtual_wall/point_cloud', 1)
        timer_period = 1 / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Create virtual wall ...")

    def get_json_yaml(self):
        # map 相关信息
        map_yaml_dir = os.path.join(
                get_package_share_directory('autoware_bringup'),
                'maps',
                'map.yaml')
        map_yaml_file = open(map_yaml_dir, 'r', encoding="utf-8")
        map_yaml_file_data = map_yaml_file.read()
        map_yaml_file.close()
        self.map_yaml_data = yaml.load(map_yaml_file_data,Loader=yaml.FullLoader)
        # print(self.map_yaml_data)
        # print(self.map_yaml_data["resolution"])
        # print(self.map_yaml_data["origin"])
        
        # 虚拟墙相关信息
        map_json_dir = os.path.join(
                get_package_share_directory('autoware_bringup'),
                'maps',
                'map.json')
        with open(map_json_dir, "r", encoding="utf-8") as f:
            self.map_json_data = json.load(f)
        # print(self.map_json_data)
        # print(self.map_json_data["shapes"])
        # print(self.map_json_data["shapes"][0]["points"][0][0])

    # 图像像素坐标->地图坐标
    def calculate_map_xy(self,image_x,image_y):
        wx = image_x * self.map_yaml_data["resolution"] + self.map_yaml_data["origin"][0]
        wy = image_y * self.map_yaml_data["resolution"] + self.map_yaml_data["origin"][1]
        # 这里的wy计算出世界地图中的位置有问题，做了一个反转并+0.5调试了一下
        return wx,-wy+21

    def init_virtual_cloud(self):
        self.rate = 5                       # virtual_wall 点云发布频率
        self.header = Header()              # header 属性部分
        self.header.frame_id = 'map'        # 定义为map
        self.dtype = PointField.FLOAT32
        self.point_step = 16
        # 仅需要用到 XYZ 的属性即可
        self.fields = [PointField(name='x', offset=0, datatype=self.dtype, count=1),
                PointField(name='y', offset=4, datatype=self.dtype, count=1),
                PointField(name='z', offset=8, datatype=self.dtype, count=1),
                PointField(name='intensity', offset=12, datatype=self.dtype, count=1)]
        # 用来存储生成虚拟墙的点云 x  y 坐标
        self.x_all = [0]
        self.y_all = [0]
        self.z_all = np.zeros(np.size(self.x_all))
        self.xyz_init_flag = False
    
    def timer_callback(self):
        now_sub_count = self.publisher_.get_subscription_count()
        if now_sub_count - self.get_sub_counts > 0 :
            # print("new sub !")
            self.ros2_virtuel_clouds_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.ros2_virtuel_clouds_msg)
            self.get_sub_counts = now_sub_count
        else :
            self.get_sub_counts = now_sub_count
    def creat_virtual_cloud(self,point_begin,point_end):
        # 选择较大的一个最为插值个数计算
        linspace_num = int(max(abs(point_begin[0]-point_end[0]),abs(point_begin[1]-point_end[1])) * 10 )
        # 计算插值序列并添加到原有的数组后面
        # print(point_begin[0], point_end[0],linspace_num)
        # print(self.x_all)
        # print(np.linspace(point_begin[0], point_end[0], linspace_num))
        if self.xyz_init_flag == False :
            self.x_all = np.linspace(point_begin[0], point_end[0], linspace_num)
            self.y_all = np.linspace(point_begin[1], point_end[1], linspace_num)
            self.xyz_init_flag = True
        else :
            self.x_all = np.concatenate((self.x_all, np.linspace(point_begin[0], point_end[0], linspace_num)), axis=0)
            self.y_all = np.concatenate((self.y_all, np.linspace(point_begin[1], point_end[1], linspace_num)), axis=0)
        self.z_all = np.zeros(np.size(self.x_all))

    def creat_ros2_virtuel_clouds(self) :
        points = np.array([self.x_all, self.y_all, self.z_all, self.z_all]).reshape(4, -1).T
        self.ros2_virtuel_clouds_msg = point_cloud2.create_cloud(self.header, self.fields, points)

def main(args=None):
    rclpy.init(args=args)
    virtual_wall_node = VirtualWall()
    rclpy.spin(virtual_wall_node)
    virtual_wall_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()