import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    file_directory = os.path.join(get_package_share_directory('pcd2pgm'),'map/')
    # pcd2pgm node 启动
    pcd2pgm = Node(
        package='pcd2pgm',
        executable='pcd2map',
        name='pcd2map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time , 
                    'file_directory': file_directory ,  # 存放pcd文件的路径 
                    'file_name' : "pointcloud_map" ,    # pcd文件名称
                    'thre_z_min' : 0.0 ,                # 选取的范围　最小的高度
                    'thre_z_max' : 1.5 ,                # 选取的范围　最大的高度
                    'flag_pass_through' : False ,       # 0 选取高度范围内的，１选取高度范围外的
                    'thre_radius' : 5.0 ,               # 半径滤波的半径
                    'thres_point_count' : 10 ,          # 半径滤波的要求点数个数
                    'map_resolution' : 0.05 ,           # 存储的栅格map的分辨率
                    'map_topic_name' : "map" ,}],       # 转换后发布的二维地图的topic，默认使用map即可，可使用map_server保存
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        pcd2pgm,
    ])