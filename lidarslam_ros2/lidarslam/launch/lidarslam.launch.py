import os

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir,{'use_sim_time': LaunchConfiguration('use_sim_time')},],
        remappings=[('/input_cloud','/vehicle/Velodyne_VLP_16/point_cloud')],
        # remappings=[('/input_cloud','/sensing/lidar/top/pointcloud_raw')],
        output='screen'
        )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','velodyne_link']
        # arguments=['0','0','0','0','0','0','1','base_link','sensor_kit_base_link']
        )


    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[main_param_dir,{'use_sim_time': LaunchConfiguration('use_sim_time')},],
        output='screen'
        )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),

        DeclareLaunchArgument('use_sim_time', default_value = 'True'),

        mapping,
        tf,
        graphbasedslam,
            ])