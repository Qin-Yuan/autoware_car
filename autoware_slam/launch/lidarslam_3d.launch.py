import os

import launch
import launch_ros.actions
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('autoware_slam'),
            'config',
            'lidarslam_config.yaml'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[params_file,{'use_sim_time': LaunchConfiguration('use_sim_time')},],
        remappings=[('/input_cloud','/sensing/lidar/top/pointcloud_raw')],
        output='screen'
        )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('autoware_slam'), 'rviz', 'lidarslam_rviz2.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[params_file,{'use_sim_time': LaunchConfiguration('use_sim_time')},],
        output='screen'
        )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='lidarslam config文件'),

        DeclareLaunchArgument('rviz', default_value='true',
                            description='Open RViz.'),
        
        DeclareLaunchArgument('use_sim_time', default_value = 'True'),
        mapping,
        rviz,
        graphbasedslam,
            ])