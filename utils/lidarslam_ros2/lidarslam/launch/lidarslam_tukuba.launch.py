import os

import launch
import launch_ros.actions
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam_tukuba.yaml'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir],
        remappings=[('/input_cloud','/velodyne_V16/pointcloud2')],
        # remappings=[('/input_cloud','/points_raw')],
        output='screen'
        )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[main_param_dir],
        output='screen'
        )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','velodyne']
        )
    
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('lidarslam'), 'rviz', 'mapping_tukuba.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
        launch.actions.DeclareLaunchArgument('rviz', default_value='true',
                            description='Open RViz.'),
        mapping,
        graphbasedslam,
        # tf,
        rviz,
            ])
