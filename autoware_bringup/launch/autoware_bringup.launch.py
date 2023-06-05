import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('autoware_bringup'),
            'maps',
            'map.yaml'))

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('autoware_bringup'),
            'params',
            'humble.yaml'))
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('autoware_bringup'), 'launch')
    bt_task_manager_launch_file_dir = os.path.join(get_package_share_directory('autoware_behavior_tree'), 'launch')
    autoware_utils_launch_file_dir = os.path.join(get_package_share_directory('autoware_utils'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('autoware_bringup'),
        'rviz',
        'autoware_nav2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([autoware_utils_launch_file_dir, '/autoware_utils.launch.py']),
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav2_bringup.launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bt_task_manager_launch_file_dir, '/bt_task_manager.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
