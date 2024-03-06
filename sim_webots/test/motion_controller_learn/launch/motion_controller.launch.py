import os
import pathlib
import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time',default=True)
    package_dir = get_package_share_directory('motion_controller_learn')
    file_path = os.path.join(package_dir,'config', 'path_data.csv')

    utils_pub_path_node = Node(
            package='motion_controller_learn',
            executable='utils_pub_path.py',
            name='utils_pub_path',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time , "file_path": file_path}],
        )
    
    utils_pub_tf2_node = Node(
            package='motion_controller_learn',
            executable='utils_pub_tf2.py',
            name='utils_pub_tf2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock if true'
        ),
        utils_pub_path_node,
        utils_pub_tf2_node,
    ])
