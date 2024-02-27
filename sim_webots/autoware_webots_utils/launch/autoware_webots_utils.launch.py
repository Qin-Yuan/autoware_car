import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    AckermannControlCommand_to_cmd_vel = Node(
        package='autoware_webots_utils',
        executable='AckermannControlCommand_to_cmd_vel',
        name='AckermannControlCommand_to_cmd_vel',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    odom_to_gnss = Node(
        package='autoware_webots_utils',
        executable='odom_to_gnss',
        name='odom_to_gnss',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    msg2pub = Node(
        package='autoware_webots_utils',
        executable='msg2pub',
        name='msg2pub',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    tf2_base_link_to_map = Node(
        package='autoware_webots_utils',
        executable='tf2_base_link_to_map',
        name='tf2_base_link_to_map',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    vehlicle_webots_pub = Node(
        package='autoware_webots_utils',
        executable='vehlicle_webots_pub',
        name='vehlicle_webots_pub',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (webots) clock if true'),
        # AckermannControlCommand_to_cmd_vel ,
        odom_to_gnss ,
        msg2pub,
        # tf2_base_link_to_map ,
        vehlicle_webots_pub ,
    ])
