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
    package_name = 'autoware_webots'
    package_dir = get_package_share_directory(package_name)
    vehice_urdf = os.path.join(get_package_share_directory('autoware_webots'),'urdf', 'vehice.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'autoware_tesla.wbt'),
    )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time , "robot_description":Command(["xacro", " ", vehice_urdf])}],
            # arguments=[urdf]
        )

    autoware_webots_run = Node(
        package='autoware_webots',
        executable='autoware_webots_run',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # autoware_utils_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('autoware_utils'), 'launch'), '/autoware_utils_launch.py'])
    #     )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock if true'
        ),
        webots,
        autoware_webots_run,
        # autoware_tf2,
        # autoware_utils_launch,
        robot_state_publisher,
    ])
