#!/usr/bin/env python3
import os
import launch
import pathlib
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def generate_launch_description():
    package_dir = get_package_share_directory('autoware_webots')

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world = LaunchConfiguration('world', default='autoware_tesla.wbt')

    webots_urdf = pathlib.Path(os.path.join(package_dir, 'resource', 'tesla_webots.urdf')).read_text()
    vehice_urdf = os.path.join(get_package_share_directory('autoware_webots'),'urdf', 'vehice.urdf')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )
    
    mappings = [('/sensing/camera/traffic_light/image_raw/image_color', '/sensing/camera/traffic_light/image_raw'), 
                ('/sensing/lidar/top/pointcloud_raw/point_cloud', '/sensing/lidar/top/pointcloud_raw')]
    tesla_driver = WebotsController(
        robot_name='vehicle',
        parameters=[
            {'robot_description': webots_urdf,
            'use_sim_time': use_sim_time,
            'set_robot_state_publisher': False}
        ],
        remappings=mappings,
        respawn=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time , "robot_description":Command(["xacro", " ", vehice_urdf])}],
        # arguments=[urdf]
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=tesla_driver,
        nodes_to_start=robot_state_publisher
    )

    pubOdom = Node(
        package='autoware_webots',
        executable='pubOdom',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        tesla_driver,
        # waiting_nodes, 
        robot_state_publisher, 
        pubOdom, 
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
