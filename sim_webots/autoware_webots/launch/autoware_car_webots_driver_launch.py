#!/usr/bin/env python
import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    package_dir = get_package_share_directory('autoware_webots')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )
    
    # mappings = [('/diffdrive_controller/cmd_vel', '/chassis_cmd_vel'), ('/diffdrive_controller/odom', '/odom1')]
    mappings = []
    robot_description_path = os.path.join(package_dir, 'resource', 'tesla_webots.urdf')
    tesla_driver = WebotsController(
        robot_name='vehicle',
        parameters=[
            {'robot_description': robot_description_path,
            'use_sim_time': use_sim_time,
            'set_robot_state_publisher': False}
        ],
        remappings=mappings,
        respawn=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>', 
            'use_sim_time': use_sim_time
        }],
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=tesla_driver,
        nodes_to_start=robot_state_publisher
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='autoware_tesla.wbt',
            description='Choose one of the world files from `/autoware_webots/worlds` directory'
        ),
        webots,
        webots._supervisor,
        tesla_driver,
        waiting_nodes, 
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
