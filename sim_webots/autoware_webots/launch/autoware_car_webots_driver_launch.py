#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots Tesla driver."""

import os
import pathlib
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix


def get_ros2_nodes(*args):
    use_sim_time = LaunchConfiguration('use_sim_time',default=True)
    package_dir = get_package_share_directory('autoware_webots')
    webots_urdf = pathlib.Path(os.path.join(package_dir, 'resource', 'tesla_webots.urdf')).read_text()
    vehice_urdf = os.path.join(get_package_share_directory('autoware_webots'),'urdf', 'vehice.urdf')
    
    urdf_file = os.path.join(get_package_share_directory('autoware_webots'),
                                'resource', 'autoware_car.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    tesla_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'vehicle'},
        remappings=[("/sensing/lidar/top/pointcloud_raw/point_cloud", "/sensing/lidar/top/pointcloud_raw")],
        parameters=[
            {'robot_description': webots_urdf},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time , "robot_description":Command(["xacro", " ", vehice_urdf])}],
            # arguments=[urdf]
        )
    
    lane_follower = Node(
        package='autoware_webots',
        executable='lane_follower',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    pubOdom = Node(
        package='autoware_webots',
        executable='pubOdom',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    return [
        # lane_follower,
        pubOdom,
        tesla_driver,
        robot_state_publisher,
    ]


def generate_launch_description():
    package_dir = get_package_share_directory('autoware_webots')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'True')
    world = LaunchConfiguration('world')
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )
    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    autoware_webots_utils_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('autoware_webots_utils'), 'launch'), '/autoware_webots_utils_webtos_driver.launch.py'])
        )
    
    return LaunchDescription([
        use_sim_time_arg,
        DeclareLaunchArgument(
            'world',
            default_value='autoware_tesla.wbt',
            description='Choose one of the world files from `/autoware_webots/worlds` directory'
        ),
        webots,
        webots._supervisor,
        autoware_webots_utils_launch,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),

        # Add the reset event handler
        reset_handler
    ] + get_ros2_nodes())
