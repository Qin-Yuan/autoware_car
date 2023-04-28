# Copyright 2019 Louise Poubel
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

"""Launch Gazebo with a world that has Dolly, as well as the follow node."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_autoware_gazebo = get_package_share_directory('autoware_gazebo')
    
    pkg_autoware_gazebo_utils_dir = os.path.join(get_package_share_directory('autoware_gazebo_utils'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf = os.path.join(pkg_autoware_gazebo,'urdf', 'autoware_car.urdf')

    subprocess.run(['killall', 'gzserver'])
    subprocess.run(['killall', 'gzclient'])

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_autoware_gazebo, 'rviz', 'gazebo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # tf2
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_autoware_gazebo, 'worlds', 'autoware_car_city_with_sensor.world'), ''],
            description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='true',
                            description='Open RViz.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_autoware_gazebo_utils_dir, '/autoware_gazebo_utils.launch.py'])
        ),

        gazebo,
        # robot_state_publisher,
        # rviz
    ])
