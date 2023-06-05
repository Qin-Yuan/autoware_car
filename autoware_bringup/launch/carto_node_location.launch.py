from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': True}],
        arguments = [
            '-configuration_directory', FindPackageShare('autoware_bringup').find('autoware_bringup') + '/params',
            '-configuration_basename', 'autoware_loc.lua',
            # '-start_trajectory_with_default_topics', 'false',
            '-load_state_filename', FindPackageShare('autoware_bringup').find('autoware_bringup') + '/maps/map.pbstream',],
        remappings = [
            ('scan', '/sensing/lidar/multi/scan'),  
            ('imu', '/sensing/imu'),
            ('odom','/odom')],
        output = 'screen'
        )
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        remappings = [
            ('map', 'carto_map'),],
        )
    return LaunchDescription([
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])