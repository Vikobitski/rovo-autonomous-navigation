# import launch
# import launch_ros.actions

# def generate_launch_description():
#     return launch.LaunchDescription([
#         launch_ros.actions.Node(
#             package='demo_nodes_cpp',
#             executable='talker',
#             name='talker'),
#   ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    #launch_dir = get_package_share_directory('launch')
    #gps_params_file = os.path.join('~/rovo_ws/src/launch', 'config', 'gps_params.yaml')

    imu_node = Node(
        package='rovo_sensors',
        executable='imu_node',
        name='imu_node',
    )

    gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=['/home/rovo/rovo_ws/src/launch/config/gps_params.yaml']
    )

    return LaunchDescription([
        imu_node,
        gps_node,
    ])