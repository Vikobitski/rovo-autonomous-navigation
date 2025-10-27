import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    bringup_dir = get_package_share_directory('rovo_bringup')
    gps_params_file = os.path.join(bringup_dir, 'config', 'gps_params.yaml')

    imu_node = Node(
        package='rovo_sensors',
        executable='imu_wrapper_node',
        name='imu_wrapper_node',
    )

    gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[gps_params_file]
    )

    return LaunchDescription([
        imu_node,
        gps_node,
    ])