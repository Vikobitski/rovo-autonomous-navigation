import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name = 'bringup'
    
    # Path to the YAML configuration file
    config_dir = os.path.join(
        get_package_share_directory(pkg_name), 
        'config', 
        'robot_localization.yaml'
    )

    # ==========================================
    # 1. THE EKF NODE (The Filter)
    # ==========================================
    # Fuses the converted GPS data and your compass (Yaw)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_dir],
    )

    # ==========================================
    # 2. THE NAVSAT TRANSFORM NODE
    # ==========================================
    # Converts longitude/latitude to metric X/Y coordinates
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[config_dir],
        remappings=[
            ('/gps/fix', '/ublox_gps_node/fix'), 
            ('/imu', '/imu/data'),               
        ]
    )

    # ==========================================
    # 3. STATIC TRANSFORMATIONS (TF2)
    # ==========================================
    
    # BNO085 Transformation
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'imu_link'
        ]
    )

    # u-blox RTK Transformation
    tf_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_gps',
        arguments=[
            '--x', '0.2', '--y', '0.0', '--z', '0.5',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'gps'
        ]
    )

    return LaunchDescription([
        ekf_node,
        navsat_node,
        tf_imu,
        tf_gps
    ])

