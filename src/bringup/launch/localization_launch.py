import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # ---------------------------------------------------------
    # WICHTIG: Ändere 'my_robot_nav' zu deinem echten Paketnamen!
    # ---------------------------------------------------------
    pkg_name = 'bringup'
    
    # Pfad zur YAML-Konfigurationsdatei
    config_dir = os.path.join(
        get_package_share_directory(pkg_name), 
        'config', 
        'robot_localization.yaml'
    )

    # ==========================================
    # 1. DER EKF KNOTEN (Der Filter)
    # ==========================================
    # Fusioniert die umgerechneten GPS-Daten und deinen Kompass (Yaw)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_dir],
        # remappings=[
        #     # Wir benennen das Standard-Topic um, damit es eindeutiger ist
        #     ('/odometry/filtered', '/odometry/global')
        # ]
    )

    # ==========================================
    # 2. DER NAVSAT TRANSFORM KNOTEN
    # ==========================================
    # Wandelt Längen-/Breitengrad in metrische X/Y-Koordinaten um
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[config_dir],
        remappings=[
            # Hier binden wir dein spezifisches RTK-Topic ein!
            ('/gps/fix', '/ublox_gps_node/fix'), 
            
            # Falls dein BNO085 auf einem anderen Topic sendet, hier anpassen:
            ('/imu', '/imu/data'),               
            
            # navsat_transform braucht die aktuelle Schätzung vom EKF
            ('/odometry/filtered', '/odometry/global') 
        ]
    )

# ==========================================
    # 3. STATISCHE TRANSFORMATIONEN (TF2) - JAZZY STYLE
    # ==========================================
    
    # BNO085 Transformation
    # WICHTIG: child-frame-id auf 'imu' geändert (statt 'imu_link')
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
    # WICHTIG: child-frame-id auf 'gps' geändert (statt 'gps_link')
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

