import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

# === KORRIGIERTE IMPORTE ===
# PythonLaunchDescriptionSource für .py-Dateien (wie ublox_gps)
from launch.launch_description_sources import PythonLaunchDescriptionSource
# XMLLaunchDescriptionSource für .xml-Dateien (wie rosbridge)
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    # === 1. Pfade zu den Paketen finden ===
    rosbridge_pkg_share = get_package_share_directory('rosbridge_server')
    ublox_gps_pkg_share = get_package_share_directory('ublox_gps') 
    ntrip_client_pkg_share = get_package_share_directory('ntrip_client')

    # === 2. Nodes definieren ===
    bno085_node = Node(
        package='bno085',
        executable='bno085_node',
        name='bno085_node'
    )

    rovo_driver_node = Node(
        package='rovo_driver',
        executable='rovo_driver_node',
        name='rovo_driver_node',
        output='screen'
    )

    web_ui_node = Node(
        package='web_ui',
        executable='web_server_node',
        name='web_server_node'
    )

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(rosbridge_pkg_share, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        # === HIER DIE LÖSUNG ===
        # Wir zwingen rosbridge, Port 9090 zu verwenden, 
        # um die Kollision mit dem 'port'-Argument (2101) des ntrip_client zu verhindern.
        launch_arguments={'port': '9090', 'delay_between_messages': '0.0'}.items()
    )

    ublox_gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ublox_gps_pkg_share, 'launch', 'ublox_gps_node-launch.py')
        )
    )

    ntrip_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ntrip_client_pkg_share, 'ntrip_client_launch.py')
        )
    )

    # Get bringup package share directory for localization launch
    bringup_pkg_share = get_package_share_directory('bringup')
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg_share, 'launch', 'localization_launch.py')
        )
    )

    # CSV logger node (logs GPS / Pose to CSV files)
    csv_logger_node = Node(
        package='csv_logger',
        executable='csv_logger_node',
        name='csv_logger',
        parameters=[
            {'output_dir': '/home/rovo/rovo_ws/csv_logs'},
            {'gps_topic': 'ublox_gps_node/fix'},
            {'log_gps': True},
        ]
    )

    # === 4. Alle Aktionen zusammenfassen ===
    return LaunchDescription([
        ntrip_client_launch,
        rosbridge_launch,
        ublox_gps_launch,
        localization_launch,
        bno085_node,
        rovo_driver_node,
        web_ui_node,
        csv_logger_node,
    ])