# import launch
# import launch_ros.actions

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource

# def generate_launch_description():

#     # === 1. Pfade zu den Paketen finden, die wir starten wollen ===
    
#     # Finde den Pfad zum rosbridge_server Paket
#     rosbridge_pkg_share = get_package_share_directory('rosbridge_server')
    
#     # Finde den Pfad zu deinem ublox_gps Paket
#     # (Passe 'ublox_gps' an, falls dein Paket anders heißt)
#     ublox_gps_pkg_share = get_package_share_directory('ublox_gps') 


#     # === 2. Nodes definieren, die mit "ros2 run" gestartet werden ===
    
#     # Startet deinen bno085_node
#     bno085_node = Node(
#         package='bno085',
#         executable='bno085_node',
#         name='bno085_node'
#     )

#     # Startet deinen web_server_node
#     web_ui_node = Node(
#         package='web_ui',
#         executable='web_server_node',
#         name='web_server_node'
#     )

#     # === 3. Launch-Dateien definieren, die wir "includen" ===

#     # Startet den rosbridge_server (dies ist eine XML-Launch-Datei)
#     rosbridge_launch = IncludeLaunchDescription(
#         XMLLaunchDescriptionSource(
#             os.path.join(rosbridge_pkg_share, 'launch', 'rosbridge_websocket_launch.xml')
#         )
#     )

#     # Startet deinen ublox_gps Node (eine Python-Launch-Datei)
#     ublox_gps_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(ublox_gps_pkg_share, 'launch', 'ublox_gps_node_zedf9p-launch.py')
#             # (Passe 'ublox_gps_node-launch.py' an, falls deine Datei anders heißt)
#         )
#     )

#     # === 4. Alle Aktionen zusammenfassen und zurückgeben ===
#     return LaunchDescription([
#         rosbridge_launch,
#         ublox_gps_launch,
#         bno085_node,
#         web_ui_node
        
#         # Du kannst die Reihenfolge ändern, falls Nodes voneinander abhängen
#     ])

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

    web_ui_node = Node(
        package='web_ui',
        executable='web_server_node',
        name='web_server_node'
    )

    # === 3. Launch-Dateien includen ===
    # rosbridge_launch = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         os.path.join(rosbridge_pkg_share, 'launch', 'rosbridge_websocket_launch.xml')
    #     )
    # )

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(rosbridge_pkg_share, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        # === HIER DIE LÖSUNG ===
        # Wir zwingen rosbridge, Port 9090 zu verwenden, 
        # um die Kollision mit dem 'port'-Argument (2101) des ntrip_client zu verhindern.
        launch_arguments={'port': '9090'}.items()
    )

    ublox_gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ublox_gps_pkg_share, 'launch', 'ublox_gps_node_zedf9p-launch.py')
        )
    )

    ntrip_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ntrip_client_pkg_share, 'ntrip_client_launch.py')
        )
    )

    # === 4. Alle Aktionen zusammenfassen ===
    return LaunchDescription([
        ntrip_client_launch,
        rosbridge_launch,
        ublox_gps_launch,
        bno085_node,
        web_ui_node
    ])