import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('bringup')
    
    # Parameters
    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    
    configured_params = LaunchConfiguration('params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        # Remap cmd_vel (output of controller) to /rovo/set_speed (input of driver)
        remappings=[('cmd_vel', '/rovo/set_speed')]
    )

    # Planner Server
    # planner_server = Node(
    #     package='nav2_planner',
    #     executable='planner_server',
    #     name='planner_server',
    #     output='screen',
    #     respawn=True,
    #     respawn_delay=2.0,
    #     parameters=[configured_params]
    # )

    # Behavior Server (formerly Recoveries Server)
    # behavior_server = Node(
    #     package='nav2_behaviors',
    #     executable='behavior_server',
    #     name='behavior_server',
    #     output='screen',
    #     respawn=True,
    #     respawn_delay=2.0,
    #     parameters=[configured_params]
    # )

    # BT Navigator
    # bt_navigator = Node(
    #     package='nav2_bt_navigator',
    #     executable='bt_navigator',
    #     name='bt_navigator',
    #     output='screen',
    #     respawn=True,
    #     respawn_delay=2.0,
    #     parameters=[configured_params]
    # )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_control',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['controller_server']}]
                                    # 'planner_server',
                                    # 'behavior_server',
                                    # 'bt_navigator']}]
    )

    return LaunchDescription([
        declare_params_file_cmd,
        controller_server,
        # planner_server,
        # behavior_server,
        # bt_navigator,
        lifecycle_manager
    ])
