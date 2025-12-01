from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        DeclareLaunchArgument('transport', default_value='serial'),
        Node(
            package='rtcm_bridge',
            executable='rtcm_bridge',
            name='rtcm_bridge',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'baudrate': LaunchConfiguration('baudrate'),
                'transport': LaunchConfiguration('transport')
            }]
        )
    ])
