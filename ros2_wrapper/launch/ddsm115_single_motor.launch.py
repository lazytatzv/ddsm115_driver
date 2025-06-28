from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_path',
            default_value='/dev/ttyUSB0',
            description='Serial device path for RS485 communication'
        ),
        DeclareLaunchArgument(
            'motor_id',
            default_value='1',
            description='Motor ID (1-255)'
        ),
        DeclareLaunchArgument(
            'motor_mode',
            default_value='velocity',
            description='Motor control mode (current, velocity, position)'
        ),
        DeclareLaunchArgument(
            'status_publish_rate',
            default_value='10.0',
            description='Status publishing rate in Hz'
        ),
        DeclareLaunchArgument(
            'auto_polling',
            default_value='true',
            description='Enable automatic status polling'
        ),
        DeclareLaunchArgument(
            'polling_interval_ms',
            default_value='100',
            description='Polling interval in milliseconds'
        ),
        
        Node(
            package='ddsm115_ros2',
            executable='ddsm115_node',
            name='ddsm115_motor',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'motor_id': LaunchConfiguration('motor_id'),
                'motor_mode': LaunchConfiguration('motor_mode'),
                'status_publish_rate': LaunchConfiguration('status_publish_rate'),
                'auto_polling': LaunchConfiguration('auto_polling'),
                'polling_interval_ms': LaunchConfiguration('polling_interval_ms'),
            }],
            output='screen'
        )
    ])
