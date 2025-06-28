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
            'motor_mode',
            default_value='velocity',
            description='Motor control mode (current, velocity, position)'
        ),
        
        # Front left motor (ID 1)
        Node(
            package='ddsm115_ros2',
            executable='ddsm115_node',
            name='front_left_motor',
            namespace='robot/motors',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'motor_id': 1,
                'motor_mode': LaunchConfiguration('motor_mode'),
                'status_publish_rate': 20.0,
                'auto_polling': True,
                'polling_interval_ms': 50,
            }],
            output='screen'
        ),
        
        # Front right motor (ID 2)
        Node(
            package='ddsm115_ros2',
            executable='ddsm115_node',
            name='front_right_motor',
            namespace='robot/motors',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'motor_id': 2,
                'motor_mode': LaunchConfiguration('motor_mode'),
                'status_publish_rate': 20.0,
                'auto_polling': True,
                'polling_interval_ms': 60,  # Slightly offset to avoid conflicts
            }],
            output='screen'
        ),
        
        # Rear left motor (ID 3)
        Node(
            package='ddsm115_ros2',
            executable='ddsm115_node',
            name='rear_left_motor',
            namespace='robot/motors',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'motor_id': 3,
                'motor_mode': LaunchConfiguration('motor_mode'),
                'status_publish_rate': 20.0,
                'auto_polling': True,
                'polling_interval_ms': 70,  # Slightly offset to avoid conflicts
            }],
            output='screen'
        ),
        
        # Rear right motor (ID 4)
        Node(
            package='ddsm115_ros2',
            executable='ddsm115_node',
            name='rear_right_motor',
            namespace='robot/motors',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'motor_id': 4,
                'motor_mode': LaunchConfiguration('motor_mode'),
                'status_publish_rate': 20.0,
                'auto_polling': True,
                'polling_interval_ms': 80,  # Slightly offset to avoid conflicts
            }],
            output='screen'
        )
    ])
