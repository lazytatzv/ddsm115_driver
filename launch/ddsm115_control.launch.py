# Copyright 2026 Tatsukiyano
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the ros2_control configuration for DDSM115 motors."""

import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Set up the launch nodes after resolving configurations."""
    # Retrieve parameters
    usb_path = LaunchConfiguration('usb_path').perform(context)
    serial_baud = LaunchConfiguration('serial_baud').perform(context)
    motor_id_left = LaunchConfiguration('motor_id_left').perform(context)
    motor_id_right = LaunchConfiguration('motor_id_right').perform(context)

    # Generate a temporary URDF file containing the ros2_control description
    urdf_content = f"""<?xml version="1.0"?>
<robot name="ddsm115_robot">
  <link name="base_link"/>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <axis xyz="0 1 0"/>
  </joint>

  <ros2_control name="DDSM115HardwareInterface" type="system">
    <hardware>
      <plugin>ddsm115_ros2_driver/DDSM115SystemHardware</plugin>
      <param name="usb_path">{usb_path}</param>
      <param name="serial_baud">{serial_baud}</param>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
      <param name="motor_id">{motor_id_left}</param>
      <param name="invert_direction">false</param>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
      <param name="motor_id">{motor_id_right}</param>
      <param name="invert_direction">true</param>
    </joint>
  </ros2_control>
</robot>
"""
    # Write to a temporary file
    temp_urdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    temp_urdf.write(urdf_content)
    temp_urdf.close()

    # Controller manager configuration
    controller_config_content = """
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.4
    wheel_radius: 0.0575  # DDSM115 wheel radius is ~57.5mm

    use_stamped_vel: false

    # Odometry limits
    linear.x.has_velocity_limits: true
    linear.x.max_velocity: 1.0
    linear.x.min_velocity: -1.0
    angular.z.has_velocity_limits: true
    angular.z.max_velocity: 2.0
    angular.z.min_velocity: -2.0
"""
    temp_controller_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    temp_controller_yaml.write(controller_config_content)
    temp_controller_yaml.close()

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[temp_urdf.name]
    )

    # Controller Manager Node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[temp_urdf.name, temp_controller_yaml.name],
        output='screen'
    )

    # Spawner nodes for controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ]


def generate_launch_description():
    """Generate launch description for differential drive control."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'usb_path',
            default_value='/dev/ttyUSB0',
            description='Path to the USB-to-RS485 interface'
        ),
        DeclareLaunchArgument(
            'serial_baud',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        DeclareLaunchArgument(
            'motor_id_left',
            default_value='1',
            description='Motor ID for the left wheel'
        ),
        DeclareLaunchArgument(
            'motor_id_right',
            default_value='2',
            description='Motor ID for the right wheel'
        ),
        OpaqueFunction(function=launch_setup)
    ])
