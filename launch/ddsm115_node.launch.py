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

"""Launch the standalone DDSM115 driver node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the DDSM115 driver node."""
    pkg_share = get_package_share_directory('ddsm115_ros2_driver')
    default_params = os.path.join(pkg_share, 'config', 'ddsm115_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the DDSM115 driver parameter file',
        ),
        Node(
            package='ddsm115_ros2_driver',
            executable='ddsm115_driver_node',
            name='ddsm115_driver_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
