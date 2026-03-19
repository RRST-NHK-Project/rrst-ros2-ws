# Copyright 2024 RRST NHK Project
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the WT901C IMU'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for the WT901C serial port'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='TF frame id for the IMU messages'
    )

    wt901_node = Node(
        package='wt901_node',
        executable='wt901_node',
        name='wt901c_node',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        frame_id_arg,
        wt901_node,
    ])
