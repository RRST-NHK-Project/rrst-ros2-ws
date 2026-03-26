# Copyright 2024 RRST-NHK-Project
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
from launch_ros.actions import Node


def generate_launch_description():
    """立方体検知ノードとビューアノードを同時に起動するlaunchファイル."""
    return LaunchDescription([
        Node(
            package='cube_detection',
            executable='cube_detector',
            name='cube_detector',
            output='screen',
            parameters=[{
                'image_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'output_image_topic': '/cube_detection/image',
                'frame_id': 'camera_color_optical_frame',
                'cube_size': 0.065,
                # HSVカラー範囲（赤立方体）
                'hsv_h_low1': 0,
                'hsv_s_low1': 100,
                'hsv_v_low1': 80,
                'hsv_h_high1': 10,
                'hsv_s_high1': 255,
                'hsv_v_high1': 255,
                'hsv_h_low2': 160,
                'hsv_s_low2': 100,
                'hsv_v_low2': 80,
                'hsv_h_high2': 180,
                'hsv_s_high2': 255,
                'hsv_v_high2': 255,
                'min_contour_area': 500,
                'max_contour_area': 100000,
                'depth_scale': 0.001,
            }],
        ),
        Node(
            package='cube_detection',
            executable='cube_viewer',
            name='cube_viewer',
            output='screen',
            parameters=[{
                'image_topic': '/cube_detection/image',
            }],
        ),
    ])
