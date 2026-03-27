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
    """平面検知ノードとビューアノードを同時に起動するlaunchファイル."""
    return LaunchDescription([
        Node(
            package='plane_detection',
            executable='plane_detector',
            name='plane_detector',
            output='screen',
            parameters=[{
                'image_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'output_image_topic': '/plane_detection/image',
                'frame_id': 'camera_color_optical_frame',
                'depth_scale': 0.001,
                'depth_min_m': 0.25,
                'depth_max_m': 3.50,
                'plane_ransac_iterations': 120,
                'plane_distance_threshold_m': 0.02,
                'plane_sample_stride': 4,
                'min_plane_inliers': 1200,
                'min_plane_contour_area': 2000,
            }],
        ),
        Node(
            package='plane_detection',
            executable='plane_viewer',
            name='plane_viewer',
            output='screen',
            parameters=[{
                'image_topic': '/plane_detection/image',
            }],
        ),
    ])
