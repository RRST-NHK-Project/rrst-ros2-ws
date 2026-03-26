from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """ArUco検出ノードとビューアノードを同時に起動するlaunchファイル."""
    return LaunchDescription([
        Node(
            package='aruco_tracker',
            executable='aruco_pose_publisher',
            name='aruco_pose_publisher',
            output='screen',
            parameters=[{
                'image_topic': '/camera/camera/color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'output_image_topic': '/camera/image_raw',
                'frame_id': 'camera_color_optical_frame',
            }],
        ),
        Node(
            package='aruco_tracker',
            executable='aruco_viewer',
            name='aruco_viewer',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
            }],
        ),
    ])
