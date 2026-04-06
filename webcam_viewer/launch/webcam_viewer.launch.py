from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Webカメラ画像を表示するViewerノードを起動するlaunchファイル."""
    return LaunchDescription(
        [
            Node(
                package="webcam_viewer",
                executable="webcam_viewer",
                name="webcam_viewer",
                output="screen",
                parameters=[
                    {
                        "topic_name": "/webcam/image_raw",
                        "window_name": "Webcam Viewer",
                    }
                ],
            ),
        ]
    )
