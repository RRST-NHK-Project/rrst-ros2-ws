from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """WebカメラのPublisherノードを起動するlaunchファイル."""
    camera_index = LaunchConfiguration("camera_index")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_index",
                default_value="0",
                description="OpenCV camera index",
            ),
            Node(
                package="webcam_publisher",
                executable="webcam_publisher",
                name="webcam_publisher",
                output="screen",
                parameters=[
                    {
                        "camera_index": camera_index,
                        "topic_name": "/webcam/image_raw",
                        "frame_id": "webcam_frame",
                        "publish_rate": 30.0,
                        "frame_width": 640,
                        "frame_height": 480,
                    }
                ],
            ),
        ]
    )
