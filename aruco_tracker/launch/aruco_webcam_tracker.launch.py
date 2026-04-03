from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Webカメラ入力のArUco検出ノードとビューアを同時起動するlaunchファイル."""
    camera_index = LaunchConfiguration("camera_index")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_index",
                default_value="0",
                description="OpenCV camera index",
            ),
            Node(
                package="aruco_tracker",
                executable="aruco_webcam_detector",
                name="aruco_webcam_detector",
                output="screen",
                parameters=[
                    {
                        "camera_index": camera_index,
                        "camera_width": 640,
                        "camera_height": 480,
                        "publish_rate": 30.0,
                        "output_image_topic": "/camera/image_raw",
                        "id_topic": "/aruco_id",
                        "frame_id": "webcam_frame",
                    }
                ],
            ),
            Node(
                package="aruco_tracker",
                executable="aruco_viewer",
                name="aruco_viewer",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera/image_raw",
                    }
                ],
            ),
        ]
    )
