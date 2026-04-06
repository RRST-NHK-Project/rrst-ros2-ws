from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """キャリブレーション済みWebカメラPublishとViewerを同時起動するlaunchファイル."""
    camera_index = LaunchConfiguration("camera_index")
    calibration_yaml = LaunchConfiguration("calibration_yaml")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    default_yaml = os.path.join(
        get_package_share_directory("webcam_publisher"),
        "calibration",
        "webcam_calibration.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_index",
                default_value="2",
                description="OpenCV camera index",
            ),
            DeclareLaunchArgument(
                "calibration_yaml",
                default_value=default_yaml,
                description="Path to camera calibration YAML",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/webcam/image_raw",
                description="Image topic name",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/webcam/camera_info",
                description="CameraInfo topic name",
            ),
            Node(
                package="webcam_publisher",
                executable="webcam_publisher",
                name="webcam_publisher",
                output="screen",
                parameters=[
                    {
                        "camera_index": camera_index,
                        "topic_name": image_topic,
                        "camera_info_topic": camera_info_topic,
                        "frame_id": "webcam_frame",
                        "publish_rate": 30.0,
                        "frame_width": 640,
                        "frame_height": 480,
                        "camera_info_yaml": calibration_yaml,
                    }
                ],
            ),
            Node(
                package="webcam_viewer",
                executable="webcam_viewer",
                name="webcam_viewer",
                output="screen",
                parameters=[
                    {
                        "topic_name": image_topic,
                        "window_name": "Webcam Viewer",
                    }
                ],
            ),
        ]
    )
