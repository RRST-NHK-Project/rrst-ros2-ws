from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """キャリブレーションYAMLを使ってWebカメラを起動するlaunchファイル."""
    camera_index = LaunchConfiguration("camera_index")
    calibration_yaml = LaunchConfiguration("calibration_yaml")

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
            Node(
                package="webcam_publisher",
                executable="webcam_publisher",
                name="webcam_publisher",
                output="screen",
                parameters=[
                    {
                        "camera_index": camera_index,
                        "topic_name": "/webcam/image_raw",
                        "camera_info_topic": "/webcam/camera_info",
                        "frame_id": "webcam_frame",
                        "publish_rate": 30.0,
                        "frame_width": 640,
                        "frame_height": 480,
                        "camera_info_yaml": calibration_yaml,
                    }
                ],
            ),
        ]
    )
