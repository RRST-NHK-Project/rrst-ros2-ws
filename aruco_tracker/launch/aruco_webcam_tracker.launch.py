from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Webカメラ入力のArUco検出ノードとビューアを同時起動するlaunchファイル."""
    camera_index = LaunchConfiguration("camera_index")
    marker_length = LaunchConfiguration("marker_length")
    camera_info_yaml = LaunchConfiguration("camera_info_yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_index",
                default_value="0",
                description="OpenCV camera index",
            ),
            DeclareLaunchArgument(
                "marker_length",
                default_value="0.05",
                description="Marker side length in meters",
            ),
            DeclareLaunchArgument(
                "camera_info_yaml",
                default_value="",
                description="Path to camera calibration YAML (camera_info format)",
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
                        "marker_length": marker_length,
                        "camera_info_yaml": camera_info_yaml,
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
