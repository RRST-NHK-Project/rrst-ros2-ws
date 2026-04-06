from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Webカメラを直接開いてArUco検出するlaunchファイル."""
    camera_index = LaunchConfiguration("camera_index")
    marker_length = LaunchConfiguration("marker_length")
    calibration_yaml = LaunchConfiguration("calibration_yaml")
    publish_rate = LaunchConfiguration("publish_rate")
    frame_width = LaunchConfiguration("frame_width")
    frame_height = LaunchConfiguration("frame_height")
    launch_viewer = LaunchConfiguration("launch_viewer")

    default_yaml = os.path.join(
        get_package_share_directory("aruco_tracker"),
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
                "marker_length",
                default_value="0.08",
                description="Marker side length in meters",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="30.0",
                description="Camera publish rate in FPS",
            ),
            DeclareLaunchArgument(
                "frame_width",
                default_value="640",
                description="Camera frame width",
            ),
            DeclareLaunchArgument(
                "frame_height",
                default_value="480",
                description="Camera frame height",
            ),
            DeclareLaunchArgument(
                "calibration_yaml",
                default_value=default_yaml,
                description="Path to camera calibration YAML (camera_info format)",
            ),
            DeclareLaunchArgument(
                "launch_viewer",
                default_value="false",
                description="Launch the detection viewer window",
            ),
            Node(
                package="aruco_tracker",
                executable="aruco_webcam_detector",
                name="aruco_webcam_detector",
                output="screen",
                parameters=[
                    {
                        "camera_index": camera_index,
                        "camera_width": frame_width,
                        "camera_height": frame_height,
                        "publish_rate": publish_rate,
                        "output_image_topic": "/camera/image_raw",
                        "id_topic": "/aruco_id",
                        "pose_topic": "/aruco_pose",
                        "distance_topic": "/aruco_distance",
                        "frame_id": "webcam_frame",
                        "marker_length": marker_length,
                        "camera_info_yaml": calibration_yaml,
                        "camera_fourcc": "YUYV",
                    }
                ],
            ),
            Node(
                package="aruco_tracker",
                executable="aruco_viewer",
                name="aruco_viewer",
                output="screen",
                condition=IfCondition(launch_viewer),
                parameters=[
                    {
                        "image_topic": "/camera/image_raw",
                    }
                ],
            ),
        ]
    )
