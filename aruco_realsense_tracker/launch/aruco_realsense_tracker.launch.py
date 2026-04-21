from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    marker_length = LaunchConfiguration("marker_length")
    camera_image_topic = LaunchConfiguration("camera_image_topic")
    frame_id = LaunchConfiguration("frame_id")
    calibration_yaml = LaunchConfiguration("calibration_yaml")

    default_yaml = os.path.join(
        get_package_share_directory("aruco_realsense_tracker"),
        "calibration",
        "realsense_calibration.yaml",
    )

    realsense_bringup = os.path.join(
        get_package_share_directory("realsense_ros2"),
        "launch",
        "bringup.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "marker_length",
                default_value="0.08",
                description="Marker side length in meters",
            ),
            DeclareLaunchArgument(
                "camera_image_topic",
                default_value="/camera/color/image_raw",
                description="Input image topic from RealSense",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera_color_optical_frame",
                description="Frame ID for published pose",
            ),
            DeclareLaunchArgument(
                "calibration_yaml",
                default_value=default_yaml,
                description="Path to camera calibration YAML (camera_info format)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_bringup),
            ),
            Node(
                package="aruco_realsense_tracker",
                executable="aruco_realsense_detector",
                name="aruco_realsense_detector",
                output="screen",
                parameters=[
                    {
                        "input_image_topic": camera_image_topic,
                        "id_topic": "/aruco_id",
                        "pose_topic": "/aruco_pose",
                        "distance_topic": "/aruco_distance",
                        "frame_id": frame_id,
                        "marker_length": marker_length,
                        "camera_info_yaml": calibration_yaml,
                    }
                ],
            ),
        ]
    )
