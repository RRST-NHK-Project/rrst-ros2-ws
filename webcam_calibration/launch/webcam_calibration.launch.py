from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Webカメラキャリブレーションノードを起動するlaunchファイル."""
    return LaunchDescription(
        [
            Node(
                package="webcam_calibration",
                executable="webcam_calibrator",
                name="webcam_calibrator",
                output="screen",
                parameters=[
                    {
                        "camera_index": 2,
                        "board_rows": 6,
                        "board_cols": 9,
                        "square_size_m": 0.024,
                        "required_samples": 20,
                        "output_yaml": "/home/dev/ros2_ws/src/webcam_calibration/calibration/webcam_calibration.yaml",
                        "camera_name": "webcam",
                        "show_undistorted": True,
                    }
                ],
            ),
        ]
    )
