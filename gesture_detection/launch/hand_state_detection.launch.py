from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Hand state detectorとビューアを同時起動する."""
    return LaunchDescription(
        [
            Node(
                package="gesture_detection",
                executable="hand_state_detector",
                name="hand_state_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera/camera/color/image_raw",
                        "output_image_topic": "/gesture_detection/image",
                        "label_topic": "/gesture_detection/label",
                        "confidence_topic": "/gesture_detection/confidence",
                        "auto_download_model": True,
                        "num_hands": 1,
                        "min_hand_detection_confidence": 0.7,
                        "min_tracking_confidence": 0.7,
                        "mirror_image": True,
                    }
                ],
            ),
            Node(
                package="gesture_detection",
                executable="gesture_viewer",
                name="gesture_viewer",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/gesture_detection/image",
                        "label_topic": "/gesture_detection/label",
                        "confidence_topic": "/gesture_detection/confidence",
                    }
                ],
            ),
        ]
    )
