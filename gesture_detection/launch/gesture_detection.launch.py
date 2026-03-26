from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """YOLOジェスチャー検知ノードとビューアノードを同時起動する."""
    return LaunchDescription([
        Node(
            package='gesture_detection',
            executable='gesture_detector',
            name='gesture_detector',
            output='screen',
            parameters=[{
                'image_topic': '/camera/camera/color/image_raw',
                'output_image_topic': '/gesture_detection/image',
                'label_topic': '/gesture_detection/label',
                'confidence_topic': '/gesture_detection/confidence',
                'backend': 'mediapipe',
                # MediaPipe Gesture Recognizer (.task) を使う場合
                'mediapipe_model_path': PathJoinSubstitution([
                    FindPackageShare('gesture_detection'),
                    'models',
                    'gesture_recognizer.task',
                ]),
                'mediapipe_num_hands': 1,
                'mediapipe_min_hand_detection_confidence': 0.5,
                'mediapipe_min_hand_presence_confidence': 0.5,
                'mediapipe_min_tracking_confidence': 0.5,
                # backend='yolo' に切り替える場合の設定
                'yolo_model_path': 'yolov8n.pt',
                'yolo_conf': 0.35,
                'yolo_iou': 0.45,
                'yolo_imgsz': 640,
                'infer_interval': 2,
                # 公式YOLO(COCO)を使う場合は person のみを対象にする。
                'target_class_names': ['person'],
            }],
        ),
        Node(
            package='gesture_detection',
            executable='gesture_viewer',
            name='gesture_viewer',
            output='screen',
            parameters=[{
                'image_topic': '/gesture_detection/image',
                'label_topic': '/gesture_detection/label',
                'confidence_topic': '/gesture_detection/confidence',
            }],
        ),
    ])
