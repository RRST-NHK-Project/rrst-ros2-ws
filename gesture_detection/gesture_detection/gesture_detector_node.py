import cv2
import os
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

try:
    import mediapipe as mp  # type: ignore[import-not-found]
    from mediapipe.tasks import python as mp_python  # type: ignore[import-not-found]
    from mediapipe.tasks.python import vision as mp_vision  # type: ignore[import-not-found]
except ImportError:
    mp = None
    mp_python = None
    mp_vision = None


class GestureDetectorNode(Node):
    """MediaPipeでジェスチャー検知を行い結果をPublishするノード."""

    def __init__(self):
        super().__init__("gesture_detector")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_image_topic", "/gesture_detection/image")
        self.declare_parameter("label_topic", "/gesture_detection/label")
        self.declare_parameter("confidence_topic", "/gesture_detection/confidence")

        # MediaPipe Gesture Recognizer parameters
        self.declare_parameter("mediapipe_model_path", "gesture_recognizer.task")
        self.declare_parameter("mediapipe_num_hands", 1)
        self.declare_parameter("mediapipe_min_hand_detection_confidence", 0.5)
        self.declare_parameter("mediapipe_min_hand_presence_confidence", 0.5)
        self.declare_parameter("mediapipe_min_tracking_confidence", 0.5)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._bridge = CvBridge()
        self._frame_count = 0
        self._last_label = "none"
        self._last_confidence = 0.0
        self._mp_recognizer = None

        image_topic = str(self.get_parameter("image_topic").value)
        output_image_topic = str(self.get_parameter("output_image_topic").value)
        label_topic = str(self.get_parameter("label_topic").value)
        confidence_topic = str(self.get_parameter("confidence_topic").value)

        self._image_pub = self.create_publisher(Image, output_image_topic, qos)
        self._label_pub = self.create_publisher(String, label_topic, qos)
        self._confidence_pub = self.create_publisher(Float32, confidence_topic, qos)
        self.create_subscription(Image, image_topic, self._image_callback, qos)

        self._init_mediapipe_backend(image_topic)

    def _init_mediapipe_backend(self, image_topic: str):
        if mp is None or mp_python is None or mp_vision is None:
            raise RuntimeError(
                "mediapipe が未インストールです。pip install mediapipe を実行してください。"
            )

        model_path = str(self.get_parameter("mediapipe_model_path").value)
        if not os.path.exists(model_path):
            raise RuntimeError(
                f"MediaPipeモデルが見つかりません: {model_path} (gesture_recognizer.task を配置してください)"
            )

        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = mp_vision.GestureRecognizerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.IMAGE,
            num_hands=int(self.get_parameter("mediapipe_num_hands").value),
            min_hand_detection_confidence=float(
                self.get_parameter("mediapipe_min_hand_detection_confidence").value
            ),
            min_hand_presence_confidence=float(
                self.get_parameter("mediapipe_min_hand_presence_confidence").value
            ),
            min_tracking_confidence=float(
                self.get_parameter("mediapipe_min_tracking_confidence").value
            ),
        )
        self._mp_recognizer = mp_vision.GestureRecognizer.create_from_options(options)
        self.get_logger().info(
            f"GestureDetector起動(MediaPipe): image={image_topic}, model={model_path}"
        )

    def _image_callback(self, msg: Image):
        self._frame_count += 1

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"画像変換失敗: {exc}")
            return

        vis = frame.copy()
        best_label, best_conf = self._run_mediapipe_inference(frame, vis)

        self._last_label = best_label
        self._last_confidence = best_conf

        label_msg = String()
        label_msg.data = self._last_label
        self._label_pub.publish(label_msg)

        conf_msg = Float32()
        conf_msg.data = float(self._last_confidence)
        self._confidence_pub.publish(conf_msg)

        out_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out_msg.header = msg.header
        self._image_pub.publish(out_msg)

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"gesture={self._last_label}, conf={self._last_confidence:.2f}"
            )

    def _run_mediapipe_inference(self, frame, vis):
        """MediaPipe Gesture Recognizerで推論し、結果を描画して返す."""
        if self._mp_recognizer is None:
            return "none", 0.0

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._mp_recognizer.recognize(mp_image)

        best_label = "none"
        best_conf = 0.0

        hand_landmarks = result.hand_landmarks if result.hand_landmarks else []
        for landmarks in hand_landmarks:
            for lm in landmarks:
                x_px = int(lm.x * vis.shape[1])
                y_px = int(lm.y * vis.shape[0])
                cv2.circle(vis, (x_px, y_px), 2, (255, 255, 0), -1)

        gestures = result.gestures if result.gestures else []
        for gesture_list in gestures:
            if not gesture_list:
                continue
            top = gesture_list[0]
            label = str(top.category_name)
            conf = float(top.score)
            if conf > best_conf:
                best_label = label
                best_conf = conf

        cv2.putText(
            vis,
            f"{best_label} {best_conf:.2f}",
            (20, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        return best_label, best_conf


def main(args=None):
    rclpy.init(args=args)
    node = GestureDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
