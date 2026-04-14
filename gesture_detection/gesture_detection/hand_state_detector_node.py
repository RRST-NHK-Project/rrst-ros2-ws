import time
import urllib.request
from pathlib import Path

import cv2
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
except ImportError:
    mp = None


DEFAULT_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/1/hand_landmarker.task"
)
DEFAULT_MODEL_PATH = str(
    Path.home() / ".ros" / "gesture_detection" / "hand_landmarker.task"
)


class HandStateDetectorNode(Node):
    """MediaPipe Hand Landmarkerで手の開閉状態を判定してPublishするノード."""

    def __init__(self):
        super().__init__("hand_state_detector")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_image_topic", "/gesture_detection/image")
        self.declare_parameter("label_topic", "/gesture_detection/label")
        self.declare_parameter("confidence_topic", "/gesture_detection/confidence")
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("model_url", DEFAULT_MODEL_URL)
        self.declare_parameter("auto_download_model", True)
        self.declare_parameter("num_hands", 1)
        self.declare_parameter("min_hand_detection_confidence", 0.7)
        self.declare_parameter("min_tracking_confidence", 0.7)
        self.declare_parameter("mirror_image", True)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._bridge = CvBridge()
        self._mirror_image = bool(self.get_parameter("mirror_image").value)

        image_topic = str(self.get_parameter("image_topic").value)
        output_image_topic = str(self.get_parameter("output_image_topic").value)
        label_topic = str(self.get_parameter("label_topic").value)
        confidence_topic = str(self.get_parameter("confidence_topic").value)

        self._image_pub = self.create_publisher(Image, output_image_topic, qos)
        self._label_pub = self.create_publisher(String, label_topic, qos)
        self._confidence_pub = self.create_publisher(Float32, confidence_topic, qos)
        self.create_subscription(Image, image_topic, self._image_callback, qos)

        self._landmarker = self._create_landmarker()

        self.get_logger().info(
            f"HandStateDetector起動: image={image_topic}, output={output_image_topic}, "
            f"label={label_topic}, confidence={confidence_topic}"
        )

    def _ensure_model_exists(
        self, model_path: Path, model_url: str, auto_download: bool
    ) -> Path:
        if model_path.exists():
            return model_path

        if not auto_download:
            raise RuntimeError(
                f"モデルが存在しません: {model_path}. auto_download_model=true にするか、モデルを配置してください。"
            )

        model_path.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"モデルをダウンロードします: {model_path}")
        urllib.request.urlretrieve(model_url, model_path)
        return model_path

    def _create_landmarker(self):
        if mp is None:
            raise RuntimeError(
                "mediapipe が未インストールです。pip install mediapipe を実行してください。"
            )

        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        model_url = str(self.get_parameter("model_url").value)
        auto_download = bool(self.get_parameter("auto_download_model").value)

        model_path = self._ensure_model_exists(model_path, model_url, auto_download)

        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=int(self.get_parameter("num_hands").value),
            min_hand_detection_confidence=float(
                self.get_parameter("min_hand_detection_confidence").value
            ),
            min_tracking_confidence=float(
                self.get_parameter("min_tracking_confidence").value
            ),
        )
        return mp.tasks.vision.HandLandmarker.create_from_options(options)

    @staticmethod
    def _classify_hand_state(hand_landmarks):
        extended = 0
        for tip_id, pip_id in ((8, 6), (12, 10), (16, 14), (20, 18)):
            if hand_landmarks[tip_id].y < hand_landmarks[pip_id].y:
                extended += 1

        state = "Open" if extended >= 3 else "Closed"
        confidence = float(extended) / 4.0
        return state, confidence

    def _image_callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"画像変換失敗: {exc}")
            return

        if self._mirror_image:
            frame = cv2.flip(frame, 1)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = time.monotonic_ns() // 1_000_000
        result = self._landmarker.detect_for_video(mp_image, int(timestamp_ms))

        if result.hand_landmarks:
            hand_state, confidence = self._classify_hand_state(result.hand_landmarks[0])
        else:
            hand_state, confidence = "No Hand", 0.0

        state_color = (
            (0, 255, 0)
            if hand_state == "Open"
            else (0, 0, 255) if hand_state == "Closed" else (200, 200, 200)
        )

        cv2.putText(
            frame,
            f"State: {hand_state}",
            (10, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            state_color,
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"Confidence: {confidence:.2f}",
            (10, 145),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        label_msg = String()
        label_msg.data = hand_state
        self._label_pub.publish(label_msg)

        conf_msg = Float32()
        conf_msg.data = float(confidence)
        self._confidence_pub.publish(conf_msg)

        out_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out_msg.header = msg.header
        self._image_pub.publish(out_msg)

    def destroy_node(self):
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandStateDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
