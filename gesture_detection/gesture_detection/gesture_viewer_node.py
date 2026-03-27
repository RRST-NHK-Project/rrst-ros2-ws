import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


class GestureViewerNode(Node):
    """ジェスチャー可視化画像と推論結果を表示するビューアノード."""

    def __init__(self):
        super().__init__('gesture_viewer')

        self.declare_parameter('image_topic', '/gesture_detection/image')
        self.declare_parameter('label_topic', '/gesture_detection/label')
        self.declare_parameter('confidence_topic', '/gesture_detection/confidence')

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._bridge = CvBridge()
        self._window_name = 'Gesture Detection'
        self._last_frame_time = None
        self._last_label = 'none'
        self._last_confidence = 0.0

        image_topic = str(self.get_parameter('image_topic').value)
        label_topic = str(self.get_parameter('label_topic').value)
        confidence_topic = str(self.get_parameter('confidence_topic').value)

        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        self.create_subscription(Image, image_topic, self._image_callback, qos)
        self.create_subscription(String, label_topic, self._label_callback, qos)
        self.create_subscription(Float32, confidence_topic, self._confidence_callback, qos)
        self.create_timer(2.0, self._health_check)

        self.get_logger().info(
            f'GestureViewer起動: image={image_topic}, label={label_topic}, confidence={confidence_topic}'
        )

    def _label_callback(self, msg: String):
        self._last_label = msg.data

    def _confidence_callback(self, msg: Float32):
        self._last_confidence = float(msg.data)

    def _image_callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'画像変換失敗: {exc}')
            return

        text = f'Gesture: {self._last_label} ({self._last_confidence:.2f})'
        cv2.putText(
            frame,
            text,
            (16, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        self._last_frame_time = self.get_clock().now()
        cv2.imshow(self._window_name, frame)
        cv2.waitKey(1)

    def _health_check(self):
        if self._last_frame_time is None:
            self.get_logger().warn('画像受信待機中...')
            return

        elapsed = self.get_clock().now() - self._last_frame_time
        if elapsed > Duration(seconds=2.0):
            self.get_logger().warn('2秒以上フレーム未受信です。トピックを確認してください。')


def main(args=None):
    rclpy.init(args=args)
    node = GestureViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
