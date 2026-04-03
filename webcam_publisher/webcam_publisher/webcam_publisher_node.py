import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image


class WebcamPublisherNode(Node):
    """Webカメラ映像をsensor_msgs/Imageで配信するノード."""

    def __init__(self) -> None:
        super().__init__("webcam_publisher")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("topic_name", "/webcam/image_raw")
        self.declare_parameter("frame_id", "webcam_frame")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.topic_name = str(self.get_parameter("topic_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)

        self.publisher_ = self.create_publisher(Image, self.topic_name, 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(self.camera_index)
        if self.frame_width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        if self.frame_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {self.camera_index}.")

        if self.publish_rate <= 0.0:
            self.get_logger().warning("publish_rate must be > 0.0. Fallback to 30.0")
            self.publish_rate = 30.0

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self._publish_frame)
        self.get_logger().info(
            f"Publishing camera {self.camera_index} to {self.topic_name} at {self.publish_rate:.1f} FPS"
        )

    def _publish_frame(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Failed to read frame from webcam.")
            return

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.frame_id
        self.publisher_.publish(image_msg)

    def destroy_node(self) -> bool:
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WebcamPublisherNode()
        rclpy.spin(node)
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"[webcam_publisher] {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
