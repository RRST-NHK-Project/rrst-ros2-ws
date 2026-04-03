import cv2
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image


class WebcamViewerNode(Node):
    """sensor_msgs/Imageを受信してOpenCVウィンドウで表示するノード."""

    def __init__(self) -> None:
        super().__init__("webcam_viewer")

        self.declare_parameter("topic_name", "/webcam/image_raw")
        self.declare_parameter("window_name", "Webcam Viewer")

        self.topic_name = str(self.get_parameter("topic_name").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self._image_callback,
            10,
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"Subscribing image topic: {self.topic_name}")

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Failed to convert image message: {exc}")
            return

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebcamViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
