import cv2
from cv_bridge import CvBridge
import os
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import CameraInfo, Image
import yaml


class WebcamPublisherNode(Node):
    """Webカメラ映像をsensor_msgs/Imageで配信するノード."""

    def __init__(self) -> None:
        super().__init__("webcam_publisher")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("topic_name", "/webcam/image_raw")
        self.declare_parameter("camera_info_topic", "/webcam/camera_info")
        self.declare_parameter("frame_id", "webcam_frame")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("camera_info_yaml", "")
        self.declare_parameter("camera_name", "webcam")

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.topic_name = str(self.get_parameter("topic_name").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.camera_info_yaml = str(self.get_parameter("camera_info_yaml").value)
        self.camera_name = str(self.get_parameter("camera_name").value)

        self.publisher_ = self.create_publisher(Image, self.topic_name, 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.camera_info_topic, 10
        )
        self.bridge = CvBridge()
        self.camera_info_msg = self._load_camera_info()

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
            "Publishing camera "
            f"{self.camera_index} to {self.topic_name} / {self.camera_info_topic} "
            f"at {self.publish_rate:.1f} FPS"
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

        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = image_msg.header.stamp
            self.camera_info_msg.header.frame_id = self.frame_id
            self.camera_info_pub.publish(self.camera_info_msg)

    def _load_camera_info(self):
        if not self.camera_info_yaml:
            self.get_logger().warning(
                "camera_info_yaml is empty. CameraInfo will not be published."
            )
            return None

        yaml_path = os.path.abspath(self.camera_info_yaml)
        if not os.path.exists(yaml_path):
            self.get_logger().warning(
                f"camera_info_yaml not found: {yaml_path}. CameraInfo will not be published."
            )
            return None

        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)

            msg = CameraInfo()
            msg.width = int(data.get("image_width", self.frame_width))
            msg.height = int(data.get("image_height", self.frame_height))
            msg.distortion_model = str(data.get("distortion_model", "plumb_bob"))
            msg.d = [float(v) for v in data["distortion_coefficients"]["data"]]
            msg.k = [float(v) for v in data["camera_matrix"]["data"]]
            msg.r = [
                float(v)
                for v in data.get("rectification_matrix", {}).get(
                    "data", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                )
            ]
            msg.p = [
                float(v)
                for v in data.get("projection_matrix", {}).get("data", [0.0] * 12)
            ]

            if "camera_name" in data and data["camera_name"]:
                self.camera_name = str(data["camera_name"])

            self.get_logger().info(f"Loaded camera calibration: {yaml_path}")
            return msg
        except Exception as exc:
            self.get_logger().error(
                f"Failed to load camera_info_yaml ({yaml_path}): {exc}"
            )
            return None

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
