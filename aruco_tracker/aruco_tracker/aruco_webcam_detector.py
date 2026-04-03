import cv2
from cv2 import aruco
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class ArucoWebcamDetector(Node):
    """Webカメラ映像からArUcoを検出して描画画像とIDを配信するノード."""

    def __init__(self):
        super().__init__("aruco_webcam_detector")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("output_image_topic", "/camera/image_raw")
        self.declare_parameter("id_topic", "/aruco_id")
        self.declare_parameter("frame_id", "webcam_frame")

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.output_image_topic = str(self.get_parameter("output_image_topic").value)
        self.id_topic = str(self.get_parameter("id_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)
        self.id_pub = self.create_publisher(Int32, self.id_topic, 10)

        self.cap = cv2.VideoCapture(self.camera_index)
        if self.camera_width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        if self.camera_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {self.camera_index}")

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        try:
            self.parameters = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
            self._use_new_api = True
        except AttributeError:
            self.parameters = aruco.DetectorParameters_create()  # type: ignore[attr-defined]
            self._use_new_api = False

        if self.publish_rate <= 0.0:
            self.publish_rate = 30.0

        self.timer = self.create_timer(1.0 / self.publish_rate, self._process_frame)
        self.get_logger().info(
            f"Webcam detector started: camera_index={self.camera_index}, topic={self.output_image_topic}"
        )

    def _detect_markers(self, gray_image):
        if self._use_new_api:
            return self.detector.detectMarkers(gray_image)
        return aruco.detectMarkers(  # type: ignore[attr-defined]
            gray_image,
            self.aruco_dict,
            parameters=self.parameters,
        )

    def _process_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Failed to read webcam frame.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)

        if ids is not None and len(corners) > 0:
            aruco.drawDetectedMarkers(frame, corners, ids)
            first_id = Int32()
            first_id.data = int(ids[0][0])
            self.id_pub.publish(first_id)

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.frame_id
        self.image_pub.publish(image_msg)

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoWebcamDetector()
        rclpy.spin(node)
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"[aruco_webcam_detector] {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
