from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class KfsCubeFusionViewerNode(Node):
    def __init__(self) -> None:
        super().__init__("kfs_cube_fusion_viewer")

        self.declare_parameter("image_topic", "/kfs_cube_fusion/debug_image")
        self.declare_parameter("window_name", "kfs_cube_fusion debug viewer")

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.bridge = CvBridge()
        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self._last_image = None

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.startWindowThread()

        self.timer = self.create_timer(0.03, self._pump_gui)

        self.get_logger().info(f"viewer started: image_topic={self.image_topic}")

    def image_callback(self, msg: Image) -> None:
        try:
            self._last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")

    def _pump_gui(self) -> None:
        if self._last_image is None:
            canvas = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                canvas,
                f"waiting for {self.image_topic}",
                (20, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(self.window_name, canvas)
        else:
            cv2.imshow(self.window_name, self._last_image)
        cv2.waitKey(1)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = KfsCubeFusionViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
