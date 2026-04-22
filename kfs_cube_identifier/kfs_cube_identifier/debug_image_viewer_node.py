from __future__ import annotations

import site
import sys
from typing import Optional, Sequence

# Prevent ABI mismatch with ROS binary modules when user-site has NumPy 2.x.
USER_SITE = site.getusersitepackages()
if USER_SITE in sys.path:
    sys.path.remove(USER_SITE)

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DebugImageViewerNode(Node):
    def __init__(self) -> None:
        super().__init__("kfs_debug_image_viewer")
        self.bridge = CvBridge()
        self._window_ok = True

        self.declare_parameter("image_topic", "/kfs_cube_identifier/debug_image")
        self.declare_parameter("window_name", "KFS Debug Image")

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self._last_frame_received = False

        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self._timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(f"Debug viewer subscribed: {self.image_topic}")

    def _tick(self) -> None:
        if not self._window_ok or self._last_frame_received:
            return
        try:
            frame = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "Waiting for /kfs_cube_identifier/debug_image ...",
                (20, 190),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
        except cv2.error as exc:
            self._window_ok = False
            self.get_logger().error(f"OpenCV window error. Disabling viewer: {exc}")

    def _on_image(self, msg: Image) -> None:
        if not self._window_ok:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._last_frame_received = True
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
        except cv2.error as exc:
            self._window_ok = False
            self.get_logger().error(f"OpenCV window error. Disabling viewer: {exc}")

    def destroy_node(self) -> bool:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = DebugImageViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
