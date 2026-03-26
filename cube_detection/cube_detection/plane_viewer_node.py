# Copyright 2024 RRST-NHK-Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class PlaneViewerNode(Node):
    """平面検知の可視化画像を表示するビューアノード."""

    def __init__(self):
        super().__init__('plane_viewer')
        self._bridge = CvBridge()

        self.declare_parameter('image_topic', '/plane_detection/image')
        image_topic = str(self.get_parameter('image_topic').value)

        self._last_frame_time = None
        self._window_name = 'Plane Detection'
        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)

        self._plane_pos = None
        self._plane_distance = None

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, image_topic, self._image_callback, qos)
        self.create_subscription(
            PoseStamped, '/plane_detection/pose', self._pose_callback, qos
        )
        self.create_subscription(
            Float32, '/plane_detection/distance', self._distance_callback, qos
        )

        self.create_timer(2.0, self._health_check)
        self.get_logger().info(f'PlaneViewer起動: {image_topic} を購読中')

    def _pose_callback(self, msg: PoseStamped):
        self._plane_pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def _distance_callback(self, msg: Float32):
        self._plane_distance = msg.data

    def _image_callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge 変換失敗: {exc}')
            return

        # 左上に平面情報を常時表示
        if self._plane_pos is not None:
            x, y, z = self._plane_pos
            dist = self._plane_distance if self._plane_distance is not None else 0.0
            font = cv2.FONT_HERSHEY_SIMPLEX
            color = (0, 255, 255)
            cv2.rectangle(frame, (6, 6), (360, 120), (20, 20, 20), -1)
            cv2.putText(frame, 'PLANE TRACKING', (14, 30), font, 0.7, color, 2, cv2.LINE_AA)
            cv2.putText(frame, f'X: {x:.3f}m', (14, 55), font, 0.6, color, 2, cv2.LINE_AA)
            cv2.putText(frame, f'Y: {y:.3f}m', (14, 80), font, 0.6, color, 2, cv2.LINE_AA)
            cv2.putText(frame, f'Z: {z:.3f}m', (190, 55), font, 0.6, color, 2, cv2.LINE_AA)
            cv2.putText(
                frame, f'Dist: {dist:.3f}m', (190, 80), font, 0.6, color, 2, cv2.LINE_AA
            )

        self._last_frame_time = self.get_clock().now()
        cv2.imshow(self._window_name, frame)
        cv2.waitKey(1)

    def _health_check(self):
        if self._last_frame_time is None:
            self.get_logger().warn(
                '画像受信待機中... トピック配信状態を確認してください。'
            )
            return
        elapsed = self.get_clock().now() - self._last_frame_time
        if elapsed > Duration(seconds=2.0):
            self.get_logger().warn('2秒以上フレーム未受信です。')


def main(args=None):
    rclpy.init(args=args)
    node = PlaneViewerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
