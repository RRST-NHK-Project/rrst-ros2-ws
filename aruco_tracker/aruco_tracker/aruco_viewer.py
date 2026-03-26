# 映像表示用プログラム

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
import cv2
from rclpy.duration import Duration

class ArucoViewer(Node):
    def __init__(self):
        super().__init__('aruco_viewer')
        self.bridge = CvBridge()
        self.declare_parameter('image_topic', '/camera/image_raw')
        image_topic = str(self.get_parameter('image_topic').value)

        self._last_frame_time = None
        self._window_name = 'Camera View'
        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)

        # Latest detection results
        self._marker_id = None
        self._marker_pos = None  # (x, y, z)
        self._marker_distance = None

        self.sub_image = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/aruco_pose', self.pose_callback, 10)
        self.sub_distance = self.create_subscription(Float32, '/aruco_distance', self.distance_callback, 10)
        self.sub_id = self.create_subscription(Int32, '/aruco_id', self.id_callback, 10)
        
        self.timer = self.create_timer(2.0, self.health_check)
        self.get_logger().info(f'Viewer subscribed to: {image_topic}')

    def id_callback(self, msg):
        self._marker_id = msg.data

    def distance_callback(self, msg):
        self._marker_distance = msg.data

    def pose_callback(self, msg):
        self._marker_pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge変換失敗: {exc}')
            return

        # Draw detection info on frame
        if self._marker_id is not None and self._marker_pos is not None:
            x, y, z = self._marker_pos
            marker_id = self._marker_id
            distance = self._marker_distance if self._marker_distance is not None else 0.0

            # Draw ID and distance on top-left
            text_id = f'ID: {marker_id}'
            text_pos_x = f'X: {x:.3f}m'
            text_pos_y = f'Y: {y:.3f}m'
            text_pos_z = f'Z: {z:.3f}m'
            text_dist = f'Distance: {distance:.3f}m'

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            color = (0, 255, 0)  # Green
            thickness = 2
            line_type = cv2.LINE_AA

            y_offset = 30
            cv2.putText(frame, text_id, (10, y_offset), font, font_scale, color, thickness, line_type)
            cv2.putText(frame, text_pos_x, (10, y_offset + 25), font, font_scale, color, thickness, line_type)
            cv2.putText(frame, text_pos_y, (10, y_offset + 50), font, font_scale, color, thickness, line_type)
            cv2.putText(frame, text_pos_z, (10, y_offset + 75), font, font_scale, color, thickness, line_type)
            cv2.putText(frame, text_dist, (10, y_offset + 100), font, font_scale, color, thickness, line_type)

        self._last_frame_time = self.get_clock().now()
        cv2.imshow(self._window_name, frame)
        cv2.waitKey(1)

    def health_check(self):
        if self._last_frame_time is None:
            self.get_logger().warn('まだ画像を受信していません。トピック名と配信状態を確認してください。')
            return

        elapsed = self.get_clock().now() - self._last_frame_time
        if elapsed > Duration(seconds=2.0):
            self.get_logger().warn('2秒以上フレーム未受信です。トピックが止まっている可能性があります。')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoViewer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
