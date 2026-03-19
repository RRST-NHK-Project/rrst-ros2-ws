import math
import numpy as np
import cv2
from cv2 import aruco

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ゼロ除算を防ぐための微小値
_EPSILON = 1e-6


class ArucoDetectorNode(Node):
    """ArUcoマーカーを検出し、座標・距離・IDをトピックとして発行するノード。"""

    def __init__(self):
        super().__init__('aruco_detector')

        # --- パラメータ宣言 ---
        self.declare_parameter('camera_index', 4)
        self.declare_parameter('marker_length', 0.05)  # メートル単位
        self.declare_parameter('fps', 30.0)
        # カメラキャリブレーション済みパラメータ
        # (camera_calibration ノードで取得した値に変更してください)
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        cam_index = self.get_parameter('camera_index').value
        self.marker_length = self.get_parameter('marker_length').value
        fps = self.get_parameter('fps').value
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value

        # --- Publishers ---
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.id_pub = self.create_publisher(Int32, '/aruco/id', 10)
        self.distance_pub = self.create_publisher(Float32, '/aruco/distance', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge()

        # --- カメラ初期化 ---
        self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'カメラ (index={cam_index}) が開けません')
            raise RuntimeError('Failed to open camera')

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # --- ArUco 辞書 / 検出パラメータ ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        # OpenCV バージョンに応じて API を選択
        try:
            self.parameters = aruco.DetectorParameters()
        except AttributeError:
            self.parameters = aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_NONE

        # --- カメラ内部パラメータ ---
        # camera_calibration ノードでキャリブレーションした値を ROS2 パラメータとして
        # 起動時に渡してください（デフォルト値は暫定値です）:
        #   ros2 run aruco_tracker aruco_detector --ros-args -p fx:=… -p fy:=… -p cx:=… -p cy:=…
        self.camera_matrix = np.array([
            [fx,   0.0,  cx],
            [ 0.0, fy,   cy],
            [ 0.0,  0.0,  1.0],
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # --- タイマー ---
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info('ArucoDetectorNode が起動しました')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('フレーム取得失敗')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        # カメラ画像を常に配信
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera'
        self.image_pub.publish(img_msg)

        if ids is None:
            return

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            self.camera_matrix,
            self.dist_coeffs,
        )

        stamp = self.get_clock().now().to_msg()

        for i, marker_id in enumerate(ids.flatten()):
            t = tvecs[i][0]   # [tx, ty, tz]
            r = rvecs[i][0]   # Rodrigues ベクトル

            # --- 距離（ユークリッド） ---
            distance = float(math.sqrt(t[0] ** 2 + t[1] ** 2 + t[2] ** 2))

            # --- PoseStamped ---
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = 'camera'
            pose_msg.pose.position.x = float(t[0])
            pose_msg.pose.position.y = float(t[1])
            pose_msg.pose.position.z = float(t[2])

            # Rodrigues → 回転行列 → クォータニオン
            R, _ = cv2.Rodrigues(r)
            trace = R[0, 0] + R[1, 1] + R[2, 2]
            qw = float(math.sqrt(max(0.0, 1.0 + trace)) / 2.0)
            denom = 4.0 * qw if qw > _EPSILON else _EPSILON
            qx = float((R[2, 1] - R[1, 2]) / denom)
            qy = float((R[0, 2] - R[2, 0]) / denom)
            qz = float((R[1, 0] - R[0, 1]) / denom)

            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            # --- ID ---
            id_msg = Int32()
            id_msg.data = int(marker_id)

            # --- 距離メッセージ ---
            dist_msg = Float32()
            dist_msg.data = distance

            self.pose_pub.publish(pose_msg)
            self.id_pub.publish(id_msg)
            self.distance_pub.publish(dist_msg)

            self.get_logger().info(
                f'ID:{marker_id}  '
                f'Pos=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m  '
                f'距離={distance:.3f} m'
            )

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
