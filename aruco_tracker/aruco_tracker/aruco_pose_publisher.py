import cv2
import numpy as np
from cv2 import aruco
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__("aruco_pose_publisher")

        # Camera topics (assume RealSense D456 node is already running)
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("output_image_topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "aruco_pose", 10)
        self.id_pub = self.create_publisher(Int32, "aruco_id", 10)
        self.distance_pub = self.create_publisher(Float32, "aruco_distance", 10)
        self.bridge = CvBridge()
        output_image_topic = str(self.get_parameter("output_image_topic").value)
        self.image_pub = self.create_publisher(Image, output_image_topic, 10)

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.get_logger().info(
            f"Subscribed image_topic={image_topic}, camera_info_topic={camera_info_topic}"
        )

        # ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Support both old (OpenCV < 4.7) and new (OpenCV >= 4.7) detector API
        try:
            self.parameters = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
            self._use_new_api = True
        except AttributeError:
            self.parameters = aruco.DetectorParameters_create()  # type: ignore[attr-defined]
            self._use_new_api = False

        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_NONE

        # Camera params are taken from CameraInfo topic
        self.camera_matrix = None
        self.dist_coeffs = None
        self._camera_ready = False
        self._warned_camera_info = False
        self.marker_length = 0.05  # マーカーの一辺の長さ [m]

        # Marker object points for solvePnP (used when estimatePoseSingleMarkers unavailable)
        half = self.marker_length / 2.0
        self._obj_pts = np.array(
            [
                [-half, half, 0],
                [half, half, 0],
                [half, -half, 0],
                [-half, -half, 0],
            ],
            dtype=np.float32,
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float32).reshape(-1, 1)
        if not self._camera_ready:
            self._camera_ready = True
            self.get_logger().info("CameraInfo received. Pose estimation is enabled.")

    def _detect_markers(self, gray):
        if self._use_new_api:
            return self.detector.detectMarkers(gray)
        return aruco.detectMarkers(  # type: ignore[attr-defined]
            gray, self.aruco_dict, parameters=self.parameters
        )

    def _estimate_pose(self, corners):
        """ポーズ推定 – estimatePoseSingleMarkers が利用できない場合は solvePnP を使用."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            raise RuntimeError("Camera intrinsics are not initialized")
        try:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(  # type: ignore[attr-defined]
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            return rvecs[0][0], tvecs[0][0]
        except AttributeError:
            _, r, t = cv2.solvePnP(
                self._obj_pts,
                corners[0][0],
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            return r.flatten(), t.flatten()

    @staticmethod
    def _rotation_to_quaternion(R):
        """回転行列をクォータニオン (x, y, z, w) に変換する (Shepperd法)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return float(qx), float(qy), float(qz), float(qw)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"画像変換失敗: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header = msg.header
        self.image_pub.publish(img_msg)

        if not self._camera_ready:
            if not self._warned_camera_info:
                self.get_logger().warn("CameraInfo未受信のため姿勢推定を保留中")
                self._warned_camera_info = True
            return

        if ids is not None:
            r, t = self._estimate_pose(corners)

            # 1つ目のマーカーだけ publish（複数対応も可能）
            marker_id = int(ids[0][0])

            # カメラからマーカーまでのユークリッド距離 [m]
            distance = float(np.linalg.norm(t))

            # PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            frame_id = str(self.get_parameter("frame_id").value)
            pose_msg.header.frame_id = msg.header.frame_id if not frame_id else frame_id

            pose_msg.pose.position.x = float(t[0])
            pose_msg.pose.position.y = float(t[1])
            pose_msg.pose.position.z = float(t[2])

            # rvec → quaternion（OpenCV の Rodrigues → Shepperd法）
            R, _ = cv2.Rodrigues(r)
            qx, qy, qz, qw = self._rotation_to_quaternion(R)

            pose_msg.pose.orientation.x = float(qx)
            pose_msg.pose.orientation.y = float(qy)
            pose_msg.pose.orientation.z = float(qz)
            pose_msg.pose.orientation.w = float(qw)

            # Publish
            self.pose_pub.publish(pose_msg)

            id_msg = Int32()
            id_msg.data = marker_id
            self.id_pub.publish(id_msg)

            dist_msg = Float32()
            dist_msg.data = distance
            self.distance_pub.publish(dist_msg)

            self.get_logger().info(
                f"ID:{marker_id} "
                f"Pos=({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}) "
                f"Dist={distance:.2f}m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
