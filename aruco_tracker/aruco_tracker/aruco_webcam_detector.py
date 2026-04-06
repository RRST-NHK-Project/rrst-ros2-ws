import cv2
import numpy as np
import os
import platform
from cv2 import aruco
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
import yaml


class ArucoWebcamDetector(Node):
    """Webカメラ映像からArUcoを検出して3D姿勢を配信するノード."""

    def __init__(self):
        super().__init__("aruco_webcam_detector")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("output_image_topic", "/camera/image_raw")
        self.declare_parameter("id_topic", "/aruco_id")
        self.declare_parameter("pose_topic", "/aruco_pose")
        self.declare_parameter("distance_topic", "/aruco_distance")
        self.declare_parameter("frame_id", "webcam_frame")
        self.declare_parameter("marker_length", 0.05)
        self.declare_parameter("camera_fourcc", "YUYV")
        self.declare_parameter("camera_info_yaml", "")

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.output_image_topic = str(self.get_parameter("output_image_topic").value)
        self.id_topic = str(self.get_parameter("id_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.distance_topic = str(self.get_parameter("distance_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.marker_length = float(self.get_parameter("marker_length").value)
        self.camera_fourcc = str(self.get_parameter("camera_fourcc").value)
        self.camera_info_yaml = str(self.get_parameter("camera_info_yaml").value)

        self.declare_parameter("fx", 600.0)
        self.declare_parameter("fy", 600.0)
        self.declare_parameter("cx", float(self.camera_width) / 2.0)
        self.declare_parameter("cy", float(self.camera_height) / 2.0)
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])

        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)
        dist_coeffs_param = self.get_parameter("dist_coeffs").value

        self.camera_matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        self.dist_coeffs = np.array(dist_coeffs_param, dtype=np.float32).reshape(-1, 1)

        self._load_intrinsics_from_yaml_if_available()

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)
        self.id_pub = self.create_publisher(Int32, self.id_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.distance_pub = self.create_publisher(Float32, self.distance_topic, 10)

        self.cap = self._open_camera()
        if self.camera_width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        if self.camera_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        if self.camera_fourcc and len(self.camera_fourcc) == 4:
            self.cap.set(
                cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.camera_fourcc)
            )
        self.cap.set(cv2.CAP_PROP_FPS, self.publish_rate)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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
            "Webcam detector started: "
            f"camera_index={self.camera_index}, image_topic={self.output_image_topic}, "
            f"pose_topic={self.pose_topic}"
        )

    def _open_camera(self):
        if platform.system().lower() == "linux":
            cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
            if cap.isOpened():
                return cap

        return cv2.VideoCapture(self.camera_index)

    def _load_intrinsics_from_yaml_if_available(self):
        if not self.camera_info_yaml:
            self.get_logger().info(
                "camera_info_yaml not set. Using fx/fy/cx/cy parameters."
            )
            return

        yaml_path = os.path.abspath(self.camera_info_yaml)
        if not os.path.exists(yaml_path):
            self.get_logger().warning(
                f"camera_info_yaml not found: {yaml_path}. Using fx/fy/cx/cy parameters."
            )
            return

        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)

            camera_matrix_data = data["camera_matrix"]["data"]
            dist_data = data["distortion_coefficients"]["data"]

            self.camera_matrix = np.array(camera_matrix_data, dtype=np.float32).reshape(
                3, 3
            )
            self.dist_coeffs = np.array(dist_data, dtype=np.float32).reshape(-1, 1)
            self.get_logger().info(f"Loaded camera intrinsics from YAML: {yaml_path}")
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to load camera_info_yaml ({yaml_path}): {exc}. "
                "Using fx/fy/cx/cy parameters."
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

            rvec, tvec = self._estimate_pose(corners[0])
            self._publish_pose_and_distance(tvec, rvec)

            try:
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_length * 0.5,
                )
            except Exception:
                pass

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.frame_id
        self.image_pub.publish(image_msg)

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        return super().destroy_node()

    def _estimate_pose(self, marker_corners):
        """1つのマーカーcornersから回転・並進ベクトルを推定する."""
        try:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(  # type: ignore[attr-defined]
                [marker_corners],
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs,
            )
            return rvecs[0][0], tvecs[0][0]
        except AttributeError:
            half = self.marker_length / 2.0
            object_points = np.array(
                [
                    [-half, half, 0.0],
                    [half, half, 0.0],
                    [half, -half, 0.0],
                    [-half, -half, 0.0],
                ],
                dtype=np.float32,
            )
            image_points = marker_corners.reshape(4, 2).astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                raise RuntimeError("solvePnP failed")
            return rvec.flatten(), tvec.flatten()

    def _publish_pose_and_distance(self, tvec, rvec):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = self._rotation_to_quaternion(rotation_matrix)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        distance_msg = Float32()
        distance_msg.data = float(np.linalg.norm(tvec))
        self.distance_pub.publish(distance_msg)

    @staticmethod
    def _rotation_to_quaternion(rotation_matrix):
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
        elif (
            rotation_matrix[0, 0] > rotation_matrix[1, 1]
            and rotation_matrix[0, 0] > rotation_matrix[2, 2]
        ):
            s = 2.0 * np.sqrt(
                1.0
                + rotation_matrix[0, 0]
                - rotation_matrix[1, 1]
                - rotation_matrix[2, 2]
            )
            qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
            s = 2.0 * np.sqrt(
                1.0
                + rotation_matrix[1, 1]
                - rotation_matrix[0, 0]
                - rotation_matrix[2, 2]
            )
            qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(
                1.0
                + rotation_matrix[2, 2]
                - rotation_matrix[0, 0]
                - rotation_matrix[1, 1]
            )
            qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            qz = 0.25 * s

        return float(qx), float(qy), float(qz), float(qw)


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
