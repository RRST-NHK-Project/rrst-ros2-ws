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
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32


class CubeDetectorNode(Node):
    """深度カメラを使って立方体を検知し位置・姿勢・距離をPublishするノード.

    RealSense 等の深度カメラからRGB画像と深度画像を受け取り、
    HSVカラーセグメンテーションと輪郭検出で立方体の正面を検知する。
    検知した立方体の3D位置・姿勢（クォータニオン）・距離をPublishし、
    可視化画像も出力する。
    """

    def __init__(self):
        super().__init__('cube_detector')

        # ---- パラメータ宣言 ----
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_image_topic', '/cube_detection/image')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        # 立方体の一辺の長さ [m]（solvePnP 用）
        self.declare_parameter('cube_size', 0.065)

        # HSVカラー範囲（デフォルト: 赤立方体）
        # 赤はH=0付近と180付近の2範囲を OR 結合する
        self.declare_parameter('hsv_h_low1', 0)
        self.declare_parameter('hsv_s_low1', 100)
        self.declare_parameter('hsv_v_low1', 80)
        self.declare_parameter('hsv_h_high1', 10)
        self.declare_parameter('hsv_s_high1', 255)
        self.declare_parameter('hsv_v_high1', 255)
        self.declare_parameter('hsv_h_low2', 160)
        self.declare_parameter('hsv_s_low2', 100)
        self.declare_parameter('hsv_v_low2', 80)
        self.declare_parameter('hsv_h_high2', 180)
        self.declare_parameter('hsv_s_high2', 255)
        self.declare_parameter('hsv_v_high2', 255)

        # 輪郭面積フィルタ [px^2]
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('max_contour_area', 100000)

        # RealSense 深度スケール（uint16 値→メートル変換）
        self.declare_parameter('depth_scale', 0.001)

        # ---- QoS ----
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ---- Publisher ----
        self._pose_pub = self.create_publisher(PoseStamped, 'cube_detection/pose', qos)
        self._dist_pub = self.create_publisher(Float32, 'cube_detection/distance', qos)
        output_image_topic = str(self.get_parameter('output_image_topic').value)
        self._image_pub = self.create_publisher(Image, output_image_topic, qos)

        # ---- カメラ内部パラメータ ----
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None
        self._camera_ready = False
        self._bridge = CvBridge()

        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 1)

        # ---- 同期サブスクリプション (RGB + Depth) ----
        image_topic = str(self.get_parameter('image_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)

        img_sub = message_filters.Subscriber(self, Image, image_topic, qos_profile=qos)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, depth_sub], queue_size=5, slop=0.05
        )
        self._sync.registerCallback(self._image_callback)

        self.get_logger().info(
            f'CubeDetector起動: image={image_topic}, depth={depth_topic}'
        )
        self._frame_count = 0
        self._warned_no_camera_info = False

    # ------------------------------------------------------------------
    # コールバック
    # ------------------------------------------------------------------

    def _camera_info_cb(self, msg: CameraInfo):
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)
        if not self._camera_ready:
            self._camera_ready = True
            self.get_logger().info('CameraInfo 受信: 3D位置推定を有効化しました。')

    def _image_callback(self, img_msg: Image, depth_msg: Image):
        self._frame_count += 1

        try:
            frame = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'画像変換失敗: {exc}')
            return

        mask = self._build_color_mask(frame)
        best_cnt = self._find_best_contour(mask)

        vis = frame.copy()
        if best_cnt is not None:
            self._process_detection(best_cnt, vis, depth, img_msg)
        elif self._frame_count % 100 == 0:
            self.get_logger().info('立方体未検知')

        out_msg = self._bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        out_msg.header = img_msg.header
        self._image_pub.publish(out_msg)

    # ------------------------------------------------------------------
    # 内部処理
    # ------------------------------------------------------------------

    def _build_color_mask(self, frame: np.ndarray) -> np.ndarray:
        """HSVカラーセグメンテーションマスクを生成する."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        def p(name: str) -> int:
            return int(self.get_parameter(name).value)

        lower1 = np.array([p('hsv_h_low1'), p('hsv_s_low1'), p('hsv_v_low1')])
        upper1 = np.array([p('hsv_h_high1'), p('hsv_s_high1'), p('hsv_v_high1')])
        lower2 = np.array([p('hsv_h_low2'), p('hsv_s_low2'), p('hsv_v_low2')])
        upper2 = np.array([p('hsv_h_high2'), p('hsv_s_high2'), p('hsv_v_high2')])

        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower1, upper1),
            cv2.inRange(hsv, lower2, upper2),
        )

        # モルフォロジー処理でノイズ除去
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def _find_best_contour(self, mask: np.ndarray):
        """最大の正方形状輪郭を返す（見つからなければ None）."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = int(self.get_parameter('min_contour_area').value)
        max_area = int(self.get_parameter('max_contour_area').value)

        best_cnt = None
        best_area = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue
            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            if w == 0 or h == 0:
                continue
            aspect = max(w, h) / min(w, h)
            if aspect > 2.5:
                continue
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt

    @staticmethod
    def _rotation_matrix_to_quaternion(R: np.ndarray):
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

    def _estimate_pose_pnp(self, rect, cube_size: float):
        """solvePnP で立方体正面の 6DOF 姿勢を推定する.

        Returns:
            rvec, tvec (np.ndarray) または (None, None)
        """
        if not self._camera_ready:
            return None, None

        box = cv2.boxPoints(rect).astype(np.float32)

        # 立方体正面の物体点（Z=0平面）
        half = cube_size / 2.0
        obj_pts = np.array(
            [
                [-half, -half, 0.0],
                [half, -half, 0.0],
                [half, half, 0.0],
                [-half, half, 0.0],
            ],
            dtype=np.float32,
        )

        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts,
                box,
                self._camera_matrix,
                self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except cv2.error:
            return None, None

        if not ok:
            return None, None
        return rvec.flatten(), tvec.flatten()

    def _process_detection(
        self,
        cnt,
        vis: np.ndarray,
        depth: np.ndarray,
        img_msg: Image,
    ):
        """検知した輪郭から3D位置・姿勢を推定し Publish する."""
        rect = cv2.minAreaRect(cnt)
        box = np.int32(cv2.boxPoints(rect))
        cx, cy = int(rect[0][0]), int(rect[0][1])
        angle_deg = float(rect[2])

        # ---- 可視化: バウンディングボックスと中心点 ----
        cv2.drawContours(vis, [box], 0, (0, 255, 0), 2)
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

        # ---- 深度取得 (中心付近の中央値) ----
        h_img, w_img = depth.shape[:2]
        r = 10
        x1, x2 = max(0, cx - r), min(w_img, cx + r)
        y1, y2 = max(0, cy - r), min(h_img, cy + r)
        roi = depth[y1:y2, x1:x2].astype(np.float32)
        valid = roi[roi > 0]
        depth_scale = float(self.get_parameter('depth_scale').value)

        if valid.size == 0:
            self.get_logger().debug('深度値が無効（ROI内に有効値なし）')
            return

        dist_m = float(np.median(valid)) * depth_scale

        # ---- solvePnP で 6DOF 姿勢推定 ----
        cube_size = float(self.get_parameter('cube_size').value)
        rvec, tvec = self._estimate_pose_pnp(rect, cube_size)

        if rvec is not None and tvec is not None:
            # solvePnP の結果を使用
            X, Y, Z = float(tvec[0]), float(tvec[1]), float(tvec[2])
            R, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R)
        else:
            # カメラ内部パラメータが未取得の場合は逆投影で位置のみ推定
            if not self._camera_ready:
                if not self._warned_no_camera_info:
                    self.get_logger().warn('CameraInfo 未受信のため位置推定を保留中')
                    self._warned_no_camera_info = True
                return
            fx = self._camera_matrix[0, 0]
            fy = self._camera_matrix[1, 1]
            ppx = self._camera_matrix[0, 2]
            ppy = self._camera_matrix[1, 2]
            X = (cx - ppx) * dist_m / fx
            Y = (cy - ppy) * dist_m / fy
            Z = dist_m
            # 姿勢は minAreaRect の角度から Z 軸回転で近似
            yaw = float(np.deg2rad(angle_deg))
            qz = float(np.sin(yaw / 2.0))
            qw = float(np.cos(yaw / 2.0))
            qx, qy = 0.0, 0.0

        # ---- PoseStamped Publish ----
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = str(self.get_parameter('frame_id').value)
        pose_msg.pose.position.x = X
        pose_msg.pose.position.y = Y
        pose_msg.pose.position.z = Z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self._pose_pub.publish(pose_msg)

        # ---- 距離 Publish ----
        dist_msg = Float32()
        dist_msg.data = dist_m
        self._dist_pub.publish(dist_msg)

        # ---- 可視化テキスト ----
        cv2.putText(
            vis,
            f'X:{X:.3f}m  Y:{Y:.3f}m  Z:{Z:.3f}m',
            (cx + 12, cy - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA,
        )
        cv2.putText(
            vis,
            f'Dist:{dist_m:.3f}m  Ang:{angle_deg:.1f}deg',
            (cx + 12, cy + 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA,
        )

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f'立方体検知: Pos=({X:.3f}, {Y:.3f}, {Z:.3f}) '
                f'Dist={dist_m:.3f}m Ang={angle_deg:.1f}deg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
