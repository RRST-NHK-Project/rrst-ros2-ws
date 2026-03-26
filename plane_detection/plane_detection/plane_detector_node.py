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


class PlaneDetectorNode(Node):
    """深度カメラを使って平面領域を検知し位置・姿勢・距離をPublishするノード."""

    def __init__(self):
        super().__init__('plane_detector')

        # ---- パラメータ宣言 ----
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_image_topic', '/plane_detection/image')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        # RealSense 深度スケール（uint16 値→メートル変換）
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('depth_min_m', 0.25)
        self.declare_parameter('depth_max_m', 3.50)
        self.declare_parameter('plane_ransac_iterations', 120)
        self.declare_parameter('plane_distance_threshold_m', 0.02)
        self.declare_parameter('plane_sample_stride', 4)
        self.declare_parameter('min_plane_inliers', 1200)
        self.declare_parameter('min_plane_contour_area', 2000)

        # ---- QoS ----
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ---- Publisher ----
        self._pose_pub = self.create_publisher(PoseStamped, 'plane_detection/pose', qos)
        self._dist_pub = self.create_publisher(Float32, 'plane_detection/distance', qos)
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
            f'PlaneDetector起動: image={image_topic}, depth={depth_topic}'
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

        vis = frame.copy()
        best_cnt, plane_mask, normal, inlier_count = self._detect_plane_from_depth(depth)

        if plane_mask is not None:
            # 平面マスク領域を半透明で強調
            plane_overlay = np.zeros_like(vis)
            plane_overlay[:, :, 1] = 220
            plane_overlay[:, :, 2] = 220
            vis = np.where(plane_mask[:, :, None], (0.65 * vis + 0.35 * plane_overlay).astype(np.uint8), vis)

        if best_cnt is not None:
            self._process_detection(best_cnt, vis, depth, img_msg, normal, inlier_count)
        elif self._frame_count % 100 == 0:
            self.get_logger().info('平面未検知')

        out_msg = self._bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        out_msg.header = img_msg.header
        self._image_pub.publish(out_msg)

    # ------------------------------------------------------------------
    # 内部処理
    # ------------------------------------------------------------------

    def _pixel_to_camera(self, u: np.ndarray, v: np.ndarray, z_m: np.ndarray):
        """画像座標と深度[m]をカメラ座標系3D点へ変換する."""
        fx = self._camera_matrix[0, 0]
        fy = self._camera_matrix[1, 1]
        ppx = self._camera_matrix[0, 2]
        ppy = self._camera_matrix[1, 2]
        x = (u - ppx) * z_m / fx
        y = (v - ppy) * z_m / fy
        return np.stack([x, y, z_m], axis=1)

    def _fit_plane_ransac(self, points: np.ndarray):
        """RANSACで平面 ax+by+cz+d=0 を推定する. Returns (normal, d, inliers)."""
        if points.shape[0] < 3:
            return None, None, None

        iters = int(self.get_parameter('plane_ransac_iterations').value)
        th = float(self.get_parameter('plane_distance_threshold_m').value)
        best_inliers = None
        best_count = 0
        best_n = None
        best_d = None

        n_points = points.shape[0]
        for _ in range(iters):
            ids = np.random.choice(n_points, 3, replace=False)
            p1, p2, p3 = points[ids]
            n = np.cross(p2 - p1, p3 - p1)
            norm = np.linalg.norm(n)
            if norm < 1e-6:
                continue
            n = n / norm
            d = -np.dot(n, p1)

            dist = np.abs(points @ n + d)
            inliers = dist < th
            cnt = int(np.count_nonzero(inliers))
            if cnt > best_count:
                best_count = cnt
                best_inliers = inliers
                best_n = n
                best_d = d

        if best_inliers is None:
            return None, None, None

        # inlier点で再フィット（SVD）
        inlier_pts = points[best_inliers]
        centroid = np.mean(inlier_pts, axis=0)
        _, _, vh = np.linalg.svd(inlier_pts - centroid, full_matrices=False)
        n = vh[-1]
        n = n / (np.linalg.norm(n) + 1e-12)
        d = -np.dot(n, centroid)

        # 法線方向をカメラ前方(+Z)へ揃える
        if n[2] < 0:
            n = -n
            d = -d

        dist = np.abs(points @ n + d)
        inliers = dist < th
        return n, d, inliers

    def _detect_plane_from_depth(self, depth: np.ndarray):
        """深度画像から平面輪郭を検出する."""
        if not self._camera_ready:
            return None, None, None, 0

        depth_scale = float(self.get_parameter('depth_scale').value)
        d_min = float(self.get_parameter('depth_min_m').value)
        d_max = float(self.get_parameter('depth_max_m').value)
        stride = max(1, int(self.get_parameter('plane_sample_stride').value))

        depth_m = depth.astype(np.float32) * depth_scale
        valid = (depth_m > d_min) & (depth_m < d_max) & np.isfinite(depth_m)

        ys, xs = np.where(valid)
        if xs.size < 1000:
            return None, None, None, 0

        # サンプリングしてRANSAC負荷を抑える
        keep = ((xs % stride) == 0) & ((ys % stride) == 0)
        xs_s = xs[keep]
        ys_s = ys[keep]
        z_s = depth_m[ys_s, xs_s]
        if xs_s.size < 400:
            return None, None, None, 0

        pts = self._pixel_to_camera(xs_s.astype(np.float64), ys_s.astype(np.float64), z_s.astype(np.float64))
        n, d, inliers = self._fit_plane_ransac(pts)
        if n is None or inliers is None:
            return None, None, None, 0

        inlier_count = int(np.count_nonzero(inliers))
        if inlier_count < int(self.get_parameter('min_plane_inliers').value):
            return None, None, None, inlier_count

        # フル解像度で平面inlierマスクを作る
        pts_all = self._pixel_to_camera(xs.astype(np.float64), ys.astype(np.float64), depth_m[ys, xs].astype(np.float64))
        dist_all = np.abs(pts_all @ n + d)
        th = float(self.get_parameter('plane_distance_threshold_m').value)
        plane_inlier = dist_all < th

        plane_mask = np.zeros(depth_m.shape, dtype=np.uint8)
        plane_mask[ys[plane_inlier], xs[plane_inlier]] = 255

        # 穴埋め・ノイズ除去
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        plane_mask = cv2.morphologyEx(plane_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        plane_mask = cv2.morphologyEx(plane_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(plane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, plane_mask.astype(bool), n, inlier_count

        best_cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(best_cnt) < float(self.get_parameter('min_plane_contour_area').value):
            return None, plane_mask.astype(bool), n, inlier_count

        return best_cnt, plane_mask.astype(bool), n, inlier_count

    def _process_detection(
        self,
        cnt,
        vis: np.ndarray,
        depth: np.ndarray,
        img_msg: Image,
        normal: np.ndarray,
        inlier_count: int,
    ):
        """検知した平面輪郭から3D位置・姿勢を推定し Publish する."""
        rect = cv2.minAreaRect(cnt)
        box = np.int32(cv2.boxPoints(rect))
        cx, cy = int(rect[0][0]), int(rect[0][1])
        angle_deg = float(rect[2])
        area = float(cv2.contourArea(cnt))

        # ---- 可視化: 平面を分かりやすく塗りつぶし＋輪郭 ----
        overlay = vis.copy()
        cv2.fillPoly(overlay, [box], (0, 180, 255))
        cv2.addWeighted(overlay, 0.28, vis, 0.72, 0.0, vis)
        cv2.drawContours(vis, [box], 0, (0, 255, 255), 3)
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

        # 面の向き（画像平面内）を矢印で表示
        yaw = float(np.deg2rad(angle_deg))
        dx = int(60 * np.cos(yaw))
        dy = int(60 * np.sin(yaw))
        cv2.arrowedLine(vis, (cx, cy), (cx + dx, cy + dy), (255, 255, 255), 2, tipLength=0.25)

        # 推定法線の投影方向も描画
        nx, ny, nz = float(normal[0]), float(normal[1]), float(normal[2])
        ndx = int(80 * nx)
        ndy = int(80 * ny)
        cv2.arrowedLine(vis, (cx, cy), (cx + ndx, cy + ndy), (255, 0, 255), 2, tipLength=0.20)

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

        # カメラ内部パラメータが未取得の場合は位置推定を保留
        if not self._camera_ready:
            if not self._warned_no_camera_info:
                self.get_logger().warn('CameraInfo 未受信のため位置推定を保留中')
                self._warned_no_camera_info = True
            return

        # 中心画素の深度から逆投影して平面中心位置を推定
        fx = self._camera_matrix[0, 0]
        fy = self._camera_matrix[1, 1]
        ppx = self._camera_matrix[0, 2]
        ppy = self._camera_matrix[1, 2]
        X = (cx - ppx) * dist_m / fx
        Y = (cy - ppy) * dist_m / fy
        Z = dist_m

        # 姿勢は画像平面上の角度を Z 軸回転として近似
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
            'PLANE DETECTED',
            (cx + 12, cy - 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            vis,
            f'Center XYZ=({X:.3f}, {Y:.3f}, {Z:.3f}) m',
            (cx + 12, cy - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            vis,
            f'Dist:{dist_m:.3f}m  Angle:{angle_deg:.1f}deg  Area:{int(area)}px',
            (cx + 12, cy + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            vis,
            f'Normal=({nx:.2f}, {ny:.2f}, {nz:.2f})  Inliers:{inlier_count}',
            (cx + 12, cy + 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f'平面検知: Pos=({X:.3f}, {Y:.3f}, {Z:.3f}) '
                f'Dist={dist_m:.3f}m Ang={angle_deg:.1f}deg Area={int(area)}px '
                f'Normal=({nx:.2f},{ny:.2f},{nz:.2f}) Inliers={inlier_count}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PlaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
