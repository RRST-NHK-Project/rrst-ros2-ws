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

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


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

        # HSVカラー範囲（デフォルト: 赤い平面マーカー）
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
        self.declare_parameter('max_aspect_ratio', 5.0)

        # RealSense 深度スケール（uint16 値→メートル変換）
        self.declare_parameter('depth_scale', 0.001)

        # YOLO 併用設定
        self.declare_parameter('use_yolo', False)
        self.declare_parameter('yolo_model_path', 'yolov8n.pt')
        self.declare_parameter('yolo_conf', 0.25)
        self.declare_parameter('yolo_iou', 0.45)
        self.declare_parameter('yolo_imgsz', 640)
        self.declare_parameter('yolo_infer_interval', 3)
        self.declare_parameter('yolo_min_iou_with_contour', 0.10)

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

        # ---- YOLO 初期化 ----
        self._use_yolo = bool(self.get_parameter('use_yolo').value)
        self._yolo_model = None
        self._cached_yolo_boxes: list[tuple[int, int, int, int]] = []
        if self._use_yolo:
            if YOLO is None:
                self.get_logger().error(
                    'use_yolo=True ですが ultralytics が未インストールです。'
                )
                self._use_yolo = False
            else:
                model_path = str(self.get_parameter('yolo_model_path').value)
                try:
                    self._yolo_model = YOLO(model_path)
                    self.get_logger().info(f'YOLO有効: model={model_path}')
                except Exception as exc:
                    self.get_logger().error(f'YOLOモデル読み込み失敗: {exc}')
                    self._use_yolo = False

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
        yolo_boxes = self._infer_yolo_boxes(frame)
        best_cnt = self._find_best_contour(mask, yolo_boxes)

        vis = frame.copy()
        for x1, y1, x2, y2 in yolo_boxes:
            cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 128, 0), 2)
            cv2.putText(
                vis,
                'YOLO',
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 128, 0),
                2,
                cv2.LINE_AA,
            )

        if best_cnt is not None:
            self._process_detection(best_cnt, vis, depth, img_msg)
        elif self._frame_count % 100 == 0:
            self.get_logger().info('平面未検知')

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

    def _find_best_contour(
        self,
        mask: np.ndarray,
        yolo_boxes: list[tuple[int, int, int, int]] | None = None,
    ):
        """最大の平面候補輪郭を返す（見つからなければ None）."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = int(self.get_parameter('min_contour_area').value)
        max_area = int(self.get_parameter('max_contour_area').value)
        min_iou = float(self.get_parameter('yolo_min_iou_with_contour').value)
        yolo_boxes = yolo_boxes or []

        best_cnt = None
        best_score = -1.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue
            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            if w == 0 or h == 0:
                continue
            aspect = max(w, h) / min(w, h)
            if aspect > float(self.get_parameter('max_aspect_ratio').value):
                continue

            iou_score = 0.0
            if self._use_yolo and yolo_boxes:
                iou_score = max(self._contour_box_iou(cnt, box) for box in yolo_boxes)
                if iou_score < min_iou:
                    continue

            score = area + 50000.0 * iou_score
            if score > best_score:
                best_score = score
                best_cnt = cnt
        return best_cnt

    def _infer_yolo_boxes(self, frame: np.ndarray) -> list[tuple[int, int, int, int]]:
        """YOLO推論で検出ボックスを返す。無効時は空配列。"""
        if not self._use_yolo or self._yolo_model is None:
            return []

        interval = max(1, int(self.get_parameter('yolo_infer_interval').value))
        if self._frame_count % interval != 0:
            return self._cached_yolo_boxes

        try:
            results = self._yolo_model.predict(
                source=frame,
                conf=float(self.get_parameter('yolo_conf').value),
                iou=float(self.get_parameter('yolo_iou').value),
                imgsz=int(self.get_parameter('yolo_imgsz').value),
                verbose=False,
            )
        except Exception as exc:
            if self._frame_count % 100 == 0:
                self.get_logger().warn(f'YOLO推論失敗: {exc}')
            return self._cached_yolo_boxes

        boxes: list[tuple[int, int, int, int]] = []
        if results:
            result = results[0]
            if result.boxes is not None and result.boxes.xyxy is not None:
                for box in result.boxes.xyxy.cpu().numpy().astype(int):
                    x1, y1, x2, y2 = box[:4]
                    boxes.append((x1, y1, x2, y2))

        self._cached_yolo_boxes = boxes
        return boxes

    @staticmethod
    def _contour_box_iou(cnt: np.ndarray, box: tuple[int, int, int, int]) -> float:
        """輪郭外接矩形とYOLO矩形のIoUを計算する."""
        x, y, w, h = cv2.boundingRect(cnt)
        ax1, ay1, ax2, ay2 = x, y, x + w, y + h
        bx1, by1, bx2, by2 = box

        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        inter = float(iw * ih)
        if inter <= 0:
            return 0.0

        area_a = float(max(0, ax2 - ax1) * max(0, ay2 - ay1))
        area_b = float(max(0, bx2 - bx1) * max(0, by2 - by1))
        union = area_a + area_b - inter
        if union <= 0:
            return 0.0
        return inter / union

    def _process_detection(
        self,
        cnt,
        vis: np.ndarray,
        depth: np.ndarray,
        img_msg: Image,
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

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f'平面検知: Pos=({X:.3f}, {Y:.3f}, {Z:.3f}) '
                f'Dist={dist_m:.3f}m Ang={angle_deg:.1f}deg Area={int(area)}px'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PlaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
