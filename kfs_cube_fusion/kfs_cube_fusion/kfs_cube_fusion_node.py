from __future__ import annotations

import math
import os
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
import message_filters
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray


@dataclass
class KfsDetection:
    detected: bool
    center_x: float
    center_y: float
    depth_mm: float
    match_count: int
    cube_center_x: float
    cube_center_y: float


class KfsCubeFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("kfs_cube_fusion_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("sync_slop_sec", 0.08)
        self.declare_parameter("template_package", "kfs_pkg")
        self.declare_parameter("template_image_name", "KFS_image_list.png")
        self.declare_parameter("query_scale", 0.5)
        self.declare_parameter("match_ratio", 0.87)
        self.declare_parameter("minimum_good_matches", 12)
        self.declare_parameter("max_matches_to_use", 80)
        self.declare_parameter("central_window_px", 40)
        self.declare_parameter("depth_band_mm", 180.0)
        self.declare_parameter("min_area_px", 350)
        self.declare_parameter("max_center_offset_px", 180)
        self.declare_parameter("morph_kernel_px", 5)
        self.declare_parameter("depth_sample_window_px", 7)
        self.declare_parameter("output_topic", "/kfs_cube_fusion/result")
        self.declare_parameter("detected_topic", "/kfs_cube_fusion/detected")
        self.declare_parameter("debug_image_topic", "/kfs_cube_fusion/debug_image")

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.sync_slop_sec = float(self.get_parameter("sync_slop_sec").value)
        self.template_package = str(self.get_parameter("template_package").value)
        self.template_image_name = str(self.get_parameter("template_image_name").value)
        self.query_scale = max(0.1, float(self.get_parameter("query_scale").value))
        self.match_ratio = float(self.get_parameter("match_ratio").value)
        self.minimum_good_matches = int(
            self.get_parameter("minimum_good_matches").value
        )
        self.max_matches_to_use = int(self.get_parameter("max_matches_to_use").value)
        self.central_window_px = int(self.get_parameter("central_window_px").value)
        self.depth_band_mm = float(self.get_parameter("depth_band_mm").value)
        self.min_area_px = int(self.get_parameter("min_area_px").value)
        self.max_center_offset_px = int(
            self.get_parameter("max_center_offset_px").value
        )
        self.morph_kernel_px = max(1, int(self.get_parameter("morph_kernel_px").value))
        self.depth_sample_window_px = max(
            1, int(self.get_parameter("depth_sample_window_px").value)
        )
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.detected_topic = str(self.get_parameter("detected_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        self.bridge = CvBridge()
        self.akaze = cv2.AKAZE_create()
        self.bf = cv2.BFMatcher()

        self.template_gray = self._load_template_image()
        self.template_kp = []
        self.template_des = None
        self.template_h = 0
        self.template_w = 0
        if self.template_gray is not None:
            self.template_h, self.template_w = self.template_gray.shape[:2]
            self.template_kp, self.template_des = self.akaze.detectAndCompute(
                self.template_gray, None
            )

        self.result_pub = self.create_publisher(
            Float32MultiArray, self.output_topic, 10
        )
        self.detected_pub = self.create_publisher(Bool, self.detected_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=self.sync_slop_sec
        )
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info(
            "kfs_cube_fusion started: color=%s depth=%s template=%s/%s"
            % (
                self.color_topic,
                self.depth_topic,
                self.template_package,
                self.template_image_name,
            )
        )

        if self.template_gray is None or self.template_des is None:
            self.get_logger().error("テンプレート画像の読み込みに失敗しました。")
        elif len(self.template_kp) == 0:
            self.get_logger().error(
                "テンプレート画像から特徴点を抽出できませんでした。"
            )
        else:
            self.get_logger().info(
                f"template loaded: {self.template_w}x{self.template_h}, kp={len(self.template_kp)}"
            )

    def _load_template_image(self) -> np.ndarray | None:
        candidates = []
        try:
            template_share = get_package_share_directory(self.template_package)
            candidates.append(
                os.path.join(template_share, "resource", self.template_image_name)
            )
        except Exception as exc:
            self.get_logger().warn(f"template package lookup failed: {exc}")

        try:
            self_share = get_package_share_directory("kfs_cube_fusion")
            candidates.append(
                os.path.join(self_share, "resource", self.template_image_name)
            )
        except Exception:
            pass

        for candidate in candidates:
            if os.path.exists(candidate):
                image = cv2.imread(candidate, cv2.IMREAD_GRAYSCALE)
                if image is not None:
                    return image
        return None

    def synced_callback(self, color_msg: Image, depth_msg: Image) -> None:
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return

        depth_mm = self._depth_to_mm(depth_image)
        cube_roi = self._detect_cube_roi(depth_mm)

        search_image = color_image
        roi_x = 0
        roi_y = 0
        roi_w = color_image.shape[1]
        roi_h = color_image.shape[0]
        cube_center_x = roi_w * 0.5
        cube_center_y = roi_h * 0.5

        if cube_roi is not None:
            roi_x, roi_y, roi_w, roi_h = cube_roi
            cube_center_x = roi_x + roi_w * 0.5
            cube_center_y = roi_y + roi_h * 0.5
            search_image = color_image[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w]

        detection = self._detect_kfs(
            search_image, roi_x, roi_y, color_image.shape[1], color_image.shape[0]
        )
        if detection is None:
            self._publish_result(
                KfsDetection(False, 0.0, 0.0, 0.0, 0, cube_center_x, cube_center_y),
                color_image,
                cube_roi,
            )
            return

        sampled_depth = self._sample_depth_mm(
            depth_mm, detection.center_x, detection.center_y
        )
        if sampled_depth is None:
            self._publish_result(
                KfsDetection(False, 0.0, 0.0, 0.0, 0, cube_center_x, cube_center_y),
                color_image,
                cube_roi,
            )
            return

        detection.depth_mm = sampled_depth
        self._publish_result(detection, color_image, cube_roi)

    @staticmethod
    def _depth_to_mm(depth_image: np.ndarray) -> np.ndarray:
        depth = depth_image.astype(np.float32)
        valid = depth[np.isfinite(depth) & (depth > 0)]
        if valid.size == 0:
            return depth
        if float(np.max(valid)) <= 20.0:
            depth *= 1000.0
        return depth

    def _detect_cube_roi(
        self, depth_mm: np.ndarray
    ) -> tuple[int, int, int, int] | None:
        if depth_mm.ndim != 2:
            return None

        height, width = depth_mm.shape[:2]
        center_x = width // 2
        center_y = height // 2
        half_window = max(1, self.central_window_px // 2)

        y0 = max(0, center_y - half_window)
        y1 = min(height, center_y + half_window + 1)
        x0 = max(0, center_x - half_window)
        x1 = min(width, center_x + half_window + 1)

        center_window = depth_mm[y0:y1, x0:x1]
        valid_center = center_window[np.isfinite(center_window) & (center_window > 0)]
        if valid_center.size == 0:
            return None

        center_depth = float(np.median(valid_center))
        lower = max(0.0, center_depth - self.depth_band_mm)
        upper = center_depth + self.depth_band_mm

        mask = np.logical_and(depth_mm >= lower, depth_mm <= upper)
        mask &= depth_mm > 0
        mask = mask.astype(np.uint8) * 255

        kernel = np.ones((self.morph_kernel_px, self.morph_kernel_px), dtype=np.uint8)
        if kernel.size > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        if num_labels <= 1:
            return None

        chosen_label = -1
        chosen_area = 0
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.min_area_px:
                continue
            if labels[center_y, center_x] == label:
                chosen_label = label
                break
            if area > chosen_area:
                chosen_area = area
                chosen_label = label

        if chosen_label < 0:
            return None

        if labels[center_y, center_x] != chosen_label:
            centroid_x, centroid_y = centroids[chosen_label]
            if (
                abs(float(centroid_x) - center_x) > self.max_center_offset_px
                or abs(float(centroid_y) - center_y) > self.max_center_offset_px
            ):
                return None

        x = int(stats[chosen_label, cv2.CC_STAT_LEFT])
        y = int(stats[chosen_label, cv2.CC_STAT_TOP])
        w = int(stats[chosen_label, cv2.CC_STAT_WIDTH])
        h = int(stats[chosen_label, cv2.CC_STAT_HEIGHT])

        pad = max(8, int(max(w, h) * 0.15))
        x = max(0, x - pad)
        y = max(0, y - pad)
        w = min(width - x, w + pad * 2)
        h = min(height - y, h + pad * 2)

        return x, y, w, h

    def _detect_kfs(
        self,
        color_image: np.ndarray,
        roi_x: int,
        roi_y: int,
        frame_w: int,
        frame_h: int,
    ) -> KfsDetection | None:
        if self.template_des is None or self.template_kp is None:
            return None

        if color_image.size == 0:
            return None

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        if self.query_scale != 1.0:
            gray = cv2.resize(gray, None, fx=self.query_scale, fy=self.query_scale)

        query_kp, query_des = self.akaze.detectAndCompute(gray, None)
        if query_des is None or len(query_kp) == 0:
            return None

        matches = self.bf.knnMatch(query_des, self.template_des, k=2)
        good_matches = []
        for pair in matches:
            if len(pair) < 2:
                continue
            first, second = pair
            if first.distance < self.match_ratio * second.distance:
                good_matches.append(first)

        if len(good_matches) < self.minimum_good_matches:
            return None

        good_matches = sorted(good_matches, key=lambda match: match.distance)
        if self.max_matches_to_use > 0:
            good_matches = good_matches[: self.max_matches_to_use]

        query_points = np.float32(
            [query_kp[match.queryIdx].pt for match in good_matches]
        ).reshape(-1, 1, 2)
        template_points = np.float32(
            [self.template_kp[match.trainIdx].pt for match in good_matches]
        ).reshape(-1, 1, 2)

        homography, mask = cv2.findHomography(
            template_points, query_points, cv2.RANSAC, 5.0
        )
        if homography is None:
            return None

        if mask is not None and int(mask.sum()) < self.minimum_good_matches:
            return None

        template_center = np.array(
            [[[self.template_w * 0.5, self.template_h * 0.5]]], dtype=np.float32
        )
        transformed_center = cv2.perspectiveTransform(template_center, homography)[0][0]

        center_x = float(transformed_center[0] / self.query_scale + roi_x)
        center_y = float(transformed_center[1] / self.query_scale + roi_y)

        if (
            center_x < 0.0
            or center_y < 0.0
            or center_x >= frame_w
            or center_y >= frame_h
        ):
            return None

        cube_center_x = roi_x + color_image.shape[1] * 0.5
        cube_center_y = roi_y + color_image.shape[0] * 0.5

        return KfsDetection(
            True,
            center_x,
            center_y,
            0.0,
            int(mask.sum()) if mask is not None else len(good_matches),
            cube_center_x,
            cube_center_y,
        )

    def _sample_depth_mm(
        self, depth_mm: np.ndarray, center_x: float, center_y: float
    ) -> float | None:
        if depth_mm.ndim != 2:
            return None

        height, width = depth_mm.shape[:2]
        px = int(round(center_x))
        py = int(round(center_y))
        if px < 0 or py < 0 or px >= width or py >= height:
            return None

        for window_px in (
            self.depth_sample_window_px,
            self.depth_sample_window_px * 2 + 1,
        ):
            half = max(1, window_px // 2)
            y0 = max(0, py - half)
            y1 = min(height, py + half + 1)
            x0 = max(0, px - half)
            x1 = min(width, px + half + 1)
            window = depth_mm[y0:y1, x0:x1]
            valid = window[np.isfinite(window) & (window > 0)]
            if valid.size > 0:
                return float(np.median(valid))

        value = float(depth_mm[py, px])
        if not math.isfinite(value) or value <= 0:
            return None
        return value

    def _publish_result(
        self,
        detection: KfsDetection,
        color_image: np.ndarray,
        cube_roi: tuple[int, int, int, int] | None,
    ) -> None:
        result_msg = Float32MultiArray()
        result_msg.data = [
            1.0 if detection.detected else 0.0,
            float(detection.center_x),
            float(detection.center_y),
            float(detection.depth_mm),
            float(detection.cube_center_x),
            float(detection.cube_center_y),
            float(detection.match_count),
        ]
        self.result_pub.publish(result_msg)

        detected_msg = Bool()
        detected_msg.data = bool(detection.detected)
        self.detected_pub.publish(detected_msg)

        debug_image = color_image.copy()
        if cube_roi is not None:
            x, y, w, h = cube_roi
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
        if detection.detected:
            cv2.circle(
                debug_image,
                (int(round(detection.center_x)), int(round(detection.center_y))),
                10,
                (0, 0, 255),
                -1,
            )
            label = f"z={detection.depth_mm:.1f}mm matches={detection.match_count}"
            cv2.putText(
                debug_image,
                label,
                (
                    int(round(detection.center_x)) + 12,
                    int(round(detection.center_y)) - 12,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        out_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        out_msg.header = self._latest_color_msg.header
        self.debug_pub.publish(out_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = KfsCubeFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
