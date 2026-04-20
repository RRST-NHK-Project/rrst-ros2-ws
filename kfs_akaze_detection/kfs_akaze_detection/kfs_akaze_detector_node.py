from __future__ import annotations

import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

from .interfaces import KfsDetectionResult, NoOpDepthFusionAdapter


class KfsAkazeDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("kfs_akaze_detector")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("template_package", "kfs_pkg")
        self.declare_parameter("template_image_name", "KFS_image_list.png")
        self.declare_parameter("query_scale", 0.5)
        self.declare_parameter("match_ratio", 0.87)
        self.declare_parameter("minimum_good_matches", 14)
        self.declare_parameter("output_topic", "/kfs_akaze_detection/result")
        self.declare_parameter("detected_topic", "/kfs_akaze_detection/detected")
        self.declare_parameter("debug_image_topic", "/kfs_akaze_detection/debug_image")
        self.declare_parameter(
            "depth_info_topic", "/kfs_akaze_detection/depth_observation"
        )
        self.declare_parameter("enable_depth_adapter", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.template_package = str(self.get_parameter("template_package").value)
        self.template_image_name = str(self.get_parameter("template_image_name").value)
        self.query_scale = max(0.1, float(self.get_parameter("query_scale").value))
        self.match_ratio = float(self.get_parameter("match_ratio").value)
        self.minimum_good_matches = int(
            self.get_parameter("minimum_good_matches").value
        )
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.detected_topic = str(self.get_parameter("detected_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.depth_info_topic = str(self.get_parameter("depth_info_topic").value)
        self.enable_depth_adapter = bool(
            self.get_parameter("enable_depth_adapter").value
        )

        self.bridge = CvBridge()
        self.akaze = cv2.AKAZE_create()
        self.bf = cv2.BFMatcher()

        self.depth_adapter = NoOpDepthFusionAdapter()

        self.template_gray = self._load_template_image()
        self.template_kp = []
        self.template_des = None

        if self.template_gray is not None:
            self.template_kp, self.template_des = self.akaze.detectAndCompute(
                self.template_gray, None
            )

        self.result_pub = self.create_publisher(
            Float32MultiArray, self.output_topic, 10
        )
        self.detected_pub = self.create_publisher(Bool, self.detected_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        if self.enable_depth_adapter:
            self.create_subscription(
                Float32MultiArray, self.depth_info_topic, self.depth_info_callback, 10
            )

        self.get_logger().info(
            "kfs_akaze_detector started: image=%s template=%s/%s"
            % (self.image_topic, self.template_package, self.template_image_name)
        )

        if self.template_gray is None or self.template_des is None:
            self.get_logger().error(
                "テンプレート画像の読み込みまたは特徴抽出に失敗しました。"
            )

    def _load_template_image(self) -> np.ndarray | None:
        candidate_paths = []

        try:
            template_share = get_package_share_directory(self.template_package)
            candidate_paths.append(
                os.path.join(template_share, "resource", self.template_image_name)
            )
        except Exception as exc:
            self.get_logger().warn(f"template package lookup failed: {exc}")

        try:
            own_share = get_package_share_directory("kfs_akaze_detection")
            candidate_paths.append(
                os.path.join(own_share, "resource", self.template_image_name)
            )
        except Exception:
            pass

        for candidate in candidate_paths:
            if not os.path.exists(candidate):
                continue
            template = cv2.imread(candidate, cv2.IMREAD_GRAYSCALE)
            if template is not None:
                self.get_logger().info(f"template loaded: {candidate}")
                return template

        return None

    def depth_info_callback(self, msg: Float32MultiArray) -> None:
        self.depth_adapter.update_depth_observation(list(msg.data))

    def image_callback(self, msg: Image) -> None:
        if self.template_des is None or len(self.template_kp) == 0:
            self._publish_result(KfsDetectionResult(detected=False), None)
            return

        try:
            color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return

        detection, debug = self._detect(color)
        detection = self.depth_adapter.inject_depth(detection)
        self._publish_result(detection, debug)

    def _detect(self, image_bgr: np.ndarray) -> tuple[KfsDetectionResult, np.ndarray]:
        query_gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        if self.query_scale != 1.0:
            query_gray = cv2.resize(
                query_gray, None, fx=self.query_scale, fy=self.query_scale
            )

        query_kp, query_des = self.akaze.detectAndCompute(query_gray, None)
        if query_des is None or len(query_kp) == 0:
            return KfsDetectionResult(detected=False), self._draw_debug(
                query_gray,
            )

        raw_matches = self.bf.knnMatch(query_des, self.template_des, k=2)
        good = []
        for pair in raw_matches:
            if len(pair) < 2:
                continue
            first, second = pair
            if first.distance < self.match_ratio * second.distance:
                good.append(first)

        if len(good) < self.minimum_good_matches:
            return KfsDetectionResult(
                detected=False, match_count=len(good)
            ), self._draw_debug(query_gray)

        src_pts = np.float32([query_kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([self.template_kp[m.trainIdx].pt for m in good]).reshape(
            -1, 1, 2
        )

        homography, inlier_mask = cv2.findHomography(
            src_pts, dst_pts, method=cv2.RANSAC, ransacReprojThreshold=4.0
        )

        if homography is None or inlier_mask is None:
            return KfsDetectionResult(
                detected=False, match_count=len(good)
            ), self._draw_debug(query_gray)

        center_query = np.array(
            [[[query_gray.shape[1] * 0.5, query_gray.shape[0] * 0.5]]], dtype=np.float32
        )
        center_map = cv2.perspectiveTransform(center_query, homography)[0, 0]

        # Estimate local yaw/scale around image center using a small probe segment.
        probe_len = 20.0
        center_x = float(center_query[0, 0, 0])
        center_y = float(center_query[0, 0, 1])
        probe_query = np.array(
            [[[center_x, center_y]], [[center_x + probe_len, center_y]]],
            dtype=np.float32,
        )
        probe_map = cv2.perspectiveTransform(probe_query, homography)
        probe_vec = probe_map[1, 0] - probe_map[0, 0]
        vec_norm = float(np.linalg.norm(probe_vec))
        if vec_norm <= 1e-6:
            return KfsDetectionResult(
                detected=False, match_count=len(good)
            ), self._draw_debug(query_gray)

        yaw_deg = float(np.degrees(np.arctan2(probe_vec[1], probe_vec[0])))
        scale = float(vec_norm / probe_len)

        inliers = int(np.sum(inlier_mask))
        inlier_ratio = float(inliers) / float(len(good))

        result = KfsDetectionResult(
            detected=True,
            map_x=float(center_map[0]),
            map_y=float(center_map[1]),
            yaw_deg=yaw_deg,
            scale=scale,
            match_count=len(good),
            inlier_ratio=inlier_ratio,
        )

        template_h, template_w = self.template_gray.shape[:2]
        template_corners = np.float32(
            [
                [[0.0, 0.0]],
                [[template_w - 1.0, 0.0]],
                [[template_w - 1.0, template_h - 1.0]],
                [[0.0, template_h - 1.0]],
            ]
        )
        try:
            inv_homography = np.linalg.inv(homography)
        except np.linalg.LinAlgError:
            return KfsDetectionResult(
                detected=False, match_count=len(good)
            ), self._draw_debug(query_gray)

        bbox_query = cv2.perspectiveTransform(template_corners, inv_homography)

        debug = self._draw_debug(
            query_gray,
            bbox_query=bbox_query,
            match_count=len(good),
            inlier_ratio=inlier_ratio,
        )
        return result, debug

    def _draw_debug(
        self,
        query_gray: np.ndarray,
        bbox_query: np.ndarray | None = None,
        match_count: int = 0,
        inlier_ratio: float = 0.0,
    ) -> np.ndarray:
        query_vis = (
            query_gray
            if query_gray.ndim == 2
            else cv2.cvtColor(query_gray, cv2.COLOR_BGR2GRAY)
        )
        query_bgr = cv2.cvtColor(query_vis, cv2.COLOR_GRAY2BGR)

        if bbox_query is not None and bbox_query.size == 8:
            polygon = np.round(bbox_query).astype(np.int32)
            cv2.polylines(
                query_bgr,
                [polygon.reshape(-1, 1, 2)],
                isClosed=True,
                color=(0, 255, 0),
                thickness=2,
                lineType=cv2.LINE_AA,
            )

            center = np.mean(polygon.reshape(-1, 2), axis=0).astype(np.int32)
            cv2.circle(query_bgr, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

            status_text = f"matches={match_count} inlier={inlier_ratio:.2f}"
            cv2.putText(
                query_bgr,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return query_bgr

    def _publish_result(
        self, result: KfsDetectionResult, debug_image: np.ndarray | None
    ) -> None:
        self.detected_pub.publish(Bool(data=bool(result.detected)))

        msg = Float32MultiArray()
        msg.data = [
            1.0 if result.detected else 0.0,
            float(result.map_x),
            float(result.map_y),
            float(result.yaw_deg),
            float(result.scale),
            float(result.match_count),
            float(result.inlier_ratio),
            float(result.depth_mm),
        ]
        self.result_pub.publish(msg)

        if debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                self.debug_pub.publish(debug_msg)
            except Exception as exc:
                self.get_logger().warn(f"debug image publish failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KfsAkazeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
