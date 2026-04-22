from __future__ import annotations

import json
from pathlib import Path
import site
import sys
from typing import Dict, List, Optional, Sequence, Tuple

# Prevent ABI mismatch with ROS binary modules (e.g. cv_bridge) when user-site has NumPy 2.x.
USER_SITE = site.getusersitepackages()
if USER_SITE in sys.path:
    sys.path.remove(USER_SITE)

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


def order_corners(pts: np.ndarray) -> np.ndarray:
    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)
    top_left = pts[np.argmin(sums)]
    bottom_right = pts[np.argmax(sums)]
    top_right = pts[np.argmin(diffs)]
    bottom_left = pts[np.argmax(diffs)]
    return np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)


class KFSCubeIdentifierNode(Node):
    def __init__(self) -> None:
        super().__init__("kfs_cube_identifier")
        self.bridge = CvBridge()

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("result_topic", "/kfs_cube_identifier/result")
        self.declare_parameter("debug_image_topic", "/kfs_cube_identifier/debug_image")
        self.declare_parameter(
            "kfs_dictionary_dir",
            "/home/dev/ros2_ws/src/KFS_judgement_machines/AKAZA/KFS Image V1.0",
        )
        self.declare_parameter("warp_size", 240)
        self.declare_parameter("min_face_area", 1200.0)
        self.declare_parameter("match_ratio", 0.85)
        self.declare_parameter("min_good_matches", 8)
        self.declare_parameter("min_match_score", 0.03)
        self.declare_parameter("min_mask_pixels", 800)

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        result_topic = (
            self.get_parameter("result_topic").get_parameter_value().string_value
        )
        debug_image_topic = (
            self.get_parameter("debug_image_topic").get_parameter_value().string_value
        )
        dict_dir = (
            self.get_parameter("kfs_dictionary_dir").get_parameter_value().string_value
        )

        self.warp_size = int(self.get_parameter("warp_size").value)
        self.min_face_area = float(self.get_parameter("min_face_area").value)
        self.match_ratio = float(self.get_parameter("match_ratio").value)
        self.min_good_matches = int(self.get_parameter("min_good_matches").value)
        self.min_match_score = float(self.get_parameter("min_match_score").value)
        self.min_mask_pixels = int(self.get_parameter("min_mask_pixels").value)

        self.akaze = cv2.AKAZE_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

        self.references = self._load_reference_descriptors(Path(dict_dir))
        self.reference_preview_map: Dict[str, np.ndarray] = {
            str(ref["name"]): ref["preview"]
            for ref in self.references
            if "preview" in ref
        }
        if not self.references:
            self.get_logger().warn(
                "No reference images loaded. Check kfs_dictionary_dir parameter."
            )
        else:
            self.get_logger().info(
                f"Loaded {len(self.references)} KFS reference images"
            )

        self.image_sub = self.create_subscription(
            Image, image_topic, self._on_image, 10
        )
        self.result_pub = self.create_publisher(String, result_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_image_topic, 10)

    def _load_reference_descriptors(self, directory: Path) -> List[Dict[str, object]]:
        references: List[Dict[str, object]] = []
        if not directory.exists() or not directory.is_dir():
            self.get_logger().error(f"Reference directory not found: {directory}")
            return references

        for path in sorted(directory.glob("*.png")):
            img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue

            kp, des = self.akaze.detectAndCompute(img, None)
            if des is None or len(kp) == 0:
                continue

            preview = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            references.append(
                {
                    "name": path.stem,
                    "des": des,
                    "keypoints": len(kp),
                    "preview": preview,
                }
            )
        return references

    def _extract_color_mask(self, hsv: np.ndarray, color_name: str) -> np.ndarray:
        if color_name == "red":
            # Hue wraps around for red and needs two ranges.
            low1 = np.array([0, 50, 40], dtype=np.uint8)
            high1 = np.array([12, 255, 255], dtype=np.uint8)
            low2 = np.array([165, 50, 40], dtype=np.uint8)
            high2 = np.array([180, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, low1, high1) | cv2.inRange(hsv, low2, high2)
        else:
            low = np.array([90, 45, 35], dtype=np.uint8)
            high = np.array([140, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, low, high)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _find_best_quad(self, mask: np.ndarray) -> Tuple[Optional[np.ndarray], float]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_quad = None
        best_area = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_face_area:
                continue
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.03 * peri, True)
            if len(approx) == 4:
                if area > best_area:
                    best_area = area
                    best_quad = approx.reshape(4, 2).astype(np.float32)
            else:
                # Fallback: min area rectangle is robust when contour is noisy.
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box_area = cv2.contourArea(box.astype(np.float32))
                if box_area > best_area and box_area >= self.min_face_area:
                    best_area = box_area
                    best_quad = box.astype(np.float32)

        if best_quad is None:
            return None, 0.0
        return order_corners(best_quad), float(best_area)

    def _warp_face(self, frame: np.ndarray, corners: np.ndarray) -> np.ndarray:
        target = np.array(
            [
                [0, 0],
                [self.warp_size - 1, 0],
                [self.warp_size - 1, self.warp_size - 1],
                [0, self.warp_size - 1],
            ],
            dtype=np.float32,
        )
        h = cv2.getPerspectiveTransform(corners, target)
        return cv2.warpPerspective(frame, h, (self.warp_size, self.warp_size))

    def _match_reference(
        self, warped_bgr: np.ndarray
    ) -> Tuple[Optional[str], float, int]:
        gray = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        kp_q, des_q = self.akaze.detectAndCompute(gray, None)
        if des_q is None or len(kp_q) == 0:
            return None, 0.0, 0

        best_label: Optional[str] = None
        best_score = 0.0
        best_good = 0

        for ref in self.references:
            des_r = ref["des"]
            matches = self.matcher.knnMatch(des_q, des_r, k=2)
            good = [m for m, n in matches if m.distance < self.match_ratio * n.distance]
            if not good:
                continue

            norm = max(min(len(kp_q), int(ref["keypoints"])), 1)
            score = float(len(good)) / float(norm)
            if score > best_score:
                best_score = score
                best_label = str(ref["name"])
                best_good = len(good)

        if best_good < self.min_good_matches and best_score < self.min_match_score:
            return None, best_score, best_good

        return best_label, best_score, best_good

    def _on_image(self, msg: Image) -> None:
        if not self.references:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        result: Dict[str, object] = {
            "detected": False,
            "cube_color": None,
            "label": None,
            "score": 0.0,
            "good_matches": 0,
            "corners": [],
            "stage": "no_color_or_match",
            "per_color": {},
        }

        debug = frame.copy()
        best_candidate: Optional[Dict[str, object]] = None
        best_warped: Optional[np.ndarray] = None
        best_reference: Optional[np.ndarray] = None

        for color in ("red", "blue"):
            mask = self._extract_color_mask(hsv, color)
            mask_pixels = int(np.count_nonzero(mask))
            color_info: Dict[str, object] = {
                "mask_pixels": mask_pixels,
                "quad_found": False,
                "quad_area": 0.0,
                "label": None,
                "score": 0.0,
                "good_matches": 0,
            }
            if mask_pixels < self.min_mask_pixels:
                result["per_color"][color] = color_info
                continue

            corners, quad_area = self._find_best_quad(mask)
            if corners is None:
                result["per_color"][color] = color_info
                continue

            color_info["quad_found"] = True
            color_info["quad_area"] = round(float(quad_area), 2)

            warped = self._warp_face(frame, corners)
            label, score, good_matches = self._match_reference(warped)
            color_info["label"] = label
            color_info["score"] = round(float(score), 4)
            color_info["good_matches"] = int(good_matches)
            result["per_color"][color] = color_info
            if label is None:
                continue

            candidate = {
                "detected": True,
                "cube_color": color,
                "label": label,
                "score": round(score, 4),
                "good_matches": int(good_matches),
                "corners": [[float(x), float(y)] for x, y in corners],
            }

            if best_candidate is None or candidate["score"] > best_candidate["score"]:
                best_candidate = candidate
                best_warped = warped
                best_reference = self.reference_preview_map.get(label)

        if best_candidate is not None:
            result = best_candidate
            result["stage"] = "detected"
            poly = np.array(result["corners"], dtype=np.int32)
            cv2.polylines(debug, [poly], True, (0, 255, 0), 2)
            cv2.putText(
                debug,
                f"{result['cube_color']}:{result['label']} score={result['score']}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (20, 220, 20),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                debug,
                "No detection",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (20, 20, 220),
                2,
                cv2.LINE_AA,
            )

        if best_warped is not None:
            preview_size = min(220, debug.shape[0] // 3, debug.shape[1] // 3)
            if preview_size >= 80:
                preview = cv2.resize(best_warped, (preview_size, preview_size))
                y1, y2 = 12, 12 + preview_size
                x2 = debug.shape[1] - 12
                x1 = x2 - preview_size
                debug[y1:y2, x1:x2] = preview
                cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(
                    debug,
                    "Warped",
                    (x1, max(20, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                if best_reference is not None:
                    ref_preview = cv2.resize(
                        best_reference, (preview_size, preview_size)
                    )
                    rx2 = x1 - 12
                    rx1 = rx2 - preview_size
                    if rx1 >= 0:
                        debug[y1:y2, rx1:rx2] = ref_preview
                        cv2.rectangle(debug, (rx1, y1), (rx2, y2), (255, 180, 0), 2)
                        cv2.putText(
                            debug,
                            "Matched Dict",
                            (rx1, max(20, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 180, 0),
                            2,
                            cv2.LINE_AA,
                        )

        out_msg = String()
        out_msg.data = json.dumps(result, ensure_ascii=True)
        self.result_pub.publish(out_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = KFSCubeIdentifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
