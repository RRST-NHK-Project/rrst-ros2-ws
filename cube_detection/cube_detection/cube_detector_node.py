import cv2
import numpy as np
import rclpy
import struct
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32MultiArray


class CubeDetectorNode(Node):
    def __init__(self):
        super().__init__("cube_detector")

        self.declare_parameter("image_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("central_window_px", 40)
        self.declare_parameter("depth_band_mm", 180)
        self.declare_parameter("min_area_px", 350)
        self.declare_parameter("max_center_offset_px", 180)
        self.declare_parameter("morph_kernel_px", 5)
        self.declare_parameter("cube_size_mm", 350.0)
        self.declare_parameter("camera_fx_px", 615.0)
        self.declare_parameter("camera_fy_px", 615.0)
        self.declare_parameter("camera_cx_px", 320.0)
        self.declare_parameter("camera_cy_px", 240.0)
        self.declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info")
        self.declare_parameter("use_camera_info_intrinsics", True)
        self.declare_parameter("size_tolerance_ratio_min", 0.45)
        self.declare_parameter("size_tolerance_ratio_max", 1.90)
        self.declare_parameter("use_3d_filtering", True)
        self.declare_parameter("cube_3d_size_tolerance_mm", 100.0)
        self.declare_parameter("mask_y_top_ratio", 0.0)
        self.declare_parameter("mask_y_bottom_ratio", 0.5)
        self.declare_parameter("pointcloud_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("use_pointcloud_for_pose", True)
        self.declare_parameter("pointcloud_max_age_sec", 0.35)
        self.declare_parameter("camera_pitch_correction_deg", 0.0)

        image_topic = self.get_parameter("image_topic").value
        self.central_window_px = int(self.get_parameter("central_window_px").value)
        self.depth_band_mm = int(self.get_parameter("depth_band_mm").value)
        self.min_area_px = int(self.get_parameter("min_area_px").value)
        self.max_center_offset_px = int(
            self.get_parameter("max_center_offset_px").value
        )
        self.morph_kernel_px = max(1, int(self.get_parameter("morph_kernel_px").value))
        self.cube_size_mm = float(self.get_parameter("cube_size_mm").value)
        self.camera_fx_px = max(1.0, float(self.get_parameter("camera_fx_px").value))
        self.camera_fy_px = max(1.0, float(self.get_parameter("camera_fy_px").value))
        self.camera_cx_px = float(self.get_parameter("camera_cx_px").value)
        self.camera_cy_px = float(self.get_parameter("camera_cy_px").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.use_camera_info_intrinsics = bool(
            self.get_parameter("use_camera_info_intrinsics").value
        )
        self.size_ratio_min = float(
            self.get_parameter("size_tolerance_ratio_min").value
        )
        self.size_ratio_max = float(
            self.get_parameter("size_tolerance_ratio_max").value
        )
        self.use_3d_filtering = bool(self.get_parameter("use_3d_filtering").value)
        self.cube_3d_size_tolerance_mm = float(
            self.get_parameter("cube_3d_size_tolerance_mm").value
        )
        self.mask_y_top_ratio = float(self.get_parameter("mask_y_top_ratio").value)
        self.mask_y_bottom_ratio = float(
            self.get_parameter("mask_y_bottom_ratio").value
        )
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.use_pointcloud_for_pose = bool(
            self.get_parameter("use_pointcloud_for_pose").value
        )
        self.pointcloud_max_age_sec = float(
            self.get_parameter("pointcloud_max_age_sec").value
        )
        self.camera_pitch_correction_deg = float(
            self.get_parameter("camera_pitch_correction_deg").value
        )

        self.runtime_fx_px = self.camera_fx_px
        self.runtime_fy_px = self.camera_fy_px
        self.runtime_cx_px = self.camera_cx_px
        self.runtime_cy_px = self.camera_cy_px

        self.bridge = CvBridge()
        self.latest_pointcloud = None

        self.detected_pub = self.create_publisher(Bool, "/cube_detection/detected", 10)
        self.info_pub = self.create_publisher(
            Float32MultiArray, "/cube_detection/info", 10
        )
        self.debug_pub = self.create_publisher(Image, "/cube_detection/debug_image", 10)

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        if self.use_camera_info_intrinsics:
            self.create_subscription(
                CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
            )
        if self.use_pointcloud_for_pose:
            self.create_subscription(
                PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 10
            )

        self.get_logger().info(
            f"cube_detector started. depth_topic={image_topic}, "
            f"pointcloud_for_pose={self.use_pointcloud_for_pose}, "
            f"pointcloud_topic={self.pointcloud_topic}"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        try:
            k = msg.k
            fx = float(k[0])
            fy = float(k[4])
            cx = float(k[2])
            cy = float(k[5])
            if fx > 1.0 and fy > 1.0:
                self.runtime_fx_px = fx
                self.runtime_fy_px = fy
                self.runtime_cx_px = cx
                self.runtime_cy_px = cy
        except Exception:
            return

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        self.latest_pointcloud = msg

    def unproject_point(self, u: float, v: float, z_mm: float) -> tuple:
        """
        逆投影：画像座標と深度からカメラ座標系での3D座標を計算
        Args:
            u: ピクセル x座標
            v: ピクセル y座標
            z_mm: 深度値 (mm)
        Returns:
            (x_mm, y_mm, z_mm): カメラ座標系での3D座標 (mm)
        """
        z_m = z_mm / 1000.0
        x_m = (u - self.runtime_cx_px) * z_m / self.runtime_fx_px
        y_m = (v - self.runtime_cy_px) * z_m / self.runtime_fy_px
        return (x_m * 1000.0, y_m * 1000.0, z_mm)

    def check_3d_cube_validity(
        self, depth_image: np.ndarray, comp_mask: np.ndarray
    ) -> tuple:
        """
        3D座標ベースのキューブ検証（立方体のサイズが期待値に近いか確認）
        Args:
            depth_image: 深度画像
            comp_mask: 接続成分のマスク
        Returns:
            (is_valid: bool, size_x_mm: float, size_y_mm: float, size_z_mm: float)
        """
        if not self.use_3d_filtering:
            return (True, 0.0, 0.0, 0.0)

        # マスク内の有効なピクセルを抽出
        ys, xs = np.where(comp_mask > 0)
        if len(xs) == 0:
            return (False, 0.0, 0.0, 0.0)

        # 逆投影して3D座標に変換
        points_3d = []
        for x, y in zip(xs, ys):
            z_mm = depth_image[y, x]
            if z_mm > 0:
                x_mm, y_mm, z_mm = self.unproject_point(float(x), float(y), float(z_mm))
                points_3d.append((x_mm, y_mm, z_mm))

        if len(points_3d) < 10:  # 十分な点がない場合はスキップ
            return (False, 0.0, 0.0, 0.0)

        points_3d = np.array(points_3d)

        # 3D空間でのAABB (Axis-Aligned Bounding Box) を計算
        min_coords = np.min(points_3d, axis=0)
        max_coords = np.max(points_3d, axis=0)
        size_x = max_coords[0] - min_coords[0]
        size_y = max_coords[1] - min_coords[1]
        size_z = max_coords[2] - min_coords[2]

        # サイズのチェック（350mm ± tolerance）
        cube_size = self.cube_size_mm
        tolerance = self.cube_3d_size_tolerance_mm

        size_ok = (
            abs(size_x - cube_size) <= tolerance
            and abs(size_y - cube_size) <= tolerance
            and abs(size_z - cube_size) <= tolerance
        )

        return (size_ok, size_x, size_y, size_z)

    def apply_camera_pitch_correction(self, points: np.ndarray) -> np.ndarray:
        if points.shape[0] == 0:
            return points

        pitch_rad = np.deg2rad(self.camera_pitch_correction_deg)
        if abs(pitch_rad) < 1e-8:
            return points

        cos_p = float(np.cos(-pitch_rad))
        sin_p = float(np.sin(-pitch_rad))

        corrected = points.copy()
        y = corrected[:, 1]
        z = corrected[:, 2]
        corrected[:, 1] = cos_p * y - sin_p * z
        corrected[:, 2] = sin_p * y + cos_p * z
        return corrected

    def estimate_face_yaw_deg_from_points(self, points: np.ndarray) -> float:
        if points.shape[0] < 10:
            return float("nan")

        corrected_points = self.apply_camera_pitch_correction(points)

        centered = corrected_points - np.mean(corrected_points, axis=0, keepdims=True)

        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return float("nan")

        normal = vh[-1, :]
        normal_norm = float(np.linalg.norm(normal))
        if normal_norm < 1e-6:
            return float("nan")

        normal = normal / normal_norm
        if normal[2] < 0.0:
            normal = -normal

        yaw_deg = float(np.degrees(np.arctan2(normal[0], normal[2])))
        return yaw_deg

    def estimate_face_yaw_deg_from_depth(
        self, depth_image: np.ndarray, comp_mask: np.ndarray
    ) -> float:
        """
        深度画像を逆投影して検出面の法線YAWを推定
        カメラに対して正対時に 0 deg
        """
        ys, xs = np.where(comp_mask > 0)
        point_count = len(xs)
        if point_count < 30:
            return float("nan")

        # 点数が多すぎると重いので間引く
        sample_step = max(1, point_count // 1500)
        points_3d = []
        for x, y in zip(xs[::sample_step], ys[::sample_step]):
            z_mm = depth_image[y, x]
            if z_mm > 0:
                x_mm, y_mm, z_mm = self.unproject_point(float(x), float(y), float(z_mm))
                points_3d.append((x_mm, y_mm, z_mm))

        if len(points_3d) < 10:
            return float("nan")

        points = np.asarray(points_3d, dtype=np.float32)
        points[:, :2] /= 1000.0
        points[:, 2] /= 1000.0
        return self.estimate_face_yaw_deg_from_points(points)

    def estimate_face_yaw_deg_from_pointcloud(
        self, comp_mask: np.ndarray, image_msg: Image
    ) -> float:
        cloud = self.latest_pointcloud
        if cloud is None:
            return float("nan")

        if cloud.height <= 1:
            return float("nan")

        image_h, image_w = comp_mask.shape
        if cloud.width != image_w or cloud.height != image_h:
            return float("nan")

        image_stamp_ns = int(image_msg.header.stamp.sec) * 1_000_000_000 + int(
            image_msg.header.stamp.nanosec
        )
        cloud_stamp_ns = int(cloud.header.stamp.sec) * 1_000_000_000 + int(
            cloud.header.stamp.nanosec
        )
        age_sec = abs(image_stamp_ns - cloud_stamp_ns) / 1_000_000_000.0
        if age_sec > self.pointcloud_max_age_sec:
            return float("nan")

        field_offsets = {field.name: int(field.offset) for field in cloud.fields}
        if (
            "x" not in field_offsets
            or "y" not in field_offsets
            or "z" not in field_offsets
        ):
            return float("nan")

        x_offset = field_offsets["x"]
        y_offset = field_offsets["y"]
        z_offset = field_offsets["z"]

        ys, xs = np.where(comp_mask > 0)
        point_count = len(xs)
        if point_count < 30:
            return float("nan")

        sample_step = max(1, point_count // 1500)
        unpack_fmt = ">f" if cloud.is_bigendian else "<f"
        point_step = int(cloud.point_step)
        row_step = int(cloud.row_step)

        points_3d = []
        for x_px, y_px in zip(xs[::sample_step], ys[::sample_step]):
            base = int(y_px) * row_step + int(x_px) * point_step
            x_m = struct.unpack_from(unpack_fmt, cloud.data, base + x_offset)[0]
            y_m = struct.unpack_from(unpack_fmt, cloud.data, base + y_offset)[0]
            z_m = struct.unpack_from(unpack_fmt, cloud.data, base + z_offset)[0]

            if not np.isfinite(x_m) or not np.isfinite(y_m) or not np.isfinite(z_m):
                continue
            if z_m <= 0.0:
                continue
            points_3d.append((x_m, y_m, z_m))

        if len(points_3d) < 10:
            return float("nan")

        points = np.asarray(points_3d, dtype=np.float32)
        return self.estimate_face_yaw_deg_from_points(points)

    def estimate_face_yaw_deg(
        self, depth_image: np.ndarray, comp_mask: np.ndarray, image_msg: Image
    ) -> float:
        if self.use_pointcloud_for_pose:
            yaw_from_cloud = self.estimate_face_yaw_deg_from_pointcloud(
                comp_mask, image_msg
            )
            if np.isfinite(yaw_from_cloud):
                return yaw_from_cloud

        return self.estimate_face_yaw_deg_from_depth(depth_image, comp_mask)

    def image_callback(self, msg: Image) -> None:
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert depth image: {exc}")
            self.publish_no_detection()
            return

        if depth_image is None:
            self.publish_no_detection()
            return

        if depth_image.ndim == 3:
            depth_image = depth_image[:, :, 0]

        if depth_image.dtype != np.uint16:
            depth_image = depth_image.astype(np.uint16)

        height, width = depth_image.shape[:2]
        center_x = width // 2
        center_y = height // 2

        valid = (depth_image > 0) & np.isfinite(depth_image)
        if not np.any(valid):
            self.publish_no_detection(depth_image=depth_image)
            return

        half = self.central_window_px
        x0 = max(center_x - half, 0)
        x1 = min(center_x + half, width)
        y0 = max(center_y - half, 0)
        y1 = min(center_y + half, height)

        central_patch = depth_image[y0:y1, x0:x1]
        central_valid = central_patch[central_patch > 0]

        if central_valid.size > 0:
            center_depth_mm = int(np.median(central_valid))
        else:
            valid_values = depth_image[valid]
            center_depth_mm = int(np.percentile(valid_values, 30.0))

        lower = max(1, center_depth_mm - self.depth_band_mm)
        upper = center_depth_mm + self.depth_band_mm

        band_mask = ((depth_image >= lower) & (depth_image <= upper) & valid).astype(
            np.uint8
        )

        # Y軸方向のマスキング（段差など不要な領域を除外）
        y_mask = np.ones((height, width), dtype=np.uint8)
        y_min_px = int(height * self.mask_y_top_ratio)
        y_max_px = int(height * self.mask_y_bottom_ratio)
        y_mask[:y_min_px, :] = 0
        y_mask[y_max_px:, :] = 0
        band_mask = band_mask & y_mask

        kernel_size = self.morph_kernel_px
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        band_mask = cv2.morphologyEx(band_mask, cv2.MORPH_OPEN, kernel)
        band_mask = cv2.morphologyEx(band_mask, cv2.MORPH_CLOSE, kernel)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            band_mask, connectivity=8
        )

        selected_label = 0
        center_label = labels[center_y, center_x]
        if (
            center_label > 0
            and stats[center_label, cv2.CC_STAT_AREA] >= self.min_area_px
        ):
            selected_label = int(center_label)
        else:
            best_distance = float("inf")
            for label in range(1, num_labels):
                area = stats[label, cv2.CC_STAT_AREA]
                if area < self.min_area_px:
                    continue
                cx, cy = centroids[label]
                distance = float(np.hypot(cx - center_x, cy - center_y))
                if distance <= self.max_center_offset_px and distance < best_distance:
                    best_distance = distance
                    selected_label = label

        if selected_label <= 0:
            self.get_logger().debug(
                f"No valid component found. num_labels={num_labels}, "
                f"center_label_valid={labels[center_y, center_x] > 0}, "
                f"min_area_threshold={self.min_area_px}"
            )
            self.publish_no_detection(depth_image=depth_image, mask=band_mask)
            return

        x = int(stats[selected_label, cv2.CC_STAT_LEFT])
        y = int(stats[selected_label, cv2.CC_STAT_TOP])
        w = int(stats[selected_label, cv2.CC_STAT_WIDTH])
        h = int(stats[selected_label, cv2.CC_STAT_HEIGHT])
        area = int(stats[selected_label, cv2.CC_STAT_AREA])

        comp_mask = labels == selected_label
        comp_depth_values = depth_image[comp_mask]
        comp_depth_values = comp_depth_values[comp_depth_values > 0]
        if comp_depth_values.size > 0:
            depth_mm = float(np.median(comp_depth_values))
        else:
            depth_mm = float(center_depth_mm)

        cx, cy = centroids[selected_label]
        cx_norm = float(cx / max(width, 1))
        cy_norm = float(cy / max(height, 1))
        w_norm = float(w / max(width, 1))
        h_norm = float(h / max(height, 1))

        fill_ratio = float(area / max(w * h, 1))
        center_distance_norm = float(
            np.hypot(cx - center_x, cy - center_y) / max(np.hypot(width, height), 1.0)
        )

        expected_w_px = (self.cube_size_mm * self.runtime_fx_px) / max(depth_mm, 1.0)
        expected_h_px = (self.cube_size_mm * self.runtime_fy_px) / max(depth_mm, 1.0)
        size_ratio_w = float(w / max(expected_w_px, 1e-6))
        size_ratio_h = float(h / max(expected_h_px, 1e-6))
        size_ok = (
            self.size_ratio_min <= size_ratio_w <= self.size_ratio_max
            and self.size_ratio_min <= size_ratio_h <= self.size_ratio_max
        )
        size_match = max(
            0.0,
            min(
                1.0,
                1.0 - 0.5 * (abs(size_ratio_w - 1.0) + abs(size_ratio_h - 1.0)),
            ),
        )

        if not size_ok:
            self.get_logger().debug(
                f"Size filtering rejected. w_ratio={size_ratio_w:.2f}, h_ratio={size_ratio_h:.2f} "
                f"(valid: {self.size_ratio_min:.2f}-{self.size_ratio_max:.2f}), "
                f"expected={expected_w_px:.1f}x{expected_h_px:.1f}px, actual={w}x{h}px, "
                f"depth={depth_mm:.0f}mm"
            )
            self.publish_no_detection(depth_image=depth_image, mask=band_mask)
            return

        # 3D座標ベースのフィルター
        if self.use_3d_filtering:
            comp_mask = labels == selected_label
            is_3d_valid, size_x, size_y, size_z = self.check_3d_cube_validity(
                depth_image, comp_mask
            )
            if not is_3d_valid:
                self.get_logger().debug(
                    f"3D size filtering rejected. "
                    f"detected_size={size_x:.1f}x{size_y:.1f}x{size_z:.1f}mm, "
                    f"expected={self.cube_size_mm:.1f}mm ± {self.cube_3d_size_tolerance_mm:.1f}mm"
                )
                self.publish_no_detection(depth_image=depth_image, mask=band_mask)
                return

        face_yaw_deg = self.estimate_face_yaw_deg(depth_image, comp_mask, msg)
        if not np.isfinite(face_yaw_deg):
            face_yaw_deg = 0.0

        score = max(
            0.0,
            min(
                1.0,
                0.35 * fill_ratio
                + 0.30 * (1.0 - center_distance_norm * 2.0)
                + 0.35 * size_match,
            ),
        )

        detected_msg = Bool()
        detected_msg.data = True
        self.detected_pub.publish(detected_msg)

        # 検出情報をFloat32MultiArrayで配信:
        # [1.0, cx_norm, cy_norm, w_norm, h_norm, depth_m, score, area, face_yaw_deg]
        info_msg = Float32MultiArray()
        info_msg.data = [
            1.0,
            cx_norm,
            cy_norm,
            w_norm,
            h_norm,
            depth_mm / 1000.0,
            score,
            float(area),
            face_yaw_deg,
        ]
        self.info_pub.publish(info_msg)

        self.publish_debug_image(
            depth_image=depth_image,
            detected=True,
            bbox=(x, y, w, h),
            center=(int(cx), int(cy)),
            depth_m=depth_mm / 1000.0,
            score=score,
            face_yaw_deg=face_yaw_deg,
            mask=band_mask,
        )

    def publish_no_detection(self, depth_image=None, mask=None):
        detected_msg = Bool()
        detected_msg.data = False
        self.detected_pub.publish(detected_msg)

        info_msg = Float32MultiArray()
        info_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.info_pub.publish(info_msg)

        if depth_image is not None:
            self.publish_debug_image(
                depth_image=depth_image,
                detected=False,
                bbox=None,
                center=None,
                depth_m=0.0,
                score=0.0,
                face_yaw_deg=0.0,
                mask=mask,
            )

    def publish_debug_image(
        self,
        depth_image,
        detected,
        bbox,
        center,
        depth_m,
        score,
        face_yaw_deg,
        mask=None,
    ):
        clipped = np.clip(depth_image.astype(np.float32), 0.0, 4000.0)
        gray = (clipped / 4000.0 * 255.0).astype(np.uint8)
        debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        h, w = debug.shape[:2]
        cv2.line(debug, (w // 2 - 10, h // 2), (w // 2 + 10, h // 2), (255, 255, 0), 1)
        cv2.line(debug, (w // 2, h // 2 - 10), (w // 2, h // 2 + 10), (255, 255, 0), 1)

        if mask is not None:
            overlay = np.zeros_like(debug)
            overlay[:, :, 1] = (mask * 120).astype(np.uint8)
            debug = cv2.addWeighted(debug, 1.0, overlay, 0.35, 0)

        if detected and bbox is not None:
            x, y, bw, bh = bbox
            cv2.rectangle(debug, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            if center is not None:
                cv2.circle(debug, center, 4, (0, 255, 0), -1)
            text = (
                f"Cube depth={depth_m:.2f}m score={score:.2f} "
                f"yaw={face_yaw_deg:+.1f}deg"
            )
            cv2.putText(
                debug,
                text,
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                debug,
                "Cube not detected",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        image_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        self.debug_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
