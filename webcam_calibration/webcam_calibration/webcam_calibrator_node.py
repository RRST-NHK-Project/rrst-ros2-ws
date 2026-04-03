import os
from typing import List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import yaml


class WebcamCalibratorNode(Node):
    """Webカメラの内部パラメータを推定してYAMLに保存するノード."""

    def __init__(self) -> None:
        super().__init__("webcam_calibrator")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("board_rows", 6)
        self.declare_parameter("board_cols", 9)
        self.declare_parameter("square_size_m", 0.024)
        self.declare_parameter("required_samples", 20)
        self.declare_parameter("output_yaml", "webcam_calibration.yaml")
        self.declare_parameter("camera_name", "webcam")
        self.declare_parameter("show_undistorted", True)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.board_rows = int(self.get_parameter("board_rows").value)
        self.board_cols = int(self.get_parameter("board_cols").value)
        self.square_size_m = float(self.get_parameter("square_size_m").value)
        self.required_samples = int(self.get_parameter("required_samples").value)
        self.output_yaml = str(self.get_parameter("output_yaml").value)
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.show_undistorted = bool(self.get_parameter("show_undistorted").value)

        self.pattern_size = (self.board_cols, self.board_rows)

    def run(self) -> None:
        """キャリブレーション用画像を収集し、最終結果を保存する."""
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            self.get_logger().error(f"Cannot open camera index {self.camera_index}.")
            return

        self.get_logger().info("Calibration started.")
        self.get_logger().info("Press c: capture, k: calibrate, q/ESC: quit")

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

        obj_template = np.zeros(
            (self.board_rows * self.board_cols, 3),
            np.float32,
        )
        grid = np.mgrid[0 : self.board_cols, 0 : self.board_rows].T.reshape(-1, 2)
        obj_template[:, :2] = grid * self.square_size_m

        obj_points: List[np.ndarray] = []
        img_points: List[np.ndarray] = []
        image_size = None

        calibrated_mtx = None
        calibrated_dist = None

        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                self.get_logger().warning("Frame capture failed.")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            image_size = (gray.shape[1], gray.shape[0])

            found, corners = cv2.findChessboardCorners(gray, self.pattern_size)

            display = frame.copy()
            if found:
                refined = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    criteria,
                )
                cv2.drawChessboardCorners(
                    display,
                    self.pattern_size,
                    refined,
                    found,
                )
            else:
                refined = None

            self._draw_help_overlay(display, len(img_points))
            cv2.imshow("webcam_calibration", display)

            if self.show_undistorted and calibrated_mtx is not None:
                undist = cv2.undistort(frame, calibrated_mtx, calibrated_dist)
                cv2.imshow("webcam_calibration_undistorted", undist)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

            if key == ord("c"):
                if not found or refined is None:
                    self.get_logger().warning(
                        "Chessboard not detected. Capture skipped."
                    )
                    continue
                obj_points.append(obj_template.copy())
                img_points.append(refined)
                self.get_logger().info(
                    f"Captured sample {len(img_points)}/{self.required_samples}"
                )

            if key == ord("k") or len(img_points) >= self.required_samples:
                if len(img_points) < 3:
                    self.get_logger().warning("Need at least 3 samples to calibrate.")
                    continue
                calibrated_mtx, calibrated_dist = self._calibrate_and_save(
                    obj_points,
                    img_points,
                    image_size,
                )

        cap.release()
        cv2.destroyAllWindows()

    def _draw_help_overlay(self, image: np.ndarray, sample_count: int) -> None:
        """入力操作と取得枚数をプレビュー画像上に表示する."""
        lines = [
            f"Samples: {sample_count}/{self.required_samples}",
            "c: capture frame with chessboard",
            "k: calibrate now",
            "q or ESC: quit",
        ]
        y = 30
        for line in lines:
            cv2.putText(
                image,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            y += 28

    def _calibrate_and_save(
        self,
        obj_points: List[np.ndarray],
        img_points: List[np.ndarray],
        image_size,
    ):
        """OpenCVのcalibrateCameraを実行してYAMLを保存する."""
        ret, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
            obj_points,
            img_points,
            image_size,
            None,
            None,
        )
        if not ret:
            self.get_logger().error("Calibration failed.")
            return None, None

        output_path = os.path.abspath(self.output_yaml)
        self._write_calibration_yaml(
            output_path,
            image_size,
            camera_matrix,
            dist_coeffs,
        )

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        self.get_logger().info("Calibration completed.")
        self.get_logger().info(f"RMS reprojection error: {ret:.6f}")
        self.get_logger().info(f"fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}")
        self.get_logger().info(f"Saved calibration file: {output_path}")

        return camera_matrix, dist_coeffs

    def _write_calibration_yaml(
        self,
        output_path: str,
        image_size,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> None:
        """camera_info_manager互換フォーマットでYAMLを書き出す."""
        width, height = image_size
        camera_matrix_list = camera_matrix.reshape(-1).tolist()
        dist_coeffs_list = dist_coeffs.reshape(-1).tolist()

        projection = [
            camera_matrix[0, 0],
            0.0,
            camera_matrix[0, 2],
            0.0,
            0.0,
            camera_matrix[1, 1],
            camera_matrix[1, 2],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        data = {
            "image_width": int(width),
            "image_height": int(height),
            "camera_name": self.camera_name,
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [float(v) for v in camera_matrix_list],
            },
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": int(len(dist_coeffs_list)),
                "data": [float(v) for v in dist_coeffs_list],
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ],
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": [float(v) for v in projection],
            },
        }

        with open(output_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)


def main(args=None) -> None:
    """ノードエントリポイント."""
    rclpy.init(args=args)
    node = WebcamCalibratorNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
