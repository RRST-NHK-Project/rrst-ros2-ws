from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cube_detection",
                executable="cube_detector",
                name="cube_detector",
                output="screen",
                parameters=[
                    {
                        # 入力の深度画像トピック (ROS2トピック名)
                        "image_topic": "/camera/camera/depth/image_rect_raw",
                        # 立方体検出の対象地域：画像中央からの検索窓サイズ (ピクセル)
                        "central_window_px": 40,
                        # 立方体がある深度範囲：中央画素からの±深度 (mm)
                        "depth_band_mm": 180,
                        # 接続成分の最小面積閾値 (ピクセル)
                        "min_area_px": 350,
                        # 検出対象立方体の実際のサイズ (mm) - ロボット上の350mm立方体を想定
                        "cube_size_mm": 350.0,
                        # カメラの焦点距離 X (ピクセル) - フォールバック値; CameraInfoで上書き可
                        "camera_fx_px": 615.0,
                        # カメラの焦点距離 Y (ピクセル) - フォールバック値; CameraInfoで上書き可
                        "camera_fy_px": 615.0,
                        # カメラの主点 X座標 (ピクセル) - フォールバック値; CameraInfoで上書き可
                        "camera_cx_px": 320.0,
                        # カメラの主点 Y座標 (ピクセル) - フォールバック値; CameraInfoで上書き可
                        "camera_cy_px": 240.0,
                        # カメラ内部パラメータ配信トピック (fx, fy, cx, cy を動的に取得)
                        "camera_info_topic": "/camera/camera/depth/camera_info",
                        # CameraInfo トピックから焦点距離を動的に取得するか (True: D456実際の値を使用)
                        "use_camera_info_intrinsics": True,
                        # 検出立方体のサイズ許容度（下限）：予想サイズに対する比率の最小値
                        "size_tolerance_ratio_min": 0.45,
                        # 検出立方体のサイズ許容度（上限）：予想サイズに対する比率の最大値 (ステップ誤検知を防ぐ)
                        "size_tolerance_ratio_max": 1.90,
                    }
                ],
            )
        ]
    )
