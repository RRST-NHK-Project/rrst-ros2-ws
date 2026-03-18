import cv2
import numpy as np
from cv2 import aruco

def main():

    # カメラを開く
    # 0 はデフォルトのカメラ（USBカメラや内蔵カメラ）
    cap = cv2.VideoCapture(4, cv2.CAP_V4L2)

    if not cap.isOpened():
        print("カメラが開けません")
        exit()

    # ArUco 辞書
    #4x4 サイズ、50 個のマーカー
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    # ===== カメラ内部パラメータ（仮）=====
    camera_matrix = np.array([
        [600,   0, 320],
        [  0, 600, 240],
        [  0,   0,   1]
    ], dtype=np.float32)

    #| 値       | 意味            |
    #| ------- | ------------- |
    #| 600     | 焦点距離 fx, fy   |
    #| 320,240 | 画像中心 (cx, cy) |
    #| 下段      | 固定            |


    dist_coeffs = np.zeros((5, 1))

    # ===== マーカーの実サイズ（m）=====
    marker_length = 0.05  # 5cm

    while True:
        ret, frame = cap.read() #ret:取得成功フラグ, frame:取得画像
        if not ret:
            print("フレーム取得失敗")
            break

        # マーカー検出 corners:マーカーの4隅の座標, ids:マーカーID, rejected:検出失敗マーカー
        corners, ids, rejected = aruco.detectMarkers(
            frame, aruco_dict, parameters=parameters
        )

        if ids is not None:
            # マーカー枠描画
            aruco.drawDetectedMarkers(frame, corners, ids)

            # ===== 姿勢推定 =====
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                marker_length,
                camera_matrix,
                dist_coeffs
            )

            for i in range(len(ids)):
                # 座標軸描画
                cv2.drawFrameAxes(
                    frame,
                    camera_matrix,
                    dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    0.03  # 軸の長さ（m）
                )

                # 位置表示（カメラ座標系）
                t = tvecs[i][0]
                text = f"ID:{ids[i][0]} x:{t[0]:.2f} y:{t[1]:.2f} z:{t[2]:.2f} m"
                cv2.putText(
                    frame,
                    text,
                    (10, 30 + 30 * i),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

        cv2.imshow("Aruco Pose Estimation", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
