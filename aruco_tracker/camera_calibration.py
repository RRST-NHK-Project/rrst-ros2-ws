import cv2
import numpy as np
import glob

# チェスボードの内側コーナー数（交点数）
CHECKERBOARD = (9, 6)

# サブピクセルコーナー検出条件
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 3D空間上の点座標（Z=0 の平面上）
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)

# 画像上の点と3D点を格納するリスト
objpoints = []  # 3D点
imgpoints = []  # 2D点

# キャプチャ開始
cap = cv2.VideoCapture(0)
print("📷 スペースキーで撮影、ESCで終了")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)

    cv2.imshow("Calibration", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == 32 and ret:  # スペースキー
        print("✅ 画像を追加")
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
    elif key == 27:  # ESCキーで終了
        break

cap.release()
cv2.destroyAllWindows()

# 十分な画像が集まったらキャリブレーション実行
if len(objpoints) > 0:
    print(f"\n📐 {len(objpoints)} 枚の画像でキャリブレーション中...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print("\n=== カメラパラメータ ===")
    print("camera_matrix:\n", camera_matrix)
    print("dist_coeffs:\n", dist_coeffs)

    # 結果を保存
    np.savez(
        "camera_parameters.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs
    )

    print("\n💾 camera_parameters.npz に保存しました。")
else:
    print("❌ キャリブレーションに十分な画像がありません。")
