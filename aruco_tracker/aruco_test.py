import cv2
from cv2 import aruco

# カメラを開く（0 でだめなら 1, 2 試す）
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("カメラが開けません")
    exit()

# 使用する辞書（4x4 の 50種類）
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

while True:
    ret, frame = cap.read()
    if not ret:
        print("フレーム取得失敗")
        break

    # マーカー検出
    corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    # マーカーが見つかったら描画
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow("Aruco Detector", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
