#camera確認用
#仮想環境を有効にする（venv を使っている場合）
#source venv/bin/activate
#cd ros2_ws/src/aruco_tracker/
#実行コードpython camera.py
import cv2

# 0 はデフォルトのカメラ（USBカメラや内蔵カメラ）
import cv2

cap = cv2.VideoCapture(32, cv2.CAP_V4L2)

if not cap.isOpened():
    print("カメラが開けません")
    exit()

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        print("フレームが取得できません")
        break

    cv2.imshow("Webcam", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


if not cap.isOpened():
    print("カメラが開けません")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("フレームが取得できません")
        break

    cv2.imshow("Webcam", frame)

    # 'q'キーで終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
