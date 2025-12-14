#camera確認用
#仮想環境を有効にする（venv を使っている場合）
#source venv/bin/activate
#cd ros2_ws/src/aruco_tracker/
#実行コードpython camera.py
import cv2

# 0 はデフォルトのカメラ（USBカメラや内蔵カメラ）
cap = cv2.VideoCapture(0)

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
