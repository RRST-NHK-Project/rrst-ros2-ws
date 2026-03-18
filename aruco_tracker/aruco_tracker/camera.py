#camera確認用
#仮想環境を有効にする（venv を使っている場合）
#source venv/bin/activate
#cd ros2_ws/src/aruco_tracker/
#実行コードpython camera.py

# 0 はデフォルトのカメラ（USBカメラや内蔵カメラ）
import cv2

def main():

    cap = cv2.VideoCapture(4, cv2.CAP_V4L2)

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

    # # 最も映る可能性が高い順に試す
    # for cam_index in [0, 2, 4]:
    #     cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
    #     if cap.isOpened():
    #         print(f"Camera opened at index {cam_index}")
    #         break
    #     cap.release()
    # else:
    #     print("どのカメラも開けません")
    #     return

    # # MJPG を設定（対応していれば高速）
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # while True:
    #     ret, frame = cap.read()
    #     if not ret:
    #         print("フレームが取得できません")
    #         break

    #     cv2.imshow("Webcam", frame)

    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    # cap.release()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()