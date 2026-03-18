# GPT製注意

import cv2
import cv2.aruco as aruco
import os

def main():
    # このスクリプト(.py)と同じディレクトリを取得
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # 使用する辞書を選択
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    # マーカー設定
    marker_id = 0
    marker_size = 200  # ピクセル単位

    # マーカー生成
    marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

    # 保存パスを作成
    filename = f"aruco_marker_{marker_id}.png"
    save_path = os.path.join(script_dir, filename)

    # 画像を保存
    cv2.imwrite(save_path, marker_image)
    print(f"Saved: {save_path}")

    # 表示（任意）
    cv2.imshow("ArUco Marker", marker_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

