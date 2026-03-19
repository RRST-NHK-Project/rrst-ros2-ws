import cv2
import numpy as np
from PIL import Image

aruco = cv2.aruco

# ===== 設定 =====
dpi = 300

# A4サイズ(px)
A4_WIDTH = int(210 / 25.4 * dpi)
A4_HEIGHT = int(297 / 25.4 * dpi)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

marker_size = 800  # ←ここだけ変えればOK（px）

margin = 20  # マーカー間の余白(px)

# =================


def create_marker(marker_id, size):
    return aruco.drawMarker(aruco_dict, marker_id, size)


def main():
    canvas = np.ones((A4_HEIGHT, A4_WIDTH), dtype=np.uint8) * 255

    step = marker_size + margin

    cols = A4_WIDTH // step
    rows = A4_HEIGHT // step

    marker_id = 0

    for y in range(rows):
        for x in range(cols):
            marker = create_marker(marker_id % 50, marker_size)

            pos_x = x * step + margin // 2
            pos_y = y * step + margin // 2

            canvas[pos_y : pos_y + marker_size, pos_x : pos_x + marker_size] = marker

            # ID表示（必要なければ消してOK）
            cv2.putText(
                canvas,
                str(marker_id % 50),
                (pos_x, pos_y + marker_size + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0),
                1,
                cv2.LINE_AA,
            )

            marker_id += 1

    img = Image.fromarray(canvas)
    img.save(f"aruco_A4_size_{marker_size}.png")

    print(f"完了：{cols}x{rows} = {cols*rows}個配置")


if __name__ == "__main__":
    main()
