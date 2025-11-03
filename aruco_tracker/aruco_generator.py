#!/usr/bin/env python3
# coding: utf-8
import cv2
import numpy as np

aruco = cv2.aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
pixel = 150
offset = 10
cnt = 9


def generateArMarker():
    img = np.zeros((pixel + offset, pixel + offset), dtype=np.uint8)
    img += 255

    x_offset = y_offset = int(offset) // 2

    for i in range(cnt):
        # OpenCV 5.x の場合、次のようにアクセスする
        try:
            ar_image = aruco.drawMarker(dictionary, i, pixel, 3)
        except AttributeError:
            # drawMarker が移動している場合
            ar_image = aruco.utils.drawMarker(dictionary, i, pixel)

        filename = f"ar{i}.png"
        img[
            y_offset : y_offset + ar_image.shape[0],
            x_offset : x_offset + ar_image.shape[1],
        ] = ar_image

        rgb_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        if i % 3 == 0:
            hconcat_img = rgb_img
        elif i % 3 <= 2:
            hconcat_img = cv2.hconcat([hconcat_img, rgb_img])
            if i % 3 == 2 and i // 3 == 0:
                vconcat_img = hconcat_img
            elif i % 3 == 2 and i // 3 > 0:
                vconcat_img = cv2.vconcat([vconcat_img, hconcat_img])

        cv2.imwrite(filename, rgb_img)

    cv2.imwrite(f"ar{cnt}.png", vconcat_img)


if __name__ == "__main__":
    generateArMarker()
