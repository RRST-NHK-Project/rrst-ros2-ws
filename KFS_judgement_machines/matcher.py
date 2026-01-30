#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#アフィン変換だけだと不十分なので、ホモグラフィ変換もやります。

import time
import cv2
import math
from PIL import Image
import statistics
import collections
import numpy as np
from matplotlib import pyplot as plt
import os

# 上位のキーポイントの相対関係を全て調べて多数決を取ることでノイズに強くする
def vote_point(query_kp, map_kp, point_num):

    # 点i, jの相対角度と相対長さを格納する配列
    deg_cand = np.zeros((point_num, point_num))
    len_cand = np.zeros((point_num, point_num))

    # 全ての点のサイズ比，相対角度を求める
    for i in range(point_num):
        for j in range(i+1, point_num):
            # クエリ画像から特徴点間の角度と距離を計算
            q_x1, q_y1 = query_kp[i].pt
            q_x2, q_y2 = query_kp[j].pt
            q_deg = math.atan2(q_y2 - q_y1, q_x2 - q_x1) * 180 / math.pi
            q_len = math.sqrt((q_x2 - q_x1) ** 2 + (q_y2 - q_y1) ** 2)

            # マップ画像から特徴点間の角度と距離を計算
            m_x1, m_y1 = map_kp[i].pt
            m_x2, m_y2 = map_kp[j].pt
            m_deg = math.atan2(m_y2 - m_y1, m_x2 - m_x1) * 180 / math.pi
            m_len = math.sqrt((m_x2 - m_x1) ** 2 + (m_y2 - m_y1) ** 2)

            #print(q_x1, q_y1, q_x2, q_y2, q_deg, q_len)
            #print(m_x1, m_y1, m_x2, m_y2, m_deg, m_len)

            # 2つの画像の相対角度と距離
            deg_value = q_deg - m_deg
            if deg_value < 0:
                deg_value += 360
            if m_len <= 0:
                continue
            size_rate = q_len/m_len

            deg_cand[i][j] = deg_value
            deg_cand[j][i] = deg_value
            len_cand[i][j] = size_rate
            len_cand[j][i] = size_rate

    # print(deg_cand)
    # print(len_cand)

    # 多数決を取る
    # ある点iについて，j, kとの相対関係が一致するかを各jについて調べる
    cand_count = np.zeros((point_num, point_num))
    size_range_min = 0.3  # 明らかに違う比率の結果を弾く重要パラメータ
    size_range_max = 3  # 明らかに違う比率の結果を弾く重要パラメータ
    dif_range = 0.05  # 重要パラメータ

    for i in range(len(deg_cand)):
        for j in range(len(deg_cand)):
            # 明らかに違う比率の結果を弾く
            if len_cand[i][j] < size_range_min or len_cand[i][j] > size_range_max:
                    continue

            for k in range(len(deg_cand)):
                # 明らかに違う比率の結果を弾く
                if len_cand[i][k] < size_range_min or len_cand[i][k] > size_range_max:
                    continue

                # 誤差がある範囲以下の値なら同じ値とみなす
                deg_dif = np.abs(deg_cand[i][k] - deg_cand[i][j])
                size_dif = np.abs(len_cand[i][k] - len_cand[i][j])
                if deg_dif <= deg_cand[i][j]*dif_range and size_dif <= len_cand[i][j]*dif_range:
                    cand_count[i][j] += 1

    # print(cand_count)

    # どの2点も同じ相対関係になかった場合
    if np.max(cand_count) <= 1:
        print("[error] no matching point pair")
        return None, None, None, None

    # もっとも多く相対関係が一致する2点を取ってくる
    maxidx = np.unravel_index(np.argmax(cand_count), cand_count.shape)
    deg_value = deg_cand[maxidx]
    size_rate = len_cand[maxidx]

    return deg_value, size_rate, maxidx[0], maxidx[1]


# 最終的な描画関数
def draw_final(result_img, m_xcenter, m_ycenter, deg_value, width_query):
    # 中心点の描画
    cv2.circle(result_img, (int(m_xcenter) + width_query, int(m_ycenter)), 20, color=(0, 0, 255), thickness=-1)

    # 向きの計算，矢印描画
    deg_front = - deg_value * math.pi / 180 - math.pi/2
    q_xfront = m_xcenter + 200 * math.cos(deg_front)
    q_yfront = m_ycenter + 200 * math.sin(deg_front)
    cv2.arrowedLine(result_img, (int(m_xcenter) + width_query, int(m_ycenter)),
                    (int(q_xfront) + width_query, int(q_yfront)), color=(255, 0, 0), thickness=15)

    final_img = cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB)
    plt.imshow(final_img)
    plt.show()


def main():
    print("----------matching start----------")
    time_start = time.time()
    akaze = cv2.AKAZE_create()
    #akaze = cv2.ORB_create()

    # query = カメラ画像
    # map = マップ

    # gamma補正の関数
    gamma = 1.8
    gamma_cvt = np.zeros((256,1),dtype = 'uint8')
    for i in range(256):
        gamma_cvt[i][0] = 255 * (float(i)/255) ** (1.0/gamma)

    # 画像の拡大，縮小の割合(重要パラメータ)
    expand_query = 0.5
    expand_map = 2
    
    # クエリ画像を読み込んで特徴量計算
    query_img = cv2.imread(r"C:\\Users\\riko2\\OneDrive\\KFS_judgement_machine(AKAZA)\\test_logo.jpg", 0)
    query_img = cv2.LUT(query_img, gamma_cvt)
    # cv2.imwrite('./log/input_img.png', query_img)
    query_img = cv2.resize(query_img, (int(query_img.shape[1] * expand_query),
                           int(query_img.shape[0] * expand_query)))
    height_query, width_query = query_img.shape[:2]
    kp_query, des_query = akaze.detectAndCompute(query_img, None)
    # print('[time] feature calculation query: ', time.time() - time_start)

    # マップ画像を読み込んで特徴量計算
    map_img = cv2.imread(r"C:\\Users\\riko2\\OneDrive\\KFS_judgement_machine(AKAZA)\\KFS_image_list.png", 0)
    map_img = cv2.resize(map_img, (int(map_img.shape[1] * expand_map),
                            int(map_img.shape[0] * expand_map)))
    height_map, width_map = map_img.shape[:2]
    # cv2.imwrite('./log/fig/sample_img.png', sample_img)
    kp_map, des_map = akaze.detectAndCompute(map_img, None)
    # print('[time] feature calculation map: ', time.time() - time_start)

    # 特徴量マッチング実行，k近傍法
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des_query, des_map, k=2)
    print('[time] feature matching: ', time.time() - time_start)

    # マッチング精度が高いもののみ抽出
    ratio = 0.8  # 重要パラメータ
    good = []
    for m, n in matches:
        if m.distance < ratio * n.distance:
            good.append([m])

    # 対応点が１個以下なら相対関係を求められないのでNoneを返す
    if len(good) <= 1:
        print("[error] can't detect matching feature point")
        return None, None, None

    # 精度が高かったもののうちスコアが高いものから指定個取り出す
    good = sorted(good, key=lambda x: x[0].distance)
    print("valid point number: ", len(good))  # これがあまりに多すぎたり少なすぎたりする場合はパラメータを変える
    point_num = 20  # 上位何個の点をマッチングに使うか（重要パラメータ）
    if len(good) < point_num:
        point_num = len(good)  # もし20個なかったら全て使う

    # マッチング結果の描画
    result_img = cv2.drawMatchesKnn(query_img, kp_query, map_img, kp_map, good[:point_num], None, flags=0)
    img_matching = cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB)
    plt.imshow(img_matching)
    plt.show()
    print('[time] draw mid result: ', time.time() - time_start)

    #------ これ以降位置と向きの計算 -------
    query_kp = []
    map_kp = []

    # 2つの画像で対応するキーポイントを抽出
    for p in good[:point_num]:
        query_kp.append(kp_query[p[0].queryIdx])
        map_kp.append(kp_map[p[0].trainIdx])

    # 投票によって２画像の相対角度，相対比率，もっとも一致度の高い点のペアが計算される
    deg_value, size_rate, m1, m2 = vote_point(query_kp, map_kp, point_num)
    if deg_value is None:
        return None, None, None

    # print(f"calcurated deg: {deg_value}, size_rate: {size_rate}")
    # print(f"two matching point index: {m1}, {m2}")
    # print('[time] point calculation: ', time.time() - time_start)

    # クエリ画像の1点目とクエリ画像の中心の相対的な関係
    q_x1, q_y1 = query_kp[m1].pt
    m_x1, m_y1 = map_kp[m1].pt
    q_xcenter = int(width_query/2)
    q_ycenter = int(height_query/2)
    q_center_deg = math.atan2(q_ycenter - q_y1, q_xcenter - q_x1) * 180 / math.pi
    q_center_len = math.sqrt((q_xcenter - q_x1) ** 2 + (q_ycenter - q_y1) ** 2)
    #print(q_xcenter, q_ycenter, q_center_deg, q_center_len)

    # 上の関係をマップ画像上のパラメータに変換
    m_center_deg = q_center_deg - deg_value
    m_center_len = q_center_len/size_rate
    #print(t_center_deg, t_center_len)

    # 中心点のマップ画像上での位置
    m_center_rad = m_center_deg * math.pi / 180
    m_xcenter = m_x1 + m_center_len * math.cos(m_center_rad)
    m_ycenter = m_y1 + m_center_len * math.sin(m_center_rad)
    # print(m_center_rad, math.cos(m_center_rad), math.sin(m_center_rad), m_xcenter, m_ycenter)

    # 算出された値が正しい座標範囲に入っているかどうか
    if (m_xcenter < 0) or (m_xcenter > width_map):
        print("[error] invalid x value")
        return None, None, None
    if (m_ycenter < 0) or (m_ycenter > height_map):
        print("[error] invalid y value")
        return None, None, None
    if (deg_value < 0) or (deg_value > 360):
        print("[error] invalid deg value")
        return None, None, None

    x_current = int(m_xcenter/expand_map)
    y_current = int(m_ycenter/expand_map)
    drc_current = deg_value

    print('*****detection scceeded!*****')
    print('[time] final time: {:.4f} (s)'.format(time.time() - time_start))
    print("final output score-> x: {}, y: {}, drc: {:.2f}°".format(x_current, y_current, drc_current))

    # 中心点描画
    draw_final(result_img, m_xcenter, m_ycenter, deg_value, width_query)
    return x_current, y_current, drc_current


if __name__ == "__main__":
    x_current, y_current, drc_current = main()