#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ver2.2変更点：boxである可能性を検出したときのみにAKAZAマッチングを行うように変更、
# 追加予定:pdfごとの判定、自動制御のための布石関数
# ver0系統：アフィン変換のみの特徴量検出,ver1系統:立体化対応・modelによる背景判定,ver2系統:リアルタイム対応
# アフィン変換だけだと不十分なので、ホモグラフィ変換もやります。
# リーダブルコードを心がけていますが、変数名や関数名が分かりにくい場合は遠慮なく質問してください。

import time
import cv2
import math
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
import os
import torch
import torch.nn as nn
from torchvision import models, transforms
import rclpy
from std_msgs.msg import Int32MultiArray

# CNN用の前処理
preprocess = transforms.Compose(
    [
        transforms.Resize(224),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ]
)


# modelの読み込み
def load_ksf_model(model_path):
    model = models.resnet18(weights=None)

    in_features = model.fc.in_features
    output_classes = 3
    model.fc = nn.Linear(in_features, output_classes)

    device = torch.device("cpu")
    state_dict = torch.load(model_path, map_location=device)
    model.load_state_dict(state_dict)

    model.eval()

    return model


def prepare_map_data(list_path, akaze):
    """マップ画像を読み込み、特徴量を抽出して返す関数"""
    if not os.path.exists(list_path):
        return None, None, None, None
        
    map_img_orig = cv2.imread(list_path, 0)
    map_img = cv2.resize(map_img_orig, (map_img_orig.shape[1], map_img_orig.shape[0]))
    kp_map, des_map = akaze.detectAndCompute(map_img, None)
    h_map, w_map = map_img.shape[:2]
    
    return kp_map, des_map, h_map, w_map


# 上位のキーポイントの相対関係を全て調べて多数決を取ることでノイズに強くする
def vote_point(query_kp, map_kp, point_num, dif_range = 0.05,size_range_min=0.3, size_range_max=3.0
               #パラメータ調整用の引数を追加
               ):


    # 点i, jの相対角度と相対長さを格納する配列
    deg_cand = np.zeros((point_num, point_num))
    len_cand = np.zeros((point_num, point_num))

    # 全ての点のサイズ比，相対角度を求める
    for i in range(point_num):
        for j in range(i + 1, point_num):
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

            # print(q_x1, q_y1, q_x2, q_y2, q_deg, q_len)
            # print(m_x1, m_y1, m_x2, m_y2, m_deg, m_len)

            # 2つの画像の相対角度と距離
            deg_value = q_deg - m_deg
            if deg_value < 0:
                deg_value += 360
            if m_len <= 0:
                continue
            size_rate = q_len / m_len

            deg_cand[i][j] = deg_value
            deg_cand[j][i] = deg_value
            len_cand[i][j] = size_rate
            len_cand[j][i] = size_rate

    # print(deg_cand)
    # print(len_cand)

    # 多数決を取る
    # ある点iについて，j, kとの相対関係が一致するかを各jについて調べる
    cand_count = np.zeros((point_num, point_num))

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
                if (
                    deg_dif <= deg_cand[i][j] * dif_range
                    and size_dif <= len_cand[i][j] * dif_range
                ):
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


def detect_on_warped(warped_img, query_img, akaze):

    # 平面化された画像(warped_img)に対して、テンプレート(query_img)がどれくらい一致するかを再検証する
    if warped_img is None:
        return 0

    # warped_imgから特徴量を抽出
    kp_w, des_w = akaze.detectAndCompute(warped_img, None)
    kp_q, des_q = akaze.detectAndCompute(query_img, None)

    if des_w is None or des_q is None:
        return 0

    # マッチング実行
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des_q, des_w, k=2)

    # 精度チェック
    good_w = [m for m in matches if m.distance < 100]
    return len(good_w)


# 最終的な描画関数
def draw_final(result_img, m_xcenter, m_ycenter, deg_value, width_query):
    # 中心点の描画
    cv2.circle(
        result_img,
        (int(m_xcenter) + width_query, int(m_ycenter)),
        20,
        color=(0, 0, 255),
        thickness=-1,
    )

    # 向きの計算，矢印描画
    deg_front = -deg_value * math.pi / 180 - math.pi / 2
    q_xfront = m_xcenter + 200 * math.cos(deg_front)
    q_yfront = m_ycenter + 200 * math.sin(deg_front)
    cv2.arrowedLine(
        result_img,
        (int(m_xcenter) + width_query, int(m_ycenter)),
        (int(q_xfront) + width_query, int(q_yfront)),
        color=(255, 0, 0),
        thickness=15,
    )

    final_img = cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB)
    plt.imshow(final_img)
    plt.show()


def main(self):
    print("realtime matching start")
    from ament_index_python.packages import get_package_share_directory

    # 初期設定（ループの外で1回だけ実行) ←←←←←重要パラメーター慎重に扱うように！！！！
    akaze = cv2.AKAZE_create()
    expand_query = 0.5  # 初期：0.5
    expand_map = 1.0  # 初期：1.0
    gamma = 1.8  # 初期：1.8
    gamma_cvt = np.zeros((256, 1), dtype="uint8")
    for i in range(256):
        gamma_cvt[i][0] = 255 * (float(i) / 255) ** (1.0 / gamma)

    ratio = 0.87  # 初期：0.85
    point_num_limit = 75  # 初期：100

    # モデルとマップのロード
    base_path = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(
        base_path, "..", "resource", "KFS_judgement_machine ver1.1.pth"
    )
    cnn_model = load_ksf_model(model_path)

    list_path = os.path.join(base_path, "..", "resource", "KFS_image_list.png")
    map_img_orig = cv2.imread(list_path, 0)
    map_img = cv2.resize(
        map_img_orig,
        (
            int(map_img_orig.shape[1] * expand_map),
            int(map_img_orig.shape[0] * expand_map),
        ),
    )
    kp_map, des_map = akaze.detectAndCompute(map_img, None)
    height_map, width_map = map_img.shape[:2]

    # カメラ取得
    cap = cv2.VideoCapture(0)

    # カメラが映っている限り判定を継続
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        time_frame_start = time.time()
        camera_img_orig = frame.copy()

        # 切り抜きとモノクロ化
        query_roi, offset = get_box_roi(camera_img_orig, cnn_model)
        query_img = cv2.cvtColor(query_roi, cv2.COLOR_BGR2GRAY)

        # ガンマ補正
        query_img = cv2.LUT(query_img, gamma_cvt)

        # リサイズ
        query_img = cv2.resize(
            query_img,
            (
                int(query_img.shape[1] * expand_query),
                int(query_img.shape[0] * expand_query),
            ),
        )

        # クエリ側の特徴量検出
        height_query, width_query = query_img.shape[:2]
        kp_query, des_query = akaze.detectAndCompute(query_img, None)

        if des_query is not None:
            # 特徴量マッチング
            bf = cv2.BFMatcher()
            matches = bf.knnMatch(des_query, des_map, k=2)

            good = []
            for m, n in matches:
                if m.distance < ratio * n.distance:
                    good.append([m])

            if len(good) > 1:
                good = sorted(good, key=lambda x: x[0].distance)
                p_num = min(len(good), point_num_limit)

                def check_homography_quality(dst):
                    # 四隅の座標を取得
                    p1, p2, p3, p4 = dst.reshape(4, 2)
                    # 1. 面積の計算（小さすぎたらノイズ）
                    area = cv2.contourArea(dst)
                    if area < 5000:
                        return False  # 5000画素以下は無視
                    # 2. 凸性のチェック（四角形がねじれていないか）
                    if not cv2.isContourConvex(np.int32(dst)):
                        return False
                    # 3. アスペクト比の確認（細長すぎたら誤検知）
                    # (辺の長さの比が極端でないか等をチェック)
                    return True

                # マッチング結果の描画
                result_img = cv2.drawMatchesKnn(
                    query_img, kp_query, map_img, kp_map, good[:p_num], None, flags=0
                )

                # ホモグラフィ行列の計算
                if len(good) >= 4:
                    src_pts = np.float32(
                        [kp_query[m[0].queryIdx].pt for m in good]
                    ).reshape(-1, 1, 2)
                    dst_pts = np.float32(
                        [kp_map[m[0].trainIdx].pt for m in good]
                    ).reshape(-1, 1, 2)
                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

                    if M is not None:
                        h, w = query_img.shape
                        pts = np.float32(
                            [[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]
                        ).reshape(-1, 1, 2)
                        dst = cv2.perspectiveTransform(pts, M)
                        dst_for_draw = dst + [width_query, 0]
                        cv2.polylines(
                            result_img,
                            [np.int32(dst_for_draw)],
                            True,
                            (0, 0, 255),
                            10,
                            cv2.LINE_AA,
                        )

                # 投票システムによる座標特定
                query_kp_list = [kp_query[p[0].queryIdx] for p in good[:p_num]]
                map_kp_list = [kp_map[p[0].trainIdx] for p in good[:p_num]]
                deg_value, size_rate, m1, m2 = vote_point(
                    query_kp_list, map_kp_list, p_num
                )

                if deg_value is not None:
                    # 中心点計算
                    q_x1, q_y1 = query_kp_list[m1].pt  # クエリ側の基準座標
                    m_x1, m_y1 = map_kp_list[m1].pt  # マップ側の基準座標
                    q_xcenter, q_ycenter = int(width_query / 2), int(
                        height_query / 2
                    )  # クエリ画像の中心座標
                    q_center_deg = (
                        math.atan2(q_ycenter - q_y1, q_xcenter - q_x1) * 180 / math.pi
                    )  # 基準点から中心点への角度
                    q_center_len = math.sqrt(
                        (q_xcenter - q_x1) ** 2 + (q_ycenter - q_y1) ** 2
                    )  # 基準点から中心点への距離
                    m_center_deg = (
                        q_center_deg - deg_value
                    )  # マップ側での中心点への角度
                    m_center_len = (
                        q_center_len / size_rate
                    )  # マップ側での中心点への距離
                    m_center_rad = m_center_deg * math.pi / 180  # ラジアン変換
                    m_xcenter = m_x1 + m_center_len * math.cos(m_center_rad)
                    m_ycenter = m_y1 + m_center_len * math.sin(m_center_rad)

                    # マップの範囲内（0〜width, 0〜height）に収まっているかチェック
                    if (0 <= m_xcenter <= width_map) and (0 <= m_ycenter <= height_map):

                        is_true = 1 if m_ycenter < (self.height_map / 2) else 0

                        # [x, y, 真偽値] の3要素でメッセージを作成
                        msg = Int32MultiArray()
                        msg.data = [int(m_xcenter), int(m_ycenter), 1]
                        self.publisher_.publish(msg)

                        # コンソールで確認用
                        self.get_logger().info(
                            f"Published: x={int(m_xcenter)}, y={int(m_ycenter)}, TF={is_true}"
                        )

                    if (0 <= m_xcenter <= width_map) and (0 <= m_ycenter <= height_map):
                        print(
                            f"Match! x: {int(m_xcenter)}, y: {int(m_ycenter)}, deg: {deg_value:.2f}"
                        )

                # リアルタイム表示
                cv2.imshow("KFS Realtime Matcher", result_img)

        # オリジナル映像の表示
        cv2.imshow("Webcam Feed", frame)

        # (補足：ビット演算によるマスク処理という名のおまじない)
        key = cv2.waitKey(1) & 0xFF
        # ASCIIコード13がEnterキー
        if key == 13:
            print("Enterキーが押されました。プログラムを終了します。")
            break

    cap.release()
    cv2.destroyAllWindows()

    return 0, 0, 0


def start_node(args=None):
    import rclpy
    from rclpy.node import Node

    if not rclpy.ok():
        rclpy.init(args=args)

    try:
        node = Node("matcher_node")

        from std_msgs.msg import Int32MultiArray

        node.publisher_ = node.create_publisher(Int32MultiArray, "/KFS_judge", 10)

        node.width_map = 0
        node.height_map = 0

        main(node)

    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    start_node() #直接起動時のみカメラが起動