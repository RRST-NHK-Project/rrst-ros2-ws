#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
/encの可視化ツール
"""

# オンラインモードの設定（ルーター接続時はTrue、デバッグ時はFalse）
online_mode = True

# 必要なライブラリのインポート
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import signal
import sys
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading


class Listener(Node):
    def __init__(self):
        super().__init__("enc_plotter")
        self.subscription = self.create_subscription(
            Float32MultiArray, "enc", self.listener_callback, 10
        )

        # グラフ用のデータ初期化
        self.times = []
        self.enc_data = [[] for _ in range(6)]  # 6つのエンコーダーデータ用
        self.start_time = time.time()

        # グラフの初期化
        self.fig, self.ax = plt.subplots()
        self.lines = [self.ax.plot([], [], label=f"Encoder {i+1}")[0] for i in range(6)]
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Encoder Value")
        self.ax.set_title("Encoder Data over Time")
        self.ax.legend()
        self.ax.grid(True)  # グリッドを追加

        # アニメーションの設定
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=True)

    def listener_callback(self, enc_msg):
        current_time = time.time() - self.start_time
        self.times.append(current_time)

        # エンコーダーデータの更新（1から6番目の要素を使用）
        for i in range(6):
            if i + 1 < len(enc_msg.data):  # 1番から6番までに対応
                self.enc_data[i].append(enc_msg.data[i + 1])  # 1から6にマッピング
            else:
                self.enc_data[i].append(0)  # データがない場合は0を追加

        # データ点数を制限（例：最新の100点のみ保持）
        max_points = 100
        if len(self.times) > max_points:
            self.times = self.times[-max_points:]
            for i in range(6):
                self.enc_data[i] = self.enc_data[i][-max_points:]

    def update_plot(self, frame):
        for i, line in enumerate(self.lines):
            line.set_data(self.times, self.enc_data[i])

        self.ax.relim()
        self.ax.autoscale_view()

        # 縦軸の範囲を固定
        self.ax.set_ylim(-1.5, 1.5)

        return self.lines


def signal_handler(sig, frame):
    print("Exiting...")
    rclpy.shutdown()
    plt.close("all")
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    listener = Listener()

    # シグナルハンドラーを設定
    signal.signal(signal.SIGINT, signal_handler)

    # ROSのスピンをバックグラウンドスレッドで実行
    thread = threading.Thread(target=rclpy.spin, args=(listener,))
    thread.start()

    # Matplotlibのメインループを実行
    plt.show()

    # スレッドが終了するのを待つ
    thread.join()


if __name__ == "__main__":
    main()
