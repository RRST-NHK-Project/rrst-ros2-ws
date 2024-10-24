#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
2024/10/21
"""

# 必要なライブラリのインポート
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from rclpy.executors import SingleThreadedExecutor
import math
import signal
import matplotlib.pyplot as plt
import numpy as np

# グローバル変数
rad_target = 0.0
rad_actual = 0.0
running = True

# プロット用の設定
fig, ax = plt.subplots()
target_arrow = None
actual_arrow = None


def update_plot():
    global target_arrow, actual_arrow
    if target_arrow:
        target_arrow.remove()
    if actual_arrow:
        actual_arrow.remove()

    # rad_target の矢印（赤色）
    target_arrow = ax.arrow(
        0,
        0,
        np.cos(rad_target),
        np.sin(rad_target),
        color="red",
        width=0.02,
        head_width=0.1,
        head_length=0.1,
        length_includes_head=True,
    )

    # rad_actual の矢印（青色）
    actual_arrow = ax.arrow(
        0,
        0,
        np.cos(rad_actual),
        np.sin(rad_actual),
        color="blue",
        width=0.02,
        head_width=0.1,
        head_length=0.1,
        length_includes_head=True,
    )

    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_aspect("equal")
    ax.set_title("Direction Visualization")
    ax.legend(["Target", "Actual"])

    plt.draw()
    plt.pause(0.1)


class PS4_Listener(Node):
    def __init__(self):
        super().__init__("dir_plotter_ps4_listener")
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )

    def listener_callback(self, ps4_msg):
        global rad_target

        # ジョイスティックの入力値を取得
        LS_X = -1 * ps4_msg.axes[0]
        LS_Y = ps4_msg.axes[1]
        RS_X = (-1) * ps4_msg.axes[2]
        RS_Y = ps4_msg.axes[5]

        # ボタン入力の取得
        CROSS = ps4_msg.buttons[1]
        CIRCLE = ps4_msg.buttons[2]
        TRIANGLE = ps4_msg.buttons[3]
        SQUARE = ps4_msg.buttons[0]

        # 方向キーの入力取得
        LEFT = ps4_msg.axes[12] == 1.0
        RIGHT = ps4_msg.axes[12] == -1.0
        UP = ps4_msg.axes[13] == 1.0
        DOWN = ps4_msg.axes[13] == -1.0

        # その他のボタン入力
        L1 = ps4_msg.buttons[4]
        R1 = ps4_msg.buttons[5]
        L2 = ps4_msg.buttons[6]
        R2 = ps4_msg.buttons[7]
        SHARE = ps4_msg.buttons[8]
        OPTION = ps4_msg.buttons[9]
        PS = ps4_msg.buttons[12]
        L3 = ps4_msg.buttons[10]
        R3 = ps4_msg.buttons[11]

        # 方向キー入力の処理
        if UP == 1:
            LS_Y = 1.0
        if DOWN == 1:
            LS_Y = -1.0
        if LEFT == 1:
            LS_X = -1.0
        if RIGHT == 1:
            LS_X = 1.0

        # 移動方向の計算
        rad_target = math.atan2(LS_Y, LS_X)
        # self.get_node().get_logger().info(f'Target angle: {rad_target}')

        # プロットの更新
        update_plot()


class ENC_Listener(Node):
    def __init__(self):
        super().__init__("dir_plotter_enc_handler")
        self.subscription = self.create_subscription(
            Float32MultiArray, "enc", self.listener_callback, 10
        )

    def listener_callback(self, enc_msg):
        global rad_actual

        Vx = enc_msg.data[1]
        Vy = enc_msg.data[2]

        rad_actual = math.atan2(Vy, Vx)
        # プロットの更新
        update_plot()


def signal_handler(sig, frame):
    global running
    print("\nCtrl+C pressed. Stopping...")
    running = False


def main(args=None):
    global running

    # Ctrl+C のハンドラを設定
    signal.signal(signal.SIGINT, signal_handler)

    # ROSの初期化
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()

    # Listenerノードの作成と実行
    ps4_listener = PS4_Listener()
    enc_listener = ENC_Listener()

    exec.add_node(ps4_listener)
    exec.add_node(enc_listener)

    try:
        while running:
            exec.spin_once(timeout_sec=0.1)
    finally:
        running = False
        ps4_listener.destroy_node()
        enc_listener.destroy_node()
        exec.shutdown()
        plt.close()


if __name__ == "__main__":
    plt.ion()  # インタラクティブモードを有効にする
    main()
