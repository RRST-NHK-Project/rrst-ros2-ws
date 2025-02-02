#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
2024/10/23
"""

# オンラインモードの設定（ルーター接続時はTrue、デバッグ時はFalse）
online_mode = True

# 必要なライブラリのインポート
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from rclpy.executors import SingleThreadedExecutor

from socket import *
import time
import math

# 以下pipでのインストールが必要
import pyfiglet
from simple_pid import PID

# モーター速度データの初期化 (0%-100%)
data = [0, 0, 0, 0, 0, 0, 0, 0, 0]

# オムニホイールパラメーター
sp_yaw = 5  # ヨー回転duty比[%]
duty_limit = 10  # duty比の上限[%]
v_limit = 3.0  # 並進速度の上限[m/s]

rad_target = 0.0
rad_actual = 0.0

target = [0, 0, 0, 0, 0, 0]
actual = [0, 0, 0, 0, 0, 0]

Output = [0, 0, 0, 0, 0, 0]
Error = [0, 0, 0, 0, 0, 0]
last_Error = [0, 0, 0, 0, 0, 0]
Integral = [0, 0, 0, 0, 0, 0]
Differential = [0, 0, 0, 0, 0, 0]

rad_Output = 0.0


PID_control_period = 0.01  # PID制御周期 [s]


# ジョイスティックのデッドゾーン
deadzone = 0.3


class PS4_Listener(Node):
    def __init__(self):
        super().__init__("nhk25_omni_driver")
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )

        self.timer = self.create_timer(PID_control_period, self.PID)

        # プログラム開始メッセージの表示
        print(pyfiglet.figlet_format("NHK2025"))
        if not online_mode:
            print("[OFFLINE] Omni driver started.")

    def listener_callback(self, ps4_msg):

        global rad_target
        global rad_actual
        global target
        global actual

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

        if PS:
            print(pyfiglet.figlet_format("HALT"))
            data[1] = 0
            data[2] = 0
            data[3] = 0
            data[4] = 0
            data[5] = 0
            data[6] = 0
            data[7] = 0
            data[8] = 0
            if online_mode == True:
                udp.send()  # 関数実行
            time.sleep(1)
            while True:
                pass

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
        # rad_target = math.atan2(LS_Y, LS_X)

        target[0] = LS_X
        target[1] = LS_Y

        # 各ホイールの速度計算
        if RS_X <= deadzone and RS_Y <= deadzone and R2 == 0 and L2 == 0:
            v1 = math.sin(rad_Output - math.pi / 4) * duty_limit
            v2 = math.sin(rad_Output - 3 * math.pi / 4) * duty_limit
            v3 = math.sin(rad_Output - 5 * math.pi / 4) * duty_limit
            v4 = math.sin(rad_Output - 7 * math.pi / 4) * duty_limit

        # 右スティックによる回転
        if RS_X >= deadzone:
            R2 = 1
        if RS_X <= -1 * deadzone:
            L2 = 1

        # 回転動作
        if R2 == 1:
            v1 = v2 = v3 = v4 = -1.0 * sp_yaw
        if L2 == 1:
            v1 = v2 = v3 = v4 = 1.0 * sp_yaw

        # 停止条件
        if (
            (math.fabs(LS_X) <= deadzone)
            and (math.fabs(LS_Y) <= deadzone)
            and R2 == 0
            and L2 == 0
        ):
            v1 = v2 = v3 = v4 = 0

        # 速度データの設定
        data[1] = v1
        data[2] = v2
        data[3] = v3
        data[4] = v4

        # オンラインモードの場合、UDPで送信
        if online_mode:
            udp.send()

    def PID(self):

        global target
        global actual
        global Output
        global Error
        global last_Error
        global Integral
        global Differential
        global PID_control_period
        global v_limit
        global rad_Output

        Kp = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        Ki = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        Kd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for i in range(6):
            Error[i] = target[i] - (actual[i] / v_limit)
            Integral[i] += Error[i] * PID_control_period
            Differential[i] = (Error[i] - last_Error[i]) / PID_control_period

            Output[i] = (Kp[i] * Error[i]) + (Ki[i] * Integral[i]) + (Kd[i] * Differential[i])

            last_Error[i] = Error[i]

        rad_Output = math.atan2(Output[1], Output[0]) * math.sqrt(Output[0] ** 2 + Output[1] ** 2)
        print(rad_Output)


class ENC_Listener(Node):
    def __init__(self):
        super().__init__("enc_handler")
        self.subscription = self.create_subscription(
            Float32MultiArray, "enc", self.listener_callback, 10
        )

    def listener_callback(self, enc_msg):

        global rad_actual
        global rad_target
        global target
        global actual

        actual[0] = enc_msg.data[1]
        actual[1] = enc_msg.data[2]

        # print(rad_actual)


class udpsend:
    def __init__(self):
        # UDP通信の設定
        SrcIP = "192.168.8.195"  # 送信元IP
        SrcPort = 4000  # 送信元ポート
        self.SrcAddr = (SrcIP, SrcPort)

        DstIP = "192.168.8.215"  # 宛先IP
        DstPort = 5000  # 宛先ポート
        self.DstAddr = (DstIP, DstPort)

        # UDPソケットの作成とバインド
        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)
        self.udpClntSock.bind(self.SrcAddr)

        print(f"Omni driver started.\nSrcIP: {SrcIP}\nDstIP: {DstIP}")

    def send(self):

        # print(data[1], data[2], data[3], data[4])

        str_data = (
            str(data[1])
            + ","
            + str(data[2])
            + ","
            + str(data[3])
            + ","
            + str(data[4])
            + ","
            + str(data[5])
            + ","
            + str(data[6])
            + ","
            + str(data[7])
            + ","
            + str(data[8])
        )  # パケットを作成

        #print(str_data)

        send_data = str_data.encode("utf-8")  # バイナリに変換

        self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信

        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        data[5] = 0
        data[6] = 0
        data[7] = 0
        data[8] = 0


# オンラインモード時のUDPインスタンス作成
if online_mode:
    udp = udpsend()


def main(args=None):
    # ROSの初期化
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()

    # Listenerノードの作成と実行
    ps4_listener = PS4_Listener()
    enc_listener = ENC_Listener()

    exec.add_node(ps4_listener)
    exec.add_node(enc_listener)

    exec.spin()

    # ノードの終了処理
    ps4_listener.destroy_node()
    enc_listener.destroy_node()
    exec.shutdown()


if __name__ == "__main__":
    main()
