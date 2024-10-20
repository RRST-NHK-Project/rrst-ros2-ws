#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
F7から速度[m/s]を受信しPublish
2024/10/21
"""

online_mode = False  # ルーター未接続でデバッグする場合はFalseにする

import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

address = ("192.168.8.196", 4000)  # 自機アドレス, ポート

if online_mode == True:
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # ソケットの作成
    udp.bind(address)  # バインド

enc_msg = Float32MultiArray()
msg = "0.0,0.0,0.0,0.0,0.0,0.0"
enc_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class ENC_OBS(Node):

    def __init__(self):
        super().__init__("enc_obs")
        self.publisher_ = self.create_publisher(Float32MultiArray, "enc", 10)

        #動作の表示
        if online_mode:
            print("ENC observer started.\nIP: " + str(address[0][:]))
        else:
            print("[OFFLINE] ENC observer started.\nIP: " + str(address[0][:]))

        # FIXME:もっといい方法がある気がする。1msごとに呼び出しはやりすぎ
        freq = 0.001  # seconds

        self.timer = self.create_timer(freq, self.timer_callback)
        # self.i = 0

    def timer_callback(self):  # 並進速度をPublishするコールバック関数

        global msg

        try:
            if online_mode == True:
                rcv_byte = bytes()
                rcv_byte, addr = udp.recvfrom(64)
                msg = rcv_byte.decode()

            # 文字列をカンマで分割してリストに格納
            splited_str = msg.split(",")
            splited_str = splited_str[:6]

            # 全要素をfloatに変換
            splited_float = [float(item) for item in splited_str]
            # print(enc_msg)

            for i in range(len(splited_float)):
                enc_msg.data[i] = splited_float[i]

            # TODO:ノイズ乗ってるからPublishする前にフィルターかけたい
            self.publisher_.publish(enc_msg)

        except KeyboardInterrupt:  # 強制終了の検知
            udp.close()


def main(args=None):
    rclpy.init(args=args)
    enc_obs = ENC_OBS()
    rclpy.spin(enc_obs)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    enc_obs.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
