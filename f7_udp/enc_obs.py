#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
F7から各エンコーダーの速度[m/s]を受信しPublish
2024/10/21
"""

# オンラインモードの設定（ルーター接続時はTrue、デバッグ時はFalse）
online_mode = True  

# 必要なライブラリとモジュールのインポート
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# UDP通信の設定
address = ("192.168.8.196", 4000)  # 自機アドレス, ポート

# オンラインモードの場合、UDPソケットを初期化
if online_mode == True:
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # ソケットの作成
    udp.bind(address)  # バインド

# グローバル変数の初期化
enc_msg = Float32MultiArray()
msg = "0.0,0.0,0.0,0.0,0.0,0.0"
enc_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ROS2ノードクラスの定義
class ENC_OBS(Node):

    def __init__(self):
        super().__init__("enc_obs")
        # パブリッシャーの作成
        self.publisher_ = self.create_publisher(Float32MultiArray, "enc", 10)

        # 起動メッセージの表示
        if online_mode:
            print("ENC observer started.\nIP: " + str(address[0][:]))
        else:
            print("[OFFLINE] ENC observer started.\nIP: " + str(address[0][:]))

        # タイマーコールバックの設定
        # FIXME: 1msごとの呼び出しはやりすぎかも
        freq = 0.001  # seconds
        self.timer = self.create_timer(freq, self.timer_callback)

    def timer_callback(self):
        """各エンコーダーの速度をPublishするコールバック関数"""
        global msg

        try:
            if online_mode == True:
                # UDPからデータを受信
                rcv_byte = bytes()
                rcv_byte, addr = udp.recvfrom(64)  # 最大64バイトのデータを受信
                msg = rcv_byte.decode()  # バイト列を文字列にデコード

            # 受信したメッセージを処理
            # 文字列をカンマで分割してリストに格納（最初の6要素のみ使用）
            splited_str = msg.split(",")
            splited_str = splited_str[:7]

            # 全要素をfloatに変換
            splited_float = [float(item) for item in splited_str]

            # 変換した浮動小数点数をenc_msgのdataにコピー
            for i in range(len(splited_float)):
                enc_msg.data[i] = splited_float[i]

            # TODO: ノイズ除去のためのフィルタリングを実装する
            # 現在のデータをそのままPublish
            self.publisher_.publish(enc_msg)

        except KeyboardInterrupt:  # Ctrl+Cなどによる強制終了の検知
            udp.close()  # UDPソケットを閉じる


# メイン関数
def main(args=None):
    rclpy.init(args=args)  # ROS2の初期化
    enc_obs = ENC_OBS()  # ノードのインスタンス化
    rclpy.spin(enc_obs)  # ノードの実行（コールバック待機）

    # ノードの明示的な破棄
    # （オプション - ガベージコレクタがノードオブジェクトを破棄する際に自動的に行われる）
    enc_obs.destroy_node()
    rclpy.shutdown()  # ROS2のシャットダウン


# スクリプトが直接実行された場合にmain関数を呼び出す
if __name__ == "__main__":
    main()
