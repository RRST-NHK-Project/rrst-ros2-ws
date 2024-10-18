#!/usr/bin/env python3
## coding: UTF-8

#F7から回転数を受信しPublish
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

online_mode = False  #ルーター未接続でデバッグする場合はFalseにする

address = ('192.168.8.195', 4000) #自機アドレス

if online_mode == True:
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #ソケットの作成
    udp.bind(address)#バインド

enc_msg = Float64MultiArray()
msg = "0.0,0.0,0.0,0.0,0.0,0.0"
enc_msg.data = [0.0,0.0,0.0,0.0,0.0,0.0]

class ENC_OBS(Node):

    def __init__(self):
        super().__init__("enc_obs")
        self.publisher_ = self.create_publisher(Float64MultiArray, "enc", 10)
        freq = 0.001  # seconds
        self.timer = self.create_timer(freq, self.timer_callback)
        # self.i = 0

    def timer_callback(self):  # callback for publishing RPM

        global msg

        try:
            if online_mode == True:
                rcv_byte = bytes()
                rcv_byte, addr = udp.recvfrom(1024)
                msg = rcv_byte.decode()

            # 文字列をカンマで分割してリストに格納
            splited_str = msg.split(',')
            splited_str = splited_str[:6]
            
            # 全要素をfloatに変換
            splited_float = [float(item) for item in splited_str]
            #print(enc_msg)

            for i in range(len(splited_float)):
                enc_msg.data[i] = splited_float[i]

            self.publisher_.publish(enc_msg)
            
        except KeyboardInterrupt:#強制終了の検知
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
