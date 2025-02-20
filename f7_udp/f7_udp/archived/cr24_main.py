#!/usr/bin/env python3
## coding: UTF-8
#:)

"""
キャチロボ2024
YOLOからの情報とGUIからの入力をもとにシューティングコンベアの位置決めを行う
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

from socket import *
import time
import math
import pyfiglet
#import serial

data = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # モタドラ用のPWMを想定
fth = 0
vth = 0
r = 0
rpm_limit = 120
sp_yaw = 0.5
sp_omni = 1.0

deadzone = 0.3  # adjust DS4 deadzone

gui_input = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
mode = 0
standby = 0

ebi_selector = 1  # えびのシュートはどこまで完了しているか
nori_selector = 1  # のりのシュートはどこまで完了しているか
yuzu_selector = 1  # ゆずのシュートはどこまで完了しているか

target = 1  # どのパックにワークをシュートするか1~6

init = False

"""
ser = serial.Serial()
ser.port = "/dev/ttyACM0"
ser.baudrate = 115200
ser.setDTR(False)
ser.open()
"""

class YOLO_Listener(Node):

    def __init__(self):
        super().__init__("Setoshio_handler")
        self.sub1 = self.create_subscription(
            Int32MultiArray, "setoshio_pub", self.yolo_callback, 10
        )
        print(pyfiglet.figlet_format("RRST"))
        print(pyfiglet.figlet_format("YOLO Mode"))
        print("Press PS to start")
        self.sub1  # prevent unused variable warning

    def yolo_callback(self, yolo_msg):  # YOLOからの入力を処理する関数

        global target
        global mode
        global ebi_selector
        global nori_selector
        global yuzu_selector

        void = yolo_msg.data[0] == -1
        ebi = yolo_msg.data[0] == 0
        nori = yolo_msg.data[0] == 1
        yuzu = yolo_msg.data[0] == 2

        if void == 1:
            #ser.write(b"0")
            if mode == 0:
                data[2] = 1
            if mode == 1:
                data[2] = 2
        else:
            data[2] = 0

        # print(data[2])

        if ebi:
            # print("ebi")
            #ser.write(b"1")
            target = ebi_selector

        if nori:
            # print("nori")
            #ser.write(b"2")
            target = nori_selector

        if yuzu:
            # print("yuzu")
            #ser.write(b"3")
            target = yuzu_selector

        r_1 = 200
        r_2 = 150

        if mode == 0:
            if target == 1:
                data[1] = -15
                data[3] = 1

            if target == 2:
                data[1] = -15
                data[3] = 2

            if target == 3:
                data[1] = 0
                data[3] = 1

            if target == 4:
                data[1] = 0
                data[3] = 2

            if target == 5:
                data[1] = 15
                data[3] = 1

            if target == 6:
                data[1] = -15
                data[3] = 2

        if mode == 1:
            if target == 1:
                data[1] = 15
                data[3] = -2

            if target == 2:
                data[1] = 15
                data[3] = -1

            if target == 3:
                data[1] = 0
                data[3] = -2

            if target == 4:
                data[1] = 0
                data[3] = -1

            if target == 5:
                data[1] = -15
                data[3] = -2

            if target == 6:
                data[1] = -15
                data[3] = -1

        # print(target)
        # print(mode)
        #print(data[1])
        # time.sleep(10)

        if init == True:
            udp.send()  # 関数実行


class GUI_Listener(Node):

    def __init__(self):
        super().__init__("gui_handler")
        self.sub2 = self.create_subscription(
            Int32MultiArray, "cr24_GUI", self.gui_callback, 10
        )
        self.sub2  # prevent unused variable warning

    def gui_callback(self, gui_msg):  # GUIからの入力を処理する関数

        global mode
        global ebi_selector
        global nori_selector
        global yuzu_selector

        mode = gui_msg.data[0]

        # GUIからの入力（１次元配列）を２次元配列に格納する
        for i in range(6):
            for j in range(3):
                gui_input[i][j] = gui_msg.data[i * 3 + j + 1]

        # print(gui_input)
        global ebi_selector
        global nori_selector
        global yuzu_selector

        for e in range(6):
            if gui_input[e][0] < 3:
                ebi_selector = e + 1
                break

        for n in range(6):
            if gui_input[n][1] < 3:
                nori_selector = n + 1
                break

        for y in range(6):
            if gui_input[y][2] < 3:
                yuzu_selector = y + 1
                break

        # print(ebi_selector, nori_selector, yuzu_selector)

        if init == True:
            udp.send()  # 関数実行


class DS4_Listener(Node):

    def __init__(self):
        super().__init__("ds4_handler")
        self.subscription = self.create_subscription(Joy, "joy", self.ds4_callback, 10)
        self.subscription  # prevent unused variable warning

    def ds4_callback(self, ps4_msg):
        """
        LS_X = (-1) * ps4_msg.axes[0]
        LS_Y = ps4_msg.axes[1]
        RS_X = (-1) * ps4_msg.axes[2]
        RS_Y = ps4_msg.axes[5]

        print(LS_X, LS_Y, RS_X, RS_Y)
        """
        CROSS = ps4_msg.buttons[1]
        CIRCLE = ps4_msg.buttons[2]
        TRIANGLE = ps4_msg.buttons[3]
        SQUARE = ps4_msg.buttons[0]

        LEFT = ps4_msg.axes[12] == 1.0
        RIGHT = ps4_msg.axes[12] == -1.0
        UP = ps4_msg.axes[13] == 1.0
        DOWN = ps4_msg.axes[13] == -1.0

        L1 = ps4_msg.buttons[4]
        R1 = ps4_msg.buttons[5]

        L2 = ps4_msg.buttons[6]
        R2 = ps4_msg.buttons[7]

        SHARE = ps4_msg.buttons[8]
        OPTION = ps4_msg.buttons[9]
        PS = ps4_msg.buttons[12]

        L3 = ps4_msg.buttons[10]
        R3 = ps4_msg.buttons[11]

        global init

        """
        if SQUARE  == 1:
            print("SAUARE ") 
        
        if CROSS == 1:
            print("CROSS") 
        
        if  CIRCLE== 1:
            print("CIRCLE") 
        
        if  TRIANGLE== 1:
            print("TRIANGLE") 
        
        if  UP == 1:
            print("UP") 
        
        if  DOWN == 1:
            print("DOWN") 
        
        if  LEFT == 1:
            print("LEFT") 
        
        if  RIGHT == 1:
            print("RIGHT") 
        
        if  L1 == 1:
            print("L1") 
        
        if  R1 == 1:
            print("R1") 
        
        if  L2 == 1:
            print("L2") 
        
        if  R2 == 1:
            print("R2") 
        
        if  L3 == 1:
            print("L3") 
        
        if  R3 == 1:
            print("R3") 
        
        if  SHARE == 1:
            print("SHARE") 
        
        if  OPTION == 1:
            print("OPTION") 
        
        if  PS == 1:
            print("PS") 
        """

        if PS:
            init = True
            print(pyfiglet.figlet_format("Start"))
            time.sleep(0.5)


class udpsend:
    def __init__(self):

        # SrcIP = "192.168.128.182"  # 送信元IP 家
        # SrcIP = "192.168.2.130"  # 送信元IP 家2
        SrcIP = "192.168.8.195"  # 送信元IP SFT1200
        SrcPort = 4000  # 送信元ポート番号
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納

        DstIP = "192.168.8.215"  # 宛先IP
        DstPort = 5000  # 宛先ポート番号
        self.DstAddr = (DstIP, DstPort)  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        self.udpClntSock.bind(self.SrcAddr)  # 送信元アドレスでバインド

    def send(self):

        # print(data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8])

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
        # str_data = (str(data[1])+str(data[2])+str(data[3])+str(data[4])+str(data[5]))
        send_data = str_data.encode("utf-8")  # バイナリに変換
        # binary = data.to_bytes(4,'big')
        
        #print(data[1])

        self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信

        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        data[5] = 0
        data[6] = 0
        data[7] = 0
        data[8] = 0
        


udp = udpsend()  # クラス呼び出し


def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()

    yolo_listener = YOLO_Listener()
    gui_listener = GUI_Listener()
    ds4_listener = DS4_Listener()

    exec.add_node(yolo_listener)
    exec.add_node(gui_listener)
    exec.add_node(ds4_listener)

    exec.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_listener.destroy_node()
    gui_listener.destroy_node()
    ds4_listener.destroy_node()
    exec.shutdown()
    #ser.close


if __name__ == "__main__":
    main()
