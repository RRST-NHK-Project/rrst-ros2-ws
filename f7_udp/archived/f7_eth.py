from socket import *
import time

## UDP送信クラス
class udpsend():
    def __init__(self):

        SrcIP = "192.168.8.205"                             # 送信元IP
        SrcPort = 4000                               # 送信元ポート番号
        self.SrcAddr = (SrcIP,SrcPort)                  # アドレスをtupleに格納

        DstIP = "192.168.8.215"                             # 宛先IP
        DstPort = 5000                            # 宛先ポート番号
        self.DstAddr = (DstIP,DstPort)                  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        self.udpClntSock.bind(self.SrcAddr)             # 送信元アドレスでバインド

    def send(self):

        data = [0,0,0,0,0,0] #モタドラ用のPWMを想定
        data[1] = 100
        data[2] = 200
        data[3] = 300
        data[4] = 400
        data[5] = 500
        
        str_data = (str(data[1])+','+str(data[2])+','+str(data[3])+','+str(data[4])+','+str(data[5])) #パケットを作成
        #str_data = (str(data[1])+str(data[2])+str(data[3])+str(data[4])+str(data[5]))
        send_data = str_data.encode('utf-8')                     # バイナリに変換
        #binary = data.to_bytes(4,'big')

        self.udpClntSock.sendto(send_data, self.DstAddr)     # 宛先アドレスに送信

udp = udpsend()     # クラス呼び出し
while 1:
    udp.send()          # 関数実行
    time.sleep(0.01)