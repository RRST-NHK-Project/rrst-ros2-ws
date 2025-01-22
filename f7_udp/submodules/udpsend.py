"""
UDPで送信するクラス
整備中！！
"""

from socket import *

class udpsend:
    def __init__(self,DstIP,DstPort):

        try:
            # ダミー接続を使ってIPアドレスを取得
            with socket(AF_INET, SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))  # Google DNSに接続 (実際には接続しない)
                ip_address = s.getsockname()[0]
        except Exception as e:
            return f"Getting IP Error: {e}"

        SrcIP = ip_address  # 送信元IP
        SrcPort = 0  # 送信元ポート番号,0にすることでポートが自動割り当てされる。これにより複数ノードでポートを使い分けることができる。
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納
        print("IP:" + str(SrcIP))

        self.DstAddr = (DstIP, DstPort)  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        try:  # 送信元アドレスでバインド
            self.udpClntSock.bind(self.SrcAddr)
        except:  # 例外処理、バインドに失敗したときはオフラインモードで開始
            print("Cannot assign requested address.\nPlease make sure connected to network.")
            exit(1)

    def send(self,data):

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

        # print(str_data)

        send_data = str_data.encode("utf-8")  # バイナリに変換

        self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信