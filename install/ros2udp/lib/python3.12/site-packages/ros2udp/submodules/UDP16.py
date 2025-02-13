"""
UDPで送信するクラスの拡張版
"""

from socket import *

class UDP16:
    def __init__(self,DstIP,DstPort):

        try:
            # ダミー接続を使ってIPアドレスを取得
            with socket(AF_INET, SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))  # Google DNSに接続 (実際には接続しない)
                ip_address = s.getsockname()[0] #IPアドレスを取得
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
            print("Cannot assign requested address.\nPlease check your network.")
            exit(1)

    def send(self,data):

        str_data = ",".join(str(int(data[i])) for i in range(0, 17))  # パケットを作成

        print(str_data)

        send_data = str_data.encode("utf-8")  # バイナリに変換

        self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信