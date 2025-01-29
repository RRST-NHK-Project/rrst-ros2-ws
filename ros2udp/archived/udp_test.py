import socket #通信用
import time

address = ('192.168.8.195', 4000) #通信を受け付けるIPアドレスと受信に使うポート番号の設定．127.0.0.1とするとPC内通信．

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #udpという名前のUDPソケット生成
udp.bind(address) #udpというソケットにaddressを紐づける

while True: #受信ループ
    try:
        rcv_byte = bytes()
        rcv_byte, addr = udp.recvfrom(1024)
        msg = rcv_byte.decode()

        # 文字列をカンマで分割してリストに格納
        fruit_list = msg.split(',')
        truncated_list = fruit_list[:6]
        
        # 全要素をfloatに変換
        float_list = [float(item) for item in truncated_list]

        # 結果を表示
        print(float_list)
        
        time.sleep(0.02)
        
        
        if msg == 'close': #受信した文字列がcloseならUDPソケットを閉じて終了
            udp.close()
            break
    except KeyboardInterrupt:#強制終了を検知したらUDPソケットを閉じて終了
        udp.close()