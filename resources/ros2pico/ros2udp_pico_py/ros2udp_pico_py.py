from machine import Pin
from utime import sleep
import network
import usocket

SSID = "GL-SFT1200-288-2G-LNX"
PASSWORD = "lnxmaster"
SRC_IP = "192.168.8.219"
SRC_PORT = 5000
SUBNET_MASK = "255.255.255.0"
GATEWAY = "192.168.8.1"
DNS = "192.168.8.1"

data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

led = Pin("LED", Pin.OUT)  # LED

wlan = network.WLAN(network.STA_IF)  # wlan作成（無線LAN）
wlan.active(True)  # 起動
wlan.ifconfig((SRC_IP, SUBNET_MASK, GATEWAY, DNS))
wlan.connect(SSID, PASSWORD)  # アクセスポイントに接続
while not wlan.isconnected():  # 接続待ち
    print(".", end="")
    led.toggle()
    sleep(0.1)
print("\nConnected.\nIP:" + wlan.ifconfig()[0])

sock = usocket.socket(usocket.AF_INET, usocket.SOCK_DGRAM)  # ソケット作成
sock.bind((SRC_IP, SRC_PORT))  # バインド

try:
    while True:
        # データを受信
        buffer, addr = sock.recvfrom(128) 
        buffer = buffer.decode("utf-8")
        str_data = buffer.split(",")
        data = list(map(int,str_data))
        led.toggle()
        print(data)

except KeyboardInterrupt:
    print("Server stopped")
finally:
    sock.close()
