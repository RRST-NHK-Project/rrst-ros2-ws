from machine import Pin
import utime
import network
import usocket
import struct
import _thread

# ===== Wi-Fi設定 =====
SSID = "TP-Link_7036"
PASSWORD = "19861050"
SRC_IP = "192.168.0.197"
SUBNET_MASK = "255.255.255.0"
GATEWAY = "192.168.0.1"
DNS = "192.168.0.1"

DST_IP = "192.168.0.100"
DST_PORT = 4000

# ===== PPR（エンコーダのパルス数） =====
PPR = 192

# ===== LED設定 =====
led = Pin("LED", Pin.OUT)

# ===== エンコーダピン（A, B）を配列で管理 =====
encoder_pins = [
    (Pin(14, Pin.IN, Pin.PULL_UP), Pin(15, Pin.IN, Pin.PULL_UP)),
    (Pin(16, Pin.IN, Pin.PULL_UP), Pin(17, Pin.IN, Pin.PULL_UP)),
    (Pin(18, Pin.IN, Pin.PULL_UP), Pin(19, Pin.IN, Pin.PULL_UP)),
    (Pin(20, Pin.IN, Pin.PULL_UP), Pin(21, Pin.IN, Pin.PULL_UP)),
]

# ===== 状態管理 =====
positions = [0, 0, 0, 0]
last_states = [(a.value(), b.value()) for a, b in encoder_pins]
lock = _thread.allocate_lock()


# ===== Core1: エンコーダ読み取りループ =====
def encoder_loop():
    global positions, last_states
    while True:
        lock.acquire()
        for i in range(4):
            a_val = encoder_pins[i][0].value()
            b_val = encoder_pins[i][1].value()
            prev_a, prev_b = last_states[i]

            if (a_val, b_val) != (prev_a, prev_b):
                if (prev_a, prev_b) == (0, 0):
                    if (a_val, b_val) == (1, 0):
                        positions[i] += 1
                    elif (a_val, b_val) == (0, 1):
                        positions[i] -= 1
                elif (prev_a, prev_b) == (1, 0):
                    if (a_val, b_val) == (1, 1):
                        positions[i] += 1
                    elif (a_val, b_val) == (0, 0):
                        positions[i] -= 1
                elif (prev_a, prev_b) == (1, 1):
                    if (a_val, b_val) == (0, 1):
                        positions[i] += 1
                    elif (a_val, b_val) == (1, 0):
                        positions[i] -= 1
                elif (prev_a, prev_b) == (0, 1):
                    if (a_val, b_val) == (0, 0):
                        positions[i] += 1
                    elif (a_val, b_val) == (1, 1):
                        positions[i] -= 1

                last_states[i] = (a_val, b_val)
        lock.release()
        # utime.sleep_us(1)


# ===== Core1起動 =====
_thread.start_new_thread(encoder_loop, ())

# ===== Wi-Fi接続 =====
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.ifconfig((SRC_IP, SUBNET_MASK, GATEWAY, DNS))
wlan.connect(SSID, PASSWORD)

while not wlan.isconnected():
    print(".", end="")
    led.toggle()
    utime.sleep(0.1)

print("\nConnected. IP:", wlan.ifconfig()[0])

# ===== UDPソケット作成 =====
sock = usocket.socket(usocket.AF_INET, usocket.SOCK_DGRAM)

# ===== Core0: 送信ループ =====
try:
    while True:
        lock.acquire()
        angles = [(pos / PPR) * 360.0 for pos in positions]
        lock.release()

        data = struct.pack("ffff", *angles)
        sock.sendto(data, (DST_IP, DST_PORT))
        print(f"Sent: {angles}")

        led.toggle()
        utime.sleep(0.01)

except KeyboardInterrupt:
    print("Stopped")
finally:
    sock.close()
