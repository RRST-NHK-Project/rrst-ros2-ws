#!/usr/bin/env python3
"""ESP32とDamiaoモータの「ふり」をするスクリプト(実機なしで自分のプログラムを試す道具)。

ros2can と実機の代わりに、serial_tx_<ID> を受け取り serial_rx_<ID> を返す。
damiao_driver_node・ps4_teleop_node・自分のプログラムは、実機の時と同じように動かせる。

使い方(ターミナル1で。ros2canは起動しないこと):
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && source ~/ros2_ws/install/setup.bash
  ros2 run damiao_ctrl fake_esp32.py                  # DEVICE_ID 201、モータ2台
  ros2 run damiao_ctrl fake_esp32.py --motors 4       # 4台
  ros2 run damiao_ctrl fake_esp32.py --device-id 204  # 2枚目のESP32のふり

まねしていること(本物のESP32ファームウェア damiao.cpp と同じ決まり):
  ・データ枠(1台6個)の意味と単位
  ・有効化に約0.25秒かかる(脱力→エラー解除→設定書き込み→有効化)
  ・PCからの指令が0.3秒途絶えたら全台脱力
  ・角度制御は、モータ内部の位置が±12rad以内の時だけ有効になる
  ・位置の返事は±12.5radで一周する
まねしていないこと: 故障・通信喪失(13)、重さや摩擦、ゆっくり回した時の位置の精度の問題。
本物とは動きの細かいところが違うので、最後は必ず実機で確かめること。
"""
import argparse
import math
import time

import rclpy
from std_msgs.msg import Int16MultiArray

SLOTS_PER_MOTOR = 6
P_MAX = 12.5            # 位置の返事が一周する範囲[rad]
POS_VEL_MAX_INTERNAL = 12.0
ENABLE_DELAY = 0.25     # 有効化にかかる時間[s]
LINK_TIMEOUT = 0.3      # PCの指令が途絶えたら脱力するまで[s]
MAX_ACCEL = 100.0       # 速さの変わり方の上限[rad/s^2](本物より大ざっぱ)
POS_GAIN = 20.0         # 角度制御で目標へ向かう強さ[1/s]


class FakeMotor:
    def __init__(self):
        self.mode = 0            # 今のモード(0=脱力 1=速度 2=角度)
        self.enable_at = None    # 有効になる時刻(有効化の途中)
        self.enabled = False
        self.pos = 0.0           # モータ内部の位置[rad](何周分も数える)
        self.vel = 0.0
        self.torque = 0.0
        self.counter = 0

    def update(self, mode, target, vel_limit, dt, now):
        # 角度制御は内部の位置が大きいと有効にしない(本物と同じ安全策)
        if mode == 2 and not (self.enabled and self.mode == 2) and abs(self.pos) > POS_VEL_MAX_INTERNAL:
            mode = 0
        if mode not in (1, 2):
            self.mode, self.enabled, self.enable_at = 0, False, None
        elif mode != self.mode or (not self.enabled and self.enable_at is None):
            self.mode, self.enabled, self.enable_at = mode, False, now + ENABLE_DELAY
        if self.enable_at is not None and now >= self.enable_at:
            self.enabled, self.enable_at = True, None

        if not self.enabled:
            want = 0.0 if abs(self.vel) < 0.5 else self.vel * 0.9   # 脱力: 摩擦で止まる
        elif self.mode == 1:
            want = target
        else:
            want = max(-vel_limit, min(vel_limit, POS_GAIN * (target - self.pos)))
        dv = max(-MAX_ACCEL * dt, min(MAX_ACCEL * dt, want - self.vel))
        self.vel += dv
        self.torque = 0.002 * dv / dt if self.enabled else 0.0
        self.pos += self.vel * dt
        self.counter = (self.counter + 1) % 30000

    def slots(self):
        wrapped = (self.pos + P_MAX) % (2 * P_MAX) - P_MAX
        state = 1 if self.enabled else 0

        def clamp16(x):
            return max(-32767, min(32767, int(round(x))))
        return [state, self.counter, clamp16(self.pos * 10), clamp16(wrapped * 1000),
                clamp16(self.vel * 100), clamp16(self.torque * 1000)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device-id", type=int, default=201)
    ap.add_argument("--motors", type=int, default=2, help="台数(1〜4)")
    ap.add_argument("--rate", type=float, default=200.0, help="返事を送る回数/秒")
    args, _ = ap.parse_known_args()
    if not 1 <= args.motors <= 4:
        ap.error("--motorsは1〜4")

    rclpy.init()
    node = rclpy.create_node("fake_esp32_%d" % args.device_id)
    motors = [FakeMotor() for _ in range(args.motors)]
    rx = {"data": [0] * 24, "time": 0.0}

    def on_tx(msg):
        rx["data"] = list(msg.data) + [0] * (24 - len(msg.data))
        rx["time"] = time.time()

    node.create_subscription(Int16MultiArray, "serial_tx_%d" % args.device_id, on_tx, 10)
    pub = node.create_publisher(Int16MultiArray, "serial_rx_%d" % args.device_id, 10)
    print(f"fake_esp32: DEVICE_ID {args.device_id}、モータ{args.motors}台のふりをしています(Ctrl+Cで終了)",
          flush=True)

    last = time.time()
    period = 1.0 / args.rate
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=period)
            now = time.time()
            dt = now - last
            if dt < period:
                continue
            last = now
            link_lost = now - rx["time"] > LINK_TIMEOUT
            out = [0] * 24
            for i, m in enumerate(motors):
                o = i * SLOTS_PER_MOTOR
                mode = 0 if link_lost else rx["data"][o]
                target = rx["data"][o + 1] * (0.01 if mode == 1 else 0.001)
                vel_limit = rx["data"][o + 2] * 0.01
                m.update(mode, target, vel_limit, dt, now)
                out[o:o + SLOTS_PER_MOTOR] = m.slots()
            msg = Int16MultiArray()
            msg.data = out
            pub.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
