#!/usr/bin/env python3
"""ESP32とDamiaoモータの「ふり」をするスクリプト(実機なしで自分のプログラムを試す道具)。

ros2can と実機の代わりに、serial_tx_<ID> を受け取り serial_rx_<ID> を返す。
damiao_driver_node・ps4_teleop_node・自分のプログラムは、実機の時と同じように動かせる。

使い方(ターミナル1で。ros2canは起動しないこと):
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && source ~/ros2_ws/install/setup.bash
  ros2 run damiao_ctrl fake_esp32.py                  # DEVICE_ID 201、モータ2台
  ros2 run damiao_ctrl fake_esp32.py --motors 4       # 4台
  ros2 run damiao_ctrl fake_esp32.py --device-id 204  # 2枚目のESP32のふり
  ros2 run damiao_ctrl fake_esp32.py --max-vel 45.6,40.0  # 独ステ1号機(操舵DM2325・駆動DM3519)
  ros2 run damiao_ctrl fake_esp32.py --runaway 2:1.0   # 試験用: モータ2が速度制御1秒後に暴走する
  ros2 run damiao_ctrl fake_esp32.py --runaway 2:1.0 --no-guard  # 偽ESP32の暴走の見張りを切る
                                                       # (駆動ノード側の見張りを試す時)

まねしていること(本物のESP32ファームウェア damiao.cpp と同じ決まり):
  ・データ枠(1台6個)の意味と単位
  ・有効化に約0.25秒かかる(脱力→エラー解除→設定書き込み→有効化)
  ・PCからの指令が0.3秒途絶えたら全台脱力して止めたままにする(状態15)。
    通信が戻っても動かず、PCが一度0(脱力)を送ると解除。起動直後も同じ(2026-11-06 ファーム変更)
  ・速度はモータごとの上限(--max-vel、既定は全台45.6rad/s)に丸める。角度制御の速さ上限が0以下なら有効にしない
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
MAX_VEL = 45.6          # 速度の上限の既定[rad/s](ファームのDAMIAO_MAX_VEL_RADPS)。--max-velで台ごとに変えられる
STATE_LATCHED = 15      # ESP32が止めたまま(一度0を送ると解除)
STATE_RUNAWAY = 7       # ESP32が暴走を見つけて止めたまま(一度0を送ると解除)
RUNAWAY_VEL = -40.0     # 試験用の暴走の速さ[rad/s](2026-11-09・11-17の実機と同じくらい)
MAX_ACCEL = 100.0       # 速さの変わり方の上限[rad/s^2](本物より大ざっぱ)
POS_GAIN = 20.0         # 角度制御で目標へ向かう強さ[1/s]


class RunawayGuard:
    """本物のファーム(ros2can の src/runaway_guard.hpp)と同じ暴走の見張り。"""
    MIN_SPEED, PROGRESS, MARGIN, TIME = 5.0, 1.0, 5.0, 0.2

    def __init__(self):
        self.reset()

    def reset(self):
        self.wrong_since = None
        self.over_since = None
        self.ref = 0.0

    def update(self, vel_mode, target, limit, vel, now):
        speed = abs(vel)
        if speed > limit + self.MARGIN:
            if self.over_since is None:
                self.over_since = now
            elif now - self.over_since >= self.TIME:
                return True
        else:
            self.over_since = None
        wrong = vel_mode and speed > self.MIN_SPEED and (vel * target < 0 or abs(target) < 0.5)
        if not wrong:
            self.wrong_since = None
            return False
        if self.wrong_since is None or speed < self.ref - self.PROGRESS:
            self.wrong_since, self.ref = now, speed
            return False
        return now - self.wrong_since >= self.TIME


class FakeMotor:
    def __init__(self, max_vel=MAX_VEL, runaway_after=None, guard=True):
        self.max_vel = max_vel   # このモータの速度の上限[rad/s](ファームのDAMIAO_MOTORSの3列目)
        self.mode = 0            # 今のモード(0=脱力 1=速度 2=角度)
        self.enable_at = None    # 有効になる時刻(有効化の途中)
        self.enabled = False
        self.pos = 0.0           # モータ内部の位置[rad](何周分も数える)
        self.vel = 0.0
        self.torque = 0.0
        self.counter = 0
        self.latched = True      # ESP32が止めたままか(起動直後はPCが0を送るまで動かさない)
        self.runaway_latched = False  # 止めたままの理由が暴走か(状態7)
        self.runaway_after = runaway_after  # 試験用: 速度制御をこの秒数続けると暴走する(Noneなら暴走しない)
        self.vel_time = 0.0      # 速度制御で有効になっている時間[s]
        self.guard = RunawayGuard() if guard else None

    def update_latch(self, pc_mode, link_lost):
        # 本物の damiao.cpp と同じ: 途絶でラッチ。外れるのは、PCが実際に0を送った時だけ
        if link_lost:
            self.latched = True
        elif pc_mode == 0:
            self.latched = False
            self.runaway_latched = False
        return 0 if self.latched else pc_mode

    def update(self, mode, target, vel_limit, dt, now):
        if mode == 1:
            target = max(-self.max_vel, min(self.max_vel, target))
        elif mode == 2:
            if vel_limit <= 0.0:
                mode = 0
            vel_limit = min(vel_limit, self.max_vel)
        # 角度制御は内部の位置が大きいと有効にしない(本物と同じ安全策)
        if mode == 2 and not (self.enabled and self.mode == 2) and abs(self.pos) > POS_VEL_MAX_INTERNAL:
            mode = 0
        if mode not in (1, 2):
            self.mode, self.enabled, self.enable_at = 0, False, None
        elif mode != self.mode or (not self.enabled and self.enable_at is None):
            self.mode, self.enabled, self.enable_at = mode, False, now + ENABLE_DELAY
        if self.enable_at is not None and now >= self.enable_at:
            self.enabled, self.enable_at = True, None

        self.vel_time = self.vel_time + dt if (self.enabled and self.mode == 1) else 0.0
        running_away = self.runaway_after is not None and self.vel_time >= self.runaway_after
        if not self.enabled:
            want = 0.0 if abs(self.vel) < 0.5 else self.vel * 0.9   # 脱力: 摩擦で止まる
        elif running_away:
            want = RUNAWAY_VEL   # 試験用の暴走: 指令に関係なく逆向きに回る
        elif self.mode == 1:
            want = target
        else:
            want = max(-vel_limit, min(vel_limit, POS_GAIN * (target - self.pos)))
        dv = max(-MAX_ACCEL * dt, min(MAX_ACCEL * dt, want - self.vel))
        self.vel += dv
        self.torque = 0.002 * dv / dt if self.enabled else 0.0
        self.pos += self.vel * dt
        self.counter = (self.counter + 1) % 30000
        # 暴走の見張り(本物のファームと同じ)。見つけたら脱力して止めたまま(状態7)
        if self.guard is not None:
            limit = self.max_vel if self.mode == 1 else vel_limit
            if self.enabled and self.guard.update(self.mode == 1, target, limit, self.vel, now):
                self.latched, self.runaway_latched = True, True
                self.mode, self.enabled, self.enable_at = 0, False, None
                self.guard.reset()
            elif not self.enabled:
                self.guard.reset()

    def slots(self):
        wrapped = (self.pos + P_MAX) % (2 * P_MAX) - P_MAX
        if self.latched:
            state = STATE_RUNAWAY if self.runaway_latched else STATE_LATCHED
        else:
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
    ap.add_argument("--runaway", default="",
                    help="試験用: 暴走させるモータと秒数(例: 2:1.0 = モータ2が速度制御を1秒続けると暴走)")
    ap.add_argument("--no-guard", action="store_true", help="偽ESP32の暴走の見張りを切る(駆動ノード側を試す時)")
    ap.add_argument("--max-vel", default="",
                    help="台ごとの速度の上限[rad/s]をカンマ区切りで(例: 45.6,59.4)。省略すると全台45.6")
    args, _ = ap.parse_known_args()
    if not 1 <= args.motors <= 4:
        ap.error("--motorsは1〜4")

    rclpy.init()
    node = rclpy.create_node("fake_esp32_%d" % args.device_id)
    max_vels = [float(v) for v in args.max_vel.split(",")] if args.max_vel else [MAX_VEL] * args.motors
    if len(max_vels) != args.motors:
        ap.error("--max-velは--motorsと同じ数にする")
    runaway = {}
    if args.runaway:
        num, sec = args.runaway.split(":")
        runaway[int(num) - 1] = float(sec)
    motors = [FakeMotor(v, runaway.get(i), not args.no_guard) for i, v in enumerate(max_vels)]
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
                mode = m.update_latch(rx["data"][o], link_lost)
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
