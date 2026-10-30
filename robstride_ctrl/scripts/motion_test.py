#!/usr/bin/env python3
"""RobStride EduLite 05 の速度制御・角度制御を、決まった手順で1回だけ試すスクリプト(rclpy)。

robstride_driver_node が起動している前提で robstride/motor<N>/cmd を50Hzで送り、
robstride/motor<N>/state を見て結果を表示する。どの終わり方(正常終了・Ctrl+C・例外)でも
最後に DISABLE(-1) を送る。

使い方:
  ros2 run robstride_ctrl motion_test.py angle  [移動量rad(既定0.5)] [速度上限rad/s(既定0.5)]
      現在位置 -> 現在位置+移動量 -> 現在位置 の順に動かし、到達した位置を表示する。
  ros2 run robstride_ctrl motion_test.py speed  [速度rad/s(既定0.5)] [秒(既定3)]
      0 -> +速度 -> 0 -> -速度 -> 0 の順に回し、平均速度を表示する。
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray

DISABLE = -1.0
VELOCITY = 0.0
POSITION = 1.0

# robstride_driver_nodeのstate: [位置, 速度, トルク, モード, 帰還OK, 故障フラグ, 温度]
FLAG_LATCHED = 0x100
FLAG_POS_UNKNOWN = 0x200
FAULT_MASK = 0x3F
MODE_RUN = 2


class MotionTest(Node):
    def __init__(self, motor_number: int = 1) -> None:
        super().__init__('robstride_motion_test')
        self.cmd_pub = self.create_publisher(
            Float32MultiArray, f'robstride/motor{motor_number}/cmd', 10)
        self.create_subscription(
            Float32MultiArray, f'robstride/motor{motor_number}/state', self._on_state, 10)
        self.state: list[float] | None = None
        self.state_time = 0.0

    def _on_state(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 7:
            self.state = list(msg.data)
            self.state_time = time.monotonic()

    # ---- 補助 ----
    def send(self, mode: float, target: float = 0.0, vel_limit: float = 0.0) -> None:
        self.cmd_pub.publish(Float32MultiArray(data=[mode, target, vel_limit]))

    def spin_for(self, seconds: float, mode: float, target: float = 0.0,
                 vel_limit: float = 0.0) -> None:
        """seconds秒の間、同じ指令を50Hzで送り続ける(driverのcmd途絶の見張りに掛からないように)。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(mode, target, vel_limit)
            rclpy.spin_once(self, timeout_sec=0.02)
            self.check_alive(mode)

    def check_alive(self, mode: float) -> None:
        s = self.state
        if s is None or time.monotonic() - self.state_time > 0.5:
            raise RuntimeError('stateが届きません(robstride_driver_node・ros2canを確認)')
        if s[4] < 0.5:
            raise RuntimeError('モータからの帰還が途絶えました(電源・CAN線を確認)')
        flags = int(s[5])
        if flags & FAULT_MASK:
            raise RuntimeError(f'モータ故障 flags=0x{flags:x}')
        if flags & FLAG_LATCHED and mode != DISABLE:
            raise RuntimeError('ESP32がモータを停止させました(運転中にモータが止まった)')

    def wait_state(self, timeout: float = 3.0) -> list[float]:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.send(DISABLE)
            rclpy.spin_once(self, timeout_sec=0.02)
            s = self.state
            if s is not None and s[4] > 0.5 and not int(s[5]) & FLAG_POS_UNKNOWN:
                return s
        raise RuntimeError('モータの帰還・位置が届きません(ros2can・ESP32・モータの電源を確認)')

    def log(self, text: str) -> None:
        s = self.state
        if s is not None:
            print(f'{text:<28} 位置 {s[0]:+8.3f} rad | 速度 {s[1]:+6.2f} rad/s | '
                  f'トルク {s[2]:+5.2f} N·m | モード {int(s[3])} | {s[6]:4.1f}℃', flush=True)

    # ---- 試験 ----
    def angle_test(self, delta: float, vel_limit: float) -> None:
        p0 = self.wait_state()[0]
        self.log('開始(停止中)')
        for name, target in (('+移動', p0 + delta), ('戻る', p0)):
            # 移動に掛かる時間 + 余裕(位置モードは速度上限の+22%程度まで出る)
            duration = abs(delta) / vel_limit + 1.5
            print(f'{name}: 目標 {target:+.3f} rad (速度上限 {vel_limit} rad/s, {duration:.1f}秒)')
            self.spin_for(duration, POSITION, target, vel_limit)
            self.log(f'  {name} 後')
            if self.state is not None and int(self.state[3]) != MODE_RUN:
                raise RuntimeError('モータが運転モードになっていません')
        err = self.state[0] - p0 if self.state else float('nan')
        print(f'結果: 移動量 {delta:+.3f} rad で往復し、元の位置との差 {err:+.4f} rad')

    def speed_test(self, speed: float, seconds: float) -> None:
        self.wait_state()
        self.log('開始(停止中)')
        for name, v in (('+速度', speed), ('-速度', -speed)):
            print(f'{name}: {v:+.2f} rad/s を {seconds} 秒')
            self.spin_for(0.5, VELOCITY, 0.0)  # 有効化(約15ms)と0指令
            p_start = self.state[0]
            t_start = time.monotonic()
            self.spin_for(seconds, VELOCITY, v)
            p_end = self.state[0]
            measured = (p_end - p_start) / (time.monotonic() - t_start)
            self.log(f'  {name} 中')
            print(f'  平均速度(位置の変化から) {measured:+.3f} rad/s  (指令 {v:+.2f}。加速分だけ小さめに出る)')
            self.spin_for(1.0, VELOCITY, 0.0)  # 減速して止まるのを待つ
            self.log(f'  {name} 停止後')


def main() -> None:
    args = sys.argv[1:]
    args = [a for a in args if not a.startswith('--ros-args')]
    if not args or args[0] not in ('angle', 'speed'):
        print(__doc__)
        sys.exit(1)
    # rclpy標準のCtrl+C処理はROSの通信を先に切ってしまい、finallyのDISABLEが届かない
    # (damiao_ctrlで実機確認した現象)。無効にして、普通のKeyboardInterruptとして受ける。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MotionTest()
    try:
        if args[0] == 'angle':
            delta = float(args[1]) if len(args) > 1 else 0.5
            vel = float(args[2]) if len(args) > 2 else 0.5
            if vel <= 0.0:
                raise ValueError('速度上限は正の値にしてください')
            node.angle_test(delta, vel)
        else:
            speed = float(args[1]) if len(args) > 1 else 0.5
            seconds = float(args[2]) if len(args) > 2 else 3.0
            node.speed_test(speed, seconds)
        print('完了')
    except KeyboardInterrupt:
        print('\nCtrl+Cで中断しました')
    except Exception as e:  # noqa: BLE001 - どんな失敗でも停止を送ってから終わる
        print(f'中断: {e}')
    finally:
        # どの終わり方でも止める(driverへのDISABLEを数回)
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            node.send(DISABLE)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.log('終了(DISABLE送信後)')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
