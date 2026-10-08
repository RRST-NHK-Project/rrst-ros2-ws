#!/usr/bin/env python3
"""実機なしで CubeMars_ctrl のROS側を確認するための疑似ESP32(MODE_CUBEMARS)。

疑似シリアルポート(pty)を作り、ファーム(firmware/soki_cubemars)と同じ
52バイトフレームで指令を受け取り、簡単なモータモデルを回して帰還を返す。

    ros2 run CubeMars_ctrl fake_esp32.py
    # 表示された /dev/pts/N を使って別ターミナルで
    ros2 launch CubeMars_ctrl cubemars.launch.py serial_port:=/dev/pts/N

--no-motor を付けると「ESP32は動いているがモータが応答しない」状態(CAN断線・
CAN ID違い)を再現する(受信カウンタが増えない)。

あくまで通信・単位換算・安全処理の確認用で、実機の動き方(加速度・剛性)は再現しない。
"""

import argparse
import math
import os
import pty
import select
import signal
import time
import tty

START_BYTE = 0xAA
SLOTS = 24
CMD_TIMEOUT_S = 0.3   # ファームのCUBEMARS_CMD_TIMEOUT_MSと同じ
TICK_S = 0.005        # ファームのcubemarsTask/serialTaskと同じ5ms周期


def encode_frame(device_id, values):
    body = bytearray([device_id, SLOTS * 2])
    for v in values:
        u = v & 0xFFFF
        body += bytes([(u >> 8) & 0xFF, u & 0xFF])
    checksum = 0
    for b in body:
        checksum ^= b
    return bytes([START_BYTE]) + bytes(body) + bytes([checksum])


class FrameParser:
    def __init__(self):
        self.buf = bytearray()

    def push(self, data):
        self.buf += data

    def pop(self):
        frame_len = 3 + SLOTS * 2 + 1
        while True:
            while self.buf and self.buf[0] != START_BYTE:
                del self.buf[0]
            if len(self.buf) < frame_len:
                return None
            if self.buf[2] != SLOTS * 2:
                del self.buf[0]
                continue
            checksum = 0
            for b in self.buf[1:frame_len - 1]:
                checksum ^= b
            if checksum != self.buf[frame_len - 1]:
                del self.buf[0]
                continue
            device_id = self.buf[1]
            values = []
            for i in range(SLOTS):
                u = (self.buf[3 + 2 * i] << 8) | self.buf[4 + 2 * i]
                values.append(u - 0x10000 if u >= 0x8000 else u)
            del self.buf[:frame_len]
            return device_id, values


def to_int16(x):
    return max(-32768, min(32767, int(round(x))))


class FakeMotor:
    """出力軸で見た簡易モデル。速度ループは一次遅れ、MITはバネ・ダンパ+慣性。"""

    def __init__(self, erpm_per_rpm):
        self.erpm_per_rpm = erpm_per_rpm
        self.pos_rad = 0.0
        self.vel_radps = 0.0
        self.current_a = 0.0

    def step(self, mode, cmd, dt):
        if mode == 2:  # MIT
            pos_des = math.radians(cmd[0] * 0.1)
            vel_des = cmd[8] * 0.01
            kp = cmd[12] * 0.1
            kd = cmd[16] * 0.01
            inertia = 0.002  # [kg m^2] 出力軸換算(適当)
            # 実機はMIT制御を数十kHzで回すが、ここは5ms刻みなのでダンパ項を陰的に
            # 解く(陽解法だとKd/慣性が大きいと数値的に発振する)。
            spring = kp * (pos_des - self.pos_rad) + kd * vel_des
            v_new = (inertia * self.vel_radps + dt * spring) / (inertia + dt * (kd + 0.02))
            torque = spring - kd * v_new
            if abs(torque) > 5.0:  # AK40-10のMITトルク上限
                torque = math.copysign(5.0, torque)
                v_new = self.vel_radps + (torque - 0.02 * self.vel_radps) / inertia * dt
            self.vel_radps = v_new
            self.current_a = torque / (0.056 * 10.0)
        else:  # 速度ループ(位置ループ mode=1 はこのツールでは未対応のため停止扱い)
            target_rpm = (cmd[0] * 10.0) / self.erpm_per_rpm if mode == 0 else 0.0
            target = target_rpm * 2.0 * math.pi / 60.0
            self.vel_radps += (target - self.vel_radps) * min(1.0, dt / 0.05)
            self.current_a = 0.3 * (target - self.vel_radps) + 0.05 * math.copysign(1, target) * (target != 0)
        self.vel_radps = max(-45.5, min(45.5, self.vel_radps))
        self.pos_rad += self.vel_radps * dt

    def feedback_slots(self):
        rpm = self.vel_radps * 60.0 / (2.0 * math.pi)
        return (
            to_int16(math.degrees(self.pos_rad) / 0.1),
            to_int16(rpm * self.erpm_per_rpm / 10.0),
            to_int16(self.current_a / 0.01),
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--device-id', type=int, default=11)
    parser.add_argument('--erpm-per-rpm', type=float, default=140.0,
                        help='疑似モータの 極対数*減速比 (既定140 = 14*10)')
    parser.add_argument('--no-motor', action='store_true',
                        help='モータが応答しない状態(受信カウンタが増えない)を再現する')
    parser.add_argument('--start-deg', type=float, default=0.0, help='初期角度[deg]')
    parser.add_argument('--link', default='', help='疑似ポートへのシンボリックリンクを作るパス(任意)')
    args = parser.parse_args()

    # kill(SIGTERM)でもCtrl+Cと同じく後片付けしてから終了する
    def on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)

    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)  # エコーを切る(切らないと自分の送信を指令として読んでしまう)
    os.set_blocking(master_fd, False)
    slave_name = os.ttyname(slave_fd)
    if args.link:
        if os.path.islink(args.link):
            os.unlink(args.link)
        os.symlink(slave_name, args.link)

    print(f'疑似ESP32を起動しました: {slave_name}  (DEVICE_ID={args.device_id}'
          f'{", モータ無応答" if args.no_motor else ""})')
    print(f'  ros2 launch CubeMars_ctrl cubemars.launch.py serial_port:={args.link or slave_name}')
    print('Ctrl+C で終了')

    motor = FakeMotor(args.erpm_per_rpm)
    motor.pos_rad = math.radians(args.start_deg)
    frame_parser = FrameParser()
    rx = [0] * SLOTS
    last_rx = None
    rx_frames = 0
    counter = 0
    next_tick = time.monotonic()
    next_print = time.monotonic()
    timed_out_prev = None

    try:
        while True:
            timeout = max(0.0, next_tick - time.monotonic())
            readable, _, _ = select.select([master_fd], [], [], timeout)
            if readable:
                try:
                    frame_parser.push(os.read(master_fd, 4096))
                except (BlockingIOError, OSError):
                    pass
                while True:
                    frame = frame_parser.pop()
                    if frame is None:
                        break
                    device_id, values = frame
                    if device_id == args.device_id:
                        rx = values
                        last_rx = time.monotonic()
                        rx_frames += 1

            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick += TICK_S
            if next_tick < now:
                next_tick = now + TICK_S

            timed_out = last_rx is None or (now - last_rx) > CMD_TIMEOUT_S
            if timed_out != timed_out_prev and last_rx is not None:
                print('[fake] 指令途絶 -> ゼロ速度(フェイルセーフ)' if timed_out else '[fake] 指令受信中')
            timed_out_prev = timed_out
            cmd = [0] * SLOTS if timed_out else rx
            mode = cmd[4]
            motor.step(mode, cmd, TICK_S)

            tx = [0] * SLOTS
            if not args.no_motor:
                counter = (counter + 1) & 0xFFFF
                pos, spd, cur = motor.feedback_slots()
                tx[0], tx[4], tx[8] = pos, spd, cur
                tx[12] = 30
                tx[16] = 0
                tx[20] = counter - 0x10000 if counter >= 0x8000 else counter
            try:
                os.write(master_fd, encode_frame(args.device_id, tx))
            except (BlockingIOError, OSError):
                pass  # 誰もポートを開いていない間は捨てる

            if now >= next_print:
                next_print = now + 1.0
                mode_name = {0: '速度', 1: '位置(未対応)', 2: 'MIT'}.get(mode, str(mode))
                print(f'[fake] 受信{rx_frames}フレーム mode={mode_name} target_slot={cmd[0]} '
                      f'pos={math.degrees(motor.pos_rad):.1f}deg '
                      f'rpm={motor.vel_radps * 60 / (2 * math.pi):.1f}')
    except KeyboardInterrupt:
        pass
    finally:
        if args.link and os.path.islink(args.link):
            os.unlink(args.link)


if __name__ == '__main__':
    main()
