#!/usr/bin/env python3
"""2台のDamiaoモータの低速動作テスト(damiao_driver_node経由。ros2canとdamiao_driver_nodeを先に起動しておく)。

使い方(どの端末でも export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST を先に実行):
  python3 two_motor_test.py [速度rad/s(既定0.5)]

  A. モータ1だけ +v で2秒 -> 停止
  B. モータ2だけ +v で2秒 -> 停止
  C. モータ1を回している途中(1秒後)にモータ2を -v で有効化し、2台で2秒 -> 停止
     (モータ2の有効化中もモータ1が止まらない=通信喪失(13)にならないことを確認)
  中止条件: 故障コード(0/1以外) / 帰還途絶 / 速度の読みが 3*v+0.5 rad/s超 / stateが届かない
  終了時(Ctrl+Cでも)は必ず2台ともDISABLEを送る。
"""
import signal
import sys
import time

import rclpy
from std_msgs.msg import Float32MultiArray

V = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5


class Abort(Exception):
    pass


def on_signal(signum, _frame):
    raise Abort(f"シグナル{signum}")


signal.signal(signal.SIGTERM, on_signal)
signal.signal(signal.SIGINT, on_signal)

rclpy.init()
node = rclpy.create_node("two_motor_test")
state = {1: None, 2: None}
stamp = {1: 0.0, 2: 0.0}
pubs = {}
for m in (1, 2):
    def cb(msg, m=m):
        state[m] = list(msg.data)
        stamp[m] = time.time()
    node.create_subscription(Float32MultiArray, f"damiao/motor{m}/state", cb, 50)
    pubs[m] = node.create_publisher(Float32MultiArray, f"damiao/motor{m}/cmd", 10)


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.005)


def send(m, vel):
    msg = Float32MultiArray()
    msg.data = [-1.0, 0.0, 0.0] if vel is None else [0.0, float(vel), 0.0]
    pubs[m].publish(msg)


def check():
    for m in (1, 2):
        s = state[m]
        if s is None or time.time() - stamp[m] > 0.3:
            raise Abort(f"モータ{m}のstateが届かない")
        if s[4] < 0.5:
            raise Abort(f"モータ{m}の帰還が途絶")
        if int(s[3]) not in (0, 1):
            raise Abort(f"モータ{m}が故障コード {int(s[3])}")
        if abs(s[1]) > 3 * abs(V) + 0.5:
            raise Abort(f"モータ{m}の速度の読み {s[1]:+.2f}rad/s が大きすぎる")


def run(name, sec, cmds, watch_enabled=None):
    """cmds: {モータ: 速度 or None(DISABLE)} をsec秒送り続ける。移動量を返す。
    watch_enabled: このモータが一度状態1になった後、1以外になったら中止(有効化中の巻き添え停止の検出)"""
    print(f"--- {name} ---", flush=True)
    p0 = {m: state[m][0] for m in (1, 2)}
    t0 = time.time()
    next_print = 0.0
    seen_enabled = False
    while time.time() - t0 < sec:
        for m, v in cmds.items():
            send(m, v)
        spin(0.02)
        check()
        if watch_enabled is not None:
            st = int(state[watch_enabled][3])
            seen_enabled = seen_enabled or st == 1
            if seen_enabled and st != 1:
                raise Abort(f"モータ{watch_enabled}が回転中に状態{st}になった")
        if time.time() >= next_print:
            next_print = time.time() + 0.5
            print("  " + "  ".join(
                f"モータ{m}: 状態{int(state[m][3])} 位置{state[m][0]:+7.3f} 速度{state[m][1]:+5.2f}"
                for m in (1, 2)), flush=True)
    return {m: state[m][0] - p0[m] for m in (1, 2)}


def stop_all(sec=0.6):
    t0 = time.time()
    while time.time() - t0 < sec:
        send(1, None)
        send(2, None)
        spin(0.02)


ok = False
try:
    spin(2.0)
    check()
    for m in (1, 2):
        if int(state[m][3]) != 0:
            raise Abort(f"開始時にモータ{m}が状態0ではない({int(state[m][3])})")
    exp = V * 2.0
    print(f"速度 {V:+.2f}rad/s で試験(2秒で約{exp:.1f}rad。有効化に約0.25秒かかるのでやや少なめになる)", flush=True)

    mv = run("A. モータ1だけ", 2.0, {1: V, 2: None})
    stop_all()
    print(f"  => 移動: モータ1 {mv[1]:+.3f}rad, モータ2 {mv[2]:+.3f}rad", flush=True)
    if not (0.5 * exp <= mv[1] <= 1.3 * exp and abs(mv[2]) < 0.05):
        raise Abort("Aの結果が想定外(モータ1が動かない、またはモータ2が動いた)")

    mv = run("B. モータ2だけ", 2.0, {1: None, 2: V})
    stop_all()
    print(f"  => 移動: モータ1 {mv[1]:+.3f}rad, モータ2 {mv[2]:+.3f}rad", flush=True)
    if not (0.5 * exp <= mv[2] <= 1.3 * exp and abs(mv[1]) < 0.05):
        raise Abort("Bの結果が想定外(モータ2が動かない、またはモータ1が動いた)")

    run("C1. モータ1を回し始める", 1.0, {1: V, 2: None}, watch_enabled=1)
    mv = run("C2. モータ1を回したままモータ2を逆向きに有効化", 2.0, {1: V, 2: -V}, watch_enabled=1)
    stop_all()
    print(f"  => C2の移動: モータ1 {mv[1]:+.3f}rad(止まらず回り続けたか), モータ2 {mv[2]:+.3f}rad", flush=True)
    if not (0.7 * exp <= mv[1] <= 1.3 * exp and -1.3 * exp <= mv[2] <= -0.5 * exp):
        raise Abort("Cの結果が想定外")
    ok = True
    print("== 結果: 合格", flush=True)
except Abort as e:
    print(f"!! {e} -> 2台ともDISABLEして終了", flush=True)
finally:
    stop_all(1.0)
    for m in (1, 2):
        if state[m]:
            print(f"DISABLE後 モータ{m}: 状態={int(state[m][3])} 位置={state[m][0]:+.3f}rad", flush=True)
    node.destroy_node()
    rclpy.shutdown()
sys.exit(0 if ok else 1)
