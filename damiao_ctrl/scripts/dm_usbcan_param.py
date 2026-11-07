#!/usr/bin/env python3
"""達妙(Damiao)USB-CAN(DM-USBCU1, /dev/ttyACM*)経由でDM3520等のESCのレジスタを読み書きするツール。

使い方:
  python3 dm_usbcan_param.py scan                  # ESC_ID 1〜32に応答するESCを探す(読むだけ)
  python3 dm_usbcan_param.py read  --id 1          # 主なレジスタを読む(読むだけ)
  python3 dm_usbcan_param.py set-id --id 1 --new-id 2 --new-mst 0x12
                                                  # ESC_ID/MST_IDを書き換えてフラッシュに保存
                                                  # (モータの設定が恒久的に変わる。確認してから使う)
  python3 dm_usbcan_param.py set-ram --id 1 --rid 27 --value 270
                                                  # レジスタをRAMにだけ書く(電源を切ると元に戻る)。
                                                  # 対象はACC/DEC/KP_APR/KI_APRのみ・範囲チェックあり
共通オプション: --port /dev/ttyACM0

USB-CAN(シリアル)の形式は達妙公式DM_Control_PythonのDM_CAN.pyと同じ:
  送信30byte: 55 AA 1E 03 01 00 00 00 0A 00 00 00 00 [CAN ID 4byte LE] 00 08 00 00 [data 8byte] 00
  受信16byte: AA [CMD] .. [CAN ID 4byte LE @3] [data 8byte @7] 55
レジスタ読み出し: CAN ID 0x7FF, data {id_lo, id_hi, 0x33, RID, 0,0,0,0}
レジスタ書き込み: CAN ID 0x7FF, data {id_lo, id_hi, 0x55, RID, 値(4byte LE)} (RAMのみ)
フラッシュ保存:   CAN ID 0x7FF, data {id_lo, id_hi, 0xAA, 0,0,0,0,0} (Disable中のみ有効)
返信はESCのMST_ID(Master ID)のCAN IDで届く。
"""
import argparse
import struct
import sys
import time

import serial

# (RID, 名前, 型) 型: "f"=float32, "u"=uint32
REGISTERS = [
    (0, "UV_Value 低電圧保護[V]", "f"),
    # 加減速・制御ループの設定(2026-11-07追加、読むだけ)。ステップ応答の実測で、速度制御は
    # 一定の加速度(約80rad/s^2)で立ち上がり、角度制御(POS_VEL)は時定数約0.46秒と遅かった。
    # その原因を確かめるために読む。RIDの番号は達妙公式 DM_CAN.py の DM_variable と同じ。
    (4, "ACC 加速度", "f"),
    (5, "DEC 減速度", "f"),
    (6, "MAX_SPD 最高速度", "f"),
    (7, "MST_ID 帰還のCAN ID", "u"),
    (8, "ESC_ID 受信CAN ID", "u"),
    (9, "TIMEOUT 通信タイムアウト(1=50us)", "u"),
    (10, "CTRL_MODE 制御モード(1=MIT,2=POS_VEL,3=VEL)", "u"),
    (14, "sw_ver ソフトウェア版", "u"),
    (20, "Gr 減速比", "f"),
    (21, "PMAX 位置レンジ[rad]", "f"),
    (22, "VMAX 速度レンジ[rad/s]", "f"),
    (23, "TMAX トルクレンジ[N・m]", "f"),
    (24, "I_BW 電流ループ帯域", "f"),
    (25, "KP_ASR 速度ループ 比例ゲイン", "f"),
    (26, "KI_ASR 速度ループ 積分ゲイン", "f"),
    (27, "KP_APR 位置ループ 比例ゲイン", "f"),
    (28, "KI_APR 位置ループ 積分ゲイン", "f"),
    (32, "V_BW 速度ループ帯域", "f"),
    (35, "can_br CANボーレート番号", "u"),
]
RID_MST_ID = 7
RID_ESC_ID = 8


class UsbCan:
    def __init__(self, port):
        self.ser = serial.Serial(port, 921600, timeout=0.01)
        self.buf = b""

    def send(self, can_id, data):
        frame = bytearray([0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
                           0, 0, 0, 0, 0x00, 0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00])
        frame[13] = can_id & 0xFF
        frame[14] = (can_id >> 8) & 0xFF
        frame[21:29] = bytes(data)
        self.ser.write(bytes(frame))

    def recv(self, window_sec):
        """window_sec秒の間に受信したCANフレーム[(can_id, data)]を返す"""
        frames = []
        end = time.time() + window_sec
        while time.time() < end:
            self.buf += self.ser.read(256)
            while len(self.buf) >= 16:
                i = self.buf.find(b"\xAA")
                if i < 0:
                    self.buf = b""
                    break
                if len(self.buf) - i < 16:
                    self.buf = self.buf[i:]
                    break
                pkt = self.buf[i:i + 16]
                if pkt[15] != 0x55:
                    self.buf = self.buf[i + 1:]
                    continue
                self.buf = self.buf[i + 16:]
                can_id = pkt[3] | (pkt[4] << 8) | (pkt[5] << 16) | (pkt[6] << 24)
                frames.append((can_id, bytes(pkt[7:15])))
        return frames


def read_register(bus, esc_id, rid, window=0.05):
    """(返信のCAN ID, 4byteの生値)を返す。応答なしならNone"""
    bus.send(0x7FF, [esc_id & 0xFF, (esc_id >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0])
    for can_id, d in bus.recv(window):
        if d[0] == (esc_id & 0xFF) and d[1] == ((esc_id >> 8) & 0xFF) and d[2] == 0x33 and d[3] == rid:
            return can_id, d[4:8]
    return None


def fmt(raw, typ):
    if typ == "f":
        return f"{struct.unpack('<f', raw)[0]:.4g}"
    v = struct.unpack('<I', raw)[0]
    return f"{v} (0x{v:X})"


def cmd_scan(bus, _args):
    found = []
    for esc_id in range(1, 33):
        r = read_register(bus, esc_id, RID_ESC_ID)
        if r:
            found.append(esc_id)
            print(f"ESC_ID {esc_id}: 応答あり(返信CAN ID 0x{r[0]:03X} = MST_ID)", flush=True)
    if not found:
        print("応答なし(ESCの電源・CAN線・USB-CANの接続を確認)")
    return 0 if found else 1


def cmd_read(bus, args):
    ok = False
    for rid, name, typ in REGISTERS:
        r = read_register(bus, args.id, rid)
        if r:
            ok = True
            print(f"  RID{rid:2d} {name}: {fmt(r[1], typ)}   (返信CAN ID 0x{r[0]:03X})", flush=True)
        else:
            print(f"  RID{rid:2d} {name}: 応答なし", flush=True)
    return 0 if ok else 1


def cmd_set_id(bus, args):
    old, new_id, new_mst = args.id, args.new_id, args.new_mst
    if read_register(bus, old, RID_ESC_ID) is None:
        print(f"ESC_ID {old} が応答しないので中止")
        return 1
    print(f"ESC_ID {old} -> {new_id}, MST_ID -> 0x{new_mst:X} に書き換えて保存します", flush=True)
    bus.send(old, [0xFF] * 7 + [0xFD])  # Disable(保存はDisable中のみ有効)
    bus.recv(0.1)
    for rid, val in ((RID_MST_ID, new_mst), (RID_ESC_ID, new_id)):
        bus.send(0x7FF, [old & 0xFF, (old >> 8) & 0xFF, 0x55, rid] + list(struct.pack('<I', val)))
        bus.recv(0.1)
    # ESC_IDを書いた直後から新しいIDで受け付けるかは機種依存のため、新旧両方で保存を送る
    for target in (new_id, old):
        bus.send(0x7FF, [target & 0xFF, (target >> 8) & 0xFF, 0xAA, 0, 0, 0, 0, 0])
        bus.recv(0.3)
    time.sleep(0.5)
    r = read_register(bus, new_id, RID_ESC_ID, window=0.2)
    if r is None:
        print("新しいIDで応答がない。ESCの電源を入れ直してから scan で確認すること")
        return 1
    print(f"新しいESC_ID {new_id} で応答あり(返信CAN ID 0x{r[0]:03X})。電源を入れ直して scan で再確認すること")
    return 0


# set-ram で書き換えてよいレジスタと、受け付ける範囲(RID: (名前, 最小, 最大))。
# 2026-11-07、角度制御(POS_VEL)が遅い原因(KP_APR=54 が減速比で割られて効く)を確かめるため追加。
# 制御に直結する値なので、決めたものだけ・範囲内だけ受け付ける(負のゲインで機構を壊した事例がある)。
SET_RAM_ALLOWED = {
    4: ("ACC 加速度", 0.5, 20.0),
    5: ("DEC 減速度", -20.0, -0.5),
    27: ("KP_APR 位置ループ 比例ゲイン", 1.0, 1000.0),
    28: ("KI_APR 位置ループ 積分ゲイン", 0.0, 100.0),
}


def cmd_set_ram(bus, args):
    """レジスタをRAMにだけ書く(フラッシュ保存0xAAは送らない)。ESCの電源を切ると元の値に戻る。"""
    esc, rid, val = args.id, args.rid, args.value
    if rid not in SET_RAM_ALLOWED:
        print(f"RID{rid} は set-ram の対象外です(対象: {sorted(SET_RAM_ALLOWED)})")
        return 1
    name, lo, hi = SET_RAM_ALLOWED[rid]
    if not (lo <= val <= hi):
        print(f"{name} は {lo}〜{hi} の範囲にしてください(指定 {val})")
        return 1
    before = read_register(bus, esc, rid, window=0.2)
    if before is None:
        print(f"ESC_ID {esc} が応答しないので中止")
        return 1
    print(f"ESC_ID {esc} RID{rid} {name}: {fmt(before[1], 'f')} -> {val:g}(RAMのみ。電源を切ると戻る)", flush=True)
    bus.send(esc, [0xFF] * 7 + [0xFD])  # 念のためDisable(書き換え中に動かないように)
    bus.recv(0.1)
    bus.send(0x7FF, [esc & 0xFF, (esc >> 8) & 0xFF, 0x55, rid] + list(struct.pack('<f', val)))
    bus.recv(0.1)
    after = read_register(bus, esc, rid, window=0.2)
    if after is None:
        print("書き込み後の読み出しに応答なし。read で確認すること")
        return 1
    got = struct.unpack('<f', after[1])[0]
    if abs(got - val) > 1e-4 * max(1.0, abs(val)):
        print(f"読み戻した値が {got:g} で、書いた値と違う")
        return 1
    print(f"読み戻し OK: {got:g}")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=["scan", "read", "set-id", "set-ram"])
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--id", type=lambda s: int(s, 0), default=1, help="対象のESC_ID")
    p.add_argument("--new-id", type=lambda s: int(s, 0), help="set-id: 新しいESC_ID")
    p.add_argument("--new-mst", type=lambda s: int(s, 0), help="set-id: 新しいMST_ID")
    p.add_argument("--rid", type=int, help="set-ram: 書き換えるレジスタ番号")
    p.add_argument("--value", type=float, help="set-ram: 書き込む値")
    args = p.parse_args()
    if args.command == "set-id" and (args.new_id is None or args.new_mst is None):
        p.error("set-id には --new-id と --new-mst が必要")
    if args.command == "set-ram" and (args.rid is None or args.value is None):
        p.error("set-ram には --rid と --value が必要")
    bus = UsbCan(args.port)
    bus.recv(0.1)  # 古い受信データを捨てる
    return {"scan": cmd_scan, "read": cmd_read, "set-id": cmd_set_id,
            "set-ram": cmd_set_ram}[args.command](bus, args)


if __name__ == "__main__":
    sys.exit(main())
