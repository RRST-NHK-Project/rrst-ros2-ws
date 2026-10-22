# robstride_ctrl

RobStride EduLite 05 (EL05) を ESP32 経由の CAN 通信で速度制御・角度制御するための
ROS 2 (Jazzy) パッケージ。PS4 コントローラ (USB 有線) でのテレオペにも対応する。
構成は damiao_ctrl (2026-10-14 実機で全項目を確認済み) と同じ。

## アーキテクチャ

```
[PS4コントローラ]--USB-->[joy_node]--/joy-->[ps4_teleop_node (C++)]
                                                  | robstride/motor1/cmd
[motion_test.py (rclpy)] -------------------------+
                                                  v
[PC: robstride_driver_node (C++)] --robstride/motor1/state--> [motor_monitor (C++)]
        |  serial_tx_202 / serial_rx_202 (Int16MultiArray, 24要素)
        v
[PC: ros2can --nogui] <--USBシリアル--> [ESP32 (SOKI基板, XIAO ESP32-S3)]
                                               | CAN 1Mbps, 29bit拡張ID
                                               v
                                        [RobStride EduLite 05]
```

- PC と ESP32 の間は RRST-NHK-Project/ros2can を使う (`~/claude_ws/src/ros2can`)。
  ESP32 には ros2can の `firmware/xiao-esp32-s3_can2io` を `MODE_ROBSTRIDE`・`DEVICE_ID 202`
  で書き込む。設定は同じファームの `config.hpp` にある `ROBSTRIDE_*`。
- ESP32 と EL05 の間は RobStride の独自 (私有) プロトコルを使う。
  - 速度は速度モード (run_mode=2) で、`spd_ref` を約 200Hz で書き込む。
  - 角度は位置モード CSP (run_mode=5) で、`loc_ref` に目標角度、`limit_spd` に速度上限を書き込む。
- 位置は出力軸の多回転位置 (`mechPos`) を使う。帰還フレームの角度は ±4π で折り返すため使わない。

## 配線

- EL05 の XT30(2+2) コネクタは、VBAT+ (15–60V、定格 48V)、GND、CAN-H、CAN-L の 4 本。
- CAN-H と CAN-L を SOKI 基板の CAN 端子につなぐ。
- SOKI 基板の TERM ジャンパは必ず付ける。Damiao のとき、これが無くて通信できなかった。
- モータ側に終端抵抗が入っているかは説明書に書かれていない。電源 OFF で CAN-H と CAN-L の間を測る:
  - 約 60Ω なら両端に終端がある。
  - 約 120Ω なら終端は片側だけ。

## スロット割り当て (`include/robstride_ctrl/slot_layout.hpp`)

24 スロットを、8 スロット/モータ × 最大 2 台と、診断用の 4 スロット (20–23) に分けている。
ESP32 ファーム (`robstride.cpp`) と必ず一致させること。

| local | 方向 | 内容 | 単位/LSB |
|---:|:---|:---|:---|
| 0 | ROS→ESP32 | mode_word (0=DISABLE, 1=速度, 2=位置) | - |
| 1,2 | ROS→ESP32 | target (int32、1 が上位) 速度は rad/s、位置は rad | 0.001 |
| 3 | ROS→ESP32 | vel_limit (位置モードの速度上限) | 0.01 rad/s |
| 0 | ESP32→ROS | モータのモード (0=停止, 1=校正, 2=運転) | - |
| 1 | ESP32→ROS | 故障フラグ (下位 6bit=モータの故障、0x100=ESP32 が止めた、0x200=位置未取得) | - |
| 2 | ESP32→ROS | 帰還の受信回数 (止まったらモータの返事が無い) | - |
| 3,4 | ESP32→ROS | 位置 (int32、多回転) | 0.001 rad |
| 5 | ESP32→ROS | 速度 | 0.01 rad/s |
| 6 | ESP32→ROS | トルク | 0.001 N・m |
| 7 | ESP32→ROS | 温度 | 0.1 ℃ |
| slot 20–23 | ESP32→ROS | ID スキャン ([20]=応答した台数、[21–23]=CAN ID) | - |

全スロット 0 は「DISABLE」になる。起動直後と通信が途絶えたときの既定値なので、その状態では必ずモータが止まる。

## トピック

| トピック | 型 | 説明 |
|:---|:---|:---|
| `robstride/motor1/cmd` | Float32MultiArray `[control_mode, target, vel_limit]` | 下の表を参照 |
| `robstride/motor1/state` | Float32MultiArray `[位置rad, 速度rad/s, トルクN・m, モード, 帰還OK, 故障フラグ, 温度℃]` | モータの帰還 |
| `serial_tx_202` / `serial_rx_202` | Int16MultiArray | PC と ESP32 の間の生の 24 スロット (ros2can が中継する) |

`control_mode` の値と `target`・`vel_limit` の意味:

| control_mode | 意味 | target | vel_limit |
|---:|:---|:---|:---|
| -1 | DISABLE | 使わない | 使わない |
| 0 | 速度 | 速度 [rad/s] | 使わない |
| 1 | 位置 | 位置 [rad] | 移動速度の上限 [rad/s] |

cmd が `cmd_timeout_sec` (既定 0.5 秒) より長く届かないと、自動で DISABLE になる。動かし続けるときは cmd を周期的に送ること。

## ビルド

```bash
cd ~/claude_ws && colcon build --packages-select robstride_ctrl && source install/setup.bash
```

## 実行

どのターミナルでも、最初に `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` を実行すること。
付けないと、同じ Wi-Fi にいる他の PC のノードとつながってしまう。

```bash
# ターミナル1: PCとESP32の通信
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && source ~/claude_ws/install/setup.bash && ros2 run ros2can ros2can --nogui
```

```bash
# ターミナル2: 駆動ノード (PS4 を使うときは use_joy:=true を付ける)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && source ~/claude_ws/install/setup.bash && ros2 launch robstride_ctrl robstride_ctrl.launch.py
```

```bash
# ターミナル3: 表示 (指令は送らない)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && source ~/claude_ws/install/setup.bash && ros2 run robstride_ctrl motor_monitor
```

```bash
# 動作確認 (rclpy)。角度: 今の位置から +0.5rad 動いて戻る (0.5rad/s)。速度: ±0.5rad/s を 3 秒ずつ
ros2 run robstride_ctrl motion_test.py angle 0.5 0.5
ros2 run robstride_ctrl motion_test.py speed 0.5 3
```

## 安全上の注意

止まる仕組みは次のとおり。ソフト上では偽の ESP32 で確認した (2026-10-22)。実機ではまだ確認していない。

| 起きたこと | 止まる仕組み |
|:---|:---|
| 上位ノード (PS4・motion_test) が落ちた | robstride_driver_node の見張り (`cmd_timeout_sec`=0.5秒) |
| robstride_driver_node を Ctrl+C で終了した | 終了直前に DISABLE を送る |
| ros2can が落ちた、USB が抜けた | ESP32 の見張り (`ROBSTRIDE_CMD_TIMEOUT_MS`=300ms) |
| ESP32 が固まった、CAN 線が抜けた | モータ自身の CAN タイムアウト (`ROBSTRIDE_MOTOR_TIMEOUT_MS`=100ms) |
| 運転中にモータが止まった (故障・電源断) | ESP32 が停止を保持し、PC が DISABLE を送るまで再始動しない |

- **DISABLE は「力を抜く」停止**で、ブレーキはかけない。速く回っているときに DISABLE すると惰性で回る。
- **位置モードの目標は多回転の絶対位置 [rad]。** 必ず「今の位置 + 移動量」で指令すること (motion_test.py はそうしている)。
  - 位置を読めていない間は、driver が位置指令を受け付けない。
- **最高速度は電源電圧で決まる** (参考資料の実測)。
  - 24V で約 23.5 rad/s。
  - 48V で約 45 rad/s。
  - driver の上限 `max_vel_radps` は 20 rad/s にしてある。
- **電流上限 `ROBSTRIDE_LIMIT_CUR_A` は 3A** (初回確認用。工場値は 11A)。トルクが足りないときに上げる。
- 参考資料の実機では、モータの CAN ID は 127 (工場値。説明書の「1」とは違った)。
  - ID が合わないと帰還が来ない。
  - ESP32 が全 ID をスキャンし、driver のログに「ESP32のIDスキャン: 1台応答 (CAN ID: …)」と出る。その値を `config.hpp` の `ROBSTRIDE_MOTOR_ID_1` に書く。

## 参考にした資料

- EL05 説明書 (CN v1.1 / EN v1.0) を実機で確かめながら整理した資料: github.com/mocchan0221/EDULITE05_Controller の `docs/research/el05_protocol.md`、`hw_bench.md`
- 実装の作り (安全機能・PS4・表示): `damiao_ctrl` と ros2can の `MODE_DAMIAO`

## 現在の状態

- [x] ファームのビルド (`pio run`) と ROS パッケージのビルド (警告なし)
- [x] 偽の ESP32 での動作確認: 角度往復・速度往復・Ctrl+C 停止・位置未取得時の拒否・cmd 途絶・PS4 ロジック
- [ ] 実機: CAN 通信と ID の確認 → 速度 → 角度 → PS4
