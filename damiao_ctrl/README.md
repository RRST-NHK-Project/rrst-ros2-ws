# damiao_ctrl

Damiao DM2325(モータ) + DM3520(ESC) を ESP32 経由のCAN通信で
速度制御・角度制御するための ROS 2 (Jazzy) パッケージ。PS4コントローラ
(USB有線)でのテレオペにも対応。

## アーキテクチャ

```
[PS4コントローラ]--USB-->[joy_node]--/joy-->[ps4_teleop_node]
                                                  |
                                                  | damiao/motor1/cmd
                                                  v
[PC: damiao_driver_node] <--damiao/motor1/state--+
        |  serial_tx_201 (Int16MultiArray, 24要素)
        v
[PC: ros2can --nogui] <--USBシリアル(52byteフレーム)--> [ESP32 (SOKI基板, XIAO ESP32-S3)]
        ^  serial_rx_201                                       |
        |                                                      v CAN 1Mbps (TWAI + MCP2561)
        +------------------------------------------  [DM3520 ESC] -- [DM2325]
```

- PC-ESP32間は RRST-NHK-Project/ros2can を使う(`~/claude_ws/src/ros2can`)。
  ESP32には ros2can の `firmware/xiao-esp32-s3_can2io` を `MODE_DAMIAO`・
  `DEVICE_ID 201` で書き込む(Damiaoの設定は同ファームの `config.hpp` の
  `DAMIAO_*`)。旧来の `bridge_node` は `use_builtin_bridge:=true` の時だけ使う。
- ESP32-DM3520間は達妙(Damiao)公式のCANプロトコル(VEL/POS_VELモード)。
  帰還は Master ID(0x000)で届く。

## スロット割り当て (`include/damiao_ctrl/slot_layout.hpp`)

24スロットを6スロット/モータ x 最大4台に分配(将来複数台に拡張可能)。
モータ1は先頭6スロット(index 0-5)を使用。

| local index | 方向 | 内容 | 単位/LSB |
|---:|:---|:---|:---|
| 0 | ROS→ESP32 | mode_word (0=DISABLE, 1=VEL, 2=POS_VEL) | - |
| 1 | ROS→ESP32 | target (VEL:rad/s, POS_VEL:rad) | 0.001 |
| 2 | ROS→ESP32 | vel_limit (POS_VELのみ有効) | 0.001 rad/s |
| 0 | ESP32→ROS | motor_state (0=Disable, 1=Enable, それ以外=故障。13=通信喪失) | - |
| 1 | ESP32→ROS | fb_counter (帰還を受け取るたび+1。止まったら帰還途絶) | - |
| 3 | ESP32→ROS | position (出力軸、±12.5rad) | 0.001 rad |
| 4 | ESP32→ROS | velocity (出力軸、モータ側分解能約0.098rad/sで静止中も±0.049) | 0.001 rad/s |
| 5 | ESP32→ROS | torque | 0.001 N・m |

**`mode_word=0`(全スロット0)が起動直後・通信途絶時の既定値**。Damiaoモータは
明示的なEnable(0xFC)コマンドを送るまでCAN指令を無視する仕様のため、この
既定値では確実にモータが無効状態(=安全)になる。

## トピック一覧

| トピック | 型 | 説明 |
|:---|:---|:---|
| `serial_tx_201` / `serial_rx_201` | `std_msgs/Int16MultiArray` | PC-ESP32間の生の24スロット(ros2canが中継) |
| `damiao/motor1/cmd` | `std_msgs/Float32MultiArray` [control_mode, target, vel_limit] | 上位からの指令。control_mode: -1=DISABLE, 0=VEL, 1=POS_VEL。**`cmd_timeout_sec`(既定0.5秒)以上届かないと自動でDISABLE**になるので、動かし続けるには周期的に送ること |
| `damiao/motor1/state` | `std_msgs/Float32MultiArray` [position, velocity, torque, motor_state, feedback_ok] | モータ帰還値。feedback_ok: 1=帰還あり, 0=途絶 |
| `/joy` | `sensor_msgs/Joy` | joy_nodeが出すPS4コントローラの生入力 |

## ビルド

```bash
cd ~/claude_ws
colcon build --packages-select damiao_ctrl
source install/setup.bash
```

## 実機なしでの単体テスト

```bash
ros2 run damiao_ctrl frame_codec_selftest
```

52バイトフレームの往復変換・ノイズ耐性(ゴミバイト混入・チェックサム破損か
らの再同期)を検証する。実機不要。

## 実行

```bash
# ターミナル1: PC<->ESP32の通信(ros2can)
ros2 run ros2can ros2can --nogui
```

```bash
# ターミナル2: 駆動ノード(PS4はまだ使わない)
ros2 launch damiao_ctrl damiao_ctrl.launch.py
```

```bash
# 動作確認の例: 0.5rad/sで回す(Ctrl+Cで止めると0.5秒後に自動DISABLE)
ros2 topic pub -r 20 damiao/motor1/cmd std_msgs/msg/Float32MultiArray "{data: [0.0, 0.5, 0.0]}"
```

## 安全上の注意

止まる仕組み(2026-10-14 実機確認済み):

| 起きたこと | 止まる仕組み | 止まるまで |
|:---|:---|:---|
| 上位ノード(PS4等)が落ちた・cmdが途絶えた | damiao_driver_nodeの見張り(`cmd_timeout_sec`) | 約0.7秒 |
| damiao_driver_nodeをCtrl+Cで終了 | 終了直前にDISABLEを送信(pre-shutdown) | 約0.2秒 |
| ros2canが落ちた・USBが抜けた | ESP32ファームの見張り(`DAMIAO_CMD_TIMEOUT_MS`=300) | 0.3秒 |
| ESP32が固まった・CAN線が抜けた | モータ自身の通信タイムアウト(`DAMIAO_MOTOR_TIMEOUT_MS`=100) | 0.1秒 |

- 故障(通信喪失を含む)でモータが止まった場合、指令を送り続けても再始動しない。
  復帰は「一度DISABLE(-1)を送る → 改めて指令する」。
- **注意: damiao_driver_nodeが`kill -9`等で強制終了した場合**、ros2can(--nogui)は
  最後に受け取った指令を送り続けるため、モータは止まらない。その場合はros2canを
  止める(Ctrl+C)とESP32の見張りで止まる。
- `ps4_teleop_node`(`use_joy:=true` で起動)の操作:
  - **R1を押している間だけ**左スティック上下で速度指令(最大 `max_vel_radps`、既定1.0rad/s)。
    R1を離すと即DISABLE。
  - **Options = E-STOP**。再開はR1を一度離して押し直す。
  - R1を押したまま起動・復帰しても動かない(必ずR1の押し直しが必要)。
  - `/joy` が0.5秒途絶える(コントローラのUSB抜け等)と停止。
  - モータが故障で止まった時も、R1を離して押し直せば復帰する。
- ボタン番号(DualShock 4をUSB有線で確認済み): R1=5, Options=9, 左スティック上下=軸1(上が+)。
  別のコントローラ・接続方法(Bluetooth等)では番号が変わることがあるので、
  `ros2 topic echo /joy` でボタンを押して確認し、`config/damiao_ctrl.yaml` を合わせること。
- コントローラのUSB接続が不安定(カーネルログに `error -71`、30秒ごとに切断)な場合は
  ケーブルを交換する(2026-10-14、ケーブル交換で解消)。

## 現在の状態 / TODO

- [x] ros2can(MODE_DAMIAO)経由でCAN通信・角度制御(POS_VEL)・速度制御(VEL)を実機確認
- [x] PMAX/VMAX/TMAX=12.5/200/10、減速比25をモータのレジスタから読み出して確認
- [x] 停止系(上表)を実機確認
- [x] damiao_driver_node を新しいスロット配置(状態コード・受信回数)に対応
- [x] ps4_teleop_node の安全機能(デッドマン・E-STOP・押し直し・joy途絶・Ctrl+C)を偽の/joyで確認
- [ ] PS4コントローラ実機でのボタン番号確認・テレオペ確認
