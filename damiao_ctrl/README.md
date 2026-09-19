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
        |  serial_tx_1 (Int16MultiArray, 24要素)
        v
[PC: bridge_node] <--USBシリアル(115200bps, 52byteフレーム)--> [ESP32]
        ^  serial_rx_1                                            |
        |                                                         v CAN(TWAI)
        +---------------------------------------------  [DM3520 ESC] -- [DM2325]
```

- PC-ESP32間は RRST-NHK-Project/ros2can と同じ52バイト固定長フレーム
  (`0xAA, DEVICE_ID, LEN=48, int16x24 big-endian, XORチェックサム`)を使用。
  ros2can本体には依存しない(フレーム形式だけ合わせてある)。
- ESP32-DM3520間は達妙(Damiao)公式のCANプロトコル(MIT/POS_VEL/VELモード)。
  今回は VELモード・POS_VELモードのみ使用(MITモードはKp/Kd調整が必要で
  初回動作確認には不向きなため見送り)。

## スロット割り当て (`include/damiao_ctrl/slot_layout.hpp`)

24スロットを6スロット/モータ x 最大4台に分配(将来複数台に拡張可能)。
モータ1は先頭6スロット(index 0-5)を使用。

| local index | 方向 | 内容 | 単位/LSB |
|---:|:---|:---|:---|
| 0 | ROS→ESP32 | mode_word (0=DISABLE, 1=VEL, 2=POS_VEL) | - |
| 1 | ROS→ESP32 | target (VEL:rad/s, POS_VEL:rad) | 0.001 |
| 2 | ROS→ESP32 | vel_limit (POS_VELのみ有効) | 0.001 rad/s |
| 3 | ESP32→ROS | position | 0.001 rad |
| 4 | ESP32→ROS | velocity | 0.001 rad/s |
| 5 | ESP32→ROS | torque | 0.001 N・m |

**`mode_word=0`(全スロット0)が起動直後・通信途絶時の既定値**。Damiaoモータは
明示的なEnable(0xFC)コマンドを送るまでCAN指令を無視する仕様のため、この
既定値では確実にモータが無効状態(=安全)になる。

## トピック一覧

| トピック | 型 | 説明 |
|:---|:---|:---|
| `serial_tx_1` / `serial_rx_1` | `std_msgs/Int16MultiArray` | PC-ESP32間の生の24スロット |
| `damiao/motor1/cmd` | `std_msgs/Float32MultiArray` [control_mode, target, vel_limit] | 上位からの指令。control_mode: -1=DISABLE, 0=VEL, 1=POS_VEL |
| `damiao/motor1/state` | `std_msgs/Float32MultiArray` [position, velocity, torque] | モータ帰還値 |
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
ros2 launch damiao_ctrl damiao_ctrl.launch.py serial_port:=/dev/ttyUSB0
# PS4を使わない場合: use_joy:=false
```

## 安全上の注意

- 初回通電時は `damiao/motor1/cmd` を一切publishしない状態(=DISABLE)で
  まずESP32↔PC間の通信(`serial_rx_1`)とESP32↔DM3520間のCAN疎通(帰還値が
  正しく届くか)を確認してから、初めて低速でのVELモード指令を試すこと。
- `ps4_teleop_node` はデッドマンスイッチ方式(既定: R1を押している間だけ
  有効、Optionsボタンで強制DISABLE)。実際のボタン/軸番号はドライバに
  依存するため、使用前に必ず `ros2 topic echo /joy` でPS4のボタンを押して
  実際のindexを確認し、`config/damiao_ctrl.yaml` を合わせること。
- ノード終了時(Ctrl+C)は自動的にDISABLE指令が送られるが、ESP32ファーム
  ウェア側にCAN/シリアル途絶に対するフェイルセーフは実装しない予定
  (チームのros2can/cubemarsファームウェアも同様の設計)。ノードが異常終了
  (kill -9等)した場合は最後の指令を保持し続ける点に注意。

## 現在の状態 / TODO

- [x] PC側: フレームコーデック・ブリッジノード・駆動ノード・PS4テレオペノード実装、単体テスト済み(実機不要分)
- [ ] ESP32ファームウェア本体(`firmware/esp32_damiao_bridge/src/`)は未実装。
      ボード型番確定後に着手する: TWAI初期化、52バイトフレーム受信、
      Damiao CANプロトコル(Enable/Disable/VEL/POS_VEL送信、フィードバック
      受信、起動時にPMAX/VMAX/TMAXをモータから読み出し)。
- [ ] `firmware/esp32_damiao_bridge/include/config.hpp` のCAN GPIOピン・
      CANビットレート・DAMIAO_MOTOR_CAN_IDを実機に合わせて確定する。
