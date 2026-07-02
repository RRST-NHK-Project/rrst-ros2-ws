# esc_ctrl

`esc_ctrl` は serial_bridge 互換 ESC MCU にコマンドを送り、受信したセンサ値を確認するためのパッケージです。

## 動作

- 周期的に `serial_tx_[device_id]` へ 24 要素 int16 を送信
- `serial_rx_[device_id]` を受信し、角度/速度/目標値/電圧制限をコンソール表示
- 実行中にキーボードから司令を変更可能
  - `v1000` : 速度モードで 1000 rpm 指令
  - `p90` : 角度モードで 90 deg 指令
- Host 風の開発雛形ノード `esc_ctrl_dev_frame` を同梱

## パラメータ

- `device_id` (int, default: 1)
- `tx_period_ms` (int, default: 20)
- `log_period_ms` (int, default: 200)
- `fixed_enable` (int, default: 1)
- `fixed_mode` (int, default: 0)
  - `0`: 速度モード (`fixed_target` の単位は rpm)
  - `1`: 角度モード (`fixed_target` の単位は deg)
- `fixed_target` (double, default: 5.0)
- `fixed_voltage_limit` (double, default: 12.0)

## 開発雛形

- `esc_ctrl_node`: 最小の固定値送信ノード
- `esc_ctrl_dev_frame`: `teleop_tools` の Host 風に、Joy 入力・送信フレーム・受信処理の編集ポイントを残した開発用ノード
- `esc_position_pid_node`: `esc_cmd_pos_<id>` のカスタム配列コマンドを受けて、`common` の PID で位置決めするノード

`esc_ctrl_dev_frame` では以下をすぐ編集できます。

- `on_joy()` : コントローラ入力から ESC 指令値を組み立てる
- `on_serial_rx()` : マイコンからの受信値を使った処理を書く
- `set_velocity_rpm()` / `set_angle_deg()` / `set_voltage_limit_volts()` : 単位付きでコマンドを設定する

## 位置 PID ノード

- executable: `esc_position_pid_node`
- command topic: `esc_cmd_pos_<device_id>` (`std_msgs/msg/Float64MultiArray`)
- command format: `[target_angle_deg, max_command_rpm]`
- feedback topic: `serial_rx_[device_id]`
- output topic: `serial_tx_[device_id]`
- optional parameter: `encoder_ppr` (`> 0` のとき `serial_rx[kRxAngle]` を符号付き 16bit のエンコーダカウントとして解釈)

`esc_position_pid_node` は `device_id` ごとに `esc_cmd_pos_<id>` を購読します。各ノードは自分の ID 用トピックだけを受け、`common/PID.hpp` の `PIDController` で位置 PID を回して velocity モードの速度指令を `serial_tx_<id>` に送ります。

既定では受信角度をマイコン側の角度値として扱い、このノード側で折り返しを検出して連続角に展開します。そのため `360 deg` を超える目標値でも連続回転できます。連続角の基準はノード起動後の最初の受信値です。

ESP32 の PCNT 出力のように `serial_rx[kRxAngle]` が符号付き 16bit の生カウント値で来る場合は、`encoder_ppr` を設定してください。`encoder_ppr` を正値にすると、このノードは int16 のオーバーフローを補正しながらカウント差分を連続化し、`360 / encoder_ppr` で度に変換して位置 PID に使います。

例:

- topic: `esc_cmd_pos_153`
- `data: [90.0, 800.0]`
- `device_id=153` のノードはこのコマンドを受けて `90 deg` へ最大 `800 rpm` で位置決め

例: ESP32 PCNT が x4 カウントで 8192 count/rev のとき

- `encoder_ppr: 8192.0`

要素数が 2 未満のコマンドは無視します。複数 ID を制御したい場合は、ID ごとに別ノードまたは別 `device_id` インスタンスを立てます。

## トピック

- Publish: `serial_tx_[device_id]` (`std_msgs/msg/Int16MultiArray`)
- Subscribe: `serial_rx_[device_id]` (`std_msgs/msg/Int16MultiArray`)

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select esc_ctrl
source install/setup.bash
```

## Run

```bash
ros2 launch esc_ctrl esc_ctrl.launch.py
```

```bash
ros2 launch esc_ctrl esc_ctrl_dev_frame.launch.py
```

```bash
ros2 launch esc_ctrl esc_position_pid.launch.py
```
