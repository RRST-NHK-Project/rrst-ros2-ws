# esc_ctrl

`esc_ctrl` は serial_bridge 互換 ESC MCU に固定値コマンドを送り続け、受信したセンサ値をログ表示する最小ノードです。

## 動作

- 周期的に `serial_tx_[device_id]` へ 24 要素 int16 を送信
- `serial_rx_[device_id]` を受信し、角度/速度/目標値/電圧制限をコンソール表示
- 実行中にキーボードから司令を変更可能
  - `v100` : 速度モードで 100 rad/s 指令
  - `p90` : 角度モードで 90 deg 指令

## パラメータ

- `device_id` (int, default: 1)
- `tx_period_ms` (int, default: 20)
- `log_period_ms` (int, default: 200)
- `fixed_enable` (int, default: 1)
- `fixed_mode` (int, default: 0)
  - `0`: 速度モード (`fixed_target` の単位は rad/s)
  - `1`: 角度モード (`fixed_target` の単位は deg)
- `fixed_target` (double, default: 5.0)
- `fixed_voltage_limit` (double, default: 12.0)

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
