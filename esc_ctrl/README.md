# esc_ctrl

`esc_ctrl` は serial_bridge 互換 ESC MCU に固定値コマンドを送り続け、受信したセンサ値をログ表示する最小ノードです。

## 動作

- 周期的に `serial_tx_[device_id]` へ 24 要素 int16 を送信
- `serial_rx_[device_id]` を受信し、角度/速度/目標値/電圧制限をコンソール表示
- 実行中にキーボードから司令を変更可能
  - `v100` : 速度モードで 100 rad/s 指令
  - `p90` : 角度モードで 90 deg 指令

加えて、Webコンソール等から以下のトピックで指令/ゲインを更新できます。

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

追加(ゲイン/リミットの初期値。コンソールから随時更新可能):

- `velocity_limit` (double, default: 1500.0)
- `current_limit` (double, default: 10.0)
- `velocity_pid_p` (double, default: 0.02)
- `velocity_pid_i` (double, default: 0.0)
- `velocity_pid_d` (double, default: 0.0)
- `velocity_pid_output_ramp` (double, default: 1000.0)
- `velocity_lpf_tf` (double, default: 0.02)
- `angle_p_gain` (double, default: 8.0)

## トピック

- Publish: `serial_tx_[device_id]` (`std_msgs/msg/Int16MultiArray`)
- Subscribe: `serial_rx_[device_id]` (`std_msgs/msg/Int16MultiArray`)

コンソール入力:

- Subscribe: `/esc_ctrl/cmd` (`std_msgs/msg/Float32MultiArray`)
  - payload: `[enable, mode, target, voltage_limit]`
  - `enable`: 0/1
  - `mode`: 0=velocity, 1=angle
  - `target`: mode=0 → rad/s, mode=1 → deg
  - `voltage_limit`: V

- Subscribe: `/esc_ctrl/cmd/params` (`std_msgs/msg/Float32MultiArray`)
  - payload: `[voltage_limit, velocity_limit, current_limit, velP, velI, velD, velRamp, velLpfTf, angleP]`
  - `velLpfTf` は秒(s)

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
