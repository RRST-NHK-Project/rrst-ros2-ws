# CubeMars_ctrl

CubeMars **AK40-10 V3.0 KV170** を、ESP32（soki基板 / XIAO ESP32-S3）経由のCAN通信で
**速度制御**・**角度制御**し、PS4コントローラで操作するためのROS 2 Jazzyパッケージ。

ファームはチームで実機動作済みの `ros2can/generated_firmware/soki_ak_11`（`MODE_CUBEMARS`）を
`firmware/soki_cubemars/` にコピーし、安全機能などを少しだけ追加したもの。

## 構成

```
PS4 ──/joy──> ps4_teleop_node ──cubemars/cmd_velocity [rpm]──┐
                                ──cubemars/cmd_angle [deg]───┤
                                ──cubemars/stop──────────────┤
                                                             v
                    cubemars_driver_node  (単位換算・モード管理・安全処理)
                         │ serial_tx_11 (Int16MultiArray x24)    ^ serial_rx_11
                         v                                       │
                    cubemars_bridge_node  (USBシリアル 52バイトフレーム)
                         │ /dev/ttyACM0
                         v
                    ESP32 soki基板 (MODE_CUBEMARS, DEVICE_ID=11)
                         │ CAN 1Mbps (拡張ID)
                         v
                    AK40-10 (CAN ID = config.hppのCUBEMARS_MOTOR_ID_1)
```

- `serial_tx_11` / `serial_rx_11` はros2canのGUIと同じ名前・型なので、ブリッジの代わりに
  ros2canのGUIを使うこともできる（逆も可）。ただし同じシリアルポートは同時に1つしか開けない。
- 速度制御はServoモードの**速度ループ**（モータ内蔵のPI制御）をそのまま使う。
- 角度制御は**MITモード**を使う（チームのsokiで実機動作実績あり。Kp/Kdで柔らかさを決められる）。

## 安全設計（どこが壊れても「その場停止」に落ちる）

| 層 | 何が起きたら | どうなる |
|:---|:---|:---|
| ps4_teleop_node | R1を離す / `/joy` が0.5秒途絶 | 停止指令 |
| cubemars_driver_node | 速度指令が0.5秒途絶 / モータ帰還が0.5秒途絶 / モータがエラー報告 | 停止 |
| cubemars_driver_node | 角度指令が範囲外 / 回転中・MIT範囲外で角度モードへ入ろうとした | 範囲内に切り詰め / 拒否 |
| cubemars_bridge_node | `serial_tx_11` が0.5秒途絶（ドライバ異常終了など） | 全スロット0（停止）を送信 |
| ESP32ファーム | 有効フレームが300ms途絶（PC側全体の異常・USB抜け） | ゼロ速度指令をモータへ送信 |

「停止」は全スロット0 = 速度ループで目標0rpm（ros2canのE-STOPと同じ）。

## ファイル

| パス | 内容 |
|:---|:---|
| `firmware/soki_cubemars/` | ESP32ファーム（PlatformIO）。`pio run -e cubemars` が通常運用、`pio run -e can_monitor` がCAN ID調査用 |
| `config/cubemars_ctrl.yaml` | 全ノードのパラメータ（最大回転数・角度範囲・Kp/Kd・ボタン番号など） |
| `launch/cubemars.launch.py` | ブリッジ＋ドライバ |
| `launch/teleop.launch.py` | joy_node＋PS4テレオペ（別ターミナルで起動） |
| `src/cubemars_driver_node.cpp` | 速度/角度指令→24スロット変換、安全処理、状態ログ |
| `src/bridge_node.cpp` | シリアル送受信（tmotor_ctrlのブリッジがベース） |
| `src/ps4_teleop_node.cpp` | PS4操作 |
| `src/selftest.cpp` | 実機なしの単体チェック（`ros2 run CubeMars_ctrl cubemars_selftest`） |
| `tools/fake_esp32.py` | 実機なしでROS側を動かすための疑似ESP32（`ros2 run CubeMars_ctrl fake_esp32.py`） |

## ファーム側の変更点（soki_ak_11 との差分）

1. 動作モードを `platformio.ini` の環境で切替（`config.hpp` のコメントアウト付け替えが不要）
2. `CUBEMARS_MOTOR_COUNT=1`、`CUBEMARS_MOTOR_ID_1` を実機のCAN IDへ
3. シリアル途絶フェイルセーフ（`CUBEMARS_CMD_TIMEOUT_MS=300`）
4. 帰還スロット20に「モータ帰還フレームの受信カウンタ」を追加（ROS側でモータの応答有無を判定するため）

3・4があるため、**ROS側はこのフォルダのファームとセットで使うこと**（元のsoki_ak_11ファームだと
受信カウンタが増えず、ドライバが「帰還なし」と判断して動かない）。

## 単位

| 量 | ROS側 | スロット（ESP32） |
|:---|:---|:---|
| 速度指令 | 出力軸rpm | 10ERPM/LSB。ERPM = rpm × 極対数(14) × 減速比(10) |
| 角度指令 | 出力軸deg | 0.1deg/LSB。MITモードでは±716.2degが上限（プロトコルの限界） |
| 位置帰還 | `cubemars/joint_state` の position [rad] | 0.1deg/LSB |
| 電流帰還 | `cubemars/joint_state` の effort [A]（トルクではない） | 0.01A/LSB |

極対数14は販売店の仕様表の値でV3.0での裏付けが無い。速度一定で回っているときに
ドライバのログに出る「ERPM/出力rpm比」が140付近なら正しい。

## 困ったときの切り分け

| 症状 | 見る場所 | 主な原因 |
|:---|:---|:---|
| ブリッジが「シリアルポートを開けません」 | `ls /dev/ttyACM*` | ポート名違い / ros2canのGUIや `pio device monitor` が開いたまま |
| ドライバが「モータからの帰還がまだありません」 | `ros2 topic hz /serial_rx_11` | 出ていない→ESP32/ブリッジ側。出ている→CAN配線・モータ電源・CAN ID |
| ブリッジが「DEVICE_ID=xx のフレームを受信」 | `config.hpp` と yaml の `device_id` | 書き込んだファームのDEVICE_IDが違う |
| 角度モードに入れない | ドライバのログ | 回転中 / 現在角度が±700deg外（速度モードで0deg付近へ戻す） |
| 指令とrpmが合わない | ログの「ERPM/出力rpm比」 | `pole_pairs` / `gear_ratio` の設定違い |
