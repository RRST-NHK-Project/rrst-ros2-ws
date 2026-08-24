# encoder_angle

`ros2can` の `serial_rx_[DEVICE_ID]_unwrapped` (`Int32MultiArray`、ros2can側で
既に連続値化済み) から、汎用IOノード (`xiao-esp32-s3_can2io`) のロータリー
エンコーダ入力 `ENC1`/`ENC2` スロットだけを取り出し、連続角度 (deg) として
配信する独立ノードです。

## アーキテクチャ

PCNT の生カウント(int16)のラップ検出(0クリアの補正)は `ros2can` 本体側
(`ros2can/ros2can/counter_unwrapper.py`)が、シリアルフレームを受信した
直後のサンプルが密な場所(`RosBackend._on_hardware_frame`/
`service_simulators`)で行い、`serial_rx_[ID]_unwrapped` として配信する。
本パッケージはそれを購読し、`ticks * (360 / encoder_ppr) * sign` の単純な
スケール変換のみを行う。

以前は本パッケージ自身がラップ検出まで行っていたが、ROSトピック経由
(GUIスレッドの詰まり等でサンプルが疎になりうる)で下流にラップ検出を
任せると、欠落量がちょうど半周期に近い場合に誤判定しうることが実機検証で
判明したため、サンプルが密な `ros2can` 側に移設した。

## なぜ「連続値化」が必要か (ESP32 PCNT の仕様、実機で確認済み)

`ENC1`/`ENC2` の生値は、`xiao-esp32-s3_can2io` の PCNT (パルスカウンタ) が
出力する `int16_t` の生カウントです。ファームウェア
(`pin_ctrl_init.cpp`) は `pcnt_config_t::counter_h_lim`/`counter_l_lim` に
`int16_t` の最大/最小値 (32767 / -32768) を設定していますが、
`pcnt_event_enable()` を一度も呼んでいません。ESP-IDF の
`pcnt_unit_config()` の doxygen コメントには「この関数は
`PCNT_EVT_L_LIM`/`PCNT_EVT_H_LIM`/`PCNT_EVT_ZERO` を無効化する」と書かれて
いるため、当初は「イベント無効 = 上限/下限到達時の自動0クリアも無効になり、
生値は素の2の補数として±32768間でオーバーフローする」と推測したが、これは
誤りだった。**実機でエンコーダーを負方向へ回転させ ros2can の
`serial_rx_[ID]` を直接観測したところ、生値は `-32767` に達すると
(2の補数の `-32768` を経由せず) そのまま `0` へ戻り、`0` から再び負方向へ
カウントが続くことを確認した。** つまり「イベント(割り込み通知)の無効化」と
「h_lim/l_lim到達時の0クリア動作」は別物で、後者はイベントの有効/無効に
関わらず常時作動している。h_lim=32767, l_lim=-32768 という設定から、この
リセット幅は正負どちらの方向も **32768** (= h_lim+1 = -l_lim) で対称になる
(単純な2の補数オーバーフロー=全域65536幅の1本のリング、とは異なる)。

詳細なアルゴリズム(欠落量が半周期に近い場合の曖昧判定への対処含む)は
`ros2can/ros2can/counter_unwrapper.py` 冒頭コメントを参照してください。

## 角度への換算

```
angle_deg = ticks * (360 / encoder_ppr) * sign
```

`encoder_ppr` は1回転あたりの生カウント数 (PCNTのx4逓倍後)。ファームウェア
既定 (`ENC_PPR_SPEC=2048` → `PPR=8192`) に合わせて既定値を `8192.0` に
しています。実機のエンコーダ規格値に応じて `config/encoder_angle.yaml` で
変更してください。

## トピック

- 購読: `/serial_rx_[device_id]_unwrapped` (`std_msgs/Int32MultiArray`,
  ros2can が配信、既に連続値化済み)
- 配信 (相対名、ノード毎に分離):
  - `encoder1/angle_deg`, `encoder1/ticks`
  - `encoder2/angle_deg`, `encoder2/ticks`

## esc_ctrl への経路

`esc_ctrl_node` は位置フィードバックとして `serial_rx_[feedback_device_id]_unwrapped`
(Int32MultiArray) の `feedback_slot` を直接購読する(`esc_ctrl.yaml` の
`encoder_ppr`/`feedback_sign` で独自にdeg換算する)ため、本パッケージを
経由しなくても ros2can → esc_ctrl の経路は繋がる。本パッケージは
odometryやロギングなど、`encoder1/angle_deg` のような単体の角度トピックが
別途必要な用途のために用意している。

## ビルド・起動

```bash
cd ~/ros2_ws
colcon build --packages-select encoder_angle
source install/setup.bash
ros2 launch encoder_angle encoder_angle.launch.py
```
