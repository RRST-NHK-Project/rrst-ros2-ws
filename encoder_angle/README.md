# encoder_angle

`ros2can` の `serial_rx_[DEVICE_ID]` (`Int16MultiArray`) から、汎用IOノード
(`xiao-esp32-s3_can2io`) のロータリーエンコーダ入力 `ENC1`/`ENC2` スロット
だけを取り出し、連続角度 (deg) として配信する独立ノードです。`ros2can` 本体
(サブモジュール) には手を入れません。

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

このため生カウントをそのまま角度に換算すると、一周(や複数周)ごとに
ジャンプが発生します。`CounterUnwrapper`
(`include/encoder_angle/counter_unwrapper.hpp`) はサンプル間の差分を監視し、
半周期(16384カウント = 32768/2)を超える跳躍をラップとみなして補正することで、
連続値 (ticks) に復元します。欠落量がちょうど半周期に近いと「絶対値最小」
判定だけでは原理的に誤判定しうるため、差分が明らかに曖昧な場合に限り直近の
移動方向(符号)を優先する補助判定を入れています(静止中の微小ノイズを
1周期分の大ジャンプと誤判定しないよう、曖昧でない場合は使いません)。

## 角度への換算

```
angle_deg = ticks * (360 / encoder_ppr) * sign
```

`encoder_ppr` は1回転あたりの生カウント数 (PCNTのx4逓倍後)。ファームウェア
既定 (`ENC_PPR_SPEC=2048` → `PPR=8192`) に合わせて既定値を `8192.0` に
しています。実機のエンコーダ規格値に応じて `config/encoder_angle.yaml` で
変更してください。

## トピック

- 購読: `/serial_rx_[device_id]` (`std_msgs/Int16MultiArray`, ros2can が配信)
- 配信 (相対名、ノード毎に分離):
  - `encoder1/angle_deg`, `encoder1/ticks`
  - `encoder2/angle_deg`, `encoder2/ticks`

## ビルド・起動

```bash
cd ~/ros2_ws
colcon build --packages-select encoder_angle
source install/setup.bash
ros2 launch encoder_angle encoder_angle.launch.py
```

## テスト

```bash
colcon test --packages-select encoder_angle
```
