# ros2can

`xiao_esp32_s3_smd_serial_bridge` (MODE_CAN_HOST) 専用のデバッグ GUI です。
この基板は USB シリアルで `serial_bridge` (ROS 2) とつながる「CANホスト」で、
自身の配下に CAN バス経由で最大4台の子マイコン(ノード)をぶら下げます。
本ツールはホストの `serial_tx_[ID]` / `serial_rx_[ID]` トピックを介して、
バス上の各ノードへアクチュエータ指令を直接送信したり、センサ値をリアルタイムに
表示するための PyQt5 デスクトップアプリです。

## 前提

- `serial_bridge` ノードが起動しており、`xiao_esp32_s3_smd_serial_bridge`
  (MODE_CAN_HOST) を書き込んだ基板が USB 接続されていること。
- `python3-pyqt5` がインストールされていること (`sudo apt install python3-pyqt5`)。

## ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select ros2can
source install/setup.bash
```

## 起動

```bash
ros2 run ros2can ros2can
```

または

```bash
ros2 launch ros2can ros2can.launch.py
```

`serial_bridge` が起動していれば、検出された `serial_tx_[ID]`/`serial_rx_[ID]`
のペアが自動的に左のデバイス一覧に表示されます。まだ接続されていない
DEVICE_ID を先に登録しておきたい場合は、ツールバーの「デバイスを手動追加」から
追加できます。

## 画面構成

- **デバイス一覧 (左)**: 検出済みの CAN ホスト (DEVICE_ID ごと)。
  接続中/未接続、TX有効(送信中)状態を表示します。
- **Control タブ**: ノード1〜4のサブタブで「どのマイコン(CANノード)を
  操作するか」を選び、各ノードの `MD1-4`(DCモータPWM) と `SERVO1`(サーボ角度)
  を直接送信できます。実際に送信するには右上の「TX有効化」チェックを
  ONにする必要があります (誤操作防止のため既定はOFF)。
- **Monitor タブ**: 同じくノードごとのサブタブで `SW1-3`(スイッチ入力) と
  `ENC1-2`(エンコーダカウンタ) をリアルタイム表示します。
- **Raw タブ**: CAN分配を意識せず、ホストが送受信する生の24 x int16スロットを
  そのまま編集/確認できます。未対応スロットのデバッグや、プロファイルの
  想定と実機がズレている場合の確認に使用します。
- **Info タブ**: 接続状態、RX周波数、送受信フレーム数、現在の生データ配列を
  表示します。不具合報告時にそのままコピーできます。

## プロファイル

既定では下記2種類のプロファイルを切り替えられます。

| プロファイル | 内容 |
|---|---|
| XIAO ESP32S3 SMD (CAN Host) | 24スロットを4ノード x 5スロットに分配 (既定の `CAN_NODE_COUNT=4`, `CAN_SLOTS_PER_NODE=5` に対応) |
| 汎用 Raw | CAN分配を意識しない生の24スロット |

ファームウェア側の `config.hpp` で `CAN_NODE_COUNT` / `CAN_SLOTS_PER_NODE` を
変更した場合は、デバイスパネル右上の「プロファイル編集」からノード数・
スロット数を指定して「自動生成」し、必要に応じてラベルやレンジを調整して
保存してください。カスタムプロファイルは `~/.config/ros2can/profiles/` に
JSON として保存され、次回起動時にも読み込まれます。

## 安全に関する注意

- 起動直後は全デバイスの「TX有効化」が OFF になっており、実際の指令は
  送信されません。意図した値を設定してから ON にしてください。
- TX有効化中は 20Hz で現在の指令値を周期送信し続けます (ウィンドウを閉じる、
  または「全ゼロ送信」/E-STOP を押すと即座にゼロ指令が送信されます)。
- ツールバーの「全デバイス E-STOP」は、接続中の全デバイスへゼロ指令を送信し
  TXを無効化します。緊急時はこれを押してください。

## 対応スロットマッピング (既定プロファイル)

```
1ノードあたり5スロット (CAN_SLOTS_PER_NODE=5):
  指令 (ROS -> ホスト -> CAN -> ノード): MD1, MD2, MD3, MD4, SERVO1
  帰還 (ノード -> CAN -> ホスト -> ROS): SW1, SW2, SW3, ENC1, ENC2

グローバルスロット index = node_index(0-origin) * 5 + local_index
ノードの CAN_ID は 101,102,103,104 (下2桁 = ノード番号)
```
