# damiao_ctrl

> **使い方・仕組み・モータの増やし方・自分のプログラムの書き方は [docs/開発ガイド.md](docs/開発ガイド.md) にまとめています。まずはこちらを読んでください。**

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

- PC-ESP32間は RRST-NHK-Project/ros2can を使う(`~/ros2_ws/src/ros2can`)。
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
| 1 | ROS→ESP32 | target (VEL:rad/s, POS_VEL:rad) | VEL:0.01 / POS_VEL:0.001 |
| 2 | ROS→ESP32 | vel_limit (POS_VELのみ有効) | 0.01 rad/s |
| 0 | ESP32→ROS | motor_state (0=Disable, 1=Enable, それ以外=故障。13=通信喪失) | - |
| 1 | ESP32→ROS | fb_counter (帰還を受け取るたび+1。止まったら帰還途絶) | - |
| 3 | ESP32→ROS | position (出力軸、±12.5rad) | 0.001 rad |
| 4 | ESP32→ROS | velocity (出力軸、モータ側分解能約0.098rad/sで静止中も±0.049) | 0.01 rad/s |
| 5 | ESP32→ROS | torque | 0.001 N・m |

速度だけ0.01単位なのは、最高速度(無負荷560rpm=58.6rad/s)がint16の0.001単位の上限
(32.767)を超えるため(2026-10-15変更)。ESP32ファームと必ず一致させること。

**速度の上限**: DM-S2325-1EC(減速比1:25)の説明書上の無負荷最高速度は560rpm(58.6rad/s、24V時)
だが、実機は**48rad/s(458rpm)で頭打ち**になる(2026-10-15実測。モータの最高速度レジスタRID6が
約1200(モータ軸rad/s)÷減速比25=48rad/sのため)。
その95%の**45.6rad/s(435rpm)**を上限とし、damiao_driver_nodeの`max_vel_radps`で
どの指令もこれを超えないよう制限している。

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
cd ~/ros2_ws
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

**どのターミナルでも最初に `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` を実行すること。**
付けないと同じWi-Fiの他のPCのノード(他チームのjoy_node・ros2can等)とつながり、
PS4の入力が混ざったり、他PCの指令がモータに届いたりする(2026-10-15に実際に発生)。

```bash
# ターミナル1: PC<->ESP32の通信(ros2can)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && source ~/ros2_ws/install/setup.bash && ros2 run ros2can ros2can --nogui
```

```bash
# ターミナル2: 駆動ノード + PS4テレオペ(R1を押している間だけ動く)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && source ~/ros2_ws/install/setup.bash && ros2 launch damiao_ctrl damiao_ctrl.launch.py use_joy:=true
```

```bash
# ターミナル3: 回転数の表示(rpm。指令は送らない。2台なら1台1行)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && source ~/ros2_ws/install/setup.bash && ros2 run damiao_ctrl motor_monitor --ros-args --params-file ~/ros2_ws/install/damiao_ctrl/share/damiao_ctrl/config/damiao_ctrl.yaml
```

終了はターミナル2 → ターミナル1の順にCtrl+C。
PCがスリープした・Wi-Fiがつなぎ直された後は、全部を一度止めて起動し直すこと
(既存のノード同士はつながったままでも、新しく起動したノードから見えなくなる)。

PS4を使わずにコマンドで速度指令を送る例(Ctrl+Cで止めると0.5秒後に自動DISABLE):

```bash
ros2 topic pub -r 20 damiao/motor1/cmd std_msgs/msg/Float32MultiArray "{data: [0.0, 0.5, 0.0]}"
```

## 2台構成(2026-11-02〜)

| | モータ1(元からのESC) | モータ2(新しいESC) |
|---|---|---|
| ESC_ID(指令を受けるCAN ID) | 1 | 2 |
| MST_ID(帰還を返すCAN ID) | 0x000 | 0x012 |
| PS4 | 左スティック上下で**速度制御** | 右スティックで**角度制御**(倒した方向を向く) |
| トピック | damiao/motor1/cmd, /state | damiao/motor2/cmd, /state |

- DM3520は工場出荷時 ESC_ID=1, MST_ID=0x000 なので、2台目以降は達妙USB-CAN(DM-USBCU1)を
  そのESCだけにつないで `scripts/dm_usbcan_param.py` で書き換える(ros2canは止めておく):
  ```bash
  python3 ~/ros2_ws/src/damiao_ctrl/scripts/dm_usbcan_param.py scan
  ```
  ```bash
  python3 ~/ros2_ws/src/damiao_ctrl/scripts/dm_usbcan_param.py set-id --id 1 --new-id 2 --new-mst 0x12
  ```
  MST_IDを台ごとに変えるのは、2台が同じCAN IDで同時に帰還を返すと衝突するため。
- 配線: 2台のCANH同士・CANL同士をつないでSOKI基板のCAN1へ(並列)。終端抵抗はバスの両端に
  120Ω(電源OFFでCANH-CANL間が約60Ωになる)。
- ESP32ファーム(ros2canのconfig.hpp): `DAMIAO_MOTOR_COUNT 2`、`DAMIAO_MOTOR_ID_n`/`DAMIAO_MASTER_ID_n`。
  有効化シーケンスは台ごとに非同期で進む(1台の有効化中も他の台への指令は途切れない)。
- PC側(config/damiao_ctrl.yaml): `damiao_driver_node.motor_count: 2`、
  `ps4_teleop_node.motor_numbers: [1, 2]` / `control_modes: ["velocity", "angle"]` /
  `stick_y_axes: [1, 4]` / `stick_x_axes: [0, 3]`。
  駆動ノードは1つで2台を扱う(2つ起動するとお互いのスロットを0で上書きするため)。

## 角度制御(PS4の右スティック、2026-11-02〜)

- R1を押しながら右スティックを倒すと、その方向をモータ2が向く(スティック上=基準の向き、
  右=基準から+90°)。スティックを離すとその向きを保つ。向きを変える速さは
  `angle_vel_limit_radps`(既定10rad/s)まで。目標は近い方の回り方で選ぶ。
- **基準の向き**: モータの電源を入れた時の向き(位置0)。**△ボタン**で「今の向き」を基準
  (=スティック上)にし直せる。
- 右に倒して左を向く(鏡写し)なら `invert_axes` の2つ目を `true` にする。
- **モータ内部の位置(レジスタ80)に注意(2026-11-02実機で確認)。** 位置の帰還は内部の位置を
  ±12.5radに折り返した値。速度制御で何百rad回すと内部の位置が大きくなり(例: 1144rad)、
  ゆっくり回した時に位置の読みが変わらなくなる(実際には回っている)。この状態で角度制御を
  使うと何百radも回って戻ろうとするため、**内部の位置が分からない/±12radを超えるモータは
  ESP32とPS4ノードが角度制御を有効にしない**。モータの電源を入れ直すと内部の位置は0に戻る。
  内部の位置は `motor_monitor` の「内部」とstateの6番目で見られる。
- さらに見張りとして、指令した量より0.5rad以上大きく回ったらモータ2を止める
  (R1を離して押し直すまで再開しない)。
- R1を押した瞬間の目標は現在位置なので、押しただけでは回らない。

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
  - **R1を押している間だけ**左スティック上下で速度指令(倒し切りで `max_vel_radps` = 45.6rad/s)。
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
- [x] PS4コントローラ実機でのボタン番号確認・テレオペ確認
- [x] 2台対応(ファーム・駆動ノード・PS4・表示)を偽の帰還で確認(2026-11-02)
- [x] 2台目のESCのID書き換え(ESC_ID 2 / MST_ID 0x012)・2台での低速実機確認(scripts/two_motor_test.py、2026-11-02)
- [x] PS4での2台同時操作(速度×2)の実機確認(ユーザー確認、2026-11-02)
- [x] 角度制御(右スティック)の論理テスト 22/22(偽の/joyと位置、2026-11-02)
- [x] 角度制御の実機確認(PS4右スティックで目標の向きへ到達、2026-11-02)
- [ ] 右に倒して右を向くか(鏡写しの有無)の確認
