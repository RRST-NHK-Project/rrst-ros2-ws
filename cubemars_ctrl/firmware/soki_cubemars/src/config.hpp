/*====================================================================
<config.h>
書き込み前にここでIDと動作モードを設定してください．MDやサーボの設定もここで行います．
MDは基本的に変更不要ですが，サーボは型番、機構に応じて適切に設定する必要があります．
Copyright (c) 2025 RRST-NHK-Project
SPDX-License-Identifier: MIT

[cubemars_ctrl版の変更点] ros2can/generated_firmware/soki_ak_11(チームで実機
動作済みの設定)をコピーし、以下だけを変更している。
  1. 動作モードをplatformio.iniの環境(env)で切り替えるようにした
     (pio run -e cubemars / pio run -e can_monitor)。手でコメントアウトを
     付け替える作業を無くし、書き込みミスを防ぐため。
  2. CUBEMARS_MOTOR_COUNTを1台に、CUBEMARS_MOTOR_ID_1を実機のCAN IDに合わせた。
  3. シリアル途絶時のフェイルセーフ(CUBEMARS_CMD_TIMEOUT_MS)を追加した。
     元のファームは最後に受信した指令を保持し続けるため、PC側ノードが落ちると
     速度指令のまま回り続ける。
====================================================================*/

#pragma once
#include <Arduino.h>

// ================= 基本設定 =================

// IDの設定，シリアルフレームのDEVICE_IDとして使用します。
// ROS側(config/cubemars_ctrl.yamlのdevice_id)と必ず一致させること。
#define DEVICE_ID 11

// CAN_IDは3桁形式で指定します。
// 1桁目はバス番号、末尾2桁はノード番号を表します。
// 例: 101, 102, 103, 104
// (MODE_CUBEMARSでは使われない。CubeMarsモータ側のCAN IDは下の
// CUBEMARS_MOTOR_ID_1で設定する)
#define CAN_ID 101

// モードの設定。platformio.iniの環境で -DMODE_xxx を渡して選択する。
//   [env:cubemars]    -> MODE_CUBEMARS    (通常運用。ROSからモータを動かす)
//   [env:can_monitor] -> MODE_CAN_MONITOR (受信専用。モータのCAN ID調査用)
// どちらも渡されなかった場合(ArduinoIDE等でビルドした場合)はMODE_CUBEMARSにする。
#if !defined(MODE_CAN) && !defined(MODE_CAN_HOST) && !defined(MODE_IO) && !defined(MODE_DEBUG) && \
    !defined(MODE_CAN_MONITOR) && !defined(MODE_ROBOMAS) && !defined(MODE_CUBEMARS)
#define MODE_CUBEMARS
#endif

// ================= 基板バリアントの設定 =================
// このマイコン(XIAO ESP32-S3)を搭載する基板ごとにピン配置が異なるため、
// defs.hppの「ピンの定義」で参照する基板を1つ選択すること。
#define BOARD_SOKI 1 // soki本体基板(root_theta/tip_theta用CubeMars、z/r用ロボマス。既存のピン配置)
#define BOARD_MES 2  // 新規基板(2026-09-03追加、ENC1/ENC2/MD1/MD2 + SW1、CAN=D6/D7)
#define BOARD_SS 3   // 新規基板(2026-09-03追加、SERVO1-5 + TR1-4(ソレノイドバルブ)、CAN=D6/D7)

// MES/SSはMODE_CAN/MODE_CAN_HOST(ノード/スロット方式のIO)専用に入出力ロジックを
// 実装済み(pin_ctrl_init.cpp/pin_ctrl_task.cpp)。MODE_IO(非CAN単体)はBOARD_SOKI
// のみ対応(Rx_16Data/Tx_16Dataの直接インデックスがSOKIのSERVO/SW配置前提のため)。
// BOARD_SOKIは既存動作のまま変更なし。
#define BOARD_VARIANT BOARD_SOKI

// ================= MD関連 =================
// ENC1_MD/ENC2_MD(下記)でENCポートをMDへ切替えた場合のみ有効。
// MDはENCのA/Bピンを流用するため、実機はDCモータ非搭載の想定のまま
// (この設定は使わずENC1_MD=0/ENC2_MD=0のまま)でも変更不要。
#define MD_PWM_FREQ 20000   // MDのPWM周波数（Hz）
#define MD_PWM_RESOLUTION 8 // MDのPWM分解能（bit）

// ================= サーボ関連 =================

// サーボ関連の設定、使用するサーボに応じて変更
#define SERVO_PWM_FREQ 50       // サーボPWM周波数（Hz）
#define SERVO_PWM_RESOLUTION 14 // サーボPWM分解能（bit）

// サーボの最小・最大パルス幅、角度範囲、初期角度の設定
#define SERVO1_MIN_US 499
#define SERVO1_MAX_US 2500
#define SERVO1_MIN_DEG 0
#define SERVO1_MAX_DEG 270
#define SERVO1_INIT_DEG 0

#define SERVO2_MIN_US 500
#define SERVO2_MAX_US 2500
#define SERVO2_MIN_DEG 0
#define SERVO2_MAX_DEG 270
#define SERVO2_INIT_DEG 0

#define SERVO3_MIN_US 500
#define SERVO3_MAX_US 2500
#define SERVO3_MIN_DEG 0
#define SERVO3_MAX_DEG 270
#define SERVO3_INIT_DEG 0

#define SERVO4_MIN_US 500
#define SERVO4_MAX_US 2500
#define SERVO4_MIN_DEG 0
#define SERVO4_MAX_DEG 270
#define SERVO4_INIT_DEG 0

// SERVO5はBOARD_SS専用(defs.hppでBOARD_SS選択時のみSERVO5ピンが定義される)
#define SERVO5_MIN_US 500
#define SERVO5_MAX_US 2500
#define SERVO5_MIN_DEG 0
#define SERVO5_MAX_DEG 270
#define SERVO5_INIT_DEG 0

// ================= 高度な設定（通常は変更不要） =================

// 以下の設定は必要に応じて変更
#define ENABLE_LED 1 // 状態表示LEDを有効にする場合1に設定

// 汎用（MULTI）ポートの設定（スイッチ:0, サーボ:1）
#define MULTI1 0
#define MULTI2 0
#define MULTI3 0

// ENC/MDポートの設定（エンコーダ:0, MD:1）
// ENC1(ENC1_A/ENC1_B)をMD1のPWM/DIRへ、ENC2(ENC2_A/ENC2_B)をMD2のPWM/DIRへ転用する。
// MDに切替えたチャンネルはエンコーダとして読めなくなる(帰還スロットは0固定になる)。
// BOARD_MESはMD1/MD2がENCと別ピン(常時専用)のため、この2つはBOARD_SOKI専用。
#define ENC1_MD 0
#define ENC2_MD 0

// BOARD_MES専用: ENC2(ENC2_A/ENC2_B)とSW2/SW3はピン共有のため、どちらで使うか選択する
// (0=ENC2をエンコーダとして使用、1=SW2/SW3をスイッチとして使用)。
#define ENC2_SW 0

// CANのノード割り当て設定
// 1つのCANバス上でCAN_NODE_COUNTノードまで対応し、1ノードあたりCAN_SLOTS_PER_NODE
// スロットをCANで送受信する。ビットレートは1Mbps固定(can_task.cpp)。
// MODE_ROBOMAS/MODE_CUBEMARSと同じ物理バスに同居させる場合でも指令/帰還のCAN ID帯が
// 重ならないよう設計されているが、CAN_NODE_COUNTを大きくしすぎるとロボマス側の
// ID帯(0x1FE, 0x200-0x208)に近づくため、増やす場合は帰還ID(0x180 + node*16 + chunk)
// の上限がそこへ届かないことを確認すること。
// 重要: ここは「対応可能な最大数」ではなく「実際にバスへ接続されているノード数
// (ホスト自身を含む)」に必ず合わせること。ホストは毎周期、自分以外の
// [0, CAN_NODE_COUNT) 全ノードへ指令フレームを送信するため、存在しないノード宛の
// 分だけACKエラーが発生し続け、ホスト自身のCANコントローラがBus-Offに陥る
// (実測: 4ノード設定・実接続2台の場合、起動後約40msでBus-Off)。
// 現在の実接続(BOARD_SOKI): ホスト(node0)+STM32 b-g431-esc1_can2io(node1)の2台のみ。
//
// BOARD_SS(SERVO1-5+TR1-4=9指令チャンネル)はCAN_SLOTS_PER_NODE=5に収まらないため
// 9へ拡張し、Tx_16Data/Rx_16Dataの24スロット枠に収まるようCAN_NODE_COUNTを2
// (host+1)までに抑えている(下のstatic_assert相当のチェックも参照)。この値を
// 手で変える場合は必ず CAN_NODE_COUNT * CAN_SLOTS_PER_NODE <= 24 を保つこと。
#if BOARD_VARIANT == BOARD_SS
#define CAN_NODE_COUNT 2
#define CAN_SLOTS_PER_NODE 9
#else
#define CAN_NODE_COUNT 4
#define CAN_SLOTS_PER_NODE 5
#endif

#if (CAN_NODE_COUNT * CAN_SLOTS_PER_NODE) > 24
#error "CAN_NODE_COUNT * CAN_SLOTS_PER_NODE exceeds the 24-slot Tx_16Data/Rx_16Data buffer (frame_data.hpp)"
#endif

// CAN_ID の下位2桁をノード番号として使用する。
// 例: CAN_ID=101 -> node 1, CAN_ID=102 -> node 2, CAN_ID=103 -> node 3, CAN_ID=104 -> node 4
#define CAN_NODE_INDEX ((CAN_ID % 100U) - 1U)

// ホスト自身のCAN状態(TxErrCnt/RxErrCnt/bus_error等)を500msごとにSerial.println
// で出力する診断ログ(can_task.cpp printHostCanDiagnostics, MODE_CAN_HOSTのみ有効)。
// このテキスト出力はros2canが読んでいるのと同じUSBシリアル(Serial)に直接書き込まれる
// ため、有効にするとserial_bridgeバイナリフレームと混ざり、ros2can側で
// 「不正な同期バイトを破棄」が継続的に発生する(実機で確認済み)。CAN bus_off等の
// 切り分けで必要な時だけ1にし、普段(ros2can使用時)は0のままにすること。
#define CAN_HOST_DIAG_ENABLE 0

// ================= CANモニタ関連 (MODE_CAN_MONITORのみ有効) =================
// バス上の任意のトラフィックを観測するための設定。ホスト用のCAN_NODE_COUNTとは
// 独立しており、ここで設定した範囲外のノードから来たフレームも
// CAN_MONITOR_RAW_ENABLEが1であれば生データとして出力される(汎用スニファ)。

#define CAN_MONITOR_RAW_ENABLE 1     // 1=受信した全フレームをID/DLC/生バイトでそのまま出力
// CubeMarsのCAN ID調査では要約(node/slot方式のデコード)は意味を持たず出力が
// 読みにくくなるだけなので、[env:can_monitor]では0にしている(platformio.ini)。
#ifndef CAN_MONITOR_SUMMARY_ENABLE
#define CAN_MONITOR_SUMMARY_ENABLE 1 // 1=node/slot単位にデコードした要約も併せて出力
#endif
#define CAN_MONITOR_MAX_NODES 16     // 要約デコード対象のノード数上限(CAN_NODE_COUNTと無関係)

// ================= ロボマス関連 (MODE_ROBOMASのみ有効) =================
// MODE_ROBOMASはxiao-esp32-s3_can2ioのノード/スロット分配方式とは別系統で、
// DJI RoboMasterシリーズのCANプロトコル(1Mbps固定, ID固定)を直接喋る独立デバイス
// として動作する。1マイコン(1バス)には同一機種のみ最大4台まで接続可能。
// ノード/スロット分配方式(MODE_CAN等)側もcan_task.cppで1Mbpsに統一してあるため、
// 同一物理バスへの混在自体は可能(CAN IDが重ならないよう設計されている)。ただし
// robomasTaskの指令送信は200Hzに落としてある(1kHzのままだとCubeMars/センサノードと
// 合計したバス帯域を超えるため)。詳細・帯域見積りはREADME.md参照。

#define ROBOMAS_MOTOR_M3508 1  // C620 + M3508 (メカナム/足回り等、ギア比19.2)
#define ROBOMAS_MOTOR_M2006 2  // C610 + M2006 (小型アクチュエータ等、ギア比36.0)
#define ROBOMAS_MOTOR_GM6020 3 // GM6020 (ダイレクトドライブ、ギア無し)

// 使用するモータ機種を1つ選択すること。
#define ROBOMAS_MOTOR_TYPE ROBOMAS_MOTOR_M2006

// 速度PIDゲイン。ros2can(PC)側からは変更できない固定値。チューニングはここで行う。
#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M3508
#define ROBOMAS_KP_VEL 0.8f
#define ROBOMAS_KI_VEL 0.0f
#define ROBOMAS_KD_VEL 0.05f
#define ROBOMAS_OUTPUT_GAIN 10.0f   // PID出力 -> 電流指令[A]への換算係数
#define ROBOMAS_MAX_CURRENT_A 20.0f // 電流指令の飽和値[A] (C620仕様上限)
#elif ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M2006
// Kp=0.8のため誤差1.25rpm(=max_out/Kp)を超えると出力は電流上限1.0Aに張り付く。
// 目標200rpmに対し実測が183rpm付近で頭打ちなのはこの電流上限による飽和であり、
// ゲインでは解消しない(電流上限を上げない前提で追い込む場合はここが天井)。
// Kdは旧増分PID(dt未除算)向けの値0.02のままだった。PID.hppのDifferential_は
// dtで除算する実装のため、dt≈1msでは同じKd値でも実効ゲインが約1000倍になり、
// 速度センサの数rpmのノイズがそのまま数十A相当に増幅されて出力を瞬間的に
// デサチュレートさせ、リップルの原因になっていた。dt換算(Kd_new≈Kd_old*dt)で
// 0とみなせる値まで下げる。
#define ROBOMAS_KP_VEL 0.8f
#define ROBOMAS_KI_VEL 0.0f
#define ROBOMAS_KD_VEL 0.0f
#define ROBOMAS_OUTPUT_GAIN 1.0f
#define ROBOMAS_MAX_CURRENT_A 1.0f
#elif ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_GM6020
// PD制御 (Ki=0)。実測: P単独でKp=0.0080から振動 -> 限界感度 Ku=0.0080。
//
// Kd: PID.hppのDifferential_は -(current - last_current)/dt とdtで除算するので、
//     Kdは連続系の単位[A*s/rpm]そのまま。微分先行時間Tdとは Kd = Kp * Td。
//     ここを増分形PID(dt未除算)の感覚で書くとdt≈1msのぶん1000倍になり、速度の
//     量子化1LSB(1rpm)だけで Kd*(1/0.001)=Kd*1000 [A] のキックが出て出力が全振幅
//     で飽和・反転し、停止中でも激しく振動する(M2006の項と同じ罠)。
//     Tdの上限を決めるのもこの量子化。Td=4msなら1LSBあたり25.6mA(上限3Aの0.9%)で
//     許容範囲、Td=10ms以上ではノイズの増幅が信号を上回る。
//     Td=4ms採用 -> Kd = Kp * 0.004。Kpを動かすときはこの比を保つこと。
// Kp: ZNのPD則 0.8*Ku = 0.0064 から開始。Dの位相進みでKu自体が上がるはずなので、
//     無音なら 0.0080 -> 0.0100 と上げられる(その都度Kdも上式で追随させる)。
//
// Kiは意図的に0。ただし摩擦・コギング抗力を打ち消す電流(実測55mA)をP項だけで
// 作ることになるため、55mA/Kp = 8.6rpm の定常偏差が恒久的に残る。PもDも偏差ゼロ
// では出力ゼロなのでこれは消えない。許容できない場合のみKiを戻すこと。
#define ROBOMAS_KP_VEL 0.02f
#define ROBOMAS_KI_VEL 0.0f
#define ROBOMAS_KD_VEL 0.0000256f
#define ROBOMAS_OUTPUT_GAIN 1.0f
// sendCurGm6020()が±3.0A(±16384)で切っているので外側もそこに合わせる。
// 10.0Aのままだと飽和点が実効上限とずれ、アンチワインドアップが機能しない。
#define ROBOMAS_MAX_CURRENT_A 3.0f
#else
#error "ROBOMAS_MOTOR_TYPE: unknown motor type"
#endif

// ---- MIT(位置PD制御)モード関連 ----
// CubeMarsのMITモードと異なり、ロボマス側ESC(C610/C620)やGM6020はCANで生の電流指令
// しか受け付けずアクチュエータ内蔵の位置/トルク制御が無いため、位置PD制御ループ
// 自体をこのマイコン(ESP32)側で計算し、結果をそのまま既存のsendCurrentCommand()
// (電流指令)へ渡す(モータへの新しいCANフレーム形式は不要、robomas.cpp参照)。
// 位置フィードバックはロボマス内蔵ロータエンコーダ(angle[]/vel[])を使うため、
// ベルト・プーリー等の外部伝達機構のバックラッシュ/弾性誤差はここでは補正できない
// (外付けエンコーダを真値とする用途に使う場合は許容できるか要検討、
// note/hardware_mapping.txtのz_joint/r_joint節参照)。
// Kp/Kd/current_ffは速度モードのROBOMAS_KP_VEL等と異なりコンパイル時固定値ではなく、
// CAN経由で毎周期ROSから送られる可変値(robomas.cppのcontrol_mode=ROBOMAS_MODE_MIT
// 時のみ参照)。値は暫定のスケール定数であり、実機チューニングで変更してよい。
// 目標位置(target流用)はint16スロットなので、0.1deg/LSBのままだと±3276.7deg
// (出力軸換算で約±9.1回転)までしか指令できない。soki z/r軸のように出力軸換算で
// 約9回転に迫る可動域がある用途向けに1deg/LSBへ粗くし、±32767deg(約±91回転)
// まで拡張してある(プーリー半径11.46mm相当の機構なら1deg≈0.2mmでまだ十分な分解能)。
// 更に多回転が必要になった場合は、2スロットを結合してint32化する等の対応が必要。
#define ROBOMAS_MIT_POSITION_LSB_DEG 1.0f    // 目標位置(target流用)。1deg/LSB、出力軸角度
#define ROBOMAS_MIT_VELOCITY_FF_LSB_RPM 1.0f // 目標速度FF。1rpm/LSB、出力軸rpm
#define ROBOMAS_MIT_KP_LSB 0.001f            // 比例ゲイン。0.001(A/deg)/LSB
#define ROBOMAS_MIT_KD_LSB 0.0001f           // 微分ゲイン。0.0001(A/rpm)/LSB
#define ROBOMAS_MIT_CURRENT_FF_LSB_A 0.001f  // 電流フィードフォワード。0.001A/LSB

// ================= CubeMars AK関連 (MODE_CUBEMARSのみ有効) =================
// MODE_CUBEMARSはMODE_ROBOMASと同様、xiao-esp32-s3_can2ioのノード/スロット分配方式
// とは別系統で、CubeMars AKシリーズのServo(CAN)モードプロトコル(1Mbps固定、
// AK Series Module Product Manual V3.2.0 4.1節)を直接喋る独立デバイスとして動作
// する。速度/位置ともアクチュエータ内蔵のクローズドループがそのまま追従するため、
// ロボマスのGM6020のようなホスト側速度PIDは不要。
// 1マイコン(1バス)には最大 CUBEMARS_MOTOR_COUNT 台まで接続可能。
// ノード/スロット分配方式・MODE_ROBOMASとは1Mbpsに統一済みのため同一物理バスへの
// 混在も可能(CAN IDは重ならない設計)。cubemarsTaskの指令送信は200Hzに落として
// あり、1kHzのままだと合計バス帯域を超える点に注意(README.md参照)。
// AKアクチュエータ自身の帰還フレーム(function ID 0x29)はホストの指令頻度とは
// 無関係にアクチュエータ側のタイマで自律送信されるため、こちらの帯域は
// robomasTask/cubemarsTaskの送信頻度を下げても減らない。

// cubemars_ctrlではAK40-10を1台だけ動かす。4のままだと存在しないID宛ての指令も
// 毎周期送ることになり、別のモータを繋いだ時に意図せず動かす可能性があるため1にする。
#define CUBEMARS_MOTOR_COUNT 1 // 1バスあたりの接続台数(最大4)

// 各モータのCAN ID (R-Link CubeMarsTool の Application Configuration -> CAN ID
// 設定と一致させること。1台ずつ重複しない値を書き込んでおく)
// [cubemars_ctrl] CUBEMARS_MOTOR_ID_1は「pio run -e can_monitor」で調べた実機の
// CAN IDに書き換えること。帰還フレームの拡張IDが 0x29XX のとき、XX(16進)が
// CAN IDになる(例: id=0x2966 -> 0x66 = 102)。
// 初期値102はsokiのroot_thetaに転用されたAK40-10のR-Link設定値(note/hardware_mapping.txt)。
#define CUBEMARS_MOTOR_ID_1 102
#define CUBEMARS_MOTOR_ID_2 102
#define CUBEMARS_MOTOR_ID_3 103
#define CUBEMARS_MOTOR_ID_4 104

// ---- MIT(Force Control)モード関連 (AK Series Module Product Manual V3.2.0 4.2節) ----
// MITモードはServo(CAN)モードと同じ拡張ID方式(control_mode_id<<8 | driver_id)の
// 別モード(control_mode_id=8)で、位置・速度・トルクFF・Kp・Kdを同時に指令し
// モータ側で torque = Kp*(p_des-p) + Kd*(v_des-v) + t_ff を計算するインピーダンス制御。
// 有効化はcubemars.cpp側、ホストからの選択はcontrol_modeスロット(=2)で行う。
//
// 下記のP/V/T(位置・速度・トルク)レンジはホスト側のエンコード基準であり、
// モータ側のデコード基準(ファーム内蔵、R-Linkからは変更不可)と一致していないと
// 指令値の意味(rad, rad/s, N・m)がズレて意図しない速度・トルクが出力される。
// 値は AK Series Module Driver Manual V1.0.18 (AK 2.0 Robotic Actuator向け) 5.3節
// Parameter rangeテーブルのAK40-10行から採用。ただし同マニュアルはAK 2.0世代向けで
// あり、本機のAK40-10 V3.0(AK Series Module Product Manual V3.2.0对象、AK 3.0
// Robotic Actuator向け)にはAK40-10自体の記載が無い。ハードウェア世代差の影響は
// 小さいと考えられるが、使用前にR-Link(CubeMarsTool)のMIT Controlタブで実機の値を
// 確認することが望ましい。Kp/Kdレンジ(0-500 / 0-5)は全モータ共通の値として両
// マニュアルに明記されているためそのまま使用してよい。
#define CUBEMARS_MIT_P_MIN_RAD -12.5f
#define CUBEMARS_MIT_P_MAX_RAD 12.5f
#define CUBEMARS_MIT_V_MIN_RADPS -45.5f
#define CUBEMARS_MIT_V_MAX_RADPS 45.5f
#define CUBEMARS_MIT_T_MIN_NM -5.0f
#define CUBEMARS_MIT_T_MAX_NM 5.0f
#define CUBEMARS_MIT_KP_MIN 0.0f
#define CUBEMARS_MIT_KP_MAX 500.0f
#define CUBEMARS_MIT_KD_MIN 0.0f
#define CUBEMARS_MIT_KD_MAX 5.0f

// ---- シリアル途絶時のフェイルセーフ [cubemars_ctrlで追加] ----
// PCから有効なフレーム(DEVICE_ID一致・チェックサム正常)がこの時間届かなければ、
// 受信済みの指令を捨てて全モータへゼロ速度指令(その場停止)を送る。
// 元のファーム(soki_ak_11)は最後の指令を保持し続けるため、ROSノードの異常終了や
// USBケーブル抜けで速度指令のまま回り続ける危険があった。
// PC側の送信周期(ROSブリッジ: 20ms、ros2can GUIのダイレクト送信: 50ms)に対して
// 十分長く、人が「止まらない」と感じるより十分短い値にしている。
#define CUBEMARS_CMD_TIMEOUT_MS 300
