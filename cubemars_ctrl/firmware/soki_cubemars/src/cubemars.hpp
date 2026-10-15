/*====================================================================
<cubemars.hpp>
・MODE_CUBEMARS(CubeMars AKシリーズ Servo(CAN)モード ドライバ)のヘッダーファイル

MODE_CUBEMARSは、xiao-esp32-s3_can2ioの他モード(MODE_IO/MODE_CAN/MODE_CAN_HOST)
が使うノード/スロット分配プロトコルとは別系統で、CubeMars AKシリーズ
のServo(CAN)モードプロトコル(1Mbps固定, AK Series Module Product Manual V3.2.0
4.1節)、およびForce Control(MIT)モード(同マニュアル4.2節、位置/速度/Kp/Kd/
トルクFFを1フレームで指令するインピーダンス制御)を直接喋る独立デバイスとして
動作する。モータごとにcontrol_modeスロットでどちらを使うか毎周期選択できる。
1マイコン(1バス)には最大 CUBEMARS_MOTOR_COUNT 台まで接続する構成を想定する。
ノード/スロット分配側もcan_task.cppで1Mbpsに統一してあるため同一物理バスへの
混在が可能(CAN IDは重ならない設計。指令送信は200Hzに抑えてある。詳細はREADME.md
参照)。

指令(PC -> 本機, Rx_16Data)/帰還(本機 -> PC, Tx_16Data)のスロット割り当ては
cubemars.cppの先頭コメント、およびREADME.mdを参照。

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"
#include <Arduino.h>

// CANドライバ初期化(1Mbps)
void cubemarsInit();

// CAN送受信タスク本体(アクチュエータ内蔵のクローズドループに指令を送るのみ、
// ホスト側での速度/位置PIDは行わない)
void cubemarsTask(void *pvParameters);
