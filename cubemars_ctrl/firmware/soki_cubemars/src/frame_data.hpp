/*====================================================================
<frame_data.hpp>
・シリアル通信のフレームデータ定義ヘッダーファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include "config.hpp"
#include <stdint.h>

#define Tx16NUM 24 // 送信するint16データの数
#define Rx16NUM 24 // 受信するint16データの数
// CANモードで扱うIO値の数。CAN送受信の1ノード分スロット数(config.hppの
// CAN_SLOTS_PER_NODE、基板ごとに異なる)と必ず一致させる。
#define CAN_IO_SLOT_COUNT CAN_SLOTS_PER_NODE

extern volatile int16_t Tx_16Data[Tx16NUM];
/*
MODE_CAN_HOST/MODE_CAN時は下記のMD/SERVO/TR直接マッピングは使用しない。
24スロットを CAN バス上の各ノードへ 5スロットずつ分配する
(詳細は README.md の CAN Slot Mapping、CanIoRxData/CanIoTxData を参照)。

MODE_IO (非CAN, スタンドアロン) 時のみ以下のマッピングを使用:
0: デバッグ用
1: ENC1 raw pulse (ENC1_MD == 1 の場合は常に0)
2: ENC2 raw pulse (ENC2_MD == 1 の場合は常に0)
3~8: 予備
9~16: SW1~8
17~23: 予備
*/

extern volatile int16_t Rx_16Data[Rx16NUM];
/*
MODE_CAN_HOST/MODE_CAN時は下記のMD/SERVO/TR直接マッピングは使用しない。
24スロットを CAN バス上の各ノードへ 5スロットずつ分配する
(詳細は README.md の CAN Slot Mapping、CanIoRxData/CanIoTxData を参照)。

MODE_IO (非CAN, スタンドアロン) 時のみ以下のマッピングを使用:
0: デバッグ用
1: MD1 PWM指令 (ENC1_MD == 1 の場合のみ有効。符号=方向、絶対値=デューティ)
2: MD2 PWM指令 (ENC2_MD == 1 の場合のみ有効。符号=方向、絶対値=デューティ)
3~8: 予備(未使用)
9~16: SERVO1~8
17~23: TR1~7 (実機は非搭載のため未使用)
*/

extern volatile int16_t CanIoRxData[CAN_IO_SLOT_COUNT];
/*
BOARD_VARIANT(config.hpp)ごとにレイアウトが異なる。詳細はpin_ctrl_task.cppの
IO_MD_Output/IO_Servo_Outout/IO_TR_Outputの実装コメントを参照。

BOARD_SOKI (5スロット):
  0: SERVO1
  1: SERVO2
  2: SERVO3
  3: MD1 PWM指令 (ENC1_MD == 1 の場合のみ有効。符号=方向、絶対値=デューティ)
  4: MD2 PWM指令 (ENC2_MD == 1 の場合のみ有効。符号=方向、絶対値=デューティ)
  (SERVOn は SWn とピン共有。config.hpp の MULTIn で入出力を切替)
  (MDn は ENCn とピン共有。config.hpp の ENCn_MD で入出力を切替)

BOARD_MES (5スロット、0-1のみ使用):
  0: MD1 PWM指令
  1: MD2 PWM指令
  2-4: 未使用

BOARD_SS (9スロット):
  0-4: SERVO1-5
  5-8: TR1-4 (ソレノイドバルブ、0以外でON)
*/

extern volatile int16_t CanIoTxData[CAN_IO_SLOT_COUNT];
/*
BOARD_VARIANT(config.hpp)ごとにレイアウトが異なる。詳細はpin_ctrl_task.cppの
IO_ENC_Input/IO_SW_Inputの実装コメントを参照。

BOARD_SOKI (5スロット):
  0-2: SW1-3
  3: ENC1 raw pulse (ENC1_MD == 1 の場合は常に0)
  4: ENC2 raw pulse (ENC2_MD == 1 の場合は常に0)

BOARD_MES (5スロット):
  0: SW1
  1: ENC1 raw pulse
  2: ENC2 raw pulse (ENC2_SW == 1 の場合はSW2、!digitalReadの論理)
  3: SW3 (ENC2_SW == 1 の場合のみ有効、それ以外は常に0)
  4: 未使用

BOARD_SS (9スロット): 帰還なし、全スロット常に0
*/
