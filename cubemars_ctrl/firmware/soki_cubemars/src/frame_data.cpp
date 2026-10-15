/*====================================================================
<frame_data.cpp>
・シリアル通信のフレームデータ定義ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "frame_data.hpp"

volatile int16_t Tx_16Data[Tx16NUM] = {0}; // 送信用データ配列
/*
MODE_CAN_HOST/MODE_CAN時は下記のMD/SERVO/TR直接マッピングは使用しない
(24スロットをCANバス上の各ノードへ分配する。README.md参照)。
MODE_IO時のみ:
0: デバッグ用
1: ENC1 raw pulse (ENC1_MD == 1 の場合は常に0)
2: ENC2 raw pulse (ENC2_MD == 1 の場合は常に0)
3~8: 予備
9~16: SW1~8
*/

volatile int16_t Rx_16Data[Rx16NUM] = {0}; // 受信用データ配列
/*
MODE_CAN_HOST/MODE_CAN時は下記のMD/SERVO/TR直接マッピングは使用しない
(24スロットをCANバス上の各ノードへ分配する。README.md参照)。
MODE_IO時のみ:
0: デバッグ用
1: MD1 PWM指令 (ENC1_MD == 1 の場合のみ有効。符号=方向、絶対値=デューティ)
2: MD2 PWM指令 (ENC2_MD == 1 の場合のみ有効。符号=方向、絶対値=デューティ)
3~8: 予備(未使用)
9~16: SERVO1~8
17~23: TR1~7 (実機は非搭載のため未使用)
*/

volatile int16_t CanIoRxData[CAN_IO_SLOT_COUNT] = {0}; // CANモード用の受信用IO値
/*
0: SERVO1
1: SERVO2
2: SERVO3
3: MD1 PWM指令 (ENC1_MD == 1 の場合のみ有効。符号=方向、絶対値=デューティ)
4: MD2 PWM指令 (ENC2_MD == 1 の場合のみ有効。符号=方向、絶対値=デューティ)
(SERVOn は SWn とピン共有。config.hpp の MULTIn で入出力を切替)
(MDn は ENCn とピン共有。config.hpp の ENCn_MD で入出力を切替)
*/

volatile int16_t CanIoTxData[CAN_IO_SLOT_COUNT] = {0}; // CANモード用の送信用IO値
/*
0~2: SW1~3
3: ENC1 raw pulse (ENC1_MD == 1 の場合は常に0)
4: ENC2 raw pulse (ENC2_MD == 1 の場合は常に0)
*/
