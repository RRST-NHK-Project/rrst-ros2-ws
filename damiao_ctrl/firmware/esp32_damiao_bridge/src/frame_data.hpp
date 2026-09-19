/*====================================================================
<frame_data.hpp>
・シリアル通信のフレームデータ定義ヘッダーファイル
ros2can(xiao-esp32-s3_can2io)のfrom_data.hppと同じ24スロットint16配列。
====================================================================*/

#pragma once
#include <stdint.h>

#define Tx16NUM 24 // 送信(本機->PC)するint16データの数
#define Rx16NUM 24 // 受信(PC->本機)するint16データの数

// PC -> 本機 (指令)。スロット割り当ては damiao_ctrl/slot_layout.hpp と
// 一致させること(6スロット/モータ、モータ1は0-5)。
extern volatile int16_t Rx_16Data[Rx16NUM];

// 本機 -> PC (帰還)
extern volatile int16_t Tx_16Data[Tx16NUM];
