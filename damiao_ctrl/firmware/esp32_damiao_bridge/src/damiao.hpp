/*====================================================================
<damiao.hpp>
・Damiao DM2325(モータ) + DM3520(ESC) 用CANドライバのヘッダーファイル

ros2canのcubemars.hpp/cubemars.cppと同じ考え方(ノード/スロット分配とは
別系統で、独立デバイスとして直接CANを喋るタスク)に倣っている。
スロット割り当て・プロトコル詳細はdamiao.cppの先頭コメントを参照。
====================================================================*/

#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"
#include <Arduino.h>

// CANドライバ初期化
void damiaoInit();

// CAN送受信タスク本体
void damiaoTask(void *pvParameters);
