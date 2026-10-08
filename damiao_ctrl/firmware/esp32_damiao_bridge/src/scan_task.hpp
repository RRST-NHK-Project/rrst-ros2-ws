/*====================================================================
<scan_task.hpp>
・DAMIAO_SCAN_MODE(config.hpp)専用: CAN ID/ビットレート探索タスク

達妙デバッグツールが無い環境で、DM3520(ESC)の実際のCAN IDとビットレートを
安全に(速度・角度指令を一切送らず)特定するための診断タスク。
====================================================================*/

#pragma once

#include "driver/twai.h"
#include <Arduino.h>

void scanTask(void *pvParameters);
