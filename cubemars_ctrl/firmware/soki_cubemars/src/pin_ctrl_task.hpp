/*====================================================================
<pin_ctrl_task.hpp>
・ピン操作関連の関数とタスクのヘッダーファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void IO_Task(void *); // 入出力タスク
void SDM15_Task(void *);
void IO_MD_Output();
void IO_Servo_Outout();
void IO_TR_Output();
void IO_ENC_Input();
void IO_SW_Input();
