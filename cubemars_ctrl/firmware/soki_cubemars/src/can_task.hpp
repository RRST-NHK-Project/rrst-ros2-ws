/*====================================================================
<can_task.hpp>
・CAN通信（TWAI + MCP2561）を用いた制御データ転送のタスク宣言
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

void canInit();
void canTask(void *arg);
