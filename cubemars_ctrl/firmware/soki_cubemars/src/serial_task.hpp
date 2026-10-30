/*====================================================================
<serial_task.hpp>
・シリアル通信関連のタスクと関数のヘッダーファイル
Copyright (c) 2025 RRST-NHK-Project
SPDX-License-Identifier: MIT
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void serialTask(void *); // 送受信を同時に行うタスク
void txTask(void *);     // 送信タスク（削除予定）
void rxTask(void *);     // 受信タスク（削除予定）
void send_frame();
void receive_frame();

// 最後に自分宛ての有効なフレームを受信した時刻[ms](millis())。一度も受信して
// いなければ false を返す。[cubemars_ctrlで追加] cubemars.cppのフェイルセーフが参照する。
bool serialLastValidRxMs(uint32_t *out_ms);