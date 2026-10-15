/*====================================================================
<status_led.hpp>
・通信アクティビティ表示用LEDの共通ヘルパー
CAN/シリアルそれぞれの活動を別々に記録し、CANが動いていれば高速点滅、
CANは止まっていてシリアルだけ動いていれば短いパルス点滅、
どちらも途絶えたら消灯、という3状態をstatusLedUpdate()側で表現する。
serialTask/canTaskの両方から呼び出されることを想定している。
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

// CANで有効なフレームを受信した際に呼び出す。CAN側の消灯タイマーをリセットする。
void statusLedPulseCan();

// シリアルで有効なフレームを受信した際に呼び出す。シリアル側の消灯タイマーをリセットする。
void statusLedPulseSerial();

// 各タスクのループから毎回呼び出す。CAN/シリアルの活動状況に応じてLEDパターンを更新する。
void statusLedUpdate();
