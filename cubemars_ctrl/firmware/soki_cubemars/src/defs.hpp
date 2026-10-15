/*====================================================================
<defs.h>
・定数の定義ファイル

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include "config.hpp"
#include <Arduino.h>

// ================= ピンの定義 =================

// パルスカウンタの上限・下限の定義
#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768
#define PCNT_FILTER_VALUE 1023 // 0~1023, 1 = 12.5ns

#define ENC_PPR_SPEC 2048      // エンコーダの設定値
#define PPR (ENC_PPR_SPEC * 4) // 実効PPR（x4カウント）

#define DEG_PER_COUNT (360.0f / PPR)
#define HALF_PPR (PPR / 2)

// ピンの定義 //
// 基板(BOARD_VARIANT、config.hpp)ごとに配置が異なる。
#if BOARD_VARIANT == BOARD_SOKI

// 状態表示LED
#define LED 21

// サーボ
#define SERVO1 7
#define SERVO2 8
#define SERVO3 9

// エンコーダ
#define ENC1_A 3
#define ENC1_B 4
#define ENC2_A 5
#define ENC2_B 6

// MD (ENCとピン共有。config.hppのENC1_MD/ENC2_MDで切替)
#define MD1P ENC1_A // MD1 PWM
#define MD1D ENC1_B // MD1 DIR
#define MD2P ENC2_A // MD2 PWM
#define MD2D ENC2_B // MD2 DIR

// スイッチ
#define SW1 7
#define SW2 8
#define SW3 9

// CAN (MCP2561 + TWAI)
#define CAN_RX 2
#define CAN_TX 1

#elif BOARD_VARIANT == BOARD_MES

// 状態表示LED
#define LED 21

// エンコーダ
#define ENC1_A 5
#define ENC1_B 6
#define ENC2_A 7 // SW2のピンと共有
#define ENC2_B 8 // SW3のピンと共有

// MD (SOKI基板と異なりENCとピンを共有しない、常時専用ピン)
#define MD1P 1
#define MD1D 3
#define MD2P 2
#define MD2D 4

// スイッチ (ENC2とピン共有。config.hppのENC2_SWで切替)
#define SW1 9
#define SW2 7 // ENC2_Aのピンと共有
#define SW3 8 // ENC2_Bのピンと共有

// CAN (MCP2561 + TWAI)
#define CAN_RX 44
#define CAN_TX 43

#elif BOARD_VARIANT == BOARD_SS

// 状態表示LED
#define LED 21

// サーボ (5ch、常時サーボ出力。MULTIによる切替なし)
#define SERVO1 1
#define SERVO2 2
#define SERVO3 3
#define SERVO4 4
#define SERVO5 5

// ソレノイドバルブ (デジタルON/OFF、0以外でON。pin_ctrl_task.cppのIO_TR_Output参照)
#define TR1 6
#define TR2 7
#define TR3 8
#define TR4 9

// CAN (MCP2561 + TWAI)
#define CAN_RX 44
#define CAN_TX 43

#else
#error "BOARD_VARIANT: unknown board"
#endif

// MD用
#define MD_PWM_MAX ((1 << MD_PWM_RESOLUTION) - 1)

// サーボ用
#define SERVO_PWM_PERIOD_US (1000000.0 / SERVO_PWM_FREQ) // 周波数から周期を計算
#define SERVO_PWM_MAX_DUTY ((1 << SERVO_PWM_RESOLUTION) - 1)
#define SERVO_PWM_SCALE (SERVO_PWM_MAX_DUTY / SERVO_PWM_PERIOD_US)

// ロボマス (MODE_ROBOMAS、詳細はrobomas.hpp/cppを参照)
#define NUM_MOTOR 4                    // 1バスあたりのモーター数
#define gear_m3508 19.2f               // M3508ギア比
#define gear_m2006 36.0f               // M2006ギア比
#define ENCODER_MAX 8192               // エンコーダ分解能
#define HALF_ENCODER (ENCODER_MAX / 2) // エンコーダ分解能の半分