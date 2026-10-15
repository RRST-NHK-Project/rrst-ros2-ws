/*====================================================================
<pin_ctrl_init.cpp>
・ピン初期化関連の関数実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "config.hpp"
#include "defs.hpp"
#include "driver/pcnt.h"
#include "frame_data.hpp"
#include <Arduino.h>

constexpr uint32_t CTRL_PERIOD_MS = 5; // ピン更新周期（ミリ秒）

void IO_init();

#if BOARD_VARIANT == BOARD_SOKI

void IO_init() {
    // 入出力の初期化

    // PWMの初期化

    // サーボのPWMの初期化
    ledcSetup(4, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcSetup(5, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcSetup(6, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);

    if (MULTI1 == 1) {
        ledcAttachPin(SERVO1, 4);
    }
    if (MULTI2 == 1) {
        ledcAttachPin(SERVO2, 5);
    }
    if (MULTI3 == 1) {
        ledcAttachPin(SERVO3, 6);
    }

#if ENC1_MD == 1
    // MD1の初期化 (ENC1_A/ENC1_BをPWM/DIRとして使用)
    pinMode(MD1D, OUTPUT);
    ledcSetup(0, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttachPin(MD1P, 0);
#else
    // プルアップを有効化
    gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLUP_ONLY);

    // パルスカウンタの設定
    pcnt_config_t pcnt_config1 = {};
    pcnt_config1.pulse_gpio_num = ENC1_A;
    pcnt_config1.ctrl_gpio_num = ENC1_B;
    pcnt_config1.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config1.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config1.pos_mode = PCNT_COUNT_INC;
    pcnt_config1.neg_mode = PCNT_COUNT_DEC;
    pcnt_config1.counter_h_lim = COUNTER_H_LIM;
    pcnt_config1.counter_l_lim = COUNTER_L_LIM;
    pcnt_config1.unit = PCNT_UNIT_0;
    pcnt_config1.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config2 = {};
    pcnt_config2.pulse_gpio_num = ENC1_B;
    pcnt_config2.ctrl_gpio_num = ENC1_A;
    pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    pcnt_config2.counter_h_lim = COUNTER_H_LIM;
    pcnt_config2.counter_l_lim = COUNTER_L_LIM;
    pcnt_config2.unit = PCNT_UNIT_0;
    pcnt_config2.channel = PCNT_CHANNEL_1;

    pcnt_unit_config(&pcnt_config1);
    pcnt_unit_config(&pcnt_config2);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    // チャタリング防止のフィルターを有効化
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);
#endif

#if ENC2_MD == 1
    // MD2の初期化 (ENC2_A/ENC2_BをPWM/DIRとして使用)
    pinMode(MD2D, OUTPUT);
    ledcSetup(1, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttachPin(MD2P, 1);
#else
    // プルアップを有効化
    gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLUP_ONLY);

    // パルスカウンタの設定
    pcnt_config_t pcnt_config3 = {};
    pcnt_config3.pulse_gpio_num = ENC2_A;
    pcnt_config3.ctrl_gpio_num = ENC2_B;
    pcnt_config3.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config3.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config3.pos_mode = PCNT_COUNT_INC;
    pcnt_config3.neg_mode = PCNT_COUNT_DEC;
    pcnt_config3.counter_h_lim = COUNTER_H_LIM;
    pcnt_config3.counter_l_lim = COUNTER_L_LIM;
    pcnt_config3.unit = PCNT_UNIT_1;
    pcnt_config3.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config4 = {};
    pcnt_config4.pulse_gpio_num = ENC2_B;
    pcnt_config4.ctrl_gpio_num = ENC2_A;
    pcnt_config4.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config4.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config4.pos_mode = PCNT_COUNT_INC;
    pcnt_config4.neg_mode = PCNT_COUNT_DEC;
    pcnt_config4.counter_h_lim = COUNTER_H_LIM;
    pcnt_config4.counter_l_lim = COUNTER_L_LIM;
    pcnt_config4.unit = PCNT_UNIT_1;
    pcnt_config4.channel = PCNT_CHANNEL_1;

    pcnt_unit_config(&pcnt_config3);
    pcnt_unit_config(&pcnt_config4);

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);

    // チャタリング防止のフィルターを有効化
    pcnt_filter_enable(PCNT_UNIT_1);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE);
#endif

    // SW ピン初期化
    if (MULTI1 == 0) {
        pinMode(SW1, INPUT_PULLUP);
    }
    if (MULTI2 == 0) {
        pinMode(SW2, INPUT_PULLUP);
    }
    if (MULTI3 == 0) {
        pinMode(SW3, INPUT_PULLUP);
    }
}

#elif BOARD_VARIANT == BOARD_MES

void IO_init() {
    // MD1/MD2はENCと共有せず常時専用ピン(defs.hpp参照)。ENC1_MD/ENC2_MDは
    // BOARD_SOKI専用の設定のためここでは参照しない。
    pinMode(MD1D, OUTPUT);
    ledcSetup(0, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttachPin(MD1P, 0);

    pinMode(MD2D, OUTPUT);
    ledcSetup(1, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttachPin(MD2P, 1);

    // ENC1: 常時エンコーダとして使用
    gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLUP_ONLY);

    pcnt_config_t pcnt_config1 = {};
    pcnt_config1.pulse_gpio_num = ENC1_A;
    pcnt_config1.ctrl_gpio_num = ENC1_B;
    pcnt_config1.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config1.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config1.pos_mode = PCNT_COUNT_INC;
    pcnt_config1.neg_mode = PCNT_COUNT_DEC;
    pcnt_config1.counter_h_lim = COUNTER_H_LIM;
    pcnt_config1.counter_l_lim = COUNTER_L_LIM;
    pcnt_config1.unit = PCNT_UNIT_0;
    pcnt_config1.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config2 = {};
    pcnt_config2.pulse_gpio_num = ENC1_B;
    pcnt_config2.ctrl_gpio_num = ENC1_A;
    pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    pcnt_config2.counter_h_lim = COUNTER_H_LIM;
    pcnt_config2.counter_l_lim = COUNTER_L_LIM;
    pcnt_config2.unit = PCNT_UNIT_0;
    pcnt_config2.channel = PCNT_CHANNEL_1;

    pcnt_unit_config(&pcnt_config1);
    pcnt_unit_config(&pcnt_config2);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);

    // ENC2/SW2/SW3: ピン共有。ENC2_SW(config.hpp)で選択
#if ENC2_SW == 0
    gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLUP_ONLY);

    pcnt_config_t pcnt_config3 = {};
    pcnt_config3.pulse_gpio_num = ENC2_A;
    pcnt_config3.ctrl_gpio_num = ENC2_B;
    pcnt_config3.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config3.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config3.pos_mode = PCNT_COUNT_INC;
    pcnt_config3.neg_mode = PCNT_COUNT_DEC;
    pcnt_config3.counter_h_lim = COUNTER_H_LIM;
    pcnt_config3.counter_l_lim = COUNTER_L_LIM;
    pcnt_config3.unit = PCNT_UNIT_1;
    pcnt_config3.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config4 = {};
    pcnt_config4.pulse_gpio_num = ENC2_B;
    pcnt_config4.ctrl_gpio_num = ENC2_A;
    pcnt_config4.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config4.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config4.pos_mode = PCNT_COUNT_INC;
    pcnt_config4.neg_mode = PCNT_COUNT_DEC;
    pcnt_config4.counter_h_lim = COUNTER_H_LIM;
    pcnt_config4.counter_l_lim = COUNTER_L_LIM;
    pcnt_config4.unit = PCNT_UNIT_1;
    pcnt_config4.channel = PCNT_CHANNEL_1;

    pcnt_unit_config(&pcnt_config3);
    pcnt_unit_config(&pcnt_config4);

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);

    pcnt_filter_enable(PCNT_UNIT_1);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE);
#else
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
#endif

    // SW1: 専用ピン、常時スイッチ
    pinMode(SW1, INPUT_PULLUP);
}

#elif BOARD_VARIANT == BOARD_SS

void IO_init() {
    // サーボ5ch分のPWMを初期化(常時サーボ、MULTIによる切替なし)
    // ESP32-S3のLEDCチャンネルは0-7の8chのみ(MDが無いBOARD_SSでは0-4を使用)。
    ledcSetup(0, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcSetup(1, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcSetup(2, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcSetup(3, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcSetup(4, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttachPin(SERVO1, 0);
    ledcAttachPin(SERVO2, 1);
    ledcAttachPin(SERVO3, 2);
    ledcAttachPin(SERVO4, 3);
    ledcAttachPin(SERVO5, 4);

    // ソレノイドバルブ(TR1-4): デジタルON/OFF出力、起動時OFF
    pinMode(TR1, OUTPUT);
    pinMode(TR2, OUTPUT);
    pinMode(TR3, OUTPUT);
    pinMode(TR4, OUTPUT);
    digitalWrite(TR1, LOW);
    digitalWrite(TR2, LOW);
    digitalWrite(TR3, LOW);
    digitalWrite(TR4, LOW);
}

#endif