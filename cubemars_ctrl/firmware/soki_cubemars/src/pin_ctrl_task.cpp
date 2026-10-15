/*====================================================================
<pin_ctrl_task.cpp>
・ピン操作関連の関数とタスクの実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "config.hpp"
#include "defs.hpp"
#include "driver/pcnt.h"
#include "frame_data.hpp"
#include "pin_ctrl_init.hpp"
#include <Arduino.h>

constexpr uint32_t CTRL_PERIOD_MS = 5; // ピン更新周期（ミリ秒）

void IO_MD_Output();
void IO_Servo_Outout();
void IO_TR_Output();
void IO_ENC_Input();
void IO_SW_Input();

void IO_Task(void *);

// ================= TASK =================

void IO_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    IO_init();

    while (1) {
        IO_MD_Output();
        IO_Servo_Outout();
        IO_TR_Output();
        IO_ENC_Input();
        IO_SW_Input();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

// ================= 関数 =================
// マイコンや基板の不具合に対応するためにfor文は使っていない。
// レイアウトはBOARD_VARIANT(config.hpp)ごとに異なる。CanIoRxData/CanIoTxDataの
// スロット割当はframe_data.hppのコメント参照。

#if BOARD_VARIANT == BOARD_SOKI
// 実機は ENCx2, SWx3, SERVOx3のみ (SW/SERVOはピン共有でconfig.hppのMULTI1-3により切替、
// ENC/MDはピン共有でconfig.hppのENC1_MD/ENC2_MDにより切替)

void IO_MD_Output() {
    // MD出力処理 (ENC1_MD/ENC2_MDでMDに切替えたチャンネルのみ)
#if defined(MODE_CAN) || defined(MODE_CAN_HOST)
#if ENC1_MD == 1
    int md1 = constrain((int)CanIoRxData[3], -MD_PWM_MAX, MD_PWM_MAX);
    digitalWrite(MD1D, md1 > 0 ? HIGH : LOW);
    ledcWrite(0, abs(md1));
#endif
#if ENC2_MD == 1
    int md2 = constrain((int)CanIoRxData[4], -MD_PWM_MAX, MD_PWM_MAX);
    digitalWrite(MD2D, md2 > 0 ? HIGH : LOW);
    ledcWrite(1, abs(md2));
#endif
#else
#if ENC1_MD == 1
    int md1 = constrain((int)Rx_16Data[1], -MD_PWM_MAX, MD_PWM_MAX);
    digitalWrite(MD1D, md1 > 0 ? HIGH : LOW);
    ledcWrite(0, abs(md1));
#endif
#if ENC2_MD == 1
    int md2 = constrain((int)Rx_16Data[2], -MD_PWM_MAX, MD_PWM_MAX);
    digitalWrite(MD2D, md2 > 0 ? HIGH : LOW);
    ledcWrite(1, abs(md2));
#endif
#endif
}

void IO_Servo_Outout() {
#if defined(MODE_CAN) || defined(MODE_CAN_HOST)
    if (MULTI1 == 1) {
        int angle1 = CanIoRxData[0];
        angle1 = constrain(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG);
        int us1 = (int)map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
        int duty1 = (int)(us1 * SERVO_PWM_SCALE);
        ledcWrite(4, duty1);
    }

    if (MULTI2 == 1) {
        int angle2 = CanIoRxData[1];
        angle2 = constrain(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
        int us2 = (int)map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
        int duty2 = (int)(us2 * SERVO_PWM_SCALE);
        ledcWrite(5, duty2);
    }

    if (MULTI3 == 1) {
        int angle3 = CanIoRxData[2];
        angle3 = constrain(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG);
        int us3 = (int)map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
        int duty3 = (int)(us3 * SERVO_PWM_SCALE);
        ledcWrite(6, duty3);
    }
#else
    if (MULTI1 == 1) {
        int angle1 = Rx_16Data[9];
        angle1 = constrain(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG);
        int us1 = (int)map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
        int duty1 = (int)(us1 * SERVO_PWM_SCALE);
        ledcWrite(4, duty1);
    }

    if (MULTI2 == 1) {
        int angle2 = Rx_16Data[10];
        angle2 = constrain(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
        int us2 = (int)map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
        int duty2 = (int)(us2 * SERVO_PWM_SCALE);
        ledcWrite(5, duty2);
    }

    if (MULTI3 == 1) {
        int angle3 = Rx_16Data[11];
        angle3 = constrain(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG);
        int us3 = (int)map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
        int duty3 = (int)(us3 * SERVO_PWM_SCALE);
        ledcWrite(6, duty3);
    }
#endif
}

void IO_TR_Output() {
    // BOARD_SOKIにTRポートは無い
}

void IO_ENC_Input() {
    // ENC入力処理 (ENC1_MD/ENC2_MDでMDに切替えたチャンネルは0のまま送る)
    int16_t enc1 = 0;
    int16_t enc2 = 0;
#if ENC1_MD == 0
    pcnt_get_counter_value(PCNT_UNIT_0, &enc1);
#endif
#if ENC2_MD == 0
    pcnt_get_counter_value(PCNT_UNIT_1, &enc2);
#endif
#if defined(MODE_CAN) || defined(MODE_CAN_HOST)
    CanIoTxData[3] = enc1;
    CanIoTxData[4] = enc2;
#elif !defined(MODE_CAN_HOST)
    // ホストモードでは CAN からのフィードバックを優先して、ローカルエンコーダ値で上書きしない
    Tx_16Data[1] = enc1;
    Tx_16Data[2] = enc2;
#endif
}

void IO_SW_Input() {
    // SW入力処理
#if defined(MODE_CAN) || defined(MODE_CAN_HOST)
    if (MULTI1 == 0) {
        CanIoTxData[0] = !digitalRead(SW1);
    } else {
        CanIoTxData[0] = 0;
    }

    if (MULTI2 == 0) {
        CanIoTxData[1] = !digitalRead(SW2);
    } else {
        CanIoTxData[1] = 0;
    }

    if (MULTI3 == 0) {
        CanIoTxData[2] = !digitalRead(SW3);
    } else {
        CanIoTxData[2] = 0;
    }
#elif !defined(MODE_CAN_HOST)
    if (MULTI1 == 0) {
        Tx_16Data[9] = !digitalRead(SW1);
    } else {
        Tx_16Data[9] = 0;
    }

    if (MULTI2 == 0) {
        Tx_16Data[10] = !digitalRead(SW2);
    } else {
        Tx_16Data[10] = 0;
    }

    if (MULTI3 == 0) {
        Tx_16Data[11] = !digitalRead(SW3);
    } else {
        Tx_16Data[11] = 0;
    }
#endif
}

#elif BOARD_VARIANT == BOARD_MES
// MD1/MD2は常時専用ピン(ENC1_MD/ENC2_MDは参照しない)。ENC2/SW2/SW3はENC2_SWで排他切替。
// CanIoRxData: 0=MD1, 1=MD2。CanIoTxData: 0=SW1, 1=ENC1, 2=ENC2(ENC2_SW==0)/SW2(==1),
// 3=SW3(ENC2_SW==1のみ)。frame_data.hppのコメントも参照。

void IO_MD_Output() {
    int md1 = constrain((int)CanIoRxData[0], -MD_PWM_MAX, MD_PWM_MAX);
    digitalWrite(MD1D, md1 > 0 ? HIGH : LOW);
    ledcWrite(0, abs(md1));

    int md2 = constrain((int)CanIoRxData[1], -MD_PWM_MAX, MD_PWM_MAX);
    digitalWrite(MD2D, md2 > 0 ? HIGH : LOW);
    ledcWrite(1, abs(md2));
}

void IO_Servo_Outout() {
    // BOARD_MESにサーボポートは無い
}

void IO_TR_Output() {
    // BOARD_MESにTRポートは無い
}

void IO_ENC_Input() {
    int16_t enc1 = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &enc1);
    CanIoTxData[1] = enc1;

#if ENC2_SW == 0
    int16_t enc2 = 0;
    pcnt_get_counter_value(PCNT_UNIT_1, &enc2);
    CanIoTxData[2] = enc2;
#endif
}

void IO_SW_Input() {
    CanIoTxData[0] = !digitalRead(SW1);

#if ENC2_SW == 1
    CanIoTxData[2] = !digitalRead(SW2);
    CanIoTxData[3] = !digitalRead(SW3);
#endif
}

#elif BOARD_VARIANT == BOARD_SS
// サーボ5ch常時出力+ソレノイドバルブ4ch(デジタルON/OFF)。ENC/MD/SWは無い。
// CanIoRxData: 0-4=SERVO1-5, 5-8=TR1-4(0以外でON)。CanIoTxData: 帰還なし(常に0)。

void IO_MD_Output() {
    // BOARD_SSにMDポートは無い
}

void IO_Servo_Outout() {
    int angle1 = constrain((int)CanIoRxData[0], SERVO1_MIN_DEG, SERVO1_MAX_DEG);
    ledcWrite(0, (int)(map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US) * SERVO_PWM_SCALE));

    int angle2 = constrain((int)CanIoRxData[1], SERVO2_MIN_DEG, SERVO2_MAX_DEG);
    ledcWrite(1, (int)(map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US) * SERVO_PWM_SCALE));

    int angle3 = constrain((int)CanIoRxData[2], SERVO3_MIN_DEG, SERVO3_MAX_DEG);
    ledcWrite(2, (int)(map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US) * SERVO_PWM_SCALE));

    int angle4 = constrain((int)CanIoRxData[3], SERVO4_MIN_DEG, SERVO4_MAX_DEG);
    ledcWrite(3, (int)(map(angle4, SERVO4_MIN_DEG, SERVO4_MAX_DEG, SERVO4_MIN_US, SERVO4_MAX_US) * SERVO_PWM_SCALE));

    int angle5 = constrain((int)CanIoRxData[4], SERVO5_MIN_DEG, SERVO5_MAX_DEG);
    ledcWrite(4, (int)(map(angle5, SERVO5_MIN_DEG, SERVO5_MAX_DEG, SERVO5_MIN_US, SERVO5_MAX_US) * SERVO_PWM_SCALE));
}

void IO_TR_Output() {
    digitalWrite(TR1, CanIoRxData[5] != 0 ? HIGH : LOW);
    digitalWrite(TR2, CanIoRxData[6] != 0 ? HIGH : LOW);
    digitalWrite(TR3, CanIoRxData[7] != 0 ? HIGH : LOW);
    digitalWrite(TR4, CanIoRxData[8] != 0 ? HIGH : LOW);
}

void IO_ENC_Input() {
    // BOARD_SSにENCポートは無い
}

void IO_SW_Input() {
    // BOARD_SSにSWポートは無い
}

#endif
