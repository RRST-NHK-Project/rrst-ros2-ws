/*====================================================================
<output_task.cpp>
・出力系タスク（ロボマスを除くアクチュエータ類）の実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include <Arduino.h>
#include <defs.h>
#include <esp32-hal-ledc.h>
#include <output_task.h>

// 受信データ格納用
extern int32_t received_data[MAX_ARRAY_SIZE]; // 受信データ

void Output_Task(void *pvParameters) {
    while (1) {

        // MD出力の制限
        received_data[1] = constrain(received_data[1], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[2] = constrain(received_data[2], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[3] = constrain(received_data[3], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[4] = constrain(received_data[4], -MD_PWM_MAX, MD_PWM_MAX);

        // ピンの操作
        digitalWrite(MD1D, received_data[1] > 0 ? HIGH : LOW);
        digitalWrite(MD2D, received_data[2] > 0 ? HIGH : LOW);
        digitalWrite(MD3D, received_data[3] > 0 ? HIGH : LOW);
        digitalWrite(MD4D, received_data[4] > 0 ? HIGH : LOW);

        ledcWrite(MD1P, abs(received_data[1]));
        ledcWrite(MD2P, abs(received_data[2]));
        ledcWrite(MD3P, abs(received_data[3]));
        ledcWrite(MD4P, abs(received_data[4]));

        // サーボ1
        int angle1 = received_data[5];
        if (angle1 < SERVO1_MIN_DEG)
            angle1 = SERVO1_MIN_DEG;
        if (angle1 > SERVO1_MAX_DEG)
            angle1 = SERVO1_MAX_DEG;
        int us1 = map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
        int duty1 = (int)(us1 * SERVO_PWM_SCALE);
        ledcWrite(SERVO1, duty1);

        // サーボ2
        int angle2 = received_data[6];
        if (angle2 < SERVO2_MIN_DEG)
            angle2 = SERVO2_MIN_DEG;
        if (angle2 > SERVO2_MAX_DEG)
            angle2 = SERVO2_MAX_DEG;
        int us2 = map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
        int duty2 = (int)(us2 * SERVO_PWM_SCALE);
        ledcWrite(SERVO2, duty2);

        // サーボ3
        int angle3 = received_data[7];
        if (angle3 < SERVO3_MIN_DEG)
            angle3 = SERVO3_MIN_DEG;
        if (angle3 > SERVO3_MAX_DEG)
            angle3 = SERVO3_MAX_DEG;
        int us3 = map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
        int duty3 = (int)(us3 * SERVO_PWM_SCALE);
        ledcWrite(SERVO3, duty3);

        // サーボ4
        int angle4 = received_data[8];
        if (angle4 < SERVO4_MIN_DEG)
            angle4 = SERVO4_MIN_DEG;
        if (angle4 > SERVO4_MAX_DEG)
            angle4 = SERVO4_MAX_DEG;
        int us4 = map(angle4, SERVO4_MIN_DEG, SERVO4_MAX_DEG, SERVO4_MIN_US, SERVO4_MAX_US);
        int duty4 = (int)(us4 * SERVO_PWM_SCALE);
        ledcWrite(SERVO4, duty4);

        digitalWrite(TR1, received_data[11] ? HIGH : LOW);
        digitalWrite(TR2, received_data[12] ? HIGH : LOW);
        digitalWrite(TR3, received_data[13] ? HIGH : LOW);
        digitalWrite(TR4, received_data[14] ? HIGH : LOW);
        digitalWrite(TR5, received_data[15] ? HIGH : LOW);
        digitalWrite(TR6, received_data[16] ? HIGH : LOW);
        digitalWrite(TR7, received_data[17] ? HIGH : LOW);

        vTaskDelay(1); // WDTのリセット(必須)
    }
}

void LED_Blink100_Task(void *pvParameters) {
    while (1) {
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
    }
}

// ledcWrite,vTaskDelayに書き換え予定
void LED_PWM_Task(void *pvParameters) {
    while (1) {
        for (int val = 0; val <= 255; val++) {
            analogWrite(LED, val);
            delay(5);
        }
        for (int val = 255; val >= 0; val--) {
            analogWrite(LED, val);
            delay(5);
        }
    }
}