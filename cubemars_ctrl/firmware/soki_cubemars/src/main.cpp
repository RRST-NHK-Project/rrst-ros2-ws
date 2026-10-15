/*====================================================================
Project: xiao-esp32-s3_can2io
Target board: XIAO ESP32-S3 with can2io board

Description:
  ROS 2・マイコン間の通信を行うserial_bridgeパッケージのマイコン側プログラム。
  PCから送られてくるバイナリデータを受信、デコードしマイコンのGPIO出力に反映させる。
  config.hppで各種設定をするだけで使用可能です。このファイル(main.cpp)を直接編集しないこと。

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "can_task.hpp"
#include "config.hpp"
#include "cubemars.hpp"
#include "defs.hpp"
#include "pin_ctrl_task.hpp"
#include "robomas.hpp"
#include "serial_task.hpp"
#include <Arduino.h>
// ================= SETUP =================

void setup() {

    // ボーレートは実機テストしながら調整する予定
    Serial.begin(115200);

    delay(200);
    delay(1 * DEVICE_ID); // 安定待ち, IDごとに開始タイミングをずらす

    pinMode(LED, OUTPUT);

    // ready
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED, HIGH);
        delay(50);
        digitalWrite(LED, LOW);
        delay(50);
    }

    // ledcSetup(1, 20000, 8);
    // ledcAttachPin(LED, 1);

#if defined(MODE_CAN_HOST) || defined(MODE_IO) || defined(MODE_DEBUG) || defined(MODE_ROBOMAS) || defined(MODE_CUBEMARS)
    xTaskCreate(
        serialTask,   // タスク関数
        "serialTask", // タスク名
        2048,         // スタックサイズ（words）
        NULL,
        10, // 優先度
        NULL);
#endif

#if defined(MODE_CAN) || defined(MODE_CAN_HOST) || defined(MODE_CAN_MONITOR)
    canInit();
    xTaskCreate(
        canTask,   // タスク関数
        "canTask", // タスク名
        4096,      // スタックサイズ（words）
        NULL,
        10, // 優先度
        NULL);
#endif

    // モードに応じた初期化
    // #if defined(MODE_OUTPUT)
    //     // 出力モード初期化
    //     xTaskCreate(
    //         Output_Task,   // タスク関数
    //         "Output_Task", // タスク名
    //         2048,          // スタックサイズ（words）
    //         NULL,
    //         11, // 優先度
    //         NULL);

    // #elif defined(MODE_INPUT)
    //     // 入力モード初期化
    //     xTaskCreate(
    //         Input_Task,   // タスク関数
    //         "Input_Task", // タスク名
    //         1024,         // スタックサイズ（words）
    //         NULL,
    //         4, // 優先度
    //         NULL);

#if defined(MODE_IO)
    // 入出力モード初期化
    xTaskCreate(
        IO_Task,   // タスク関数
        "IO_Task", // タスク名
        2048,      // スタックサイズ（words）
        NULL,
        11, // 優先度
        NULL);

#elif defined(MODE_CAN)
    // CANノードモード初期化
    xTaskCreate(
        IO_Task,   // タスク関数
        "IO_Task", // タスク名
        2048,      // スタックサイズ（words）
        NULL,
        11, // 優先度
        NULL);

#elif defined(MODE_CAN_HOST)
    // CANホストモード初期化
    // HOSTも1バス4マイコンの一台としてIO_Taskを回す
    xTaskCreate(
        IO_Task,   // タスク関数
        "IO_Task", // タスク名
        2048,      // スタックサイズ（words）
        NULL,
        11, // 優先度
        NULL);

#elif defined(MODE_DEBUG)
    // デバッグモード初期化

    // xTaskCreate(
    //     LED_PWM_Task,   // タスク関数
    //     "LED_PWM_Task", // タスク名
    //     1024,           // スタックサイズ（words）
    //     NULL,
    //     9, // 優先度
    //     NULL)0;

    // xTaskCreate(
    //     LED_Blink100_Task,   // タスク関数
    //     "LED_Blink100_Task", // タスク名
    //     1024,                // スタックサイズ（words）
    //     NULL,
    //     9, // 優先度
    //     NULL);

    xTaskCreate(
        PID_Task,   // タスク関数
        "PID_Task", // タスク名
        2048,       // スタックサイズ（words）
        NULL,
        11, // 優先度
        NULL);

#elif defined(MODE_CAN_MONITOR)
    // CANモニタモード初期化
    // シリアルモニタへ受信したCAN IDと生データだけを表示する
    // 追加のIOタスクは起動しない

#elif defined(MODE_ROBOMAS)
    // ロボマス専用モード初期化
    // xiao-esp32-s3_can2ioの他モードが使うノード/スロット分配プロトコルとは
    // 別系統で、DJI RoboMasterシリーズのCANプロトコル(1Mbps固定)を直接喋る独立デバイス
    // として動作する。canInit()/canTask()は使わず、robomas.cpp側で独自にCANを初期化する。
    // ノード/スロット分配側も1Mbpsに統一済みのため同一物理バスへの混在が可能。
    robomasInit();
    xTaskCreate(
        robomasTask,   // タスク関数
        "robomasTask", // タスク名
        4096,          // スタックサイズ（words）
        NULL,
        11, // 優先度
        NULL);

#elif defined(MODE_CUBEMARS)
    // CubeMars AKシリーズ専用モード初期化
    // xiao-esp32-s3_can2ioの他モードが使うノード/スロット分配プロトコルとは
    // 別系統で、CubeMars AKシリーズのServo(CAN)モードプロトコル(1Mbps固定)を直接喋る
    // 独立デバイスとして動作する。canInit()/canTask()は使わず、cubemars.cpp側で
    // 独自にCANを初期化する。ノード/スロット分配側も1Mbpsに統一済みのため
    // 同一物理バスへの混在が可能。
    cubemarsInit();
    xTaskCreate(
        cubemarsTask,   // タスク関数
        "cubemarsTask", // タスク名
        4096,           // スタックサイズ（words）
        NULL,
        11, // 優先度
        NULL);

#else
#error "No mode defined. Please define one mode in config.hpp."
#endif

#if (defined(MODE_IO) + defined(MODE_CAN) + defined(MODE_CAN_HOST) + defined(MODE_DEBUG) + defined(MODE_CAN_MONITOR) + defined(MODE_ROBOMAS) + defined(MODE_CUBEMARS)) != 1
#error "Invalid mode configuration. Please define exactly *one mode* in config.hpp."
#endif

#if defined(MODE_IO) && BOARD_VARIANT != BOARD_SOKI
#error "MODE_IO is only implemented for BOARD_SOKI (Rx_16Data/Tx_16Data direct-index layout assumes SOKI's SERVO/SW pin mapping). Use MODE_CAN/MODE_CAN_HOST for BOARD_MES/BOARD_SS."
#endif
}

// ================= LOOP =================

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // メインループはなにもしない、処理はすべてFreeRTOSタスクで行う
}
