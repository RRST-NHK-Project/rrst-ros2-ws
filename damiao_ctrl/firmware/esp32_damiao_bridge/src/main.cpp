/*====================================================================
Project: esp32_damiao_bridge
Target board: Seeed XIAO ESP32-S3 + MCP2561 CANトランシーバ

Description:
  damiao_ctrl(ROS 2パッケージ)のPC側とDamiao DM2325+DM3520(ESC)を
  橋渡しするファームウェア。RRST-NHK-Project/ros2can
  (firmware/xiao-esp32-s3_can2io) と同じファイル構成・52バイトフレーム
  プロトコル・FreeRTOSタスク構成に倣っている。

  serialTask: PCとのUSBシリアル通信(52バイトフレーム, Rx_16Data/Tx_16Data)
  damiaoTask: DM3520とのCAN通信(Damiao公式プロトコルのVEL/POS_VELモード)

  設定はconfig.hppで行うこと。このファイルは直接編集不要。
====================================================================*/

#include "config.hpp"
#include "damiao.hpp"
#include "frame_data.hpp"
#include "logic_probe.hpp"
#include "pin_hunt.hpp"
#include "scan_task.hpp"
#include "serial_task.hpp"
#include <Arduino.h>

void setup() {
    Serial.begin(115200);

    delay(200);
    delay(1 * DEVICE_ID); // 安定待ち、IDごとに開始タイミングをずらす

#if DAMIAO_PIN_HUNT_MODE
    // CANピン特定モード。pinHuntTaskのみ動作する。
    xTaskCreate(
        pinHuntTask,   // タスク関数
        "pinHuntTask", // タスク名
        4096,          // スタックサイズ(words)
        NULL,
        11, // 優先度
        NULL);
#elif DAMIAO_LOGIC_PROBE_MODE
    // 簡易ロジックアナライザモード。logicProbeTaskのみ動作する。
    xTaskCreate(
        logicProbeTask,   // タスク関数
        "logicProbeTask", // タスク名
        4096,             // スタックサイズ(words)
        NULL,
        11, // 優先度
        NULL);
#elif DAMIAO_SCAN_MODE
    // CAN ID/ビットレート探索モード。scanTaskのみ動作し、PCとの52バイト
    // フレーム通信(serialTask)・実際のモータ制御(damiaoTask)は行わない。
    xTaskCreate(
        scanTask,   // タスク関数
        "scanTask", // タスク名
        4096,       // スタックサイズ(words)
        NULL,
        11, // 優先度
        NULL);
#else
    damiaoInit();

    xTaskCreate(
        serialTask,   // タスク関数
        "serialTask", // タスク名
        2048,         // スタックサイズ(words)
        NULL,
        10, // 優先度
        NULL);

    xTaskCreate(
        damiaoTask,   // タスク関数
        "damiaoTask", // タスク名
        4096,         // スタックサイズ(words)
        NULL,
        11, // 優先度
        NULL);
#endif
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // メインループはなにもしない、処理はすべてFreeRTOSタスクで行う
}
