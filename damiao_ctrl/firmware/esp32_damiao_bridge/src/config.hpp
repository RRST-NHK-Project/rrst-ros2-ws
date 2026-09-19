/*====================================================================
<config.hpp>
書き込み前にここでID・CANパラメータを設定してください。

RRST-NHK-Project/ros2can (firmware/xiao-esp32-s3_can2io) の
config.hpp/defs.hppと同じ構成に合わせてある。damiao_ctrlは単一モータ
専用のためMODE_*切り替えは無く、常にserialTask(PC<->本機)+damiaoTask
(本機<->DM3520)の2タスク構成で動く。
====================================================================*/

#pragma once
#include <Arduino.h>

// ================= 基本設定 =================

// シリアルフレームのDEVICE_ID。PC側 bridge_node の device_id パラメータと
// 一致させること。
#define DEVICE_ID 1

// ================= CAN(TWAI)配線 =================
// XIAO ESP32-S3 + MCP2561 (ros2canのBOARD_SOKIと同じ組み合わせ)。
// 実機のCANトランシーバとの配線が違う場合はここを変更すること
// (candumpで送受信が確認できない場合、まずここを疑う)。
#define CAN_TX 1
#define CAN_RX 2

// CANビットレート。DM3520(ESC)側の設定(達妙デバッグツールで確認)と
// 必ず一致させること。不一致だとBus-Offになる。
// TODO: 実機のデバッグツールで実際の設定値を確認してから確定させる。
#define CAN_BITRATE_1MBPS 1 // 1=1Mbps(既定,ros2canの他モードと同じ), 0=500kbps

// ================= Damiaoモータ(DM2325+DM3520)設定 =================

// 接続台数。将来複数台に増やす場合はDAMIAO_MOTOR_ID_2以降を追加し、
// damiao_ctrl/include/damiao_ctrl/slot_layout.hppのkMaxMotors(=4)まで
// 拡張できる(cubemars.cppのCUBEMARS_MOTOR_COUNTと同じ考え方)。
#define DAMIAO_MOTOR_COUNT 1

// モータのCAN ID。工場出荷時は0x01のことが多いが、達妙デバッグツールで
// 実際の値を確認すること。
#define DAMIAO_MOTOR_ID_1 0x01 // TODO: 要実機確認

// MasterID(本機のCAN ID)。0以外を推奨(公式ライブラリのコメントより)。
#define DAMIAO_MASTER_CAN_ID 0x10

// フィードバックのデコードに使うレンジ(位置±P_MAX[rad], 速度±V_MAX[rad/s],
// トルク±T_MAX[N・m])。DM4310クラスの一般的な値を暫定的に入れてあるが、
// DM2325固有の正確な値ではない。
//
// 重要: これはVEL/POS_VELモードでの「制御」には影響しない(生のfloat値を
// そのまま送るため)。影響するのはフィードバック(damiao/motor1/state)の
// 数値の正しさのみ。実機入手後、達妙デバッグツールでPMAX/VMAX/TMAXの
// 実際の値を確認し、ここを書き換えること。
#define DAMIAO_P_MAX 12.5f // TODO: 要実機確認(DM2325の実際値に置き換える)
#define DAMIAO_V_MAX 30.0f // TODO: 要実機確認
#define DAMIAO_T_MAX 10.0f // TODO: 要実機確認(DM2325は5N・m級との情報あり、要裏取り)
