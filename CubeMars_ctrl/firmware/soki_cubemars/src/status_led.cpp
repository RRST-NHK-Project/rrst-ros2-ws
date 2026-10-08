/*====================================================================
<status_led.cpp>
・通信アクティビティ表示用LEDの共通ヘルパー

CANは複数ノード分のフレームが数msおきに届くため、「受信のたびに点灯して
一定時間保持する」方式では次のフレームが保持時間内に届き続けてしまい、
実質的に常時点灯（消灯する暇がない）になってしまう。
そのため「直近に通信があった間は固定レートで点滅し、一定時間通信が
途絶えたら消灯する」方式にしている。通信量に関わらず点滅が視認でき、
通信が止まればすぐ消灯するので状態が分かりやすい。

CANとシリアルの活動は別々に記録する。ホストモードでノードマイコンが
未接続だとCAN側は活動なしのままになるが、PCとのシリアル通信自体は
生きていることが多く、その場合でも「シリアルは繋がっている」ことを
LEDだけで見分けたい。そこでCAN活動があれば従来通りの高速点滅、
CANは途絶えていてシリアルだけ活動していれば短いパルス点滅、という
異なるパターンで表示する。
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "status_led.hpp"
#include "config.hpp"
#include "defs.hpp"
#include <Arduino.h>

namespace {
constexpr uint32_t LED_IDLE_TIMEOUT_MS = 300; // この時間フレームが来なければそのソースは非活動とみなす

constexpr uint32_t CAN_BLINK_INTERVAL_MS = 100; // CAN活動あり時の点滅周期(トグル間隔)

// シリアルのみ活動時は「短いパルスを一定周期で光らせる」パターンにし、
// CANの高速点滅と見た目で区別できるようにする。
constexpr uint32_t SERIAL_PULSE_ON_MS = 50;
constexpr uint32_t SERIAL_PULSE_PERIOD_MS = 500;

volatile uint32_t g_last_can_activity_ms = 0;
volatile bool g_can_active = false;
volatile uint32_t g_last_serial_activity_ms = 0;
volatile bool g_serial_active = false;

volatile uint32_t g_last_toggle_ms = 0;
volatile bool g_led_on = false;

void setLed(bool on) {
    if (on != g_led_on) {
        digitalWrite(LED, on ? HIGH : LOW);
        g_led_on = on;
    }
}
} // namespace

void statusLedPulseCan() {
    if (!ENABLE_LED) {
        return;
    }

    g_last_can_activity_ms = millis();
    g_can_active = true;
}

void statusLedPulseSerial() {
    if (!ENABLE_LED) {
        return;
    }

    g_last_serial_activity_ms = millis();
    g_serial_active = true;
}

void statusLedUpdate() {
    if (!ENABLE_LED) {
        return;
    }

    const uint32_t now = millis();

    if (g_can_active && now - g_last_can_activity_ms >= LED_IDLE_TIMEOUT_MS) {
        g_can_active = false;
    }
    if (g_serial_active && now - g_last_serial_activity_ms >= LED_IDLE_TIMEOUT_MS) {
        g_serial_active = false;
    }

    if (g_can_active) {
        // CAN活動あり: 従来通りの固定レート点滅
        if (now - g_last_toggle_ms >= CAN_BLINK_INTERVAL_MS) {
            setLed(!g_led_on);
            g_last_toggle_ms = now;
        }
        return;
    }

    if (g_serial_active) {
        // シリアルのみ活動中: 短いパルスを周期的に光らせる(CANの点滅とは異なる見た目にする)
        const uint32_t phase = now % SERIAL_PULSE_PERIOD_MS;
        setLed(phase < SERIAL_PULSE_ON_MS);
        return;
    }

    // どちらの通信も途絶えている間は消灯を維持する
    setLed(false);
}
