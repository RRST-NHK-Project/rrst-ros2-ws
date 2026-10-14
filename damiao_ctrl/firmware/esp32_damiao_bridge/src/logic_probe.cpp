/*====================================================================
<logic_probe.cpp>
・簡易ロジックアナライザの実装

CAN_RXピンの電圧変化(HIGH<->LOW遷移)をGPIO割り込みで検出し、遷移の
タイミング(マイクロ秒単位)をリングバッファに記録する。TWAIドライバは
一切使わない(=CANプロトコルの解釈や前提を挟まない、最も生に近い観測)。

正常なCAN通信であれば、遷移間隔(delta)はビット周期の整数倍に揃うはず
(例: 1Mbpsなら最小1us刻み、崩れたstuffビット等を考慮しても概ね規則的)。
delta がバラバラ・極端に短い/長い場合はノイズや電源変動である可能性が
高く、規則的な場合はCAN信号(または何らかのデジタル通信)である可能性が
高い、という切り分けに使う。
====================================================================*/

#include "logic_probe.hpp"
#include "config.hpp"
#include <Arduino.h>

namespace {

constexpr int BUF_SIZE = 4000;
volatile uint32_t g_timestamps_us[BUF_SIZE];
volatile uint8_t g_levels[BUF_SIZE];
volatile int g_count = 0;

void IRAM_ATTR onEdge() {
    if (g_count >= BUF_SIZE) {
        return;
    }
    g_timestamps_us[g_count] = micros();
    g_levels[g_count] = (uint8_t)digitalRead(CAN_RX);
    g_count++;
}

} // namespace

void logicProbeTask(void *) {
    delay(500);
    Serial.println("=================================================");
    Serial.println("[probe] 簡易ロジックアナライザ (TWAI未使用)");
    Serial.printf("[probe] CAN_RX(GPIO%d)の電圧遷移を記録します\n", CAN_RX);
    Serial.println("=================================================");

    while (1) {
        pinMode(CAN_RX, INPUT);
        g_count = 0;
        const int initial_level = digitalRead(CAN_RX);

        Serial.printf("\n[probe] 記録開始 (3秒間, 初期レベル=%s)...\n",
                      initial_level == HIGH ? "HIGH" : "LOW");

        attachInterrupt(digitalPinToInterrupt(CAN_RX), onEdge, CHANGE);
        delay(3000);
        detachInterrupt(digitalPinToInterrupt(CAN_RX));

        const int n = g_count;
        Serial.printf("[probe] 記録終了: %d 回の電圧遷移を検出\n", n);

        if (n == 0) {
            Serial.println("[probe] -> 3秒間、電圧変化が一切ありませんでした。");
            Serial.println("[probe]    バスは完全に静止(DC的に一定)しているようです。");
        } else {
            // delta時間の統計(最小/最大/平均)と、最初の50件のdeltaを表示。
            uint32_t min_delta = 0xFFFFFFFF, max_delta = 0;
            uint64_t sum_delta = 0;
            uint32_t prev_t = g_timestamps_us[0];
            for (int i = 1; i < n; i++) {
                uint32_t delta = g_timestamps_us[i] - prev_t;
                prev_t = g_timestamps_us[i];
                if (delta < min_delta) min_delta = delta;
                if (delta > max_delta) max_delta = delta;
                sum_delta += delta;
            }
            if (n > 1) {
                Serial.printf(
                    "[probe] delta統計(us): min=%lu max=%lu avg=%.1f (n=%d)\n",
                    (unsigned long)min_delta, (unsigned long)max_delta,
                    (double)sum_delta / (n - 1), n - 1);
            }

            Serial.println("[probe] 先頭50件の遷移 (経過us, レベル):");
            uint32_t t0 = g_timestamps_us[0];
            for (int i = 0; i < n && i < 50; i++) {
                Serial.printf(
                    "  t=%8lu us  level=%s\n", (unsigned long)(g_timestamps_us[i] - t0),
                    g_levels[i] == HIGH ? "HIGH" : "LOW");
            }

            if (n >= BUF_SIZE) {
                Serial.println(
                    "[probe] -> バッファが満杯になりました(遷移が速すぎる/頻繁すぎる)。"
                    "本物のノイズか、高速なビット列の可能性があります。");
            }
        }

        Serial.println("[probe] 5秒後に再記録します...");
        delay(5000);
    }
}
