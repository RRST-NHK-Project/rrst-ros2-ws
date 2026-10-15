/*====================================================================
<can_task.cpp>
・CAN通信（TWAI + MCP2561）を用いた制御データ転送

シリアル通信の 24 スロットを CAN バス上の最大 4 ノードへ分配し、各ノードが自分の担当スロットだけを受信・送信する。
ノードモードでは、自分の担当スロットを GPIO / サーボへ反映し、エンコーダ / スイッチの状態をそのスロットへ戻す。
ホストモードでは、シリアルで受け取った 24 スロットを各ノードへ配り、各ノードから返ってきたデータをまとめてシリアルへ返す。
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "config.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include "status_led.hpp"
#include <Arduino.h>
#include <cstring>
#include <driver/twai.h>

// 指令(host->node)と帰還(node->host)は必ず別々のID帯を使うこと。
// 以前は両方向とも同じ base+node*16+chunk を使っており、ホストの指令送信と
// ノード自身の帰還送信が同一IDで衝突し、ホストが送信を始めた途端に
// bus_error(エラーカウンタ)が秒間数千件規模で急増する不具合の原因になっていた
// (STM32側は継続的にACKエラー、ESP側もbus_errorが際限なく増加し続けて確認された)。
constexpr uint32_t CAN_FRAME_ID_CMD_BASE = 0x100; // host -> node (指令)
constexpr uint32_t CAN_FRAME_ID_FB_BASE = 0x180;  // node -> host (帰還)
constexpr uint8_t CAN_FRAME_DLC = 8;
constexpr uint8_t CAN_VALUES_PER_FRAME = 4;
constexpr uint8_t CAN_CHUNK_COUNT_PER_NODE = (CAN_SLOTS_PER_NODE + CAN_VALUES_PER_FRAME - 1) / CAN_VALUES_PER_FRAME;
constexpr uint32_t CAN_TX_PERIOD_MS = 5;

static twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
// MODE_ROBOMAS/MODE_CUBEMARSと同一バスに同居できるよう1Mbpsに統一。
// (以前は500kbps固定だった。ロボマス/CubeMarsは仕様上1Mbps固定で変更不可なため、
// 混在させる場合はこちら側を合わせる必要がある。単独バスで使う場合も1Mbpsで動作する)
static twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static portMUX_TYPE g_can_frame_lock = portMUX_INITIALIZER_UNLOCKED;

#if defined(MODE_CAN_MONITOR)
constexpr uint32_t CAN_MONITOR_SUMMARY_PRINT_INTERVAL_MS = 50; // ノードごとの要約出力スロットル
static int16_t g_monitor_node_slots[CAN_MONITOR_MAX_NODES * CAN_SLOTS_PER_NODE] = {0};
static uint8_t g_monitor_node_chunk_mask[CAN_MONITOR_MAX_NODES] = {0};
static uint32_t g_monitor_node_last_print_ms[CAN_MONITOR_MAX_NODES] = {0};
#endif

static uint8_t canFrameNodeIndex(uint32_t id_base, uint32_t identifier);
static uint8_t canFrameChunkIndex(uint32_t id_base, uint32_t identifier);
static uint8_t canChunkValueCount(uint8_t chunk);

static void copyRx16DataSnapshot(int16_t *buffer) {
    portENTER_CRITICAL(&g_can_frame_lock);
    std::memcpy(buffer, (const void *)Rx_16Data, sizeof(int16_t) * Rx16NUM);
    portEXIT_CRITICAL(&g_can_frame_lock);
}

static void publishCanFeedbackToTxBuffer(const int16_t *buffer) {
    portENTER_CRITICAL(&g_can_frame_lock);
    std::memcpy((void *)Tx_16Data, buffer, sizeof(int16_t) * Tx16NUM);
    portEXIT_CRITICAL(&g_can_frame_lock);
}

#if defined(MODE_CAN_MONITOR)

#if CAN_MONITOR_RAW_ENABLE
// バス上の全フレームをそのまま出力する(node/chunk構成を一切仮定しない汎用スニファ)。
// スロットル無し: MODE_CAN_MONITORは他タスクを起動しないため、Serial出力がブロックしても
// 実害はなく、むしろ全フレームを漏らさず見えることを優先する。
static void printCanRawFrame(const twai_message_t &message) {
    char line[96];
    int len = snprintf(line, sizeof(line), "[CAN RAW] t=%lu id=0x%03lX dlc=%u data=",
                        (unsigned long)millis(), (unsigned long)message.identifier,
                        (unsigned)message.data_length_code);
    for (uint8_t i = 0; i < message.data_length_code && len > 0 && (size_t)len < sizeof(line) - 3; i++) {
        len += snprintf(line + len, sizeof(line) - (size_t)len, "%02X ", message.data[i]);
    }
    Serial.println(line);
}
#endif

#if CAN_MONITOR_SUMMARY_ENABLE
// node/chunk構成が既知の場合向けに、スロット単位でデコードした値も出力する。
// ラベルはプロファイル(SW/ENC/rpm等)に依存しないよう、生のslot番号のみで表示する。
static void printCanMonitorNodeSummary(uint8_t node_index, const int16_t *slot_buffer) {
    char line[220] = {0};
    const uint8_t slot_offset = node_index * CAN_SLOTS_PER_NODE;
    int len = snprintf(line, sizeof(line), "[CAN MON] node=%u", (unsigned)node_index);
    for (uint8_t s = 0; s < CAN_SLOTS_PER_NODE && len > 0 && (size_t)len < sizeof(line); s++) {
        len += snprintf(line + len, sizeof(line) - (size_t)len, " slot%u=%d",
                         (unsigned)s, (int)slot_buffer[slot_offset + s]);
    }
    Serial.println(line);
}

// 受信したフレームをスロットバッファへ反映する。呼ばれるたびに必ず取り込む
// (印字のスロットルとは独立させ、取りこぼしで要約が完成しなくなるのを防ぐ)。
// 要約デコードは帰還(node->host, CAN_FRAME_ID_FB_BASE)側のみを対象とする。
// 指令(host->node)の中身も見たい場合は生フレーム出力([CAN RAW])を参照すること。
static void updateCanMonitorNodeFromFrame(const twai_message_t &message) {
    if (message.identifier < CAN_FRAME_ID_FB_BASE) {
        return;
    }
    const uint8_t frame_node = canFrameNodeIndex(CAN_FRAME_ID_FB_BASE, message.identifier);
    if (frame_node >= CAN_MONITOR_MAX_NODES) {
        return;
    }

    const uint8_t chunk = canFrameChunkIndex(CAN_FRAME_ID_FB_BASE, message.identifier);
    if (chunk >= CAN_CHUNK_COUNT_PER_NODE) {
        return;
    }
    const uint8_t values_to_unpack = canChunkValueCount(chunk);
    const uint8_t slot_offset = frame_node * CAN_SLOTS_PER_NODE + chunk * CAN_VALUES_PER_FRAME;

    for (uint8_t i = 0; i < values_to_unpack; ++i) {
        const uint8_t slot_index = slot_offset + i;
        if (slot_index >= CAN_MONITOR_MAX_NODES * CAN_SLOTS_PER_NODE) {
            break;
        }

        const uint8_t byte_index = i * 2;
        const int16_t hi = (int16_t)((uint8_t)message.data[byte_index] << 8);
        const int16_t lo = (int16_t)((uint8_t)message.data[byte_index + 1]);
        g_monitor_node_slots[slot_index] = (int16_t)(hi | lo);
    }

    g_monitor_node_chunk_mask[frame_node] |= (uint8_t)(1u << chunk);
    if (g_monitor_node_chunk_mask[frame_node] == ((1u << CAN_CHUNK_COUNT_PER_NODE) - 1u)) {
        const uint32_t now_ms = millis();
        if (now_ms - g_monitor_node_last_print_ms[frame_node] >= CAN_MONITOR_SUMMARY_PRINT_INTERVAL_MS) {
            printCanMonitorNodeSummary(frame_node, g_monitor_node_slots);
            g_monitor_node_last_print_ms[frame_node] = now_ms;
        }
        // 完成マスクをリセットし、次の1周期分が揃ってから再度出力する
        // (リセットしないと以後どのチャンクが来ても「完成済み」のまま毎回出力されてしまう)。
        g_monitor_node_chunk_mask[frame_node] = 0;
    }
}
#endif

#endif // MODE_CAN_MONITOR

static uint8_t canFrameNodeIndex(uint32_t id_base, uint32_t identifier) {
    // CANフレームIDからノード番号を取り出す(指令/帰還で異なるid_baseを渡す)
    return (uint8_t)((identifier - id_base) / 16U);
}

static uint8_t canFrameChunkIndex(uint32_t id_base, uint32_t identifier) {
    // CANフレームIDからチャンク番号を取り出す(指令/帰還で異なるid_baseを渡す)
    return (uint8_t)((identifier - id_base) % 16U);
}

static uint8_t canChunkValueCount(uint8_t chunk) {
    const uint8_t remaining_slots = (uint8_t)(CAN_SLOTS_PER_NODE - chunk * CAN_VALUES_PER_FRAME);
    return (remaining_slots < CAN_VALUES_PER_FRAME) ? remaining_slots : CAN_VALUES_PER_FRAME;
}

static void canSendNodeSlotBlock(const int16_t *data, uint8_t node_index, uint32_t id_base) {
    // 指定ノード向け(指令)/指定ノードとしての(帰還)スロットデータを複数フレームに分けて送信する
    for (uint8_t chunk = 0; chunk < CAN_CHUNK_COUNT_PER_NODE; chunk++) {
        twai_message_t message{};
        message.identifier = id_base + (node_index * 16U) + chunk;
        message.data_length_code = CAN_FRAME_DLC;
        message.flags = 0;
        std::memset(message.data, 0, sizeof(message.data));

        const uint8_t values_to_pack = canChunkValueCount(chunk);
        for (uint8_t i = 0; i < values_to_pack; i++) {
            const uint8_t slot_index = node_index * CAN_SLOTS_PER_NODE + chunk * CAN_VALUES_PER_FRAME + i;
            if (slot_index >= Tx16NUM) {
                break;
            }

            const int16_t value = data[slot_index];
            message.data[i * 2] = (uint8_t)(value >> 8);
            message.data[i * 2 + 1] = (uint8_t)(value & 0xFF);
        }

        // CAN未接続/相手ノード無応答時にTXキューが詰まっても、ここでブロックすると
        // 同じcanTask内で行っているホストのローカルエンコーダ値publishまで遅延してしまう
        // (エンコーダ取得が遅延して見える不具合の原因だった)。1周期分は捨てても
        // 次のCAN_TX_PERIOD_MS周期で最新値を送り直すため、非ブロッキングで十分。
        (void)twai_transmit(&message, 0);
    }
}

static void canRecvNodeSlotBlock(int16_t *buffer, uint8_t node_index) {
    // 自ノード宛て(指令, CAN_FRAME_ID_CMD_BASE)のCANフレームだけを受信してローカルバッファへ反映する
    twai_message_t message{};

    while (twai_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK) {
        if (message.data_length_code != CAN_FRAME_DLC) {
            continue;
        }
        if (message.identifier < CAN_FRAME_ID_CMD_BASE) {
            continue;
        }

        const uint8_t frame_node = canFrameNodeIndex(CAN_FRAME_ID_CMD_BASE, message.identifier);
        if (frame_node != node_index) {
            continue;
        }

        const uint8_t chunk = canFrameChunkIndex(CAN_FRAME_ID_CMD_BASE, message.identifier);
        if (chunk >= CAN_CHUNK_COUNT_PER_NODE) {
            continue;
        }

        statusLedPulseCan();

        const uint8_t slot_offset = node_index * CAN_SLOTS_PER_NODE + chunk * CAN_VALUES_PER_FRAME;
        const uint8_t values_to_unpack = canChunkValueCount(chunk);
        for (uint8_t i = 0; i < values_to_unpack; i++) {
            const uint8_t slot_index = slot_offset + i;
            if (slot_index >= Rx16NUM) {
                break;
            }

            const uint8_t byte_index = i * 2;
            const int16_t hi = (int16_t)((uint8_t)message.data[byte_index] << 8);
            const int16_t lo = (int16_t)((uint8_t)message.data[byte_index + 1]);
            buffer[slot_index] = (int16_t)(hi | lo);
        }
    }
}

static void canRecvAllNodeSlotBlocks(int16_t *buffer) {
    // ホスト側で全ノードの帰還(CAN_FRAME_ID_FB_BASE)スロットデータをまとめて受信する
    twai_message_t message{};

    while (twai_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK) {
        if (message.data_length_code != CAN_FRAME_DLC) {
            continue;
        }
        if (message.identifier < CAN_FRAME_ID_FB_BASE) {
            continue;
        }

        const uint8_t frame_node = canFrameNodeIndex(CAN_FRAME_ID_FB_BASE, message.identifier);
        if (frame_node >= CAN_NODE_COUNT) {
            continue;
        }

        const uint8_t chunk = canFrameChunkIndex(CAN_FRAME_ID_FB_BASE, message.identifier);
        if (chunk >= CAN_CHUNK_COUNT_PER_NODE) {
            continue;
        }

        statusLedPulseCan();

        const uint8_t slot_offset = frame_node * CAN_SLOTS_PER_NODE + chunk * CAN_VALUES_PER_FRAME;
        const uint8_t values_to_unpack = canChunkValueCount(chunk);
        for (uint8_t i = 0; i < values_to_unpack; i++) {
            const uint8_t slot_index = slot_offset + i;
            if (slot_index >= Rx16NUM) {
                break;
            }

            const uint8_t byte_index = i * 2;
            const int16_t hi = (int16_t)((uint8_t)message.data[byte_index] << 8);
            const int16_t lo = (int16_t)((uint8_t)message.data[byte_index + 1]);
            buffer[slot_index] = (int16_t)(hi | lo);
        }
    }
}

static void applyNodeSlotBlockToLocalControl(const int16_t *slot_buffer, uint8_t node_index) {
    // 受信したノードデータをローカル制御用配列へ反映する
    // slot_indexはslot_buffer（全ノード分, CAN_NODE_COUNT*CAN_SLOTS_PER_NODE要素）内の絶対位置なので、
    // 書き込み先のCanIoRxData（CAN_IO_SLOT_COUNT要素、基板ごとに異なる）ではなく全体サイズと比較すること
    const uint8_t slot_offset = node_index * CAN_SLOTS_PER_NODE;
    constexpr uint8_t kSlotBufferSize = CAN_NODE_COUNT * CAN_SLOTS_PER_NODE;

    for (uint8_t i = 0; i < CAN_IO_SLOT_COUNT; i++) {
        const uint8_t slot_index = slot_offset + i;
        if (slot_index < kSlotBufferSize) {
            CanIoRxData[i] = slot_buffer[slot_index];
        }
    }
}

static void buildNodeSlotBlockFromLocalFeedback(int16_t *slot_buffer, uint8_t node_index) {
    // ローカルのフィードバック情報をCAN送信用バッファへ組み立てる
    // スロット順序・意味は基板(BOARD_VARIANT)ごとに異なる。frame_data.hppのCanIoTxData
    // コメント参照。
    const uint8_t slot_offset = node_index * CAN_SLOTS_PER_NODE;

    for (uint8_t i = 0; i < CAN_IO_SLOT_COUNT; i++) {
        if (slot_offset + i < Tx16NUM) {
            slot_buffer[slot_offset + i] = CanIoTxData[i];
        }
    }
}

void canInit() {
    twai_stop();
    twai_driver_uninstall();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    Serial.println("[CAN] TWAI started (1Mbit).");
}

#if defined(MODE_CAN_HOST) && CAN_HOST_DIAG_ENABLE
// STM32側の診断ログ(can_task.cpp canTaskPrintDiagnostics)と同じ周期で出し、
// 突き合わせやすくする。ホスト自身が送信を始めた途端に壊れる問題の切り分け用。
static void printHostCanDiagnostics() {
    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) {
        Serial.println("[CAN HOST] twai_get_status_info failed");
        return;
    }
    const char *state_str = "?";
    switch (status.state) {
    case TWAI_STATE_STOPPED: state_str = "STOPPED"; break;
    case TWAI_STATE_RUNNING: state_str = "RUNNING"; break;
    case TWAI_STATE_BUS_OFF: state_str = "BUS_OFF"; break;
    case TWAI_STATE_RECOVERING: state_str = "RECOVERING"; break;
    }
    char line[220];
    snprintf(line, sizeof(line),
             "[CAN HOST] t=%lu state=%s TxErrCnt=%lu RxErrCnt=%lu tx_failed=%lu "
             "rx_missed=%lu rx_overrun=%lu arb_lost=%lu bus_error=%lu msgs_to_tx=%lu msgs_to_rx=%lu",
             (unsigned long)millis(), state_str,
             (unsigned long)status.tx_error_counter, (unsigned long)status.rx_error_counter,
             (unsigned long)status.tx_failed_count, (unsigned long)status.rx_missed_count,
             (unsigned long)status.rx_overrun_count, (unsigned long)status.arb_lost_count,
             (unsigned long)status.bus_error_count,
             (unsigned long)status.msgs_to_tx, (unsigned long)status.msgs_to_rx);
    Serial.println(line);
}
#endif

// CAN未接続/相手ノード無応答が続くとエラーカウンタが上限に達しBus-Offへ遷移するが、
// TWAIドライバはBus-Offから自動復帰しない(twai_initiate_recovery()が必須、かつ
// 復帰後もRUNNINGへは戻らずSTOPPEDで止まるため明示的にtwai_start()が要る)。
// これを放置すると一度Bus-Offに陥ったら再接続してもCAN通信が永久に復旧しない。
static void canRecoverBusIfNeeded() {
    constexpr uint32_t CAN_RECOVERY_CHECK_PERIOD_MS = 100;
    static uint32_t last_check_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_check_ms < CAN_RECOVERY_CHECK_PERIOD_MS) {
        return;
    }
    last_check_ms = now_ms;

    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) {
        return;
    }

    if (status.state == TWAI_STATE_BUS_OFF) {
        twai_initiate_recovery();
    } else if (status.state == TWAI_STATE_STOPPED) {
        twai_start();
    }
}

void canTask(void *) {
    TickType_t last_tx = xTaskGetTickCount();
    // 1周期ごとに送受信を行う
    static int16_t node_slot_buffer[Tx16NUM] = {0};
    static int16_t node_feedback_buffer[Tx16NUM] = {0};
    static int16_t host_tx_payload[Rx16NUM] = {0};
#if defined(MODE_CAN_HOST) && CAN_HOST_DIAG_ENABLE
    constexpr uint32_t CAN_HOST_DIAG_PERIOD_MS = 500; // STM32側の診断ログ周期と合わせる
    uint32_t last_host_diag_ms = millis();
#endif

    while (1) {
        canRecoverBusIfNeeded();
#if defined(MODE_CAN_MONITOR)
        twai_message_t message{};
        if (twai_receive(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
            statusLedPulseCan();
#if CAN_MONITOR_RAW_ENABLE
            printCanRawFrame(message);
#endif
#if CAN_MONITOR_SUMMARY_ENABLE
            updateCanMonitorNodeFromFrame(message);
#endif
        }
#elif defined(MODE_CAN_HOST)
        // node_feedback_bufferは持続的なバッファ。受信したスロットだけを更新し、
        // まだ受信していないスロットは前回値を保持する（毎ループ全消去すると
        // ノード送信周期(5ms)とホストループ周期(1ms)のズレでほぼ常にゼロを送ってしまう）
        canRecvAllNodeSlotBlocks(node_feedback_buffer);
        // ホスト自身もバス上の1ノード（CAN_NODE_INDEX）として振る舞う。
        // CANを介さずローカルのスイッチ/エンコーダ値を直接自分の担当スロットへ書き込む
        buildNodeSlotBlockFromLocalFeedback(node_feedback_buffer, CAN_NODE_INDEX);
        publishCanFeedbackToTxBuffer(node_feedback_buffer);

        if (xTaskGetTickCount() - last_tx >= pdMS_TO_TICKS(CAN_TX_PERIOD_MS)) {
            copyRx16DataSnapshot(host_tx_payload);
            // ホスト自身の担当スロットはCANへ送らず、ローカルのMD/サーボ制御へ直接反映する
            applyNodeSlotBlockToLocalControl(host_tx_payload, CAN_NODE_INDEX);
            for (uint8_t node_index = 0; node_index < CAN_NODE_COUNT; node_index++) {
                if (node_index == CAN_NODE_INDEX) {
                    continue;
                }
                canSendNodeSlotBlock(host_tx_payload, node_index, CAN_FRAME_ID_CMD_BASE);
            }
            last_tx = xTaskGetTickCount();
        }

#if CAN_HOST_DIAG_ENABLE
        if (millis() - last_host_diag_ms >= CAN_HOST_DIAG_PERIOD_MS) {
            last_host_diag_ms = millis();
            printHostCanDiagnostics();
        }
#endif  // CAN_HOST_DIAG_ENABLE
#else
        // 指令(node_slot_buffer)と帰還(node_feedback_buffer)で必ず別のバッファを
        // 使うこと(2026-09-11修正、ユーザー報告「ros2can単体でテスト、101,102,103,104が
        // 接続されている状態でサーボを動かしたが細かく振動する挙動」)。
        // 以前は両方ともnode_slot_bufferを使っており、buildNodeSlotBlockFromLocal
        // Feedbackが自ノードの担当スロット(= 受信した指令が入っているのと同じ
        // スロット範囲)をCanIoTxData(スイッチ状態・エンコーダ値等の帰還値)で
        // 上書きしていた。このループはvTaskDelay(1)の1ms周期でapplyNodeSlotBlockTo
        // LocalControlを呼ぶのに対し、ホストからの指令フレームはCAN_TX_PERIOD_MS
        // (5ms)間隔でしか来ないため、指令が届いた直後の1周期だけ本来の角度、
        // 残りは帰還値がそのままサーボへ出る。結果、サーボが約200Hzで2つの値の間を
        // 往復して細かく振動していた。MODE_CAN_HOST側は元から
        // node_feedback_buffer/host_tx_payloadと分けてあり、この問題は無い
        // (ホスト自身のポンプ/LED出力は正常だがノード配下のサーボだけ振動する、
        // という切り分けと一致する)。
        canRecvNodeSlotBlock(node_slot_buffer, CAN_NODE_INDEX);
        applyNodeSlotBlockToLocalControl(node_slot_buffer, CAN_NODE_INDEX);

        if (xTaskGetTickCount() - last_tx >= pdMS_TO_TICKS(CAN_TX_PERIOD_MS)) {
            buildNodeSlotBlockFromLocalFeedback(node_feedback_buffer, CAN_NODE_INDEX);
            canSendNodeSlotBlock(node_feedback_buffer, CAN_NODE_INDEX, CAN_FRAME_ID_FB_BASE);
            last_tx = xTaskGetTickCount();
        }
#endif

        statusLedUpdate();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
