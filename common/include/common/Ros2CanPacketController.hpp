// マイコン(ros2can CANホスト)へ送受信する配列tx_/rx_へ安全にアクセスするための
// ライブラリ。serial_bridge向け PacketController.hpp の後継。
//
// ros2can (xiao_esp32_s3_smd_serial_bridge, MODE_CAN_HOST) は serial_bridge と
// ワイヤ互換の24 x int16フレームを使うが、その24スロットをCANバス上の最大
// NODE_COUNT台の子ノードへ SLOTS_PER_NODE ずつ分配する点が serial_bridge と異なる。
// [ros2can/ros2can/device_profiles.py の make_can_host_profile() (既定プロファイル)
//  と一致させること。firmware/xiao-esp32-s3_can2io/src/config.hpp の
//  CAN_NODE_COUNT / CAN_SLOTS_PER_NODE を変更した場合はここも合わせて変更する]
//
// ノードごとの担当スロット (SLOTS_PER_NODE = 5、実機はDCモータ非搭載):
//   指令 (TX, ROS -> ホスト -> CAN -> ノード): SERVO1, SERVO2, SERVO3, 予備, 予備
//   帰還 (RX, ノード -> CAN -> ホスト -> ROS): SW1, SW2, SW3, ENC1, ENC2
// SERVOn と SWn はピン共有 (ファームウェア config.hpp の MULTIn で切替,
// 0=スイッチ入力/1=サーボ出力)。
//
// グローバルスロット index = node(0-origin) * SLOTS_PER_NODE + local_index
// ノードの CAN_ID は 101,102,103,104 (canId() 参照)。

// ===== 使用例コメント =====
/*
// 変数宣言 (Ros2CanPacketController のインスタンス)
Ros2CanPacketController ctrlPkt;

// ノード1(CAN_ID=101)のSERVO2を90度に設定
ctrlPkt.setServo(0, 2, 90);

// 送信用配列を取得してpublish
auto sendArray = ctrlPkt.toVector();

// 受信したInt16MultiArrayでrx_を更新
ctrlPkt.updateRx(msg->data);

// ノード1のSW1状態、ENC1カウンタを取得
bool sw1 = ctrlPkt.getSW(0, 1);
int16_t enc1 = ctrlPkt.getEnc(0, 1);
*/

#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

class Ros2CanPacketController {
public:
    // ファームウェア既定値 (config.hpp の CAN_NODE_COUNT / CAN_SLOTS_PER_NODE)
    static constexpr int NODE_COUNT = 4;
    static constexpr int SLOTS_PER_NODE = 5;
    static constexpr int DATA_SIZE = 24; // serial_bridge互換フレームのスロット数(固定)

    // ノード内ローカルスロット (TX/RXで意味が異なる。上記コメント参照)
    enum LocalSlot : int {
        SLOT_SERVO1 = 0,
        SLOT_SERVO2,
        SLOT_SERVO3,
        SLOT_RESERVED1,
        SLOT_RESERVED2,
    };

    // 送信配列本体 (ROS -> ホスト -> CAN -> 各ノード)
    std::array<int16_t, DATA_SIZE> tx_{};
    // 受信配列本体 (各ノード -> CAN -> ホスト -> ROS)
    std::array<int16_t, DATA_SIZE> rx_{};

    // 指定ノードのSERVOn(1~3)を角度[deg]で設定 (範囲外は無視、範囲外の値はクランプ)
    void setServo(int node, int servo_no, int deg) {
        if (node < 0 || node >= NODE_COUNT)
            return;
        if (servo_no < 1 || servo_no > 3)
            return;
        tx_[slotIndex(node, SLOT_SERVO1 + (servo_no - 1))] = std::clamp(deg, 0, 270);
    }

    // 指定ノードのSWn(1~3)状態を取得 (SERVOnとピン共有、MULTIn=0のときのみ有効)
    bool getSW(int node, int sw_no) const {
        if (node < 0 || node >= NODE_COUNT)
            return false;
        if (sw_no < 1 || sw_no > 3)
            return false;
        return rx_[slotIndex(node, SLOT_SERVO1 + (sw_no - 1))] != 0;
    }

    // 指定ノードのENCn(1~2)カウンタ値を取得
    int16_t getEnc(int node, int enc_no) const {
        if (node < 0 || node >= NODE_COUNT)
            return 0;
        if (enc_no < 1 || enc_no > 2)
            return 0;
        return rx_[slotIndex(node, SLOT_RESERVED1 + (enc_no - 1))];
    }

    // node(0-origin)に対応するCAN_ID (101,102,103,104) を返す
    static int canId(int node) { return 101 + node; }

    // 受信したInt16MultiArray相当のdataでrx_を更新
    void updateRx(const std::vector<int16_t> &data) {
        for (size_t i = 0; i < DATA_SIZE && i < data.size(); ++i) {
            rx_[i] = data[i];
        }
    }

    // 送信用配列を vector で取得
    std::vector<int16_t> toVector() const {
        return std::vector<int16_t>(tx_.begin(), tx_.end());
    }

    // 直接アクセスも残す (TX配列)
    int16_t &operator[](int index) { return tx_[index]; }
    const int16_t &operator[](int index) const { return tx_[index]; }

private:
    static int slotIndex(int node, int local) {
        return node * SLOTS_PER_NODE + local;
    }
};
