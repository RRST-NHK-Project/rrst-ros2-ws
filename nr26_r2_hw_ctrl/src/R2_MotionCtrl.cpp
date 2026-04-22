/*
R2機構制御
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

// ROS
#include "include/PacketController.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
PacketController pkt;

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 7 // 宛先マイコンのID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

std::atomic<int16_t> g_micro1_sw{0}; // マイクロスイッチ(始発): 1=押されている
std::atomic<int16_t> g_micro2_sw{0}; // マイクロスイッチ(終点): 1=押されている
std::atomic<int16_t> g_enc1_val{0};  // エンコーダ1: data[1]から受信

// フォークリフト座標管理 (EncoderCoordinator)
// エンコーダ減少 -> 座標増加 / エンコーダ増加 -> 座標減少
std::atomic<int32_t> g_rotation_count{0};     // エンコーダの回転数(巻回り数)
std::atomic<int64_t> g_zero_offset{0};        // 下端リセット時の絶対エンコーダ値
std::atomic<int16_t> g_last_enc1_val{0};      // 前回のエンコーダ生値
std::atomic<bool> g_coord_initialized{false}; // 初期化フラグ
std::atomic<int64_t> g_abs_coord{0};          // 最終的な座標(下端=0方向=プラス)

class HardWareControl : public rclcpp::Node {
public:
    HardWareControl(uint8_t device_id)
        : Node("hardware_control_" + std::to_string(device_id)),
          device_id_(device_id) {

        // SERVO3初期値
        pkt.setServo(SERVO3, 150);
        /*
        マイコンに送信される配列"data_"
        debug: 機能未割り当て, MD: モータードライバー, TR: トランジスタ
        | data[n] | 詳細 | 範囲 |
        | ---- | ---- | ---- |
        | data[0] | debug | 0 or 1 |
        | data[1] | MD1 | -100 ~ 100 |
        | data[2] | MD2 | -100 ~ 100 |
        | data[3] | MD3 | -100 ~ 100 |
        | data[4] | MD4 | -100 ~ 100 |
        | data[5] | MD5 | -100 ~ 100 |
        | data[6] | MD6 | -100 ~ 100 |
        | data[7] | MD7 | -100 ~ 100 |
        | data[8] | MD8 | -100 ~ 100 |
        | data[9] | Servo1 | 0 ~ 270 |
        | data[10] | Servo2 | 0 ~ 270 |
        | data[11] | Servo3 | 0 ~ 270 |
        | data[12] | Servo4 | 0 ~ 270 |
        | data[13] | Servo5 | 0 ~ 270 |
        | data[14] | Servo6 | 0 ~ 270 |
        | data[15] | Servo7 | 0 ~ 270 |
        | data[16] | Servo8 | 0 ~ 270 |
        | data[17] | TR1 | 0 or 1|
        | data[18] | TR2 | 0 or 1|
        | data[19] | TR3 | 0 or 1|
        | data[20] | TR4 | 0 or 1|
        | data[21] | TR5 | 0 or 1|
        | data[22] | TR6 | 0 or 1|
        | data[23] | TR7 | 0 or 1|
        | data[24] | TR8 | 0 or 1|
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // serial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(device_id_),
            10,
            std::bind(&HardWareControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        // GUIから直接状態が送られる場合の互換経路
        state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "r2/task_state", 10,
            std::bind(&HardWareControl::task_state_callback, this, std::placeholders::_1));

        // r2_plannerが管理する現在状態（[state, color, cell, mode]）
        auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "r2/task_status", status_qos,
            std::bind(&HardWareControl::task_status_callback, this, std::placeholders::_1));

        status_text_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/r2/task_status_text", status_qos,
            std::bind(&HardWareControl::task_status_text_callback, this, std::placeholders::_1));

        camera_servo_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/r2/camera_servo_angle", 10,
            std::bind(&HardWareControl::camera_servo_callback, this, std::placeholders::_1));

        // GUIから直接モードが送られる場合の直接受信（rosbridge互換: best_effort）
        transition_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "r2/task_transition_mode", rclcpp::QoS(10).best_effort(),
            std::bind(&HardWareControl::transition_mode_callback, this, std::placeholders::_1));

        // task_manager_node 経由のドライブモードコマンド（r2_auto と同じ経路、transient_local）
        drive_mode_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "r2_drive_mode_cmd", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&HardWareControl::drive_mode_cmd_callback, this, std::placeholders::_1));

        // 初期状態を適用
        apply_state_to_packet(current_planner_state_);

        // RCLCPP_INFO(get_logger(),
        //             "serial_tx_%d started.", device_id_);
    }

private:
    // GUIの task_status[3] に合わせる運転モード
    static constexpr int32_t DRIVE_MODE_MANUAL = 0;
    static constexpr int32_t DRIVE_MODE_AUTO = 1;

    // =====================================================================
    // 状態定義
    // =====================================================================
    static constexpr int32_t STATE_HEAD_HAND_INIT = 0;
    static constexpr int32_t STATE_HEAD_HAND_PICK_UP_SETTING = 1;
    static constexpr int32_t STATE_HEAD_HAND_GATTAI_WAITING = 2;
    static constexpr int32_t STATE_HEAD_HAND_GATTAI_ASSEMBLY = 3;

    static constexpr int32_t STATE_KFS_HAND_INIT = 6;
    static constexpr int32_t STATE_KFS_PICK_WAITING = 7;
    static constexpr int32_t STATE_PICK_UP = 8;
    static constexpr int32_t STATE_PICK_MIDDLE = 9;
    static constexpr int32_t STATE_PICK_DOWN = 10;
    static constexpr int32_t STATE_KFS_HOLD = 11;
    static constexpr int32_t STATE_KFS_MOVE = 12;
    static constexpr int32_t STATE_TTR_SHOOT_MIDDLE = 13;

    enum class TargetHeight {
        UP,
        MIDDLE,
        DOWN
    };

    int32_t current_planner_state_ = -1;
    std::atomic<int32_t> current_drive_mode_{DRIVE_MODE_MANUAL};
    std::string current_state_name_ = "";
    TargetHeight target_height_ = TargetHeight::DOWN;

    // モーター1回転あたりのエンコーダのカウント数
    static constexpr double COUNTS_PER_ROTATION = 355.0;

    // =====================================================================
    // 状態名→状態コード変換
    // =====================================================================
    static int32_t state_code_from_name(const std::string &state_name) {
        // 既存のシーケンス名
        if (state_name == "HEAD_HAND_INIT")
            return STATE_HEAD_HAND_INIT;
        if (state_name == "HEAD_HAND_PICK_UP_SETTING")
            return STATE_HEAD_HAND_PICK_UP_SETTING;
        if (state_name == "HEAD_HAND_GATTAI_WAITING")
            return STATE_HEAD_HAND_GATTAI_WAITING;
        if (state_name == "HEAD_HAND_GATTAI_ASSEMBLY")
            return STATE_HEAD_HAND_GATTAI_ASSEMBLY;
        if (state_name == "KFS_HAND_INIT")
            return STATE_KFS_HAND_INIT;
        if (state_name == "KFS_PICK_WAITING")
            return STATE_KFS_PICK_WAITING;
        if (state_name == "PICK_UP")
            return STATE_PICK_UP;
        if (state_name == "PICK_MIDDLE")
            return STATE_PICK_MIDDLE;
        if (state_name == "PICK_DOWN")
            return STATE_PICK_DOWN;
        if (state_name == "KFS_HOLD")
            return STATE_KFS_HOLD;
        if (state_name == "KFS_MOVE")
            return STATE_KFS_MOVE;
        if (state_name == "TTR_SHOOT_MIDDLE")
            return STATE_TTR_SHOOT_MIDDLE;

        // GUI既定の状態名（r2/task_status_text の state=...）
        if (state_name == "WAITING" || state_name == "Waiting")
            return STATE_HEAD_HAND_INIT;
        if (state_name == "ENTER_MFF" || state_name == "Enter_MFF" || state_name == "Enter MFF")
            return STATE_HEAD_HAND_PICK_UP_SETTING;
        if (state_name == "LEAVE_MFF" || state_name == "Leave_MFF" || state_name == "Leave MFF")
            return STATE_HEAD_HAND_GATTAI_WAITING;
        if (state_name == "STAFF_ASSEMBLY" || state_name == "Staff_Assembly" || state_name == "Staff Assembly")
            return STATE_HEAD_HAND_GATTAI_ASSEMBLY;

        return -1;
    }

    // =====================================================================
    // 状態テキストからパース
    // =====================================================================
    static std::string parse_state_name(const std::string &status_text) {
        const std::string key = "state=";
        const auto pos = status_text.find(key);
        if (pos == std::string::npos) {
            return "";
        }

        const auto begin = pos + key.size();

        // r2_planner の "state=... color=... cell=... mode=..." 形式に対応
        const auto color_pos = status_text.find(" color=", begin);
        if (color_pos != std::string::npos) {
            return status_text.substr(begin, color_pos - begin);
        }

        // フォールバック: 末尾まで、または最初の空白まで
        const auto end = status_text.find(' ', begin);
        if (end == std::string::npos) {
            return status_text.substr(begin);
        }
        return status_text.substr(begin, end - begin);
    }

    // =====================================================================
    // 全体共通: モーター方向制限
    // ・micro1_sw（始発）押下 → 正回転禁止、逆回転のみ許可
    // ・micro2_sw（終点）押下 → 逆回転禁止、正回転のみ許可
    // =====================================================================
    int16_t apply_direction_limit(int16_t raw_speed) {
        int16_t micro1_sw = g_micro1_sw.load();
        int16_t micro2_sw = g_micro2_sw.load();
        // int64_t abs_coord = g_abs_coord.load();
        // double rot_units = static_cast<double>(abs_coord) / COUNTS_PER_ROTATION;

        // 始発SW押下時 → 正回転(speed > 0)禁止
        if (micro1_sw == 1 && raw_speed > 0) {
            return 0;
        }

        // 終点SW押下時 → 逆回転(speed < 0)禁止
        if (micro2_sw == 1 && raw_speed < 0) {
            return 0;
        }

        return raw_speed;
    }

    void reset_hand_outputs() {
        pkt.setServo(SERVO1, 200);
        pkt.setServo(SERVO3, 190);
        pkt.setServo(SERVO4, 0);
        pkt.setMD(MD1, 0);
        pkt.setTR(TR1, false);
    }

    void set_home_values() {
        pkt.setServo(SERVO1, 200);
        pkt.setServo(SERVO3, 190);
        pkt.setServo(SERVO4, 0);
        pkt.setMD(MD1, 0);
        pkt.setTR(TR1, false); // シリンダー縮める
    }

    void set_ready_values() {
        pkt.setServo(SERVO1, 23);
        pkt.setServo(SERVO3, 190);
        pkt.setServo(SERVO4, 0);
        pkt.setMD(MD1, 0);
        pkt.setTR(TR1, true); // シリンダー伸ばす
    }

    void set_pick_values() {
        pkt.setServo(SERVO1, 13);
        pkt.setServo(SERVO3, 84);
        pkt.setServo(SERVO4, 0);
        pkt.setMD(MD1, 255); // ダイアフラムで吸う
        pkt.setTR(TR1, true);
    }

    void set_hold_values() {
        pkt.setServo(SERVO1, 200);
        pkt.setServo(SERVO3, 194);
        pkt.setServo(SERVO4, 0);
        pkt.setMD(MD1, 255);
        pkt.setTR(TR1, false);
    }

    void set_moving_values() {
        pkt.setServo(SERVO1, 200);
        pkt.setServo(SERVO3, 194);
        pkt.setServo(SERVO4, 0);
        pkt.setMD(MD1, 0);
        pkt.setTR(TR1, false);
    }

    void set_shoot_values() {
        pkt.setServo(SERVO1, 200);
        pkt.setServo(SERVO3, 194);
        pkt.setServo(SERVO4, 50);
        pkt.setMD(MD1, 0);
        pkt.setTR(TR1, false);
    }

    static const char *target_height_to_string(TargetHeight h) {
        switch (h) {
        case TargetHeight::UP:
            return "UP";
        case TargetHeight::MIDDLE:
            return "MIDDLE";
        case TargetHeight::DOWN:
        default:
            return "DOWN";
        }
    }

    void log_current_operation_state(const char *source, bool throttled = false) {
        const int16_t micro1_sw = g_micro1_sw.load();
        const int16_t micro2_sw = g_micro2_sw.load();
        const int64_t abs_coord = g_abs_coord.load();
        const double rot_units = static_cast<double>(abs_coord) / COUNTS_PER_ROTATION;

        const std::string state_name = current_state_name_.empty()
                                           ? std::string("STATE_") + std::to_string(current_planner_state_)
                                           : current_state_name_;
        const bool is_manual = is_manual_mode();
        const char *mode_str = is_manual ? "MANUAL" : "AUTO";

        if (throttled) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "[MotionCtrlState:%s] mode=%s, state=%s(%ld), height=%s, rot=%.3f, sw_up=%d, sw_down=%d, MD1=%d, MD2=%d, SERVO1=%d, SERVO2=%d, TR1=%d, TR2=%d",
                source,
                mode_str,
                state_name.c_str(),
                static_cast<long>(current_planner_state_),
                target_height_to_string(target_height_),
                rot_units,
                static_cast<int>(micro1_sw),
                static_cast<int>(micro2_sw),
                static_cast<int>(pkt[MD1]),
                static_cast<int>(pkt[MD2]),
                static_cast<int>(pkt[SERVO1]),
                static_cast<int>(pkt[SERVO2]),
                static_cast<int>(pkt[TR1]),
                static_cast<int>(pkt[TR2]));
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "[MotionCtrlState:%s] mode=%s, state=%s(%ld), height=%s, rot=%.3f, sw_up=%d, sw_down=%d, MD1=%d, MD2=%d, SERVO1=%d, SERVO2=%d, TR1=%d, TR2=%d",
            source,
            mode_str,
            state_name.c_str(),
            static_cast<long>(current_planner_state_),
            target_height_to_string(target_height_),
            rot_units,
            static_cast<int>(micro1_sw),
            static_cast<int>(micro2_sw),
            static_cast<int>(pkt[MD1]),
            static_cast<int>(pkt[MD2]),
            static_cast<int>(pkt[SERVO1]),
            static_cast<int>(pkt[SERVO2]),
            static_cast<int>(pkt[TR1]),
            static_cast<int>(pkt[TR2]));
    }

    bool is_manual_mode() const {
        return current_drive_mode_.load() == DRIVE_MODE_MANUAL;
    }

    void update_drive_mode(int32_t next_mode, const char *source) {
        const int32_t prev_mode = current_drive_mode_.load();
        if (prev_mode == next_mode) {
            return;
        }

        current_drive_mode_.store(next_mode);
        const bool manual = (next_mode == DRIVE_MODE_MANUAL);
        RCLCPP_INFO(
            get_logger(),
            "drive_mode(%s) -> %s(%ld)",
            source,
            manual ? "MANUAL" : "AUTO",
            static_cast<long>(next_mode));

        if (!manual) {
            apply_state_to_packet(current_planner_state_);
            log_current_operation_state("drive_mode_switch");
        }
    }

    // =====================================================================
    // 状態別モーター制御（タイマーコールバック毎周期呼び出し）
    // =====================================================================
    void apply_state_to_packet(int32_t state_code) {
        int16_t micro1_sw = g_micro1_sw.load();
        int16_t micro2_sw = g_micro2_sw.load();
        int64_t abs_coord = g_abs_coord.load();
        double rot_units = static_cast<double>(abs_coord) / COUNTS_PER_ROTATION;

        // 状態ごとに明示的に値を作り直し、残留値を防ぐ
        pkt.setMD(MD1, 0);
        reset_hand_outputs();

        switch (state_code) {
        case STATE_HEAD_HAND_INIT:
            // モーターを速度30で回し続ける（正回転＝始発方向へ）
            // マイクロスイッチ（始発）が押されたとき → モーター停止
            if (micro1_sw == 1) {
                pkt.setMD(MD2, 0);
            } else {
                pkt.setMD(MD2, apply_direction_limit(130));
            }
            break;

        case STATE_HEAD_HAND_PICK_UP_SETTING:
            // モーターを逆回転
            // micro2_sw が押されたとき → モーター即停止
            if (micro2_sw == 1) {
                pkt.setMD(MD2, 0);
            } else if (rot_units >= 7.0) {
                // 7.0以上は少し減速させる（例：-100など）
                pkt.setMD(MD2, apply_direction_limit(-100)); // 減速
            } else {
                pkt.setMD(MD2, apply_direction_limit(-150));
            }
            break;

        case STATE_HEAD_HAND_GATTAI_WAITING:
            // モーターを速度30で回し続ける（正回転＝始発方向へ）
            // マイクロスイッチ（始発）が押されたとき → モーター停止
            if (micro1_sw == 1) {
                pkt.setMD(MD2, 0);
            } else {
                pkt.setMD(MD2, apply_direction_limit(130));
            }
            break;

        case STATE_HEAD_HAND_GATTAI_ASSEMBLY:
            // 回転数が0でない限り、回転数を0にするべくモーターを速度10で回転
            // （始発SWによるリセットも含む）
            // 回転数が0になったとき → モーター停止
            // この監視状態は終了指示か別状態への移行まで維持
            if (micro1_sw == 1) {
                pkt.setMD(MD2, 0);
            } else if (std::abs(rot_units) < 0.05) {
                pkt.setMD(MD2, 0);
            } else {
                pkt.setMD(MD2, apply_direction_limit(10));
            }
            break;

        case STATE_KFS_HAND_INIT:
            set_home_values();
            break;
        case STATE_KFS_PICK_WAITING:
            set_ready_values();
            break;
        case STATE_PICK_UP:
            target_height_ = TargetHeight::UP;
            set_ready_values();
            set_pick_values();
            break;
        case STATE_PICK_MIDDLE:
            target_height_ = TargetHeight::MIDDLE;
            set_ready_values();
            set_pick_values();
            break;
        case STATE_PICK_DOWN:
            target_height_ = TargetHeight::DOWN;
            set_ready_values();
            set_pick_values();
            break;
        case STATE_KFS_HOLD:
            set_hold_values();
            break;
        case STATE_KFS_MOVE:
            set_moving_values();
            break;
        case STATE_TTR_SHOOT_MIDDLE:
            set_shoot_values();
            break;

        default:
            // 未初期化(-1)または未知状態では何もしない（手動操作を優先）
            break;
        }

        // 現在の動作状態を定期表示
        log_current_operation_state("apply", true);
    }

    // =====================================================================
    // 状態遷移コールバック群
    // =====================================================================
    void task_state_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // 状態制御は task_status_text の state 名を正とする
        RCLCPP_DEBUG(get_logger(), "task_state ignored (state-by-name mode): %ld", static_cast<long>(msg->data));
    }

    void transition_mode_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // GUIから直接モードを受信（task_manager_node経由より確実）
        update_drive_mode(msg->data, "transition_mode_direct");
    }

    void drive_mode_cmd_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // task_manager_node 経由のモードコマンド（r2_drive_mode_cmd[0] がモードコード）
        if (msg->data.empty())
            return;
        update_drive_mode(msg->data[0], "drive_mode_cmd");
    }

    void task_status_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // task_status からのモード更新は行わない。
        // モードは r2/task_transition_mode の直接受信（transition_mode_callback）でのみ管理する。
        // task_manager_node が古い mode=MANUAL を周期送信し続けると上書きされるため。
        (void)msg;
    }

    void camera_servo_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        pkt.setServo(SERVO2, msg->data);
    }

    void task_status_text_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string next_state_name = parse_state_name(msg->data);
        if (next_state_name.empty() || next_state_name == current_state_name_) {
            return;
        }

        current_state_name_ = next_state_name;
        const int32_t mapped_state = state_code_from_name(current_state_name_);
        if (mapped_state >= 0) {
            current_planner_state_ = mapped_state;
            if (!is_manual_mode()) {
                apply_state_to_packet(current_planner_state_);
                log_current_operation_state("task_status_text");
            }
        }

        RCLCPP_INFO(get_logger(), "task_status_text.state -> %s", current_state_name_.c_str());
    }

    // =====================================================================
    // PS4コントローラーコールバック（手動制御用）
    // =====================================================================
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (!is_manual_mode()) {
            return;
        }

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // static bool last_option = false;
        // static bool option_latch = false;

        // static bool last_share = false;
        // static bool share_latch = false;

        // 以降、配列data_を操作する

        // =================================================================
        // L1,R1:「フォークリフト上下」
        // 絶対座標に基づく減速 + マイクロスイッチによる方向制限
        // =================================================================

        int16_t micro1_sw = g_micro1_sw.load();
        int16_t micro2_sw = g_micro2_sw.load();

        static const int NORMAL_SPEED = 130;
        static const int SLOW_SPEED = 100;

        int64_t abs_coord = g_abs_coord.load();
        double rot_units = static_cast<double>(abs_coord) / COUNTS_PER_ROTATION;

        // ヒステリシス（遊び）を持たせた減速ゾーン判定（チャタリング防止用）
        static bool is_fork_slow = false;
        if (rot_units >= 7.0) {
            is_fork_slow = true;
        } else if (rot_units <= 6.8) {
            is_fork_slow = false;
        }
        bool in_slow_zone = is_fork_slow;

        int fwd_speed = in_slow_zone ? SLOW_SPEED : NORMAL_SPEED;
        int rev_speed = in_slow_zone ? -SLOW_SPEED : -NORMAL_SPEED;

        // micro1_sw が始発(正回転禁止) = fwd_speed禁止 -> rev_speed(R1)のみ許可
        // micro2_sw が終点(逆回転禁止) = rev_speed禁止 -> fwd_speed(L1)のみ許可
        if (micro1_sw == 1) {
            if (L1 == 1) {
                pkt.setMD(MD2, fwd_speed);
            } else {
                pkt.setMD(MD2, 0);
            }
        } else if (micro2_sw == 1) {
            if (R1 == 1) {
                pkt.setMD(MD2, rev_speed);
            } else {
                pkt.setMD(MD2, 0);
            }
        } else {
            if (L1 == 1) {
                pkt.setMD(MD2, fwd_speed);
            } else if (R1 == 1) {
                pkt.setMD(MD2, rev_speed);
            } else {
                pkt.setMD(MD2, 0);
            }
        }

        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        //                      "【フォーク制御】回転数=%.2f, 減速=%s, 始発SW=%d, 終点SW=%d, 出力=%d",
        //                      rot_units, in_slow_zone ? "ON" : "OFF",
        //                      micro2_sw, micro1_sw, data_[2]);

        // デバッグ用
        // RCLCPP_INFO(
        //     get_logger(),
        //     "data_[1-4]=[%d,%d,%d,%d], data_[9-12]=[%d,%d,%d,%d]",
        //     data_[1], data_[2], data_[3], data_[4],
        //     data_[9], data_[10], data_[11], data_[12]);

        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        // ★★★ 状態別のモーター制御を毎周期適用 ★★★
        if (!is_manual_mode()) {
            apply_state_to_packet(current_planner_state_);
        }

        // ★★★ マイクロスイッチの安全停止を最優先で適用 ★★★
        int16_t micro1_sw = g_micro1_sw.load();
        int16_t micro2_sw = g_micro2_sw.load();

        if (micro1_sw == 1 && pkt[MD2] > 0) {
            pkt.setMD(MD2, 0);
            // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "【安全装置】始発リミット到達！モーターの正回転を即時遮断しました！");
        }

        if (micro2_sw == 1 && pkt[MD2] < 0) {
            pkt.setMD(MD2, 0);
            // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "【安全装置】終点リミット到達！モーターの逆回転を即時遮断しました！");
        }

        msg.data = pkt.toVector();
        publisher_->publish(msg);
        log_current_operation_state("publish", true);
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() < RX16NUM) {
            // RCLCPP_WARN(this->get_logger(),
            //             "serial_rx_%d: data too short (%zu)",
            //             device_id_, msg->data.size());
            return;
        }

        // マイクロスイッチの値を更新
        g_micro1_sw = msg->data[10]; // 上端スイッチ(micro1)
        g_micro2_sw = msg->data[9];  // 下端スイッチ(micro2)
        g_enc1_val = msg->data[1];   // エンコーダ1

        // =============================================================
        // 座標調査・ラップアラウンド計算
        // =============================================================
        int16_t current_enc1 = msg->data[1];

        if (!g_coord_initialized.load()) {
            g_last_enc1_val.store(current_enc1);
            g_coord_initialized.store(true);
        }

        const int HALF_ENCODER = 16384;
        const int64_t ENCODER_MAX = 32768;

        int diff = (int)current_enc1 - (int)g_last_enc1_val.load();
        int32_t r_count = g_rotation_count.load();

        if (diff > HALF_ENCODER) {
            r_count--;
        } else if (diff < -HALF_ENCODER) {
            r_count++;
        }

        g_rotation_count.store(r_count);
        g_last_enc1_val.store(current_enc1);

        int64_t total_encoder = (int64_t)r_count * ENCODER_MAX + (int64_t)current_enc1;

        // 始発スイッチ押下時に座標をゼロリセット
        if (msg->data[10] != 0) {
            g_zero_offset.store(total_encoder);
            // RCLCPP_INFO(get_logger(), "[COORD RESET!] 始発スイッチ押下により座標0へオフセット設定");
        }

        int64_t zero_offset = g_zero_offset.load();
        int64_t abs_coord = -(total_encoder - zero_offset);
        g_abs_coord.store(abs_coord);

        double rot = (double)abs_coord / COUNTS_PER_ROTATION;

        if (diff != 0) {
            // RCLCPP_INFO(get_logger(),
            //             "\n--- ROTATION DEBUG ---\n"
            //             "  生値の変化 : %d -> %d (diff: %d)\n"
            //             "  デジタルラップ : %d 回\n"
            //             "  絶対カウント   : %ld\n"
            //             "  現在回転数     : %.3f 回転 (1周355)\n"
            //             "----------------------",
            //             (int)current_enc1 - diff, (int)current_enc1, diff,
            //             (int)r_count, abs_coord, rot);
        }

        // --- 通信デバッグ追加 ---
        static uint64_t packet_count = 0;
        packet_count++;

        static int16_t l9 = 0, l10 = 0;
        if (msg->data[9] != l9 || msg->data[10] != l10) {
            // RCLCPP_INFO(get_logger(), "SW Changed! [下(9):%d, 上(10):%d]",
            //             msg->data[9], msg->data[10]);
            l9 = msg->data[9];
            l10 = msg->data[10];
        }

        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        //                      "RX Heartbeat (Total:%lu) | Dump: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
        //                      packet_count,
        //                      msg->data[0], msg->data[1], msg->data[2], msg->data[3],
        //                      msg->data[4], msg->data[5], msg->data[6], msg->data[7],
        //                      msg->data[8], msg->data[9], msg->data[10], msg->data[11],
        //                      msg->data[12], msg->data[13], msg->data[14], msg->data[15]);
    }

    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_text_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr camera_servo_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr transition_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr drive_mode_cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet R2_MotionCtrl";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::MultiThreadedExecutor exec;

    auto hardware_control = std::make_shared<HardWareControl>(TARGET_DEVICE_ID);
    exec.add_node(hardware_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
