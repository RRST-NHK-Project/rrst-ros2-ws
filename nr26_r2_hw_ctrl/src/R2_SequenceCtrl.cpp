/*
R2統合シーケンス制御ノード
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

/*
R2について以下の不具合を確認しています。
L1、R1で回転しようとすると前進、後進してしまう
マイコンのモードによりロボマスとエンコーダの取得のみしかできずソレノイドの駆動ができない（新規モードの作成が必要）
シーケンス内の前進において前進せずその場で回転してしまう
現状
前 sdm15_value_[1],sdm15_value_[0],sdm15_value_[2] 後
順番ずつ装着する

*/

// 標準
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>

// ROS　
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float64.hpp"

// 自作
#include "include/PacketController.hpp"
PacketController pkt;
#include "include/PID.hpp"

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 6 // 宛先マイコンのID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 50 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

using namespace std::chrono_literals;

class SequenceControl;
class HardWareControl;

// 状態を管理しながらシーケンス制御
class SequenceControl : public rclcpp::Node {
public:
    SequenceControl()
        : Node("sequence_ctrl_node"),
          pid_angle_(2.0f, 0.0f, 0.1f, 0.6f),
          pid_distance_(0.0008f, 0.0f, 0.0f, 0.4f),
          pid_cube_yaw_(2.0f, 0.0f, 0.0f, 0.8f),
          pid_cube_dist_(3.0f, 0.0f, 0.0f, 1.0f),
          pid_cube_lat_(1.2f, 0.0f, 0.0f, 0.6f) {
        pid_angle_.set_target(0.0f);
        pid_distance_.set_target(wall_approach_target_mm);
        pid_cube_yaw_.set_target(0.0f);
        pid_cube_dist_.set_target(cube_approach_target_m);
        pid_cube_lat_.set_target(0.5f);
        this->declare_parameter("use_camera_offset", false);
        last_cube_update_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

        timer_ = this->create_wall_timer(
            10ms,
            std::bind(&SequenceControl::loop, this));
    }

    // トリガー関数

    // 壁調整シーケンス開始（PIDで角度・距離を整えてから段差上りへ自動移行）
    void start_wall_alignment() {
        if (mode_ != StepMode::NONE) {
            RCLCPP_WARN(get_logger(), "Sequence busy. WALL_ALIGN ignored.");
            return;
        }
        pid_angle_.reset();
        pid_distance_.reset();
        pid_distance_.set_target(wall_approach_target_mm);
        mode_ = StepMode::WALL_ALIGN;
        RCLCPP_INFO(get_logger(), "Wall alignment started.");
    }

    void start_step_up() {
        if (mode_ != StepMode::NONE) {
            RCLCPP_WARN(get_logger(), "Sequence busy. STEP_UP ignored.");
            return; // 実行中なら無視
        }
        mode_ = StepMode::STEP_UP;
        next_up(StepUpState::ALL_FORWARD);
    }

    bool start_mff_turn(int32_t turn_deg) {
        if (!mff_mode_enabled_) {
            return false;
        }

        if (turn_deg == 0) {
            return true;
        }

        if (mode_ != StepMode::NONE) {
            RCLCPP_WARN(get_logger(), "Sequence busy. MFF_TURN ignored.");
            return false;
        }

        // AutoDriveのメカナム逆運動学を参考に計算
        const float wz = (turn_deg > 0) ? turn_speed_norm_ : -turn_speed_norm_;

        // メカナム逆運動学（vx=0, vy=0）
        float v1 = 0.0f + 0.0f + wz; // vx + vy + wz
        float v3 = 0.0f - 0.0f - wz; // vx - vy - wz
        float v4 = 0.0f - 0.0f + wz; // vx - vy + wz
        float v2 = 0.0f + 0.0f - wz; // vx + vy - wz

        // 向き補正
        v3 *= -1;
        v2 *= -1;

        // 正規化
        float max_v = std::max(
            std::max(fabsf(v1), fabsf(v2)),
            std::max(fabsf(v3), fabsf(v4)));

        if (max_v < 1.0f)
            max_v = 1.0f;

        turn_v1_ = v1 / max_v;
        turn_v2_ = v2 / max_v;
        turn_v3_ = v3 / max_v;
        turn_v4_ = v4 / max_v;

        const double turn_sec = (std::abs(turn_deg) / 90.0) * turn_sec_per_90deg_;
        turn_end_time_ = this->now() + rclcpp::Duration::from_seconds(turn_sec);
        mode_ = StepMode::MFF_TURN;

        RCLCPP_INFO(get_logger(), "MFF turn started: %ld deg (%.2f s)", static_cast<long>(turn_deg), turn_sec);
        return true;
    }

    void start_step_down() {
        if (mode_ != StepMode::NONE) {
            RCLCPP_WARN(get_logger(), "Sequence busy. STEP_DOWN ignored.");
            return; // 実行中なら無視
        }
        mode_ = StepMode::STEP_DOWN;
        next_down(StepDownState::FIRST_FORWARD);
    }

    // シーケンス実行中の判定
    bool is_busy() const {
        return mode_ != StepMode::NONE;
    }

    bool is_mff_mode_enabled() const {
        return mff_mode_enabled_;
    }

    void set_mff_mode_enabled(bool enabled) {
        if (mff_mode_enabled_ == enabled) {
            return;
        }

        mff_mode_enabled_ = enabled;

        if (!mff_mode_enabled_) {
            mode_ = StepMode::NONE;
            state_up_ = StepUpState::IDLE;
            state_down_ = StepDownState::IDLE;
            state_executed_ = false;
            pkt.setMD(MD5, 0);
            pkt.setMD(MD6, 0);
            pkt.setMD(MD7, 0);
            pkt.setMD(MD8, 0);
        }

        RCLCPP_INFO(get_logger(), "Sequence mode: %s", mff_mode_enabled_ ? "MFF ENABLED" : "MFF DISABLED");
    }

    // sdm15の値を更新する関数
    void set_sdm15_value(int index, int32_t value) {
        sdm15_value_[index] = value;
    }

    // lidar値を更新する関数
    void set_lidar_value(int16_t value) {
        lidar_value = value;
    }

    // wall角度を更新する関数
    void set_wall_angle(double angle) {
        wall_angle = angle;
    }

    // カメラサーボの現在角度を返す
    int get_camera_servo_angle() const { return static_cast<int>(servo_camera_angle_); }

    // cube_detectionの情報を更新する関数
    void set_cube_info(float depth_m, float cx_norm, float cy_norm, float yaw_deg, bool detected) {
        cube_detected_ = detected;
        if (detected) {
            cube_depth_m_ = depth_m;
            cube_cx_norm_ = cx_norm;
            cube_cy_norm_ = cy_norm;
            cube_yaw_deg_ = yaw_deg;
            last_cube_update_ = this->now();
        }
    }

    // キューブへの平行接近PIDシーケンス開始
    void start_cube_align() {
        if (mode_ != StepMode::NONE) {
            RCLCPP_WARN(get_logger(), "Sequence busy. CUBE_ALIGN ignored.");
            return;
        }
        pid_cube_yaw_.reset();
        pid_cube_dist_.reset();
        pid_cube_lat_.reset();
        pid_cube_yaw_.set_target(0.0f);
        pid_cube_dist_.set_target(cube_approach_target_m);
        last_cube_update_ = this->now();
        servo_camera_angle_ = static_cast<float>(servo_scan_start);
        servo_scan_dir_ = 1.0f;
        // すでに検出中なら直接アライン、未検出ならスキャンから開始
        mode_ = cube_detected_ ? StepMode::CUBE_ALIGN : StepMode::CUBE_SCAN;
        RCLCPP_INFO(get_logger(), "Cube alignment started. target_dist=%.2f m", cube_approach_target_m);
    }

private:
    // 以下シーケンス内で使用する変数
    //  待機時間（要調整）
    static constexpr double up_first_forward_wait = 1.5;
    static constexpr double up_second_forward_wait = 7.0;
    static constexpr double up_final_forward_wait = 6.0;
    static constexpr double down_first_forward_wait = 1.0;
    static constexpr double down_second_forward_wait = 1.0;
    static constexpr double down_final_forward_wait = 1.0;

    // 速度関連
    static constexpr int forward_speed = 60;
    static constexpr int up_speed = 60;
    static constexpr int dis = 100;      // 障害物と見なす距離の閾値（要調整）
    static constexpr int down_dis = 100; // sdm15の値がこの時間(ms)更新されなければタイムアウトと見なす

    static constexpr int wall = 500; // 前に障害物があると見なす距離の閾値（要調整）

    // 壁調整PID関連定数
    // !! wall_approach_target_mm < wall_distance_threshold の関係を必ず保つこと !!
    // PIDがapproach_targetまで積極的に近づくことでdistance_thresholdを通過しトリガーが発火する
    static constexpr int wall_distance_threshold = 350;      // 段差上り開始トリガー距離 [mm]
    static constexpr float wall_approach_target_mm = 300.0f; // PIDの接近目標距離 [mm]（threshold未満に設定）
    static constexpr double wall_angle_threshold = 0.10;     // 角度整列完了閾値 [rad]（約6度）
    static constexpr float wall_angle_approach_thr = 0.20f;  // 前進開始角度閾値 [rad]（約11度）
    static constexpr float align_duty_max = 100.0f;

    // PIDコントローラ
    // pid_angle_   : 目標0[rad]、入力wall_angle[rad]、出力=回転速度(正=CW, 負=CCW)
    // pid_distance_: 目標wall_approach_target_mm[mm]、入力lidar_value[mm]、出力=前後速度
    //               (equilibrium < wall_distance_threshold なのでトリガーを確実に通過する)
    PIDController pid_angle_;
    PIDController pid_distance_;

    // キューブ接近用PID
    // pid_cube_dist_: 目標cube_approach_target_m[m]、入力depth_m[m]、出力=vx(前後)
    // pid_cube_lat_ : 目標cx_target[norm]、入力cx_norm[0-1]、出力=vy(横移動)
    // pid_cube_yaw_ : 目標0[rad]、入力face_yaw[rad]、出力=wz(回転)
    //                 現在は「距離・横位置を先に合わせ、最後にYAWを仕上げる」段階制御で使用
    PIDController pid_cube_yaw_;
    PIDController pid_cube_dist_;
    PIDController pid_cube_lat_;

    // シーケンスの状態管理に必要な変数
    int32_t sdm15_value_[4] = {0, 0, 0, 0};
    int16_t lidar_value = 0;
    double wall_angle = 0.0;

    // キューブ検出データ
    float cube_depth_m_ = 0.0f;
    float cube_cx_norm_ = 0.5f;
    float cube_cy_norm_ = 0.5f;
    float cube_yaw_deg_ = 0.0f;
    bool cube_detected_ = false;
    rclcpp::Time last_cube_update_;
    // カメラサーボ
    float servo_camera_angle_ = 70.0f; // 現在のサーボ角度 [deg]（起動時から原点）
    float servo_scan_dir_ = 1.0f;      // スキャン方向 (+1=増加, -1=減少)

    // キューブ接近PID定数

    static constexpr float cube_approach_target_m = 0.5f;   // 接近目標距離 [m]
    static constexpr double cube_angle_threshold = 0.05;    // YAW整列完了閾値 [rad]（約3度）
    static constexpr float cube_yaw_deadband_rad = 0.05f;   // YAW制御デッドバンド [rad]（約3度）
    static constexpr float cube_lateral_threshold = 0.08f;  // 横方向完了閾値 [cx_norm]
    static constexpr float cube_distance_threshold = 0.08f; // 距離完了閾値 [m]
    static constexpr float cube_yaw_approach_thr = 0.30f;   // 予備: YAW優先前進ゲート閾値 [rad]（現在未使用）
    static constexpr float cube_wz_max = 0.35f;             // 旋回速度上限（過旋回防止）
    static constexpr double cube_detect_timeout_sec = 0.5;  // 検出途切れ待機タイムアウト [s]
    static constexpr double cube_align_abort_sec = 1.0;     // アボートタイムアウト [s]

    // カメラ取り付けオフセット補正
    // カメラが前方中心から右に180mmオフセット→目標cx_normを左にシフト
    // cx_offset = 0.5 * camera_offset_m / (depth_m * tan(hfov/2))
    // tan_hfov_half: カメラの水平FOVの半角タンジェント（90°→1.0、60°→0.577）
    static constexpr float camera_offset_right_m = 0.290f;
    static constexpr float camera_tan_hfov_half = 1.0f; // 要調整（90°FOV想定）
    // カメラサーボ定数
    static constexpr int camera_servo_idx = SERVO1;      // ★使用するサーボ番号
    static constexpr int servo_scan_start = 40;          // スキャン開始角度 [deg]
    static constexpr int servo_scan_end = 70;            // スキャン終了角度 [deg]
    static constexpr float servo_scan_speed_dps = 20.0f; // スキャン速度 [deg/s]
    static constexpr float servo_track_kp = 60.0f;       // cy追跡ゲイン [deg/cy_err]
    static constexpr int servo_angle_min = 0;
    static constexpr int servo_angle_max = 270;

    // モードの管理
    enum class StepMode {
        NONE,
        WALL_ALIGN, // 壁調整PID（角度・距離を整えてから段差上りへ自動移行）
        STEP_UP,
        STEP_DOWN,
        MFF_TURN,
        CUBE_SCAN, // キューブ探索（サーボスキャン）
        CUBE_ALIGN // cube_detectionを使ったキューブへの平行接近PID
    };

    // 状態管理（上り）
    enum class StepUpState {
        IDLE,
        WALL,
        ALL_UP,
        ALL_FORWARD,
        FIRST_FORWARD,
        FRONT_DOWN,
        SECOND_FORWARD,
        REAR_DOWN,
        FINAL_FORWARD,
        DONE
    };

    // 状態管理（下り）
    enum class StepDownState {
        IDLE,
        FIRST_FORWARD,
        FRONT_UP,
        STOP,
        SECOND_FORWARD,
        REAR_UP,
        FINAL_FORWARD,
        ALL_DOWN,
        DONE
    };

    StepMode mode_ = StepMode::NONE;
    bool mff_mode_enabled_ = false;
    rclcpp::Time turn_end_time_;
    float turn_v1_ = 0.0f;
    float turn_v2_ = 0.0f;
    float turn_v3_ = 0.0f;
    float turn_v4_ = 0.0f;
    static constexpr float turn_speed_norm_ = 0.5f;
    static constexpr double turn_sec_per_90deg_ = 3.00;

    StepUpState state_up_ = StepUpState::IDLE;
    StepDownState state_down_ = StepDownState::IDLE;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time state_start_time_;
    bool state_executed_ = false; // 各状態での処理の実行状況を保存

    // 状態遷移関数
    void next_up(StepUpState next) {
        state_start_time_ = this->now();
        state_up_ = next;
        state_executed_ = false;
    }

    void next_down(StepDownState next) {
        state_start_time_ = this->now();
        state_down_ = next;
        state_executed_ = false;
    }

    // 機構関数
    void all_up() {
        RCLCPP_INFO(get_logger(), "ALL UP");
        pkt.setTR(TR1, 1);
        pkt.setTR(TR2, 1);
    }

    void front_down() {
        RCLCPP_INFO(get_logger(), "FRONT DOWN");
        pkt.setTR(TR2, 0);
    }

    void rear_down() {
        RCLCPP_INFO(get_logger(), "REAR DOWN");
        pkt.setTR(TR1, 0);
        pkt.setMD(MD5, -forward_speed);
        pkt.setMD(MD6, forward_speed);
        pkt.setMD(MD7, forward_speed);
        pkt.setMD(MD8, -forward_speed);
    }

    void stop_motion() {
        RCLCPP_INFO(get_logger(), "STOP");
        pkt.setTR(TR1, 0);
        pkt.setTR(TR2, 0);
        pkt.setMD(MD5, 0);
        pkt.setMD(MD6, 0);
        pkt.setMD(MD7, 0);
        pkt.setMD(MD8, 0);
    }

    void front_up() {
        RCLCPP_INFO(get_logger(), "FRONT UP");
        pkt.setTR(TR2, 1);
        pkt.setMD(MD5, 0);
        pkt.setMD(MD6, 0);
        pkt.setMD(MD7, 0);
        pkt.setMD(MD8, 0);
    }

    void rear_up() {
        RCLCPP_INFO(get_logger(), "REAR UP");
        pkt.setTR(TR1, 1);
    }

    void all_down() {
        RCLCPP_INFO(get_logger(), "ALL DOWN");
        pkt.setTR(TR1, 0);
        pkt.setTR(TR2, 0);
    }

    void move_forward() {
        RCLCPP_INFO(get_logger(), "MOVE FORWARD");
        pkt.setMD(MD5, -forward_speed);
        pkt.setMD(MD6, forward_speed);
        pkt.setMD(MD7, forward_speed);
        pkt.setMD(MD8, -forward_speed);
    }

    void move_stop() {
        RCLCPP_INFO(get_logger(), "MOVE STOP");
        pkt.setMD(MD5, 0);
        pkt.setMD(MD6, 0);
        pkt.setMD(MD7, 0);
        pkt.setMD(MD8, 0);
    }
    // void move_backward() {
    //     RCLCPP_INFO(get_logger(), "MOVE BACKWARD");
    // }

    // 壁調整シーケンス（PID制御版）
    // 角度PID: wall_angle → 0[rad] への回転制御
    // 距離PID: lidar_value → wall_distance_threshold[mm] への前進制御
    //          角度がwall_angle_approach_thr以内になってから前進を開始
    void wall_alignment_sequence() {
        constexpr float dt = 0.01f; // ループ周期 [s]（10msタイマー）

        // LiDARデータなし → 停止して待機
        if (lidar_value <= 0) {
            pkt.setMD(MD5, 0);
            pkt.setMD(MD6, 0);
            pkt.setMD(MD7, 0);
            pkt.setMD(MD8, 0);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "WALL_ALIGN: Waiting for LiDAR...");
            return;
        }

        // 終了条件：角度と距離の両方が閾値内 → 段差上りへ移行（ALL_UPから開始）
        if (std::abs(wall_angle) < wall_angle_threshold && lidar_value < wall_distance_threshold) {
            RCLCPP_INFO(get_logger(),
                        "Wall aligned. angle=%.4f rad, dist=%d mm -> Starting STEP_UP.",
                        wall_angle, lidar_value);
            pkt.setMD(MD5, 0);
            pkt.setMD(MD6, 0);
            pkt.setMD(MD7, 0);
            pkt.setMD(MD8, 0);
            // 既に壁に近いのでALL_FORWARDをスキップしてALL_UPから開始
            mode_ = StepMode::STEP_UP;
            next_up(StepUpState::ALL_UP);
            return;
        }

        // ── 角度PID ──────────────────────────────────────────────
        // error = 0 - wall_angle
        // wall_angle > 0 → error < 0 → 負のwz → CCW回転
        // wall_angle < 0 → error > 0 → 正のwz → CW回転
        const float wz = pid_angle_.update(static_cast<float>(wall_angle), dt);

        // ── 距離PID ──────────────────────────────────────────────
        // 角度がapproach閾値以内になってから前進を開始する
        float vx = 0.0f;
        if (std::abs(wall_angle) < wall_angle_approach_thr) {
            vx = pid_distance_.update(static_cast<float>(lidar_value), dt);
        } else {
            // 角度が大きい間はintegratorを蓄積させない
            pid_distance_.reset();
        }

        // ── メカナム逆運動学（vy=0） ─────────────────────────────
        // MD5(v1) =  vx + wz
        // MD6(v2) = -(vx - wz)  （向き補正 *=-1）
        // MD7(v3) = -(vx - wz)  （向き補正 *=-1）
        // MD8(v4) =  vx + wz
        float v1 = vx + wz;
        float v2 = -(vx - wz);
        float v3 = -(vx - wz);
        float v4 = vx + wz;

        v1 = std::clamp(v1, -1.0f, 1.0f);
        v2 = std::clamp(v2, -1.0f, 1.0f);
        v3 = std::clamp(v3, -1.0f, 1.0f);
        v4 = std::clamp(v4, -1.0f, 1.0f);

        pkt.setMD(MD5, static_cast<int16_t>(v1 * align_duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * align_duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * align_duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * align_duty_max));

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
                             "WALL_ALIGN: angle=%.4f rad, dist=%d mm | vx=%.3f wz=%.3f",
                             wall_angle, lidar_value, vx, wz);
    }

    // 段差超えシーケンス（上り）
    void step_up_sequence() {
        auto now_time = now();
        switch (state_up_) {
        case StepUpState::IDLE: // アイドリングストップ
            break;

        case StepUpState::WALL: // （未使用：壁調整はWALL_ALIGNモードで実施）
            break;

            /////////////
        case StepUpState::ALL_FORWARD: // 前進
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if (lidar_value < wall) // 前に障害物があるなら
            {
                next_up(StepUpState::ALL_UP);
            }
            break;

            //////////
        case StepUpState::ALL_UP: // 全て上げる
            if (!state_executed_) {
                all_up();
                state_executed_ = true;
            }
            next_up(StepUpState::FIRST_FORWARD);
            break;

        case StepUpState::FIRST_FORWARD: // 前進
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > up_first_forward_wait) {
                next_up(StepUpState::FRONT_DOWN);
            }
            break;

        case StepUpState::FRONT_DOWN: // 前を下げる
            if (!state_executed_) {
                front_down();
                state_executed_ = true;
            }
            // エアシリンダが動き終わるまで待つ（要調整）
            if ((now_time - state_start_time_).seconds() > 1.0) {
                next_up(StepUpState::SECOND_FORWARD);
            }
            break;

        case StepUpState::SECOND_FORWARD: // 前進
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if (sdm15_value_[2] < 120) {
                next_up(StepUpState::REAR_DOWN);
            }
            break;

        case StepUpState::REAR_DOWN: // 後ろを下げる
            if (!state_executed_) {
                rear_down();
                state_executed_ = true;
            }
            // エアシリンダが動き終わるまで待つ（要調整）
            if ((now_time - state_start_time_).seconds() > 1.0) {
                next_up(StepUpState::FINAL_FORWARD);
            }
            break;

        case StepUpState::FINAL_FORWARD: // 前進
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if (sdm15_value_[0] < dis) {
                next_up(StepUpState::DONE);
            }
            break;

        case StepUpState::DONE:
            if (!state_executed_) {
                stop_motion();
                state_executed_ = true;
            }
            mode_ = StepMode::NONE;
            state_up_ = StepUpState::IDLE;
            break;
        }
    }

    // 段差超えシーケンス（下り）
    void step_down_sequence() {
        auto now_time = now();
        switch (state_down_) {
        case StepDownState::IDLE:
            break;

        case StepDownState::FIRST_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if (sdm15_value_[2] > down_dis || sdm15_value_[3] > down_dis) // 前のセンサーで障害物がなくなったら
            {
                next_down(StepDownState::FRONT_UP);
            }
            break;

        case StepDownState::FRONT_UP:
            if (!state_executed_) {
                front_up();
                state_executed_ = true;
            }
            next_down(StepDownState::STOP);
            break;
        case StepDownState::STOP:
            if (!state_executed_) {
                move_stop();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > down_second_forward_wait) {
                next_down(StepDownState::SECOND_FORWARD);
            }
            break;

        case StepDownState::SECOND_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if (sdm15_value_[1] > down_dis) {
                next_down(StepDownState::REAR_UP);
            }
            break;

        case StepDownState::REAR_UP:
            if (!state_executed_) {
                rear_up();
                state_executed_ = true;
            }
            next_down(StepDownState::FINAL_FORWARD);
            break;

        case StepDownState::FINAL_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if (sdm15_value_[0] > down_dis) {
                next_down(StepDownState::ALL_DOWN);
            }
            break;

        case StepDownState::ALL_DOWN:
            if (!state_executed_) {
                all_down();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > 0.5)
                next_down(StepDownState::DONE);
            break;

        case StepDownState::DONE:
            if (!state_executed_) {
                stop_motion();
                state_executed_ = true;
            }
            mode_ = StepMode::NONE;
            state_down_ = StepDownState::IDLE;
            break;
        }
    }

    void mff_turn_sequence() {
        if (this->now() >= turn_end_time_) {
            pkt.setMD(MD5, 0);
            pkt.setMD(MD6, 0);
            pkt.setMD(MD7, 0);
            pkt.setMD(MD8, 0);
            mode_ = StepMode::NONE;
            RCLCPP_INFO(get_logger(), "MFF turn finished.");
            return;
        }

        pkt.setMD(MD5, static_cast<int16_t>(turn_v1_ * align_duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(turn_v2_ * align_duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(turn_v3_ * align_duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(turn_v4_ * align_duty_max));
    }

    // キューブ探索シーケンス: サーボを往復スキャンしてキューブを探す
    void cube_scan_sequence() {
        constexpr float dt = 0.01f;

        if (cube_detected_) {
            // 新しい検出に切り替わるタイミングでPID内部状態をクリアして、
            // スキャン中の履歴がCUBE_ALIGNに持ち越されないようにする。
            pid_cube_yaw_.reset();
            pid_cube_dist_.reset();
            pid_cube_lat_.reset();
            mode_ = StepMode::CUBE_ALIGN;
            RCLCPP_INFO(get_logger(), "CUBE_SCAN: cube found at servo=%.1f deg -> CUBE_ALIGN", servo_camera_angle_);
            return;
        }

        servo_camera_angle_ += servo_scan_dir_ * servo_scan_speed_dps * dt;
        if (servo_camera_angle_ >= static_cast<float>(servo_scan_end)) {
            servo_camera_angle_ = static_cast<float>(servo_scan_end);
            servo_scan_dir_ = -1.0f;
        } else if (servo_camera_angle_ <= static_cast<float>(servo_scan_start)) {
            servo_camera_angle_ = static_cast<float>(servo_scan_start);
            servo_scan_dir_ = 1.0f;
        }
        // サーボ送信はHardWareControl経由（Device7）
        pkt.setMD(MD5, 0);
        pkt.setMD(MD6, 0);
        pkt.setMD(MD7, 0);
        pkt.setMD(MD8, 0);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "CUBE_SCAN: angle=%.1f deg", servo_camera_angle_);
    }

    // cube_detectionを使ったキューブへの平行接近シーケンス（PID制御）
    // 制御順序:
    // 1) 距離PID  : depth_m → cube_approach_target_m [m]（目標距離まで前進）
    // 2) 横方向PID: cx_norm → cx_target（画像上の横位置を合わせる）
    // 3) YAW PID  : face_yaw_deg → 0 [rad]（最後に向きを仕上げる）
    // サーボ追跡: cy_norm → 0.5（カメラがキューブを捉え続けるよう垂直角調整）
    void cube_alignment_sequence() {
        constexpr float dt = 0.01f;

        const double age_sec = (this->now() - last_cube_update_).seconds();

        // アボート: 長時間未検出 → CUBE_SCANへ戻る
        if (age_sec > cube_align_abort_sec) {
            pkt.setMD(MD5, 0);
            pkt.setMD(MD6, 0);
            pkt.setMD(MD7, 0);
            pkt.setMD(MD8, 0);
            mode_ = StepMode::CUBE_SCAN;
            servo_camera_angle_ = static_cast<float>(servo_scan_start);
            servo_scan_dir_ = 1.0f;
            RCLCPP_WARN(get_logger(), "CUBE_ALIGN aborted: cube lost for %.1f s -> CUBE_SCAN", age_sec);
            return;
        }

        // 検出途切れ: 安全のため移動を停止
        if (!cube_detected_) {

            pkt.setMD(MD5, 0);
            pkt.setMD(MD6, 0);
            pkt.setMD(MD7, 0);
            pkt.setMD(MD8, 0);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                 "CUBE_ALIGN: cube lost, stopped. (%.1f s)", age_sec);
            return;
        }

        // カメラオフセット補正: 右290mmオフセットを常に適用
        const float cx_offset = 0.5f * camera_offset_right_m /
                                (std::max(cube_depth_m_, 0.1f) * camera_tan_hfov_half);
        const float cx_target = std::clamp(0.5f - cx_offset, 0.0f, 1.0f);

        const float yaw_rad = cube_yaw_deg_ * static_cast<float>(M_PI) / 180.0f;
        const float lat_error = cube_cx_norm_ - cx_target;
        const float dist_error = cube_depth_m_ - cube_approach_target_m;

        // 完了条件：YAW・距離・横位置が閾値内
        if (std::abs(yaw_rad) < cube_angle_threshold &&
            std::abs(dist_error) < cube_distance_threshold &&
            std::abs(lat_error) < cube_lateral_threshold) {
            pkt.setMD(MD5, 0);
            pkt.setMD(MD6, 0);
            pkt.setMD(MD7, 0);
            pkt.setMD(MD8, 0);
            mode_ = StepMode::NONE;
            RCLCPP_INFO(get_logger(),
                        "Cube aligned. yaw=%.2f deg, depth=%.3f m, cx=%.3f(tgt=%.2f) -> Done.",
                        cube_yaw_deg_, cube_depth_m_, cube_cx_norm_, cx_target);
            return;
        }

        // 仕上げYAWへ移るための前提条件（距離・横位置の粗合わせ完了）
        const bool dist_ready = std::abs(dist_error) < cube_distance_threshold;
        const bool lat_ready = std::abs(lat_error) < cube_lateral_threshold;

        // ── 距離・横方向PID ───────────────────────────────────────
        // yaw は最後に回すため、距離と横位置がまだ合っていない間は yaw を動かさない
        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;

        if (!(dist_ready && lat_ready)) {
            // YAWは最後に行うため、この段階では積分蓄積を避ける。
            pid_cube_yaw_.reset();
            vx = pid_cube_dist_.update(cube_depth_m_, dt);

            // 画像右側にキューブがある(cube_cx_norm_ > target)ときに
            // ロボットが右へ移動するよう符号を反転している。
            pid_cube_lat_.set_target(cx_target);
            vy = -pid_cube_lat_.update(cube_cx_norm_, dt);
        } else {
            vx = 0.0f;
            vy = 0.0f;

            // ── YAW制御 ───────────────────────────────────────────
            // PID出力の回転正方向と機体の回転正方向が逆のため符号を反転。
            wz = -pid_cube_yaw_.update(yaw_rad, dt);
            if (std::abs(yaw_rad) < cube_yaw_deadband_rad) {
                wz = 0.0f;
            }
            wz = std::clamp(wz, -cube_wz_max, cube_wz_max);
        }

        // ── サーボ追跡: cy_normで垂直方向を調整 ───────────────────
        const float cy_err = 0.5f - cube_cy_norm_;
        servo_camera_angle_ += servo_track_kp * cy_err * dt;
        servo_camera_angle_ = std::clamp(servo_camera_angle_, static_cast<float>(servo_angle_min), static_cast<float>(servo_angle_max));

        // ── メカナム逆運動学（フル3軸） ───────────────────────────
        // 各軸の比率を保つため、個別clampではなく4輪まとめて正規化する
        float v1 = vx + vy + wz;
        float v2 = vx + vy - wz;
        float v3 = vx - vy - wz;
        float v4 = vx - vy + wz;

        // 配線方向補正
        v2 *= -1.0f;
        v3 *= -1.0f;

        float max_v = std::max(
            std::max(std::abs(v1), std::abs(v2)),
            std::max(std::abs(v3), std::abs(v4)));
        if (max_v < 1.0f) {
            max_v = 1.0f;
        }

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        // align_duty_max を100%基準とした duty 指令へ変換して送信。
        pkt.setMD(MD5, static_cast<int16_t>(v1 * align_duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * align_duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * align_duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * align_duty_max));

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
                             "CUBE_ALIGN: yaw=%.2f deg, depth=%.3f m, cx=%.3f(tgt=%.2f) cy=%.3f servo=%.1f | vx=%.3f vy=%.3f wz=%.3f",
                             cube_yaw_deg_, cube_depth_m_, cube_cx_norm_, cx_target, cube_cy_norm_, servo_camera_angle_, vx, vy, wz);
    }

    void loop() {
        if (!mff_mode_enabled_) {
            return;
        }

        // 追加（0.5秒ごとに表示）
        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(),
        //     *this->get_clock(),
        //     500,
        //     "lidar_value: %d mm, %d mm",
        //     wall - lidar_value, lidar_value);
        switch (mode_) {

        case StepMode::NONE:
            // サーボはDevice7経由（R2_HandCtrl）で制御するため、ここでは待機中角度をキープ
            servo_camera_angle_ = 70.0f;
            break;

        case StepMode::WALL_ALIGN:
            wall_alignment_sequence();
            break;

        case StepMode::STEP_UP:
            step_up_sequence();
            break;

        case StepMode::STEP_DOWN:
            step_down_sequence();
            break;

        case StepMode::MFF_TURN:
            mff_turn_sequence();
            break;

        case StepMode::CUBE_SCAN:
            cube_scan_sequence();
            break;

        case StepMode::CUBE_ALIGN:
            cube_alignment_sequence();
            break;
        }
    }
};

class HardWareControl : public rclcpp::Node {
public:
    HardWareControl(uint8_t device_id, std::shared_ptr<SequenceControl> seq)
        : Node("hardware_control_" + std::to_string(device_id)),
          device_id_(device_id),
          seq_(seq) {

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
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

        // sdm15のSubscribe
        sdm15_sub1_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_16",
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Int16MultiArray::SharedPtr msg) {
                this->sdm15_callback(msg, 0);
            });

        sdm15_sub2_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_17",
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Int16MultiArray::SharedPtr msg) {
                this->sdm15_callback(msg, 1);
            });

        sdm15_sub3_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_18",
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Int16MultiArray::SharedPtr msg) {
                this->sdm15_callback(msg, 2);
            });

        sdm15_sub4_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_19",
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Int16MultiArray::SharedPtr msg) {
                this->sdm15_callback(msg, 3);
            });

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/wall_detection/filtered_points",
            rclcpp::SensorDataQoS(),
            std::bind(&HardWareControl::lidar_callback, this, std::placeholders::_1));

        wall_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/wall_detection/angle",
            10,
            std::bind(&HardWareControl::wall_callback, this, std::placeholders::_1));

        step_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/r2_mff_step_cmd",
            10,
            std::bind(&HardWareControl::step_callback, this, std::placeholders::_1));

        turn_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/r2_mff_turn_cmd",
            10,
            std::bind(&HardWareControl::turn_callback, this, std::placeholders::_1));

        mode_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "r2_drive_mode_cmd",
            10,
            std::bind(&HardWareControl::mode_cmd_callback, this, std::placeholders::_1));

        cube_detect_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/cube_detection/info",
            rclcpp::SensorDataQoS(),
            std::bind(&HardWareControl::cube_detect_callback, this, std::placeholders::_1));

        cube_align_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/r2_cube_align_cmd",
            10,
            std::bind(&HardWareControl::cube_align_cmd_callback, this, std::placeholders::_1));

        odom_reset_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "odom_reset", 10);

        camera_servo_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/r2/camera_servo_angle", 10);

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", device_id_);
    }

private:
    // 定数・変数
    float duty_max = 100;
    float for_speed = 50;
    float back_speed = 50;
    float sp_yaw = 0.5;
    float deadzone = 0.3; // adjust DS4 deadzone

    float v1, v2, v3, v4; // 各メカナムホイールの速度指令値
                          // v1:第一象限, v2:第二象限, v3:第三象限, v4:第四象限

    int32_t sdm15_value[4] = {0, 0, 0, 0}; // sdm15の値を保存する配列
    int16_t lidar_x_value = 0;
    int16_t lidar_y_value = 0;
    int32_t pending_step_cmd_ = 0;
    int32_t pending_turn_deg_ = 0;
    bool has_pending_turn_ = false;

    void publish_odom_reset() {
        std_msgs::msg::Bool reset_msg;
        reset_msg.data = true;
        odom_reset_pub_->publish(reset_msg);
        RCLCPP_INFO(get_logger(), "Published odom_reset before MFF turn.");
    }

    void dispatch_step_command(int cmd) {
        if (cmd == 1) {
            // 壁調整PID → 自動段差上り
            seq_->start_wall_alignment();
        } else if (cmd == 2) {
            // 壁調整なしで直接段差上り（角度・距離に関係なく実行）
            seq_->start_step_up();
        } else if (cmd == -1) {
            seq_->start_step_down();
        }
    }

    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (!seq_->is_mff_mode_enabled()) {
            return;
        }

        if (seq_->is_busy()) {
            return;
        }

        float LS_X = -msg->axes[0]; // 左右
        float LS_Y = msg->axes[1];  // 前後
        float RS_X = -msg->axes[3]; // 回転

        float R2 = (-msg->axes[5] + 1) / 2;

        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        bool PS = msg->buttons[10];

        static bool last_up = false;
        static bool last_down = false;
        static bool last_ps = false;

        if (UP && !last_up) {
            // 十字キー上：壁調整PID → 自動段差上り
            seq_->start_wall_alignment();
        }

        if (DOWN && !last_down) {
            seq_->start_step_down();
        }

        if (PS && !last_ps) {
            // PSボタン：キューブへの平行接近PID
            seq_->start_cube_align();
        }

        last_up = UP;
        last_down = DOWN;
        last_ps = PS;

        if (fabsf(LS_X) < deadzone)
            LS_X = 0;
        if (fabsf(LS_Y) < deadzone)
            LS_Y = 0;
        if (fabsf(RS_X) < deadzone)
            RS_X = 0;

        float vx = -LS_Y * R2;    // 前後
        float vy = LS_X * R2;     // 左右
        float wz = RS_X * sp_yaw; // 回転

        // 逆運動学
        v1 = vx + vy + wz; // 前左
        v3 = vx - vy - wz; // 前右
        v4 = vx - vy + wz; // 後左
        v2 = vx + vy - wz; // 後右

        // 向き補正
        v3 *= -1;
        v2 *= -1;

        // 回転
        if (R1) {
            v1 = sp_yaw;
            v2 = -sp_yaw;
            v3 = -sp_yaw;
            v4 = sp_yaw;
        }

        if (L1) {
            v1 = -sp_yaw;
            v2 = sp_yaw;
            v3 = sp_yaw;
            v4 = -sp_yaw;
        }

        // 正規化
        float max_v = std::max(
            std::max(fabsf(v1), fabsf(v2)),
            std::max(fabsf(v3), fabsf(v4)));

        if (max_v < 1.0f)
            max_v = 1.0f;

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        // 出力
        pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));
    }

    void publisher_timer_callback() {
        // MFFモード無効でもサーボ角度は常に送信
        {
            std_msgs::msg::Int32 servo_msg;
            servo_msg.data = seq_->get_camera_servo_angle();
            camera_servo_pub_->publish(servo_msg);
        }

        if (!seq_->is_mff_mode_enabled()) {
            return;
        }

        if (!seq_->is_busy()) {
            if (has_pending_turn_) {
                publish_odom_reset();
                if (seq_->start_mff_turn(pending_turn_deg_)) {
                    has_pending_turn_ = false;
                }
            } else if (pending_step_cmd_ != 0) {
                const int32_t cmd = pending_step_cmd_;
                pending_step_cmd_ = 0;
                dispatch_step_command(cmd);
            }
        }

        std_msgs::msg::Int16MultiArray msg;
        msg.data = pkt.toVector();
        publisher_->publish(msg);
        print_data();

        // カメラサーボ角度をDevice7（R2_HandCtrl）へ通知
        std_msgs::msg::Int32 servo_msg;
        servo_msg.data = seq_->get_camera_servo_angle();
        camera_servo_pub_->publish(servo_msg);
    }

    void print_data() {
        const auto &pkt_data = pkt.data_;
        std::cout << "TX DATA: [";
        for (size_t i = 0; i < pkt_data.size(); ++i) {
            std::cout << pkt_data[i];
            if (i + 1 < pkt_data.size())
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    void sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() < RX16NUM) {
            RCLCPP_WARN(this->get_logger(),
                        "serial_rx_%d: data too short (%zu)",
                        device_id_, msg->data.size());
            return;
        }
    }

    void sdm15_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg, int index) {
        // 配列が空でないか一応安全のためにチェック
        if (msg->data.empty()) {
            return;
        }

        // 2. 配列(msg->data)の中から、距離データが入っている「番目」を取り出す
        // ※ここでは仮に 0番目 としていますが、実際のマイコンの仕様に合わせて変更してください。
        int16_t distance_val = msg->data[0];

        // 3. 取り出した値を保存
        sdm15_value[index] = distance_val;
        seq_->set_sdm15_value(index, distance_val);

        // RCLCPP_INFO(this->get_logger(),
        //             "distance: %d, %d, %d",
        //             sdm15_value[0], sdm15_value[1], sdm15_value[2]);
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->width == 0)
            return;

        float nearest_x = 0.0f;
        float nearest_y = 0.0f;
        float min_distance = std::numeric_limits<float>::max();
        bool has_valid_point = false;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        const size_t point_count = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);

        for (size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y) {
            const float x = *iter_x;
            const float y = *iter_y;

            if (!std::isfinite(x) || !std::isfinite(y)) {
                continue;
            }

            const float distance = std::sqrt(x * x + y * y);
            if (distance < min_distance) {
                min_distance = distance;
                nearest_x = x;
                nearest_y = y;
                has_valid_point = true;
            }
        }

        if (!has_valid_point) {
            return;
        }

        // // --- ここから追加：デバッグ用表示コード ---
        // // 500ミリ秒（0.5秒）に1回、ターミナルに最も近い点の座標を表示します
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        // "Nearest Point -> x: %.3f m, y: %.3f m, dist: %.3f m",
        // nearest_x, nearest_y, min_distance);
        // // ---------------------------------------

        lidar_x_value = static_cast<int16_t>(nearest_x * 1000.0f);
        lidar_y_value = static_cast<int16_t>(nearest_y * 1000.0f);
        seq_->set_lidar_value(static_cast<int16_t>(min_distance * 1000.0f));
    }

    void wall_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        seq_->set_wall_angle(msg->data);
    }

    void step_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (!seq_->is_mff_mode_enabled()) {
            return;
        }

        const int32_t cmd = msg->data;
        if (cmd == 0) {
            return;
        }

        if (seq_->is_busy()) {
            pending_step_cmd_ = cmd;
            return;
        }

        dispatch_step_command(cmd);
    }

    void turn_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (!seq_->is_mff_mode_enabled()) {
            return;
        }

        const int32_t turn_deg = msg->data;
        if (turn_deg == 0) {
            return;
        }

        publish_odom_reset();
        if (!seq_->start_mff_turn(turn_deg)) {
            pending_turn_deg_ = turn_deg;
            has_pending_turn_ = true;
        }
    }

    void mode_cmd_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.empty()) {
            return;
        }

        const int32_t mode_code = msg->data[0];
        seq_->set_mff_mode_enabled(mode_code == 4);
    }

    // /cube_detection/info [flag, cx_norm, cy_norm, w_norm, h_norm, depth_m, score, area, face_yaw_deg]
    void cube_detect_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 9) {
            return;
        }
        const bool detected = msg->data[0] > 0.5f;
        const float cx_norm = msg->data[1];
        const float cy_norm = msg->data[2];
        const float depth_m = msg->data[5];
        const float yaw_deg = msg->data[8];
        seq_->set_cube_info(depth_m, cx_norm, cy_norm, yaw_deg, detected);
    }

    void cube_align_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (!seq_->is_mff_mode_enabled()) {
            return;
        }
        if (msg->data == 1) {
            seq_->start_cube_align();
        }
    }
    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wall_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sdm15_sub1_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sdm15_sub2_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sdm15_sub3_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sdm15_sub4_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr step_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr turn_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cube_detect_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cube_align_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr odom_reset_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr camera_servo_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;

    // シーケンスの管理に使う
    std::shared_ptr<SequenceControl> seq_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet R2_SequenceCtrl";
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

    auto sequence_control = std::make_shared<SequenceControl>();
    auto hardware_control = std::make_shared<HardWareControl>(TARGET_DEVICE_ID, sequence_control);
    exec.add_node(hardware_control);
    exec.add_node(sequence_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}