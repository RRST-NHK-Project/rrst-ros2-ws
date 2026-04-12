/*
R2段差超えシーケンス（状態管理）
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
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>

// ROS　
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

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
class SequenceControl : public rclcpp::Node
{
public:
    SequenceControl()
        : Node("sequence_ctrl_node"),
          pid_angle_(2.0f, 0.0f, 0.1f, 0.6f),
          pid_distance_(0.0008f, 0.0f, 0.0f, 0.4f)
    {
        pid_angle_.set_target(0.0f);
        pid_distance_.set_target(wall_approach_target_mm);

        timer_ = this->create_wall_timer(
            10ms,
            std::bind(&SequenceControl::loop, this));
    }

    // トリガー関数

    void start_wall_alignment()
    {
        if (mode_ != StepMode::NONE)
        {
            RCLCPP_WARN(get_logger(), "Sequence busy. WALL_ALIGN ignored.");
            return;
        }
        pid_angle_.reset();
        pid_distance_.reset();
        pid_distance_.set_target(wall_approach_target_mm);
        mode_ = StepMode::WALL_ALIGN;
        alignment_start_time_ = std::chrono::steady_clock::now();
    }

    void start_step_up()
    {
        if (mode_ != StepMode::NONE)
        {
            RCLCPP_WARN(get_logger(), "Sequence busy. STEP_UP ignored.");
            return; // 実行中なら無視
        }
        mode_ = StepMode::STEP_UP;
        next_up(StepUpState::ALL_UP);
    }

    void start_step_down()
    {
        if (mode_ != StepMode::NONE)
        {
            RCLCPP_WARN(get_logger(), "Sequence busy. STEP_DOWN ignored.");
            return; // 実行中なら無視
        }
        mode_ = StepMode::STEP_DOWN;
        next_down(StepDownState::FIRST_FORWARD);
    }

    // シーケンス実行中の判定
    bool is_busy() const
    {
        return mode_ != StepMode::NONE;
    }
    // sdm15の値を更新する関数
    void set_sdm15_value(int index, int32_t value)
    {
        sdm15_value_[index] = value;
    }

    // 壁角度を更新する関数
    void set_wall_angle(double angle)
    {
        wall_angle_ = angle;
    }

    // LiDAR距離を更新する関数
    void set_lidar_value(int16_t value)
    {
        lidar_value = value;
    }

private:
    // 以下シーケンス内で使用する変数
    //  待機時間（要調整）
    static constexpr double up_first_forward_wait = 2.0;
    static constexpr double up_second_forward_wait = 7.0;
    static constexpr double up_final_forward_wait = 6.0;
    static constexpr double down_first_forward_wait = 1.0;
    static constexpr double down_second_forward_wait = 1.0;
    static constexpr double down_final_forward_wait = 1.0;

    // 速度関連
    static constexpr int forward_speed = 40;
    static constexpr int up_speed = 40;
    static constexpr int dis = 100;      // 障害物と見なす距離の閾値（要調整）
    static constexpr int down_dis = 100; // sdm15の値がこの時間(ms)更新されなければタイムアウトと見なす

    static constexpr int wall = 100; // 前に障害物があると見なす距離の閾値（要調整）

    // 壁調整PID関連定数
    static constexpr float align_duty_max_ = 100.0f;
    // 距離PIDを起動する角度閾値 [rad]（この角度以内になったら前進を開始）
    static constexpr float wall_angle_approach_threshold_ = 0.15f; // 約9度

    // シーケンスの状態管理に必要な変数
    int32_t sdm15_value_[4] = {0, 0, 0, 0};
    int16_t lidar_value = 0;  // 前方最短距離 [mm]
    double wall_angle_ = 0.0; // 壁の角度 [rad]

    // モードの管理
    enum class StepMode
    {
        NONE,
        WALL_ALIGN,
        STEP_UP,
        STEP_DOWN
    };

    // 状態管理（上り）
    enum class StepUpState
    {
        IDLE,
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
    enum class StepDownState
    {
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

    // PIDコントローラ
    // pid_angle_    : 目標0[rad]、入力wall_angle_[rad]、出力=回転速度(正=CW, 負=CCW)
    // pid_distance_ : 目標wall_approach_target_mm[mm]、入力lidar_value[mm]、出力=前後速度
    //                 (equilibrium < wall_distance_threshold なのでトリガーを確実に通過する)
    PIDController pid_angle_;
    PIDController pid_distance_;

    StepMode mode_ = StepMode::NONE;

    StepUpState state_up_ = StepUpState::IDLE;
    StepDownState state_down_ = StepDownState::IDLE;

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point alignment_start_time_;
    static constexpr double wall_alignment_timeout    = 10.0;  // 壁調整タイムアウト [s]
    static constexpr double wall_angle_threshold      = 0.10;  // 角度整列完了閾値 [rad]（約6度）
    static constexpr int    wall_distance_threshold   = 250;   // 段差上り開始トリガー距離 [mm]
    static constexpr float  wall_approach_target_mm   = 120.0f; // PIDの接近目標距離 [mm]（threshold未満）
    rclcpp::Time state_start_time_;
    bool state_executed_ = false; // 各状態での処理の実行状況を保存

    // 状態遷移関数
    void next_up(StepUpState next)
    {
        state_start_time_ = this->now();
        state_up_ = next;
        state_executed_ = false;
    }

    void next_down(StepDownState next)
    {
        state_start_time_ = this->now();
        state_down_ = next;
        state_executed_ = false;
    }

    // 機構関数
    void all_up()
    {
        RCLCPP_INFO(get_logger(), "ALL UP");
        pkt.setTR(TR1, 1);
        pkt.setTR(TR2, 1);
    }

    void front_down()
    {
        RCLCPP_INFO(get_logger(), "FRONT DOWN");
        pkt.setTR(TR2, 0);
    }

    void rear_down()
    {
        RCLCPP_INFO(get_logger(), "REAR DOWN");
        pkt.setTR(TR1, 0);
        pkt.setMD(MD5, -up_speed);
        pkt.setMD(MD6, up_speed);
        pkt.setMD(MD7, up_speed);
        pkt.setMD(MD8, -up_speed);
    }

    void stop_motion()
    {
        RCLCPP_INFO(get_logger(), "STOP");
        pkt.setTR(TR1, 0);
        pkt.setTR(TR2, 0);
        pkt.setMD(MD5, 0);
        pkt.setMD(MD6, 0);
        pkt.setMD(MD7, 0);
        pkt.setMD(MD8, 0);
    }

    void front_up()
    {
        RCLCPP_INFO(get_logger(), "FRONT UP");
        pkt.setTR(TR2, 1);
        pkt.setMD(MD5, 0);
        pkt.setMD(MD6, 0);
        pkt.setMD(MD7, 0);
        pkt.setMD(MD8, 0);
    }

    void rear_up()
    {
        RCLCPP_INFO(get_logger(), "REAR UP");
        pkt.setTR(TR1, 1);
    }

    void all_down()
    {
        RCLCPP_INFO(get_logger(), "ALL DOWN");
        pkt.setTR(TR1, 0);
        pkt.setTR(TR2, 0);
    }

    void move_forward()
    {
        RCLCPP_INFO(get_logger(), "MOVE FORWARD");
        pkt.setMD(MD5, -forward_speed);
        pkt.setMD(MD6, forward_speed);
        pkt.setMD(MD7, forward_speed);
        pkt.setMD(MD8, -forward_speed);
    }

    void move_stop()
    {
        RCLCPP_INFO(get_logger(), "MOVE STOP");
        pkt.setMD(MD5, 10);
        pkt.setMD(MD6, 10);
        pkt.setMD(MD7, 10);
        pkt.setMD(MD8, 10);
    }
    // void move_backward() {
    //     RCLCPP_INFO(get_logger(), "MOVE BACKWARD");
    // }

    // 壁調整シーケンス（PID制御版）
    // 角度PID: wall_angle_→0[rad] への回転制御
    // 距離PID: lidar_value→wall_approach_target_mm[mm] への前進制御
    //          PID平衡点 < wall_distance_threshold なのでトリガーを確実に通過する
    //          （角度がwall_angle_approach_threshold_以内になってから前進起動）
    void wall_alignment_sequence()
    {
        constexpr float dt = 0.01f; // ループ周期 [s]（10msタイマー）

        // LiDARデータなし → モーター停止して待機
        if (lidar_value <= 0)
        {
            pkt.setMD(MD5, 0);
            pkt.setMD(MD6, 0);
            pkt.setMD(MD7, 0);
            pkt.setMD(MD8, 0);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Waiting for LiDAR data...");
            return;
        }

        // 終了条件：角度と距離の両方が閾値内 → 段差超えシーケンス開始
        if (std::abs(wall_angle_) < wall_angle_threshold && lidar_value < wall_distance_threshold)
        {
            RCLCPP_INFO(get_logger(),
                        "Wall aligned. Angle: %.4f rad, Distance: %d mm. Starting step up.",
                        wall_angle_, lidar_value);
            pkt.setMD(MD5, 0);
            pkt.setMD(MD6, 0);
            pkt.setMD(MD7, 0);
            pkt.setMD(MD8, 0);
            mode_ = StepMode::NONE;
            start_step_up();
            return;
        }

        // ── 角度PID ──────────────────────────────────────────────
        // error = target(0) - current(wall_angle_)
        // wall_angle_ > 0 → error < 0 → 負のwz → CCW回転
        // wall_angle_ < 0 → error > 0 → 正のwz → CW回転
        const float wz = pid_angle_.update(static_cast<float>(wall_angle_), dt);

        // ── 距離PID ──────────────────────────────────────────────
        // 角度がapproach閾値以内になってから前進を開始する
        // error = target(wall_distance_threshold) - current(lidar_value)
        // lidar > target → error < 0 → 負のvx → 前進（move_forward()の符号系に準拠）
        float vx = 0.0f;
        if (std::abs(wall_angle_) < wall_angle_approach_threshold_)
        {
            vx = pid_distance_.update(static_cast<float>(lidar_value), dt);
        }
        else
        {
            // 角度が大きい間は距離PIDの積分をリセットして偏差を蓄積させない
            pid_distance_.reset();
        }

        // ── メカナム逆運動学（vy=0） ─────────────────────────────
        // v1(MD5) =  vx + wz
        // v2(MD6) = -(vx - wz)  （向き補正 *=-1）
        // v3(MD7) = -(vx - wz)  （向き補正 *=-1）
        // v4(MD8) =  vx + wz
        float v1 =  vx + wz;
        float v2 = -(vx - wz);
        float v3 = -(vx - wz);
        float v4 =  vx + wz;

        // 出力クランプ（-1.0 ～ 1.0 の正規化済み値）
        v1 = std::clamp(v1, -1.0f, 1.0f);
        v2 = std::clamp(v2, -1.0f, 1.0f);
        v3 = std::clamp(v3, -1.0f, 1.0f);
        v4 = std::clamp(v4, -1.0f, 1.0f);

        pkt.setMD(MD5, static_cast<int16_t>(v1 * align_duty_max_));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * align_duty_max_));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * align_duty_max_));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * align_duty_max_));

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
                             "Wall align PID: angle=%.4f rad, dist=%d mm | vx=%.3f wz=%.3f",
                             wall_angle_, lidar_value, vx, wz);
    }

    // 段差超えシーケンス（上り）
    void step_up_sequence()
    {
        auto now_time = now();
        switch (state_up_)
        {
        case StepUpState::IDLE: // アイドリングストップ
            break;
            /////////////
        case StepUpState::ALL_FORWARD: // 前進
            if (!state_executed_)
            {
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
            if (!state_executed_)
            {
                all_up();
                state_executed_ = true;
            }
            next_up(StepUpState::FIRST_FORWARD);
            break;

        case StepUpState::FIRST_FORWARD: // 前進
            if (!state_executed_)
            {
                move_forward();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > up_first_forward_wait)
            {
                next_up(StepUpState::FRONT_DOWN);
            }
            break;

        case StepUpState::FRONT_DOWN: // 前を下げる
            if (!state_executed_)
            {
                front_down();
                state_executed_ = true;
            }
            next_up(StepUpState::SECOND_FORWARD);
            break;

        case StepUpState::SECOND_FORWARD: // 前進
            if (!state_executed_)
            {
                move_forward();
                state_executed_ = true;
            }
            // if ((now_time - state_start_time_).seconds() > up_second_forward_wait)
            // {
            //     next_up(StepUpState::REAR_DOWN);
            // }
            if (sdm15_value_[0] < dis)
            {
                next_up(StepUpState::REAR_DOWN);
            }
            break;

        case StepUpState::REAR_DOWN: // 後ろを下げる
            if (!state_executed_)
            {
                rear_down();
                state_executed_ = true;
            }
            next_up(StepUpState::FINAL_FORWARD);
            break;

        case StepUpState::FINAL_FORWARD: // 前進
            if (!state_executed_)
            {
                move_forward();
                state_executed_ = true;
            }
            if (sdm15_value_[2] < dis)
            {
                next_up(StepUpState::DONE);
            }
            break;

        case StepUpState::DONE:
            if (!state_executed_)
            {
                stop_motion();
                state_executed_ = true;
            }
            mode_ = StepMode::NONE;
            state_up_ = StepUpState::IDLE;
            break;
        }
    }

    // 段差超えシーケンス（下り）
    void step_down_sequence()
    {
        auto now_time = now();
        switch (state_down_)
        {
        case StepDownState::IDLE:
            break;

        case StepDownState::FIRST_FORWARD:
            if (!state_executed_)
            {
                move_forward();
                state_executed_ = true;
            }
            if (sdm15_value_[1] > down_dis || sdm15_value_[3] > down_dis) // 前のセンサーで障害物がなくなったら
            {
                next_down(StepDownState::FRONT_UP);
            }
            break;

        case StepDownState::FRONT_UP:
            if (!state_executed_)
            {
                front_up();
                state_executed_ = true;
            }
            next_down(StepDownState::STOP);
            break;
        case StepDownState::STOP:
            if (!state_executed_)
            {
                move_stop();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > down_second_forward_wait)
            {
                next_down(StepDownState::SECOND_FORWARD);
            }
            break;

        case StepDownState::SECOND_FORWARD:
            if (!state_executed_)
            {
                move_forward();
                state_executed_ = true;
            }
            if (sdm15_value_[0] > down_dis)
            {
                next_down(StepDownState::REAR_UP);
            }
            break;

        case StepDownState::REAR_UP:
            if (!state_executed_)
            {
                rear_up();
                state_executed_ = true;
            }
            next_down(StepDownState::FINAL_FORWARD);
            break;

        case StepDownState::FINAL_FORWARD:
            if (!state_executed_)
            {
                move_forward();
                state_executed_ = true;
            }
            if (sdm15_value_[2] > down_dis)
            {
                next_down(StepDownState::ALL_DOWN);
            }
            break;

        case StepDownState::ALL_DOWN:
            if (!state_executed_)
            {
                all_down();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > 0.5)
                next_down(StepDownState::DONE);
            break;

        case StepDownState::DONE:
            if (!state_executed_)
            {
                stop_motion();
                state_executed_ = true;
            }
            mode_ = StepMode::NONE;
            state_down_ = StepDownState::IDLE;
            break;
        }
    }

    void loop()
    {

        switch (mode_)
        {

        case StepMode::NONE:
            // std::cout << "None sequence" << std::endl;
            break;

        case StepMode::WALL_ALIGN:
            wall_alignment_sequence();
            break;

        case StepMode::STEP_UP:
            step_up_sequence();
            // std::cout << "Up sequence" << std::endl;
            break;

        case StepMode::STEP_DOWN:
            step_down_sequence();
            // std::cout << "Down sequence" << std::endl;
            break;
        }
    }
};

class HardWareControl : public rclcpp::Node
{
public:
    HardWareControl(uint8_t device_id, std::shared_ptr<SequenceControl> seq)
        : Node("hardware_control_" + std::to_string(device_id)),
          device_id_(device_id),
          seq_(seq)
    {

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

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/wall_detection/filtered_points",
            rclcpp::SensorDataQoS(),
            std::bind(&HardWareControl::lidar_callback, this, std::placeholders::_1));

        wall_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/wall_detection/angle",
            10,
            std::bind(&HardWareControl::wall_callback, this, std::placeholders::_1));

        sdm15_sub1_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_17",
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Int16MultiArray::SharedPtr msg)
            {
                this->sdm15_callback(msg, 0);
            });

        sdm15_sub2_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_18",
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Int16MultiArray::SharedPtr msg)
            {
                this->sdm15_callback(msg, 1);
            });

        sdm15_sub3_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_16",
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Int16MultiArray::SharedPtr msg)
            {
                this->sdm15_callback(msg, 2);
            });

        sdm15_sub4_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_19",
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Int16MultiArray::SharedPtr msg)
            {
                this->sdm15_callback(msg, 3);
            });

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

    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (seq_->is_busy())
        {
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

        static bool last_up = false;
        static bool last_down = false;

        if (UP && !last_up)
        {
            seq_->start_wall_alignment();
        }

        if (DOWN && !last_down)
        {
            seq_->start_step_down();
        }

        last_up = UP;
        last_down = DOWN;

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
        if (R1)
        {
            v1 = sp_yaw;
            v2 = -sp_yaw;
            v3 = -sp_yaw;
            v4 = sp_yaw;
        }

        if (L1)
        {
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

    void publisher_timer_callback()
    {
        std_msgs::msg::Int16MultiArray msg;

        // msg.data = data_;
        msg.data = pkt.toVector();

        publisher_->publish(msg);
        // print_data();
    }

    void print_data()
    {
        const auto &pkt_data = pkt.data_;
        std::cout << "TX DATA: [";
        for (size_t i = 0; i < pkt_data.size(); ++i)
        {
            std::cout << pkt_data[i];
            if (i + 1 < pkt_data.size())
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    void sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < RX16NUM)
        {
            RCLCPP_WARN(this->get_logger(),
                        "serial_rx_%d: data too short (%zu)",
                        device_id_, msg->data.size());
            return;
        }
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->width == 0)
            return;

        float nearest_x = 0.0f;
        float nearest_y = 0.0f;
        float min_distance = std::numeric_limits<float>::max();
        bool has_valid_point = false;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        const size_t point_count = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);

        for (size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y)
        {
            const float x = *iter_x;
            const float y = *iter_y;

            if (!std::isfinite(x) || !std::isfinite(y))
            {
                continue;
            }

            const float distance = std::sqrt(x * x + y * y);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_x = x;
                nearest_y = y;
                has_valid_point = true;
            }
        }

        if (!has_valid_point)
        {
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

    void wall_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        seq_->set_wall_angle(msg->data);
    }

    void sdm15_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg, int index)
    {
        // 配列が空でないか一応安全のためにチェック
        if (msg->data.empty())
        {
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
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
    int16_t lidar_x_value = 0;
    int16_t lidar_y_value = 0;

    // シーケンスの管理に使う
    std::shared_ptr<SequenceControl> seq_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet R2_SequenceCtrl";
    int result = std::system(figletout.c_str());
    if (result != 0)
    {
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