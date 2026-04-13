/*
R2 PID メカナム制御ノード
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// 標準
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

// ROS
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

// 自作
#include "include/PacketController.hpp"
PacketController pkt;
#include "include/PID.hpp"

#define TARGET_DEVICE_ID 6
#define PUBLISH_RATE_MS 50

class PIDMecanumController : public rclcpp::Node {
public:
    PIDMecanumController()
        : Node("pid_mecanum_controller"),
          pid_x_(5.0, 0.0, 0.0, 1.0),
          pid_y_(5.0, 0.0, 0.0, 1.0),
          pid_yaw_(3.0, 0.0, 0.0, 1.0) {

        // odom subscriber
        odom_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odom_xy_yaw", 10,
            std::bind(&PIDMecanumController::odom_callback, this, std::placeholders::_1));

        // マイコン送信
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(TARGET_DEVICE_ID), 10);

        // GUI表示用モード配信
        mode_pub_ = this->create_publisher<std_msgs::msg::String>(
            "r2_drive_mode", rclcpp::QoS(1).transient_local().reliable());

        // オドメトリリセット
        odom_reset_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "odom_reset", rclcpp::QoS(10));

        // PS4入力
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PIDMecanumController::ps4_listener_callback, this, std::placeholders::_1));

        // r2_console からの目標位置/姿勢
        target_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "r2_autodrive_cmd", 10,
            std::bind(&PIDMecanumController::target_callback, this, std::placeholders::_1));

        // r2_console からのドライブモードコマンド
        mode_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "r2_drive_mode_cmd", 10,
            std::bind(&PIDMecanumController::mode_cmd_callback, this, std::placeholders::_1));

        // ArUco追従入力
        aruco_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_pose", 10,
            std::bind(&PIDMecanumController::aruco_pose_callback, this, std::placeholders::_1));
        aruco_distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/aruco_distance", 10,
            std::bind(&PIDMecanumController::aruco_distance_callback, this, std::placeholders::_1));
        aruco_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/aruco_id", 10,
            std::bind(&PIDMecanumController::aruco_id_callback, this, std::placeholders::_1));
        aruco_target_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "r2_aruco_target_id", 10,
            std::bind(&PIDMecanumController::aruco_target_id_callback, this, std::placeholders::_1));
        aruco_camera_offset_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "r2_aruco_camera_offset", 10,
            std::bind(&PIDMecanumController::aruco_camera_offset_callback, this, std::placeholders::_1));

        // MFF 旋回コマンド
        mff_turn_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "r2_mff_turn_cmd", 10,
            std::bind(&PIDMecanumController::mff_turn_cmd_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to r2_mff_turn_cmd topic");

        // timer（制御周期固定）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&PIDMecanumController::publish_timer, this));

        // 初期target
        target_x_ = 0.0;
        target_y_ = 0.0;
        target_yaw_ = 0.0;
        pid_x_.set_target(target_x_);
        pid_y_.set_target(target_y_);
        pid_yaw_.set_target(target_yaw_);
        this->declare_parameter("aruco_target_forward_m", 0.0);
        this->declare_parameter("aruco_target_lateral_m", 0.0);
        this->declare_parameter("aruco_target_yaw_rad", 0.0);
        this->declare_parameter("aruco_target_id", -1);
        this->declare_parameter("aruco_camera_offset_x_m", -0.1735);
        this->declare_parameter("aruco_camera_offset_y_m", 0.0);
        this->declare_parameter("enable_ps4_mode_toggle", false);
        aruco_target_forward_ = static_cast<float>(this->get_parameter("aruco_target_forward_m").as_double());
        aruco_target_lateral_ = static_cast<float>(this->get_parameter("aruco_target_lateral_m").as_double());
        aruco_target_yaw_ = static_cast<float>(this->get_parameter("aruco_target_yaw_rad").as_double());
        aruco_target_id_ = this->get_parameter("aruco_target_id").as_int();
        aruco_camera_offset_x_ = static_cast<float>(this->get_parameter("aruco_camera_offset_x_m").as_double());
        aruco_camera_offset_y_ = static_cast<float>(this->get_parameter("aruco_camera_offset_y_m").as_double());
        enable_ps4_mode_toggle_ = this->get_parameter("enable_ps4_mode_toggle").as_bool();
        last_aruco_update_ = this->get_clock()->now();
        publish_mode();

        RCLCPP_INFO(this->get_logger(), "PS4 mode toggle (OPTION): %s",
                    enable_ps4_mode_toggle_ ? "ENABLED" : "DISABLED");
    }

private:
    enum class DriveMode {
        MANUAL,
        AUTO,
        ARUCO,
        PLANE,
        MFF,
    };

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr aruco_distance_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_id_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_target_id_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr aruco_camera_offset_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mff_turn_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr odom_reset_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // PID
    PIDController pid_x_;
    PIDController pid_y_;
    PIDController pid_yaw_;

    // odom
    float X_ = 0.0;
    float Y_ = 0.0;
    float yaw_ = 0.0;

    // target
    float target_x_ = 0.0;
    float target_y_ = 0.0;
    float target_yaw_ = 0.0;
    DriveMode drive_mode_ = DriveMode::MANUAL;
    bool has_target_cmd_ = false;

    // ArUco
    float aruco_x_ = 0.0f;
    float aruco_y_ = 0.0f;
    float aruco_z_ = 0.0f;
    float aruco_distance_ = 0.0f;
    rclcpp::Time last_aruco_update_;
    bool has_aruco_pose_ = false;
    float aruco_target_forward_ = 0.0f;
    float aruco_target_lateral_ = 0.0f;
    float aruco_target_yaw_ = 0.0f;
    int aruco_target_id_ = -1;
    int current_aruco_id_ = -1;
    float aruco_camera_offset_x_ = 0.0f;
    float aruco_camera_offset_y_ = 0.0f;

    // 制御出力
    float vx_ = 0.0;
    float vy_ = 0.0;
    float wz_ = 0.0;

    // mecanum
    float duty_max = 100;
    float sp_yaw = 0.5;
    float deadzone = 0.3;
    float v1 = 0, v2 = 0, v3 = 0, v4 = 0;
    float aruco_max_vx_ = 0.4f;
    float aruco_max_vy_ = 0.4f;
    float aruco_max_wz_ = 0.6f;
    float aruco_k_vx_ = 1.0f;
    float aruco_k_vy_ = 1.2f;
    float aruco_k_wz_ = 1.4f;
    float aruco_pose_timeout_sec_ = 0.5f;
    float aruco_horizontal_deadband_m_ = 0.02f;
    bool enable_ps4_mode_toggle_ = false;

    static std::string drive_mode_to_string(DriveMode mode) {
        switch (mode) {
        case DriveMode::MANUAL:
            return "MANUAL";
        case DriveMode::AUTO:
            return "AUTO";
        case DriveMode::ARUCO:
            return "ARUCO";
        case DriveMode::PLANE:
            return "PLANE";
        case DriveMode::MFF:
            return "MFF";
        }
        return "MANUAL";
    }

    static bool is_pose_pid_mode(DriveMode mode) {
        return mode == DriveMode::AUTO || mode == DriveMode::PLANE;
    }

    static float normalize_angle_rad(float angle) {
        while (angle > static_cast<float>(M_PI))
            angle -= static_cast<float>(2.0 * M_PI);
        while (angle < static_cast<float>(-M_PI))
            angle += static_cast<float>(2.0 * M_PI);
        return angle;
    }

    void publish_mode() {
        std_msgs::msg::String mode_msg;
        mode_msg.data = drive_mode_to_string(drive_mode_);
        mode_pub_->publish(mode_msg);
    }

    void stop_motors() {
        pkt.setMD(MD5, 0);
        pkt.setMD(MD6, 0);
        pkt.setMD(MD7, 0);
        pkt.setMD(MD8, 0);
        vx_ = 0.0f;
        vy_ = 0.0f;
        wz_ = 0.0f;
    }

    void enter_auto_mode() {
        if (!has_target_cmd_) {
            target_x_ = X_;
            target_y_ = Y_;
            target_yaw_ = yaw_;
        }

        pid_x_.set_target(target_x_);
        pid_y_.set_target(target_y_);
        pid_yaw_.set_target(target_yaw_);
        pid_x_.reset();
        pid_y_.reset();
        pid_yaw_.reset();

        drive_mode_ = DriveMode::AUTO;
        stop_motors();
        publish_mode();
    }

    void enter_aruco_mode() {
        drive_mode_ = DriveMode::ARUCO;
        stop_motors();
        publish_mode();
    }

    void enter_manual_mode() {
        drive_mode_ = DriveMode::MANUAL;
        stop_motors();
        publish_mode();
    }

    // odom（状態更新のみ）
    void odom_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3)
            return;

        X_ = msg->data[0];
        Y_ = msg->data[1];
        yaw_ = msg->data[2];
    }

    void target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3)
            return;

        if (drive_mode_ == DriveMode::ARUCO) {
            aruco_target_forward_ = msg->data[0];
            aruco_target_lateral_ = msg->data[1];
            aruco_target_yaw_ = msg->data[2];
            RCLCPP_INFO(this->get_logger(),
                        "Aruco relative target updated forward=%.3f lateral=%.3f yaw=%.3f [rad]",
                        aruco_target_forward_, aruco_target_lateral_, aruco_target_yaw_);
            return;
        }

        // Check if this is a "rotation only" command (element [3] == 1)
        bool rotate_only = (msg->data.size() >= 4 && msg->data[3] != 0.0f);

        if (rotate_only) {
            // Rotation only: keep current position as target, only update yaw
            target_x_ = X_;
            target_y_ = Y_;
            target_yaw_ = msg->data[2];

            RCLCPP_INFO(this->get_logger(),
                        "Rotation-only commanded: keeping x=%.3f y=%.3f, targeting yaw=%.3f [rad]",
                        target_x_, target_y_, target_yaw_);
        } else {
            // Normal mode: update all three targets
            target_x_ = msg->data[0];
            target_y_ = msg->data[1];
            target_yaw_ = msg->data[2];

            RCLCPP_INFO(this->get_logger(),
                        "Target updated x=%.3f y=%.3f yaw=%.3f [rad]",
                        target_x_, target_y_, target_yaw_);
        }

        has_target_cmd_ = true;

        pid_x_.set_target(target_x_);
        pid_y_.set_target(target_y_);
        pid_yaw_.set_target(target_yaw_);
        pid_x_.reset();
        pid_y_.reset();
        pid_yaw_.reset();

        if (!is_pose_pid_mode(drive_mode_)) {
            drive_mode_ = DriveMode::PLANE;
            stop_motors();
            publish_mode();
            RCLCPP_INFO(this->get_logger(), "Mode changed: PLANE (by target command)");
        }
    }

    void mode_cmd_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!msg || msg->data.size() < 1) {
            RCLCPP_WARN(this->get_logger(), "Invalid mode command message");
            return;
        }

        int32_t mode_code = msg->data[0];
        bool rotate_only = (msg->data.size() >= 2 && msg->data[1] != 0);

        if (mode_code < 0 || mode_code > 4) {
            RCLCPP_WARN(this->get_logger(), "Invalid mode code: %ld (valid: 0-4)", static_cast<long>(mode_code));
            return;
        }

        DriveMode new_mode = static_cast<DriveMode>(mode_code);
        if (drive_mode_ == new_mode) {
            return; // No change
        }

        if (rotate_only) {
            // Rotation only: set current position as target, keep mode but apply rotation-only
            target_x_ = X_;
            target_y_ = Y_;
            has_target_cmd_ = true;

            pid_x_.set_target(target_x_);
            pid_y_.set_target(target_y_);
            pid_x_.reset();
            pid_y_.reset();

            RCLCPP_INFO(this->get_logger(), "Mode changed: %s (rotation-only, target set to current position)", drive_mode_to_string(new_mode).c_str());
        } else {
            drive_mode_ = new_mode;
            stop_motors();
            publish_mode();
            RCLCPP_INFO(this->get_logger(), "Mode changed: %s (by mode command)", drive_mode_to_string(new_mode).c_str());
        }
    }

    void aruco_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (aruco_target_id_ >= 0 && current_aruco_id_ != aruco_target_id_) {
            has_aruco_pose_ = false;
            return;
        }

        aruco_x_ = msg->pose.position.x;
        aruco_y_ = msg->pose.position.y;
        aruco_z_ = msg->pose.position.z;
        has_aruco_pose_ = true;
        last_aruco_update_ = this->get_clock()->now();
    }

    void aruco_distance_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        aruco_distance_ = msg->data;
    }

    void aruco_id_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_aruco_id_ = msg->data;
    }

    void aruco_target_id_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        aruco_target_id_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Aruco target ID updated: %d", aruco_target_id_);
    }

    void aruco_camera_offset_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            return;
        }

        aruco_camera_offset_x_ = msg->data[0];
        aruco_camera_offset_y_ = msg->data[1];
        RCLCPP_INFO(this->get_logger(),
                    "Aruco camera offset updated x=%.3f y=%.3f [m]",
                    aruco_camera_offset_x_, aruco_camera_offset_y_);
    }

    void mff_turn_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "[DEBUG] mff_turn_cmd received: turn_deg=%ld, current_mode=%d",
                    static_cast<long>(msg->data), static_cast<int>(drive_mode_));

        if (drive_mode_ != DriveMode::MFF) {
            RCLCPP_WARN(this->get_logger(), "[MFF] Ignoring turn command: not in MFF mode (current mode=%d)",
                        static_cast<int>(drive_mode_));
            return; // MFF モード以外では無視
        }

        int32_t turn_deg = msg->data;
        if (turn_deg == 0) {
            // 旋回指令なし = 旋回完了、停止
            target_yaw_ = yaw_;
            has_target_cmd_ = false;
            RCLCPP_INFO(this->get_logger(), "[MFF] No turn command (stay current yaw: %.2f rad)", yaw_);
            return;
        }

        // オドメトリをリセット
        std_msgs::msg::Bool reset_msg;
        reset_msg.data = true;
        odom_reset_pub_->publish(reset_msg);
        RCLCPP_INFO(this->get_logger(), "[MFF] Odometry reset before turn command");

        // 度数法から弧度法へ
        // オドメトリをリセットするため、目標角は絶対値として turn_rad を使用
        const float turn_rad = static_cast<float>(turn_deg) * static_cast<float>(M_PI) / 180.0f;
        target_yaw_ = normalize_angle_rad(turn_rad);

        pid_yaw_.set_target(target_yaw_);
        pid_yaw_.reset();
        has_target_cmd_ = true;

        RCLCPP_INFO(this->get_logger(), "[MFF] Turn command: turn_deg=%ld, current_yaw=%.2f rad, target_yaw=%.2f rad",
                    static_cast<long>(turn_deg), yaw_, target_yaw_);
    }

    // PS4入力
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 8 || msg->buttons.size() < 10)
            return;

        bool OPTION = msg->buttons[9];
        static bool last_option = false;

        if (enable_ps4_mode_toggle_ && OPTION && !last_option) {
            if (drive_mode_ == DriveMode::MANUAL) {
                drive_mode_ = DriveMode::PLANE;
                stop_motors();
                publish_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: PLANE");
            } else if (drive_mode_ == DriveMode::PLANE || drive_mode_ == DriveMode::AUTO) {
                enter_aruco_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: ARUCO");
            } else {
                enter_manual_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: MANUAL");
            }
        }
        last_option = OPTION;

        if (drive_mode_ != DriveMode::MANUAL) {
            return;
        }

        float LS_X = -msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -msg->axes[3];
        float R2 = (-msg->axes[5] + 1) / 2;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        if (fabsf(LS_X) < deadzone)
            LS_X = 0;
        if (fabsf(LS_Y) < deadzone)
            LS_Y = 0;
        if (fabsf(RS_X) < deadzone)
            RS_X = 0;

        float vx = -LS_Y * R2;
        float vy = LS_X * R2;
        float wz = RS_X * sp_yaw;

        // SequenceCtrl と同じ手動メカナム計算
        v1 = vx + vy + wz;
        v3 = vx - vy - wz;
        v4 = vx - vy + wz;
        v2 = vx + vy - wz;

        v3 *= -1;
        v2 *= -1;

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

        float max_v = std::max(
            std::max(fabsf(v1), fabsf(v2)),
            std::max(fabsf(v3), fabsf(v4)));

        if (max_v < 1.0f)
            max_v = 1.0f;

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));
    }
    // 制御ループ
    void publish_timer() {
        if (drive_mode_ == DriveMode::MANUAL) {
            std_msgs::msg::Int16MultiArray msg;
            msg.data = pkt.toVector();
            publisher_->publish(msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50,
                                 "[MANUAL] X: %.2f Y: %.2f Yaw: %.2f | v1 %.2f v2 %.2f v3 %.2f v4 %.2f",
                                 X_, Y_, yaw_, v1, v2, v3, v4);
            return;
        }

        if (drive_mode_ == DriveMode::ARUCO) {
            const float now_sec = static_cast<float>(this->get_clock()->now().seconds());
            const float last_update_sec = static_cast<float>(last_aruco_update_.seconds());
            const float age_sec = now_sec - last_update_sec;

            if (!has_aruco_pose_ || age_sec > aruco_pose_timeout_sec_) {
                stop_motors();

                std_msgs::msg::Int16MultiArray msg;
                msg.data = pkt.toVector();
                publisher_->publish(msg);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "[ARUCO] waiting for marker pose (age=%.2f s)",
                                     age_sec);
                return;
            }

            // OpenCVのカメラ座標系: x=右, y=下, z=前
            // カメラは機体左方向を向いている前提:
            //   camera +x(右) -> 機体 +X(前方)
            //   camera +z(前) -> 機体 +Y(左方)
            // 横方向合わせのみ行う（depth/yawは不使用）
            const float measured_forward = aruco_x_;

            // 目標はカメラ基準の横位置だけを合わせる
            const float target_forward = aruco_target_forward_;
            const float forward_error = target_forward - measured_forward;

            if (std::fabs(forward_error) < aruco_horizontal_deadband_m_) {
                vx_ = 0.0f;
                vy_ = 0.0f;
                wz_ = 0.0f;
            } else {
                vx_ = 0.0f;
                vy_ = std::clamp(aruco_k_vy_ * forward_error, -aruco_max_vy_, aruco_max_vy_);
                wz_ = 0.0f;
            }

            v1 = vy_ + vx_ + wz_;
            v3 = vy_ - vx_ - wz_;
            v4 = vy_ - vx_ + wz_;
            v2 = vy_ + vx_ - wz_;

            v3 *= -1;
            v2 *= -1;

            float max_v = std::max({fabs(v1), fabs(v2), fabs(v3), fabs(v4)});
            if (max_v < 1.0f)
                max_v = 1.0f;

            v1 /= max_v;
            v2 /= max_v;
            v3 /= max_v;
            v4 /= max_v;

            pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
            pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
            pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
            pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));

            std_msgs::msg::Int16MultiArray msg;
            msg.data = pkt.toVector();
            publisher_->publish(msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50,
                                 "[ARUCO] X: %.2f Y: %.2f Z: %.2f | TargetHorizontal: %.2f | vy %.2f",
                                 aruco_x_, aruco_y_, aruco_z_,
                                 aruco_target_forward_,
                                 vy_);
            return;
        }

        if (drive_mode_ == DriveMode::MFF) {
            // MFF モード時は、mff_turn_cmd で指定された角度に旋回
            // Yaw 制御のみ、XY は停止
            const float dt = PUBLISH_RATE_MS / 1000.0f;
            const float max_wz = 1.0f;
            const float yaw_deadband_rad = 2.0f * static_cast<float>(M_PI) / 180.0f;

            vx_ = 0.0f;
            vy_ = 0.0f;
            const float yaw_err = normalize_angle_rad(target_yaw_ - yaw_);
            if (std::fabs(yaw_err) <= yaw_deadband_rad) {
                wz_ = 0.0f;
            } else {
                // AUTO/PLANE と同じ符号系に合わせる
                wz_ = -pid_yaw_.update(yaw_, dt);
                wz_ = std::clamp(wz_, -max_wz, max_wz);
            }

            v1 = vy_ + vx_ + wz_;
            v3 = vy_ - vx_ - wz_;
            v4 = vy_ - vx_ + wz_;
            v2 = vy_ + vx_ - wz_;

            v3 *= -1;
            v2 *= -1;

            float max_v = std::max({fabs(v1), fabs(v2), fabs(v3), fabs(v4)});
            if (max_v < 1.0f)
                max_v = 1.0f;

            v1 /= max_v;
            v2 /= max_v;
            v3 /= max_v;
            v4 /= max_v;

            pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
            pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
            pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
            pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));

            std_msgs::msg::Int16MultiArray msg;
            msg.data = pkt.toVector();
            publisher_->publish(msg);
            return;
        }

        const float dt = PUBLISH_RATE_MS / 1000.0f;

        // PID計算
        vx_ = pid_x_.update(X_, dt);
        vy_ = -pid_y_.update(Y_, dt); // Y軸反転（座標系による）
        wz_ = -pid_yaw_.update(yaw_, dt);

        float cos_yaw = cos(-yaw_);
        float sin_yaw = sin(-yaw_);

        // オドメトリのデータをロボット座標系に変換
        float vx_robot = cos_yaw * vx_ + sin_yaw * vy_;
        float vy_robot = -sin_yaw * vx_ + cos_yaw * vy_;

        v1 = vy_robot + vx_robot + wz_;
        v3 = vy_robot - vx_robot - wz_;
        v4 = vy_robot - vx_robot + wz_;
        v2 = vy_robot + vx_robot - wz_;
        // メカナム逆運動学
        // v1 = vy_ + vx_ + wz_;
        // v3 = vy_ - vx_ - wz_;
        // v4 = vy_ - vx_ + wz_;
        // v2 = vy_ + vx_ - wz_;

        // 右側モータ反転
        v3 *= -1;
        v2 *= -1;

        // 正規化
        float max_v = std::max({fabs(v1), fabs(v2), fabs(v3), fabs(v4)});
        if (max_v < 1.0)
            max_v = 1.0;

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        // 送信
        pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));

        std_msgs::msg::Int16MultiArray msg;
        msg.data = pkt.toVector();
        publisher_->publish(msg);

        const char *mode_tag = drive_mode_ == DriveMode::PLANE ? "PLANE" : "AUTO";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50,
                             "[%s] X: %.2f Y: %.2f Yaw: %.2f | T: %.2f %.2f %.2f | vx %.2f vy %.2f wz %.2f",
                             mode_tag, X_, Y_, yaw_, target_x_, target_y_, target_yaw_, vx_, vy_, wz_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDMecanumController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}