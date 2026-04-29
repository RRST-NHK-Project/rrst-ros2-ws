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

        // AUTO目標到達完了フラッグ
        autodrive_complete_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "r2/autodrive_complete", rclcpp::QoS(10));

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
            "r2_drive_mode_cmd", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&PIDMecanumController::mode_cmd_callback, this, std::placeholders::_1));

        transition_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "r2/task_transition_mode", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local(),
            std::bind(&PIDMecanumController::transition_mode_callback, this, std::placeholders::_1));

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
        this->declare_parameter("auto_complete_pos_thresh_m", 0.05);
        this->declare_parameter("auto_complete_yaw_thresh_rad", 0.10);
        this->declare_parameter("auto_complete_hold_sec", 0.30);
        aruco_target_forward_ = static_cast<float>(this->get_parameter("aruco_target_forward_m").as_double());
        aruco_target_lateral_ = static_cast<float>(this->get_parameter("aruco_target_lateral_m").as_double());
        aruco_target_yaw_ = static_cast<float>(this->get_parameter("aruco_target_yaw_rad").as_double());
        aruco_target_id_ = this->get_parameter("aruco_target_id").as_int();
        aruco_camera_offset_x_ = static_cast<float>(this->get_parameter("aruco_camera_offset_x_m").as_double());
        aruco_camera_offset_y_ = static_cast<float>(this->get_parameter("aruco_camera_offset_y_m").as_double());
        auto_complete_pos_thresh_m_ = static_cast<float>(this->get_parameter("auto_complete_pos_thresh_m").as_double());
        auto_complete_yaw_thresh_rad_ = static_cast<float>(this->get_parameter("auto_complete_yaw_thresh_rad").as_double());
        auto_complete_hold_sec_ = static_cast<float>(this->get_parameter("auto_complete_hold_sec").as_double());
        last_aruco_update_ = this->get_clock()->now();
        auto_goal_in_range_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        reset_auto_complete_tracking(true);
        publish_mode();
    }

private:
    enum class DriveMode {
        MANUAL,
        AUTO,
        ARUCO,
        PLANE,
        MFF,
        ARENA,
    };

    static constexpr int32_t TRANSITION_MODE_MANUAL = 0;
    static constexpr int32_t TRANSITION_MODE_STATE_MANAGEMENT = 1;

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr transition_mode_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr aruco_distance_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_id_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_target_id_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr aruco_camera_offset_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autodrive_complete_pub_;
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
    int32_t transition_mode_code_ = TRANSITION_MODE_MANUAL;
    int32_t deferred_mode_code_ = -1;

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

    // AUTO/PLANE 到達完了判定
    float auto_complete_pos_thresh_m_ = 0.05f;
    float auto_complete_yaw_thresh_rad_ = 0.10f;
    float auto_complete_hold_sec_ = 0.30f;
    bool auto_goal_in_range_ = false;
    bool auto_complete_published_ = false;
    rclcpp::Time auto_goal_in_range_since_;

    static float normalize_angle_rad(float angle) {
        while (angle > static_cast<float>(M_PI))
            angle -= static_cast<float>(2.0 * M_PI);
        while (angle < static_cast<float>(-M_PI))
            angle += static_cast<float>(2.0 * M_PI);
        return angle;
    }

    void reset_auto_complete_tracking(bool publish_false) {
        auto_goal_in_range_ = false;
        auto_complete_published_ = false;
        auto_goal_in_range_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

        if (publish_false && autodrive_complete_pub_) {
            std_msgs::msg::Bool msg;
            msg.data = false;
            autodrive_complete_pub_->publish(msg);
        }
    }

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
        case DriveMode::ARENA:
            return "ARENA";
        }
        return "MANUAL";
    }

    static const char *transition_mode_to_string(int32_t mode) {
        return mode == TRANSITION_MODE_MANUAL ? "MANUAL" : "STATE_MANAGEMENT";
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

    void publish_packet() {
        std_msgs::msg::Int16MultiArray msg;
        msg.data = pkt.toVector();
        publisher_->publish(msg);
    }

    void reset_pose_target_control(bool reset_auto_complete) {
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

        if (reset_auto_complete) {
            reset_auto_complete_tracking(false);
        }
    }

    void switch_drive_mode(DriveMode mode, bool publish_stop_packet = false) {
        drive_mode_ = mode;
        stop_motors();
        publish_mode();

        if (publish_stop_packet) {
            publish_packet();
        }
    }

    void apply_wheel_outputs(float wheel_1, float wheel_2, float wheel_3, float wheel_4) {
        v1 = wheel_1;
        v2 = wheel_2;
        v3 = wheel_3;
        v4 = wheel_4;

        float max_v = std::max({fabsf(v1), fabsf(v2), fabsf(v3), fabsf(v4)});
        if (max_v < 1.0f) {
            max_v = 1.0f;
        }

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));
    }

    void enter_auto_mode() {
        reset_pose_target_control(true);
        switch_drive_mode(DriveMode::AUTO);
    }

    void enter_aruco_mode() {
        switch_drive_mode(DriveMode::ARUCO);
    }

    void enter_manual_mode() {
        switch_drive_mode(DriveMode::MANUAL);
    }

    void enter_plane_mode() {
        reset_pose_target_control(true);
        switch_drive_mode(DriveMode::PLANE);
    }

    void enter_mff_mode() {
        // MFF中はSequenceCtrlを主系にするため、停止パケットを1回だけ送る。
        switch_drive_mode(DriveMode::MFF, true);
    }

    void enter_arena_mode() {
        reset_pose_target_control(false);
        switch_drive_mode(DriveMode::ARENA);
    }

    bool apply_mode_code(int32_t mode_code, const char *source) {
        switch (mode_code) {
        case 0:
            if (drive_mode_ != DriveMode::MANUAL) {
                enter_manual_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: MANUAL (%s)", source);
            }
            return true;
        case 1:
            if (drive_mode_ != DriveMode::AUTO) {
                enter_auto_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: AUTO (%s)", source);
            }
            return true;
        case 2:
            if (drive_mode_ != DriveMode::ARUCO) {
                enter_aruco_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: ARUCO (%s)", source);
            }
            return true;
        case 3:
            if (drive_mode_ != DriveMode::PLANE) {
                enter_plane_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: PLANE (%s)", source);
            }
            return true;
        case 4:
            if (drive_mode_ != DriveMode::MFF) {
                enter_mff_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: MFF (%s)", source);
            }
            return true;
        case 5:
            if (drive_mode_ != DriveMode::ARENA) {
                enter_arena_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: ARENA (%s)", source);
            }
            return true;
        default:
            return false;
        }
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

        target_x_ = msg->data[0];
        target_y_ = msg->data[1];
        target_yaw_ = msg->data[2];
        has_target_cmd_ = true;
        reset_auto_complete_tracking(true);
        reset_pose_target_control(false);

        if (transition_mode_code_ == TRANSITION_MODE_MANUAL) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Deferred target command while transition mode is MANUAL");
            RCLCPP_INFO(this->get_logger(),
                        "Target updated x=%.3f y=%.3f yaw=%.3f [rad]",
                        target_x_, target_y_, target_yaw_);
            return;
        }

        if (drive_mode_ != DriveMode::AUTO && drive_mode_ != DriveMode::ARENA) {
            switch_drive_mode(DriveMode::AUTO);
            RCLCPP_INFO(this->get_logger(), "Mode changed: AUTO (by target command)");
        }

        RCLCPP_INFO(this->get_logger(),
                    "Target updated x=%.3f y=%.3f yaw=%.3f [rad]",
                    target_x_, target_y_, target_yaw_);
    }

    void mode_cmd_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.empty()) {
            return;
        }

        int32_t mode_code = msg->data[0];
        if (mode_code < 0 || mode_code > 5) {
            RCLCPP_WARN(this->get_logger(), "Invalid mode code: %ld (valid: 0-5)", static_cast<long>(mode_code));
            return;
        }

        if (transition_mode_code_ == TRANSITION_MODE_MANUAL && mode_code != 0) {
            deferred_mode_code_ = mode_code;
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Deferred mode command %ld while transition mode is MANUAL",
                static_cast<long>(mode_code));
            return;
        }

        if (mode_code == 0) {
            deferred_mode_code_ = -1;
        }
        apply_mode_code(mode_code, "by mode command");
    }

    void transition_mode_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        const int32_t next_mode = msg->data;
        if (next_mode != TRANSITION_MODE_MANUAL && next_mode != TRANSITION_MODE_STATE_MANAGEMENT) {
            return;
        }

        if (transition_mode_code_ == next_mode) {
            return;
        }

        transition_mode_code_ = next_mode;
        RCLCPP_INFO(
            this->get_logger(),
            "Transition mode changed: %s (current drive mode: %s)",
            transition_mode_to_string(transition_mode_code_),
            drive_mode_to_string(drive_mode_).c_str());

        if (transition_mode_code_ == TRANSITION_MODE_MANUAL && drive_mode_ != DriveMode::MANUAL) {
            enter_manual_mode();
            RCLCPP_INFO(this->get_logger(), "Mode changed: MANUAL (by transition mode)");
            return;
        }

        if (transition_mode_code_ == TRANSITION_MODE_STATE_MANAGEMENT) {
            if (deferred_mode_code_ >= 0) {
                const int32_t deferred_mode = deferred_mode_code_;
                deferred_mode_code_ = -1;
                apply_mode_code(deferred_mode, "by deferred mode command");
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "No deferred mode command. Drive mode remains %s.",
                    drive_mode_to_string(drive_mode_).c_str());
            }
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

    // PS4入力
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 8 || msg->buttons.size() < 10)
            return;

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

        apply_wheel_outputs(v1, v2, v3, v4);
    }
    // 制御ループ
    void publish_timer() {
        if (drive_mode_ == DriveMode::MANUAL) {
            publish_packet();
            return;
        }

        if (drive_mode_ == DriveMode::ARUCO) {
            const float now_sec = static_cast<float>(this->get_clock()->now().seconds());
            const float last_update_sec = static_cast<float>(last_aruco_update_.seconds());
            const float age_sec = now_sec - last_update_sec;

            if (!has_aruco_pose_ || age_sec > aruco_pose_timeout_sec_) {
                stop_motors();

                publish_packet();
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

            apply_wheel_outputs(v1, v2, v3, v4);

            publish_packet();

            return;
        }

        if (drive_mode_ == DriveMode::MFF) {
            return;
        }

        if (drive_mode_ == DriveMode::ARENA) {
            // ARENAモード中はSequenceCtrlが主系のため制御しない
            return;
        }

        const float dt = PUBLISH_RATE_MS / 1000.0f;

        // 目標到達判定（AUTO/PLANE）
        const float pos_error = std::hypot(target_x_ - X_, target_y_ - Y_);
        const float yaw_error = std::fabs(normalize_angle_rad(target_yaw_ - yaw_));
        const bool in_goal_range =
            pos_error <= auto_complete_pos_thresh_m_ &&
            yaw_error <= auto_complete_yaw_thresh_rad_;

        if (in_goal_range) {
            if (!auto_goal_in_range_) {
                auto_goal_in_range_ = true;
                auto_goal_in_range_since_ = this->get_clock()->now();
            }

            const double in_range_sec =
                (this->get_clock()->now() - auto_goal_in_range_since_).seconds();

            if (in_range_sec >= static_cast<double>(auto_complete_hold_sec_)) {
                if (!auto_complete_published_) {
                    std_msgs::msg::Bool complete_msg;
                    complete_msg.data = true;
                    autodrive_complete_pub_->publish(complete_msg);
                    auto_complete_published_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "AUTO target reached. Published r2/autodrive_complete=true (pos_err=%.3f m, yaw_err=%.3f rad)",
                                pos_error, yaw_error);
                }

                stop_motors();
                publish_packet();
                return;
            }
        } else {
            auto_goal_in_range_ = false;
            auto_goal_in_range_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }

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

        apply_wheel_outputs(v1, v2, v3, v4);

        publish_packet();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDMecanumController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}