// ak_gamepad_node.cpp
// PS4 コントローラ (/joy) で CubeMars AK を操作する ROS2 ノード。
// ros2can (serial_bridge) の serial_tx_<ID> / serial_rx_<ID> トピック経由で、
// xiao-esp32-s3_can2io (MODE_CUBEMARS) に指令を送る。
//
// 起動例:
//   ros2 run joy joy_node
//   ros2 run <ros2can のノード> ...        (serial_tx_11 / serial_rx_11 を橋渡しするもの)
//   ros2 run AKmode ak_gamepad_node --ros-args -p device_id:=11 -p motor:=1
//
// 操作 (PS4):
//   L1 : 押している間だけ動作 (デッドマン)。離すと 速度=減速停止 / 位置=その位置を保持
//   左スティック上下 : 速度モード=速度 / 位置モード=目標位置を動かす
//   ×  : 速度モード     ○ : 位置モード (現在位置を目標に取り込んで開始)
//   △  : 位置モード中に目標を現在位置へ再同期
//   十字キー上/下 : 位置モード中、目標を ±step_deg (L1 を押しながら)

#include <atomic>
#include <chrono>
#include <thread>

#include "ak_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace {
// PS4 + joy_node の標準的な割当 (ros2 の他のノードと同じ)
constexpr int kAxisLeftY = 1;
constexpr int kAxisDpadY = 7;
constexpr int kBtnCross = 0;
constexpr int kBtnCircle = 1;
constexpr int kBtnTriangle = 2;
constexpr int kBtnL1 = 4;
}  // namespace

class AkGamepadNode : public rclcpp::Node {
public:
    AkGamepadNode() : Node("ak_gamepad_node"), ctrl_(makeParams()) {
        const int id = static_cast<int>(get_parameter("device_id").as_int());
        const int period_ms = static_cast<int>(get_parameter("publish_period_ms").as_int());
        joy_timeout_s_ = getNum("joy_timeout_s");
        period_s_ = period_ms / 1000.0;

        const std::string tx_topic = "serial_tx_" + std::to_string(id);
        const std::string rx_topic = "serial_rx_" + std::to_string(id);

        pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(tx_topic, 10);
        fb_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
            rx_topic, 10, std::bind(&AkGamepadNode::feedbackCallback, this, std::placeholders::_1));
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&AkGamepadNode::joyCallback, this, std::placeholders::_1));
        timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                                   std::bind(&AkGamepadNode::timerCallback, this));

        // Ctrl+C でコンテキストが閉じる前にゼロ指令を送る
        // (ファームは最後に受信した指令を保持し続けるため)
        rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
            [this]() { sendZeroOnShutdown(); });

        RCLCPP_INFO(get_logger(), "tx: %s  rx: %s  motor: %d", tx_topic.c_str(), rx_topic.c_str(),
                    static_cast<int>(get_parameter("motor").as_int()));
        RCLCPP_INFO(get_logger(), "L1:デッドマン ×:速度 ○:位置 △:再同期 十字上下:目標±step");
    }

private:
    // -p max_erpm:=500 (整数) でも 500.0 (小数) でも受け付ける
    void declareNum(const std::string& name, double def) {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.dynamic_typing = true;
        declare_parameter(name, rclcpp::ParameterValue(def), desc);
    }
    double getNum(const std::string& name) {
        const rclcpp::Parameter p = get_parameter(name);
        return p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                   ? static_cast<double>(p.as_int())
                   : p.as_double();
    }

    ak::ControlParams makeParams() {
        declare_parameter<int>("device_id", 11);       // config.hpp の DEVICE_ID
        declare_parameter<int>("motor", 1);             // 1-4
        declareNum("max_erpm", 3000.0);
        declareNum("accel_erpm", 6000.0);
        declareNum("max_dps", 90.0);
        declareNum("step_deg", 90.0);
        declareNum("deadzone", 0.1);
        declare_parameter<int>("publish_period_ms", 10);
        declareNum("joy_timeout_s", 0.5);  // 0 以下で無効

        ak::ControlParams p;
        const int motor = static_cast<int>(get_parameter("motor").as_int());
        p.motor_index = std::min(std::max(motor, 1), 4) - 1;
        p.max_erpm = getNum("max_erpm");
        p.accel_erpm = getNum("accel_erpm");
        p.max_dps = getNum("max_dps");
        p.step_deg = getNum("step_deg");
        p.deadzone = getNum("deadzone");
        return p;
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() <= static_cast<size_t>(kAxisDpadY) ||
            msg->buttons.size() <= static_cast<size_t>(kBtnL1)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "joy の軸/ボタン数が足りません (axes=%zu buttons=%zu)",
                                 msg->axes.size(), msg->buttons.size());
            return;
        }
        last_joy_ = std::chrono::steady_clock::now();
        joy_seen_ = true;

        ctrl_.setStick(msg->axes[kAxisLeftY]);   // joy_node は上が +1
        ctrl_.setDeadman(msg->buttons[kBtnL1] != 0);

        const bool cross = msg->buttons[kBtnCross] != 0;
        const bool circle = msg->buttons[kBtnCircle] != 0;
        const bool triangle = msg->buttons[kBtnTriangle] != 0;
        const bool up = msg->axes[kAxisDpadY] > 0.5;
        const bool down = msg->axes[kAxisDpadY] < -0.5;

        if (cross && !last_cross_) ctrl_.selectVelocity();
        if (circle && !last_circle_ && !ctrl_.selectPosition()) {
            RCLCPP_WARN(get_logger(), "フィードバック未受信のため位置モードに入れません (%s を確認)",
                        ("serial_rx_" + std::to_string(get_parameter("device_id").as_int())).c_str());
        }
        if (triangle && !last_triangle_) ctrl_.syncTarget();
        if (up && !last_up_) ctrl_.stepTarget(+1);
        if (down && !last_down_) ctrl_.stepTarget(-1);

        last_cross_ = cross;
        last_circle_ = circle;
        last_triangle_ = triangle;
        last_up_ = up;
        last_down_ = down;
    }

    void feedbackCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() < static_cast<size_t>(ak::kSlots)) return;
        ctrl_.setFeedback(msg->data.data());
    }

    void timerCallback() {
        if (shutting_down_) return;

        // joy が途絶えたら停止側に倒す (joy_node が落ちた/USB 抜けなど)
        if (joy_timeout_s_ > 0.0) {
            const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_joy_).count();
            if (!joy_seen_ || age > joy_timeout_s_) {
                ctrl_.setStick(0.0);
                ctrl_.setDeadman(false);
                if (joy_seen_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joy が %.1f 秒以上途絶えています。停止します", age);
                }
            }
        }

        std::array<int16_t, ak::kSlots> out{};
        ctrl_.update(period_s_, out);

        std_msgs::msg::Int16MultiArray msg;
        msg.data.assign(out.begin(), out.end());
        pub_->publish(msg);

        const ak::Feedback& fb = ctrl_.feedback();
        if (ctrl_.hasFeedback()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                 "[%s]%s cmd=%.1f | pos=%.1fdeg spd=%.0fERPM cur=%.2fA T=%dC err=%d",
                                 ctrl_.mode() == ak::Mode::Velocity ? "VEL" : "POS",
                                 ctrl_.deadman() ? "*" : " ",
                                 ctrl_.mode() == ak::Mode::Velocity ? ctrl_.velCmdErpm() : ctrl_.targetDeg(),
                                 fb.pos_deg, fb.speed_erpm, fb.current_a, fb.temp_c, fb.error);
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "フィードバック未受信 (serial_rx を待機中)");
        }
    }

    void sendZeroOnShutdown() {
        shutting_down_ = true;
        std_msgs::msg::Int16MultiArray zero;
        zero.data.assign(ak::kSlots, 0);
        for (int i = 0; i < 5; ++i) {
            pub_->publish(zero);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    ak::Controller ctrl_;
    double joy_timeout_s_ = 0.5;
    double period_s_ = 0.01;
    bool joy_seen_ = false;
    std::chrono::steady_clock::time_point last_joy_{};
    bool last_cross_ = false, last_circle_ = false, last_triangle_ = false;
    bool last_up_ = false, last_down_ = false;
    std::atomic<bool> shutting_down_{false};

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr fb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AkGamepadNode>());
    rclcpp::shutdown();
    return 0;
}
