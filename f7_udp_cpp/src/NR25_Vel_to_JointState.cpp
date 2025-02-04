/*
RRST NHK2025
Velocity to JointState
*/

// 標準
#include <chrono>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

float v1;
float v2;
float v3;
float v4;

int servo_deg;

class ENC_Listener : public rclcpp::Node {
public:
    ENC_Listener()
        : Node("nhk25_vel2js") {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "enc", 10,
            std::bind(&ENC_Listener::enc_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 MR Vel to JointState");
    }

private:
    void enc_listener_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        v1 = msg->data[0];
        v2 = msg->data[1];
        v3 = msg->data[2];
        v4 = msg->data[3];
        // std::cout << v1 << " " << v2 << " " << v3 << " " << v4 <<std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

class JointStatePublisher : public rclcpp::Node {
public:
    JointStatePublisher() : Node("mr_joint_state_publisher") {
        // Publisherの作成
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("mr_joint_states", 10);

        // タイマ
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&JointStatePublisher::publish_joint_state, this));
    }

private:
    void publish_joint_state() {
        auto msg = sensor_msgs::msg::JointState();

        // ヘッダ情報
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link"; // フレームID

        // ジョイント名
        msg.name = {"Steering_servo_feed", "Steering_enc_actual", "Wheel"}; // ステア(サーボに司令している値), 実装予定：ステア(エンコーダーから計算したリアルタイムの角度), 駆動輪

        // 機体の変位[m]
        msg.position = {float(servo_deg), v3, v4};

        // 速度[mm/s]
        msg.velocity = {0.0, v1, v2};

        // なし
        msg.effort = {0.0, 0.0, 0.0};

        // Publish
        joint_state_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

class Servo_Deg_Subscriber : public rclcpp::Node {
public:
    Servo_Deg_Subscriber()
        : Node("Servo_Deg_Subscriber") {
        // Subscriberの作成
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "mr_servo_deg", 10,
            std::bind(&Servo_Deg_Subscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // メッセージを受け取って表示
        // RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->data);
        servo_deg = msg->data;
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto enc_listener = std::make_shared<ENC_Listener>();
    auto joint_state_publisher = std::make_shared<JointStatePublisher>();
    auto servo_deg_subscriber = std::make_shared<Servo_Deg_Subscriber>();
    exec.add_node(enc_listener);
    exec.add_node(joint_state_publisher);
    exec.add_node(servo_deg_subscriber);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
