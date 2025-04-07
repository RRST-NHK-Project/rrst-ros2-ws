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
#include <std_msgs/msg/int32_multi_array.hpp>

float v[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float d[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float deg[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

int servo_deg;

/*
この座標で統一します
            　↑
            　↑
   v_fl---------------------v_fr
    |                       |
    |                       |
    |                       |
    |                       |　　　　　　　　　　　　　　　　y　　　　　
    |                       |                                ↑
    |                       |                                |
    |                       |                                |  theta
   v_rl---------------------v_rr                             |--------->x

すべての車輪について速度、変位をPublish

*/

/*
mbed側から引用
速度、変位の計算部分を貼っておきます

float Pulse[6]; // エンコーダーのパルス格納用
float v[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 速度の格納[mm/s]
float d[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 変位[m]

float period = 10; // 制御周期[ms]
float R = 80;      // オムニ直径[mm]
int PPRx4 = 8192;  // エンコーダーのResolution

〜　中略　〜

v[1] = Pulse[1] * (R * PI / PPRx4) *
        (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算
v[2] = Pulse[2] * (R * PI / PPRx4) *
        (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算
v[3] = Pulse[3] * (R * PI / PPRx4) *
        (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算
v[4] = Pulse[4] * (R * PI / PPRx4) *
        (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算

d[1] += Pulse[1] * R * PI / PPRx4 / 1000; //変位[m]
d[2] += Pulse[2] * R * PI / PPRx4 / 1000; //変位[m]
d[3] += Pulse[3] * R * PI / PPRx4 / 1000; //変位[m]
d[4] += Pulse[4] * R * PI / PPRx4 / 1000; //変位[m]

*/

// 別ノードにてUDP受信した速度、変位をとってくる
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
        v[1] = msg->data[0];
        v[2] = msg->data[1];
        v[3] = msg->data[2];
        v[4] = msg->data[3];
        d[1] = msg->data[4];
        d[2] = msg->data[5];
        d[3] = msg->data[6];
        d[4] = msg->data[7];
        // std::cout << v1 << " " << v2 << " " << v3 << " " << v4 <<std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

class Servo_Deg_Subscriber : public rclcpp::Node {
public:
    Servo_Deg_Subscriber()
        : Node("Servo_Deg_Subscriber") {
        // Subscriberの作成
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "mr_servo_deg", 10,
            std::bind(&Servo_Deg_Subscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // メッセージを受け取って表示
        // RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->data);
        deg[1] = msg->data[0];
        deg[2] = msg->data[1];
        deg[3] = msg->data[2];
        deg[4] = msg->data[3];
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

// JointState型に格納しPublish
class JointStatePublisher : public rclcpp::Node {
public:
    JointStatePublisher() : Node("mr_joint_state_publisher") {
        // Publisherの作成
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("mr_joint_states", 10);

        // タイマー
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
        msg.name = {"fl", "fr", "rl", "rr", "fl_servo", "fr_servo", "rl_servo", "rr_servo"};

        // 変位[m]
        msg.position = {d[1], d[2], d[3], d[4], deg[1], deg[2], deg[3], deg[4]};

        // 速度[mm/s]
        msg.velocity = {v[1], v[2], v[3], v[4], 0.0, 0.0, 0.0, 0.0};

        // なし
        msg.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Publish
        joint_state_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
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
