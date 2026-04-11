/*
R2ハンド制御
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// まだ未確認なので絶対に許可なしに起動しないこと！！
// 壊しても筆者は責任を負いません！！
// リーダブルコードを心がけていますが、変数名や関数名が分かりにくい場合は遠慮なく質問してください。
// ラムダ式を多用しています。

#include <chrono>  // 時間管理
#include <memory>  //ポインタ用
#include <vector> //動的配列 std::vector を使うため
#include "std_msgs/msg/int8.hpp"            
#include "std_msgs/msg/int16_multi_array.hpp" 

using namespace std::chrono_literals;

class R2_HandCtrl : public rclcpp::Node { //rclcpp::Nodeを継承してノードを作成
    public:
    R2_HandCtrl() : Node("r2_hand_ctrl_node") {

        tx_data_.assign(24, 0); // マイコンへの送信データを0で初期化

        // GUIからノードを受け取りたい（未実装）

        // マイコンへのデータ送信
        publisher_ = create_publisher<std_msgs::msg::Int16MultiArray>("serial_tx_2", 10);

        // 50ms周期で送信タイマーを回す（番兵用）(C++だとselfの代わりにthisポインタを第一引数に使う)
        timer_ = create_wall_timer(50ms, [this]() {
            on_timer_tick();
        });

        RCLCPP_INFO(get_logger(), "R2 Hand Control Node Started.");
    }

    private:
    void on_command_received(const std_msgs::msg::Int8::SharedPtr msg) {
        // コマンドに応じてtx_data_を更新する処理
        // 例: if (msg->data == 1) { tx_data_[0] = 100; } // コマンド1でモーター1を100に設定
    }

    void on_timer_tick() {
        // 定期的にマイコンへデータを送信する処理
        std_msgs::msg::Int16MultiArray msg;
        msg.data = tx_data_;
        publisher_->publish(msg);
    }

    std::vector<int16_t> tx_data_; // マイコンへ送信するデータを格納するベクター（可変長配列用）
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv); // ROS2の初期化
    auto hand_control_node = std::make_shared<R2HandControl>();// ノードのインスタンス作成
    rclcpp::spin(hand_control_node); // ノードをスピンしてコールバックを処理
    rclcpp::shutdown(); // 処理終了

    return 0;
}