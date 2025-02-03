#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <boost/asio.hpp>
#include <array>
#include <string>
#include <vector>
#include <sstream>

class ENC_OBS : public rclcpp::Node {
public:
    ENC_OBS() : Node("enc_obs"), socket_(io_service_) {
        // パブリッシャーの作成
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("enc", 10);

        // UDPソケットの初期化
        boost::asio::ip::udp::endpoint local_endpoint(boost::asio::ip::address::from_string("192.168.8.233"), 4000);
        socket_.open(local_endpoint.protocol());
        socket_.bind(local_endpoint);

        RCLCPP_INFO(this->get_logger(), "ENC observer started. IP: %s", local_endpoint.address().to_string().c_str());

        // 受信バッファの確保
        start_receive();

        // 10msごとにデータをPublish
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ENC_OBS::publish_enc_data, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint remote_endpoint_;
    std::array<char, 64> recv_buffer_;
    
    std::vector<float> enc_data_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // エンコーダーデータ

    void start_receive() {
        socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            [this](boost::system::error_code ec, std::size_t bytes_recvd) {
                if (!ec && bytes_recvd > 0) {
                    process_received_data(bytes_recvd);
                }
                start_receive();  // 次の受信を開始
            }
        );
    }

    void process_received_data(std::size_t bytes_recvd) {
        // 受信したデータを文字列に変換
        std::string received_str(recv_buffer_.data(), bytes_recvd);

        // 受信データをログに表示
        RCLCPP_INFO(this->get_logger(), "Received data: %s", received_str.c_str());

        std::vector<std::string> split_str;
        std::stringstream ss(received_str);
        std::string token;

        // 受信データをカンマで分割
        while (std::getline(ss, token, ',')) {
            split_str.push_back(token);
        }

        // 最初の6要素を float に変換
        for (size_t i = 0; i < std::min(split_str.size(), enc_data_.size()); ++i) {
            try {
                enc_data_[i] = std::stof(split_str[i]);
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Invalid data received: %s", split_str[i].c_str());
                enc_data_[i] = 0.0;  // 変換エラー時は0に
            }
        }
    }

    void publish_enc_data() {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = enc_data_;
        publisher_->publish(msg);
    }
};

// メイン関数
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ENC_OBS>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
