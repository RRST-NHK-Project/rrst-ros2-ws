/*
RRST NHK2025
PID on ROS
PID制御用にエンコーダーの速度、変位を受信してPublish
いろいろやってみたけど無理そうなので凍結
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <arpa/inet.h>
#include <cstring>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#define UDP_PORT 4001   // 受信ポート
#define BUFFER_SIZE 128 // 受信バッファサイズ

class UDPReceiver : public rclcpp::Node {
public:
    UDPReceiver() : Node("nr25_enc_pid") {
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("enc_pid", 10);

        // UDP ソケットの設定
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        struct sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(UDP_PORT);

        if (bind(udp_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            close(udp_socket_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "UDP Receiver started on port %d", UDP_PORT);

        // 受信処理を非同期に実行
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&UDPReceiver::receive_udp_data, this));
    }

    ~UDPReceiver() {
        if (udp_socket_ >= 0) {
            close(udp_socket_);
        }
    }

private:
    int udp_socket_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void receive_udp_data() {
        struct sockaddr_in sender_addr;
        socklen_t addr_len = sizeof(sender_addr);
        char buffer[BUFFER_SIZE];

        // UDPデータ受信
        ssize_t len = recvfrom(udp_socket_, buffer, sizeof(buffer) - 1, 0,
                               (struct sockaddr *)&sender_addr, &addr_len);

        if (len < 0) {
            RCLCPP_ERROR(this->get_logger(), "recvfrom failed: %s", strerror(errno));
            return;
        }

        buffer[len] = '\0'; // 文字列の終端を設定
        RCLCPP_INFO(this->get_logger(), "Received raw: %s", buffer);
        // RCLCPP_INFO(this->get_logger(), "Received %ld bytes", len);

        // カンマ区切りのデータをパース
        std_msgs::msg::Float32MultiArray msg;
        std::vector<float> data = parse_udp_data(buffer);
        msg.data = data;

        // データをPublish
        publisher_->publish(msg);
    }

    std::vector<float> parse_udp_data(const std::string &data_str) {
        std::vector<float> values;
        std::stringstream ss(data_str);
        std::string token;

        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stof(token));
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Invalid data: %s", token.c_str());
            }
        }
        return values;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
