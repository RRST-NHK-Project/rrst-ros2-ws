/*
RRST NHK2025
UDPで送信するクラス
*/

#include "UDP.hpp"

int max = 999; // 最大値を3桁に制限

UDP::UDP(const std::string &ip_address, int port) {
    try {
        udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket < 0) {
            throw std::runtime_error("Failed to create socket.");
        }

        memset(&dst_addr, 0, sizeof(dst_addr));
        dst_addr.sin_family = AF_INET;
        dst_addr.sin_port = htons(port);
        if (inet_pton(AF_INET, ip_address.c_str(), &dst_addr.sin_addr) <= 0) {
            throw std::runtime_error("Invalid IP address: " + ip_address);
        }

        memset(&src_addr, 0, sizeof(src_addr));
        src_addr.sin_family = AF_INET;
        src_addr.sin_port = 0;
    } catch (const std::exception &e) {
        std::cerr << "UDP initialization error: " << e.what() << std::endl;
    }
}

void UDP::send(std::vector<int> &data) {
    for (auto &value : data) {
        if (value > max) {
            value = max;
        } else if (value < -max) {
            value = -max;
        }
    }

    // カンマ区切りで文字列に変換
    std::ostringstream oss;
    for (size_t i = 1; i < data.size(); ++i) {
        oss << data[i];
        if (i != data.size() - 1) {
            oss << ",";
        }
    }

    std::string str_data = oss.str();
    if (sendto(udp_socket, str_data.c_str(), str_data.length(), 0, (struct sockaddr *)&dst_addr, sizeof(dst_addr)) < 0) {
        std::cerr << "Failed to send data." << std::endl;
    }

    // デバッグ用
    //std::cerr << str_data << std::endl;

}
