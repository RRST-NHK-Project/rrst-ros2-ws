/*
RRST-NHK-Project 2025
UDPで送信するクラス
charからint16_tに変更することで通信量を削減
*/

#include "UDP.hpp"

#include <stdexcept>
#include <unistd.h>

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
    } catch (const std::exception &e) {
        std::cerr << "UDP initialization error: " << e.what() << std::endl;
    }
}

UDP::~UDP() {
    close(udp_socket);
}

void UDP::send(const std::vector<int16_t> &data) {
    std::vector<int16_t> limited_data = data;
    for (auto &value : limited_data) {
        if (value > max) {
            value = max;
        } else if (value < -max) {
            value = -max;
        }
    }

    ssize_t sent_bytes = sendto(
        udp_socket,
        limited_data.data(),
        limited_data.size() * sizeof(int16_t), // サイズも変更
        0,
        (struct sockaddr *)&dst_addr,
        sizeof(dst_addr));

    if (sent_bytes < 0) {
        std::cerr << "Failed to send data: " << strerror(errno) << std::endl;
    }
}
