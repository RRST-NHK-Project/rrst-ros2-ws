/*
RRST NHK2025
UDPで送信するクラス
*/

#include "UDP_vector_int.hpp"

int max = 9999; // 最大値を4桁に制限

UDP_vector_int::UDP_vector_int(const std::string &ip_address, int port) {
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

void UDP_vector_int::send(std::vector<int> &data) {
    for (auto &value : data) {
        if (value > max) {
            value = max;
        } else if (value < -max) {
            value = -max;
        }
    }

    // int配列をそのまま送信
    ssize_t sent_bytes = sendto(
        udp_socket,
        data.data(),               // 送信データのポインタ
        data.size() * sizeof(int), // バイトサイズ
        0,
        (struct sockaddr *)&dst_addr,
        sizeof(dst_addr));

    if (sent_bytes < 0) {
        std::cerr << "Failed to send data." << std::endl;
    }
}
