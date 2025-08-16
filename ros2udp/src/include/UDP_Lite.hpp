#ifndef UDP_Lite_HPP
#define UDP_Lite_HPP

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

class UDP_Lite {
public:
    UDP_Lite(const std::string &ip_address, int port);
    ~UDP_Lite();

    void send(const std::vector<int16_t> &data); // 引数を int16_t に変更

private:
    int udp_socket;
    struct sockaddr_in dst_addr;
    const int16_t max = 32767; // 16ビットの最大値（32767）
};

#endif // UDP_LITE_HPP
