#ifndef UDP_HPP
#define UDP_HPP

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <vector>

class UDP {
public:
    UDP(const std::string &ip_address, int port);
    void send(std::vector<int> &data);

private:
    int udp_socket;
    struct sockaddr_in src_addr, dst_addr;
};

#endif // UDP_HPP
