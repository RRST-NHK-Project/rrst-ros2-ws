#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"
#include "lwip/udp.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <sstream>
#include <stdio.h>
#include <string>
#include <vector>

#define WIFI_SSID "GL-SFT1200-288-2G-LNX"
#define WIFI_PASS "lnxmaster"
#define STATIC_IP "192.168.8.219"
#define STATIC_NETMASK "255.255.255.0"
#define STATIC_GATEWAY "192.168.8.1"
#define UDP_PORT 5000

void toggle_led() {
    static bool led_on = false;
    led_on = !led_on;
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

std::vector<int> parse_csv(const std::string &input) {
    std::vector<int> result;
    std::stringstream ss(input);
    std::string item;
    while (std::getline(ss, item, ',')) {
        try {
            result.push_back(std::stoi(item));
        } catch (...) {
            // skip invalid entries
        }
    }
    return result;
}

int main() {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    ip4_addr_t ip, netmask, gw;
    ipaddr_aton(STATIC_IP, &ip);
    ipaddr_aton(STATIC_NETMASK, &netmask);
    ipaddr_aton(STATIC_GATEWAY, &gw);

    netif *netif = &cyw43_state.netif[CYW43_ITF_STA];
    netif_set_addr(netif, &ip, &netmask, &gw);

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to Wi-Fi\n");
        return -1;
    }

    printf("Connected. IP: %s\n", STATIC_IP);

    int sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        printf("Socket creation failed\n");
        return -1;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    addr.sin_addr.s_addr = ipaddr_addr(STATIC_IP);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("Bind failed\n");
        lwip_close(sock);
        return -1;
    }

    char buffer[128];
    while (true) {
        int len = recv(sock, buffer, sizeof(buffer) - 1, 0);
        if (len > 0) {
            buffer[len] = 0;
            std::string data_str(buffer);
            std::vector<int> data = parse_csv(data_str);

            toggle_led();
            printf("Received: ");
            for (int v : data)
                printf("%d ", v);
            printf("\n");
        }
        sleep_ms(10);
    }

    lwip_close(sock);
    return 0;
}
