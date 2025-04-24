#include "hardware/gpio.h"
#include "lwip/arch.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <cstring>

// Wi-Fi情報
const char *SSID = "GL-SFT1200-288-2G-LNX";
const char *PASSWORD = "lnxmaster";
const char *SRC_IP = "192.168.8.219";
const int SRC_PORT = 5000;
#define PICO_DEFAULT_LED_PIN 25

bool connect_to_wifi(const char *ssid, const char *password, const char *ip_address) {
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_JAPAN)) {
        printf("Wi-Fi init failed\n");
        return false;
    }

    cyw43_arch_enable_sta_mode();

    int result = cyw43_arch_wifi_connect_timeout_ms(
        ssid, password,
        CYW43_AUTH_WPA2_AES_PSK,
        30000);

    if (result != 0) {
        printf("Wi-Fi connection failed: %d\n", result);
        return false;
    }

    // 固定IP設定（非公式：内部構造へのアクセス）
    struct netif *netif = netif_list;
    ip4_addr_t ip, netmask, gw;
    ip4addr_aton(SRC_IP, &ip);
    ip4addr_aton("255.255.255.0", &netmask);
    ip4addr_aton("192.168.8.1", &gw);
    netif_set_addr(netif, &ip, &netmask, &gw);

    printf("IP: %s\n", ipaddr_ntoa(&ip));
    return true;
}

int main() {
    stdio_init_all();

    // LEDピン初期化
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("Connecting to Wi-Fi...\n");
    if (!connect_to_wifi(SSID, PASSWORD, SRC_IP)) {
        printf("Wi-Fi接続失敗\n");
        return 1;
    }
    printf("Wi-Fi接続成功\n");

    // UDPソケット作成
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("ソケット作成失敗");
        return 1;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(SRC_PORT);
    addr.sin_addr.s_addr = inet_addr(SRC_IP);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("バインド失敗");
        close(sock);
        return 1;
    }

    char buffer[128];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    while (true) {
        int recv_len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                                (struct sockaddr *)&client_addr, &addr_len);
        if (recv_len < 0) {
            perror("受信失敗");
            break;
        }

        buffer[recv_len] = '\0';
        printf("受信: %s\n", buffer);
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }

    close(sock);
    return 0;
}
