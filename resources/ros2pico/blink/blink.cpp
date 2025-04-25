#include "pico/stdlib.h"
#include <stdio.h>

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

bool receive_packet_minimal(uart_inst_t *uart, int16_t *data_buffer, uint8_t &out_size) {
    static enum { WAIT_HEADER1,
                  WAIT_HEADER2,
                  WAIT_SIZE,
                  WAIT_DATA } state = WAIT_HEADER1;
    static uint8_t size = 0, data_pos = 0;
    static uint8_t raw_buffer[64];

    while (uart_is_readable(uart)) {
        uint8_t byte = uart_getc(uart);

        switch (state) {
        case WAIT_HEADER1:
            if (byte == 0xAA)
                state = WAIT_HEADER2;
            break;
        case WAIT_HEADER2:
            if (byte == 0x55)
                state = WAIT_SIZE;
            else
                state = WAIT_HEADER1;
            break;
        case WAIT_SIZE:
            size = byte;
            if (size > 32)
                state = WAIT_HEADER1;
            else {
                data_pos = 0;
                state = WAIT_DATA;
            }
            break;
        case WAIT_DATA:
            raw_buffer[data_pos++] = byte;
            if (data_pos >= size * 2) {
                for (uint8_t i = 0; i < size; ++i) {
                    data_buffer[i] = raw_buffer[2 * i] | (raw_buffer[2 * i + 1] << 8);
                }
                out_size = size;
                state = WAIT_HEADER1;
                return true;
            }
            break;
        }
    }
    return false;
}

int main() {
    stdio_init_all();
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    int16_t data[32];
    uint8_t size = 0;

    while (true) {
        if (receive_packet_minimal(UART_ID, data, size)) {
            printf("Received %d values:\n", size);
            for (int i = 0; i < size; i++) {
                printf("  [%d] = %d\n", i, data[i]);
            }
        }
    }
}
