#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "uart.h"
#include "configuration.h"

#define TXD_PIN 17
#define RXD_PIN 16

static const char *TAG = "UART_";

void uart_init(QueueHandle_t *uart_queue)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    int queue_size = 10;
    // Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, RX_BUF_SIZE * 2, 0, queue_size, uart_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    // Set UART pins
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    // uart_enable_pattern_det_baud_intr(EX_UART_NUM, __SYN, 1, 0, 0, 0);
    // uart_pattern_queue_reset(EX_UART_NUM, queue_size);
}