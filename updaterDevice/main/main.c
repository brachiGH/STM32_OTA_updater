#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sntp.h"
#include "esp_netif.h"

#include "https_request.h"
#include "wifi_init.h"
#include "uart.h"
#include "uartCommands.h"
#include "configuration.h"

static const char *TAG = "UART_";

typedef struct
{
    uint32_t deviceModelID;
    uint32_t dataField1;
    uint32_t dataField2;
    uint16_t dataLen;
    uint8_t cmd;
} resquest_item;

const uint8_t __NAK__CMD[CMD_TX_BUF_SIZE] = {__NAK};

static QueueHandle_t uart_queue;
static QueueHandle_t http_request_queue;

static bool dataIntegrityCheck(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        if (data[i] != data[i + len])
        {
            return false;
        }
    }

    return true;
}

static void requestHandler_task(void *pvParameters)
{
    resquest_item event;
    char *_path = (char *)malloc(256);
    assert(_path);
    while (1)
    {
        if (xQueueReceive(http_request_queue, (void *)&event, portMAX_DELAY))
        {
            memset(_path, 0x00, 256);
            if (event.cmd == _CMD_GET_LASTEST_VERSION_ID)
            {
                snprintf(_path, 256, "/device/%lu/latest", event.deviceModelID);
                request_writeBackUART(_path, event.dataLen);
            }
            else if (event.cmd == _CMD_GET_UPDATE)
            {
                snprintf(_path, 256, "/update/%lu/%lu/%lu", event.deviceModelID, event.dataField1,
                         event.dataField2);
                request_writeBackUART(_path, event.dataLen);
            }
        }
    }

    vTaskDelete(NULL);
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *rxBuff = (uint8_t *)malloc(RX_BUF_SIZE);
    assert(rxBuff);

    // txBuff[0] = ACK , and txBuff[1] = data lengh
    uint8_t *txBuff = (uint8_t *)malloc(CMD_TX_BUF_SIZE);
    assert(txBuff);

    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
        {
            // clear buffers
            memset(rxBuff, 0x00, RX_BUF_SIZE);
            memset(txBuff, 0x00, CMD_TX_BUF_SIZE);

            switch (event.type)
            {
            case UART_DATA:
                // read uart
                uart_read_bytes(EX_UART_NUM, rxBuff, event.size, portMAX_DELAY);

                // An uart event must be at least made of 2 bytes
                // a 'SYN' and a 'CMD'
                if (event.size > 1 && rxBuff[0] == __SYN)
                {
                    uint8_t cmd = rxBuff[1];
                    // data is sent twice, to verify it integrity.
                    uint16_t dataLen = (event.size / 2) - 1;
                    if (dataLen > 0 && cmd != _CMD_LOG)
                    {
                        // that log messages are not checked.
                        if (!dataIntegrityCheck(rxBuff + 2, dataLen))
                            goto sendNAK;
                    }

                    /* txBuff[1] and txBuff[2] represent the lengh of the data is gonna be send
                       Don't forget about endianess
                    */
                    txBuff[0] = __ACK;
                    txBuff[1] = 0x00;
                    txBuff[2] = 0x00;

                    resquest_item item;
                    item.cmd = cmd;
                    switch (cmd)
                    {
                    case _CMD_PING:
                        if (dataLen != 0)
                            goto sendNAK;
                        ESP_LOGI(TAG, "[_CMD_PING]");
                        break;
                    case _CMD_LOG:
                        rxBuff[event.size] = '\0';
                        ESP_LOGI(TAG, "[_CMD_LOG]: %s", rxBuff + 2);
                        continue;
                        break;
                    case _CMD_GET_LASTEST_VERSION_ID:
                        if (dataLen != 4)
                            goto sendNAK;
                        ESP_LOGI(TAG, "[_CMD_GET_LASTEST_VERSION_ID]");

                        uint32_t deviceModel = *(uint32_t *)(rxBuff + 2);

                        item.deviceModelID = deviceModel;
                        item.dataLen = 4;

                        break;
                    case _CMD_GET_UPDATE:
                        if (dataLen != 10)
                            goto sendNAK;
                        ESP_LOGI(TAG, "[_CMD_START_UPDATE]");

                        uint32_t deviceModelID = *(uint32_t *)(rxBuff + 2);
                        uint32_t requestedAppVersion = *(uint32_t *)(rxBuff + 6);
                        uint16_t requestedAppCursor = *(uint16_t *)(rxBuff + 10); // length is in 1020bytes

                        item.deviceModelID = deviceModelID;
                        item.dataField1 = requestedAppVersion;
                        item.dataField2 = requestedAppCursor;
                        item.dataLen = TX_BUF_SIZE;
                        break;
                    }

                    if (item.dataLen > 0)
                    {
                        txBuff[1] = item.dataLen & 0xFF;
                        txBuff[2] = (item.dataLen & 0xFF00) >> 8;

                        if (xQueueSend(http_request_queue, &item, 10) != pdPASS)
                            goto sendNAK;
                    }
                    uart_write_bytes(EX_UART_NUM, txBuff, CMD_TX_BUF_SIZE);
                }
                else
                {
                sendNAK:
                    uart_write_bytes(EX_UART_NUM, __NAK__CMD, CMD_TX_BUF_SIZE);
                    xQueueReset(uart_queue);
                    uart_flush_input(EX_UART_NUM);
                }
                break;
            default:
                xQueueReset(uart_queue);
                uart_flush_input(EX_UART_NUM);
                break;
            }
        }
    }

    free(rxBuff);
    rxBuff = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(wifi_init_sta());

    uart_init(&uart_queue);

    http_request_queue = xQueueCreate(3, sizeof(resquest_item));

    xTaskCreatePinnedToCore(uart_event_task, "uart event", 6144, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(requestHandler_task, "request handler", 4096, NULL, 5, NULL, 1);
}