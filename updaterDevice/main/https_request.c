#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_tls.h"

#include "https_request.h"
#include "uart.h"
#include "uartCommands.h"
#include "configuration.h"


static const char WEB_SERVER[] = __WEB_SERVER;
static const char WEB_URL[] = __WEB_URL;

static const char *TAG = "HTTPS";

static SemaphoreHandle_t uart_mutex = NULL;

static char *REQUEST;
static char *RESPONSE;

extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t server_root_cert_pem_end[] asm("_binary_server_root_cert_pem_end");

static int https_get_request()
{
    int ret, len = 0;
    esp_tls_cfg_t cfg = {
        .cacert_buf = (const unsigned char *)server_root_cert_pem_start,
        .cacert_bytes = server_root_cert_pem_end - server_root_cert_pem_start,
    };

    esp_tls_t *tls = esp_tls_init();
    if (!tls)
    {
        ESP_LOGE(TAG, "Failed to allocate esp_tls handle!");
        return 0;
    }

    if (esp_tls_conn_http_new_sync(WEB_URL, &cfg, tls) == 1)
    {
        ESP_LOGI(TAG, "Connection established...");
    }
    else
    {
        ESP_LOGE(TAG, "Connection failed...");
        int esp_tls_code = 0, esp_tls_flags = 0;
        esp_tls_error_handle_t tls_e = NULL;
        esp_tls_get_error_handle(tls, &tls_e);
        /* Try to get TLS stack level error and certificate failure flags, if any */
        ret = esp_tls_get_and_clear_last_error(tls_e, &esp_tls_code, &esp_tls_flags);
        if (ret == ESP_OK)
        {
            ESP_LOGE(TAG, "TLS error = -0x%x, TLS flags = -0x%x", esp_tls_code, esp_tls_flags);
        }
        goto cleanup;
    }

    size_t written_bytes = 0;
    do
    {
        ret = esp_tls_conn_write(tls,
                                 REQUEST + written_bytes,
                                 strlen(REQUEST) - written_bytes);
        if (ret >= 0)
        {
            written_bytes += ret;
        }
        else if (ret != ESP_TLS_ERR_SSL_WANT_READ && ret != ESP_TLS_ERR_SSL_WANT_WRITE)
        {
            ESP_LOGE(TAG, "esp_tls_conn_write  returned: [0x%02X](%s)", ret, esp_err_to_name(ret));
            goto cleanup;
        }
    } while (written_bytes < strlen(REQUEST));

    ESP_LOGI(TAG, "Reading HTTP response...");
    do
    {
        len = HTTP_RESPONSE_LEN - 1;
        len = esp_tls_conn_read(tls, (void *)(RESPONSE), len);

        if (len == ESP_TLS_ERR_SSL_WANT_WRITE || ret == ESP_TLS_ERR_SSL_WANT_READ)
        {
            continue;
        }
        else if (len < 0)
        {
            ESP_LOGE(TAG, "esp_tls_conn_read  returned [-0x%02X](%s)", -ret, esp_err_to_name(ret));
            break;
        }
        else if (len == 0)
        {
            break;
        }

    } while (1);

cleanup:
    esp_tls_conn_destroy(tls);
    return len;
}

// path must start with /
void request_writeBackUART(const char *path, const uint16_t dataLen)
{
    if (uart_mutex == NULL)
    {
        uart_mutex = xSemaphoreCreateMutex();
    }
    if (REQUEST == NULL)
    {
        REQUEST = (char *)malloc(HTTP_REQUEST_LEN);
        assert(REQUEST);
    }
    if (RESPONSE == NULL)
    {
        RESPONSE = (char *)malloc(HTTP_RESPONSE_LEN);
        assert(RESPONSE);
    }

    if (xSemaphoreTake(uart_mutex, portMAX_DELAY) == pdTRUE)
    {
        memset(REQUEST, 0x00, HTTP_REQUEST_LEN);
        memset(RESPONSE, 0x00, HTTP_RESPONSE_LEN);

        uint16_t dataLenCRC = dataLen + 4;

        snprintf(REQUEST, HTTP_REQUEST_LEN, "GET %s HTTP/1.1\r\n"
                                       "Host: %s\r\n"
                                       "User-Agent: firmwareUpdater/1.0 esp32\r\n"
                                       "\r\n",
                 path, WEB_SERVER);

        https_get_request();
        ESP_LOG_BUFFER_HEXDUMP("[RESPONSE]", RESPONSE, dataLenCRC, ESP_LOG_INFO);

        uart_write_bytes(EX_UART_NUM, RESPONSE, dataLenCRC);
        xSemaphoreGive(uart_mutex);
    }
}