idf_component_register(SRCS "uart.c" "main.c" "https_request.c" "wifi_init.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart esp_driver_gpio esp-tls esp_wifi nvs_flash
                    EMBED_TXTFILES server_root_cert.pem)
