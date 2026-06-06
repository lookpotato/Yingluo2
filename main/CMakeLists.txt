idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES board_config servo led_group audio_io IIC XL9555 ws2812b esp_driver_gpio esp_wifi esp_netif esp_event nvs_flash esp_http_client esp_websocket_client esp_psram
)
