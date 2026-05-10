idf_component_register(
    SRCS "ws2812b.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_rmt esp_driver_gpio freertos esp_common
)
