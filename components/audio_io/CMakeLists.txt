idf_component_register(
    SRCS "audio_io.c" "es8388_hw.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2s driver freertos IIC XL9555 board_config esp_common
)
