idf_component_register(
    SRCS "audio_io.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common esp_system board_config
)
