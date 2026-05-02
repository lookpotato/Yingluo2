idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES board_config servo led_group audio_io IIC XL9555
)
