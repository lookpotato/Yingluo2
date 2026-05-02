idf_component_register(
    SRCS "xl9555.c"
    INCLUDE_DIRS "."
    REQUIRES IIC driver freertos esp_common
)
