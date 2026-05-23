#include "ws2812b.h"

#include <stdlib.h>
#include <string.h>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#define WS2812B_RMT_RESOLUTION_HZ 10000000
#define WS2812B_RESET_US 80
#define WS2812B_BYTES_PER_LED 3

static const char *TAG = "ws2812b";

static rmt_channel_handle_t s_tx_channel;
static rmt_encoder_handle_t s_bytes_encoder;
static uint8_t *s_pixels_grb;
static uint32_t s_led_count;

esp_err_t ws2812b_init(gpio_num_t gpio_num, uint32_t led_count)
{
    ESP_RETURN_ON_FALSE(led_count > 0, ESP_ERR_INVALID_ARG, TAG, "led_count must be greater than 0");

    if (s_tx_channel != NULL) {
        ESP_LOGI(TAG, "already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "init start: gpio=%d, led_count=%lu, rmt_resolution=%d Hz",
             gpio_num, led_count, WS2812B_RMT_RESOLUTION_HZ);

    s_pixels_grb = calloc(led_count, WS2812B_BYTES_PER_LED);
    ESP_RETURN_ON_FALSE(s_pixels_grb != NULL, ESP_ERR_NO_MEM, TAG, "no memory for pixel buffer");
    s_led_count = led_count;
    ESP_LOGI(TAG, "pixel buffer allocated: %lu bytes", led_count * WS2812B_BYTES_PER_LED);

    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = WS2812B_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false,
        .flags.io_loop_back = false,
        .flags.io_od_mode = false,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_config, &s_tx_channel), TAG, "create RMT TX channel failed");
    ESP_LOGI(TAG, "RMT TX channel created");

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .duration0 = 4,
            .level0 = 1,
            .duration1 = 9,
            .level1 = 0,
        },
        .bit1 = {
            .duration0 = 8,
            .level0 = 1,
            .duration1 = 5,
            .level1 = 0,
        },
        .flags.msb_first = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &s_bytes_encoder), TAG,
                        "create bytes encoder failed");
    ESP_LOGI(TAG, "RMT bytes encoder created: 0 code=0.4us high/0.9us low, 1 code=0.8us high/0.5us low");
    ESP_RETURN_ON_ERROR(rmt_enable(s_tx_channel), TAG, "enable RMT TX channel failed");
    ESP_LOGI(TAG, "RMT TX channel enabled");

    return ws2812b_clear();
}

esp_err_t ws2812b_set_pixel(uint32_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    ESP_RETURN_ON_FALSE(s_pixels_grb != NULL, ESP_ERR_INVALID_STATE, TAG, "WS2812B is not initialized");
    ESP_RETURN_ON_FALSE(index < s_led_count, ESP_ERR_INVALID_ARG, TAG, "pixel index out of range");

    uint8_t *pixel = &s_pixels_grb[index * WS2812B_BYTES_PER_LED];
    pixel[0] = green;
    pixel[1] = red;
    pixel[2] = blue;

    ESP_LOGD(TAG, "pixel[%lu] RGB=(%u,%u,%u), GRB bytes=(%u,%u,%u)",
             index, red, green, blue, pixel[0], pixel[1], pixel[2]);

    return ESP_OK;
}

esp_err_t ws2812b_fill(uint8_t red, uint8_t green, uint8_t blue)
{
    ESP_RETURN_ON_FALSE(s_pixels_grb != NULL, ESP_ERR_INVALID_STATE, TAG, "WS2812B is not initialized");

    for (uint32_t i = 0; i < s_led_count; ++i) {
        ESP_RETURN_ON_ERROR(ws2812b_set_pixel(i, red, green, blue), TAG, "set pixel failed");
    }

    return ws2812b_refresh();
}

esp_err_t ws2812b_refresh(void)
{
    ESP_RETURN_ON_FALSE(s_tx_channel != NULL && s_bytes_encoder != NULL && s_pixels_grb != NULL,
                        ESP_ERR_INVALID_STATE, TAG, "WS2812B is not initialized");

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    ESP_RETURN_ON_ERROR(rmt_transmit(s_tx_channel, s_bytes_encoder, s_pixels_grb,
                                     s_led_count * WS2812B_BYTES_PER_LED, &tx_config),
                        TAG, "transmit WS2812B data failed");
    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(s_tx_channel, -1), TAG, "wait RMT TX done failed");

    esp_rom_delay_us(WS2812B_RESET_US);
    ESP_LOGD(TAG, "refresh ok: sent %lu LED(s), %lu byte(s), reset=%dus",
             s_led_count, s_led_count * WS2812B_BYTES_PER_LED, WS2812B_RESET_US);
    return ESP_OK;
}

esp_err_t ws2812b_clear(void)
{
    ESP_RETURN_ON_FALSE(s_pixels_grb != NULL, ESP_ERR_INVALID_STATE, TAG, "WS2812B is not initialized");

    memset(s_pixels_grb, 0, s_led_count * WS2812B_BYTES_PER_LED);
    return ws2812b_refresh();
}
